#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import math

class LPLManager(Node):
    def __init__(self):
        super().__init__('lpl_manager')
        
        self.confidence = 1.0
        self.recovery_mode = False
        self.recovery_val = 0.0
        
        # We now track WHICH object is scaring us
        self.target_object = "None"
        
        # 1. Perception
        self.sub_models = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.perception_callback, 10)
            
        # 2. Human Input
        self.sub_input = self.create_subscription(
            Twist, 'cmd_vel_input', self.control_callback, 10)

        # 3. Authorization
        self.sub_auth = self.create_subscription(
            Empty, '/lpl/authorize', self.authorize_callback, 10)
            
        # 4. Output
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 5. Recovery Timer
        self.create_timer(0.05, self.update_recovery)

    def authorize_callback(self, msg):
        if self.confidence < 0.9:
            self.recovery_mode = True
            self.recovery_val = self.confidence 
            self.get_logger().info(f"Authorized to pass: {self.target_object}")

    def update_recovery(self):
        if self.recovery_mode:
            self.recovery_val += 0.005 # Slow ramp (Crawl)
            if self.recovery_val >= 1.0: self.recovery_val = 1.0

    def perception_callback(self, msg):
        try:
            # --- FIND ROBOT ---
            robot_idx = -1
            possible_names = ['waffle_pi', 'turtlebot3_waffle_pi', 'turtlebot3_burger', 'mobile_base']
            for name in possible_names:
                if name in msg.name:
                    robot_idx = msg.name.index(name)
                    break
            if robot_idx == -1: return 
            robot_pos = msg.pose[robot_idx].position

            # --- FIND CLOSEST OBJECT & ITS NAME ---
            closest_dist = 999.0
            closest_name = "None"
            
            for i, name in enumerate(msg.name):
                if name in possible_names or 'ground' in name or 'sun' in name: continue
                p = msg.pose[i].position
                dist = math.sqrt((robot_pos.x - p.x)**2 + (robot_pos.y - p.y)**2)
                
                if dist < closest_dist: 
                    closest_dist = dist
                    closest_name = name

            # --- LOGIC: DID THE THREAT CHANGE? ---
            # If we are closer to a NEW object, we must revoke authorization immediately.
            if closest_name != self.target_object:
                if self.recovery_mode: 
                    # self.get_logger().warn(f"Switching Focus: {self.target_object} -> {closest_name}. Revoking Auth.")
                    self.recovery_mode = False # Safety Reset
                self.target_object = closest_name

            # --- CALCULATE CONFIDENCE ---
            safe_dist = 2.0
            crit_dist = 0.9 
            
            if closest_dist >= safe_dist:
                raw_confidence = 1.0
                self.recovery_mode = False 
            elif closest_dist <= crit_dist:
                raw_confidence = 0.0
            else:
                raw_confidence = (closest_dist - crit_dist) / (safe_dist - crit_dist)

            if raw_confidence < 0.05: raw_confidence = 0.0

            # --- COMBINE ---
            if self.recovery_mode:
                self.confidence = max(raw_confidence, self.recovery_val)
            else:
                self.confidence = raw_confidence

        except Exception:
            pass

    def control_callback(self, msg):
        safe_msg = Twist()
        safe_msg.angular.z = msg.angular.z 
        
        velocity_request = msg.linear.x
        
        # Retreat is always allowed
        if velocity_request < 0.0:
            safe_msg.linear.x = velocity_request
        else:
            # Forward is throttled
            allowed_speed = velocity_request * self.confidence
            
            # CRAWL CAP: If recovering, limit speed to 0.1 m/s
            if self.recovery_mode and self.confidence < 0.95:
                crawl_cap = 0.1
                if allowed_speed > crawl_cap: allowed_speed = crawl_cap
            
            safe_msg.linear.x = allowed_speed

        self.pub_cmd.publish(safe_msg)
        self.print_status(msg.linear.x, safe_msg.linear.x, self.target_object)

    def print_status(self, input_vel, output_vel, obs_name):
        if input_vel < 0:
            state = "RETREAT"
            color = "\033[96m" 
        elif self.recovery_mode and self.confidence < 1.0:
            state = "CRAWLING" 
            color = "\033[94m" 
        elif self.confidence >= 0.8:
            state = "PROCEED"
            color = "\033[92m" 
        elif self.confidence == 0.0: 
            state = "PAUSE  "
            color = "\033[91m" 
        else:
            state = "CAUTION"
            color = "\033[93m" 
        
        reset = "\033[0m"
        # Print Object Name so you know what it sees
        print(f"\r[{obs_name[:6]}] Conf:[{self.confidence:.2f}] {color}{state}{reset} | "
              f"In:{input_vel:.2f}->Out:{output_vel:.2f}   ", end="")

def main(args=None):
    rclpy.init(args=args)
    node = LPLManager()
    rclpy.spin(node)
    rclpy.shutdown()