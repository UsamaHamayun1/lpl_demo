#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import math
import json
import statistics 
import time

class LPLManagerFormula(Node):
    def __init__(self):
        super().__init__('lpl_manager_formula')
        
        self.confidence = 1.0
        self.manual_override = False
        self.target_object = "None"
        
        # Velocity Tracking (The Fix for Humans)
        self.prev_pos_map = {} # Stores {name: (x, y, time)}
        
        self.vel_history = [] 
        self.history_size = 10 
        
        self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
        self.sub_input = self.create_subscription(Twist, '/cmd_vel_input', self.control_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.authorize_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

        print("--- LPL MANAGER: CONTINUOUS CONTROL + ACTOR FIX LOADED ---")

    def authorize_callback(self, msg):
        self.manual_override = not self.manual_override

    def calculate_stability_score(self, new_velocity):
        self.vel_history.append(new_velocity)
        if len(self.vel_history) > self.history_size: self.vel_history.pop(0)
        if len(self.vel_history) < 2: return 1.0
        
        try: variance = statistics.variance(self.vel_history)
        except: return 1.0
        
        jitter_threshold = 0.05 # Lowered threshold to catch human jitters
        if variance >= jitter_threshold: return 0.0
        else: return 1.0 - (variance / jitter_threshold)

    def perception_callback(self, msg):
        try:
            # 1. Find Robot
            robot_idx = -1
            possible_names = ['waffle_pi', 'turtlebot3_waffle_pi', 'turtlebot3_burger', 'mobile_base']
            for name in possible_names:
                if name in msg.name:
                    robot_idx = msg.name.index(name)
                    break
            if robot_idx == -1: return 
            
            rx = msg.pose[robot_idx].position.x
            ry = msg.pose[robot_idx].position.y

            closest_dist = 999.0
            closest_name = "None"
            closest_vel_mag = 0.0
            now = time.time()
            
            # 2. Find Closest Object & Calculate Velocity Manually
            for i, name in enumerate(msg.name):
                # Ignore static world items
                if name in possible_names or 'ground' in name or 'sun' in name or 'cafe' in name or 'table' in name: 
                    continue
                
                ox = msg.pose[i].position.x
                oy = msg.pose[i].position.y
                dist = math.sqrt((rx - ox)**2 + (ry - oy)**2)
                
                if dist < closest_dist: 
                    closest_dist = dist
                    closest_name = name
                    
                    # --- MANUAL VELOCITY CALCULATION (CRITICAL FIX) ---
                    # Gazebo Actors report Twist=0, so we calculate dx/dt
                    if name in self.prev_pos_map:
                        old_x, old_y, old_t = self.prev_pos_map[name]
                        dt = now - old_t
                        if dt > 0:
                            dx = ox - old_x
                            dy = oy - old_y
                            closest_vel_mag = math.sqrt(dx**2 + dy**2) / dt
                    else:
                        # Fallback for first frame
                        v = msg.twist[i].linear
                        closest_vel_mag = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                    
                    self.prev_pos_map[name] = (ox, oy, now)

            self.target_object = closest_name

            # 3. Calculate Formula (Multiplicative as per your code)
            safe_dist = 2.5
            crit_dist = 0.6 
            
            if closest_dist >= safe_dist: prox_score = 1.0
            elif closest_dist <= crit_dist: prox_score = 0.0
            else: prox_score = (closest_dist - crit_dist) / (safe_dist - crit_dist)

            stab_score = self.calculate_stability_score(closest_vel_mag)
            
            # HYBRID FORMULA: Multiplicative
            self.confidence = prox_score * stab_score
            
            # Hard cutoff for safety
            if self.confidence < 0.1: self.confidence = 0.0

            self.publish_status_to_ui(closest_name)

        except Exception:
            pass

    def control_callback(self, msg):
        safe_msg = Twist()
        safe_msg.angular.z = msg.angular.z 
        velocity_request = msg.linear.x

        if self.manual_override:
            # HUMAN CONTROL
            safe_msg.linear.x = velocity_request
        else:
            # AUTO CONTROL (Continuous Scaling)
            if velocity_request < 0.0:
                safe_msg.linear.x = velocity_request # Allow reverse
            else:
                # Scale speed by confidence (Your requested logic)
                safe_msg.linear.x = velocity_request * self.confidence

        self.pub_cmd.publish(safe_msg)

    def publish_status_to_ui(self, obs_name):
        if self.manual_override:
            state = "MANUAL"
        elif self.confidence >= 0.8:
            state = "PROCEED"
        elif self.confidence < 0.1:
            state = "UNSTABLE" # Changed "PAUSE" to "UNSTABLE" for clarity
        else:
            state = "AVOIDING" # "CAUTION" -> "AVOIDING"

        status_packet = {
            "confidence": round(self.confidence, 2),
            "state": state,
            "object": obs_name,
            "override": self.manual_override
        }
        self.pub_status.publish(String(data=json.dumps(status_packet)))

def main(args=None):
    rclpy.init(args=args)
    node = LPLManagerFormula()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()