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
from collections import deque 

# --- AVOIDANCE STATES ---
STATE_CRUISE = 0
STATE_SWERVE_OUT = 1
STATE_PASSING = 2
STATE_MERGE_BACK = 3

class LPLManagerFormula(Node):
    def __init__(self):
        super().__init__('lpl_manager_formula')
        
        # --- CONFIGURATION ---
        self.LPL_ENABLED = True 
        
        # --- STATE VARIABLES ---
        self.smooth_conf = 1.0
        self.last_prox_score = 1.0
        self.last_stab_score = 1.0
        
        self.drive_state = STATE_CRUISE
        self.state_start_time = 0
        self.closest_dist = 99.0 
        
        # Stability Buffers (We now track BOTH X and Y)
        self.vel_x_history = deque(maxlen=10)
        self.vel_y_history = deque(maxlen=10)
        
        self.recovery_mode = False
        self.recovery_value = 0.0
        
        self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
        self.sub_input = self.create_subscription(Twist, '/cmd_vel_input', self.control_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.authorize_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

        self.get_logger().info("--- LPL MANAGER: DUAL-AXIS DETECTION READY ---")

    def authorize_callback(self, msg):
        if self.smooth_conf < 0.3:
            self.recovery_mode = True
            self.recovery_value = self.smooth_conf 

    def calculate_stability_score(self, vx, vy):
        # Track X and Y separately
        self.vel_x_history.append(vx)
        self.vel_y_history.append(vy)
        
        if len(self.vel_x_history) < 5: return 1.0
        
        try: 
            # Calculate Variance for BOTH axes
            var_x = statistics.variance(self.vel_x_history)
            var_y = statistics.variance(self.vel_y_history)
            
            # Total Instability is the SUM of chaos in X and Y
            total_variance = var_x + var_y
        except: return 1.0
        
        # Threshold: 0.1 is usually enough to trigger on "Choice" snapping
        jitter_threshold = 0.1
        
        if total_variance >= jitter_threshold: 
            return 0.0 # INSTANT STOP
        else: 
            return 1.0 - (total_variance / jitter_threshold)

    def perception_callback(self, msg):
        try:
            # 1. FIND ROBOT
            robot_idx = -1
            possible_names = ['waffle_pi', 'turtlebot3_waffle_pi', 'mobile_base']
            for name in possible_names:
                if name in msg.name:
                    robot_idx = msg.name.index(name)
                    break
            if robot_idx == -1: return 
            
            robot_pos = msg.pose[robot_idx].position
            rx, ry = robot_pos.x, robot_pos.y

            # 2. FIND CLOSEST OBSTACLE
            self.closest_dist = 999.0
            closest_name = "None"
            target_vx = 0.0
            target_vy = 0.0
            
            for i, name in enumerate(msg.name):
                if i == robot_idx: continue
                if any(x in name for x in ['ground', 'sun', 'floor']): continue
                
                ox = msg.pose[i].position.x
                oy = msg.pose[i].position.y
                dist = math.sqrt((rx - ox)**2 + (ry - oy)**2)
                
                if dist < self.closest_dist: 
                    self.closest_dist = dist
                    closest_name = name
                    # Capture BOTH velocity components
                    target_vx = msg.twist[i].linear.x
                    target_vy = msg.twist[i].linear.y

            # 3. CALCULATE CONFIDENCE
            if self.LPL_ENABLED:
                # Proximity
                prox_score = 1.0
                if self.closest_dist <= 0.6: prox_score = 0.0
                elif self.closest_dist < 2.5: prox_score = (self.closest_dist - 0.6) / (2.5 - 0.6)

                # Stability (Passes both X and Y now)
                stab_score = self.calculate_stability_score(target_vx, target_vy)
                
                self.last_prox_score = prox_score
                self.last_stab_score = stab_score

                # Combined
                raw_confidence = prox_score * stab_score
                
                # Hard Stop Logic
                if stab_score < 0.2:
                    self.smooth_conf = 0.0
                else:
                    self.smooth_conf = (0.7 * self.smooth_conf) + (0.3 * raw_confidence)

                # Recovery
                if self.recovery_mode:
                    self.recovery_value += 0.05
                    if self.recovery_value >= 1.0:
                        self.recovery_value = 1.0
                        self.recovery_mode = False 
                    self.smooth_conf = max(self.smooth_conf, self.recovery_value)
            else:
                self.smooth_conf = 1.0 

            self.publish_status_to_ui(closest_name)

        except Exception as e:
            pass

    def control_callback(self, msg):
        final_cmd = Twist()
        current_time = time.time()
        
        # State Machine (Same as before)
        if self.drive_state == STATE_CRUISE:
            final_cmd.linear.x = msg.linear.x * 3.5 
            if final_cmd.linear.x > 0.8: final_cmd.linear.x = 0.8 
            final_cmd.angular.z = msg.angular.z
            
            if self.closest_dist < 2.0 and final_cmd.linear.x > 0:
                # Only Swerve if Stable
                if self.smooth_conf > 0.5:
                    self.drive_state = STATE_SWERVE_OUT
                    self.state_start_time = current_time
                else:
                    pass # Keep going straight -> Safety Filter will catch us

        elif self.drive_state == STATE_SWERVE_OUT:
            final_cmd.linear.x = 0.5; final_cmd.angular.z = 0.8 
            if current_time - self.state_start_time > 0.8: 
                self.drive_state = STATE_PASSING; self.state_start_time = current_time

        elif self.drive_state == STATE_PASSING:
            final_cmd.linear.x = 0.8; final_cmd.angular.z = 0.0
            if current_time - self.state_start_time > 1.5: 
                self.drive_state = STATE_MERGE_BACK; self.state_start_time = current_time

        elif self.drive_state == STATE_MERGE_BACK:
            final_cmd.linear.x = 0.5; final_cmd.angular.z = -0.8
            if current_time - self.state_start_time > 0.8:
                self.drive_state = STATE_CRUISE 

        # Safety Filter
        final_cmd.linear.x = final_cmd.linear.x * self.smooth_conf
        final_cmd.angular.z = final_cmd.angular.z * self.smooth_conf
        self.pub_cmd.publish(final_cmd)

    def publish_status_to_ui(self, obs_name):
        current_val = self.smooth_conf
        if self.drive_state != STATE_CRUISE: state_label = "AVOIDING"
        elif current_val < 0.3: state_label = "DANGER STOP"
        elif current_val < 0.7: state_label = "UNSTABLE"
        else: state_label = "PROCEED"

        status_packet = {
            "confidence": round(current_val, 2),
            "prox": round(self.last_prox_score, 2),
            "stab": round(self.last_stab_score, 2),
            "state": state_label,
            "object": obs_name,
            "override": self.recovery_mode
        }
        self.pub_status.publish(String(data=json.dumps(status_packet)))

def main():
    rclpy.init()
    node = LPLManagerFormula()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()