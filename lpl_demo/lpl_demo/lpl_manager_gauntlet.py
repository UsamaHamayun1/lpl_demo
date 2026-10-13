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

# --- STATES ---
STATE_CRUISE = 0       
STATE_SWERVE_OUT = 1   
STATE_PASSING = 2      
STATE_MERGE_BACK = 3   
class LPLManagerGauntlet(Node):
    def __init__(self):
        super().__init__('lpl_manager_gauntlet')
        
        # --- CONFIGURATION ---
        self.LPL_ENABLED = False
        
        # Distances & Bounds
        self.SWERVE_TRIGGER = 1.8       # Start avoiding at 1.8m
        self.PASS_LIMIT_X = 7.0         # Drive past this X before merging
        self.LANE_Y = 1.2               # The "Safe Lane" Y coordinate
        self.STOPPING_DISTANCE = 2.0    
        self.SWERVE_DISABLE_X = 6.0     # DO NOT SWERVE after X=6.0 (Forces Red Box Crash)

        # --- VARIABLES ---
        self.smooth_conf = 1.0
        self.drive_state = STATE_CRUISE
        self.state_start_time = 0
        self.closest_dist = 99.0 
        self.robot_x = 0.0 
        self.robot_y = 0.0 
        
        self.vel_x_history = deque(maxlen=10)
        self.vel_y_history = deque(maxlen=10)
        self.last_prox_score = 1.0
        self.last_stab_score = 1.0
        self.recovery_mode = False
        self.recovery_value = 0.0
        
        self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
        self.sub_input = self.create_subscription(Twist, '/cmd_vel_input', self.control_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.authorize_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

        self.get_logger().info("--- LPL MANAGER: FIXED PASSING & RED BOX BEHAVIOR ---")

    def authorize_callback(self, msg):
        if self.smooth_conf < 0.3:
            self.recovery_mode = True; self.recovery_value = self.smooth_conf 

    def calculate_stability_score(self, vx, vy):
        self.vel_x_history.append(vx); self.vel_y_history.append(vy)
        if len(self.vel_x_history) < 5: return 1.0
        try: 
            total_variance = statistics.variance(self.vel_x_history) + statistics.variance(self.vel_y_history)
        except: return 1.0
        if total_variance >= 0.1: return 0.0 
        else: return 1.0 - (total_variance / 0.1)

    def perception_callback(self, msg):
        try:
            # 1. FIND ROBOT
            robot_idx = -1
            for name in ['waffle_pi', 'turtlebot3_waffle_pi', 'mobile_base']:
                if name in msg.name: robot_idx = msg.name.index(name); break
            if robot_idx == -1: return 
            
            self.robot_x = msg.pose[robot_idx].position.x
            self.robot_y = msg.pose[robot_idx].position.y

            # 2. FIND CLOSEST OBSTACLE
            self.closest_dist = 999.0
            closest_name = "None"
            target_vx = 0.0; target_vy = 0.0
            
            for i, name in enumerate(msg.name):
                if i == robot_idx or any(x in name for x in ['ground', 'sun', 'floor']): continue
                dist = math.sqrt((self.robot_x - msg.pose[i].position.x)**2 + (self.robot_y - msg.pose[i].position.y)**2)
                if dist < self.closest_dist: 
                    self.closest_dist = dist; closest_name = name
                    target_vx = msg.twist[i].linear.x; target_vy = msg.twist[i].linear.y

            # 3. LPL LOGIC
            if self.LPL_ENABLED:
                stab_score = self.calculate_stability_score(target_vx, target_vy)
                
                # A. MANEUVER OVERRIDE: 
                # If we are swerving, passing, or merging -> IGNORE PROXIMITY
                # This fixes the "Robot stops at 6.5" bug
                if self.drive_state != STATE_CRUISE:
                     self.smooth_conf = 1.0

                # B. RED BOX STOP LOGIC (Only when cruising)
                elif stab_score < 0.2:
                    if self.closest_dist > self.STOPPING_DISTANCE:
                        self.smooth_conf = 0.5 
                    else:
                        self.smooth_conf = 0.0 
                
                # C. GREEN BOX PROXIMITY (Only applies if STABLE and NOT maneuvering)
                else:
                    if self.closest_dist <= 0.6: prox_score = 0.0
                    elif self.closest_dist > 2.0: prox_score = 1.0
                    else: prox_score = (self.closest_dist - 0.6) / 1.4
                    self.smooth_conf = (0.7 * self.smooth_conf) + (0.3 * prox_score)

                self.last_prox_score = prox_score
                self.last_stab_score = stab_score

                if self.recovery_mode:
                    self.recovery_value += 0.05
                    if self.recovery_value >= 1.0: self.recovery_value = 1.0; self.recovery_mode = False 
                    self.smooth_conf = max(self.smooth_conf, self.recovery_value)
            else:
                self.smooth_conf = 1.0 

            self.publish_status_to_ui(closest_name)

        except Exception: pass

    def control_callback(self, msg):
        final_cmd = Twist()
        current_time = time.time()
        
        # --- STATE MACHINE ---
        
        if self.drive_state == STATE_CRUISE:
            # Standard Driving
            final_cmd.linear.x = min(0.8, msg.linear.x * 3.5)
            final_cmd.angular.z = msg.angular.z
            
            # TRIGGER SWERVE LOGIC
            # Condition 1: Object is close (< 1.8m)
            # Condition 2: Robot is centered
            # Condition 3: Robot is BEFORE the Swerve Disable Limit (X < 6.0)
            # This fixes "Avoiding Red Box" -> It will now ONLY swerve for Green Box.
            if self.closest_dist < self.SWERVE_TRIGGER and abs(self.robot_y) < 0.5 and self.robot_x < self.SWERVE_DISABLE_X:
                
                if self.smooth_conf > 0.4:
                    self.drive_state = STATE_SWERVE_OUT
                    self.state_start_time = current_time
                    self.get_logger().info("STATE: SWERVE OUT (Green Box Detected)")

        elif self.drive_state == STATE_SWERVE_OUT:
            # Turn Left gently
            final_cmd.linear.x = 0.5
            final_cmd.angular.z = 0.6
            
            # Stop turning once we are clearly moving into the left lane.
            if self.robot_y > 0.8: 
                self.drive_state = STATE_PASSING
                self.get_logger().info("STATE: PASSING (LANE KEEPING)")

        elif self.drive_state == STATE_PASSING:
            # ACTIVE LANE KEEPING (Target Y = 1.2)
            target_y = self.LANE_Y
            error_y = target_y - self.robot_y
            
            # Steer towards the lane (P-Control)
            final_cmd.angular.z = error_y * 1.5
            final_cmd.linear.x = 0.8
            
            # Clamp turning
            final_cmd.angular.z = max(-0.6, min(0.6, final_cmd.angular.z))
            
            # EXIT CONDITION: Drive until X > 7.0
            if self.robot_x > self.PASS_LIMIT_X: 
                self.drive_state = STATE_MERGE_BACK
                self.get_logger().info("STATE: MERGING BACK")

        elif self.drive_state == STATE_MERGE_BACK:
            # Return to Center (Target Y = 0.0)
            target_y = 0.0
            error_y = self.robot_y - target_y 
            
            # Negative error -> Turn Right
            final_cmd.angular.z = -1.0 * error_y 
            final_cmd.linear.x = 0.6
            
            final_cmd.angular.z = max(-0.7, min(0.7, final_cmd.angular.z))
            
            # Done
            if abs(error_y) < 0.1:
                self.drive_state = STATE_CRUISE 
                self.get_logger().info("STATE: CRUISE (Proceeding to Red Box)")

        # SAFETY FILTER
        final_cmd.linear.x *= self.smooth_conf
        final_cmd.angular.z *= self.smooth_conf
        self.pub_cmd.publish(final_cmd)

    def publish_status_to_ui(self, obs_name):
        current_val = self.smooth_conf
        if self.drive_state != STATE_CRUISE: state_label = "AVOIDING"
        elif current_val < 0.1: state_label = "DANGER STOP"
        elif current_val < 0.5: state_label = "CAUTION"
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
    node = LPLManagerGauntlet()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()