# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# from std_msgs.msg import String, Empty, Bool
# import json
# import statistics
# import math
# from collections import deque
# import time

# class LPLNavGuard(Node):
#     def __init__(self):
#         super().__init__('lpl_nav_guard')

#         self.FAR_SIGHT_RADIUS = 2.0    
#         self.PROXIMITY_CRIT = 0.5      
        
#         self.lpl_active = True
#         self.manual_lockout = False
#         self.blue_box_locked_permanently = False 
        
#         self.target_confidence = 1.0
#         self.current_confidence = 1.0 

#         # State Tracking
#         self.crashed = False
#         self.was_red_hesitating = False
#         self.red_pause_start = 0.0

#         self.histories = {
#             'obstacle_safe': deque(maxlen=10),
#             'obstacle_danger': deque(maxlen=10),
#             'obstacle_blue': deque(maxlen=10)   
#         }
#         self.last_y = {'obstacle_safe': None, 'obstacle_danger': None, 'obstacle_blue': None}
#         self.last_time = time.time()
#         self.last_teleop_time = 0.0


#         self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
#         self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
#         self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
#         self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
#         self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/lpl/status', 10)

#     def lpl_mode_callback(self, msg):
#         self.lpl_active = msg.data

#     def auth_callback(self, msg):
#         # Allow human to restore authority via UI IF they haven't crashed
#         if self.manual_lockout and not self.crashed:
#             self.manual_lockout = False
#             self.blue_box_locked_permanently = False
#             self.get_logger().info("Authority Restored by User.")

#     def teleop_callback(self, msg):
#         self.last_teleop_time = time.time()  # <--- ADD THIS LINE
        
#         # Allow human to drive during a lockout (unless crashed)
#         if (self.manual_lockout or not self.lpl_active) and not self.crashed:
#             self.pub_cmd.publish(msg)

#     def perception_callback(self, msg):
#         try:
#             if 'turtlebot3_waffle_pi' not in msg.name: return
#             rob_idx = msg.name.index('turtlebot3_waffle_pi')
#             rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

#             closest_dist = 999.0
#             closest_name = "None"
            
#             current_time = time.time()
#             dt = current_time - self.last_time
#             self.last_time = current_time

#             red_hesitation = False
#             yellow_caution = False

#             for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#                 if name in msg.name:
#                     idx = msg.name.index(name)
#                     ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#                     dist = math.hypot(ox - rx, oy - ry)
                    
#                     if dist < closest_dist:
#                         closest_dist = dist
#                         closest_name = name

#                     # 1. CRASH DETECTION
#                     if dist < 0.45:
#                         self.crashed = True

#                     # 2. DISTANCE TRIGGERS (For Safe Yellow and Blue objects)
#                     if dist < self.FAR_SIGHT_RADIUS:
#                         if name == 'obstacle_safe':
#                             yellow_caution = True
#                         elif name == 'obstacle_blue':
#                             self.blue_box_locked_permanently = True  # <--- Moved here!

#                     # 3. VARIANCE TRIGGER (Only for the Red Danger object)
#                     if name in self.histories and dt > 0:
#                         if self.last_y[name] is not None:
#                             vel = (oy - self.last_y[name]) / dt
#                             self.histories[name].append(vel)
#                         self.last_y[name] = oy

#                         if dist < self.FAR_SIGHT_RADIUS and len(self.histories[name]) >= 5:
#                             var = statistics.variance(self.histories[name])
                            
#                             # Only the Red Box needs to be jittery to trigger
#                             if var > 0.15 and name == 'obstacle_danger': 
#                                 red_hesitation = True
#             # for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#             #     if name in msg.name:
#             #         idx = msg.name.index(name)
#             #         ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#             #         dist = math.hypot(ox - rx, oy - ry)
                    
#             #         if dist < closest_dist:
#             #             closest_dist = dist
#             #             closest_name = name

#             #         # 1. CRASH DETECTION
#             #         if dist < 0.45:
#             #             self.crashed = True

#             #         # if name in self.histories and dt > 0:
#             #         #     if self.last_y[name] is not None:
#             #         #         vel = (oy - self.last_y[name]) / dt
#             #         #         self.histories[name].append(vel)
#             #         #     self.last_y[name] = oy

#             #         #     if dist < self.FAR_SIGHT_RADIUS and len(self.histories[name]) >= 5:
#             #         #         var = statistics.variance(self.histories[name])
                            
#             #         #         # Red and Blue require heavy jitter (>0.15)
#             #         #         if var > 0.15:
#             #         #             if name == 'obstacle_danger': 
#             #         #                 red_hesitation = True
#             #         #             elif name == 'obstacle_blue': 
#             #         #                 self.blue_box_locked_permanently = True
                            
#             #         #         # Yellow is highly sensitive (>0.02) to ensure it triggers
#             #         #         elif var > 0.02 and name == 'obstacle_safe':
#             #         #             yellow_caution = True

#             #         if name in self.histories and dt > 0:
#             #             if self.last_y[name] is not None:
#             #                 vel = (oy - self.last_y[name]) / dt
#             #                 self.histories[name].append(vel)
#             #             self.last_y[name] = oy

#             #             if dist < self.FAR_SIGHT_RADIUS:
#             #                 # 1. Check Yellow Box purely based on distance
#             #                 if name == 'obstacle_safe':
#             #                     yellow_caution = True
                                
#             #                 # 2. Keep Variance checks only for moving objects (Red/Blue)
#             #                 elif len(self.histories[name]) >= 5:
#             #                     var = statistics.variance(self.histories[name])
#             #                     if var > 0.15:
#             #                         if name == 'obstacle_danger': 
#             #                             red_hesitation = True
#             #                         elif name == 'obstacle_blue': 
#             #                             self.blue_box_locked_permanently = True

#             # --- 1. CRASH HANDLER (Overrides Everything) ---
#             if self.crashed:
#                 self.target_confidence = 0.0
#                 self.current_confidence = 0.0 
#                 self.manual_lockout = True
#                 self.publish_status("DANGER STOP", "COLLISION DETECTED!")
#                 return

#             # --- 2. LPL BYPASS (If LPL is OFF) ---
#             if not self.lpl_active:
#                 self.target_confidence = 1.0
#                 self.manual_lockout = False
#                 self.apply_smoothing(dt)
#                 self.publish_status("LPL DISABLED", closest_name)
#                 return

#             # --- 3. BLUE BOX LOGIC (Permanent Lockout -> Human Teleop) ---
#             if self.blue_box_locked_permanently:
#                 self.target_confidence = 0.0
#                 self.manual_lockout = True
#                 self.apply_smoothing(dt)
#                 self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                 return

#             # --- 4. RED BOX LOGIC (Fluid Evasion) ---
#             if red_hesitation:
#                 if not self.was_red_hesitating:
#                     self.red_pause_start = time.time()
#                     self.was_red_hesitating = True
                
#                 # FIX Reduced from 2.0 seconds to 0.5 seconds. 
                
#                 if time.time() - self.red_pause_start < 0.5:
#                     self.target_confidence = 0.0
#                     self.apply_smoothing(dt)
#                     self.publish_status("HESITATING", "ASSESSING THREAT")
#                     return
#                 else:
#                     # FIX 2: Increased minimum confidence from 0.15 to 0.50
#                     # The robot will now slow down to 50% speed instead of 15% 
#                     # while dodging the red box, making it much faster.
#                     dist_scale = (closest_dist - 0.45) / (self.FAR_SIGHT_RADIUS - 0.45)
#                     self.target_confidence = max(0.50, min(1.0, dist_scale))
#                     self.apply_smoothing(dt)
#                     self.publish_status("MODERATING", f"EVADING THREAT: {int(closest_dist*100)}cm")
#                     return
#             else:
#                 self.was_red_hesitating = False

#             # --- 5. YELLOW BOX LOGIC (Cautious Mode) ---
#             if yellow_caution:
#                 self.target_confidence = 0.7
#                 self.apply_smoothing(dt)
#                 self.publish_status("CAUTIOUS", "MONITORING SAFE OBJECT")
#                 return

#             # --- 6. DEFAULT (Clear Path) ---
#             self.target_confidence = 1.0
#             self.apply_smoothing(dt)
            
#             if self.current_confidence < 0.9:
#                 state_text = "ACCELERATING"
#             else:
#                 state_text = "CONFIDENT"
                
#             self.publish_status(state_text, closest_name)

#         except Exception: pass

# ### Apply smmotiing 

#     # def apply_smoothing(self, dt):
#     #     rate_of_change = 0.5 * dt 
#     #     diff = self.target_confidence - self.current_confidence
        
#     #     if abs(diff) <= rate_of_change:
#     #         self.current_confidence = self.target_confidence
#     #     else:
#     #         self.current_confidence += math.copysign(rate_of_change, diff)

#     ### end smoothing

#     def apply_smoothing(self, dt):
#         # CHANGED: Increased from 0.5 to 2.5 for much faster acceleration
#         rate_of_change = 2.5 * dt 
#         diff = self.target_confidence - self.current_confidence
        
#         if abs(diff) <= rate_of_change:
#             self.current_confidence = self.target_confidence
#         else:
#             self.current_confidence += math.copysign(rate_of_change, diff)

#     # def nav_callback(self, msg):
#     #     if not self.lpl_active and not self.crashed:
#     #         self.pub_cmd.publish(msg)
#     #         return

#     #     if self.manual_lockout:
#     #         self.pub_cmd.publish(Twist()) 
#     #     else:
#     #         final_cmd = Twist()
#     #         final_cmd.linear.x = msg.linear.x * self.current_confidence
#     #         final_cmd.angular.z = msg.angular.z * self.current_confidence
#     #         self.pub_cmd.publish(final_cmd)

#     def nav_callback(self, msg):
#         # 1. Hardware stop for crashes overrides everything
#         if self.crashed:
#             self.pub_cmd.publish(Twist())
#             return

#         # 2. Pass through if LPL is disabled
#         if not self.lpl_active:
#             self.pub_cmd.publish(msg)
#             return

#         # 3. MANUAL LOCKOUT LOGIC
#         if self.manual_lockout:
#             # If the user hasn't pressed a key in the last 0.2 seconds, hit the brakes!
#             # This stops the robot instantly when manual override begins, 
#             # and stops it from coasting when you release the WASD keys.
#             if time.time() - self.last_teleop_time > 0.2:
#                 self.pub_cmd.publish(Twist()) 
#             return 
            
#         # 4. NORMAL LPL BEHAVIOR
#         else:
#             final_cmd = Twist()
#             final_cmd.linear.x = msg.linear.x * self.current_confidence
#             final_cmd.angular.z = msg.angular.z * self.current_confidence
#             self.pub_cmd.publish(final_cmd)

#     def publish_status(self, state, obs_name):
#         status = {
#             "confidence": round(self.current_confidence, 2), 
#             "state": state, 
#             "object": obs_name, 
#             "override": self.manual_lockout
#         }
#         self.pub_status.publish(String(data=json.dumps(status)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = LPLNavGuard()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# from std_msgs.msg import String, Empty, Bool
# import json
# import statistics
# import math
# from collections import deque
# import time

# class LPLNavGuard(Node):
#     def __init__(self):
#         super().__init__('lpl_nav_guard')

#         self.FAR_SIGHT_RADIUS = 2.0    
#         self.PROXIMITY_CRIT = 0.5      
        
#         self.lpl_active = True
#         self.manual_lockout = False
#         self.blue_box_locked_permanently = False 
        
#         self.target_confidence = 1.0
#         self.current_confidence = 1.0 

#         # State Tracking
#         self.crashed = False
#         self.was_red_hesitating = False
#         self.red_pause_start = 0.0

#         self.histories = {
#             'obstacle_safe': deque(maxlen=10),
#             'obstacle_danger': deque(maxlen=10),
#             'obstacle_blue': deque(maxlen=10)   
#         }
#         self.last_y = {'obstacle_safe': None, 'obstacle_danger': None, 'obstacle_blue': None}
#         self.last_time = time.time()
        
#         # CRITICAL FIX: Initialize teleop time to prevent node crash on startup
#         self.last_teleop_time = 0.0 

#         self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
#         self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
#         self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
#         self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
#         self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/lpl/status', 10)

#     def lpl_mode_callback(self, msg):
#         self.lpl_active = msg.data

#     def auth_callback(self, msg):
#         # Allow human to restore authority via UI IF they haven't crashed
#         if self.manual_lockout and not self.crashed:
#             self.manual_lockout = False
#             self.blue_box_locked_permanently = False
#             self.get_logger().info("Authority Restored by User.")

#     def teleop_callback(self, msg):
#         # Update the dead-man switch timer the moment a key is pressed
#         self.last_teleop_time = time.time()  
        
#         # Allow human to drive during a lockout (unless crashed)
#         if (self.manual_lockout or not self.lpl_active) and not self.crashed:
#             self.pub_cmd.publish(msg)

#     def perception_callback(self, msg):
#         try:
#             if 'turtlebot3_waffle_pi' not in msg.name: return
#             rob_idx = msg.name.index('turtlebot3_waffle_pi')
#             rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

#             closest_dist = 999.0
#             closest_name = "None"
            
#             current_time = time.time()
#             dt = current_time - self.last_time
#             self.last_time = current_time

#             red_hesitation = False
#             yellow_caution = False

#             for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#                 if name in msg.name:
#                     idx = msg.name.index(name)
#                     ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#                     dist = math.hypot(ox - rx, oy - ry)
                    
#                     if dist < closest_dist:
#                         closest_dist = dist
#                         closest_name = name

#                     # 1. CRASH DETECTION
#                     if dist < 0.45:
#                         self.crashed = True

#                     # 2. DISTANCE TRIGGERS (For Safe Yellow and Static Blue objects)
#                     if dist < self.FAR_SIGHT_RADIUS:
#                         # Yellow Box triggered purely based on distance
#                         if name == 'obstacle_safe':
#                             yellow_caution = True
#                         # Blue Box triggered purely based on distance
#                         elif name == 'obstacle_blue':
#                             self.blue_box_locked_permanently = True

#                     # 3. VARIANCE TRIGGER (Only for the Red Danger object)
#                     if name in self.histories and dt > 0:
#                         if self.last_y[name] is not None:
#                             vel = (oy - self.last_y[name]) / dt
#                             self.histories[name].append(vel)
#                         self.last_y[name] = oy

#                         if dist < self.FAR_SIGHT_RADIUS and len(self.histories[name]) >= 5:
#                             var = statistics.variance(self.histories[name])
                            
#                             # Only the Red Box needs to be jittery to trigger
#                             if var > 0.15 and name == 'obstacle_danger': 
#                                 red_hesitation = True

#             # --- 1. CRASH HANDLER (Overrides Everything) ---
#             if self.crashed:
#                 self.target_confidence = 0.0
#                 self.current_confidence = 0.0 
#                 self.manual_lockout = True
#                 self.publish_status("DANGER STOP", "COLLISION DETECTED!")
#                 return

#             # --- 2. LPL BYPASS (If LPL is OFF) ---
#             if not self.lpl_active:
#                 self.target_confidence = 1.0
#                 self.manual_lockout = False
#                 self.apply_smoothing(dt)
#                 self.publish_status("LPL DISABLED", closest_name)
#                 return

#             # --- 3. BLUE BOX LOGIC (Permanent Lockout -> Human Teleop) ---
#             if self.blue_box_locked_permanently:
#                 self.target_confidence = 0.0
#                 self.manual_lockout = True
#                 self.apply_smoothing(dt)
#                 self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                 return

#             # --- 4. RED BOX LOGIC (Fluid Evasion) ---
#             if red_hesitation:
#                 if not self.was_red_hesitating:
#                     self.red_pause_start = time.time()
#                     self.was_red_hesitating = True
                
#                 # Snappy 0.5s pause to assess threat rather than dying for 2 seconds
#                 if time.time() - self.red_pause_start < 0.5:
#                     self.target_confidence = 0.0
#                     self.apply_smoothing(dt)
#                     self.publish_status("HESITATING", "ASSESSING THREAT")
#                     return
#                 else:
#                     # Fluid evasion: drops to 50% speed instead of 15% crawl
#                     dist_scale = (closest_dist - 0.45) / (self.FAR_SIGHT_RADIUS - 0.45)
#                     self.target_confidence = max(0.50, min(1.0, dist_scale))
#                     self.apply_smoothing(dt)
#                     self.publish_status("MODERATING", f"EVADING THREAT: {int(closest_dist*100)}cm")
#                     return
#             else:
#                 self.was_red_hesitating = False

#             # --- 5. YELLOW BOX LOGIC (Cautious Mode) ---
#             if yellow_caution:
#                 self.target_confidence = 0.7
#                 self.apply_smoothing(dt)
#                 self.publish_status("CAUTIOUS", "MONITORING SAFE OBJECT")
#                 return

#             # --- 6. DEFAULT (Clear Path) ---
#             self.target_confidence = 1.0
#             self.apply_smoothing(dt)
            
#             if self.current_confidence < 0.9:
#                 state_text = "ACCELERATING"
#             else:
#                 state_text = "CONFIDENT"
                
#             self.publish_status(state_text, closest_name)

#         except Exception: pass

#     def apply_smoothing(self, dt):
#         # Increased to 2.5 so the robot snappy accelerates after passing obstacles
#         rate_of_change = 2.5 * dt 
#         diff = self.target_confidence - self.current_confidence
        
#         if abs(diff) <= rate_of_change:
#             self.current_confidence = self.target_confidence
#         else:
#             self.current_confidence += math.copysign(rate_of_change, diff)

#     def nav_callback(self, msg):
#         # 1. Hardware stop for crashes overrides everything
#         if self.crashed:
#             self.pub_cmd.publish(Twist())
#             return

#         # 2. Pass through if LPL is disabled
#         if not self.lpl_active:
#             self.pub_cmd.publish(msg)
#             return

#         # 3. MANUAL LOCKOUT LOGIC (Dead-Man Switch)
#         if self.manual_lockout:
#             # If the user hasn't pressed a key in the last 0.2 seconds, hit the brakes!
#             # This stops the robot instantly when manual override begins, 
#             # and stops it from coasting when you release the WASD keys.
#             if time.time() - self.last_teleop_time > 0.2:
#                 self.pub_cmd.publish(Twist()) 
#             return 
            
#         # 4. NORMAL LPL BEHAVIOR
#         else:
#             final_cmd = Twist()
#             final_cmd.linear.x = msg.linear.x * self.current_confidence
#             final_cmd.angular.z = msg.angular.z * self.current_confidence
#             self.pub_cmd.publish(final_cmd)

#     def publish_status(self, state, obs_name):
#         status = {
#             "confidence": round(self.current_confidence, 2), 
#             "state": state, 
#             "object": obs_name, 
#             "override": self.manual_lockout
#         }
#         self.pub_status.publish(String(data=json.dumps(status)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = LPLNavGuard()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# from std_msgs.msg import String, Empty, Bool
# import json
# import math
# import time

# class LPLNavGuard(Node):
#     def __init__(self):
#         super().__init__('lpl_nav_guard')

#         self.FAR_SIGHT_RADIUS = 2.0    
#         self.PROXIMITY_CRIT = 0.5      
        
#         self.lpl_active = True
#         self.manual_lockout = False
#         self.blue_box_locked_permanently = False 
        
#         self.current_confidence = 1.0 

#         # State Tracking
#         self.crashed = False
#         self.was_red_paused = False
#         self.red_pause_start = 0.0

#         self.last_time = time.time()
#         self.last_teleop_time = 0.0 

#         self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
#         self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
#         self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
#         self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
#         self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/lpl/status', 10)

#     def lpl_mode_callback(self, msg):
#         self.lpl_active = msg.data

#     def auth_callback(self, msg):
#         # Allow human to restore authority via UI IF they haven't crashed
#         if self.manual_lockout and not self.crashed:
#             self.manual_lockout = False
#             self.blue_box_locked_permanently = False
#             self.get_logger().info("Authority Restored by User.")

#     def teleop_callback(self, msg):
#         # Update the dead-man switch timer the moment a key is pressed
#         self.last_teleop_time = time.time()  
        
#         # Allow human to drive during a lockout (unless crashed)
#         if (self.manual_lockout or not self.lpl_active) and not self.crashed:
#             self.pub_cmd.publish(msg)

#     def perception_callback(self, msg):
#         try:
#             if 'turtlebot3_waffle_pi' not in msg.name: return
#             rob_idx = msg.name.index('turtlebot3_waffle_pi')
#             rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

#             closest_dist = 999.0
#             closest_name = "None"

#             red_caution = False
#             yellow_caution = False

#             for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#                 if name in msg.name:
#                     idx = msg.name.index(name)
#                     ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#                     dist = math.hypot(ox - rx, oy - ry)
                    
#                     if dist < closest_dist:
#                         closest_dist = dist
#                         closest_name = name

#                     # 1. CRASH DETECTION
#                     if dist < 0.45:
#                         self.crashed = True

#                     # 2. DISTANCE TRIGGERS (Trigger purely on distance for ALL boxes)
#                     if dist < self.FAR_SIGHT_RADIUS:
#                         if name == 'obstacle_safe':
#                             yellow_caution = True
#                         elif name == 'obstacle_danger':
#                             red_caution = True
#                         elif name == 'obstacle_blue':
#                             self.blue_box_locked_permanently = True

#             # --- 1. CRASH HANDLER (Overrides Everything) ---
#             if self.crashed:
#                 self.current_confidence = 0.0 
#                 self.manual_lockout = True
#                 self.publish_status("DANGER STOP", "COLLISION DETECTED!")
#                 return

#             # --- 2. LPL BYPASS (If LPL is OFF) ---
#             if not self.lpl_active:
#                 self.current_confidence = 1.0
#                 self.manual_lockout = False
#                 self.publish_status("LPL DISABLED", closest_name)
#                 return

#             # --- 3. BLUE BOX LOGIC (Permanent Lockout -> Human Teleop) ---
#             if self.blue_box_locked_permanently:
#                 self.current_confidence = 0.0
#                 self.manual_lockout = True
#                 self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                 return

#             # --- 4. RED BOX LOGIC (3s Stop, No UI change, then Cautious mode) ---
#             if red_caution:
#                 # If we haven't paused yet, start the timer
#                 if not self.was_red_paused:
#                     self.red_pause_start = time.time()
#                     self.was_red_paused = True
                
#                 # Check if we are still inside the 3-second window
#                 if time.time() - self.red_pause_start < 3.0:
#                     self.current_confidence = 0.0
#                     self.publish_status("CAUTIOUS", "EVADING DANGER")
#                     return
#                 else:
#                     # After 3 seconds, proceed exactly as the yellow box. 
#                     self.current_confidence = 0.7
#                     self.publish_status("CAUTIOUS", "EVADING DANGER")
#                     return
#             else:
#                 # Reset the pause flag if the red box leaves our sight radius
#                 self.was_red_paused = False

#             # --- 5. YELLOW BOX LOGIC (Cautious Mode) ---
#             if yellow_caution:
#                 self.current_confidence = 0.7
#                 self.publish_status("CAUTIOUS", "MONITORING SAFE OBJECT")
#                 return

#             # --- 6. DEFAULT (Clear Path) ---
#             self.current_confidence = 1.0
#             self.publish_status("CONFIDENT", closest_name)

#         except Exception: pass

#     def nav_callback(self, msg):
#         # 1. Hardware stop for crashes overrides everything
#         if self.crashed:
#             self.pub_cmd.publish(Twist())
#             return

#         # 2. Pass through if LPL is disabled
#         if not self.lpl_active:
#             self.pub_cmd.publish(msg)
#             return

#         # 3. MANUAL LOCKOUT LOGIC (Dead-Man Switch)
#         if self.manual_lockout:
#             if time.time() - self.last_teleop_time > 0.2:
#                 self.pub_cmd.publish(Twist()) 
#             return 
            
#         # 4. NORMAL LPL BEHAVIOR
#         else:
#             final_cmd = Twist()
#             # Speed snaps instantly to current_confidence
#             final_cmd.linear.x = msg.linear.x * self.current_confidence
#             final_cmd.angular.z = msg.angular.z * self.current_confidence
#             self.pub_cmd.publish(final_cmd)

#     def publish_status(self, state, obs_name):
#         status = {
#             "confidence": round(self.current_confidence, 2), 
#             "state": state, 
#             "object": obs_name, 
#             "override": self.manual_lockout
#         }
#         self.pub_status.publish(String(data=json.dumps(status)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = LPLNavGuard()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# from std_msgs.msg import String, Empty, Bool
# import json
# import math
# import time

# class LPLNavGuard(Node):
#     def __init__(self):
#         super().__init__('lpl_nav_guard')

#         self.FAR_SIGHT_RADIUS = 2.0    
#         self.PROXIMITY_CRIT = 0.5      
        
#         self.lpl_active = True
#         self.manual_lockout = False
        
#         self.current_confidence = 1.0 

#         # State Tracking
#         self.crashed = False
#         self.was_red_paused = False
#         self.red_pause_start = 0.0
        
#         # New trackers for Blue Box 3-second pause
#         self.was_blue_paused = False
#         self.blue_pause_start = 0.0

#         self.last_time = time.time()
#         self.last_teleop_time = 0.0 

#         self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
#         self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
#         self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
#         self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
#         self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/lpl/status', 10)

#     def lpl_mode_callback(self, msg):
#         self.lpl_active = msg.data

#     def auth_callback(self, msg):
#         if self.manual_lockout and not self.crashed:
#             self.manual_lockout = False
#             self.get_logger().info("Authority Restored by User.")

#     def teleop_callback(self, msg):
#         self.last_teleop_time = time.time()  
#         if (self.manual_lockout or not self.lpl_active) and not self.crashed:
#             self.pub_cmd.publish(msg)

#     def perception_callback(self, msg):
#         try:
#             if 'turtlebot3_waffle_pi' not in msg.name: return
#             rob_idx = msg.name.index('turtlebot3_waffle_pi')
#             rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

#             closest_dist = 999.0
#             closest_name = "None"

#             red_caution = False
#             yellow_caution = False
#             blue_present = False

#             for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#                 if name in msg.name:
#                     idx = msg.name.index(name)
#                     ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#                     dist = math.hypot(ox - rx, oy - ry)
                    
#                     if dist < closest_dist:
#                         closest_dist = dist
#                         closest_name = name

#                     if dist < 0.45:
#                         self.crashed = True

#                     if dist < self.FAR_SIGHT_RADIUS:
#                         if name == 'obstacle_safe':
#                             yellow_caution = True
#                         elif name == 'obstacle_danger':
#                             red_caution = True
#                         elif name == 'obstacle_blue':
#                             blue_present = True

#             # --- 1. CRASH HANDLER ---
#             if self.crashed:
#                 self.current_confidence = 0.0 
#                 self.manual_lockout = True
#                 self.publish_status("DANGER STOP", "COLLISION DETECTED!")
#                 return

#             # --- 2. LPL BYPASS ---
#             if not self.lpl_active:
#                 self.current_confidence = 1.0
#                 self.manual_lockout = False
#                 self.publish_status("LPL DISABLED", closest_name)
#                 return

#             # --- 3. BLUE BOX LOGIC (3s Pause -> MANUAL OVERRIDE) ---
#             if blue_present:
#                 if not self.manual_lockout:
#                     # If we haven't paused yet, start the timer
#                     if not self.was_blue_paused:
#                         self.blue_pause_start = time.time()
#                         self.was_blue_paused = True
                    
#                     # Stop for 3 seconds first (Trigger RED UI)
#                     if time.time() - self.blue_pause_start < 3.0:
#                         self.current_confidence = 0.0
#                         self.publish_status("PAUSED", "ASSESSING THREAT") 
#                         return
#                     else:
#                         # After 3 seconds, hand over authority (Trigger BLUE UI)
#                         self.current_confidence = 0.0
#                         self.manual_lockout = True
#                         self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                         return
#                 else:
#                     # We are already in lockout and the blue box is still here
#                     self.current_confidence = 0.0
#                     self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                     return
#             else:
#                 self.was_blue_paused = False

#             # --- 4. RED BOX LOGIC (3s Pause -> CAUTIOUS) ---
#             if red_caution:
#                 if not self.was_red_paused:
#                     self.red_pause_start = time.time()
#                     self.was_red_paused = True
                
#                 if time.time() - self.red_pause_start < 3.0:
#                     self.current_confidence = 0.0
#                     self.publish_status("PAUSED", "ASSESSING THREAT") 
#                     return
#                 else:
#                     self.current_confidence = 0.7
#                     self.publish_status("CAUTIOUS", "EVADING DANGER") 
#                     return
#             else:
#                 self.was_red_paused = False

#             # --- 5. YELLOW BOX LOGIC ---
#             if yellow_caution:
#                 self.current_confidence = 0.7
#                 self.publish_status("CAUTIOUS", "MONITORING SAFE OBJECT")
#                 return

#             # --- 6. DEFAULT (Clear Path) ---
#             self.current_confidence = 1.0
#             self.publish_status("CONFIDENT", closest_name)

#         except Exception: pass

#     def nav_callback(self, msg):
#         if self.crashed:
#             self.pub_cmd.publish(Twist())
#             return

#         if not self.lpl_active:
#             self.pub_cmd.publish(msg)
#             return

#         # If locked out, robot stays physically stopped (unless WASD is pressed)
#         if self.manual_lockout:
#             if time.time() - self.last_teleop_time > 0.2:
#                 self.pub_cmd.publish(Twist()) 
#             return 
#         else:
#             final_cmd = Twist()
#             final_cmd.linear.x = msg.linear.x * self.current_confidence
#             final_cmd.angular.z = msg.angular.z * self.current_confidence
#             self.pub_cmd.publish(final_cmd)

#     def publish_status(self, state, obs_name):
#         status = {
#             "confidence": round(self.current_confidence, 2), 
#             "state": state, 
#             "object": obs_name, 
#             "override": self.manual_lockout # This tells the UI we are still waiting for Spacebar
#         }
#         self.pub_status.publish(String(data=json.dumps(status)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = LPLNavGuard()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# from std_msgs.msg import String, Empty, Bool
# import json
# import math
# import time

# class LPLNavGuard(Node):
#     def __init__(self):
#         super().__init__('lpl_nav_guard')

#         self.FAR_SIGHT_RADIUS = 2.0    
#         self.PROXIMITY_CRIT = 0.5      
        
#         self.lpl_active = True
#         self.manual_lockout = False
        
#         self.target_confidence = 1.0 
#         self.current_confidence = 1.0 

#         # State Tracking
#         self.crashed = False
#         self.was_red_paused = False
#         self.red_pause_start = 0.0
        
#         self.was_blue_paused = False
#         self.blue_pause_start = 0.0

#         self.last_time = time.time()
#         self.last_teleop_time = 0.0 

#         self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
#         self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
#         self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
#         self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
#         self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/lpl/status', 10)

#     def lpl_mode_callback(self, msg):
#         self.lpl_active = msg.data

#     def auth_callback(self, msg):
#         if self.manual_lockout and not self.crashed:
#             self.manual_lockout = False
#             self.get_logger().info("Authority Restored by User.")

#     def teleop_callback(self, msg):
#         self.last_teleop_time = time.time()  
#         if (self.manual_lockout or not self.lpl_active) and not self.crashed:
#             self.pub_cmd.publish(msg)

#     def apply_smoothing(self, dt):
#         # A rate of 2.0 means it takes 0.5 seconds to drop from 100% to 0%
#         # It's fast enough to save the robot, but slow enough to smoothly slide the UI!
#         rate_of_change = 2.0 * dt 
#         diff = self.target_confidence - self.current_confidence
        
#         if abs(diff) <= rate_of_change:
#             self.current_confidence = self.target_confidence
#         else:
#             self.current_confidence += math.copysign(rate_of_change, diff)

#     def perception_callback(self, msg):
#         try:
#             if 'turtlebot3_waffle_pi' not in msg.name: return
#             rob_idx = msg.name.index('turtlebot3_waffle_pi')
#             rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

#             current_time = time.time()
#             dt = current_time - self.last_time
#             self.last_time = current_time

#             closest_dist = 999.0
#             closest_name = "None"

#             red_caution = False
#             yellow_caution = False
#             blue_present = False

#             for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#                 if name in msg.name:
#                     idx = msg.name.index(name)
#                     ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#                     dist = math.hypot(ox - rx, oy - ry)
                    
#                     if dist < closest_dist:
#                         closest_dist = dist
#                         closest_name = name

#                     if dist < 0.45:
#                         self.crashed = True

#                     if dist < self.FAR_SIGHT_RADIUS:
#                         if name == 'obstacle_safe':
#                             yellow_caution = True
#                         elif name == 'obstacle_danger':
#                             red_caution = True
#                         elif name == 'obstacle_blue':
#                             blue_present = True

#             # --- 1. CRASH HANDLER ---
#             if self.crashed:
#                 self.target_confidence = 0.0 
#                 self.manual_lockout = True
#                 self.apply_smoothing(dt)
#                 self.publish_status("DANGER STOP", "COLLISION DETECTED!")
#                 return

#             # --- 2. LPL BYPASS ---
#             if not self.lpl_active:
#                 self.target_confidence = 1.0
#                 self.manual_lockout = False
#                 self.apply_smoothing(dt)
#                 self.publish_status("LPL DISABLED", closest_name)
#                 return

#             # --- 3. BLUE BOX LOGIC (3s Pause -> MANUAL OVERRIDE) ---
#             if blue_present:
#                 if not self.manual_lockout:
#                     if not self.was_blue_paused:
#                         self.blue_pause_start = time.time()
#                         self.was_blue_paused = True
                    
#                     if time.time() - self.blue_pause_start < 3.0:
#                         self.target_confidence = 0.0
#                         self.apply_smoothing(dt)
#                         self.publish_status("PAUSED", "ASSESSING THREAT") 
#                         return
#                     else:
#                         self.target_confidence = 0.0
#                         self.manual_lockout = True
#                         self.apply_smoothing(dt)
#                         self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                         return
#                 else:
#                     self.target_confidence = 0.0
#                     self.apply_smoothing(dt)
#                     self.publish_status("MANUAL OVERRIDE", "BLUE ZONE: HUMAN IN CONTROL")
#                     return
#             else:
#                 self.was_blue_paused = False

#             # --- 4. RED BOX LOGIC (3s Pause -> CAUTIOUS) ---
#             if red_caution:
#                 if not self.was_red_paused:
#                     self.red_pause_start = time.time()
#                     self.was_red_paused = True
                
#                 if time.time() - self.red_pause_start < 3.0:
#                     self.target_confidence = 0.0
#                     self.apply_smoothing(dt)
#                     self.publish_status("PAUSED", "ASSESSING THREAT") 
#                     return
#                 else:
#                     self.target_confidence = 0.7
#                     self.apply_smoothing(dt)
#                     self.publish_status("CAUTIOUS", "EVADING DANGER") 
#                     return
#             else:
#                 self.was_red_paused = False

#             # --- 5. YELLOW BOX LOGIC ---
#             if yellow_caution:
#                 self.target_confidence = 0.7
#                 self.apply_smoothing(dt)
#                 self.publish_status("CAUTIOUS", "MONITORING SAFE OBJECT")
#                 return

#             # --- 6. DEFAULT (Clear Path) ---
#             self.target_confidence = 1.0
#             self.apply_smoothing(dt)
#             self.publish_status("CONFIDENT", closest_name)

#         except Exception: pass

#     def nav_callback(self, msg):
#         if self.crashed:
#             self.pub_cmd.publish(Twist())
#             return

#         if not self.lpl_active:
#             self.pub_cmd.publish(msg)
#             return

#         if self.manual_lockout:
#             if time.time() - self.last_teleop_time > 0.2:
#                 self.pub_cmd.publish(Twist()) 
#             return 
#         else:
#             final_cmd = Twist()
#             # Because of smoothing, speed gracefully slides down!
#             final_cmd.linear.x = msg.linear.x * self.current_confidence
#             final_cmd.angular.z = msg.angular.z * self.current_confidence
#             self.pub_cmd.publish(final_cmd)

#     def publish_status(self, state, obs_name):
#         status = {
#             "confidence": round(self.current_confidence, 3), 
#             "state": state, 
#             "object": obs_name, 
#             "override": self.manual_lockout 
#         }
#         self.pub_status.publish(String(data=json.dumps(status)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = LPLNavGuard()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# from std_msgs.msg import String, Empty, Bool
# import json
# import math
# import time

# class LPLNavGuard(Node):
#     def __init__(self):
#         super().__init__('lpl_nav_guard')

#         self.FAR_SIGHT_RADIUS = 2.0    
#         self.PROXIMITY_CRIT = 0.5      
        
#         self.lpl_active = True
#         self.manual_lockout = False
        
#         self.target_confidence = 1.0 
#         self.current_confidence = 1.0 

#         # State Tracking
#         self.crashed = False
#         self.was_red_paused = False
#         self.red_pause_start = 0.0
        
#         self.was_blue_paused = False
#         self.blue_pause_start = 0.0

#         self.last_time = time.time()
#         self.last_teleop_time = 0.0 

#         self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
#         self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
#         self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
#         self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
#         self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.pub_status = self.create_publisher(String, '/lpl/status', 10)

#     def lpl_mode_callback(self, msg):
#         self.lpl_active = msg.data

#     def auth_callback(self, msg):
#         if self.manual_lockout and not self.crashed:
#             self.manual_lockout = False
#             self.get_logger().info("Authority Restored by User.")

#     def teleop_callback(self, msg):
#         self.last_teleop_time = time.time()  
#         if (self.manual_lockout or not self.lpl_active) and not self.crashed:
#             self.pub_cmd.publish(msg)

#     def apply_smoothing(self, dt):
#         # Takes ~0.5 seconds to slide smoothly from 100% to 0%
#         rate_of_change = 2.0 * dt 
#         diff = self.target_confidence - self.current_confidence
        
#         if abs(diff) <= rate_of_change:
#             self.current_confidence = self.target_confidence
#         else:
#             self.current_confidence += math.copysign(rate_of_change, diff)

#     def perception_callback(self, msg):
#         try:
#             if 'turtlebot3_waffle_pi' not in msg.name: return
#             rob_idx = msg.name.index('turtlebot3_waffle_pi')
#             rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

#             current_time = time.time()
#             dt = current_time - self.last_time
#             self.last_time = current_time

#             closest_dist = 999.0
#             closest_name = "None"

#             red_caution = False
#             yellow_caution = False
#             blue_present = False

#             for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
#                 if name in msg.name:
#                     idx = msg.name.index(name)
#                     ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
#                     dist = math.hypot(ox - rx, oy - ry)
                    
#                     if dist < closest_dist:
#                         closest_dist = dist
#                         closest_name = name

#                     if dist < 0.45:
#                         self.crashed = True

#                     if dist < self.FAR_SIGHT_RADIUS:
#                         if name == 'obstacle_safe':
#                             yellow_caution = True
#                         elif name == 'obstacle_danger':
#                             red_caution = True
#                         elif name == 'obstacle_blue':
#                             blue_present = True

#             # --- 1. CRASH HANDLER ---
#             if self.crashed:
#                 self.target_confidence = 0.0 
#                 self.manual_lockout = True
#                 self.apply_smoothing(dt)
#                 self.publish_status("DANGER STOP", closest_name)
#                 return

#             # --- 2. LPL BYPASS ---
#             if not self.lpl_active:
#                 self.target_confidence = 1.0
#                 self.manual_lockout = False
#                 self.apply_smoothing(dt)
#                 self.publish_status("LPL DISABLED", closest_name)
#                 return

#             # --- 3. BLUE BOX LOGIC (3s Pause -> MANUAL OVERRIDE) ---
#             if blue_present:
#                 if not self.manual_lockout:
#                     if not self.was_blue_paused:
#                         self.blue_pause_start = time.time()
#                         self.was_blue_paused = True
                    
#                     if time.time() - self.blue_pause_start < 3.0:
#                         self.target_confidence = 0.0
#                         self.apply_smoothing(dt)
#                         self.publish_status("PAUSED", closest_name) # Explicitly pass the obstacle name!
#                         return
#                     else:
#                         self.target_confidence = 0.0
#                         self.manual_lockout = True
#                         self.apply_smoothing(dt)
#                         self.publish_status("MANUAL OVERRIDE", closest_name)
#                         return
#                 else:
#                     self.target_confidence = 0.0
#                     self.apply_smoothing(dt)
#                     self.publish_status("MANUAL OVERRIDE", closest_name)
#                     return
#             else:
#                 self.was_blue_paused = False

#             # --- 4. RED BOX LOGIC (3s Pause -> CAUTIOUS) ---
#             if red_caution:
#                 if not self.was_red_paused:
#                     self.red_pause_start = time.time()
#                     self.was_red_paused = True
                
#                 if time.time() - self.red_pause_start < 3.0:
#                     self.target_confidence = 0.0
#                     self.apply_smoothing(dt)
#                     self.publish_status("PAUSED", closest_name) 
#                     return
#                 else:
#                     self.target_confidence = 0.7
#                     self.apply_smoothing(dt)
#                     self.publish_status("CAUTIOUS", closest_name) 
#                     return
#             else:
#                 self.was_red_paused = False

#             # --- 5. YELLOW BOX LOGIC ---
#             if yellow_caution:
#                 self.target_confidence = 0.7
#                 self.apply_smoothing(dt)
#                 self.publish_status("CAUTIOUS", closest_name)
#                 return

#             # --- 6. DEFAULT (Clear Path) ---
#             self.target_confidence = 1.0
#             self.apply_smoothing(dt)
#             self.publish_status("CONFIDENT", closest_name)

#         except Exception: pass

#     def nav_callback(self, msg):
#         if self.crashed:
#             self.pub_cmd.publish(Twist())
#             return

#         if not self.lpl_active:
#             self.pub_cmd.publish(msg)
#             return

#         if self.manual_lockout:
#             if time.time() - self.last_teleop_time > 0.2:
#                 self.pub_cmd.publish(Twist()) 
#             return 
#         else:
#             final_cmd = Twist()
#             # Because of smoothing, speed gracefully slides down!
#             final_cmd.linear.x = msg.linear.x * self.current_confidence
#             final_cmd.angular.z = msg.angular.z * self.current_confidence
#             self.pub_cmd.publish(final_cmd)

#     def publish_status(self, state, obs_name):
#         status = {
#             "confidence": round(self.current_confidence, 3), 
#             "state": state, 
#             "object": obs_name, 
#             "override": self.manual_lockout 
#         }
#         self.pub_status.publish(String(data=json.dumps(status)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = LPLNavGuard()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Empty, Bool
import json
import math
import time

from lpl_demo import model_states

class LPLNavGuard(Node):
    def __init__(self):
        super().__init__('lpl_nav_guard')

        self.FAR_SIGHT_RADIUS = 2.0    
        self.PROXIMITY_CRIT = 0.5      
        
        self.lpl_active = True
        self.manual_lockout = False
        self.recovering_confidence = False # NEW: Decouples authority from confidence
        
        self.target_confidence = 1.0 
        self.current_confidence = 1.0 

        # State Tracking
        self.crashed = False
        self.was_red_paused = False
        self.red_pause_start = 0.0
        
        self.was_blue_paused = False
        self.blue_pause_start = 0.0

        self.last_time = time.time()
        self.last_teleop_time = 0.0 

        # Autonomy commands (lpl_nav2_demo, or Nav2 via lpl_nav2.launch.py), TwistStamped on Jazzy.
        # Not /cmd_vel_nav: Nav2 on Jazzy uses that name internally for the raw controller output.
        self.sub_nav = self.create_subscription(TwistStamped, '/cmd_vel_auto', self.nav_callback, 10)
        self.sub_teleop = self.create_subscription(Twist, '/cmd_vel_input', self.teleop_callback, 10)
        self.sub_models = self.create_subscription(TFMessage, model_states.MODEL_STATES_TOPIC, self.perception_callback, 10)
        self.sub_lpl_mode = self.create_subscription(Bool, '/lpl/activation', self.lpl_mode_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.auth_callback, 10)
        
        # TurtleBot3 on Jazzy takes TwistStamped on /cmd_vel
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

    def lpl_mode_callback(self, msg):
        self.lpl_active = msg.data

    def auth_callback(self, msg):
        if self.manual_lockout and not self.crashed:
            self.manual_lockout = False
            # When the human gives back authority, tell the AI to rebuild confidence
            self.recovering_confidence = True 
            self.get_logger().info("Authority Restored. AI Rebuilding Confidence.")

    def teleop_callback(self, msg):
        self.last_teleop_time = time.time()  
        if (self.manual_lockout or not self.lpl_active) and not self.crashed:
            self.publish_cmd(msg)

    def apply_smoothing(self, dt):
        # If we are recovering from a Blue Box, fill the bar slower (takes ~1.3 seconds)
        # Otherwise, drop the confidence fast to save the robot (takes ~0.5 seconds)
        rate = 0.75 if self.recovering_confidence else 2.0
        rate_of_change = rate * dt 
        
        diff = self.target_confidence - self.current_confidence
        
        if abs(diff) <= rate_of_change:
            self.current_confidence = self.target_confidence
        else:
            self.current_confidence += math.copysign(rate_of_change, diff)

    def perception_callback(self, tf_msg):
        msg = model_states.from_tf(tf_msg)
        try:
            if 'turtlebot3_waffle_pi' not in msg.name: return
            rob_idx = msg.name.index('turtlebot3_waffle_pi')
            rx, ry = msg.pose[rob_idx].position.x, msg.pose[rob_idx].position.y

            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            closest_dist = 999.0
            closest_name = "None"

            red_caution = False
            yellow_caution = False
            blue_present = False

            for name in ['obstacle_safe', 'obstacle_danger', 'obstacle_blue']:
                if name in msg.name:
                    idx = msg.name.index(name)
                    ox, oy = msg.pose[idx].position.x, msg.pose[idx].position.y
                    dist = math.hypot(ox - rx, oy - ry)
                    
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_name = name

                    if dist < 0.45:
                        self.crashed = True

                    if dist < self.FAR_SIGHT_RADIUS:
                        if name == 'obstacle_safe':
                            yellow_caution = True
                        elif name == 'obstacle_danger':
                            red_caution = True
                        elif name == 'obstacle_blue':
                            blue_present = True

            # --- 0. RECOVERY STATE LOGIC ---
            if self.recovering_confidence:
                # If a new danger appears while rebuilding confidence, abort recovery immediately!
                if blue_present or red_caution or self.crashed:
                    self.recovering_confidence = False
                else:
                    self.target_confidence = 1.0
                    self.apply_smoothing(dt)
                    
                    if self.current_confidence >= 0.99:
                        self.recovering_confidence = False # Confidence fully recovered!
                    else:
                        # Publish "CAUTIOUS" so the Dashboard turns the text Yellow automatically
                        self.publish_status("CAUTIOUS", "REBUILDING AI MODEL")
                        return

            # --- 1. CRASH HANDLER ---
            if self.crashed:
                self.target_confidence = 0.0 
                self.manual_lockout = True
                self.apply_smoothing(dt)
                self.publish_status("DANGER STOP", closest_name)
                return

            # --- 2. LPL BYPASS ---
            if not self.lpl_active:
                self.target_confidence = 1.0
                self.manual_lockout = False
                self.apply_smoothing(dt)
                self.publish_status("LPL DISABLED", closest_name)
                return

            # --- 3. BLUE BOX LOGIC ---
            if blue_present:
                if not self.manual_lockout:
                    if not self.was_blue_paused:
                        self.blue_pause_start = time.time()
                        self.was_blue_paused = True
                    
                    if time.time() - self.blue_pause_start < 3.0:
                        self.target_confidence = 0.0
                        self.apply_smoothing(dt)
                        self.publish_status("PAUSED", closest_name) 
                        return
                    else:
                        self.target_confidence = 0.0
                        self.manual_lockout = True
                        self.apply_smoothing(dt)
                        self.publish_status("MANUAL OVERRIDE", closest_name)
                        return
                else:
                    self.target_confidence = 0.0
                    self.apply_smoothing(dt)
                    self.publish_status("MANUAL OVERRIDE", closest_name)
                    return
            else:
                self.was_blue_paused = False

            # --- 4. RED BOX LOGIC ---
            if red_caution:
                if not self.was_red_paused:
                    self.red_pause_start = time.time()
                    self.was_red_paused = True
                
                if time.time() - self.red_pause_start < 3.0:
                    self.target_confidence = 0.0
                    self.apply_smoothing(dt)
                    self.publish_status("PAUSED", closest_name) 
                    return
                else:
                    self.target_confidence = 0.7
                    self.apply_smoothing(dt)
                    self.publish_status("CAUTIOUS", closest_name) 
                    return
            else:
                self.was_red_paused = False

            # --- 5. YELLOW BOX LOGIC ---
            if yellow_caution:
                self.target_confidence = 0.7
                self.apply_smoothing(dt)
                self.publish_status("CAUTIOUS", closest_name)
                return

            # --- 6. DEFAULT (Clear Path) ---
            self.target_confidence = 1.0
            self.apply_smoothing(dt)
            self.publish_status("CONFIDENT", closest_name)

        except Exception: pass

    def nav_callback(self, stamped_msg):
        msg = stamped_msg.twist
        if self.crashed:
            self.publish_cmd(Twist())
            return

        if not self.lpl_active:
            self.publish_cmd(msg)
            return

        # NEW: If we are rebuilding confidence, force the robot to stay stopped!
        if self.recovering_confidence:
            self.publish_cmd(Twist())
            return

        if self.manual_lockout:
            if time.time() - self.last_teleop_time > 0.2:
                self.publish_cmd(Twist()) 
            return 
        else:
            final_cmd = Twist()
            final_cmd.linear.x = msg.linear.x * self.current_confidence
            final_cmd.angular.z = msg.angular.z * self.current_confidence
            self.publish_cmd(final_cmd)

    def publish_cmd(self, twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = twist
        self.pub_cmd.publish(stamped)

    def publish_status(self, state, obs_name):
        status = {
            "confidence": round(self.current_confidence, 3), 
            "state": state, 
            "object": obs_name, 
            "override": self.manual_lockout 
        }
        self.pub_status.publish(String(data=json.dumps(status)))

def main(args=None):
    rclpy.init(args=args)
    node = LPLNavGuard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()