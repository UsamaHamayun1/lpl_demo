#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import random
import math
import time  # <-- Moved to the top for clean Python practice

from lpl_demo import model_states

class ActorDirector(Node):
    def __init__(self):
        super().__init__('actor_director')
        
        # --- CONFIGURATION ---
        self.OBSTACLE_NAME = "obstacle_danger" # Must match Gazebo name
        self.Y_LIMIT_MIN = -3.0  # Right side of road edge
        self.Y_LIMIT_MAX = 3.0   # Left side of road edge
        
        self.pub_vel = self.create_publisher(Twist, '/actor_cmd_vel', 10)
        self.sub_state = self.create_subscription(TFMessage, model_states.MODEL_STATES_TOPIC, self.state_callback, 10)
        
        # --- FIXED TOPIC --- 
        # This now exactly matches what the dashboard publishes!
        self.create_subscription(String, '/lpl/actor_mode', self.mode_callback, 10)

        

        self.mode = "SMOOTH" 
        self.current_y = 0.0
        self.target_y = 0.0
        self.move_speed = 0.0
        self.state_initialized = False

        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop) 
        self.get_logger().info("ACTOR: READY (Random Target Walker)")

    def state_callback(self, tf_msg):
        msg = model_states.from_tf(tf_msg)
        try:
            if self.OBSTACLE_NAME in msg.name:
                idx = msg.name.index(self.OBSTACLE_NAME)
                self.current_y = msg.pose[idx].position.y
                self.state_initialized = True
        except: pass

    def mode_callback(self, msg):
        self.mode = msg.data.upper()
        self.get_logger().info(f"MODE CHANGED TO: {self.mode}")
        self.pick_new_target()

    def pick_new_target(self):
        self.target_y = random.uniform(self.Y_LIMIT_MIN, self.Y_LIMIT_MAX)
        self.move_speed = random.uniform(0.5, 2.5) 

    def control_loop(self):
        if not self.state_initialized: return

        cmd = Twist()
        t = time.time()
        
        if self.mode == "JITTER":
            # --- THE DRUNKEN SWAY ---
            # Lowering the multiplier from 15.0 to 1.5 slows down the wave.
            # Setting amplitude to 0.8 gives it a nice, wide sweeping speed.
            cmd.linear.y = 2 * math.sin(t * 10)
            
            # Keep it from flying off the map into the walls
            if self.current_y > self.Y_LIMIT_MAX: cmd.linear.y = -1.0
            if self.current_y < self.Y_LIMIT_MIN: cmd.linear.y = 1.0
            
            cmd.linear.x = 0.0 
            cmd.angular.z = 0.0

        else:
            # --- STABLE MODE ---
            # The box tries to return to the center (Y=0) smoothly
            if abs(self.current_y) > 0.05:
                cmd.linear.y = -0.8 * self.current_y 
            else:
                cmd.linear.y = 0.0
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub_vel.publish(cmd)

def main():
    rclpy.init()
    node = ActorDirector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()