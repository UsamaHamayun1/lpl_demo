# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from gazebo_msgs.msg import ModelStates
# import math

# class DemoPlanner(Node):
#     def __init__(self):
#         super().__init__('demo_planner')
#         self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
#         self.sub_state = self.create_subscription(ModelStates, '/gazebo/model_states', self.pose_callback, 10)
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_yaw = 0.0
#         self.timer = self.create_timer(0.1, self.drive_loop)

#     def pose_callback(self, msg):
#         try:
#             idx = msg.name.index('turtlebot3_waffle_pi')
#             self.robot_x = msg.pose[idx].position.x
#             self.robot_y = msg.pose[idx].position.y
#             q = msg.pose[idx].orientation
#             self.robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
#         except ValueError:
#             pass

#     def drive_loop(self):
#         cmd = Twist()
        
#         # Path logic
#         if self.robot_x < 2.5: target_y = 0.0
#         elif self.robot_x < 6.0: target_y = -1.2
#         elif self.robot_x < 10.0: target_y = 0.0
#         elif self.robot_x < 14.5: target_y = 1.0
#         elif self.robot_x < 25.0: target_y = 0.0 # Goal at 25.0
#         else:
#             self.pub_cmd.publish(Twist())
#             return

#         lookahead_x = 1.0
#         error_y = target_y - self.robot_y
#         desired_yaw = math.atan2(error_y, lookahead_x)
#         yaw_error = desired_yaw - self.robot_yaw
        
#         while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
#         while yaw_error < -math.pi: yaw_error += 2.0 * math.pi
        
#         # INCREASED BASE SPEED TO SHOW LPL DIFFERENCE
#         cmd.linear.x = 0.7 
#         cmd.angular.z = 1.8 * yaw_error
        
#         self.pub_cmd.publish(cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DemoPlanner()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

    #  if there is crash always stop for lpl and non lpl
    #  for the red obs before path avoiding , take a pause of 2 seconds , then take a wide angle (for lpl only) 
    #  for non_lpl increase of the robot (to show visible in cautions and normals states)   - less priority
    #  for lpl blue scenario ......   hand off authority, stop , obstacle gets out of the way and confidence goes back up to green once the obstacle is away as per lpl logic, the robot should remoain stoped for few seconds then autonomy is restored back after avoiding the blue   


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool
import math

from lpl_demo import model_states

class DemoPlanner(Node):
    def __init__(self):
        super().__init__('demo_planner')
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel_auto', 10)
        self.sub_state = self.create_subscription(TFMessage, model_states.MODEL_STATES_TOPIC, self.pose_callback, 10)
        self.sub_lpl = self.create_subscription(Bool, '/lpl/activation', self.lpl_callback, 10)
        
        self.lpl_active = True
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.timer = self.create_timer(0.1, self.drive_loop)

    def lpl_callback(self, msg):
        self.lpl_active = msg.data

    def pose_callback(self, tf_msg):
        msg = model_states.from_tf(tf_msg)
        try:
            idx = msg.name.index('turtlebot3_waffle_pi')
            self.robot_x = msg.pose[idx].position.x
            self.robot_y = msg.pose[idx].position.y
            q = msg.pose[idx].orientation
            self.robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        except ValueError:
            pass

    def drive_loop(self):
        cmd = Twist()
        
        # --- PATH LOGIC ---
        if self.robot_x < 2.5: target_y = 0.0
        elif self.robot_x < 6.0: target_y = -1.2
        elif self.robot_x < 10.0: target_y = 0.0
        elif self.robot_x < 14.5: 
            # DYNAMIC WIDE ANGLE: 
            # LPL takes a wide safe route (Y=1.6). Non-LPL takes a tight dangerous route (Y=0.6).
            target_y = 1.6 if self.lpl_active else 0.6
        elif self.robot_x < 25.0: target_y = 0.0 
        else:
            self.send(Twist())
            return

        lookahead_x = 1.0
        error_y = target_y - self.robot_y
        desired_yaw = math.atan2(error_y, lookahead_x)
        yaw_error = desired_yaw - self.robot_yaw
        
        while yaw_error > math.pi: yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi: yaw_error += 2.0 * math.pi
        
        # --- SPEED LOGIC ---
        # Non-LPL is aggressively fast to show visible danger. LPL is cautious.
        cmd.linear.x = 0.4 if self.lpl_active else 0.9 
        cmd.angular.z = 1.8 * yaw_error
        
        self.send(cmd)

    def send(self, twist):
        # Same message type Nav2 publishes on Jazzy
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = twist
        self.pub_cmd.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = DemoPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()