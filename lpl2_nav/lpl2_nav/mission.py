#!/usr/bin/env python3
# Mission runner for the plain-autonomy runs (no LPL): send Robot R to its goal with
# NavigateToPose and report how it went. This is the "same intelligence" baseline;
# in the LPL runs the option planner takes over this job.
#
# Publishes /mission/status (std_msgs/String, JSON):
#   {"state": WAITING|ACTIVE|SUCCEEDED|ABORTED|CANCELED, "distance_remaining": m,
#    "error_code": int, "elapsed": s}
import json

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.runner import run

RESULT_STATE = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.goal_xy = corridor.robot_r['goal'][:2]
        self.status = {'state': 'WAITING', 'distance_remaining': None, 'error_code': 0, 'elapsed': 0.0}
        self.t_start = None
        self.pub = self.create_publisher(String, '/mission/status', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(1.0, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.status['state'] == 'WAITING' and self.client.server_is_ready():
            self.send_goal()
        if self.t_start is not None and self.status['state'] == 'ACTIVE':
            self.status['elapsed'] = round(self.now() - self.t_start, 1)
        self.pub.publish(String(data=json.dumps(self.status)))

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.goal_xy
        goal.pose.pose.orientation.w = 1.0
        self.status['state'] = 'ACTIVE'
        self.t_start = self.now()
        self.get_logger().info(f'Goal sent: {self.goal_xy}')
        self.client.send_goal_async(goal, feedback_callback=self.on_feedback).add_done_callback(self.on_accepted)

    def on_feedback(self, msg):
        self.status['distance_remaining'] = round(msg.feedback.distance_remaining, 2)

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.status['state'] = 'REJECTED'
            self.get_logger().error('Goal rejected')
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        res = future.result()
        self.status['state'] = RESULT_STATE.get(res.status, str(res.status))
        self.status['error_code'] = res.result.error_code
        self.status['elapsed'] = round(self.now() - self.t_start, 1)
        self.get_logger().info(f"Mission {self.status['state']} after {self.status['elapsed']} s "
                               f"(error_code {res.result.error_code})")


def main(args=None):
    run(Mission, args)


if __name__ == '__main__':
    main()
