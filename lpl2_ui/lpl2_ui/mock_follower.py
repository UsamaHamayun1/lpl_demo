#!/usr/bin/env python3
# MOCK executor for dashboard development only. NOT the LPL execution layer.
# Drives Robot R along the selected option's path from /lpl2/decision_space with a
# simple pure-pursuit on ground truth, holds for option H, stops on /lpl2/estop.
import math

from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from lpl2_interfaces.msg import DecisionSpace
from lpl2_sim.runner import run
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SPEED, LOOKAHEAD, GOAL_TOL = 0.26, 0.6, 0.2


class MockFollower(Node):
    def __init__(self):
        super().__init__('mock_follower')
        self.path, self.pose, self.estop = [], None, False
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(DecisionSpace, '/lpl2/decision_space', self.on_decision, LATCHED)
        self.create_subscription(TFMessage, TRUTH_TOPIC, self.on_truth, 10)
        self.create_subscription(Bool, '/lpl2/estop', lambda m: setattr(self, 'estop', m.data), LATCHED)
        self.create_timer(0.05, self.step)

    def on_decision(self, msg):
        sel = next((o for o in msg.options if o.id == msg.selected_id), None)
        self.path = [(p.x, p.y) for p in sel.path] if sel else []

    def on_truth(self, msg):
        self.pose = poses_by_name(msg).get('robot_r', self.pose)

    def target(self, x, y):
        # nearest point on the path, then walk LOOKAHEAD metres further along it
        best = (1e9, 0, 0.0)
        for i in range(len(self.path) - 1):
            (ax, ay), (bx, by) = self.path[i], self.path[i + 1]
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy or 1)))
            d = math.hypot(x - ax - t * dx, y - ay - t * dy)
            if d < best[0]:
                best = (d, i, t)
        _, i, t = best
        left = LOOKAHEAD
        px, py = self.path[i][0] + t * (self.path[i + 1][0] - self.path[i][0]), \
            self.path[i][1] + t * (self.path[i + 1][1] - self.path[i][1])
        while i < len(self.path) - 1:
            nx, ny = self.path[i + 1]
            seg = math.hypot(nx - px, ny - py)
            if seg >= left:
                return px + (nx - px) * left / seg, py + (ny - py) * left / seg
            left -= seg
            px, py, i = nx, ny, i + 1
        return self.path[-1]

    def step(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        if self.path and self.pose and not self.estop:
            x, y, yaw = self.pose
            gx, gy = self.path[-1]
            if math.hypot(gx - x, gy - y) > GOAL_TOL:
                tx, ty = self.target(x, y)
                err = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw), math.cos(math.atan2(ty - y, tx - x) - yaw))
                cmd.twist.angular.z = max(-1.0, min(1.0, 2.5 * err))
                cmd.twist.linear.x = SPEED * max(0.0, math.cos(err)) if abs(err) < 1.0 else 0.0
        self.pub.publish(cmd)


def main(args=None):
    run(MockFollower, args)


if __name__ == '__main__':
    main()
