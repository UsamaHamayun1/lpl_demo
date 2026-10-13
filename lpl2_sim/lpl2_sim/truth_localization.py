#!/usr/bin/env python3
# Simulated localization: publishes map -> odom so that the robot's believed pose
# (map -> base_footprint) equals its true pose in Gazebo.
#
# Stand-in for a real localizer (AMCL, SLAM). Without it the robot runs on wheel
# odometry alone: slip makes the heading drift, the lidar then paints walls in the
# wrong place, and planning and feasibility checks fail on phantom obstacles.
#   map -> odom (this node) -> base_footprint (Gazebo DiffDrive) -> base_link (URDF)
import math

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time as RclTime
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from lpl2_sim.runner import run
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name


class TruthLocalization(Node):
    def __init__(self):
        super().__init__('truth_localization')
        self.robot = self.declare_parameter('robot', 'robot_r').value
        rate = self.declare_parameter('rate', 30.0).value
        self.truth = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, TRUTH_TOPIC, self.on_truth, 10)
        self.create_timer(1.0 / rate, self.publish)

    def on_truth(self, msg):
        self.truth = poses_by_name(msg).get(self.robot, self.truth)

    def publish(self):
        if self.truth is None:
            return
        try:
            o = self.tf.lookup_transform('odom', 'base_footprint', RclTime())
        except TransformException:
            return
        # odom -> base (x_o, y_o, th_o); map -> base (x_m, y_m, th_m) = truth
        q = o.transform.rotation
        th_o = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x_o, y_o = o.transform.translation.x, o.transform.translation.y
        x_m, y_m, th_m = self.truth
        # map -> odom = (map -> base) * (odom -> base)^-1
        th = th_m - th_o
        c, s = math.cos(th), math.sin(th)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x = x_m - (c * x_o - s * y_o)
        t.transform.translation.y = y_m - (s * x_o + c * y_o)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.broadcaster.sendTransform(t)


def main(args=None):
    run(TruthLocalization, args)


if __name__ == '__main__':
    main()
