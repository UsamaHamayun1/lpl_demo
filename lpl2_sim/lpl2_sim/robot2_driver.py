#!/usr/bin/env python3
# Drives Robot 2 (the oncoming traffic) along the route in corridor.yaml.
# Part of the scripted world: it is controlled by the scenario director through
# std_srvs/Trigger services and never reacts to Robot R.
#   /sim/robot2/release  PARKED  -> DRIVING
#   /sim/robot2/stop     DRIVING -> STOPPED   (e.g. a broken-down vehicle)
#   /sim/robot2/resume   STOPPED -> DRIVING
# Closed loop on ground truth: open-loop VelocityControl drifted into Lane 2.
# Robot 2 does not watch for traffic, but it does brake when something is right in front of
# it (on contact, like any vehicle), instead of shoving Robot R along the corridor. A brake
# is reported once on /sim/world_events as R2_BRAKED, so runs can count contacts.
import json
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name
from lpl2_sim.runner import run

NAME = 'robot2'
REACHED = 0.15        # m, waypoint switch distance
TURN_IN_PLACE = 0.5   # rad, heading error above which we rotate without driving
K_HEADING = 2.0
MAX_TURN = 1.0        # rad/s
BRAKE_AHEAD = 0.85    # m centre-to-centre: R2 half-length 0.45 + Robot R ~0.2 + small margin
BRAKE_SIDE = 0.55     # m lateral: half-widths 0.3 + 0.16 + margin


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Robot2Driver(Node):
    def __init__(self):
        super().__init__('robot2_driver')
        path = self.declare_parameter('geometry_file', '').value or default_path()
        cfg = Corridor(path).robot2
        self.route = [tuple(p) for p in cfg['route']]
        self.speed = self.declare_parameter('speed', float(cfg['speed'])).value

        self.state = 'PARKED'
        self.wp = 0
        self.pose = None
        self.other = None          # Robot R (only used to brake on contact)
        self.braked = False

        self.pub_cmd = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/sim/robot2/state', 10)
        self.pub_events = self.create_publisher(String, '/sim/world_events', QoSProfile(
            depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, TRUTH_TOPIC, self.on_truth, 10)
        self.create_service(Trigger, '/sim/robot2/release', self.make_transition('PARKED', 'DRIVING'))
        self.create_service(Trigger, '/sim/robot2/stop', self.make_transition('DRIVING', 'STOPPED'))
        self.create_service(Trigger, '/sim/robot2/resume', self.make_transition('STOPPED', 'DRIVING'))
        self.create_timer(0.05, self.step)
        self.create_timer(0.5, self.publish_state)

    def make_transition(self, src, dst):
        def handler(_req, res):
            if self.state != src:
                res.success, res.message = False, f'robot2 is {self.state}, expected {src}'
            else:
                self.set_state(dst)
                res.success, res.message = True, dst
            return res
        return handler

    def set_state(self, state):
        self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.publish_state()

    def publish_state(self):
        self.pub_state.publish(String(data=self.state))

    def on_truth(self, msg):
        poses = poses_by_name(msg)
        self.pose = poses.get(NAME, self.pose)
        self.other = poses.get('robot_r', self.other)

    def blocked_ahead(self):
        if self.other is None:
            return False
        x, y, yaw = self.pose
        dx, dy = self.other[0] - x, self.other[1] - y
        ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
        side = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return 0.0 < ahead < BRAKE_AHEAD and abs(side) < BRAKE_SIDE

    def step(self):
        cmd = Twist()
        if self.state == 'DRIVING' and self.pose is not None:
            if self.blocked_ahead():
                if not self.braked:
                    self.braked = True
                    t = self.get_clock().now().nanoseconds * 1e-9
                    self.get_logger().warn('braking: Robot R right in front')
                    self.pub_events.publish(String(data=json.dumps({
                        'id': 'R2_BRAKED', 'note': 'Robot 2 braked: Robot R in its path (contact)',
                        'sim_time': round(t, 3), 'robot_x': round(self.other[0], 3), 'do': []})))
                self.pub_cmd.publish(cmd)
                return
            self.braked = False
            x, y, yaw = self.pose
            tx, ty = self.route[self.wp]
            if math.hypot(tx - x, ty - y) < REACHED:
                self.wp += 1
                if self.wp == len(self.route):
                    self.set_state('DONE')
                    self.pub_cmd.publish(cmd)
                    return
                tx, ty = self.route[self.wp]
            err = wrap(math.atan2(ty - y, tx - x) - yaw)
            cmd.angular.z = max(-MAX_TURN, min(MAX_TURN, K_HEADING * err))
            if abs(err) < TURN_IN_PLACE:
                cmd.linear.x = self.speed * math.cos(err)
        self.pub_cmd.publish(cmd)


def main(args=None):
    run(Robot2Driver, args)


if __name__ == '__main__':
    main()
