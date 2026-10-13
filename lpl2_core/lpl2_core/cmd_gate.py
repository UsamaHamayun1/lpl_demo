#!/usr/bin/env python3
# Execution gate: the ONLY writer of /cmd_vel. Last line of enforcement before the wheels.
#
# Autonomy commands (/lpl2/cmd_vel_auto, from Nav2's collision monitor) pass only if:
#   - e-stop is off                                   (else ESTOP)
#   - the authority envelope is fresh                 (else BLOCK_LEASE: nothing is authorized)
#   - the command stream is alive                     (else WATCHDOG)
#   - the robot's footprint, predicted ~1 s ahead, does not enter a region whose permission
#     is not usable or a standing keepout (else BLOCK_PERMISSION). Moving OUT of such a
#     region (egress) and turning on the spot are always allowed.
# Operator teleop (/lpl2/teleop) passes only while the operator has armed a manual takeover
# (/lpl2/manual): the human's own decision, reported as MANUAL.
# The gate is independent of the planner: even a rogue command cannot drive into a
# region LPL has closed.
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclTime
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from lpl2_core.regions import Regions
from lpl2_interfaces.msg import AuthorityState, ExecutionState, GateStatus, PermissionState
from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.runner import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
USABLE = (PermissionState.AVAILABLE, PermissionState.HUMAN_GRANTED, PermissionState.EGRESS_ONLY)
LOOKAHEAD = (0.25, 0.5, 1.0)      # s
WATCHDOG_S = 0.5
TELEOP_TIMEOUT_S = 0.5


def depth_into(rect, x, y, r):
    """> 0 if a circle of radius r at (x, y) overlaps rect: how far it reaches in (m)."""
    x0, x1, y0, y1 = rect
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    outside = math.hypot(dx, dy)
    if outside > 0:
        return r - outside
    return r + min(x - x0, x1 - x, y - y0, y1 - y)   # centre inside: distance to the nearest edge


class CmdGate(Node):
    def __init__(self):
        super().__init__('cmd_gate')
        default_rules = os.path.join(get_package_share_directory('lpl2_core'), 'config', 'rules.yaml')
        with open(self.declare_parameter('rules_file', default_rules).value) as f:
            rules = yaml.safe_load(f)
        corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.regions = Regions(corridor, rules)
        self.radius = self.declare_parameter('robot_radius', 0.18).value
        self.lease = self.declare_parameter('lease_s', 1.0).value

        self.auto = None
        self.auto_at = None
        self.teleop = Twist()
        self.teleop_at = None
        self.estop = False
        self.manual = False
        self.authority = None
        self.authority_at = None
        self.exec_state = None
        self.blocks = 0
        self.status = (GateStatus.PASS, 'starting')
        self.out = (0.0, 0.0)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(GateStatus, '/lpl2/gate_status', 10)
        self.create_subscription(TwistStamped, '/lpl2/cmd_vel_auto', self.on_auto, 10)
        self.create_subscription(Twist, '/lpl2/teleop', self.on_teleop, 10)
        self.create_subscription(Bool, '/lpl2/estop', lambda m: setattr(self, 'estop', m.data), LATCHED)
        self.create_subscription(Bool, '/lpl2/manual', lambda m: setattr(self, 'manual', m.data), LATCHED)
        self.create_subscription(AuthorityState, '/lpl2/authority', self.on_authority, LATCHED)
        self.create_subscription(ExecutionState, '/lpl2/execution_state',
                                 lambda m: setattr(self, 'exec_state', m), 10)
        self.create_timer(0.02, self.step)
        self.create_timer(0.1, self.publish_status)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_auto(self, msg):
        self.auto, self.auto_at = msg.twist, self.now()

    def on_teleop(self, msg):
        self.teleop, self.teleop_at = msg, self.now()

    def on_authority(self, msg):
        self.authority, self.authority_at = msg, self.now()

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', RclTime())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def closed_rects(self):
        usable = {p.region_id for p in self.authority.permissions if p.state in USABLE}
        return [(f'no-passing area', r) for r in self.regions.standing] + \
               [(name, r) for name, r in self.regions.regions.items() if name not in usable]

    def geofence(self, v, w):
        """-> None if allowed, else the name of the region the command would enter."""
        pose = self.pose()
        if pose is None or abs(v) < 1e-3:
            return None               # no localization: other checks still apply; rotation never moves the footprint
        x, y, th = pose
        for name, rect in self.closed_rects():
            now_depth = depth_into(rect, x, y, self.radius)
            for t in LOOKAHEAD:
                if abs(w) > 1e-3:
                    px = x + v / w * (math.sin(th + w * t) - math.sin(th))
                    py = y - v / w * (math.cos(th + w * t) - math.cos(th))
                else:
                    px, py = x + v * t * math.cos(th), y + v * t * math.sin(th)
                d = depth_into(rect, px, py, self.radius)
                if d > 0 and d > now_depth + 1e-3:     # would go further in (not leaving)
                    return name
        return None

    def step(self):
        now = self.now()
        v = w = 0.0
        if self.estop:
            state, reason = GateStatus.ESTOP, 'operator e-stop'
        elif self.manual and self.teleop_at is not None and now - self.teleop_at < TELEOP_TIMEOUT_S:
            state, reason = GateStatus.MANUAL, 'operator teleop (manual takeover)'
            v, w = self.teleop.linear.x, self.teleop.angular.z
        elif self.manual:
            state, reason = GateStatus.MANUAL, 'manual takeover armed, no teleop input'
        elif self.authority_at is None or now - self.authority_at > self.lease:
            state, reason = GateStatus.BLOCK_LEASE, 'authority missing or stale: nothing is authorized'
        elif self.auto_at is None or now - self.auto_at > WATCHDOG_S:
            executing = self.exec_state is not None and self.exec_state.state in (ExecutionState.EXECUTING,
                                                                                ExecutionState.EGRESS)
            state, reason = (GateStatus.WATCHDOG, 'autonomy command stream silent') if executing else \
                (GateStatus.PASS, 'idle')
        else:
            v, w = self.auto.linear.x, self.auto.angular.z
            region = self.geofence(v, w)
            if region:
                state, reason = GateStatus.BLOCK_PERMISSION, f'command would enter {region} (not authorized)'
                v = w = 0.0
            else:
                state, reason = GateStatus.PASS, 'autonomy command passed'
        if state not in (GateStatus.PASS, GateStatus.MANUAL) and self.status[0] != state:
            self.blocks += 1
            self.get_logger().warn(f'gate: {reason}')
        self.status, self.out = (state, reason), (v, w)
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x, cmd.twist.angular.z = v, w
        self.pub.publish(cmd)

    def publish_status(self):
        m = GateStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state, m.reason = self.status
        m.blocks_total = self.blocks
        m.linear, m.angular = float(self.out[0]), float(self.out[1])
        self.pub_status.publish(m)


def main(args=None):
    run(CmdGate, args)


if __name__ == '__main__':
    main()
