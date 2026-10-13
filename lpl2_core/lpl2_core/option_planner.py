#!/usr/bin/env python3
# Option planner: available actions + selection + escalation (downstream of LPL).
#
# Every cycle it judges each predefined option (config/options.yaml) as reachable /
# possible / evidenced / authorized / adequate, selects the fastest adequate one, and
# decides whether a human is needed (advisory, HUMAN JUDGMENT REQUIRED) or something is
# broken (SYSTEM FAULT). Logic lives in decision.py; this node only wires ROS to it.
#
#   in:  /lpl2/authority (permissions only), robot pose (TF map -> base_footprint),
#        /feasibility/is_path_valid (robot's physical-only map), /lpl2/execution_state
#   out: /lpl2/decision_space, /lpl2/escalation
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.srv import IsPathValid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclTime
from tf2_ros import Buffer, TransformException, TransformListener

from lpl2_core.decision import Decider, PermView
from lpl2_core.options import load_options
from lpl2_core.regions import Regions
from lpl2_interfaces.msg import (AuthorityState, DecisionSpace, Escalation, ExecutionState, OptionStatus,
                                 PermissionState)
from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.runner import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STATE = {PermissionState.AVAILABLE: 'AVAILABLE', PermissionState.UNAVAILABLE: 'UNAVAILABLE',
         PermissionState.HUMAN_GRANTED: 'HUMAN_GRANTED', PermissionState.EGRESS_ONLY: 'EGRESS_ONLY'}
LEVEL = {'NONE': Escalation.NONE, 'ADVISORY': Escalation.ADVISORY,
         'HUMAN_JUDGMENT_REQUIRED': Escalation.HUMAN_JUDGMENT_REQUIRED, 'SYSTEM_FAULT': Escalation.SYSTEM_FAULT}
KIND = {'NONE': Escalation.KIND_NONE, 'BETTER_OPTION_NEEDS_PERMISSION': Escalation.KIND_BETTER_OPTION_NEEDS_PERMISSION,
        'AUTHORITY_EXHAUSTED': Escalation.KIND_AUTHORITY_EXHAUSTED, 'NO_FEASIBLE': Escalation.KIND_NO_FEASIBLE,
        'LPL_STALE': Escalation.KIND_LPL_STALE, 'EXECUTION_FAILED': Escalation.KIND_EXECUTION_FAILED}


def share(*p):
    return os.path.join(get_package_share_directory('lpl2_core'), 'config', *p)


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


class OptionPlanner(Node):
    def __init__(self):
        super().__init__('option_planner')
        self.mission = load(self.declare_parameter('mission_file', share('mission.yaml')).value)
        options_cfg = load(self.declare_parameter('options_file', share('options.yaml')).value)
        options = load_options(options_cfg)
        rules = load(self.declare_parameter('rules_file', share('rules.yaml')).value)
        corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.decider = Decider(options, Regions(corridor, rules), self.mission, options_cfg.get('safe_hold'))
        self.options = {o.id: o for o in options}

        self.authority = None
        self.authority_rx = None
        self.exec_state = None
        self.started = False       # Nav2 is up and executing: its costmaps can be trusted
        self.cycle = 0
        self.last_esc = None
        self.pending = set()

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.feasibility = self.create_client(IsPathValid, '/feasibility/is_path_valid')
        self.pub_dec = self.create_publisher(DecisionSpace, '/lpl2/decision_space', LATCHED)
        self.pub_esc = self.create_publisher(Escalation, '/lpl2/escalation', LATCHED)
        self.create_subscription(AuthorityState, '/lpl2/authority', self.on_authority, LATCHED)
        self.create_subscription(ExecutionState, '/lpl2/execution_state', self.on_execution, 10)
        self.create_timer(0.5, self.cycle_once)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_execution(self, msg):
        self.exec_state = msg
        self.started |= msg.state == ExecutionState.EXECUTING

    def on_authority(self, msg):
        changed = self.authority is None or msg.revision != self.authority.revision
        self.authority, self.authority_rx = msg, self.now()
        if changed:
            self.cycle_once()

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', RclTime())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    # ---------------------------------------------------------------- feasibility
    def check_feasibility(self, statuses):
        # before Nav2 runs, the feasibility costmap may not have the map yet (everything looks blocked)
        if not self.started or not self.feasibility.service_is_ready():
            return
        for s in statuses:
            if s.hold or not s.reachable or s.id in self.pending or len(s.remaining_path) < 2:
                continue
            if s.id not in self.decider.feas:
                continue
            req = IsPathValid.Request()
            req.path.header.frame_id = 'map'
            pts = s.remaining_path[::2]
            for i, (x, y) in enumerate(pts):
                nx, ny = pts[min(i + 1, len(pts) - 1)]
                px, py = pts[max(i - 1, 0)]
                yaw = math.atan2(ny - py, nx - px)      # footprint oriented along the path
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                req.path.poses.append(p)
            self.pending.add(s.id)
            self.feasibility.call_async(req).add_done_callback(lambda f, oid=s.id: self.on_feasibility(oid, f))

    def on_feasibility(self, oid, future):
        self.pending.discard(oid)
        res = future.result()
        if res is not None:
            self.decider.report_feasibility(oid, res.is_valid)

    # ---------------------------------------------------------------- cycle
    def cycle_once(self):
        pose = self.pose()
        if self.authority is None or pose is None:
            return          # nothing to decide until LPL and localization are up
        fresh = self.now() - self.authority_rx <= self.mission['lease_s']
        perms = {p.id: PermView(p.id, p.region_id, STATE[p.state], p.evidence_supported,
                                p.provenance == PermissionState.PROV_EVIDENCE, p.human_grantable)
                 for p in self.authority.permissions}
        gx, gy = self.mission['goal']
        # the mission is complete only when the robot is at the mission goal (Nav2 finishing some
        # sub-goal, e.g. an egress back to its own lane, is not mission success)
        done = ((pose[0] - gx) ** 2 + (pose[1] - gy) ** 2) ** 0.5 <= self.mission['goal_tolerance_m']
        statuses, sel, reason, esc = self.decider.evaluate(self.now(), pose, perms, fresh, done)
        ex = self.exec_state
        if ex is not None and ex.state == ExecutionState.FAILED and ex.option_id == sel and 'giving up' in ex.detail:
            # a possible, authorized option that execution cannot carry out is a fault, not a judgment call
            esc.level, esc.kind = 'SYSTEM_FAULT', 'EXECUTION_FAILED'
            esc.headline = f'option {sel} keeps failing in execution ({ex.detail})'
            esc.asks, esc.advised = [], ''
        self.check_feasibility(statuses)
        self.publish(statuses, sel, reason, esc)

    def publish(self, statuses, sel, reason, esc):
        stamp = self.get_clock().now().to_msg()
        self.cycle += 1
        d = DecisionSpace()
        d.header.stamp, d.header.frame_id = stamp, 'map'
        d.cycle, d.authority_revision, d.mission_id = self.cycle, self.authority.revision, self.mission['mission_id']
        d.goal = Point(x=float(self.mission['goal'][0]), y=float(self.mission['goal'][1]))
        d.selected_id, d.selection_reason = sel, reason
        for s in statuses:
            o = OptionStatus()
            o.id, o.name = s.id, s.name
            o.kind = OptionStatus.HOLD if s.hold else OptionStatus.EGRESS if s.egress else OptionStatus.ROUTE
            o.requires, o.missing = s.requires, s.missing
            o.reachable, o.possible, o.possible_reason = s.reachable, s.possible, s.possible_reason
            o.evidenced, o.authorized, o.adequate = s.evidenced, s.authorized, s.adequate
            o.inadequate_reason, o.selected = s.inadequate_reason, s.selected
            o.eta_s, o.remaining_m = float(s.eta_s), float(s.remaining_m)
            o.path = [Point(x=float(x), y=float(y)) for x, y in s.remaining_path[::3]]
            d.options.append(o)
        self.pub_dec.publish(d)

        key = (esc.level, esc.kind, tuple(esc.asks), esc.advised)
        if key != self.last_esc:
            self.get_logger().info(f'escalation {esc.level} {esc.kind} asks={esc.asks} ({reason})')
            self.last_esc = key
        e = Escalation()
        e.header.stamp = stamp
        e.level, e.kind = LEVEL[esc.level], KIND.get(esc.kind, Escalation.KIND_NONE)
        e.headline, e.option_lines, e.asks = esc.headline, esc.lines, esc.asks
        e.advised_option, e.eta_gain_s = esc.advised, float(esc.gain_s)
        self.pub_esc.publish(e)


def main(args=None):
    run(OptionPlanner, args)


if __name__ == '__main__':
    main()
