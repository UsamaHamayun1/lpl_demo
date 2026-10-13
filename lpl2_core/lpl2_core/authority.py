#!/usr/bin/env python3
# LPL authority node: evidence -> permissions (the authority envelope).
#
# Wraps the pure engine in policy.py. Publishes the full AuthorityState on
# /lpl2/authority on every change and as a heartbeat, and serves /lpl2/grant_permission
# for the operator. It never plans and never commands the robot.
#
# Region occupancy (for single-pass grants and egress) comes from the robot's own
# localization (TF map -> base_footprint), not from simulator ground truth.
#
# mode: lpl (normal) | permissive (no LPL: everything allowed) | global_revoke
#       (conventional "any uncertainty stops everything"). The last two exist only
#       for the comparison runs.
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclTime
from tf2_ros import Buffer, TransformException, TransformListener

from lpl2_core.policy import AuthorityCore, Evidence
from lpl2_core.regions import Regions
from lpl2_interfaces.msg import AuthorityState, EvidenceArray, EvidenceItem, PermissionState
from lpl2_interfaces.srv import GrantPermission
from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.runner import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LEVEL = {EvidenceItem.ADEQUATE: 'ADEQUATE', EvidenceItem.INSUFFICIENT: 'INSUFFICIENT',
         EvidenceItem.UNKNOWN: 'UNKNOWN', EvidenceItem.CONTRADICTED: 'CONTRADICTED'}
STATE = {'AVAILABLE': PermissionState.AVAILABLE, 'UNAVAILABLE': PermissionState.UNAVAILABLE,
         'HUMAN_GRANTED': PermissionState.HUMAN_GRANTED, 'EGRESS_ONLY': PermissionState.EGRESS_ONLY}
PROV = {'EVIDENCE': PermissionState.PROV_EVIDENCE, 'HUMAN_GRANT': PermissionState.PROV_HUMAN_GRANT,
        'DWELL_PENDING': PermissionState.PROV_DWELL_PENDING, 'STALE': PermissionState.PROV_STALE,
        'FIXED': PermissionState.PROV_FIXED}


def to_time(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9)) if t > 0 else Time()


def secs(t):
    return t.sec + t.nanosec * 1e-9


class Authority(Node):
    def __init__(self):
        super().__init__('lpl_authority')
        default_rules = os.path.join(get_package_share_directory('lpl2_core'), 'config', 'rules.yaml')
        with open(self.declare_parameter('rules_file', default_rules).value) as f:
            self.rules = yaml.safe_load(f)
        self.mode = self.declare_parameter('mode', 'lpl').value
        self.robot_radius = self.declare_parameter('robot_radius', 0.2).value
        # the robot counts as out of a region only when this much clear of it, so a region
        # closing behind it (single-pass grant used up, egress done) never closes on top of
        # it: Nav2 inflates keepout areas by ~0.45 m
        self.exit_clearance = self.declare_parameter('exit_clearance', 0.3).value
        corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.regions = Regions(corridor, self.rules)
        self.core = AuthorityCore(self.rules, self.mode)
        self.evidence = {}
        self.last_stamp = -1.0
        self.lease = float(self.rules.get('lease_s', 1.0))
        self.cause = 'startup'

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(AuthorityState, '/lpl2/authority', LATCHED)
        self.create_subscription(EvidenceArray, '/lpl2/evidence', self.on_evidence, LATCHED)
        self.create_service(GrantPermission, '/lpl2/grant_permission', self.on_grant)
        self.create_timer(1.0 / float(self.rules.get('heartbeat_hz', 10.0)), self.step)
        self.get_logger().info(f"LPL authority up: rules {self.rules['rules_id']}, mode {self.mode}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_evidence(self, msg):
        # order by stamp, not seq: a restarted evidence source starts its seq again at 1
        stamp = secs(msg.header.stamp)
        if stamp < self.last_stamp:
            return          # out of order or replayed
        self.last_stamp = stamp
        for it in msg.items:
            self.evidence[it.id] = Evidence(LEVEL.get(it.level, 'UNKNOWN'), secs(it.observed_at), it.ttl_s, it.reason)
        self.step()

    def occupancy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', RclTime())
        except TransformException:
            return {}
        x, y = t.transform.translation.x, t.transform.translation.y
        r = self.robot_radius + self.exit_clearance
        return {name: self.regions.inside(name, x, y, r) for name in self.regions.regions}

    def step(self):
        res = self.core.update(self.now(), self.evidence, self.occupancy())
        for c in res.causes:
            self.get_logger().info(f'rev {self.core.revision}: {c}')
        if res.causes:
            self.cause = '; '.join(res.causes)
        self.publish(res.permissions)

    def publish(self, perms):
        msg = AuthorityState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.revision, msg.mode, msg.rules_id, msg.cause = self.core.revision, self.mode, self.rules['rules_id'], self.cause
        msg.lease_s = self.lease
        for p in perms:
            m = PermissionState()
            m.id, m.name, m.region_id = p.id, p.name, p.region
            m.state, m.provenance = STATE[p.state], PROV[p.provenance]
            m.evidence_supported, m.evidence_ids, m.reason = p.evidence_supported, p.evidence_ids, p.reason
            m.since, m.human_grantable = to_time(p.since), p.human_grantable
            m.grant_id, m.grant_expires_at = p.grant_id, to_time(p.grant_expires_at)
            msg.permissions.append(m)
        self.pub.publish(msg)

    def on_grant(self, req, res):
        now = self.now()
        if req.revoke:
            res.accepted, res.reason = self.core.revoke_grant(now, req.permission_id)
        else:
            res.accepted, res.reason, res.grant_id, exp = self.core.request_grant(
                now, req.permission_id, req.for_option, req.ttl_s, req.authority_revision_seen)
            res.expires_at = to_time(exp)
        self.get_logger().info(f"operator {'revoke' if req.revoke else 'grant'} {req.permission_id} "
                               f'for {req.for_option or "-"}: {res.reason}')
        self.step()      # publish the new envelope right away
        return res


def main(args=None):
    run(Authority, args)


if __name__ == '__main__':
    main()
