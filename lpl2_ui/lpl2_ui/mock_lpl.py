#!/usr/bin/env python3
# MOCK LPL for dashboard development only. NOT the LPL mechanism.
#
# Publishes plausible evidence / authority / decision / escalation messages so the
# dashboard can be built and checked before the real nodes exist. Its "decisions" are
# a fixed script keyed to the world events (W1 = view of Lane 1 lost, W2 = bypass
# blocked); nothing here is computed. Every AuthorityState says mode = "MOCK", and the
# dashboard shows a MOCK DATA badge.
#
#   ros2 run lpl2_ui mock_lpl
# Implements /lpl2/grant_permission so SPACE / BACKSPACE can be tried.
import json
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from lpl2_interfaces.msg import (AuthorityState, DecisionSpace, EvidenceArray, EvidenceItem, Escalation,
                                 ExecutionState, OptionStatus, PermissionState)
from lpl2_interfaces.srv import GrantPermission
from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.runner import run
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
SPEED = 0.26
PATH_A = [(0, -0.6), (10.5, -0.6), (12.0, 0.6), (14.8, 0.6), (16.5, -0.6), (24.0, -0.6)]
PATH_B = [(0, -0.6), (9.0, -0.6), (9.0, -3.4), (18.0, -3.4), (18.0, -0.6), (24.0, -0.6)]
COMMIT_X = 8.8           # after this R is in the bypass door: option A no longer reachable


def remaining(path, x, y):
    """Distance left along a polyline from the point nearest to (x, y)."""
    best, best_i, best_pt = 1e9, 0, path[0]
    for i in range(len(path) - 1):
        (ax, ay), (bx, by) = path[i], path[i + 1]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy or 1)))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        if d < best:
            best, best_i, best_pt = d, i, (px, py)
    rest = math.hypot(path[best_i + 1][0] - best_pt[0], path[best_i + 1][1] - best_pt[1])
    rest += sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                for i in range(best_i + 1, len(path) - 1))
    return rest


class MockLpl(Node):
    def __init__(self):
        super().__init__('mock_lpl')
        self.corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.w1 = self.w2 = False
        self.grant = None            # active grant id for P3
        self.revision = 1
        self.cycle = 0
        self.seq = 0
        self.robot = (0.0, -0.6)
        self.last_key = None

        self.pub_ev = self.create_publisher(EvidenceArray, '/lpl2/evidence', LATCHED)
        self.pub_auth = self.create_publisher(AuthorityState, '/lpl2/authority', LATCHED)
        self.pub_dec = self.create_publisher(DecisionSpace, '/lpl2/decision_space', LATCHED)
        self.pub_esc = self.create_publisher(Escalation, '/lpl2/escalation', LATCHED)
        self.pub_exe = self.create_publisher(ExecutionState, '/lpl2/execution_state', 10)
        self.create_subscription(String, '/sim/world_events', self.on_event, EVENTS_QOS)
        self.create_subscription(TFMessage, TRUTH_TOPIC, self.on_truth, 10)
        self.create_service(GrantPermission, '/lpl2/grant_permission', self.on_grant)
        self.create_timer(0.2, self.step)
        self.get_logger().warn('MOCK LPL running: scripted messages for dashboard development only')

    def on_event(self, msg):
        ev = json.loads(msg.data)
        self.w1 |= ev['id'] == 'W1'
        self.w2 |= ev['id'] == 'W2'

    def on_truth(self, msg):
        r = poses_by_name(msg).get('robot_r')
        if r:
            self.robot = r[:2]

    def on_grant(self, req, res):
        if req.permission_id != 'P3':
            res.accepted, res.reason = False, 'NOT_GRANTABLE'
        elif req.revoke:
            res.accepted, res.reason = self.grant is not None, 'REVOKED' if self.grant else 'NO_ACTIVE_GRANT'
            self.grant = None
        elif not self.w1:
            res.accepted, res.reason = False, 'NOT_NEEDED'
        else:
            self.grant = f'G{self.revision}'
            res.accepted, res.reason, res.grant_id = True, 'GRANTED', self.grant
        return res

    def now(self):
        return self.get_clock().now().to_msg()

    def step(self):
        x, y = self.robot
        committed_b = x > COMMIT_X and y < -1.2 or y < -2.0
        p3_by_evidence = not self.w1
        p3_usable = p3_by_evidence or self.grant is not None
        b_possible = not self.w2
        key = (self.w1, self.w2, self.grant, committed_b)
        if key != self.last_key:
            self.revision += 1
            self.last_key = key

        stamp = self.now()
        # evidence
        ev = EvidenceArray()
        ev.header.stamp = stamp
        self.seq += 1
        ev.seq = self.seq
        ev.items = [
            self.item('E1_own_lane_ok', 'Own lane and localization OK', True, 'OK', 'Lane 2 clear within stopping distance'),
            self.item('E2_bypass_access_ok', 'Service bypass open and unoccupied', True, 'OPEN', 'bypass door status: open'),
            self.item('E3_lane1_sector_clear', 'Lane 1 verified clear through the passing sector', not self.w1,
                      'CLEAR' if not self.w1 else 'OCCLUDED',
                      'Lane 1 visible to x=28.4' if not self.w1 else 'tall stack hides Lane 1 beyond x≈24.8 (need 28.4)'),
        ]
        self.pub_ev.publish(ev)

        # authority
        auth = AuthorityState()
        auth.header.stamp = stamp
        auth.revision, auth.mode, auth.rules_id = self.revision, 'MOCK', 'corridor-frozen-v1'
        auth.cause = 'E3 INSUFFICIENT (OCCLUDED)' if self.w1 else 'initial'
        p3 = self.perm('P3', 'Borrow oncoming lane (passing window)', 'lane1_window', ['E3_lane1_sector_clear'],
                       p3_by_evidence, True)
        if self.grant and not p3_by_evidence:
            p3.state, p3.provenance, p3.grant_id = PermissionState.HUMAN_GRANTED, PermissionState.PROV_HUMAN_GRANT, self.grant
            p3.reason = 'authorized by operator (single pass); E3 still insufficient'
        elif not p3_by_evidence:
            p3.reason = 'E3 INSUFFICIENT: Lane 1 cannot be verified (occluded)'
        auth.permissions = [
            self.perm('P1', 'Proceed in own lane', 'lane2', ['E1_own_lane_ok'], True, False),
            self.perm('P2', 'Use service bypass', 'bypass', ['E2_bypass_access_ok'], True, True),
            p3,
        ]
        self.pub_auth.publish(auth)

        # decision space
        a_reach = not committed_b
        options = [
            self.option('A', 'Pass through Lane 1', PATH_A, ['P1', 'P3'], [] if p3_usable else ['P3'],
                        a_reach, True, '', p3_by_evidence, p3_usable),
            self.option('B', 'Service bypass', PATH_B, ['P1', 'P2'], [], True, b_possible,
                        '' if b_possible else 'bypass door blocked (lidar)', True, True),
            self.option('H', 'Hold in own lane', [], [], [], True, True, '', True, True),
        ]
        a, b, h = options
        a.adequate = a.reachable and a.possible and a.authorized
        b.adequate = b.possible and b.authorized
        h.adequate, h.inadequate_reason = False, 'no progress'
        if not a.authorized and a.reachable:
            a.inadequate_reason = 'not authorized'
        adequate = [o for o in (a, b) if o.adequate]
        sel = min(adequate, key=lambda o: o.eta_s) if adequate else h
        sel.selected = True
        dec = DecisionSpace()
        dec.header.stamp = stamp
        self.cycle += 1
        dec.cycle, dec.authority_revision, dec.mission_id = self.cycle, self.revision, 'M1'
        dec.goal = Point(x=24.0, y=-0.6)
        dec.options, dec.selected_id = options, sel.id
        dec.selection_reason = ('fastest adequate option' if adequate
                                else 'no adequate option: holding safely')
        self.pub_dec.publish(dec)

        # escalation
        esc = Escalation()
        esc.header.stamp = stamp
        if not adequate:
            esc.level, esc.kind = Escalation.HUMAN_JUDGMENT_REQUIRED, Escalation.KIND_AUTHORITY_EXHAUSTED
            esc.headline = 'HUMAN JUDGMENT REQUIRED'
            esc.option_lines = ['A: possible · evidence ✗ (E3 occluded, scripted) · not authorized (P3)',
                                'B: not possible (bypass door blocked)', 'H: authorized · makes no progress']
            esc.asks, esc.advised_option = ['P3'], 'A'
        elif sel.id == 'B' and a.reachable and not a.authorized:
            esc.level, esc.kind = Escalation.ADVISORY, Escalation.KIND_BETTER_OPTION_NEEDS_PERMISSION
            esc.headline = 'Better option available'
            esc.option_lines = [f'A: {a.eta_s:.0f} s but needs P3 (Lane 1 not verifiable)',
                                f'B: {b.eta_s:.0f} s, fully authorized (selected)']
            esc.asks, esc.advised_option, esc.eta_gain_s = ['P3'], 'A', max(0.0, b.eta_s - a.eta_s)
        self.pub_esc.publish(esc)

        exe = ExecutionState()
        exe.header.stamp = stamp
        exe.option_id = sel.id
        exe.state = ExecutionState.HOLDING if sel.id == 'H' else ExecutionState.EXECUTING
        exe.remaining_m = sel.remaining_m
        self.pub_exe.publish(exe)

    def item(self, id_, prop, ok, reason, detail):
        it = EvidenceItem()
        it.id, it.proposition, it.reason, it.detail = id_, prop, reason, detail
        it.level = EvidenceItem.ADEQUATE if ok else EvidenceItem.INSUFFICIENT
        it.confidence, it.ttl_s, it.source, it.scripted = 1.0, 1.0, 'scripted', True
        it.observed_at = self.now()
        return it

    def perm(self, id_, name, region, evidence, ok, grantable):
        p = PermissionState()
        p.id, p.name, p.region_id, p.evidence_ids = id_, name, region, evidence
        p.state = PermissionState.AVAILABLE if ok else PermissionState.UNAVAILABLE
        p.evidence_supported, p.provenance, p.human_grantable = ok, PermissionState.PROV_EVIDENCE, grantable
        p.reason = 'evidence adequate' if ok else ''
        p.since = Time()
        return p

    def option(self, id_, name, path, requires, missing, reachable, possible, why, evidenced, authorized):
        o = OptionStatus()
        o.id, o.name, o.requires, o.missing = id_, name, requires, missing
        o.kind = OptionStatus.HOLD if not path else OptionStatus.ROUTE
        o.reachable, o.possible, o.possible_reason = reachable, possible, why
        o.evidenced, o.authorized = evidenced, authorized
        o.path = [Point(x=float(px), y=float(py)) for px, py in path]
        o.remaining_m = remaining(path, *self.robot) if path else 0.0
        o.eta_s = o.remaining_m / SPEED if path else -1.0
        return o


def main(args=None):
    run(MockLpl, args)


if __name__ == '__main__':
    main()
