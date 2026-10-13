# ROS side of the LPL 2.0 dashboard.
#
# A background thread spins the node. Callbacks only store the latest message and
# append change events to a log; the Qt window reads a snapshot on its own timer.
# Change detection happens here, on every message, so the log never misses a flip
# that happened between two screen refreshes.
import json
import threading
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from lpl2_interfaces.msg import (AuthorityState, DecisionSpace, EvidenceArray, EvidenceItem,
                                 Escalation, ExecutionState, GateStatus, PermissionState)
from lpl2_interfaces.srv import GrantPermission
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

EVIDENCE_LEVEL = {EvidenceItem.ADEQUATE: 'ADEQUATE', EvidenceItem.INSUFFICIENT: 'INSUFFICIENT',
                  EvidenceItem.UNKNOWN: 'UNKNOWN', EvidenceItem.CONTRADICTED: 'CONTRADICTED'}
PERMISSION_STATE = {PermissionState.AVAILABLE: 'AVAILABLE', PermissionState.UNAVAILABLE: 'UNAVAILABLE',
                    PermissionState.HUMAN_GRANTED: 'HUMAN_GRANTED', PermissionState.EGRESS_ONLY: 'EGRESS_ONLY'}
ESCALATION_LEVEL = {Escalation.NONE: 'NONE', Escalation.ADVISORY: 'ADVISORY',
                    Escalation.HUMAN_JUDGMENT_REQUIRED: 'HUMAN JUDGMENT REQUIRED',
                    Escalation.SYSTEM_FAULT: 'SYSTEM FAULT'}


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class RosBridge:
    """Owns the dashboard's ROS node; thread-safe snapshot for the UI."""

    def __init__(self):
        rclpy.init()
        self.node = Node('lpl2_dashboard', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.latest = {}
        self.log = deque(maxlen=300)        # (sim_time, kind, text)
        # counters come from what the system did (authority grants, gate states), not key presses
        self.counters = {'authorizations': 0, 'revocations': 0, 'takeovers': 0,
                         'judgment_requests': 0, 'estops': 0}
        self.prev = {'evidence': {}, 'permissions': {}, 'selected': None, 'escalation': None,
                     'execution': None, 'gate': None}
        self.grant_ids = set()
        self.estop_latched = None      # last /lpl2/estop value (survives a dashboard restart)
        self.record = self.node.declare_parameter('record', False).value
        self.scenario = self.node.declare_parameter('scenario', 'run').value

        n = self.node
        n.create_subscription(EvidenceArray, '/lpl2/evidence', self.on_evidence, LATCHED)
        n.create_subscription(AuthorityState, '/lpl2/authority', self.on_authority, LATCHED)
        n.create_subscription(DecisionSpace, '/lpl2/decision_space', self.on_decision, LATCHED)
        n.create_subscription(Escalation, '/lpl2/escalation', self.on_escalation, LATCHED)
        n.create_subscription(ExecutionState, '/lpl2/execution_state', self.on_execution, 10)
        n.create_subscription(GateStatus, '/lpl2/gate_status', self.on_gate, 10)
        n.create_subscription(Bool, '/lpl2/estop', lambda m: setattr(self, 'estop_latched', m.data), LATCHED)
        n.create_subscription(String, '/sim/world_events', self.on_world_event, EVENTS_QOS)
        n.create_subscription(TFMessage, TRUTH_TOPIC, lambda m: self.store('poses', poses_by_name(m)), 10)
        n.create_subscription(Image, '/camera/image_raw', lambda m: self.store('fpv', m), 2)
        n.create_subscription(Image, '/topdown/image', lambda m: self.store('topdown', m), 2)

        self.grant_client = n.create_client(GrantPermission, '/lpl2/grant_permission')
        self.pub_estop = n.create_publisher(Bool, '/lpl2/estop', LATCHED)
        self.pub_teleop = n.create_publisher(Twist, '/lpl2/teleop', 10)
        self.pub_manual = n.create_publisher(Bool, '/lpl2/manual', LATCHED)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- plumbing
    def _spin(self):
        try:
            self.executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RuntimeError:
            if rclpy.ok():
                raise

    def shutdown(self):
        # stop the spin thread before destroying the node it is spinning
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def store(self, key, value):
        with self.lock:
            self.latest[key] = value
            self.latest[key + '_at'] = self.now()

    def add_log(self, kind, text):
        with self.lock:
            self.log.append((self.now(), kind, text))

    def snapshot(self):
        with self.lock:
            return dict(self.latest), list(self.log), dict(self.counters), self.now()

    # ---------------------------------------------------------------- callbacks
    def on_evidence(self, msg):
        self.store('evidence', msg)
        for item in msg.items:
            level = EVIDENCE_LEVEL.get(item.level, '?')
            if self.prev['evidence'].get(item.id) != level:
                if item.id in self.prev['evidence']:
                    self.add_log('EVIDENCE', f'{item.id} -> {level} ({item.reason})')
                self.prev['evidence'][item.id] = level

    def on_authority(self, msg):
        self.store('authority', msg)
        for p in msg.permissions:
            if p.grant_id and p.grant_id not in self.grant_ids:
                self.grant_ids.add(p.grant_id)
                with self.lock:
                    self.counters['authorizations'] += 1
            state = PERMISSION_STATE.get(p.state, '?')
            if self.prev['permissions'].get(p.id) != state:
                if p.id in self.prev['permissions']:
                    self.add_log('AUTHORITY', f'{p.id} -> {state}: {p.reason}')
                self.prev['permissions'][p.id] = state

    def on_decision(self, msg):
        self.store('decision', msg)
        if msg.selected_id != self.prev['selected']:
            if self.prev['selected'] is not None:
                self.add_log('DECISION', f'option {msg.selected_id} selected: {msg.selection_reason}')
            self.prev['selected'] = msg.selected_id

    def on_escalation(self, msg):
        self.store('escalation', msg)
        key = (msg.level, msg.kind)
        if key != self.prev['escalation']:
            if msg.level == Escalation.HUMAN_JUDGMENT_REQUIRED:
                with self.lock:
                    self.counters['judgment_requests'] += 1
            if self.prev['escalation'] is not None or msg.level != Escalation.NONE:
                self.add_log('ESCALATION', ESCALATION_LEVEL.get(msg.level, '?')
                             + (f': {msg.headline}' if msg.headline else ''))
            self.prev['escalation'] = key

    def on_execution(self, msg):
        self.store('execution', msg)
        key = (msg.option_id, msg.state)
        if key != self.prev['execution']:
            self.prev['execution'] = key

    def on_gate(self, msg):
        self.store('gate', msg)
        if msg.state != self.prev['gate']:
            with self.lock:
                if msg.state == GateStatus.MANUAL:
                    self.counters['takeovers'] += 1
                elif msg.state == GateStatus.ESTOP:
                    self.counters['estops'] += 1
            if self.prev['gate'] is not None and msg.state not in (GateStatus.PASS,):
                self.add_log('GATE', msg.reason)
            self.prev['gate'] = msg.state

    def on_world_event(self, msg):
        ev = json.loads(msg.data)
        self.add_log('WORLD', f"{ev['id']}: {ev.get('note', '')}")

    # ---------------------------------------------------------------- operator actions
    def request_grant(self, permission_id, for_option, revision, revoke=False):
        if not self.grant_client.service_is_ready():
            self.add_log('OPERATOR', 'grant service not available (is LPL running?)')
            return
        req = GrantPermission.Request()
        req.permission_id = permission_id
        req.for_option = for_option
        req.ttl_s = 60.0
        req.operator_id = 'dashboard'
        req.authority_revision_seen = revision
        req.revoke = revoke
        verb = 'revoke' if revoke else 'authorize'
        self.add_log('OPERATOR', f'{verb} {permission_id} requested')

        def done(future):
            res = future.result()
            if res.accepted and revoke:
                with self.lock:
                    self.counters['revocations'] += 1
            self.add_log('OPERATOR', f"{verb} {permission_id}: {'accepted' if res.accepted else 'refused'} "
                                     f'({res.reason})')
        self.grant_client.call_async(req).add_done_callback(done)

    def set_estop(self, on):
        self.pub_estop.publish(Bool(data=on))
        self.add_log('OPERATOR', 'E-STOP ON' if on else 'E-STOP released')

    def takeover_started(self):
        self.pub_manual.publish(Bool(data=True))
        self.add_log('OPERATOR', 'manual takeover (teleop)')

    def takeover_ended(self):
        self.pub_manual.publish(Bool(data=False))
        self.teleop(0.0, 0.0)
        self.add_log('OPERATOR', 'manual takeover ended')

    def teleop(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub_teleop.publish(msg)
