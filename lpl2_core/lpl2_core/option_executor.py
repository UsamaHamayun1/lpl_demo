#!/usr/bin/env python3
# Option executor: carries out the planner's selected option with Nav2.
#
# A route option is sent to Nav2 as NavigateThroughPoses along that option's remaining
# path. Nav2 plans on the LPL-constrained costmap (keepout mask), so even this layer
# cannot drive into a region whose permission is not usable. HOLD cancels the goal.
# A new selection pre-empts the running goal.
#
#   in:  /lpl2/decision_space
#   out: navigate_through_poses (Nav2), /lpl2/execution_state (10 Hz heartbeat)
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt32

from lpl2_interfaces.msg import DecisionSpace, ExecutionState, OptionStatus
from lpl2_sim.runner import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
WAYPOINT_SPACING = 1.5   # m between NavigateThroughPoses waypoints
RETRY_AFTER = 2.0        # s before re-sending an option whose goal ended without success
MASK_SETTLE = 0.5        # s for Nav2's costmaps to pick up a new keepout mask
MAX_RETRIES = 3          # consecutive failures of the same option before giving up (reported as FAILED)


class OptionExecutor(Node):
    def __init__(self):
        super().__init__('option_executor')
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.pub = self.create_publisher(ExecutionState, '/lpl2/execution_state', 10)
        self.create_subscription(DecisionSpace, '/lpl2/decision_space', self.on_decision, LATCHED)
        self.create_subscription(UInt32, '/lpl2/constraints_revision', self.on_constraints, LATCHED)
        self.mask_rev, self.mask_rev_at = 0, None
        self.create_timer(0.1, self.heartbeat)
        self.option = ''            # option currently being executed (or held)
        self.kind = OptionStatus.ROUTE
        self.goal_handle = None
        self.active = False         # a goal is sent and not yet finished
        self.goal_seq = 0
        self.state = ExecutionState.IDLE
        self.error = 0
        self.detail = 'waiting for a decision'
        self.remaining = 0.0
        self.ended_at = None
        self.failures = 0

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_constraints(self, msg):
        if msg.data != self.mask_rev:
            self.mask_rev, self.mask_rev_at = msg.data, self.now()

    def mask_ready(self, revision):
        """Nav2 has (very likely) seen the keepout mask for this authority revision."""
        return self.mask_rev >= revision and self.mask_rev_at is not None and \
            self.now() - self.mask_rev_at >= MASK_SETTLE

    def on_decision(self, d):
        sel = next((o for o in d.options if o.id == d.selected_id), None)
        if sel is None:
            return
        self.remaining = sel.remaining_m
        if d.selection_reason == 'mission complete':
            if self.state != ExecutionState.SUCCEEDED:
                self.cancel()
                self.set(ExecutionState.SUCCEEDED, sel.id, 'goal reached')
            return
        if sel.kind == OptionStatus.HOLD:
            if self.state != ExecutionState.HOLDING:
                self.cancel()
                self.set(ExecutionState.HOLDING, sel.id, d.selection_reason)
            return
        if sel.id != self.option:
            self.failures = 0
        running = self.active and self.option == sel.id
        if not running and self.option == sel.id and self.failures >= MAX_RETRIES:
            return                                     # given up on this option; planner reports a fault
        retry_ok = self.ended_at is None or self.now() - self.ended_at >= RETRY_AFTER
        if not running and (self.option != sel.id or retry_ok):
            if not self.mask_ready(d.authority_revision):
                self.detail = f'waiting for the keepout mask of authority rev {d.authority_revision}'
                return
            self.send(sel)

    def send(self, opt):
        if not self.client.server_is_ready():
            self.detail = 'waiting for Nav2'
            return
        pts = [(p.x, p.y) for p in opt.path]
        if len(pts) < 2:
            return
        goal = NavigateThroughPoses.Goal()
        picked, walked = [pts[0]], 0.0
        for a, b in zip(pts, pts[1:]):
            walked += math.hypot(b[0] - a[0], b[1] - a[1])
            if walked >= WAYPOINT_SPACING:
                picked.append(b)
                walked = 0.0
        if picked[-1] != pts[-1]:
            picked.append(pts[-1])
        prev = pts[0]
        for x, y in picked[1:] or [pts[-1]]:
            yaw = math.atan2(y - prev[1], x - prev[0])      # heading of the path into this waypoint
            prev = (x, y)
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(p)
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_handle, self.ended_at, self.active = None, None, True
        self.kind = opt.kind
        state = ExecutionState.EGRESS if opt.kind == OptionStatus.EGRESS else ExecutionState.EXECUTING
        self.set(state, opt.id, f'{len(goal.poses)} waypoints sent to Nav2')
        self.get_logger().info(f'executing option {opt.id} ({len(goal.poses)} waypoints)')
        self.client.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, seq))

    def on_accepted(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:
            return                                     # superseded meanwhile
        if not handle.accepted:
            self.set(ExecutionState.REFUSED, self.option, 'Nav2 rejected the goal')
            self.ended_at, self.active = self.now(), False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, future, seq):
        if seq != self.goal_seq:
            return                                     # result of a pre-empted goal
        res = future.result()
        self.goal_handle, self.ended_at, self.active = None, self.now(), False
        if res.status == GoalStatus.STATUS_SUCCEEDED and self.kind == OptionStatus.EGRESS:
            self.failures = 0
            self.set(ExecutionState.HOLDING, self.option, 'back in own lane')     # not the mission goal
        elif res.status == GoalStatus.STATUS_SUCCEEDED:
            self.failures = 0
            self.set(ExecutionState.SUCCEEDED, self.option, 'goal reached')
        elif res.status != GoalStatus.STATUS_CANCELED:
            self.error = res.result.error_code
            self.failures += 1
            tail = 'giving up' if self.failures >= MAX_RETRIES else 'will retry'
            self.set(ExecutionState.FAILED, self.option,
                     f'Nav2 ended the goal (code {res.result.error_code} {res.result.error_msg}); '
                     f'failure {self.failures}/{MAX_RETRIES}, {tail}')

    def cancel(self):
        self.goal_seq += 1
        self.active = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def set(self, state, option, detail):
        if (state, option) != (self.state, self.option):
            self.get_logger().info(f'{option}: {detail}')
        self.state, self.option, self.detail = state, option, detail

    def heartbeat(self):
        m = ExecutionState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.option_id, m.state, m.nav2_error_code = self.option, self.state, self.error
        m.detail, m.remaining_m = self.detail, float(self.remaining)
        self.pub.publish(m)


def main(args=None):
    run(OptionExecutor, args)


if __name__ == '__main__':
    main()
