#!/usr/bin/env python3
# Constraint compiler: authority envelope -> Nav2 keepout mask.
#
# This is where a permission change becomes real to the autonomy stack: the regions of
# permissions that are not usable become keepout cells on /keepout_mask, which Nav2's
# costmaps read (lpl2_nav keepout_layer). The planner then cannot route through them.
#
# Fail-safe: until an AuthorityState arrives, or if it goes silent for longer than
# lease_s, every permission region is closed (missing permissions = unavailable).
import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import UInt32
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from lpl2_core.regions import Regions
from lpl2_interfaces.msg import AuthorityState, PermissionState
from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.runner import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
USABLE = (PermissionState.AVAILABLE, PermissionState.HUMAN_GRANTED, PermissionState.EGRESS_ONLY)


class ConstraintCompiler(Node):
    def __init__(self):
        super().__init__('constraint_compiler')
        default_rules = os.path.join(get_package_share_directory('lpl2_core'), 'config', 'rules.yaml')
        with open(self.declare_parameter('rules_file', default_rules).value) as f:
            rules = yaml.safe_load(f)
        self.lease = self.declare_parameter('lease_s', 1.0).value
        corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.regions = Regions(corridor, rules)
        self.usable = None
        self.last_rx = None

        self.pub = self.create_publisher(OccupancyGrid, '/keepout_mask', LATCHED)
        # which authority revision the published mask reflects (the executor waits for it)
        self.pub_rev = self.create_publisher(UInt32, '/lpl2/constraints_revision', LATCHED)
        self.confirmed = None
        self.create_subscription(AuthorityState, '/lpl2/authority', self.on_authority, LATCHED)
        self.create_timer(0.2, self.check_lease)
        self.publish(frozenset(), 'no authority yet: all permission regions closed')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_authority(self, msg):
        self.last_rx = self.now()
        usable = frozenset(p.region_id for p in msg.permissions if p.state in USABLE and p.region_id)
        if usable != self.usable:
            self.publish(usable, f'authority rev {msg.revision}')
        if msg.revision != self.confirmed:
            self.confirmed = msg.revision
            self.pub_rev.publish(UInt32(data=msg.revision))

    def check_lease(self):
        if self.last_rx is not None and self.now() - self.last_rx > self.lease and self.usable:
            self.publish(frozenset(), f'authority silent for > {self.lease:.1f} s: all permission regions closed')

    def publish(self, usable, why):
        self.usable = usable
        r = self.regions
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = r.res
        grid.info.width, grid.info.height = r.w, r.h
        grid.info.origin.position.x, grid.info.origin.position.y = r.origin
        grid.info.origin.orientation.w = 1.0
        grid.data = np.where(r.keepout(usable), 100, 0).astype(np.int8).ravel().tolist()
        self.pub.publish(grid)
        closed = sorted(set(r.regions) - usable)
        self.get_logger().info(f"mask: open {sorted(usable) or '-'}, closed {closed or '-'} ({why})")


def main(args=None):
    run(ConstraintCompiler, args)


if __name__ == '__main__':
    main()
