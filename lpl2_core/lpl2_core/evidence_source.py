#!/usr/bin/env python3
# Evidence layer, scripted mode (stand-in for Quang's perception).
#
# Reports the evidence items in config/evidence/<file>.yaml and changes them when a
# world event fires, exactly as the script says. Publishes the full state on
# /lpl2/evidence at a fixed rate; each item carries observed_at + ttl so consumers can
# tell fresh from stale. Everything is marked scripted=true.
# Quang's node can replace this one by publishing the same EvidenceArray contract.
import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from lpl2_interfaces.msg import EvidenceArray, EvidenceItem
from lpl2_sim.runner import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
LEVELS = {'ADEQUATE': EvidenceItem.ADEQUATE, 'INSUFFICIENT': EvidenceItem.INSUFFICIENT,
          'UNKNOWN': EvidenceItem.UNKNOWN, 'CONTRADICTED': EvidenceItem.CONTRADICTED}


class EvidenceSource(Node):
    def __init__(self):
        super().__init__('evidence_source')
        default = os.path.join(get_package_share_directory('lpl2_core'), 'config', 'evidence', 'corridor.yaml')
        path = self.declare_parameter('evidence_file', default).value
        rate = self.declare_parameter('rate', 10.0).value
        self.ttl = self.declare_parameter('ttl_s', 1.0).value
        with open(path) as f:
            self.script = yaml.safe_load(f)
        self.props = {k: v['proposition'] for k, v in self.script['items'].items()}
        self.state = {k: dict(v) for k, v in self.script['initial'].items()}
        self.seq = 0
        self.seen_events = set()

        self.pub = self.create_publisher(EvidenceArray, '/lpl2/evidence', LATCHED)
        self.create_subscription(String, '/sim/world_events', self.on_event, EVENTS_QOS)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'Scripted evidence from {os.path.basename(path)}: {", ".join(self.state)}')

    def on_event(self, msg):
        ev = json.loads(msg.data)
        if ev['id'] in self.seen_events:
            return
        self.seen_events.add(ev['id'])
        for item_id, change in (self.script.get('on_world_event') or {}).get(ev['id'], {}).items():
            self.state[item_id] = dict(change)
            self.get_logger().info(f"world event {ev['id']} -> {item_id} {change['level']} ({change.get('reason', '')})")

    def publish(self):
        now = self.get_clock().now().to_msg()
        out = EvidenceArray()
        out.header.stamp = now
        self.seq += 1
        out.seq = self.seq
        for item_id, s in self.state.items():
            it = EvidenceItem()
            it.id, it.proposition = item_id, self.props.get(item_id, '')
            it.level = LEVELS[s['level']]
            it.reason, it.detail = s.get('reason', ''), s.get('detail', '')
            it.confidence = float(s.get('confidence', 1.0))
            it.observed_at, it.ttl_s = now, float(self.ttl)
            it.source, it.scripted = 'scripted', True
            out.items.append(it)
        self.pub.publish(out)


def main(args=None):
    run(EvidenceSource, args)


if __name__ == '__main__':
    main()
