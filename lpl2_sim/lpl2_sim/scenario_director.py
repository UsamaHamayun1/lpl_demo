#!/usr/bin/env python3
# Scenario director: plays a scenario file (config/scenarios/*.yaml) against the world.
#
# It scripts the WORLD only (boxes appearing, Robot 2 moving). It never touches
# evidence, permissions or the planner: "script the world, not LPL's conclusion".
#
# Scenario format:
#   name: lpl1
#   description: ...
#   events:
#     - id: W1
#       note: Tall stack placed on the pallet
#       when: {robot_x_ge: 3.0}          # or {sim_time_ge: 10.0} or {after: W1, delay: 5.0}
#       do:
#         - {show: obstacle_tall}         # scripted box from corridor.yaml
#         - {hide: bypass_shutter}
#         - {robot2: release}             # release | stop | resume
# Every fired event is published on /sim/world_events (JSON, transient local) for logs/UI.
import json
import os

import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name
from lpl2_sim.runner import run

ROBOT_R = 'robot_r'
EVENTS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ScenarioDirector(Node):
    def __init__(self):
        super().__init__('scenario_director')
        geometry = self.declare_parameter('geometry_file', '').value or default_path()
        scenario_file = self.declare_parameter('scenario_file', '').value
        self.corridor = Corridor(geometry)
        with open(scenario_file) as f:
            self.scenario = yaml.safe_load(f)
        self.pending = list(self.scenario.get('events', []))
        self.fired = {}          # event id -> sim time fired
        self.robot_x = None

        self.set_pose = self.create_client(
            SetEntityPose, f'/world/{self.corridor.world_name}/set_pose')
        self.robot2 = {a: self.create_client(Trigger, f'/sim/robot2/{a}')
                       for a in ('release', 'stop', 'resume')}
        self.pub_events = self.create_publisher(String, '/sim/world_events', EVENTS_QOS)
        self.create_subscription(TFMessage, TRUTH_TOPIC, self.on_truth, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"Scenario '{self.scenario.get('name')}' ({os.path.basename(scenario_file)}): "
            f'{len(self.pending)} events')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        pose = poses_by_name(msg).get(ROBOT_R)
        if pose:
            self.robot_x = pose[0]

    def due(self, when):
        if 'robot_x_ge' in when:
            return self.robot_x is not None and self.robot_x >= when['robot_x_ge']
        if 'sim_time_ge' in when:
            return self.now() >= when['sim_time_ge']
        if 'after' in when:
            t = self.fired.get(when['after'])
            return t is not None and self.now() >= t + when.get('delay', 0.0)
        raise ValueError(f'unknown trigger {when}')

    def tick(self):
        if not self.set_pose.service_is_ready():
            return   # world not up yet
        for ev in list(self.pending):
            if self.due(ev['when']):
                self.pending.remove(ev)
                self.fire(ev)

    def fire(self, ev):
        t = self.now()
        self.fired[ev['id']] = t
        for action in ev.get('do', []):
            self.apply(action)
        record = {'id': ev['id'], 'note': ev.get('note', ''), 'sim_time': round(t, 3),
                  'robot_x': None if self.robot_x is None else round(self.robot_x, 3),
                  'do': ev.get('do', [])}
        self.pub_events.publish(String(data=json.dumps(record)))
        self.get_logger().info(f"[{ev['id']}] t={t:.2f}s robot_x={record['robot_x']}: {record['note']}")

    def apply(self, action):
        if 'show' in action or 'hide' in action:
            name = action.get('show') or action.get('hide')
            x, y, z = (self.corridor.shown_pose(name) if 'show' in action
                       else self.corridor.hidden_pose(name))
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
            self.set_pose.call_async(req).add_done_callback(
                lambda f, n=name: f.result().success or self.get_logger().error(f'set_pose {n} failed'))
        elif 'robot2' in action:
            client = self.robot2[action['robot2']]
            if not client.service_is_ready():
                self.get_logger().error('robot2_driver not running')
                return
            client.call_async(Trigger.Request()).add_done_callback(
                lambda f: f.result().success or self.get_logger().error(f.result().message))
        else:
            raise ValueError(f'unknown action {action}')


def main(args=None):
    run(ScenarioDirector, args)


if __name__ == '__main__':
    main()
