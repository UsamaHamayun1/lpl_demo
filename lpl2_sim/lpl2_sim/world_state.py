#!/usr/bin/env python3
# Ground-truth poses from Gazebo, published as /sim/model_poses (TFMessage,
# child_frame_id = entity name, frame 'world'). Reads /world/<world>/pose/info
# straight from gz-transport because the ros_gz bridge drops entity names.
# This is simulator truth: scripted-world tools and the top-down view may use it;
# robot autonomy must not (it gets evidence through the LPL layers instead).
import threading

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

from lpl2_sim.truth import TOPIC
from lpl2_sim.runner import run


class WorldState(Node):
    def __init__(self):
        super().__init__('world_state')
        world = self.declare_parameter('world', 'lpl2_corridor').value
        rate = self.declare_parameter('rate', 50.0).value
        self.topic = f'/world/{world}/pose/info'

        self.lock = threading.Lock()
        self.latest = None
        self.gz = GzNode()
        if not self.gz.subscribe(Pose_V, self.topic, self.gz_callback):
            raise RuntimeError(f'cannot subscribe to {self.topic}')
        self.pub = self.create_publisher(TFMessage, TOPIC, 10)
        self.create_timer(1.0 / rate, self.publish_latest)
        self.get_logger().info(f'Streaming {self.topic} -> {TOPIC}')

    def gz_callback(self, msg):
        with self.lock:
            self.latest = msg

    def publish_latest(self):
        with self.lock:
            msg, self.latest = self.latest, None
        if msg is None:
            return
        out = TFMessage()
        stamp = self.get_clock().now().to_msg()
        seen = set()
        for p in msg.pose:
            # pose/info also lists links/visuals whose names repeat ('link', 'visual');
            # model names are unique, so de-duplicating never hides a model.
            if p.name in seen:
                continue
            seen.add(p.name)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'world'
            t.child_frame_id = p.name
            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation.x = p.orientation.x
            t.transform.rotation.y = p.orientation.y
            t.transform.rotation.z = p.orientation.z
            t.transform.rotation.w = p.orientation.w
            out.transforms.append(t)
        self.pub.publish(out)


def main(args=None):
    run(WorldState, args)


if __name__ == '__main__':
    main()
