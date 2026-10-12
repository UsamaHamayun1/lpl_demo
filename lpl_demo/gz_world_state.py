#!/usr/bin/env python3
# Replacement for Gazebo Classic's gazebo_ros_state plugin (/gazebo/model_states).
# Reads /world/<world>/pose/info straight from gz-transport (the ros_gz bridge drops
# the entity names) and republishes every entity pose as a TFMessage where
# child_frame_id is the entity name. Use lpl_demo.model_states to consume it.
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

from lpl_demo.model_states import MODEL_STATES_TOPIC


class GzWorldState(Node):
    def __init__(self):
        super().__init__('gz_world_state')

        self.world = self.declare_parameter('world', '').value
        rate = self.declare_parameter('rate', 50.0).value

        self.lock = threading.Lock()
        self.latest = None
        self.gz = GzNode()
        self.subscribed = False

        self.pub = self.create_publisher(TFMessage, MODEL_STATES_TOPIC, 10)
        self.create_timer(1.0 / rate, self.publish_latest)
        self.create_timer(1.0, self.try_subscribe)

    def try_subscribe(self):
        if self.subscribed:
            return
        topic = self.find_topic()
        if topic is None:
            self.get_logger().info('Waiting for Gazebo world...', throttle_duration_sec=5.0)
            return
        if self.gz.subscribe(Pose_V, topic, self.gz_callback):
            self.subscribed = True
            self.get_logger().info(f'Streaming {topic} -> {MODEL_STATES_TOPIC}')

    def find_topic(self):
        if self.world:
            return f'/world/{self.world}/pose/info'
        # No world given: pick the first one Gazebo advertises
        for t in self.gz.topic_list():
            if t.startswith('/world/') and t.endswith('/pose/info') and '/dynamic_pose/' not in t:
                return t
        return None

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
            # pose/info also carries links/visuals whose names repeat ('link', 'visual');
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
    rclpy.init(args=args)
    node = GzWorldState()
    # Ctrl+C under ros2 launch delivers SIGINT twice (terminal + launch), so the
    # second one can land during cleanup.
    try:
        rclpy.spin(node)
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
