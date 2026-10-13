#!/usr/bin/env python3
# Split-screen recorder: Robot R's camera ("what the robot knows") above the top-down
# ground-truth view ("what is actually happening"), written to mp4.
#
# One output frame per top-down frame, using the image's sim-time stamp, so the video
# plays at true sim speed even when Gazebo runs slower than real time.
# Encoder: H.264 via GStreamer x264 (ffmpeg is not installed); falls back to mp4v.
import json
import math
import os
from datetime import datetime

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from lpl2_sim.geometry import Corridor, default_path
from lpl2_sim.truth import TOPIC as TRUTH_TOPIC, poses_by_name
from lpl2_sim.runner import run

WIDTH = 1280
BAR_H = 60
EVENT_SHOW_S = 8.0     # sim seconds a world event stays highlighted
FONT = cv2.FONT_HERSHEY_SIMPLEX
EVENTS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def to_bgr(msg):
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a.copy()


class TopdownProjection:
    """World (x, y, z) -> pixel in the top-down camera image (pinhole looking straight down)."""

    def __init__(self, cam):
        self.cx, self.cy, self.cz = cam['pose']
        self.w, self.h = cam['size']
        self.f = (self.w / 2) / math.tan(cam['hfov'] / 2)

    def pixel(self, x, y, z=0.0, scale=1.0):
        d = self.cz - z
        u = self.w / 2 + self.f * (x - self.cx) / d
        v = self.h / 2 - self.f * (y - self.cy) / d
        return int(u * scale), int(v * scale)


def label(img, text, org, color=(255, 255, 255), size=0.7, thick=2):
    cv2.putText(img, text, org, FONT, size, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, size, color, thick, cv2.LINE_AA)


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        corridor = Corridor(self.declare_parameter('geometry_file', '').value or default_path())
        self.scenario = self.declare_parameter('scenario', 'none').value
        out_dir = os.path.expanduser(self.declare_parameter('output_dir', '~/lpl_recordings').value)
        encoder = self.declare_parameter('encoder', 'x264').value

        cam = corridor.cameras['topdown']
        self.fps = float(cam['rate'])
        self.proj = TopdownProjection(cam)
        self.td_scale = WIDTH / cam['size'][0]
        self.td_h = int(round(cam['size'][1] * self.td_scale))
        fw, fh = corridor.robot_r['camera_size']
        self.fpv_h = int(round(fh * WIDTH / fw))
        self.size = (WIDTH, BAR_H + self.fpv_h + self.td_h)

        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(out_dir, f'{self.scenario}_{stamp}.mp4')
        self.writer = self.open_writer(encoder)

        self.fpv = None
        self.poses = {}
        self.events = []        # (sim_time, text)
        self.frames = 0
        self.t_first = None
        self.create_subscription(Image, '/camera/image_raw', self.on_fpv, 2)
        self.create_subscription(Image, '/topdown/image', self.on_topdown, 2)
        self.create_subscription(TFMessage, TRUTH_TOPIC, self.on_truth, 10)
        self.create_subscription(String, '/sim/world_events', self.on_event, EVENTS_QOS)
        self.get_logger().info(f'Recording to {self.path} ({self.size[0]}x{self.size[1]} @ {self.fps:g} fps sim time)')

    def open_writer(self, encoder):
        if encoder == 'x264':
            pipe = ('appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=veryfast bitrate=6000 '
                    f'! mp4mux ! filesink location={self.path}')
            w = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, self.fps, self.size, True)
            if w.isOpened():
                return w
            self.get_logger().warn('GStreamer x264 unavailable, falling back to mp4v')
        w = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, self.size)
        if not w.isOpened():
            raise RuntimeError(f'cannot open video writer for {self.path}')
        return w

    def on_fpv(self, msg):
        self.fpv = msg

    def on_truth(self, msg):
        self.poses = poses_by_name(msg)

    def on_event(self, msg):
        ev = json.loads(msg.data)
        self.events.append((ev['sim_time'], f"{ev['id']}: {ev['note']}"))

    def on_topdown(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_first is None:
            self.t_first = t
        canvas = np.zeros((self.size[1], self.size[0], 3), np.uint8)

        # status bar
        label(canvas, f'LPL 2.0  |  scenario: {self.scenario}', (14, 40), size=0.8)
        label(canvas, f't = {t - self.t_first:6.1f} s', (WIDTH - 190, 40), size=0.8)

        # robot camera
        y0 = BAR_H
        if self.fpv is not None:
            canvas[y0:y0 + self.fpv_h] = cv2.resize(to_bgr(self.fpv), (WIDTH, self.fpv_h))
        label(canvas, "ROBOT R CAMERA - what the robot can see", (14, y0 + 34))

        # top-down ground truth
        y1 = y0 + self.fpv_h
        td = cv2.resize(to_bgr(msg), (WIDTH, self.td_h))
        for name, text, color in (('robot_r', 'R', (230, 210, 0)), ('robot2', 'R2', (255, 0, 255))):
            if name in self.poses:
                x, y, _ = self.poses[name]
                u, v = self.proj.pixel(x, y, 0.3, self.td_scale)
                label(td, text, (u - 10, v - 18), color, 0.7)
        canvas[y1:y1 + self.td_h] = td
        label(canvas, 'TOP-DOWN - ground truth (not robot knowledge)', (14, y1 + 30))

        # world events, highlighted for a few seconds
        recent = [e for e in self.events if 0 <= t - e[0] < EVENT_SHOW_S]
        for i, (_, text) in enumerate(recent[-2:]):
            label(canvas, f'WORLD EVENT  {text}', (14, y1 - 20 - 34 * i), (0, 200, 255), 0.75)

        self.writer.write(canvas)
        self.frames += 1

    def close(self):
        self.writer.release()
        self.get_logger().info(f'Saved {self.frames} frames to {self.path}')


def main(args=None):
    run(Recorder, args, on_exit=Recorder.close)


if __name__ == '__main__':
    main()
