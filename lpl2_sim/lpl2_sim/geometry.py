# Loads config/corridor.yaml, the single source of truth for the LPL 2.0 corridor.
# Used by the generator (tools/gen_corridor.py) and by the runtime nodes, so the
# world, the map, the masks and the scenario events always agree.
import os
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Box:
    name: str
    x: tuple
    y: tuple
    z: tuple
    rgba: tuple

    @property
    def size(self):
        return (self.x[1] - self.x[0], self.y[1] - self.y[0], self.z[1] - self.z[0])

    @property
    def center(self):
        return ((self.x[0] + self.x[1]) / 2, (self.y[0] + self.y[1]) / 2, (self.z[0] + self.z[1]) / 2)


class Corridor:
    def __init__(self, path):
        with open(path) as f:
            self.cfg = yaml.safe_load(f)
        c = self.cfg
        self.world_name = c['world_name']
        self.hidden_z = c['hidden_z']
        self.colors = {k: tuple(v) for k, v in c['colors'].items()}
        self.static_boxes = [self._box(n, b) for n, b in c['static_boxes'].items()]
        self.scripted_boxes = {n: self._box(n, b) for n, b in c['scripted_boxes'].items()}
        self.zones = c['zones']
        self.robot_r = c['robot_r']
        self.robot2 = c['robot2']
        self.cameras = c['cameras']
        self.map = c['map']
        self.centerline = c['centerline']

    def _box(self, name, b):
        return Box(name, tuple(b['x']), tuple(b['y']), tuple(b['z']), self.colors[b['color']])

    def shown_pose(self, name):
        """Pose of a scripted box when present in the world."""
        return self.scripted_boxes[name].center

    def hidden_pose(self, name):
        cx, cy, _ = self.scripted_boxes[name].center
        return (cx, cy, self.hidden_z)


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('lpl2_sim'), 'config', 'corridor.yaml')
