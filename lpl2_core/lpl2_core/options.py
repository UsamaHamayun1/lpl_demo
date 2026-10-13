# Option geometry: build paths from config/options.yaml, project the robot onto them,
# and derive which permission regions the remaining path needs.
import math
from dataclasses import dataclass

STEP = 0.1          # path sampling (m)
FOOTPRINT_R = 0.2   # radius used to decide which regions a path touches


@dataclass
class Option:
    id: str
    name: str
    hold: bool
    path: list            # [(x, y)] sampled every STEP
    unreachable_inside: list


def _ramp(a, b):
    """Cosine lane change from a to b (x increasing)."""
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / STEP))
    return [(a[0] + (b[0] - a[0]) * i / n,
             a[1] + (b[1] - a[1]) * (1 - math.cos(math.pi * i / n)) / 2) for i in range(n)]


def _line(a, b):
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / STEP))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]


def build_path(waypoints):
    pts = []
    for a, b in zip(waypoints, waypoints[1:]):
        lane_change = abs(b[1] - a[1]) > 1e-6 and abs(b[0] - a[0]) > 1.0
        pts += _ramp(a, b) if lane_change else _line(a, b)
    pts.append(tuple(waypoints[-1]))
    return pts


def load_options(cfg):
    out = []
    for oid, o in cfg['options'].items():
        hold = bool(o.get('hold', False))
        out.append(Option(oid, o.get('name', oid), hold, [] if hold else build_path(o['waypoints']),
                          o.get('unreachable_inside', [])))
    return out


def project(path, x, y):
    """-> (index of the nearest path point, distance to it)."""
    best_i, best_d = 0, float('inf')
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_i, best_d = i, d
    return best_i, math.sqrt(best_d)


def length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def in_box(box, x, y):
    return box['x'][0] <= x <= box['x'][1] and box['y'][0] <= y <= box['y'][1]


def regions_touched(path, regions):
    """Names of permission regions the robot's footprint enters along `path`."""
    names = set()
    for x, y in path[::2]:
        for name in regions.regions:
            if name not in names and regions.inside(name, x, y, FOOTPRINT_R):
                names.add(name)
    return names
