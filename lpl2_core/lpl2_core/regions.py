# Permission regions on the map, from rules.yaml `constraints` + corridor.yaml zones.
# One place answers both "which cells are keepout for this permission set?"
# (constraint compiler -> Nav2 mask) and "is the robot inside this region?"
# (authority -> single-pass grants and egress), so the two can never disagree.
import numpy as np


def _rect(zone, grow_y_min=0.0):
    return (zone['x'][0], zone['x'][1], zone['y'][0] - grow_y_min, zone['y'][1])


class Regions:
    def __init__(self, corridor, rules):
        cons = rules['constraints']
        zones = corridor.zones
        self.standing = []          # list of rects that are always keepout
        for s in cons.get('standing_keepout', []):
            x0, x1, y0, y1 = _rect(zones[s['zone']], s.get('grow_y_min', 0.0))
            if 'except_zone' in s:
                ex0, ex1 = zones[s['except_zone']]['x']
                self.standing += [(x0, ex0, y0, y1), (ex1, x1, y0, y1)]
            else:
                self.standing.append((x0, x1, y0, y1))
        self.regions = {name: _rect(zones[r['zone']], r.get('grow_y_min', 0.0))
                        for name, r in cons.get('regions', {}).items()}

        m = corridor.map
        self.res = m['resolution']
        self.origin = tuple(m['origin'])
        self.w = int(round(m['size'][0] / self.res))
        self.h = int(round(m['size'][1] / self.res))
        xs = self.origin[0] + (np.arange(self.w) + 0.5) * self.res
        ys = self.origin[1] + (np.arange(self.h) + 0.5) * self.res
        self._X, self._Y = np.meshgrid(xs, ys)     # row 0 = lowest y (OccupancyGrid order)

    def _cells(self, rect):
        x0, x1, y0, y1 = rect
        return (self._X >= x0) & (self._X <= x1) & (self._Y >= y0) & (self._Y <= y1)

    def keepout(self, usable_regions):
        """Boolean grid: True where the robot may not go given the usable permission regions."""
        mask = np.zeros_like(self._X, dtype=bool)
        for rect in self.standing:
            mask |= self._cells(rect)
        for name, rect in self.regions.items():
            if name not in usable_regions:
                mask |= self._cells(rect)
        return mask

    def inside(self, name, x, y, radius=0.0):
        """Does a circle of `radius` around (x, y) overlap region `name`?"""
        x0, x1, y0, y1 = self.regions[name]
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        return dx * dx + dy * dy <= radius * radius
