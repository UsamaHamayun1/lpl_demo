# The compiled keepout must match the masks generated from corridor.yaml, and region
# membership must use the same areas.
import os

import numpy as np
import pytest
import yaml

from lpl2_core.regions import Regions
from lpl2_sim.geometry import Corridor

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(os.path.dirname(HERE), 'lpl2_sim')


@pytest.fixture
def regions():
    with open(os.path.join(HERE, 'config', 'rules.yaml')) as f:
        rules = yaml.safe_load(f)
    return Regions(Corridor(os.path.join(SIM, 'config', 'corridor.yaml')), rules)


def load_mask(name):
    d = open(os.path.join(SIM, 'maps', f'mask_{name}.pgm'), 'rb').read().split(b'\n', 3)
    w, h = map(int, d[1].split())
    img = np.frombuffer(d[3], np.uint8).reshape(h, w)[::-1]   # pgm row 0 = top -> flip to row 0 = lowest y
    return img == 0


def test_all_usable_matches_p3_available_mask(regions):
    assert np.array_equal(regions.keepout({'lane1_window', 'bypass', 'lane2'}), load_mask('p3_available'))


def test_p3_unusable_matches_p3_unavailable_mask(regions):
    assert np.array_equal(regions.keepout({'bypass', 'lane2'}), load_mask('p3_unavailable'))


def test_inside(regions):
    assert regions.inside('lane1_window', 13.0, 0.5)
    assert regions.inside('lane1_window', 13.0, -0.30)          # within the 0.35 m margin
    assert not regions.inside('lane1_window', 13.0, -0.75)
    assert regions.inside('lane1_window', 10.3, 0.5, radius=0.25)
    assert not regions.inside('lane1_window', 9.0, 0.5, radius=0.25)
    assert regions.inside('bypass', 13.0, -3.4)
