from lpl2_core.cmd_gate import depth_into

RECT = (10.5, 17.0, -0.35, 1.2)     # lane1_window as a keepout area


def test_outside_far():
    assert depth_into(RECT, 5.0, -0.75, 0.18) < 0


def test_touching_edge():
    assert depth_into(RECT, 12.0, -0.50, 0.18) > 0        # footprint reaches 0.03 m in


def test_deeper_is_larger():
    a = depth_into(RECT, 12.0, -0.30, 0.18)
    b = depth_into(RECT, 12.0, 0.20, 0.18)
    assert 0 < a < b
