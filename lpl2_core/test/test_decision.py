# Unit tests for available actions / selection / escalation (no ROS needed).
import os

import pytest
import yaml

from lpl2_core.decision import ADVISORY, FAULT, HJR, NONE, Decider, PermView
from lpl2_core.options import load_options
from lpl2_core.regions import Regions
from lpl2_sim.geometry import Corridor

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(os.path.dirname(HERE), 'lpl2_sim')


def cfg(name):
    with open(os.path.join(HERE, 'config', name)) as f:
        return yaml.safe_load(f)


@pytest.fixture
def decider():
    regions = Regions(Corridor(os.path.join(SIM, 'config', 'corridor.yaml')), cfg('rules.yaml'))
    opts = cfg('options.yaml')
    return Decider(load_options(opts), regions, cfg('mission.yaml'), opts['safe_hold'])


def perms(p3='AVAILABLE', p3_by_evidence=True, p2='AVAILABLE'):
    return {
        'P1': PermView('P1', 'lane2', 'AVAILABLE', True, True, False),
        'P2': PermView('P2', 'bypass', p2, p2 == 'AVAILABLE', True, True),
        'P3': PermView('P3', 'lane1_window', p3, p3 == 'AVAILABLE', p3_by_evidence, True),
    }


def get(statuses, oid):
    return next(s for s in statuses if s.id == oid)


def test_requires_are_derived_from_geometry(decider):
    st, sel, _, esc = decider.evaluate(0.0, (0.0, -0.6), perms(), True, False)
    assert get(st, 'A').requires == ['P1', 'P3']
    assert get(st, 'B').requires == ['P1', 'P2']
    assert get(st, 'H').requires == []
    assert sel == 'A' and esc.level == NONE


def test_requires_shrink_as_path_is_used(decider):
    st, *_ = decider.evaluate(0.0, (18.0, -0.6), perms(), True, False)
    assert get(st, 'A').requires == ['P1']        # past the window: P3 no longer needed


def test_p3_revoked_selects_b_and_advises(decider):
    st, sel, reason, esc = decider.evaluate(0.0, (3.0, -0.6), perms('UNAVAILABLE'), True, False)
    a = get(st, 'A')
    assert a.possible and a.reachable and not a.authorized and not a.evidenced and a.missing == ['P3']
    assert sel == 'B' and get(st, 'B').adequate
    assert esc.level == ADVISORY and esc.asks == ['P3'] and esc.advised == 'A' and esc.gain_s > 5


def test_no_advisory_while_restoring(decider):
    _, sel, _, esc = decider.evaluate(0.0, (3.0, -0.6), perms('UNAVAILABLE', p3_by_evidence=False), True, False)
    assert sel == 'B' and esc.level == NONE


def test_commit_point_removes_a_and_advisory(decider):
    st, sel, _, esc = decider.evaluate(0.0, (9.0, -2.0), perms('UNAVAILABLE'), True, False)
    assert not get(st, 'A').reachable and sel == 'B' and esc.level == NONE


def test_human_grant_makes_a_executable(decider):
    st, sel, _, esc = decider.evaluate(0.0, (5.0, -0.6), perms('HUMAN_GRANTED', False), True, False)
    a = get(st, 'A')
    assert a.authorized and not a.evidenced and a.adequate and sel == 'A' and esc.level == NONE


def block(decider, oid):
    for _ in range(decider.m['feasibility']['blocked_after']):
        decider.report_feasibility(oid, False)


def test_handover_only_after_confirm_time(decider):
    block(decider, 'B')
    st, sel, _, esc = decider.evaluate(10.0, (6.5, -0.6), perms('UNAVAILABLE'), True, False)
    assert not get(st, 'B').possible and sel == 'H' and esc.level == NONE      # confirming
    _, sel, _, esc = decider.evaluate(11.1, (6.5, -0.6), perms('UNAVAILABLE'), True, False)
    assert esc.level == HJR and esc.kind == 'AUTHORITY_EXHAUSTED' and esc.asks == ['P3'] and esc.advised == 'A'


def test_no_feasible_when_nothing_possible(decider):
    block(decider, 'A')
    block(decider, 'B')
    decider.evaluate(0.0, (6.5, -0.6), perms('UNAVAILABLE'), True, False)
    _, _, _, esc = decider.evaluate(2.0, (6.5, -0.6), perms('UNAVAILABLE'), True, False)
    assert esc.level == HJR and esc.kind == 'NO_FEASIBLE' and esc.asks == []


def test_blocked_latch_hysteresis(decider):
    block(decider, 'B')
    decider.report_feasibility('B', True)
    st, *_ = decider.evaluate(0.0, (5.0, -0.6), perms(), True, False)
    assert not get(st, 'B').possible
    for _ in range(decider.m['feasibility']['clear_after'] - 1):
        decider.report_feasibility('B', True)
    st, *_ = decider.evaluate(0.1, (5.0, -0.6), perms(), True, False)
    assert get(st, 'B').possible


def test_stale_authority_is_a_fault_not_a_handover(decider):
    st, sel, _, esc = decider.evaluate(0.0, (3.0, -0.6), perms(), False, False)
    assert sel == 'H' and esc.level == FAULT and not any(s.authorized for s in st if not s.hold)


def test_mission_done_holds_quietly(decider):
    _, sel, reason, esc = decider.evaluate(0.0, (24.0, -0.6), perms(), True, True)
    assert sel == 'H' and reason == 'mission complete' and esc.level == NONE


def test_never_holds_in_lane1_egress_first(decider):
    """Granted pass, then Robot 2 seen head-on: A and B impossible while R is in Lane 1."""
    block(decider, 'A')
    block(decider, 'B')
    st, sel, reason, esc = decider.evaluate(0.0, (12.7, 0.43), perms('HUMAN_GRANTED', False), True, False)
    e = get(st, 'E')
    assert sel == 'E' and e.egress and e.adequate and esc.level == NONE
    assert e.remaining_path[-1] == pytest.approx((12.0, -0.6))     # nearest own-lane point, not under the pallet
    st, sel, _, _ = decider.evaluate(5.0, (12.0, -0.6), perms('HUMAN_GRANTED', False), True, False)
    assert sel == 'H' and not any(s.id == 'E' for s in st)          # back in own lane: holding is safe


def test_safe_to_hold(decider):
    assert decider.safe_to_hold(5.0, -0.7)
    assert decider.safe_to_hold(13.0, -3.4)          # in the bypass
    assert not decider.safe_to_hold(12.7, 0.43)      # Lane 1
    assert not decider.safe_to_hold(12.0, -0.30)     # inside the lane-1 margin
