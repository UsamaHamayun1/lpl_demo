# Unit tests for the LPL authority engine (no ROS needed).
import os

import pytest
import yaml

from lpl2_core.policy import (ADEQUATE, AVAILABLE, CONTRADICTED, DWELL_PENDING, EGRESS_ONLY, EVIDENCE, FIXED,
                              HUMAN_GRANT, HUMAN_GRANTED, INSUFFICIENT, STALE, UNAVAILABLE, AuthorityCore, Evidence)

RULES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'rules.yaml')
IDS = ['E1_own_lane_ok', 'E2_bypass_access_ok', 'E3_lane1_sector_clear']


@pytest.fixture
def rules():
    with open(RULES) as f:
        return yaml.safe_load(f)


def ev(now, e3=ADEQUATE, others=ADEQUATE, age=0.0):
    out = {i: Evidence(others, now - age, 1.0, 'OK') for i in IDS}
    out['E3_lane1_sector_clear'] = Evidence(e3, now - age, 1.0, 'X')
    return out


def state(res, pid):
    return next(p for p in res.permissions if p.id == pid)


def run(core, t, e3=ADEQUATE, inside=None, **kw):
    return core.update(t, ev(t, e3, **kw), inside or {})


def test_boot_all_available_without_dwell(rules):
    core = AuthorityCore(rules)
    res = run(core, 0.0)
    assert [p.state for p in res.permissions] == [AVAILABLE] * 3
    assert res.changed and core.revision == 1


def test_no_dwell_when_evidence_arrives_after_startup(rules):
    core = AuthorityCore(rules)
    res = core.update(0.0, {}, {})                  # authority up before the evidence source
    assert all(p.state == UNAVAILABLE and p.provenance == STALE for p in res.permissions)
    res = run(core, 0.5)
    assert all(p.state == AVAILABLE for p in res.permissions)


def test_only_p3_revoked_immediately(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    res = run(core, 0.1, INSUFFICIENT)
    assert state(res, 'P3').state == UNAVAILABLE and state(res, 'P3').provenance == EVIDENCE
    assert state(res, 'P1').state == AVAILABLE and state(res, 'P2').state == AVAILABLE
    assert res.changed and any(c.startswith('P3 AVAILABLE -> UNAVAILABLE') for c in res.causes)


def test_no_change_no_revision(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    rev = core.revision
    res = run(core, 0.1)
    assert not res.changed and core.revision == rev


def test_restore_dwell(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    run(core, 0.1, INSUFFICIENT)
    res = run(core, 0.2)
    assert state(res, 'P3').state == UNAVAILABLE and state(res, 'P3').provenance == DWELL_PENDING
    res = run(core, 1.1)
    assert state(res, 'P3').state == UNAVAILABLE
    res = run(core, 1.25)
    assert state(res, 'P3').state == AVAILABLE


def test_stale_evidence_denies(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    res = run(core, 5.0, age=2.0)       # observed 2 s ago with a 1 s ttl
    assert state(res, 'P3').state == UNAVAILABLE and state(res, 'P3').provenance == STALE
    assert state(res, 'P1').state == UNAVAILABLE


def test_grant_single_pass(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    run(core, 1.0, INSUFFICIENT)
    ok, reason, gid, exp = core.request_grant(2.0, 'P3', 'A', 60.0, core.revision)
    assert ok and reason == 'GRANTED' and exp == pytest.approx(62.0)
    res = run(core, 2.1, INSUFFICIENT)
    p3 = state(res, 'P3')
    assert p3.state == HUMAN_GRANTED and p3.provenance == HUMAN_GRANT and not p3.evidence_supported
    assert p3.grant_id == gid
    run(core, 10.0, INSUFFICIENT, inside={'lane1_window': True})
    assert core.grants['P3'].phase == 'IN_USE'
    res = run(core, 70.0, INSUFFICIENT, inside={'lane1_window': True})    # never expires while in use
    assert state(res, 'P3').state == HUMAN_GRANTED
    res = run(core, 71.0, INSUFFICIENT, inside={'lane1_window': False})
    assert state(res, 'P3').state == UNAVAILABLE
    assert core.grant_log[-1][1].phase == 'CONSUMED'


def test_grant_expires_unused(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    run(core, 1.0, INSUFFICIENT)
    core.request_grant(2.0, 'P3', 'A', 60.0, core.revision)
    assert state(run(core, 61.0, INSUFFICIENT), 'P3').state == HUMAN_GRANTED
    assert state(run(core, 62.5, INSUFFICIENT), 'P3').state == UNAVAILABLE
    assert core.grant_log[-1][1].phase == 'EXPIRED'


def test_contradiction_cancels_grant_and_refuses_new_one(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    run(core, 1.0, INSUFFICIENT)
    core.request_grant(2.0, 'P3', 'A', 60.0, core.revision)
    res = run(core, 3.0, CONTRADICTED)
    assert state(res, 'P3').state == UNAVAILABLE
    assert core.grant_log[-1][1].phase == 'CANCELLED'
    ok, reason, *_ = core.request_grant(3.5, 'P3', 'A', 60.0, core.revision)
    assert not ok and reason == 'CONTRADICTED'


def test_evidence_supersedes_grant(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    run(core, 1.0, INSUFFICIENT)
    core.request_grant(2.0, 'P3', 'A', 60.0, core.revision)
    run(core, 3.0)                       # evidence back, dwell running: still granted
    res = run(core, 4.1)
    assert state(res, 'P3').state == AVAILABLE and state(res, 'P3').provenance == EVIDENCE
    assert core.grant_log[-1][1].phase == 'SUPERSEDED'


def test_grant_refusals(rules):
    core = AuthorityCore(rules)
    run(core, 0.0)
    assert core.request_grant(0.5, 'P3', 'A', 60, core.revision)[1] == 'NOT_NEEDED'
    run(core, 1.0, INSUFFICIENT)
    assert core.request_grant(1.5, 'P1', 'A', 60, core.revision)[1] == 'NOT_GRANTABLE'
    assert core.request_grant(1.5, 'P3', 'A', 60, core.revision - 1)[1] == 'STALE_CONSENT'
    assert core.request_grant(1.5, 'P3', 'A', 60, core.revision)[1] == 'GRANTED'
    assert core.request_grant(1.6, 'P3', 'A', 60, core.revision)[1] == 'ALREADY_GRANTED'
    assert core.revoke_grant(1.7, 'P3') == (True, 'REVOKED')
    assert core.revoke_grant(1.8, 'P3') == (False, 'NO_ACTIVE_GRANT')


def test_egress_when_revoked_inside_region(rules):
    core = AuthorityCore(rules)
    run(core, 0.0, inside={'lane1_window': True})
    res = run(core, 1.0, INSUFFICIENT, inside={'lane1_window': True})
    assert state(res, 'P3').state == EGRESS_ONLY
    res = run(core, 2.0, INSUFFICIENT, inside={'lane1_window': True})
    assert state(res, 'P3').state == EGRESS_ONLY
    res = run(core, 3.0, INSUFFICIENT, inside={'lane1_window': False})
    assert state(res, 'P3').state == UNAVAILABLE


def test_no_egress_if_never_usable(rules):
    core = AuthorityCore(rules)
    run(core, 0.0, INSUFFICIENT)
    res = run(core, 1.0, INSUFFICIENT, inside={'lane1_window': True})
    assert state(res, 'P3').state == UNAVAILABLE


def test_permissive_mode_ignores_evidence(rules):
    core = AuthorityCore(rules, 'permissive')
    res = run(core, 0.0, INSUFFICIENT)
    assert all(p.state == AVAILABLE and p.provenance == FIXED for p in res.permissions)
    assert core.request_grant(1.0, 'P3', 'A', 60, core.revision)[1] == 'NOT_GRANTABLE'


def test_global_revoke_mode_freezes_everything(rules):
    core = AuthorityCore(rules, 'global_revoke')
    assert all(p.state == AVAILABLE for p in run(core, 0.0).permissions)
    res = run(core, 1.0, INSUFFICIENT)
    assert all(p.state == UNAVAILABLE for p in res.permissions)
