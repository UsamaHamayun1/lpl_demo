# LPL authority engine: evidence -> decision permissions.
#
# Pure Python, no ROS, so every rule can be unit-tested. It answers only
# "which decisions belong to the machine right now?". It knows nothing about routes,
# options, costs or the mission (the planner's job), and it never commands the robot.
#
# Inputs each cycle: the latest evidence items, the current time, and which permission
# regions the robot currently occupies (needed for single-pass grants and egress).
from dataclasses import dataclass, field

AVAILABLE, UNAVAILABLE, HUMAN_GRANTED, EGRESS_ONLY = 'AVAILABLE', 'UNAVAILABLE', 'HUMAN_GRANTED', 'EGRESS_ONLY'
EVIDENCE, HUMAN_GRANT, DWELL_PENDING, STALE, FIXED = 'EVIDENCE', 'HUMAN_GRANT', 'DWELL_PENDING', 'STALE', 'FIXED'
ADEQUATE, INSUFFICIENT, UNKNOWN, CONTRADICTED = 'ADEQUATE', 'INSUFFICIENT', 'UNKNOWN', 'CONTRADICTED'
MODES = ('lpl', 'permissive', 'global_revoke')
USABLE = (AVAILABLE, HUMAN_GRANTED, EGRESS_ONLY)


@dataclass
class Evidence:
    level: str
    observed_at: float
    ttl_s: float
    reason: str = ''


@dataclass
class Grant:
    id: str
    permission: str
    for_option: str
    granted_at: float
    expires_at: float          # deadline to start using it
    phase: str = 'ACTIVE_UNUSED'   # -> IN_USE -> ended (CONSUMED / EXPIRED / REVOKED / SUPERSEDED / CANCELLED)


@dataclass
class Permission:
    id: str
    name: str
    region: str
    state: str
    evidence_supported: bool
    provenance: str
    evidence_ids: list
    reason: str
    since: float
    human_grantable: bool
    grant_id: str = ''
    grant_expires_at: float = 0.0


@dataclass
class _Track:
    supported_since: float = None   # start of the current continuous "evidence supports it" stretch
    state: str = None
    provenance: str = None
    since: float = 0.0
    changed_rev: int = 0
    seen: bool = False
    contradicted: bool = False
    ever_available: bool = False    # restore dwell applies only to a permission that was available before


@dataclass
class Result:
    permissions: list
    changed: bool
    causes: list = field(default_factory=list)


class AuthorityCore:
    def __init__(self, rules, mode='lpl'):
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.rules = rules
        self.mode = mode
        self.ttl_default = float(rules.get('evidence_ttl_default_s', 1.0))
        self.perms = rules['permissions']
        self.track = {pid: _Track() for pid in self.perms}
        self.grants = {}           # permission id -> active Grant
        self.grant_log = []        # ended grants, for audit
        self.revision = 0
        self._grant_seq = 0
        self.last = None

    # ------------------------------------------------------------------ evidence
    def _evidence_status(self, now, evidence, ids):
        """-> (supported, stale, contradicted, text) for the evidence a permission needs."""
        supported, stale, contradicted, parts = True, False, False, []
        for eid in ids:
            ev = evidence.get(eid)
            if ev is None or now > ev.observed_at + (ev.ttl_s or self.ttl_default):
                supported, stale = False, True
                parts.append(f'{eid} missing or stale')
                continue
            if ev.level != ADEQUATE:
                supported = False
            contradicted |= ev.level == CONTRADICTED
            parts.append(f'{eid} {ev.level}' + (f' ({ev.reason})' if ev.reason else ''))
        return supported, stale, contradicted, '; '.join(parts)

    # ------------------------------------------------------------------ main update
    def update(self, now, evidence, inside):
        """evidence: {id: Evidence}; inside: {region: bool}. Returns Result."""
        out, causes = [], []
        if self.mode == 'global_revoke':
            bad = [f'{eid} {ev.level}' for eid, ev in evidence.items()
                   if ev.level != ADEQUATE or now > ev.observed_at + (ev.ttl_s or self.ttl_default)]
        for pid, rule in self.perms.items():
            ids = rule.get('requires', [])
            supported, stale, contradicted, ev_text = self._evidence_status(now, evidence, ids)
            region = rule.get('region', '')
            t = self.track[pid]
            t.contradicted = contradicted

            if self.mode == 'permissive':
                state, prov, reason = AVAILABLE, FIXED, 'permissive mode: evidence ignored (no LPL)'
            elif self.mode == 'global_revoke':
                ok = not bad
                state, prov = (AVAILABLE, EVIDENCE) if ok else (UNAVAILABLE, EVIDENCE)
                reason = 'all evidence adequate' if ok else 'global revoke: ' + ', '.join(bad)
            else:
                state, prov, reason = self._lpl_state(now, pid, rule, t, supported, stale, contradicted,
                                                      ev_text, inside.get(region, False))
            if not t.seen or (state, prov) != (t.state, t.provenance):
                if t.seen:
                    causes.append(f'{pid} {t.state} -> {state}: {reason}')
                t.state, t.provenance, t.since, t.seen = state, prov, now, True
                t.changed_rev = self.revision + 1
            g = self.grants.get(pid)
            out.append(Permission(
                id=pid, name=rule.get('name', pid), region=region, state=state,
                evidence_supported=supported, provenance=prov, evidence_ids=list(ids), reason=reason,
                since=t.since, human_grantable=bool(rule.get('human_grantable', False)),
                grant_id=g.id if g else '', grant_expires_at=g.expires_at if g and g.phase == 'ACTIVE_UNUSED' else 0.0))
        changed = bool(causes) or self.last is None
        if changed:
            self.revision += 1
        self.last = out
        return Result(out, changed, causes)

    def _lpl_state(self, now, pid, rule, t, supported, stale, contradicted, ev_text, inside):
        # restore dwell: after a revocation, evidence must hold continuously for restore_dwell_s
        # before the permission comes back (fast revoke, slow restore). The first grant at
        # startup has no dwell: nothing is being restored.
        if supported:
            if t.supported_since is None:
                t.supported_since = now if t.ever_available else float('-inf')
        else:
            t.supported_since = None
        dwell = float(rule.get('restore_dwell_s', 0.0))
        evidence_ok = supported and now - t.supported_since >= dwell
        t.ever_available |= evidence_ok

        self._update_grant(now, pid, rule, evidence_ok, contradicted, inside)
        if evidence_ok:
            return AVAILABLE, EVIDENCE, ev_text
        if pid in self.grants:
            return HUMAN_GRANTED, HUMAN_GRANT, f'authorized by operator (single pass); {ev_text}'
        if (inside and rule.get('on_revoke_inside_region') == 'EGRESS'
                and t.state in (AVAILABLE, HUMAN_GRANTED, EGRESS_ONLY)):
            return EGRESS_ONLY, (STALE if stale else EVIDENCE), f'lost while inside the region: leave only; {ev_text}'
        if stale:
            return UNAVAILABLE, STALE, ev_text
        if supported:
            left = dwell - (now - t.supported_since)
            return UNAVAILABLE, DWELL_PENDING, f'{ev_text}; restoring in {left:.1f} s'
        return UNAVAILABLE, EVIDENCE, ev_text

    def _update_grant(self, now, pid, rule, evidence_ok, contradicted, inside):
        g = self.grants.get(pid)
        if g is None:
            return
        cfg = rule.get('grant', {})
        end = None
        if contradicted and cfg.get('cancel_on_contradiction', True):
            end = 'CANCELLED'          # evidence now actively against it
        elif evidence_ok:
            end = 'SUPERSEDED'         # evidence supports it again; no longer needs the human
        elif g.phase == 'ACTIVE_UNUSED' and inside:
            g.phase = 'IN_USE'
        elif g.phase == 'ACTIVE_UNUSED' and now > g.expires_at:
            end = 'EXPIRED'
        elif g.phase == 'IN_USE' and not inside and cfg.get('single_pass', True):
            end = 'CONSUMED'
        if end:
            self._end_grant(pid, end, now)

    def _end_grant(self, pid, how, now):
        g = self.grants.pop(pid)
        g.phase = how
        self.grant_log.append((now, g))

    # ------------------------------------------------------------------ operator
    def request_grant(self, now, pid, for_option, ttl_s, revision_seen):
        """-> (accepted, reason, grant_id, expires_at)"""
        rule = self.perms.get(pid)
        if rule is None or not rule.get('human_grantable') or self.mode != 'lpl':
            return False, 'NOT_GRANTABLE', '', 0.0
        t = self.track[pid]
        if t.state == AVAILABLE:
            return False, 'NOT_NEEDED', '', 0.0
        if pid in self.grants:
            return False, 'ALREADY_GRANTED', self.grants[pid].id, self.grants[pid].expires_at
        if revision_seen < t.changed_rev:
            return False, 'STALE_CONSENT', '', 0.0      # the operator saw an older state of this permission
        if t.contradicted:
            return False, 'CONTRADICTED', '', 0.0      # evidence actively says no
        max_unused = float(rule.get('grant', {}).get('max_unused_s', 60.0))
        self._grant_seq += 1
        g = Grant(id=f'G{self._grant_seq}', permission=pid, for_option=for_option, granted_at=now,
                  expires_at=now + min(ttl_s, max_unused) if ttl_s > 0 else now + max_unused)
        self.grants[pid] = g
        return True, 'GRANTED', g.id, g.expires_at

    def revoke_grant(self, now, pid):
        if pid not in self.grants:
            return False, 'NO_ACTIVE_GRANT'
        self._end_grant(pid, 'REVOKED', now)
        return True, 'REVOKED'
