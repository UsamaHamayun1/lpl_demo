# Available actions, selection and escalation (the planner side, downstream of LPL).
#
# Pure Python, no ROS. Per cycle it judges every predefined option against:
#   reachable  - the robot has not passed the option's commit point
#   possible   - physically feasible on the robot's own (physical-only) map
#   evidenced  - every permission it needs is supported by evidence
#   authorized - every permission it needs is usable now (evidence or human grant)
#   adequate   - all of the above and it makes progress within the mission's bound
# then picks the fastest adequate option ("the AI chooses the move"), and decides
# whether the human is needed. It reads permissions only through the authority
# envelope: no evidence ids, no rules, no LPL mode.
from dataclasses import dataclass, field

from lpl2_core.options import in_box, length, project, regions_touched

USABLE = ('AVAILABLE', 'HUMAN_GRANTED', 'EGRESS_ONLY')
NONE, ADVISORY, HJR, FAULT = 'NONE', 'ADVISORY', 'HUMAN_JUDGMENT_REQUIRED', 'SYSTEM_FAULT'


@dataclass
class PermView:
    """What the planner may know about a permission (from AuthorityState)."""
    id: str
    region: str
    state: str
    evidence_supported: bool
    by_evidence: bool          # provenance is EVIDENCE (not dwell/stale/human/fixed)
    human_grantable: bool


@dataclass
class Status:
    id: str
    name: str
    hold: bool
    requires: list = field(default_factory=list)
    missing: list = field(default_factory=list)
    reachable: bool = True
    possible: bool = True
    possible_reason: str = ''
    evidenced: bool = True
    authorized: bool = True
    adequate: bool = False
    inadequate_reason: str = ''
    selected: bool = False
    egress: bool = False
    eta_s: float = -1.0
    remaining_m: float = 0.0
    remaining_path: list = field(default_factory=list)


@dataclass
class Escalation:
    level: str = NONE
    kind: str = 'NONE'
    headline: str = ''
    lines: list = field(default_factory=list)
    asks: list = field(default_factory=list)
    advised: str = ''
    gain_s: float = 0.0


class Decider:
    def __init__(self, options, regions, mission, safe_hold=None):
        self.options = options
        self.regions = regions
        self.m = mission
        self.speed = mission['speed_mps']
        self.selected = None
        self.empty_since = None
        self.feas = {o.id: {'bad': 0, 'good': 0, 'blocked': False} for o in options}
        self.feas['E'] = {'bad': 0, 'good': 0, 'blocked': False}
        safe_hold = safe_hold or {}
        self.safe_regions = set(safe_hold.get('regions', []))
        t = safe_hold.get('egress_targets')
        self.egress_targets = []
        if t:
            x = t['x_from']
            while x <= t['x_to'] + 1e-9:
                if not any(a <= x <= b for a, b in t.get('exclude_x', [])):
                    self.egress_targets.append((x, t['y']))
                x += t['step']

    def safe_to_hold(self, x, y):
        """Inside a safe-hold region and not inside any other permission region (e.g. Lane 1)."""
        if not self.safe_regions:
            return True
        inside = {n for n in self.regions.regions if self.regions.inside(n, x, y)}
        return bool(inside & self.safe_regions) and not (inside - self.safe_regions)

    def _egress(self, x, y):
        """Option E: straight back to the nearest safe-hold point."""
        tx, ty = min(self.egress_targets, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
        n = max(2, int(((tx - x) ** 2 + (ty - y) ** 2) ** 0.5 / 0.1))
        path = [(x + (tx - x) * i / n, y + (ty - y) * i / n) for i in range(n + 1)]
        s = Status('E', 'Return to own lane', False, egress=True, remaining_path=path)
        s.remaining_m = length(path)
        s.eta_s = s.remaining_m / self.speed
        f = self.feas['E']
        s.possible = not f['blocked']
        s.possible_reason = '' if s.possible else 'way back blocked on the robot\'s map'
        s.adequate = s.possible
        s.inadequate_reason = '' if s.possible else 'not possible'
        return s

    # ---------------------------------------------------------------- feasibility input
    def report_feasibility(self, option_id, valid):
        """Called with each /feasibility/is_path_valid result; latches BLOCKED with hysteresis."""
        f = self.feas[option_id]
        if valid:
            f['good'], f['bad'] = f['good'] + 1, 0
            if f['blocked'] and f['good'] >= self.m['feasibility']['clear_after']:
                f['blocked'] = False
        else:
            f['bad'], f['good'] = f['bad'] + 1, 0
            if f['bad'] >= self.m['feasibility']['blocked_after']:
                f['blocked'] = True

    # ---------------------------------------------------------------- one cycle
    def evaluate(self, now, pose, perms, authority_fresh, mission_done):
        """pose (x, y) or None; perms: {id: PermView}. -> (statuses, selected_id, reason, Escalation)"""
        by_region = {p.region: p for p in perms.values() if p.region}
        out = []
        for o in self.options:
            s = Status(o.id, o.name, o.hold)
            if o.hold:
                s.inadequate_reason = 'makes no progress'
                out.append(s)
                continue
            x, y = pose
            if any(in_box(b, x, y) for b in o.unreachable_inside):
                s.reachable = False
                s.remaining_path = o.path
                s.remaining_m = length(o.path)
            else:
                i, d = project(o.path, x, y)
                s.remaining_path = o.path[i:]
                s.remaining_m = length(s.remaining_path) + d
            s.eta_s = s.remaining_m / self.speed
            need = sorted((by_region[r] for r in regions_touched(s.remaining_path, self.regions) if r in by_region),
                          key=lambda p: p.id)
            s.requires = [p.id for p in need]
            s.missing = [p.id for p in need if p.state not in USABLE or not authority_fresh]
            s.evidenced = all(p.evidence_supported for p in need)
            s.authorized = authority_fresh and not s.missing
            f = self.feas[o.id]
            s.possible = not f['blocked']
            s.possible_reason = '' if s.possible else 'path blocked on the robot\'s map (lidar)'
            if not s.reachable:
                s.inadequate_reason = 'past its commit point'
            elif not s.possible:
                s.inadequate_reason = 'not possible'
            elif not s.authorized:
                s.inadequate_reason = 'not authorized: needs ' + ', '.join(s.missing)
            elif s.eta_s > self.m['max_eta_s']:
                s.inadequate_reason = 'too slow for the mission'
            else:
                s.adequate = True
            out.append(s)

        egress = None
        if (not mission_done and self.egress_targets and not any(s.adequate for s in out)
                and not self.safe_to_hold(*pose)):
            egress = self._egress(*pose)           # never stop outside a safe-hold area
            out.append(egress)
        if egress is not None and egress.adequate:
            sel, reason = 'E', 'no adequate option and not safe to stop here: returning to own lane first'
            self.selected = sel
            self.empty_since = None
            for s in out:
                s.selected = s.id == sel
            return out, sel, reason, Escalation()
        sel, reason = self._select(out, mission_done)
        for s in out:
            s.selected = s.id == sel
        esc = self._escalation(now, out, perms, sel, authority_fresh, mission_done)
        return out, sel, reason, esc

    def _select(self, statuses, mission_done):
        hold = next(s for s in statuses if s.hold)
        if mission_done:
            self.selected = hold.id
            return hold.id, 'mission complete'
        adequate = sorted((s for s in statuses if s.adequate), key=lambda s: s.eta_s)
        if not adequate:
            self.selected = hold.id
            return hold.id, 'no adequate option: holding safely in own lane'
        best = adequate[0]
        current = next((s for s in adequate if s.id == self.selected), None)
        if current and current.id != best.id and current.eta_s - best.eta_s < self.m['selector']['hysteresis_s']:
            best = current
        others = ', '.join(f'{s.id} {s.inadequate_reason}' for s in statuses if not s.adequate and not s.hold)
        reason = f'fastest adequate option ({best.eta_s:.0f} s)' + (f'; {others}' if others else '')
        self.selected = best.id
        return best.id, reason

    def _escalation(self, now, statuses, perms, sel, fresh, mission_done):
        e = Escalation()
        if mission_done:
            self.empty_since = None
            return e
        if not fresh:
            e.level, e.kind, e.headline = FAULT, 'LPL_STALE', 'authority missing or stale: nothing is authorized'
            return e
        routes = [s for s in statuses if not s.hold and not s.egress]
        adequate = [s for s in routes if s.adequate]
        # options a human grant could make executable: reachable, possible, missing only grantable permissions
        grantable = [s for s in routes if s.reachable and s.possible and not s.authorized and s.missing
                     and all(perms[p].human_grantable for p in s.missing)]
        grantable.sort(key=lambda s: s.eta_s)

        if not adequate:
            if self.empty_since is None:
                self.empty_since = now
            if now - self.empty_since < self.m['t_confirm_s']:
                return e                                 # confirming: no flicker on transient states
            e.level, e.headline = HJR, 'HUMAN JUDGMENT REQUIRED'
            e.lines = [self._line(s) for s in statuses]
            if grantable:
                e.kind, e.asks, e.advised = 'AUTHORITY_EXHAUSTED', list(grantable[0].missing), grantable[0].id
            else:
                e.kind = 'NO_FEASIBLE'
            return e
        self.empty_since = None

        adv = self.m.get('advisory', {})
        current = next(s for s in statuses if s.id == sel)
        if adv.get('enabled') and grantable:
            x = grantable[0]
            by_evidence = all(perms[p].state == 'UNAVAILABLE' and perms[p].by_evidence for p in x.missing)
            gain = current.eta_s - x.eta_s
            if (by_evidence and gain >= adv['min_gain_s'] and x.eta_s <= (1 - adv['min_gain_frac']) * current.eta_s):
                e.level, e.kind, e.headline = ADVISORY, 'BETTER_OPTION_NEEDS_PERMISSION', 'Better option available'
                e.asks, e.advised, e.gain_s = list(x.missing), x.id, gain
                e.lines = [f'{x.id}: {x.eta_s:.0f} s, possible, needs {", ".join(x.missing)}',
                           f'{current.id}: {current.eta_s:.0f} s, fully authorized (selected)']
        return e

    @staticmethod
    def _line(s):
        if s.hold:
            return f'{s.id}: authorized · makes no progress'
        if not s.reachable:
            return f'{s.id}: past its commit point'
        parts = ['possible' if s.possible else f'NOT possible ({s.possible_reason})',
                 'evidenced' if s.evidenced else 'not evidenced',
                 'authorized' if s.authorized else f'not authorized (needs {", ".join(s.missing)})']
        return f'{s.id}: ' + ' · '.join(parts)
