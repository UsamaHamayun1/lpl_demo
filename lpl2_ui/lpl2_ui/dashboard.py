#!/usr/bin/env python3
# LPL 2.0 mission control dashboard (PyQt5).
#
# Shows the whole chain at once:  Evidence -> Permissions (LPL) -> Options (planner)
# -> Execution, plus escalation. The operator can authorize a permission without
# taking manual control (SPACE), revoke it (BACKSPACE), e-stop (ESC), and, as a last
# resort, take over manually (M, then WASD).
#
#   ros2 run lpl2_ui dashboard
import os
import queue
import signal
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np
from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QFrame, QGridLayout, QHBoxLayout,
                             QHeaderView, QLabel, QPlainTextEdit, QPushButton, QSizePolicy,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from lpl2_interfaces.msg import Escalation, EvidenceItem, ExecutionState, GateStatus, OptionStatus, PermissionState
from lpl2_sim.geometry import Corridor, default_path
from lpl2_ui.ros_bridge import EVIDENCE_LEVEL, PERMISSION_STATE, RosBridge, stamp_s
from lpl2_ui.topdown import Projection, draw_overlay

REFRESH_MS = 66
GREEN, RED, AMBER, ORANGE, GREY, PURPLE, BLUE = (
    '#28a745', '#d32f2f', '#ffb300', '#fd7e14', '#6c757d', '#8e44ad', '#1e88e5')
PERMISSION_COLOR = {PermissionState.AVAILABLE: GREEN, PermissionState.UNAVAILABLE: RED,
                    PermissionState.HUMAN_GRANTED: AMBER, PermissionState.EGRESS_ONLY: ORANGE}
PROVENANCE = {PermissionState.PROV_EVIDENCE: 'by evidence', PermissionState.PROV_HUMAN_GRANT: 'by human',
              PermissionState.PROV_DWELL_PENDING: 'restoring', PermissionState.PROV_STALE: 'evidence stale',
              PermissionState.PROV_FIXED: 'fixed'}
EXECUTION = {ExecutionState.IDLE: 'IDLE', ExecutionState.EXECUTING: 'EXECUTING',
             ExecutionState.HOLDING: 'HOLDING', ExecutionState.EGRESS: 'EGRESS',
             ExecutionState.SUCCEEDED: 'SUCCEEDED', ExecutionState.FAILED: 'FAILED',
             ExecutionState.REFUSED: 'REFUSED'}
GATE = {GateStatus.PASS: 'PASS', GateStatus.BLOCK_PERMISSION: 'BLOCKED (permission)',
        GateStatus.BLOCK_LEASE: 'BLOCKED (authority stale)', GateStatus.WATCHDOG: 'WATCHDOG',
        GateStatus.ESTOP: 'E-STOP', GateStatus.MANUAL: 'MANUAL', GateStatus.EGRESS: 'EGRESS'}
TELEOP_KEYS = {Qt.Key_W: (0.22, 0.0), Qt.Key_S: (-0.15, 0.0), Qt.Key_A: (0.0, 0.8), Qt.Key_D: (0.0, -0.8)}
ADVISORY_TEXT = 'ADVISORY: Better option available — requires {} authorization.'
STALE_AFTER = 1.5        # s without LPL messages -> show as stale (nothing is authorized)

CARD = 'background-color: #1e1e1e; border-radius: 8px;'
TITLE = 'color: #888; font-weight: bold; font-size: 11px; letter-spacing: 1px;'


class WindowRecorder:
    """Writes dashboard frames to mp4 on a background thread (H.264 via GStreamer, else mp4v)."""

    def __init__(self, path, fps, size):
        self.size = size
        pipe = ('appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=veryfast bitrate=8000 '
                f'! mp4mux ! filesink location={path}')
        self.writer = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, fps, size, True)
        if not self.writer.isOpened():
            self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        self.q = queue.Queue(maxsize=120)
        self.frames = 0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def push(self, bgr):
        try:
            self.q.put_nowait(bgr)
        except queue.Full:
            pass                            # encoder behind: drop rather than stall the UI

    def _run(self):
        while True:
            frame = self.q.get()
            if frame is None:
                break
            if (frame.shape[1], frame.shape[0]) != self.size:
                frame = cv2.resize(frame, self.size)
            self.writer.write(frame)
            self.frames += 1

    def close(self):
        self.q.put(None)
        self.thread.join(timeout=10)
        self.writer.release()


def tick(ok):
    return ('✓', GREEN) if ok else ('✗', RED)


def to_qimage(msg):
    fmt = QImage.Format_RGB888 if msg.encoding in ('rgb8', 'bgr8') else QImage.Format_Grayscale8
    img = QImage(bytes(msg.data), msg.width, msg.height, msg.step, fmt).copy()
    return img.rgbSwapped() if msg.encoding == 'bgr8' else img


def section(title):
    lbl = QLabel(title)
    lbl.setStyleSheet(TITLE)
    return lbl


def table(headers, stretch_col):
    t = QTableWidget(0, len(headers))
    t.setHorizontalHeaderLabels(headers)
    t.verticalHeader().setVisible(False)
    t.setEditTriggers(QAbstractItemView.NoEditTriggers)
    t.setSelectionMode(QAbstractItemView.NoSelection)
    t.setFocusPolicy(Qt.NoFocus)
    t.setShowGrid(False)
    t.setWordWrap(True)
    t.setStyleSheet('QTableWidget { background-color: #1e1e1e; border: none; font-size: 13px; }'
                    'QHeaderView::section { background-color: #262626; color: #aaa; border: none; '
                    'padding: 4px; font-weight: bold; }')
    h = t.horizontalHeader()
    for i in range(len(headers)):
        h.setSectionResizeMode(i, QHeaderView.Stretch if i == stretch_col else QHeaderView.ResizeToContents)
    t.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
    return t


def set_rows(t, rows):
    """rows: list of lists of (text, color or None, bold)."""
    t.setRowCount(len(rows))
    for r, row in enumerate(rows):
        for c, (text, color, bold) in enumerate(row):
            item = QTableWidgetItem(text)
            if color:
                item.setForeground(QColor(color))
            if bold:
                f = item.font()
                f.setBold(True)
                item.setFont(f)
            t.setItem(r, c, item)
    t.resizeRowsToContents()
    t.setFixedHeight(t.horizontalHeader().height() + sum(t.rowHeight(i) for i in range(len(rows))) + 4)


class Dashboard(QWidget):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.corridor = Corridor(default_path())
        self.proj = Projection(self.corridor.cameras['topdown'])
        self.estop = False
        self.manual = False
        self.held = set()
        self.blink = False
        self.view = {}
        self.build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(REFRESH_MS)
        self.recorder = None
        self.last_td_stamp = None
        self.teleop_timer = QTimer(self)
        self.teleop_timer.timeout.connect(self.send_teleop)
        self.teleop_timer.start(100)

    # ------------------------------------------------------------------ layout
    def build_ui(self):
        self.setWindowTitle('LPL 2.0 MISSION CONTROL')
        self.resize(1680, 980)
        self.setStyleSheet('background-color: #121212; color: #e0e0e0; font-family: "DejaVu Sans";')
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        root.setSpacing(10)

        # header
        head = QHBoxLayout()
        title = QLabel('LPL 2.0  MISSION CONTROL')
        title.setFont(QFont('DejaVu Sans', 16, QFont.Bold))
        head.addWidget(title)
        self.lbl_mode = QLabel('')
        self.lbl_mode.setStyleSheet('padding: 2px 10px; border-radius: 4px; font-weight: bold;')
        head.addWidget(self.lbl_mode)
        head.addStretch()
        self.lbl_clock = QLabel('t = --')
        self.lbl_clock.setFont(QFont('DejaVu Sans Mono', 13))
        head.addWidget(self.lbl_clock)
        root.addLayout(head)

        body = QHBoxLayout()
        body.setSpacing(14)
        root.addLayout(body, 1)

        # left: cameras
        left = QVBoxLayout()
        left.addWidget(section('ROBOT R CAMERA — what the robot can see'))
        self.lbl_fpv = self.video_label(560, 300)
        left.addWidget(self.lbl_fpv, 5)
        left.addWidget(section('TOP-DOWN — ground truth (operator view, not robot knowledge)'))
        self.lbl_top = self.video_label(560, 180)
        left.addWidget(self.lbl_top, 4)
        body.addLayout(left, 11)

        # right: decision chain
        right = QVBoxLayout()
        right.setSpacing(8)
        self.lbl_banner = QLabel('WAITING FOR LPL')
        self.lbl_banner.setAlignment(Qt.AlignCenter)
        self.lbl_banner.setWordWrap(True)
        self.lbl_banner.setFont(QFont('DejaVu Sans', 17, QFont.Bold))
        self.lbl_banner.setMinimumHeight(56)
        right.addWidget(self.lbl_banner)
        self.lbl_cause = QLabel('')
        self.lbl_cause.setWordWrap(True)
        self.lbl_cause.setStyleSheet('color: #bbb; font-size: 12px; padding: 0 4px;')
        right.addWidget(self.lbl_cause)

        self.lbl_mission = QLabel('')
        self.lbl_mission.setWordWrap(True)
        self.lbl_mission.setStyleSheet(CARD + 'padding: 8px; font-size: 13px;')
        right.addWidget(self.lbl_mission)

        right.addWidget(section('1  EVIDENCE — what the robot has evidence for'))
        self.tbl_evidence = table(['Evidence', 'Level', 'Detail', 'Source'], 2)
        right.addWidget(self.tbl_evidence)

        right.addWidget(section('2  AUTHORITY (LPL) — which decisions still belong to the machine'))
        self.tbl_perm = table(['Permission', 'State', 'Basis', 'Reason'], 3)
        right.addWidget(self.tbl_perm)

        right.addWidget(section('3  DECISION SPACE (planner) — options for the same mission'))
        self.tbl_opt = table(['', 'Option', 'Possible', 'Evidenced', 'Authorized', 'Adequate', 'ETA', 'Note'], 7)
        right.addWidget(self.tbl_opt)

        # escalation / operator
        esc = QFrame()
        esc.setStyleSheet(CARD)
        el = QVBoxLayout(esc)
        el.setContentsMargins(10, 8, 10, 8)
        self.lbl_esc = QLabel('No human input needed.')
        self.lbl_esc.setWordWrap(True)
        self.lbl_esc.setStyleSheet('font-size: 13px;')
        el.addWidget(self.lbl_esc)
        buttons = QHBoxLayout()
        self.btn_grant = self.button('AUTHORIZE  [SPACE]', '#444', self.grant)
        self.btn_revoke = self.button('REVOKE GRANT  [BACKSPACE]', '#444', self.revoke)
        self.btn_estop = self.button('E-STOP  [ESC]', RED, self.estop_button)
        for b in (self.btn_grant, self.btn_revoke, self.btn_estop):
            buttons.addWidget(b)
        el.addLayout(buttons)
        right.addWidget(esc)

        right.addWidget(section('EVENT LOG (sim time)'))
        self.txt_log = QPlainTextEdit()
        self.txt_log.setMinimumHeight(90)
        self.txt_log.setReadOnly(True)
        self.txt_log.setFocusPolicy(Qt.NoFocus)
        self.txt_log.setStyleSheet('background-color: #1e1e1e; border: none; font-family: "DejaVu Sans Mono"; '
                                   'font-size: 12px;')
        right.addWidget(self.txt_log, 1)

        foot = QGridLayout()
        self.lbl_counters = QLabel('')
        self.lbl_counters.setStyleSheet('font-size: 13px; font-weight: bold;')
        foot.addWidget(self.lbl_counters, 0, 0)
        self.lbl_gate = QLabel('')
        self.lbl_gate.setStyleSheet('font-size: 12px; color: #aaa;')
        foot.addWidget(self.lbl_gate, 1, 0)
        keys = QLabel('SPACE authorize · BACKSPACE revoke · ESC e-stop (release with the button) · '
                      'M manual takeover (WASD) — last resort')
        keys.setStyleSheet('font-size: 11px; color: #777;')
        foot.addWidget(keys, 2, 0)
        right.addLayout(foot)

        rw = QWidget()
        rw.setLayout(right)
        rw.setMinimumWidth(720)
        body.addWidget(rw, 9)

    def video_label(self, w, h):
        lbl = QLabel('NO SIGNAL')
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumSize(w, h)
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lbl.setStyleSheet('background-color: black; border: 1px solid #333; color: #555;')
        return lbl

    def button(self, text, color, slot):
        b = QPushButton(text)
        b.setFocusPolicy(Qt.NoFocus)
        b.setFixedHeight(44)
        b.setFont(QFont('DejaVu Sans', 11, QFont.Bold))
        b.setStyleSheet(f'background-color: {color}; color: white; border-radius: 5px;')
        b.clicked.connect(slot)
        return b

    # ------------------------------------------------------------------ refresh
    def refresh(self):
        latest, log, counters, now = self.bridge.snapshot()
        self.view = latest
        self.blink = int(time.monotonic() * 2) % 2 == 0          # 1 Hz, not a strobe
        authority, decision, escalation = latest.get('authority'), latest.get('decision'), latest.get('escalation')
        evidence, execution, gate = latest.get('evidence'), latest.get('execution'), latest.get('gate')
        if self.bridge.estop_latched is not None:
            self.estop = self.bridge.estop_latched                # survives a dashboard restart
        stale = authority is not None and now - latest.get('authority_at', now) > STALE_AFTER

        self.lbl_clock.setText(f't = {now:7.1f} s')
        self.update_mode(authority)
        self.update_banner(authority, decision, escalation, execution, gate, stale)
        self.lbl_cause.setText(f'Why: {authority.cause}' if authority and authority.cause else '')
        self.update_mission(decision, execution)
        self.update_evidence(evidence, now)
        self.update_permissions(authority)
        self.update_options(decision)
        self.update_escalation(escalation, authority)
        self.update_log(log)
        self.lbl_counters.setText(
            f"Human authorizations {counters['authorizations']}   ·   Takeovers {counters['takeovers']}   ·   "
            f"Judgment requests {counters['judgment_requests']}   ·   E-stops {counters['estops']}")
        if gate:
            self.lbl_gate.setText(f'Execution gate: {GATE.get(gate.state, "?")}  {gate.reason}   '
                                  f'cmd {gate.linear:+.2f} m/s {gate.angular:+.2f} rad/s')
        self.update_video(latest, authority, decision)
        self.record_frame(latest)

    def update_mode(self, authority):
        if authority is None:
            self.lbl_mode.setText('NO LPL')
            self.lbl_mode.setStyleSheet(self.lbl_mode.styleSheet() + f'background-color: {GREY};')
            return
        mode = authority.mode.upper()
        color = {'LPL': GREEN, 'MOCK': AMBER}.get(mode, ORANGE)
        text = 'MOCK DATA — not the real LPL' if mode == 'MOCK' else f'MODE {mode}'
        self.lbl_mode.setText(f'{text}   rules {authority.rules_id}   rev {authority.revision}')
        self.lbl_mode.setStyleSheet('padding: 2px 10px; border-radius: 4px; font-weight: bold; '
                                    f'background-color: {color}; color: black;')

    def set_banner(self, text, bg, fg='white'):
        self.lbl_banner.setText(text)
        self.lbl_banner.setStyleSheet(f'background-color: {bg}; color: {fg}; border-radius: 8px; padding: 6px;')

    def update_banner(self, authority, decision, escalation, execution, gate, stale):
        # what the robot is actually doing comes from the execution gate, not local key state
        if (gate and gate.state == GateStatus.ESTOP) or self.estop:
            return self.set_banner('E-STOP ACTIVE', RED)
        if (gate and gate.state == GateStatus.MANUAL) or self.manual:
            return self.set_banner('MANUAL TAKEOVER — operator driving', ORANGE, 'black')
        if authority is None:
            return self.set_banner('WAITING FOR LPL', '#333', '#aaa')
        if stale:
            return self.set_banner('LPL DATA STALE — nothing is authorized, robot held', PURPLE)
        level = escalation.level if escalation else Escalation.NONE
        if level == Escalation.SYSTEM_FAULT:
            return self.set_banner(f'SYSTEM FAULT — {escalation.headline}', PURPLE)
        if level == Escalation.HUMAN_JUDGMENT_REQUIRED:
            return self.set_banner('HUMAN JUDGMENT REQUIRED', RED if self.blink else '#8b1a1a')
        if decision and decision.selection_reason == 'mission complete':
            return self.set_banner('MISSION COMPLETE', BLUE)
        selected = self.selected_option(decision)
        name = f'OPTION {selected.id} — {selected.name}' if selected else '—'
        human = [p.id for p in authority.permissions if p.state == PermissionState.HUMAN_GRANTED and selected
                 and p.id in selected.requires]
        if level == Escalation.ADVISORY:
            return self.set_banner(f'AUTONOMOUS · {name}\n' + ADVISORY_TEXT.format(', '.join(escalation.asks)),
                                   AMBER, 'black')
        if human:
            return self.set_banner(f'AUTONOMOUS · {name}\nrunning on HUMAN authority for {", ".join(human)} '
                                   f'(not supported by evidence)', '#b8860b')
        return self.set_banner(f'AUTONOMOUS · {name}', GREEN)

    @staticmethod
    def selected_option(decision):
        if decision is None:
            return None
        return next((o for o in decision.options if o.id == decision.selected_id), None)

    def update_mission(self, decision, execution):
        if decision is None:
            self.lbl_mission.setText('<b>MISSION</b>  waiting for the planner')
            return
        g = decision.goal
        sel = self.selected_option(decision)
        if not hasattr(self, 'first_goal'):
            self.first_goal = (decision.mission_id, round(g.x, 2), round(g.y, 2))
        same = self.first_goal == (decision.mission_id, round(g.x, 2), round(g.y, 2))
        tag = (f'<span style="color:{GREEN}">— same mission since start</span>' if same
               else f'<span style="color:{RED}">— MISSION CHANGED</span>')
        lines = [f'<b>MISSION</b> {decision.mission_id}: Robot R → goal ({g.x:.1f}, {g.y:.1f}) {tag}']
        if sel and sel.kind == OptionStatus.HOLD:
            lines.append(f'<b>Selected:</b> hold in place · <i>{decision.selection_reason}</i>')
        elif sel:
            eta = f'{sel.eta_s:.0f} s' if sel.eta_s >= 0 else '—'
            lines.append(f'<b>Selected:</b> option {sel.id} ({sel.name}) · {sel.remaining_m:.1f} m left · '
                         f'ETA {eta} · <i>{decision.selection_reason}</i>')
        if execution:
            lines.append(f'<b>Execution:</b> {EXECUTION.get(execution.state, "?")} {execution.option_id} '
                         f'{execution.detail}')
        self.lbl_mission.setText('<br>'.join(lines))

    def update_evidence(self, msg, now):
        rows = []
        for it in (msg.items if msg else []):
            stale = stamp_s(it.observed_at) + it.ttl_s < now and it.ttl_s > 0
            level = 'STALE' if stale else EVIDENCE_LEVEL.get(it.level, '?')
            color = {EvidenceItem.ADEQUATE: GREEN, EvidenceItem.INSUFFICIENT: RED,
                     EvidenceItem.CONTRADICTED: PURPLE}.get(it.level, GREY)
            rows.append([(it.id, None, True), (level, GREY if stale else color, True),
                         (it.detail or it.proposition, None, False),
                         ('SCRIPTED' if it.scripted else it.source, AMBER if it.scripted else None, it.scripted)])
        set_rows(self.tbl_evidence, rows)

    def update_permissions(self, msg):
        rows = []
        for p in (msg.permissions if msg else []):
            state = PERMISSION_STATE.get(p.state, '?')
            basis = PROVENANCE.get(p.provenance, '?')
            if p.state == PermissionState.HUMAN_GRANTED and not p.evidence_supported:
                basis = 'by human · not evidenced'
            rows.append([(f'{p.id} {p.name}', None, True), (state, PERMISSION_COLOR.get(p.state), True),
                         (basis, AMBER if p.provenance == PermissionState.PROV_HUMAN_GRANT else None, False),
                         (p.reason, None, False)])
        set_rows(self.tbl_perm, rows)

    def update_options(self, msg):
        rows = []
        for o in (msg.options if msg else []):
            note = o.inadequate_reason or o.possible_reason
            if not o.authorized and o.missing:
                note = f"needs {', '.join(o.missing)}" + (f' · {note}' if note else '')
            if not o.reachable:
                note = 'no longer reachable' + (f' · {note}' if note else '')
            eta = f'{o.eta_s:.0f} s' if o.eta_s >= 0 else '—'
            row = [('▶' if o.selected else '', GREEN, True), (f'{o.id}  {o.name}', None, o.selected)]
            for ok in (o.possible, o.evidenced, o.authorized, o.adequate):
                row.append((*tick(ok), True))
            row += [(eta, None, False), (note, None, False)]
            rows.append(row)
        set_rows(self.tbl_opt, rows)

    def update_escalation(self, esc, authority):
        asks = list(esc.asks) if esc else []
        if esc and esc.level != Escalation.NONE:
            color = {Escalation.ADVISORY: AMBER, Escalation.HUMAN_JUDGMENT_REQUIRED: RED,
                     Escalation.SYSTEM_FAULT: PURPLE}.get(esc.level, GREY)
            head = (ADVISORY_TEXT.format(', '.join(asks)) if esc.level == Escalation.ADVISORY
                    else esc.headline)
            lines = [f'<b style="color:{color}">{head}</b>'] + [f'· {line}' for line in esc.option_lines]
            for pid in asks:
                lines.append(f'<b>SPACE = you vouch:</b> {self.vouch_text(pid, authority)} (the robot cannot verify '
                             'it). It stays autonomous.')
            if esc.level == Escalation.ADVISORY:
                lines.append(f'If you do nothing, the robot carries on with its current option. '
                             f'Granting saves ≈{esc.eta_gain_s:.0f} s.')
            self.lbl_esc.setText('<br>'.join(lines))
        else:
            self.lbl_esc.setText('No human input needed. The robot is working within its current authority.')
        gate = self.view.get('gate')
        can_grant = bool(asks) and not self.estop and not (gate and gate.state == GateStatus.ESTOP)
        self.btn_grant.setEnabled(can_grant)
        self.btn_grant.setText(f"AUTHORIZE {' '.join(asks)}  [SPACE]" if asks else 'AUTHORIZE  [SPACE]')
        self.btn_grant.setStyleSheet(f'background-color: {AMBER if can_grant else "#333"}; '
                                     f'color: {"black" if can_grant else "#777"}; border-radius: 5px;')
        has_grant = any(p.grant_id for p in authority.permissions) if authority else False
        self.btn_revoke.setEnabled(has_grant)
        self.btn_revoke.setStyleSheet(f'background-color: {"#555" if has_grant else "#333"}; '
                                      f'color: {"white" if has_grant else "#777"}; border-radius: 5px;')
        self.btn_estop.setText('RELEASE E-STOP (click)' if self.estop else 'E-STOP  [ESC]')

    def vouch_text(self, pid, authority):
        """The evidence propositions a grant of `pid` stands in for, in words."""
        evidence = self.view.get('evidence')
        perm = next((p for p in authority.permissions if p.id == pid), None) if authority else None
        if perm is None:
            return pid
        props = {it.id: it.proposition for it in evidence.items} if evidence else {}
        return '; '.join(props.get(e, e) for e in perm.evidence_ids) or perm.name

    def update_log(self, log):
        text = '\n'.join(f'{t:7.1f}  {kind:<10} {msg}' for t, kind, msg in log[-60:])
        if text != self.txt_log.toPlainText():
            self.txt_log.setPlainText(text)
            self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def record_frame(self, latest):
        """One video frame per new top-down camera frame, so the video runs at sim-time speed."""
        if not self.bridge.record or 'topdown' not in latest:
            return
        stamp = stamp_s(latest['topdown'].header.stamp)
        if stamp == self.last_td_stamp:
            return
        self.last_td_stamp = stamp
        img = self.grab().toImage().convertToFormat(QImage.Format_RGB888)
        # GStreamer reads BGR rows padded to 4 bytes: keep width a multiple of 4, height even
        w, h = img.width() - img.width() % 4, img.height() - img.height() % 2
        if self.recorder is None:
            out = os.path.expanduser('~/lpl_recordings')
            os.makedirs(out, exist_ok=True)
            path = os.path.join(out, f"{self.bridge.scenario}_dashboard_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4")
            fps = float(self.corridor.cameras['topdown']['rate'])
            self.recorder = WindowRecorder(path, fps, (w, h))
            self.bridge.add_log('RECORD', f'recording to {path}')
        ptr = img.constBits()
        ptr.setsize(img.byteCount())
        rgb = np.frombuffer(ptr, np.uint8).reshape(img.height(), img.bytesPerLine())[:, :img.width() * 3]
        rgb = rgb.reshape(img.height(), img.width(), 3)[:h, :w]
        self.recorder.push(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def closeEvent(self, e):
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        super().closeEvent(e)

    def update_video(self, latest, authority, decision):
        if 'fpv' in latest:
            self.show_image(self.lbl_fpv, to_qimage(latest['fpv']))
        if 'topdown' in latest:
            img = to_qimage(latest['topdown'])
            draw_overlay(img, self.corridor, self.proj, authority, decision, latest.get('poses'))
            self.show_image(self.lbl_top, img)

    @staticmethod
    def show_image(label, img):
        label.setPixmap(QPixmap.fromImage(img).scaled(label.width(), label.height(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation))

    # ------------------------------------------------------------------ operator
    def grant(self):
        esc, authority = self.view.get('escalation'), self.view.get('authority')
        if self.estop:
            self.bridge.add_log('OPERATOR', 'SPACE ignored: e-stop is active')
            return
        if not esc or not esc.asks or authority is None:
            self.bridge.add_log('OPERATOR', 'SPACE: nothing to authorize right now')
            return
        self.bridge.request_grant(esc.asks[0], esc.advised_option, authority.revision)

    def revoke(self):
        authority = self.view.get('authority')
        granted = [p for p in authority.permissions if p.grant_id] if authority else []
        if not granted:
            self.bridge.add_log('OPERATOR', 'BACKSPACE: no active grant')
            return
        self.bridge.request_grant(granted[0].id, '', authority.revision, revoke=True)

    def engage_estop(self):
        if not self.estop:
            self.estop = True
            self.bridge.set_estop(True)

    def estop_button(self):
        """Button engages, or releases if engaged (release is deliberate: mouse only, not ESC)."""
        self.estop = not self.estop
        self.bridge.set_estop(self.estop)

    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        k = e.key()
        if k == Qt.Key_Space:
            self.grant()
        elif k == Qt.Key_Backspace:
            self.revoke()
        elif k == Qt.Key_Escape:
            self.engage_estop()
        elif k == Qt.Key_M:
            self.manual = not self.manual
            self.held.clear()
            if self.manual:
                self.bridge.takeover_started()
            else:
                self.bridge.takeover_ended()
        elif k in TELEOP_KEYS:
            self.held.add(k)

    def changeEvent(self, e):
        # a key released while another window has focus never reaches us: drop held keys
        if e.type() == QEvent.ActivationChange and not self.isActiveWindow():
            self.held.clear()
            if self.manual:
                self.bridge.teleop(0.0, 0.0)
        super().changeEvent(e)

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.held.discard(e.key())

    def send_teleop(self):
        if self.manual:
            lin = sum(TELEOP_KEYS[k][0] for k in self.held)
            ang = sum(TELEOP_KEYS[k][1] for k in self.held)
            self.bridge.teleop(lin, ang)


def main():
    app = QApplication(sys.argv)
    bridge = RosBridge()
    win = Dashboard(bridge)
    win.showMaximized()
    # let Ctrl+C in the terminal close the window
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    keepalive = QTimer()
    keepalive.timeout.connect(lambda: None)
    keepalive.start(200)
    code = app.exec_()
    if win.recorder is not None:
        win.recorder.close()
        win.recorder = None
    bridge.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
