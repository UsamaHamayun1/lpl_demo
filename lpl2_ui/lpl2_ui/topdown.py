# Top-down view overlays for the dashboard: permission regions, option paths, robots,
# goal and a legend. Everything here is drawn from LPL messages onto the camera image;
# nothing is painted into the simulated world.
import math

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF

from lpl2_interfaces.msg import OptionStatus, PermissionState

PERMISSION_COLOR = {
    PermissionState.AVAILABLE: QColor(40, 167, 69),
    PermissionState.UNAVAILABLE: QColor(211, 47, 47),
    PermissionState.HUMAN_GRANTED: QColor(255, 179, 0),
    PermissionState.EGRESS_ONLY: QColor(253, 126, 20),
}
PERMISSION_WORD = {
    PermissionState.AVAILABLE: 'authorized',
    PermissionState.UNAVAILABLE: 'NOT authorized',
    PermissionState.HUMAN_GRANTED: 'authorized by HUMAN',
    PermissionState.EGRESS_ONLY: 'exit only',
}
OPTION_COLOR = {'A': QColor(64, 156, 255), 'B': QColor(0, 210, 130), 'E': QColor(255, 140, 0)}
GHOST = QColor(255, 255, 255)
BLOCKED = QColor(235, 40, 40)
UNREACHABLE = QColor(120, 120, 120)
FONT = 'DejaVu Sans'


class Projection:
    """World (x, y, z) -> pixel of the top-down camera (pinhole looking straight down)."""

    def __init__(self, cam):
        self.cx, self.cy, self.cz = cam['pose']
        self.w, self.h = cam['size']
        self.f = (self.w / 2) / math.tan(cam['hfov'] / 2)

    def px(self, x, y, z=0.0):
        d = self.cz - z
        return QPointF(self.w / 2 + self.f * (x - self.cx) / d, self.h / 2 - self.f * (y - self.cy) / d)

    def rect(self, zone):
        return QRectF(self.px(zone['x'][0], zone['y'][1]), self.px(zone['x'][1], zone['y'][0]))


def outline_text(p, pos, text, color, size=17):
    p.setFont(QFont(FONT, size, QFont.Bold))
    p.setPen(QPen(QColor(0, 0, 0), 5))
    for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1), (0, 2)):
        p.drawText(pos + QPointF(dx, dy), text)
    p.setPen(QPen(color, 1))
    p.drawText(pos, text)


def path_style(opt):
    """-> (colour, width, Qt pen style, tag) for an option's path."""
    base = OPTION_COLOR.get(opt.id, QColor(200, 200, 200))
    if not opt.reachable:
        return UNREACHABLE, 3, Qt.DotLine, f'{opt.id}  past commit point'
    if not opt.possible:
        return BLOCKED, 4, Qt.DashLine, f'{opt.id}  BLOCKED'
    if not opt.authorized:
        return GHOST, 4, Qt.DashLine, f"{opt.id}  possible · needs {', '.join(opt.missing)}"
    if opt.selected:
        return base, 8, Qt.SolidLine, f'{opt.id}  CHOSEN'
    return base, 4, Qt.SolidLine, f'{opt.id}  allowed'


def draw_polyline(p, pts, color, width, style):
    under = QPen(QColor(0, 0, 0, 220), width + 4, style)
    under.setCapStyle(Qt.RoundCap)
    p.setPen(under)
    p.drawPolyline(pts)
    pen = QPen(color, width, style)
    pen.setCapStyle(Qt.RoundCap)
    p.setPen(pen)
    p.drawPolyline(pts)


def draw_overlay(image, corridor, proj, authority, decision, poses):
    """Paint onto a QImage (the top-down frame, in its native resolution)."""
    p = QPainter(image)
    p.setRenderHint(QPainter.Antialiasing)
    zones = corridor.zones

    # Lane 1 outside the passing window: standing traffic rule, never usable
    lane1, window = zones['lane1'], zones['lane1_window']
    for x0, x1 in ((lane1['x'][0], window['x'][0]), (window['x'][1], lane1['x'][1])):
        p.fillRect(proj.rect({'x': (x0, x1), 'y': lane1['y']}), QBrush(QColor(60, 60, 60, 110), Qt.BDiagPattern))

    # Permission regions, labelled with their state
    perms = {pm.region_id: pm for pm in authority.permissions} if authority else {}
    for region_id, side in (('lane1_window', 'top'), ('bypass', 'bottom')):
        pm = perms.get(region_id)
        if pm is None or region_id not in zones:
            continue
        color = QColor(PERMISSION_COLOR.get(pm.state, QColor(150, 150, 150)))
        r = proj.rect(zones[region_id])
        fill = QColor(color)
        fill.setAlpha(80)
        p.fillRect(r, fill)
        p.setPen(QPen(color, 3))
        p.drawRect(r)
        text = f'{pm.id}: {PERMISSION_WORD.get(pm.state, "?")}'
        anchor = r.topLeft() + QPointF(0, -10) if side == 'top' else r.bottomLeft() + QPointF(0, 26)
        outline_text(p, anchor, text, color)

    # Option paths: unselected first, the chosen one last (on top)
    hold_selected = False
    if decision:
        for opt in sorted(decision.options, key=lambda o: (o.selected, o.authorized and o.possible)):
            if opt.kind == OptionStatus.HOLD:
                hold_selected |= opt.selected
                continue
            if len(opt.path) < 2:
                continue
            pts = QPolygonF([proj.px(pt.x, pt.y) for pt in opt.path])
            color, width, style, tag = path_style(opt)
            draw_polyline(p, pts, color, width, style)
            mid = pts[int(len(pts) * 0.45)]
            if not opt.possible and opt.reachable:
                p.setPen(QPen(BLOCKED, 5))
                p.drawLine(mid + QPointF(-9, -9), mid + QPointF(9, 9))
                p.drawLine(mid + QPointF(-9, 9), mid + QPointF(9, -9))
            outline_text(p, mid + QPointF(12, -12), tag, color, 15)

    # Goal
    if decision:
        g = proj.px(decision.goal.x, decision.goal.y)
        p.setPen(QPen(QColor(0, 0, 0), 5))
        p.drawEllipse(g, 11, 11)
        p.setPen(QPen(QColor(255, 255, 255), 3))
        p.drawEllipse(g, 11, 11)
        outline_text(p, g + QPointF(-22, 34), 'GOAL', QColor(255, 255, 255), 14)

    # Robots (ground truth)
    for name, text, color in (('robot_r', 'R', QColor(0, 230, 255)), ('robot2', 'R2', QColor(255, 60, 255))):
        if poses and name in poses:
            x, y, _ = poses[name]
            c = proj.px(x, y, 0.3)
            p.setPen(QPen(QColor(0, 0, 0), 6))
            p.drawEllipse(c, 13, 13)
            p.setPen(QPen(color, 3))
            p.drawEllipse(c, 13, 13)
            outline_text(p, c + QPointF(-10, -20), text, color, 17)
            if name == 'robot_r' and hold_selected:
                outline_text(p, c + QPointF(18, 6), 'HOLD', QColor(255, 80, 80), 15)

    draw_legend(p, proj)
    p.end()


def draw_legend(p, proj):
    """In the empty floor north of the corridor."""
    origin = proj.px(-0.6, 3.3)
    items = [(OPTION_COLOR['A'], 8, Qt.SolidLine, 'chosen'), (OPTION_COLOR['B'], 4, Qt.SolidLine, 'allowed'),
             (GHOST, 4, Qt.DashLine, 'possible, NOT authorized'), (BLOCKED, 4, Qt.DashLine, 'physically blocked'),
             (UNREACHABLE, 3, Qt.DotLine, 'past its commit point')]
    x = origin.x()
    for color, width, style, text in items:
        y = origin.y()
        draw_polyline(p, QPolygonF([QPointF(x, y), QPointF(x + 46, y)]), color, width, style)
        outline_text(p, QPointF(x + 54, y + 6), text, QColor(235, 235, 235), 13)
        x += 54 + 9 * len(text) + 40
