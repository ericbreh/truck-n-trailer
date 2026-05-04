import math

import numpy as np

from truck_n_trailer import params
from truck_n_trailer.geometry import body_centers_cm

try:
    from PyQt6.QtCore import Qt, QPointF, QRectF, QSize, QTimer
    from PyQt6.QtGui import QBrush, QColor, QFont, QLinearGradient, QPainter, QPen, QPolygonF
    from PyQt6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyQt6 is required. Install with: pip install PyQt6") from exc


class AutoStateView(QWidget):
    _TRAIL_MAX = 300

    def __init__(self, truck_len_cm: float, trailer_len_cm: float):
        super().__init__()
        self.truck_len_cm = truck_len_cm
        self.trailer_len_cm = trailer_len_cm
        self.current_q: np.ndarray | None = None
        self.goal_q: np.ndarray | None = None
        self.path_points: list[tuple[float, float]] = []
        self.pred_path_xy: list[tuple[float, float]] = []
        self._pulse_t: float = 0.0
        self.setMinimumSize(320, 280)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._pulse_timer = QTimer(self)
        self._pulse_timer.timeout.connect(self._tick_pulse)
        self._pulse_timer.start(33)

    def set_goal(self, q_goal: np.ndarray) -> None:
        self.goal_q = q_goal.copy()
        self.update()

    def reset_path(self, q_start: np.ndarray) -> None:
        self.path_points = [(float(q_start[0]), float(q_start[1]))]
        self.current_q = q_start.copy()
        self.update()

    def set_state(self, q: np.ndarray) -> None:
        self.current_q = q.copy()
        self.path_points.append((float(q[0]), float(q[1])))
        if len(self.path_points) > self._TRAIL_MAX:
            self.path_points = self.path_points[-self._TRAIL_MAX:]
        self.update()

    def set_pred_path(self, xy_list: list[tuple[float, float]]) -> None:
        self.pred_path_xy = list(xy_list)
        self.update()

    def _tick_pulse(self) -> None:
        self._pulse_t = (self._pulse_t + 0.07) % (2.0 * math.pi)
        if self.goal_q is not None:
            self.update()

    def _bounds(self) -> tuple[float, float, float, float]:
        xs: list[float] = []
        ys: list[float] = []
        for x, y in self.path_points:
            xs.append(x)
            ys.append(y)
        if self.current_q is not None:
            xs.extend([float(self.current_q[0])])
            ys.extend([float(self.current_q[1])])
        if self.goal_q is not None:
            xs.extend([float(self.goal_q[0])])
            ys.extend([float(self.goal_q[1])])
        for x, y in self.pred_path_xy:
            xs.append(float(x))
            ys.append(float(y))
        if not xs:
            return 0.0, 1.0, 0.0, 1.0

        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)
        geom_pad = max(self.truck_len_cm, self.trailer_len_cm) + 4.0
        pad = max(8.0, geom_pad)
        if abs(x_max - x_min) < 1e-6:
            x_max += 1.0
        if abs(y_max - y_min) < 1e-6:
            y_max += 1.0
        return x_min - pad, x_max + pad, y_min - pad, y_max + pad

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        W, H = self.width(), self.height()

        # ── background ──────────────────────────────────────────────────── #
        bg_grad = QLinearGradient(0, 0, 0, H)
        bg_grad.setColorAt(0.0, QColor("#060b14"))
        bg_grad.setColorAt(1.0, QColor("#0a1020"))
        painter.fillRect(self.rect(), bg_grad)

        # ── "waiting" placeholder ───────────────────────────────────────── #
        if self.current_q is None:
            painter.setPen(QPen(QColor("#1e3a5f"), 1))
            painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
            font = painter.font()
            font.setPointSize(11)
            font.setFamily("Courier New")
            painter.setFont(font)
            painter.setPen(QColor("#1e3a5f"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "WAITING FOR STATE")
            painter.end()
            return

        # ── coordinate transform ────────────────────────────────────────── #
        x_min, x_max, y_min, y_max = self._bounds()
        world_w = max(1.0, x_max - x_min)
        world_h = max(1.0, y_max - y_min)
        margin = 24.0
        draw_w = max(1.0, W - 2.0 * margin)
        draw_h = max(1.0, H - 2.0 * margin)
        scale = min(draw_w / world_w, draw_h / world_h)

        def to_px(wx: float, wy: float) -> tuple[float, float]:
            return (margin + (wx - x_min) * scale,
                    H - (margin + (wy - y_min) * scale))

        # ── grid ────────────────────────────────────────────────────────── #
        grid_step = 10.0
        grid_pen_minor = QPen(QColor(14, 26, 46, 180), 1)
        grid_pen_major = QPen(QColor(30, 58, 95, 120), 1)
        font = painter.font()
        font.setPointSize(7)
        font.setFamily("Courier New")
        painter.setFont(font)
        label_color = QColor(56, 189, 248, 60)

        gx = math.floor(x_min / grid_step) * grid_step
        while gx <= x_max:
            px, _ = to_px(gx, y_min)
            _, py_top = to_px(gx, y_max)
            is_major = abs(round(gx / grid_step) % 5) == 0
            painter.setPen(grid_pen_major if is_major else grid_pen_minor)
            painter.drawLine(int(px), int(py_top), int(px), H - int(margin))
            if is_major:
                painter.setPen(QPen(label_color))
                painter.drawText(int(px) - 10, H - int(margin / 2) + 4, f"{int(gx)}")
            gx += grid_step

        gy = math.floor(y_min / grid_step) * grid_step
        while gy <= y_max:
            _, py = to_px(x_min, gy)
            is_major = abs(round(gy / grid_step) % 5) == 0
            painter.setPen(grid_pen_major if is_major else grid_pen_minor)
            painter.drawLine(int(margin), int(py), W - int(margin), int(py))
            if is_major:
                painter.setPen(QPen(label_color))
                painter.drawText(2, int(py) + 4, f"{int(gy)}")
            gy += grid_step

        # ── workspace boundary ───────────────────────────────────────────── #
        ws_w, ws_h = params.WORKSPACE_WIDTH_CM, params.WORKSPACE_HEIGHT_CM
        ws_corners = [(0.0, 0.0), (ws_w, 0.0), (ws_w, ws_h), (0.0, ws_h)]
        ws_px = [to_px(cx, cy) for cx, cy in ws_corners]
        ws_pen = QPen(QColor(56, 189, 248, 30), 1)
        ws_pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(ws_pen)
        for i in range(4):
            ax, ay = ws_px[i]
            bx, by = ws_px[(i + 1) % 4]
            painter.drawLine(int(ax), int(ay), int(bx), int(by))
        bracket_pen = QPen(QColor(56, 189, 248, 80), 1)
        painter.setPen(bracket_pen)
        bracket_len = 8
        for cx, cy in ws_px:
            ix, iy = int(cx), int(cy)
            sx = 1 if cx < W / 2 else -1
            sy = 1 if cy > H / 2 else -1
            painter.drawLine(ix, iy, ix + sx * bracket_len, iy)
            painter.drawLine(ix, iy, ix, iy + sy * bracket_len)

        # ── path trail ──────────────────────────────────────────────────── #
        pts = self.path_points
        n = len(pts)
        if n >= 2:
            for i in range(1, n):
                frac = (i / n) ** 1.8
                painter.setPen(QPen(QColor(6, 182, 212, int(18 * frac)), 5))
                x0, y0 = pts[i - 1]; x1, y1 = pts[i]
                p0x, p0y = to_px(x0, y0); p1x, p1y = to_px(x1, y1)
                painter.drawLine(int(p0x), int(p0y), int(p1x), int(p1y))
                pen = QPen(QColor(6, 182, 212, int(200 * frac)), 1.5)
                pen.setCapStyle(Qt.PenCapStyle.RoundCap)
                painter.setPen(pen)
                painter.drawLine(int(p0x), int(p0y), int(p1x), int(p1y))

        # ── prediction path ──────────────────────────────────────────────── #
        if len(self.pred_path_xy) >= 2:
            for i in range(1, len(self.pred_path_xy)):
                ax, ay = self.pred_path_xy[i - 1]
                bx, by = self.pred_path_xy[i]
                pax, pay = to_px(ax, ay); pbx, pby = to_px(bx, by)
                gp = QPen(QColor(6, 182, 212, 25), 6)
                painter.setPen(gp)
                painter.drawLine(int(pax), int(pay), int(pbx), int(pby))
                dp = QPen(QColor(34, 211, 238, 180), 1)
                dp.setStyle(Qt.PenStyle.DashLine)
                painter.setPen(dp)
                painter.drawLine(int(pax), int(pay), int(pbx), int(pby))

        # ── vehicle bodies ──────────────────────────────────────────────── #
        q = self.current_q
        x, y, theta_t, theta_l = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        v = float(q[4]) if len(q) > 4 else 0.0
        hx, hy = to_px(x, y)

        body_w_px = max(self.truck_len_cm, self.trailer_len_cm) * 0.28 * scale

        def draw_body_glowing(cx: float, cy: float, heading: float,
                              length_cm: float, fill: QColor,
                              glow_rgb: tuple, label: str) -> None:
            half_l = length_cm * scale / 2.0
            half_w = body_w_px / 2.0
            rect = QRectF(-half_l, -half_w, half_l * 2, half_w * 2)
            gr, gg, gb = glow_rgb

            painter.save()
            painter.translate(cx, cy)
            painter.rotate(math.degrees(-heading))

            for gw, ga in ((18, 12), (12, 30), (7, 60)):
                painter.setPen(QPen(QColor(gr, gg, gb, ga), gw))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawRoundedRect(rect, 3, 3)

            painter.setBrush(QBrush(fill))
            painter.setPen(QPen(QColor(gr, gg, gb, 220), 1.5))
            painter.drawRoundedRect(rect, 3, 3)

            ap = QPen(QColor(gr, gg, gb, 255), 2)
            ap.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(ap)
            tip = half_l - 3
            painter.drawLine(0, 0, int(tip), 0)
            painter.drawLine(int(tip), 0, int(tip - 7), -4)
            painter.drawLine(int(tip), 0, int(tip - 7), 4)

            lc = QColor(gr, gg, gb, 200)
            painter.setPen(QPen(lc))
            lf = painter.font()
            lf.setPointSize(7)
            lf.setBold(True)
            lf.setFamily("Courier New")
            painter.setFont(lf)
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, label)
            painter.restore()

        (tmx, tmy), (trx, try_) = body_centers_cm(q, self.truck_len_cm, self.trailer_len_cm)
        t_px, t_py = to_px(tmx, tmy)
        draw_body_glowing(t_px, t_py, theta_l, self.trailer_len_cm,
                          QColor(88, 28, 200, 190), (167, 139, 250), "TRAILER")

        tr_px, tr_py = to_px(trx, try_)
        draw_body_glowing(tr_px, tr_py, theta_t, self.truck_len_cm,
                          QColor(154, 52, 18, 200), (251, 146, 60), "TRUCK")

        # ── speed vector ────────────────────────────────────────────────── #
        if abs(v) > 1.0:
            vec_len_px = min(abs(v) * 3.5, 60.0)
            sign = 1.0 if v > 0 else -1.0
            vx = hx + sign * vec_len_px * math.cos(theta_t)
            vy = hy - sign * vec_len_px * math.sin(theta_t)
            painter.setPen(QPen(QColor(250, 204, 21, 30), 8))
            painter.drawLine(int(hx), int(hy), int(vx), int(vy))
            painter.setPen(QPen(QColor("#facc15"), 2))
            painter.drawLine(int(hx), int(hy), int(vx), int(vy))
            arrow_dx = vx - hx; arrow_dy = vy - hy
            norm = math.hypot(arrow_dx, arrow_dy)
            if norm > 1e-6:
                ux, uy = arrow_dx / norm, arrow_dy / norm
                px_l = vx - ux * 8 - uy * 4
                py_l = vy - uy * 8 + ux * 4
                px_r = vx - ux * 8 + uy * 4
                py_r = vy - uy * 8 - ux * 4
                painter.setBrush(QBrush(QColor("#facc15")))
                painter.setPen(Qt.PenStyle.NoPen)
                painter.drawPolygon(QPolygonF([
                    QPointF(vx, vy), QPointF(px_l, py_l), QPointF(px_r, py_r)
                ]))

        # ── hitch dot ───────────────────────────────────────────────────── #
        for r, a in ((16, 10), (11, 35), (7, 90)):
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor(253, 224, 71, a)))
            painter.drawEllipse(QRectF(hx - r, hy - r, r * 2, r * 2))
        painter.setBrush(QBrush(QColor("#fde047")))
        painter.setPen(QPen(QColor("#fbbf24"), 1))
        painter.drawEllipse(QRectF(hx - 4, hy - 4, 8, 8))

        # ── goal reticle ─────────────────────────────────────────────────── #
        if self.goal_q is not None:
            gx_w, gy_w = to_px(float(self.goal_q[0]), float(self.goal_q[1]))
            pulse = math.sin(self._pulse_t)
            outer_r = 13.0 + 3.0 * pulse
            arm = 18.0 + 2.0 * pulse
            for gr, ga in ((outer_r + 8, 15), (outer_r + 4, 30)):
                painter.setPen(QPen(QColor(249, 115, 22, ga), 2))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(QRectF(gx_w - gr, gy_w - gr, gr * 2, gr * 2))
            rpen = QPen(QColor("#f97316"), 1.5)
            painter.setPen(rpen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(QRectF(gx_w - outer_r, gy_w - outer_r, outer_r * 2, outer_r * 2))
            for p0, p1 in [
                ((gx_w - arm, gy_w), (gx_w - outer_r + 2, gy_w)),
                ((gx_w + outer_r - 2, gy_w), (gx_w + arm, gy_w)),
                ((gx_w, gy_w - arm), (gx_w, gy_w - outer_r + 2)),
                ((gx_w, gy_w + outer_r - 2), (gx_w, gy_w + arm)),
            ]:
                painter.drawLine(int(p0[0]), int(p0[1]), int(p1[0]), int(p1[1]))
            painter.setBrush(QBrush(QColor(239, 68, 68, int(180 + 60 * pulse))))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QRectF(gx_w - 3, gy_w - 3, 6, 6))

        # ── scan-line overlay ────────────────────────────────────────────── #
        scan_pen = QPen(QColor(0, 0, 0, 18), 1)
        painter.setPen(scan_pen)
        for row in range(0, H, 3):
            painter.drawLine(0, row, W, row)

        painter.end()


class HitchGauge(QWidget):
    """Arc gauge showing hitch angle.  -90° left, 0° centre, +90° right."""
    _SAFE_DEG = 35.0
    _WARN_DEG = 45.0

    def __init__(self, label: str = "", parent=None):
        super().__init__(parent)
        self._deg: float = 0.0
        self._connected: bool = False
        self._label_text = label
        self.setMinimumSize(130, 110)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)

    def setValue(self, deg: float) -> None:
        self._deg = max(-90.0, min(90.0, float(deg)))
        self.update()

    def setConnected(self, connected: bool) -> None:
        self._connected = connected
        self.update()

    def sizeHint(self) -> QSize:
        return QSize(140, 110)

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        w, h = self.width(), self.height()
        painter.fillRect(self.rect(), QColor("#0b1322"))

        arc_h = int(h * 0.72)
        cx = w / 2.0
        cy = arc_h * 1.0
        radius = min(cx - 12, arc_h - 8)
        arc_rect = QRectF(cx - radius, cy - radius, radius * 2, radius * 2)

        arc_pen_w = 10

        def draw_arc_segment(start_angle_deg: float, span_deg: float, color: str) -> None:
            pen = QPen(QColor(color), arc_pen_w)
            pen.setCapStyle(Qt.PenCapStyle.FlatCap)
            painter.setPen(pen)
            painter.drawArc(arc_rect, int(start_angle_deg * 16), int(span_deg * 16))

        draw_arc_segment(0, self._SAFE_DEG, "#22c55e")
        draw_arc_segment(self._SAFE_DEG, self._WARN_DEG - self._SAFE_DEG, "#eab308")
        draw_arc_segment(self._WARN_DEG, 90.0 - self._WARN_DEG, "#ef4444")
        draw_arc_segment(180.0 - self._SAFE_DEG, self._SAFE_DEG, "#22c55e")
        draw_arc_segment(180.0 - self._WARN_DEG, self._WARN_DEG - self._SAFE_DEG, "#eab308")
        draw_arc_segment(90.0, 90.0 - self._WARN_DEG, "#ef4444")

        painter.setPen(QPen(QColor("#475569"), 1))
        painter.drawLine(int(cx), int(cy - radius + 2), int(cx), int(cy - radius - 6))

        needle_angle_rad = math.radians(90.0 - self._deg)
        needle_len = radius - arc_pen_w // 2 - 2
        nx = cx + needle_len * math.cos(needle_angle_rad)
        ny = cy - needle_len * math.sin(needle_angle_rad)
        needle_color = "#f0f9ff" if self._connected else "#334155"
        needle_pen = QPen(QColor(needle_color), 3)
        needle_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(needle_pen)
        painter.drawLine(int(cx), int(cy), int(nx), int(ny))

        painter.setBrush(QBrush(QColor(needle_color)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(int(cx - 4), int(cy - 4), 8, 8)

        value_str = f"{self._deg:+.1f}°" if self._connected else "---"
        painter.setPen(QPen(QColor("#e2e8f0" if self._connected else "#475569")))
        font = painter.font()
        font.setFamily("Courier New")
        font.setPointSize(11)
        font.setBold(True)
        painter.setFont(font)
        text_y = arc_h + int((h - arc_h) * 0.45)
        painter.drawText(self.rect().adjusted(0, text_y - h, 0, 0), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, value_str)

        painter.setPen(QPen(QColor("#64748b")))
        font.setPointSize(9)
        font.setBold(False)
        painter.setFont(font)
        painter.drawText(self.rect().adjusted(0, 4, 0, 0), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, self._label_text)

        painter.end()


class TelemetryCard(QWidget):
    def __init__(self, initial_value: str = "---", label: str = "", parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            "background-color: #0b1322; border: 1px solid #1e3a5f; border-radius: 6px;"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)

        self._value_label = QLabel(initial_value)
        self._value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._value_label.setStyleSheet(
            "font-family: 'Courier New', 'Consolas', monospace; font-size: 26px; "
            "color: #e2e8f0; background: transparent; border: none;"
        )

        self._label = QLabel(label)
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setStyleSheet(
            "font-size: 10px; color: #64748b; background: transparent; border: none;"
        )

        layout.addWidget(self._value_label)
        layout.addWidget(self._label)

    def setText(self, text: str) -> None:
        self._value_label.setText(text)

    def text(self) -> str:
        return self._value_label.text()


class StatusBadge(QLabel):
    _STATES = {
        "running":    ("#14532d", "#4ade80"),
        "recovering": ("#713f12", "#fde047"),
        "recovery":   ("#713f12", "#fde047"),
        "finished":   ("#1e3a5f", "#60a5fa"),
        "stopped":    ("#1e293b", "#94a3b8"),
        "idle":       ("#1e293b", "#94a3b8"),
        "fail":       ("#7f1d1d", "#f87171"),
        "need":       ("#7f1d1d", "#f87171"),
    }
    _DEFAULT = ("#1e293b", "#94a3b8")

    def __init__(self, text: str = "", parent=None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._apply_state(text)

    def setText(self, text: str) -> None:
        super().setText(text)
        self._apply_state(text)

    def _apply_state(self, text: str) -> None:
        lower = text.lower()
        bg, fg = self._DEFAULT
        for key, colors in self._STATES.items():
            if key in lower:
                bg, fg = colors
                break
        self.setStyleSheet(
            f"background-color: {bg}; color: {fg}; border-radius: 10px; "
            f"padding: 4px 14px; font-weight: bold; font-size: 13px;"
        )


class DashboardHeader(QWidget):
    """Title bar: logo text + connection LED + mode badge."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._mode = "MANUAL"
        self._pulse_t = 0.0
        self.setFixedHeight(52)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def set_connected(self, connected: bool) -> None:
        self._connected = connected

    def set_mode(self, mode: str) -> None:
        self._mode = mode.upper()

    def _tick(self) -> None:
        self._pulse_t = (self._pulse_t + 0.12) % (2.0 * math.pi)
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        W, H = self.width(), self.height()

        grad = QLinearGradient(0, 0, W, 0)
        grad.setColorAt(0.0, QColor("#0a0f1e"))
        grad.setColorAt(0.5, QColor("#0d1a35"))
        grad.setColorAt(1.0, QColor("#0a0f1e"))
        painter.fillRect(self.rect(), grad)

        painter.setPen(QPen(QColor("#1e3a5f"), 1))
        painter.drawLine(0, H - 1, W, H - 1)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor("#0ea5e9")))
        painter.drawRect(0, 8, 3, H - 16)

        font = painter.font()
        font.setFamily("Segoe UI")

        # Calculate badge dimensions first
        mode_colors = {"MANUAL": ("#1e3a5f", "#7dd3fc"), "AUTOMATIC": ("#14532d", "#4ade80")}
        mbg, mfg = mode_colors.get(self._mode, ("#1e293b", "#94a3b8"))
        badge_text = f"  {self._mode}  "
        font.setPointSize(9)
        font.setBold(True)
        font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.5)
        painter.setFont(font)
        fm_badge = painter.fontMetrics()
        bw = fm_badge.horizontalAdvance(badge_text) + 8

        # Available width for subtitle (left of badge + padding)
        avail_w = W - 22 - bw - 56

        # Draw main title
        font.setPointSize(16)
        font.setBold(True)
        font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 4.0)
        painter.setFont(font)

        glow_color = QColor(56, 189, 248, 40)
        painter.setPen(QPen(glow_color))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                painter.drawText(QRectF(20 + dx, dy, W - 20, H),
                                 Qt.AlignmentFlag.AlignVCenter, "TRUCK N TRAILER")
        painter.setPen(QPen(QColor("#e0f2fe")))
        painter.drawText(QRectF(20, 0, W - 20, H), Qt.AlignmentFlag.AlignVCenter, "TRUCK N TRAILER")

        # Draw subtitle - adjust font size to fit
        font.setPointSize(8)
        font.setBold(False)
        font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.5)
        painter.setFont(font)
        fm = painter.fontMetrics()
        text = "AUTONOMOUS PARKING SYSTEM"
        while fm.horizontalAdvance(text) > avail_w and font.pointSize() > 6:
            font.setPointSize(font.pointSize() - 1)
            painter.setFont(font)
            fm = painter.fontMetrics()
        painter.setPen(QPen(QColor("#38bdf8")))
        painter.drawText(QRectF(22, 26, avail_w, H - 26),
                         Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, text)

        # Draw mode badge
        bh = 22
        bx = W - bw - 56
        by = (H - bh) // 2
        painter.setBrush(QBrush(QColor(mbg)))
        painter.setPen(QPen(QColor(mfg), 1))
        painter.drawRoundedRect(QRectF(bx, by, bw, bh), 4, 4)
        font.setPointSize(9)
        font.setBold(True)
        font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.5)
        painter.setFont(font)
        painter.setPen(QPen(QColor(mfg)))
        painter.drawText(QRectF(bx, by, bw, bh), Qt.AlignmentFlag.AlignCenter, badge_text)

        # Draw LED
        led_x = W - 22
        led_y = H // 2
        led_r = 6
        if self._connected:
            pulse = 0.5 + 0.5 * math.sin(self._pulse_t)
            glow_r = int(led_r + 5 * pulse)
            glow_a = int(60 * pulse)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor(34, 197, 94, glow_a)))
            painter.drawEllipse(led_x - glow_r, led_y - glow_r, glow_r * 2, glow_r * 2)
            painter.setBrush(QBrush(QColor("#22c55e")))
        else:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor("#1e293b")))
        painter.drawEllipse(led_x - led_r, led_y - led_r, led_r * 2, led_r * 2)
        painter.end()
