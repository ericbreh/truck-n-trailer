"""High-tech terminal / scan-line animations for startup and tab switching."""

import random

from PyQt6.QtCore import Qt, QRectF, QTimer, QPropertyAnimation, QEasingCurve
from PyQt6.QtGui import QBrush, QColor, QFont, QLinearGradient, QPainter, QPen
from PyQt6.QtWidgets import QGraphicsOpacityEffect, QSizePolicy, QWidget

# ---------------------------------------------------------------------------
# Boot overlay
# ---------------------------------------------------------------------------

_P_TYPING   = 0
_P_PROGRESS = 1
_P_TITLE    = 2   # large glowy title reveal
_P_HOLD     = 3
_P_FADE     = 4

_BOOT_SEQ: list[tuple[str, str | None]] = [
    ("sep", None),
    ("log", "INIT  motor controller ............... [OK]"),
    ("log", "INIT  vision pipeline ................. [OK]"),
    ("log", "INIT  MPC solver ...................... [OK]"),
    ("log", "INIT  hitch potentiometer cal ......... [OK]"),
    ("log", "INIT  UART bus ........................ [OK]"),
    ("log", "INIT  GUI subsystem ................... [OK]"),
    ("log", "INIT  all subsystems nominal"),
    ("sep", None),
    ("ready", "SYSTEM READY"),
]

_HEX = "0123456789ABCDEF"


def _mono(size: int) -> QFont:
    f = QFont()
    f.setFamilies(["SF Mono", "Menlo", "Courier New"])
    f.setPointSize(size)
    return f


def _heading(size: int) -> QFont:
    f = QFont()
    f.setFamilies(["Avenir Next", "Avenir", "Helvetica Neue", "Arial"])
    f.setPointSize(size)
    f.setBold(True)
    return f


class BootOverlay(QWidget):
    """Full-screen terminal boot sequence overlay placed over the main window."""

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setGeometry(parent.rect())
        self.raise_()
        self.show()

        self._typed: list[tuple[str, str]] = []
        self._partial = ""
        self._line_idx = 0
        self._char_idx = 0
        self._phase = _P_TYPING
        self._progress = 0.0
        self._title_alpha = 0.0

        self._ncols, self._nrows = 52, 26
        self._noise: list[list[str]] = [
            [random.choice(_HEX) for _ in range(self._ncols)]
            for _ in range(self._nrows)
        ]

        self._noise_t = QTimer(self)
        self._noise_t.timeout.connect(self._tick_noise)
        self._noise_t.start(65)

        self._type_t = QTimer(self)
        self._type_t.timeout.connect(self._tick_type)
        self._type_t.start(14)

        self._prog_t = QTimer(self)
        self._prog_t.timeout.connect(self._tick_progress)

        self._title_t = QTimer(self)
        self._title_t.timeout.connect(self._tick_title)

    # -- timers --

    def _tick_noise(self) -> None:
        for _ in range(22):
            r = random.randrange(self._nrows)
            c = random.randrange(self._ncols)
            self._noise[r][c] = random.choice(_HEX)
        self.update()

    def _tick_type(self) -> None:
        if self._phase != _P_TYPING:
            return
        while self._line_idx < len(_BOOT_SEQ):
            kind, text = _BOOT_SEQ[self._line_idx]
            if kind in ("sep",):
                self._typed.append((kind, ""))
                self._line_idx += 1
                self.update()
                return
            break
        else:
            self._finish_typing()
            return

        kind, text = _BOOT_SEQ[self._line_idx]
        assert text is not None
        if self._char_idx < len(text):
            self._partial = text[: self._char_idx + 1]
            self._char_idx += 1
        else:
            self._typed.append((kind, text))
            self._partial = ""
            self._char_idx = 0
            self._line_idx += 1
            if self._line_idx >= len(_BOOT_SEQ):
                self._finish_typing()
                return
        self.update()

    def _finish_typing(self) -> None:
        self._type_t.stop()
        self._phase = _P_PROGRESS
        self._prog_t.start(13)
        self.update()

    def _tick_progress(self) -> None:
        self._progress = min(1.0, self._progress + 0.013)
        self.update()
        if self._progress >= 1.0:
            self._prog_t.stop()
            self._phase = _P_TITLE
            QTimer.singleShot(80, self._start_title)

    def _start_title(self) -> None:
        self._title_t.start(16)

    def _tick_title(self) -> None:
        self._title_alpha = min(1.0, self._title_alpha + 0.030)
        self.update()
        if self._title_alpha >= 1.0:
            self._title_t.stop()
            self._phase = _P_HOLD
            QTimer.singleShot(900, self._start_fade)

    def _start_fade(self) -> None:
        self._phase = _P_FADE
        self._noise_t.stop()
        eff = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(eff)
        anim = QPropertyAnimation(eff, b"opacity")
        anim.setDuration(600)
        anim.setStartValue(1.0)
        anim.setEndValue(0.0)
        anim.setEasingCurve(QEasingCurve.Type.InCubic)
        anim.finished.connect(self.deleteLater)
        self._fade_anim = anim
        anim.start()

    # -- paint --

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()

        # solid dark background
        p.fillRect(0, 0, W, H, QColor(2, 8, 18))

        # hex noise grid
        p.setFont(_mono(7))
        cw = W / self._ncols
        ch_cell = H / self._nrows
        p.setPen(QColor(0, 55, 38, 52))
        for r, row in enumerate(self._noise):
            for c, ch_ in enumerate(row):
                p.drawText(
                    int(c * cw), int(r * ch_cell),
                    int(cw) + 1, int(ch_cell) + 1,
                    Qt.AlignmentFlag.AlignCenter, ch_,
                )

        # scanlines
        for sy in range(0, H, 3):
            p.fillRect(0, sy, W, 1, QColor(0, 0, 0, 32))

        # corner brackets
        cyan = QColor(0, 200, 255)
        p.setPen(QPen(cyan, 2))
        mg, sz = 14, 20
        p.drawLine(mg, mg, mg+sz, mg);         p.drawLine(mg, mg, mg, mg+sz)
        p.drawLine(W-mg-sz, mg, W-mg, mg);     p.drawLine(W-mg, mg, W-mg, mg+sz)
        p.drawLine(mg, H-mg, mg+sz, H-mg);     p.drawLine(mg, H-mg-sz, mg, H-mg)
        p.drawLine(W-mg-sz, H-mg, W-mg, H-mg); p.drawLine(W-mg, H-mg-sz, W-mg, H-mg)

        # content area geometry
        margin_x = 64
        content_w = W - margin_x * 2
        y = 40

        # terminal lines
        p.setFont(_mono(10))
        lh = 19
        for kind, text in self._typed:
            if kind == "sep":
                p.setPen(QPen(QColor(0, 55, 90), 1))
                p.drawLine(margin_x, y + lh // 2, margin_x + content_w, y + lh // 2)
                y += lh
                continue
            if kind == "ready":
                p.setPen(QColor(0, 255, 128))
            else:
                p.setPen(QColor(0, 185, 95))
            p.drawText(margin_x, y, content_w, lh,
                       Qt.AlignmentFlag.AlignVCenter, f"  {text}")
            y += lh

        # partial line with typing cursor
        if self._partial and self._phase == _P_TYPING:
            p.setPen(QColor(0, 255, 128))
            p.drawText(margin_x, y, content_w, lh,
                       Qt.AlignmentFlag.AlignVCenter, f"  {self._partial}_")
            y += lh

        # segmented progress bar
        if self._phase >= _P_PROGRESS:
            bar_y = y + 14
            segs = 34
            seg_w = content_w / segs
            filled = int(self._progress * segs)

            p.fillRect(margin_x, bar_y, content_w, 10, QColor(0, 22, 36))
            for i in range(filled):
                seg_alpha = 255 if i < filled - 1 else 180
                x0 = int(margin_x + i * seg_w) + 1
                x1 = int(margin_x + (i + 1) * seg_w) - 1
                p.fillRect(x0, bar_y + 1, x1 - x0, 8, QColor(0, 185, 255, seg_alpha))

            p.setPen(QPen(QColor(0, 90, 140), 1))
            p.drawRect(margin_x, bar_y, content_w, 10)

            p.setFont(_mono(8))
            p.setPen(QColor(0, 150, 210))
            p.drawText(margin_x, bar_y + 14, content_w, 13,
                       Qt.AlignmentFlag.AlignHCenter, f"{int(self._progress*100)}%")

        # ---- large glowy title reveal ----
        if self._phase >= _P_TITLE and self._title_alpha > 0:
            ta = self._title_alpha
            title_cy = int(H * 0.63)

            # wide background glow bloom
            bloom = QLinearGradient(0, title_cy - 40, 0, title_cy + 40)
            bloom.setColorAt(0.0, QColor(0, 100, 220, 0))
            bloom.setColorAt(0.5, QColor(0, 70, 180, int(28 * ta)))
            bloom.setColorAt(1.0, QColor(0, 100, 220, 0))
            p.fillRect(0, title_cy - 40, W, 80, QBrush(bloom))

            # separator lines above and below title
            sep_a = int(120 * ta)
            p.setPen(QPen(QColor(0, 120, 200, sep_a), 1))
            p.drawLine(margin_x, title_cy - 26, margin_x + content_w, title_cy - 26)
            p.drawLine(margin_x, title_cy + 26, margin_x + content_w, title_cy + 26)

            # title font — match header style (Avenir Next Bold + letter spacing)
            tf = _heading(24)
            tf.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 5.0)
            p.setFont(tf)
            tr = QRectF(margin_x, title_cy - 24, content_w, 48)
            align = Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter

            # outer glow: large offsets, low alpha (same technique as DashboardHeader)
            p.setOpacity(ta * 0.10)
            p.setPen(QColor(0, 185, 255))
            for dx in (-3, -2, 2, 3):
                for dy in (-1, 1):
                    p.drawText(tr.translated(dx, dy), align, "TRUCK N TRAILER")

            # inner glow: ±1 px, medium alpha
            p.setOpacity(ta * 0.16)
            p.setPen(QColor(56, 189, 248))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    p.drawText(tr.translated(dx, dy), align, "TRUCK N TRAILER")

            # main crisp text
            p.setOpacity(ta)
            p.setPen(QColor(224, 242, 254))
            p.drawText(tr, align, "TRUCK N TRAILER")
            p.setOpacity(1.0)

            # subtitle
            sf = _heading(11)
            sf.setBold(False)
            sf.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 3.0)
            p.setFont(sf)
            p.setOpacity(ta * 0.65)
            p.setPen(QColor(0, 175, 255))
            p.drawText(QRectF(margin_x, title_cy + 30, content_w, 20),
                       Qt.AlignmentFlag.AlignHCenter, "AUTONOMOUS PARKING SYSTEM")
            p.setOpacity(1.0)

        p.end()


# ---------------------------------------------------------------------------
# Tab-switch scan wipe
# ---------------------------------------------------------------------------

class _ScanWipe(QWidget):
    """Vertical glowing bar sweeps left→right across the mode stack."""

    def __init__(self, stack: QWidget, target_idx: int) -> None:
        super().__init__(stack)
        self._stack = stack
        self._target = target_idx
        self._pos = 0.0      # 0 → 1+ (fraction of width)
        self._switched = False

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setGeometry(stack.rect())
        self.raise_()
        self.show()

        self._t = QTimer(self)
        self._t.timeout.connect(self._tick)
        self._t.start(11)

    def _tick(self) -> None:
        self._pos += 0.065
        if not self._switched and self._pos >= 0.5:
            self._stack.setCurrentIndex(self._target)
            self._switched = True
        if self._pos >= 1.25:
            self._t.stop()
            self.deleteLater()
            return
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        W, H = self.width(), self.height()
        x = int(self._pos * W)

        # dark curtain on left side, fades away as bar passes midpoint
        fade = max(0.0, 1.0 - max(0.0, (self._pos - 0.5) * 3.5))
        left_alpha = int(165 * fade)
        if left_alpha > 0 and x > 0:
            p.fillRect(0, 0, min(x, W), H, QColor(0, 6, 14, left_alpha))

        # subtle scanline tint on right side after reveal
        if self._switched and x < W:
            for sy in range(0, H, 3):
                p.fillRect(x, sy, W - x, 1, QColor(0, 200, 255, 8))

        # glow trail
        for off, bw, alpha in ((-55, 55, 18), (-32, 32, 38), (-14, 14, 80), (-5, 5, 140)):
            bx = max(0, x + off)
            if bx < W:
                actual_w = min(bw, W - bx)
                p.fillRect(bx, 0, actual_w, H, QColor(0, 195, 255, alpha))

        # main bar
        if 0 <= x < W:
            p.fillRect(x, 0, 3, H, QColor(195, 238, 255, 235))
            p.fillRect(x + 3, 0, 9, H, QColor(0, 195, 255, 55))

        p.end()


def animate_tab_switch(stack: QWidget, target_index: int) -> None:
    """Cross-fade the mode stack to target_index (fade out → switch → fade in)."""
    effect = QGraphicsOpacityEffect(stack)
    stack.setGraphicsEffect(effect)

    anim_out = QPropertyAnimation(effect, b"opacity", effect)
    anim_out.setDuration(110)
    anim_out.setStartValue(1.0)
    anim_out.setEndValue(0.0)
    anim_out.setEasingCurve(QEasingCurve.Type.OutQuad)

    def _switch() -> None:
        stack.setCurrentIndex(target_index)
        anim_in = QPropertyAnimation(effect, b"opacity", effect)
        anim_in.setDuration(200)
        anim_in.setStartValue(0.0)
        anim_in.setEndValue(1.0)
        anim_in.setEasingCurve(QEasingCurve.Type.OutCubic)
        anim_in.finished.connect(lambda: stack.setGraphicsEffect(None))
        anim_in.start()

    anim_out.finished.connect(_switch)
    anim_out.start()


# ---------------------------------------------------------------------------
# Camera placeholder
# ---------------------------------------------------------------------------

class CameraPlaceholder(QWidget):
    """Animated targeting-reticle placeholder shown until the first camera frame arrives.

    Drop-in replacement for the QLabel camera_view — supports setPixmap() and setText()
    with the same semantics, plus size()/setMinimumSize() via QWidget inheritance.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._pixmap = None
        self._frame = 0

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

    # -- public interface matching QLabel --

    def setPixmap(self, pxm) -> None:
        if pxm is not None and not pxm.isNull():
            self._pixmap = pxm
            self._timer.stop()
        else:
            self._pixmap = None
            if not self._timer.isActive():
                self._timer.start(33)
        self.update()

    def setText(self, _text: str) -> None:
        self._pixmap = None
        if not self._timer.isActive():
            self._timer.start(33)
        self.update()

    def setAlignment(self, _alignment) -> None:
        pass  # centering handled in paintEvent

    # -- internal --

    def _tick(self) -> None:
        self._frame += 1
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()

        if self._pixmap is not None:
            px = (W - self._pixmap.width()) // 2
            py = (H - self._pixmap.height()) // 2
            p.drawPixmap(px, py, self._pixmap)
            p.end()
            return

        cx, cy = W // 2, H // 2

        # background
        p.fillRect(0, 0, W, H, QColor(15, 23, 42))

        # subtle grid
        p.setPen(QPen(QColor(30, 55, 90, 28), 1))
        for gx in range(0, W, 50):
            p.drawLine(gx, 0, gx, H)
        for gy in range(0, H, 50):
            p.drawLine(0, gy, W, gy)

        # horizontal scan sweep
        scan_frac = (self._frame * 0.007) % 1.0
        scan_y = int(scan_frac * (H + 60)) - 30
        grad = QLinearGradient(0, scan_y - 30, 0, scan_y + 30)
        grad.setColorAt(0.0, QColor(0, 200, 255, 0))
        grad.setColorAt(0.5, QColor(0, 200, 255, 28))
        grad.setColorAt(1.0, QColor(0, 200, 255, 0))
        p.fillRect(0, max(0, scan_y - 30), W, 60, QBrush(grad))

        # staggered pulse rings
        period = 85
        max_r = min(W, H) * 0.21
        for i in range(3):
            rf = (self._frame - i * (period // 3)) % period
            r = 28.0 + (rf / period) * max_r
            alpha = int(155 * max(0.0, 1.0 - rf / period))
            if alpha > 0:
                p.setPen(QPen(QColor(0, 200, 255, alpha), 1))
                p.drawEllipse(QRectF(cx - r, cy - r, r * 2, r * 2))

        # reticle: center dot
        p.setPen(QPen(QColor(0, 200, 255, 170), 1))
        p.drawEllipse(QRectF(cx - 3, cy - 3, 6, 6))

        # reticle: crosshair (gap in the middle)
        gap, reach = 12, 38
        p.setPen(QPen(QColor(0, 200, 255, 165), 1))
        p.drawLine(cx,        cy - gap,  cx,        cy - reach)
        p.drawLine(cx,        cy + gap,  cx,        cy + reach)
        p.drawLine(cx - gap,  cy,        cx - reach, cy)
        p.drawLine(cx + gap,  cy,        cx + reach, cy)

        # reticle: inner corner brackets
        d, bsz = 46, 11
        p.setPen(QPen(QColor(0, 200, 255, 125), 1))
        for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
            bx, by = cx + sx * d, cy + sy * d
            p.drawLine(bx, by, bx - sx * bsz, by)
            p.drawLine(bx, by, bx, by - sy * bsz)

        # outer corner brackets (widget edges)
        p.setPen(QPen(QColor(0, 200, 255, 145), 2))
        sz, mg = 20, 16
        p.drawLine(mg,     mg,     mg+sz,   mg);      p.drawLine(mg,   mg,     mg,   mg+sz)
        p.drawLine(W-mg-sz, mg,    W-mg,    mg);      p.drawLine(W-mg, mg,     W-mg, mg+sz)
        p.drawLine(mg,     H-mg,   mg+sz,   H-mg);    p.drawLine(mg,   H-mg-sz, mg,  H-mg)
        p.drawLine(W-mg-sz, H-mg,  W-mg,    H-mg);    p.drawLine(W-mg, H-mg-sz, W-mg, H-mg)

        # top-right status tag
        p.setFont(_mono(9))
        p.setPen(QColor(0, 90, 145))
        p.drawText(W - 150, 14, 136, 16,
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                   "CAM OFFLINE")

        # widget border
        p.setPen(QPen(QColor(71, 85, 105), 1))
        p.drawRect(0, 0, W - 1, H - 1)

        # bottom status text with blinking cursor
        cursor = "_" if (self._frame // 18) % 2 == 0 else " "
        p.setFont(_mono(11))
        p.setPen(QColor(0, 185, 255, 195))
        p.drawText(0, H - 42, W, 22,
                   Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter,
                   f"AWAITING CAMERA FEED {cursor}")

        p.end()
