import os as _os
import sys as _sys

# Allow `python app.py` in addition to `python -m truck_n_trailer.gui`
if __package__ is None or __package__ == "":
    _repo_root = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    if _repo_root not in _sys.path:
        _sys.path.insert(0, _repo_root)
    __package__ = "truck_n_trailer.gui"

import argparse
import time

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from PyQt6.QtCore import Qt, QTimer
    from PyQt6.QtGui import QImage, QPixmap
    from PyQt6.QtWidgets import (
        QApplication,
        QButtonGroup,
        QComboBox,
        QMessageBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QRadioButton,
        QSizePolicy,
        QSlider,
        QStackedWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyQt6 is required. Install with: pip install PyQt6") from exc

from .widgets import AutoStateView, HitchGauge, TelemetryCard, StatusBadge, DashboardHeader
from .stylesheet import get_blue_stylesheet
from truck_n_trailer import params
from truck_n_trailer.hitch_calibration import (
    HitchPotCalibration,
    default_hitch_calibration,
    fit_hitch_calibration,
    hitch_calibration_prompts,
)
from truck_n_trailer.control import AutoController
from truck_n_trailer.control.teleop import motion_to_rpms

from truck_n_trailer.vision.processor import VisionProcessor
from truck_n_trailer.uart import UartPacketSender, validate_sender_config


class TruckControlGui(QWidget):
    def __init__(self, args: argparse.Namespace):
        super().__init__()
        self.setWindowTitle("Truck UART Teleop")
        self.resize(1280, 800)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setStyleSheet(get_blue_stylesheet())

        self.header = DashboardHeader(self)

        self.sender: UartPacketSender | None = None
        self.current_motion = "STOP"
        self.latest_hitch_raw_deg: float | None = None
        self.latest_hitch_display_deg = 0.0
        self.hitch_pot: HitchPotCalibration = default_hitch_calibration()

        # Controllers
        self.vision = VisionProcessor()
        self.auto = AutoController()
        self.auto_state_view = None  # Set in _build_ui

        # UI widgets
        self.port_box = QComboBox()
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 120)
        self.speed_slider.setValue(int(args.rpm))
        self.speed_value = QLabel(f"{int(args.rpm)} RPM")

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.refresh_btn = QPushButton("Refresh")

        self.forward_btn = QPushButton("▲")
        self.back_btn = QPushButton("▼")
        self.left_btn = QPushButton("◀")
        self.right_btn = QPushButton("▶")
        for btn in (self.forward_btn, self.back_btn, self.left_btn, self.right_btn):
            btn.setObjectName("dpad_btn")
        self.drive_buttons = [
            self.forward_btn,
            self.back_btn,
            self.left_btn,
            self.right_btn,
        ]

        self.mode_manual_radio = QRadioButton("Manual")
        self.mode_auto_radio = QRadioButton("Automatic")
        self.mode_manual_radio.setChecked(True)
        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.mode_manual_radio, 0)
        self.mode_group.addButton(self.mode_auto_radio, 1)

        self.mode_stack = QStackedWidget()
        self.auto_start_btn = QPushButton("Start")
        self.auto_start_btn.setObjectName("auto_start_btn")
        self.auto_stop_btn = QPushButton("Stop")
        self.auto_stop_btn.setObjectName("auto_stop_btn")
        self.auto_status_label = StatusBadge("MPC: Idle")
        self.auto_state_view = AutoStateView(
            truck_len_cm=params.TRUCK_LENGTH_CM,
            trailer_len_cm=params.TRAILER_LENGTH_CM,
        )
        self.hitch_angle_gauge = HitchGauge("POT")
        self.hitch_vision_gauge = HitchGauge("VISION")
        self.calibrate_btn = QPushButton("Calibration")
        self.manual_left_rpm = TelemetryCard("0.0", "Left RPM")
        self.manual_right_rpm = TelemetryCard("0.0", "Right RPM")
        self.auto_left_rpm = TelemetryCard("0.0", "Left RPM")
        self.auto_right_rpm = TelemetryCard("0.0", "Right RPM")

        self.camera_source_box = QComboBox()
        self.camera_refresh_btn = QPushButton("Refresh")
        self.camera_connect_btn = QPushButton("Connect")
        self.camera_disconnect_btn = QPushButton("Stop")
        self.camera_status_label = QLabel("Camera: Disconnected")
        self.camera_view = QLabel("No camera feed")
        self.camera_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_view.setMinimumSize(560, 360)
        self.camera_view.setStyleSheet("background: #0f172a; border: 1px solid #475569;")

        self.send_timer = QTimer(self)
        self.send_timer.timeout.connect(self._tick_send)
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self._tick_camera)

        self._build_ui()
        self._bind_controls()
        self._refresh_ports()
        self._refresh_cameras()
        if args.port:
            self._select_port(args.port)
        self._set_connected(False)
        self._set_camera_connected(False)

    def _build_ui(self) -> None:
        outer = QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.setLayout(outer)
        outer.addWidget(self.header)

        body = QWidget()
        root = QHBoxLayout(body)
        root.setContentsMargins(8, 8, 8, 8)
        outer.addWidget(body, stretch=1)

        left_col = QVBoxLayout()
        right_col = QVBoxLayout()
        root.addLayout(left_col, stretch=2)
        root.addLayout(right_col, stretch=3)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        mode_row.addWidget(self.mode_manual_radio)
        mode_row.addWidget(self.mode_auto_radio)
        mode_row.addStretch()
        left_col.addLayout(mode_row)

        uart_row = QHBoxLayout()
        uart_row.setSpacing(6)
        uart_row.addWidget(QLabel("UART"))
        uart_row.addWidget(self.port_box, stretch=1)
        uart_row.addWidget(self.refresh_btn)
        uart_row.addWidget(self.connect_btn)
        uart_row.addWidget(self.disconnect_btn)
        left_col.addLayout(uart_row)

        hitch_group = QGroupBox("Hitch Angle")
        hitch_layout = QVBoxLayout(hitch_group)
        hitch_gauges = QHBoxLayout()
        hitch_gauges.addWidget(self.hitch_angle_gauge)
        hitch_gauges.addWidget(self.hitch_vision_gauge)
        hitch_layout.addLayout(hitch_gauges)
        hitch_layout.addWidget(self.calibrate_btn, alignment=Qt.AlignmentFlag.AlignHCenter)
        left_col.addWidget(hitch_group)

        manual_page = QWidget()
        manual_layout = QVBoxLayout(manual_page)
        speed_row = QHBoxLayout()
        speed_row.addWidget(self.speed_slider)
        speed_row.addWidget(self.speed_value)
        manual_form = QFormLayout()
        manual_form.addRow("Base RPM", speed_row)
        manual_layout.addLayout(manual_form)

        manual_telemetry_group = QGroupBox("Telemetry")
        manual_telemetry_group.setSizePolicy(
            QSizePolicy.Policy.Preferred,
            QSizePolicy.Policy.Maximum,
        )
        manual_telemetry_group.setMaximumHeight(108)
        manual_telemetry_row = QHBoxLayout(manual_telemetry_group)
        manual_telemetry_row.setSpacing(6)
        manual_telemetry_row.addWidget(self.manual_left_rpm)
        manual_telemetry_row.addWidget(self.manual_right_rpm)
        manual_layout.addWidget(manual_telemetry_group)
        manual_layout.addStretch(1)

        drive_group = QGroupBox("Drive")
        drive_grid = QGridLayout()
        drive_group.setLayout(drive_grid)
        drive_grid.addWidget(self.forward_btn, 0, 1)
        drive_grid.addWidget(self.left_btn, 1, 0)
        drive_grid.addWidget(self.right_btn, 1, 2)
        drive_grid.addWidget(self.back_btn, 2, 1)
        manual_layout.addWidget(drive_group)

        auto_page = QWidget()
        auto_layout = QVBoxLayout(auto_page)
        auto_ctrl_row = QHBoxLayout()
        auto_ctrl_row.setSpacing(10)
        auto_ctrl_row.addWidget(self.auto_start_btn, 0, Qt.AlignmentFlag.AlignVCenter)
        auto_ctrl_row.addWidget(self.auto_stop_btn, 0, Qt.AlignmentFlag.AlignVCenter)
        auto_ctrl_row.addStretch(1)
        auto_ctrl_row.addWidget(
            self.auto_status_label,
            0,
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
        )
        auto_layout.addLayout(auto_ctrl_row)

        auto_telemetry_group = QGroupBox("Telemetry")
        auto_telemetry_group.setSizePolicy(
            QSizePolicy.Policy.Preferred,
            QSizePolicy.Policy.Maximum,
        )
        auto_telemetry_group.setMaximumHeight(108)
        auto_telemetry_row = QHBoxLayout(auto_telemetry_group)
        auto_telemetry_row.setSpacing(6)
        auto_telemetry_row.addWidget(self.auto_left_rpm)
        auto_telemetry_row.addWidget(self.auto_right_rpm)
        auto_layout.addWidget(auto_telemetry_group)
        auto_layout.addStretch(1)

        cam_row = QHBoxLayout()
        cam_row.setSpacing(6)
        cam_row.addWidget(QLabel("Camera"))
        cam_row.addWidget(self.camera_source_box, stretch=1)
        cam_row.addWidget(self.camera_refresh_btn)
        cam_row.addWidget(self.camera_connect_btn)
        cam_row.addWidget(self.camera_disconnect_btn)

        self.mode_stack.addWidget(manual_page)
        self.mode_stack.addWidget(auto_page)
        left_col.addWidget(self.mode_stack, stretch=1)
        right_col.addLayout(cam_row)
        right_col.addWidget(self.camera_status_label)
        right_col.addWidget(self.camera_view, stretch=2)
        self.auto_state_view.setMinimumHeight(200)
        right_col.addWidget(self.auto_state_view, stretch=1)

    def _bind_controls(self) -> None:
        self.mode_group.idClicked.connect(self._on_mode_changed)
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn.clicked.connect(self._disconnect)
        self.speed_slider.valueChanged.connect(self._update_speed_label)

        self.forward_btn.pressed.connect(lambda: self._set_motion("FORWARD"))
        self.forward_btn.released.connect(lambda: self._set_motion("STOP"))
        self.back_btn.pressed.connect(lambda: self._set_motion("BACK"))
        self.back_btn.released.connect(lambda: self._set_motion("STOP"))
        self.left_btn.pressed.connect(lambda: self._set_motion("LEFT"))
        self.left_btn.released.connect(lambda: self._set_motion("STOP"))
        self.right_btn.pressed.connect(lambda: self._set_motion("RIGHT"))
        self.right_btn.released.connect(lambda: self._set_motion("STOP"))

        self.auto_start_btn.clicked.connect(self._auto_start)
        self.auto_stop_btn.clicked.connect(self._auto_stop)
        self.calibrate_btn.clicked.connect(self._run_hitch_calibration)
        self.camera_refresh_btn.clicked.connect(self._refresh_cameras)
        self.camera_connect_btn.clicked.connect(self._connect_camera)
        self.camera_disconnect_btn.clicked.connect(self._disconnect_camera)

    def _update_speed_label(self, value: int) -> None:
        self.speed_value.setText(f"{value} RPM")

    def _update_hitch_angle_display(self, raw_adc: float) -> None:
        corrected = self.hitch_pot.raw_to_physical_deg(raw_adc)
        clamped = max(-180.0, min(180.0, float(corrected)))
        self.latest_hitch_display_deg = clamped
        self.hitch_angle_gauge.setValue(clamped)
        self.hitch_angle_gauge.setConnected(True)

    def _update_hitch_vision_display(self, angle_deg: float | None) -> None:
        if angle_deg is None:
            self.hitch_vision_gauge.setConnected(False)
            return
        clamped = max(-90.0, min(90.0, float(angle_deg)))
        self.hitch_vision_gauge.setValue(clamped)
        self.hitch_vision_gauge.setConnected(True)

    def _capture_hitch_measurement(self) -> float | None:
        raw = self.latest_hitch_raw_deg
        if raw is None:
            sender = self.sender
            if sender is None:
                return None
            raw = sender.read_pot_angle_deg()
            if raw is None:
                return None
            self.latest_hitch_raw_deg = raw
        return raw

    def _run_hitch_calibration(self) -> None:
        if self.sender is None:
            QMessageBox.warning(self, "Calibration", "Connect to the ESP first.")
            return

        captured: list[float] = []
        for message in hitch_calibration_prompts():
            response = QMessageBox.question(
                self,
                "Calibration",
                message,
                QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel,
            )
            if response != QMessageBox.StandardButton.Ok:
                return
            measured = self._capture_hitch_measurement()
            if measured is None:
                QMessageBox.warning(
                    self,
                    "Calibration",
                    "No hitch telemetry received. Move the pot and try again.",
                )
                return
            captured.append(measured)

        neg, zero_first, pos, zero_return = captured
        try:
            self.hitch_pot = fit_hitch_calibration(neg, zero_first, pos, zero_return)
        except ValueError:
            QMessageBox.warning(
                self,
                "Calibration",
                "Captured points are not ordered. Please repeat calibration.",
            )
            return

        zero = self.hitch_pot.meas_zero
        if self.latest_hitch_raw_deg is not None:
            self._update_hitch_angle_display(self.latest_hitch_raw_deg)
        QMessageBox.information(
            self,
            "Calibration",
            (
                "Hitch calibration updated.\n"
                f"Zero hysteresis compensation applied using both zero captures:\n"
                f"first={zero_first:.2f}, return={zero_return:.2f}, center={zero:.2f}"
            ),
        )

    def _reset_manual_stats(self) -> None:
        self.manual_left_rpm.setText("0.0")
        self.manual_right_rpm.setText("0.0")
        self.auto_left_rpm.setText("0.0")
        self.auto_right_rpm.setText("0.0")

    def _update_manual_stats(self, rpm_l: float, rpm_r: float) -> None:
        self.manual_left_rpm.setText(f"{rpm_l:.1f}")
        self.manual_right_rpm.setText(f"{rpm_r:.1f}")
        self.auto_left_rpm.setText(f"{rpm_l:.1f}")
        self.auto_right_rpm.setText(f"{rpm_r:.1f}")

    def _is_manual_mode(self) -> bool:
        return self.mode_stack.currentIndex() == 0

    def _on_mode_changed(self, mode_id: int) -> None:
        self.mode_stack.setCurrentIndex(mode_id)
        self.header.set_mode("MANUAL" if mode_id == 0 else "AUTOMATIC")
        if mode_id == 0:
            self.auto.stop()
            self._reset_manual_stats()
            self.auto_status_label.setText("MPC: Idle")
            self._update_button_states()
        else:
            self._set_motion("STOP")
            self.auto_status_label.setText("MPC: Idle")
            self._update_button_states()

    def _auto_start(self) -> None:
        if self.sender is None:
            return
        if self.vision.vision_q is None or self.vision.goal_xy is None:
            self.auto_status_label.setText("MPC: Need vision lock (truck, trailer, refs)")
            return
        motor_rpms = self.sender.read_motor_rpms()
        if self.auto.start(self.vision.vision_q, self.vision.goal_xy, motor_rpms):
            self.auto_state_view.reset_path(self.auto.q)
            self._reset_manual_stats()
            self.auto_status_label.setText("MPC: Running")
            self._update_button_states()

    def _auto_stop(self) -> None:
        self.auto.stop()
        self.auto_status_label.setText("MPC: Stopped")
        self._update_button_states()

    def _select_port(self, port: str) -> None:
        idx = self.port_box.findText(port)
        if idx >= 0:
            self.port_box.setCurrentIndex(idx)
        else:
            self.port_box.addItem(port)
            self.port_box.setCurrentIndex(self.port_box.count() - 1)

    def _refresh_ports(self) -> None:
        try:
            from serial.tools import list_ports  # type: ignore
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []

        current = self.port_box.currentText().strip()
        self.port_box.clear()
        self.port_box.addItems(ports)
        if current:
            self._select_port(current)

    def _refresh_cameras(self) -> None:
        import cv2
        current = self.camera_source_box.currentData()
        self.camera_source_box.clear()
        self.camera_source_box.addItem("Configured camera", "configured")
        for idx in range(5):
            cap = cv2.VideoCapture(idx)
            opened = cap.isOpened()
            cap.release()
            if opened:
                self.camera_source_box.addItem(f"Camera {idx}", idx)

        if current is not None:
            for i in range(self.camera_source_box.count()):
                if self.camera_source_box.itemData(i) == current:
                    self.camera_source_box.setCurrentIndex(i)
                    break

    def _set_camera_connected(self, connected: bool) -> None:
        self.camera_connect_btn.setEnabled(not connected)
        self.camera_disconnect_btn.setEnabled(connected)
        self.camera_source_box.setEnabled(not connected)

    def _connect_camera(self) -> None:
        source = self.camera_source_box.currentData()
        status = self.vision.connect(source)
        if "Connected" in status:
            self.camera_timer.start(33)
            self._set_camera_connected(True)
        self.camera_status_label.setText(f"Camera: {status}")

    def _disconnect_camera(self) -> None:
        self.camera_timer.stop()
        self.vision.disconnect()
        self.camera_view.setText("No camera feed")
        self.camera_view.setPixmap(QPixmap())
        self.camera_status_label.setText("Camera: Disconnected")
        self._update_hitch_vision_display(None)
        self._set_camera_connected(False)

    def _tick_camera(self) -> None:
        if self.auto.running:
            pred_for_cam = self.auto.pred_path_xy_cm
        elif self.vision.vision_q is not None and self.vision.goal_xy is not None:
            self.auto.preview_parking_path(self.vision.vision_q, self.vision.goal_xy)
            pred_for_cam = self.auto.preview_pred_path_xy_cm
        else:
            pred_for_cam = None
        result = self.vision.tick(pred_for_cam)
        if result is None:
            return
        view, found_count, hitch_deg = result
        self._update_hitch_vision_display(hitch_deg)
        self.camera_status_label.setText(f"Camera: Connected | Markers: {found_count}")

        if view is not None and cv2 is not None:
            rgb = cv2.cvtColor(view, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(image).scaled(
                self.camera_view.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
            self.camera_view.setPixmap(pixmap)

        if self.vision.vision_q is not None:
            vq = self.vision.vision_q
            if self.vision.goal_xy is not None:
                g = self.vision.goal_xy
                self.auto_state_view.set_goal(
                    np.array(
                        [
                            float(g[0]),
                            float(g[1]),
                            float(vq[2]),
                            float(vq[3]),
                            0.0,
                            0.0,
                        ],
                        dtype=float,
                    )
                )
            else:
                self.auto_state_view.clear_goal()
            if self.auto.running:
                self.auto_state_view.set_state(vq)
                self.auto_state_view.set_pred_path(
                    [(float(p[0]), float(p[1])) for p in self.auto.pred_path_xy_cm]
                )
            else:
                self.auto_state_view.reset_path(vq)
                self.auto_state_view.set_pred_path(
                    [
                        (float(p[0]), float(p[1]))
                        for p in self.auto.preview_pred_path_xy_cm
                    ]
                )

    def _set_motion(self, motion: str) -> None:
        self.current_motion = motion

    def _read_settings(self) -> tuple[str, float]:
        port = self.port_box.currentText().strip()
        if not port:
            raise ValueError("Port is required")
        rpm = float(self.speed_slider.value())
        if rpm < 0:
            raise ValueError("Base RPM must be >= 0")
        return port, rpm

    def _connect(self) -> None:
        if self.sender is not None:
            return

        try:
            port, _rpm = self._read_settings()
            validate_sender_config(hz=params.DEFAULT_HZ)
            sender = UartPacketSender(port=port, baud=params.DEFAULT_BAUD)
            sender.connect()
        except Exception:
            return

        self.sender = sender
        self.current_motion = "STOP"
        self._reset_manual_stats()
        self._set_connected(True)

        interval_ms = max(1, int(1000.0 / params.DEFAULT_HZ))
        self.send_timer.start(interval_ms)
        self.setFocus()

    def _disconnect(self) -> None:
        sender = self.sender
        if sender is None:
            return

        self.send_timer.stop()
        self.current_motion = "STOP"
        try:
            sender.send_targets(0.0, 0.0)
        except Exception:
            pass
        sender.close()
        self.sender = None

        self._set_connected(False)

    def _set_connected(self, connected: bool) -> None:
        if not connected:
            self.auto.stop()
            self.auto_status_label.setText("MPC: Idle")
            self._update_hitch_angle_display(0.0)
            self._reset_manual_stats()
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self._update_button_states()
        self.header.set_connected(connected)

    def _update_button_states(self) -> None:
        connected = self.sender is not None
        for btn in self.drive_buttons:
            btn.setEnabled(connected)
        self.auto_start_btn.setEnabled(connected and not self.auto.running)
        self.auto_stop_btn.setEnabled(connected and self.auto.running)

    def _tick_send(self) -> None:
        sender = self.sender
        if sender is None:
            return

        pot_angle_deg = sender.read_pot_angle_deg()
        if pot_angle_deg is not None:
            self.latest_hitch_raw_deg = pot_angle_deg
            self._update_hitch_angle_display(pot_angle_deg)
        motor_rpms = sender.read_motor_rpms()
        if motor_rpms is not None:
            self._update_manual_stats(motor_rpms[0], motor_rpms[1])

        if self.mode_stack.currentIndex() == 1:
            if self.auto.running:
                rpm_l, rpm_r = self.auto.tick(self.vision.vision_q, self.vision.goal_xy, motor_rpms)
                self.auto_status_label.setText("MPC: Running")
                if self.auto.reached_goal():
                    self.auto.stop()
                    self.auto_status_label.setText("MPC: Finished")
                    self._update_button_states()
            else:
                rpm_l, rpm_r = 0.0, 0.0
        else:
            base_rpm = float(self.speed_slider.value())
            rpm_l, rpm_r = motion_to_rpms(self.current_motion, base_rpm)
        try:
            sender.send_targets(rpm_l, rpm_r)
        except Exception:
            self._disconnect()

    def keyPressEvent(self, event) -> None:
        if not self._is_manual_mode():
            super().keyPressEvent(event)
            return
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == Qt.Key.Key_Up:
            self._set_motion("FORWARD")
        elif key == Qt.Key.Key_Down:
            self._set_motion("BACK")
        elif key == Qt.Key.Key_Left:
            self._set_motion("LEFT")
        elif key == Qt.Key.Key_Right:
            self._set_motion("RIGHT")
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if not self._is_manual_mode():
            super().keyReleaseEvent(event)
            return
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
            self._set_motion("STOP")
        else:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event) -> None:
        self._set_motion("STOP")
        super().focusOutEvent(event)

    def closeEvent(self, event) -> None:
        self._disconnect()
        self._disconnect_camera()
        super().closeEvent(event)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Truck UART Teleoperation GUI")
    parser.add_argument("--port", default="", help="Serial port for UART (e.g. /dev/ttyACM0)")
    parser.add_argument("--rpm", type=float, default=80, help="Initial base RPM (0-120)")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    app = QApplication([])
    app.setStyle("Fusion")
    window = TruckControlGui(args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
