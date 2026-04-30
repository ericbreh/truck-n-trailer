#!/usr/bin/env python3

import argparse
import sys

try:
    from PyQt6.QtCore import Qt, QTimer
    from PyQt6.QtWidgets import (
        QApplication,
        QComboBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyQt6 is required. Install with: pip install PyQt6") from exc

from .uart_sender import UartPacketSender, validate_sender_config


DEFAULT_BAUD = 115200
DEFAULT_HZ = 10.0


def motion_targets(motion: str, base_rpm: float) -> tuple[float, float]:
    if motion == "FORWARD":
        return base_rpm, base_rpm
    if motion == "BACK":
        return -base_rpm, -base_rpm
    if motion == "LEFT":
        return -base_rpm, base_rpm
    if motion == "RIGHT":
        return base_rpm, -base_rpm
    return 0.0, 0.0


class TruckControlGui(QWidget):
    def __init__(self, args: argparse.Namespace):
        super().__init__()
        self.setWindowTitle("Truck UART Teleop")
        self.resize(440, 340)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        
        # Apply the Blue Theme
        self.setStyleSheet(self._get_blue_stylesheet())

        self.sender: UartPacketSender | None = None
        self.current_motion = "STOP"

        self.port_box = QComboBox()
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 120)
        self.speed_slider.setValue(int(args.rpm))
        self.speed_value = QLabel(f"{int(args.rpm)} RPM")

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.refresh_btn = QPushButton("Refresh")

        self.forward_btn = QPushButton("Forward")
        self.back_btn = QPushButton("Back")
        self.left_btn = QPushButton("Left")
        self.right_btn = QPushButton("Right")
        self.drive_buttons = [
            self.forward_btn,
            self.back_btn,
            self.left_btn,
            self.right_btn,
        ]

        self.send_timer = QTimer(self)
        self.send_timer.timeout.connect(self._tick_send)

        self._build_ui()
        self._bind_controls()
        self._refresh_ports()
        if args.port:
            self._select_port(args.port)
        self._set_connected(False)

    def _get_blue_stylesheet(self) -> str:
        """Returns the CSS logic for the blue theme."""
        return """
            QWidget {
                background-color: #121826;
                color: #e0e6ed;
                font-family: 'Segoe UI', sans-serif;
                font-size: 13px;
            }
            QGroupBox {
                border: 2px solid #1e293b;
                border-radius: 8px;
                margin-top: 12px;
                font-weight: bold;
                color: #38bdf8;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #1e293b;
                border: 1px solid #38bdf8;
                border-radius: 5px;
                padding: 8px;
                min-width: 70px;
                color: #f8fafc;
            }
            QPushButton:hover {
                background-color: #334155;
            }
            QPushButton:pressed {
                background-color: #0ea5e9;
                color: #ffffff;
            }
            QPushButton:disabled {
                background-color: #0f172a;
                border: 1px solid #1e293b;
                color: #475569;
            }
            QComboBox {
                background-color: #1e293b;
                border: 1px solid #38bdf8;
                border-radius: 3px;
                padding: 2px 5px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #1e293b;
                height: 6px;
                background: #0f172a;
                margin: 2px 0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #38bdf8;
                border: 1px solid #0ea5e9;
                width: 16px;
                height: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
        """

    def _build_ui(self) -> None:
        root = QVBoxLayout()
        self.setLayout(root)

        conn_group = QGroupBox("Connection")
        conn_form = QFormLayout()
        conn_group.setLayout(conn_form)
        conn_form.addRow("Port", self.port_box)

        speed_row = QHBoxLayout()
        speed_row.addWidget(self.speed_slider)
        speed_row.addWidget(self.speed_value)
        conn_form.addRow("Base RPM", speed_row)

        conn_buttons = QHBoxLayout()
        conn_buttons.addWidget(self.refresh_btn)
        conn_buttons.addWidget(self.connect_btn)
        conn_buttons.addWidget(self.disconnect_btn)
        conn_form.addRow(conn_buttons)

        drive_group = QGroupBox("Drive")
        drive_grid = QGridLayout()
        drive_group.setLayout(drive_grid)
        drive_grid.addWidget(self.forward_btn, 0, 1)
        drive_grid.addWidget(self.left_btn, 1, 0)
        drive_grid.addWidget(self.right_btn, 1, 2)
        drive_grid.addWidget(self.back_btn, 2, 1)

        root.addWidget(conn_group)
        root.addWidget(drive_group)

    def _bind_controls(self) -> None:
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

    def _update_speed_label(self, value: int) -> None:
        self.speed_value.setText(f"{value} RPM")

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

    def _set_connected(self, connected: bool) -> None:
        for btn in self.drive_buttons:
            btn.setEnabled(connected)
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)

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
            validate_sender_config(hz=DEFAULT_HZ)
            sender = UartPacketSender(port=port, baud=DEFAULT_BAUD)
            sender.connect()
        except Exception:
            return

        self.sender = sender
        self.current_motion = "STOP"
        self._set_connected(True)

        interval_ms = max(1, int(1000.0 / DEFAULT_HZ))
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

    def _tick_send(self) -> None:
        sender = self.sender
        if sender is None:
            return

        base_rpm = float(self.speed_slider.value())
        rpm_l, rpm_r = motion_targets(self.current_motion, base_rpm)
        try:
            sender.send_targets(rpm_l, rpm_r)
        except Exception:
            self._disconnect()

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
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
        elif key == Qt.Key.Key_Space:
            self._set_motion("STOP")

    def keyReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
            self._set_motion("STOP")

    def focusOutEvent(self, event) -> None:  # type: ignore[override]
        self._set_motion("STOP")
        super().focusOutEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._disconnect()
        super().closeEvent(event)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PyQt UART drive GUI")
    parser.add_argument("--port", default="", help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--rpm", type=float, default=30.0, help="Initial base RPM")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    app = QApplication(sys.argv)
    
    # Use Fusion to ensure custom blue styling is applied properly
    app.setStyle("Fusion")
    
    window = TruckControlGui(args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())