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
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyQt6 is required. Install with: pip install PyQt6") from exc

from .uart_sender import UartPacketSender, validate_sender_config


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
        self.resize(560, 440)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.sender: UartPacketSender | None = None
        self.current_motion = "STOP"
        self.base_rpm = args.rpm

        self.port_box = QComboBox()
        self.baud_edit = QLineEdit(str(args.baud))
        self.hz_edit = QLineEdit(f"{args.hz}")
        self.ttl_edit = QLineEdit(str(args.ttl_ms))
        self.rpm_edit = QLineEdit(f"{args.rpm}")

        self.connection_label = QLabel("Disconnected")
        self.motion_label = QLabel("STOP")
        self.info_label = QLabel("Idle")
        self.packet_label = QLabel("")

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.refresh_btn = QPushButton("Refresh")

        self.forward_btn = QPushButton("Forward")
        self.back_btn = QPushButton("Back")
        self.left_btn = QPushButton("Left")
        self.right_btn = QPushButton("Right")
        self.stop_btn = QPushButton("Stop")
        self.drive_buttons = [
            self.forward_btn,
            self.back_btn,
            self.left_btn,
            self.right_btn,
            self.stop_btn,
        ]

        self.send_timer = QTimer(self)
        self.send_timer.timeout.connect(self._tick_send)

        self._build_ui()
        self._bind_controls()
        self._refresh_ports()
        if args.port:
            self._select_port(args.port)
        self._set_connected(False)

    def _build_ui(self) -> None:
        root = QVBoxLayout()
        self.setLayout(root)

        conn_group = QGroupBox("Connection")
        conn_form = QFormLayout()
        conn_group.setLayout(conn_form)
        conn_form.addRow("Port", self.port_box)
        conn_form.addRow("Baud", self.baud_edit)
        conn_form.addRow("Hz", self.hz_edit)
        conn_form.addRow("TTL ms", self.ttl_edit)
        conn_form.addRow("Base RPM", self.rpm_edit)

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
        drive_grid.addWidget(self.stop_btn, 1, 1)
        drive_grid.addWidget(self.right_btn, 1, 2)
        drive_grid.addWidget(self.back_btn, 2, 1)

        status_group = QGroupBox("Status")
        status_form = QFormLayout()
        status_group.setLayout(status_form)
        status_form.addRow("Connection", self.connection_label)
        status_form.addRow("Motion", self.motion_label)
        status_form.addRow("Info", self.info_label)
        status_form.addRow("Last packet", self.packet_label)

        root.addWidget(conn_group)
        root.addWidget(drive_group)
        root.addWidget(status_group)

    def _bind_controls(self) -> None:
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._connect)
        self.disconnect_btn.clicked.connect(self._disconnect)

        self.forward_btn.pressed.connect(lambda: self._set_motion("FORWARD"))
        self.forward_btn.released.connect(lambda: self._set_motion("STOP"))
        self.back_btn.pressed.connect(lambda: self._set_motion("BACK"))
        self.back_btn.released.connect(lambda: self._set_motion("STOP"))
        self.left_btn.pressed.connect(lambda: self._set_motion("LEFT"))
        self.left_btn.released.connect(lambda: self._set_motion("STOP"))
        self.right_btn.pressed.connect(lambda: self._set_motion("RIGHT"))
        self.right_btn.released.connect(lambda: self._set_motion("STOP"))
        self.stop_btn.clicked.connect(lambda: self._set_motion("STOP"))

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
        self.connection_label.setText("Connected" if connected else "Disconnected")

    def _set_motion(self, motion: str) -> None:
        self.current_motion = motion
        self.motion_label.setText(motion)

    def _read_settings(self) -> tuple[str, int, float, int, float]:
        port = self.port_box.currentText().strip()
        if not port:
            raise ValueError("Port is required")

        baud = int(self.baud_edit.text())
        hz = float(self.hz_edit.text())
        ttl_ms = int(self.ttl_edit.text())
        rpm = float(self.rpm_edit.text())

        if baud <= 0:
            raise ValueError("Baud must be > 0")
        if rpm < 0:
            raise ValueError("Base RPM must be >= 0")

        validate_sender_config(hz=hz, ttl_ms=ttl_ms)
        return port, baud, hz, ttl_ms, rpm

    def _connect(self) -> None:
        if self.sender is not None:
            return

        try:
            port, baud, hz, ttl_ms, rpm = self._read_settings()
            sender = UartPacketSender(port=port, baud=baud, ttl_ms=ttl_ms)
            sender.connect()
        except Exception as exc:
            self.info_label.setText(f"Connect failed: {exc}")
            return

        self.sender = sender
        self.base_rpm = rpm
        self._set_motion("STOP")
        self._set_connected(True)
        self.info_label.setText(f"Sending at {hz:.1f} Hz, base RPM {rpm:.1f}")

        interval_ms = max(1, int(1000.0 / hz))
        self.send_timer.start(interval_ms)
        self.setFocus()

    def _disconnect(self) -> None:
        sender = self.sender
        if sender is None:
            return

        self.send_timer.stop()
        self._set_motion("STOP")
        try:
            sender.send_targets(0.0, 0.0)
        except Exception:
            pass
        sender.close()
        self.sender = None

        self._set_connected(False)
        self.info_label.setText("Disconnected")
        self.packet_label.setText("")

    def _tick_send(self) -> None:
        sender = self.sender
        if sender is None:
            return

        rpm_l, rpm_r = motion_targets(self.current_motion, self.base_rpm)
        try:
            packet = sender.send_targets(rpm_l, rpm_r).strip()
            self.packet_label.setText(packet)
        except Exception as exc:
            self.info_label.setText(f"Serial error: {exc}")
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
    parser = argparse.ArgumentParser(description="PyQt UART drive GUI for truck teleop")
    parser.add_argument("--port", default="", help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--hz", type=float, default=10.0, help="Send rate in Hz")
    parser.add_argument("--ttl-ms", type=int, default=500, help="Packet TTL in milliseconds")
    parser.add_argument("--rpm", type=float, default=30.0, help="Base RPM for directional buttons")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    app = QApplication(sys.argv)
    window = TruckControlGui(args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
