import os as _os
import sys as _sys

# Allow `python app.py` in addition to `python -m truck_n_trailer.gui`
if __package__ is None or __package__ == "":
    _repo_root = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    if _repo_root not in _sys.path:
        _sys.path.insert(0, _repo_root)
    __package__ = "truck_n_trailer.gui"

import argparse
import math
import time

import numpy as np

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

from .constants import (
    DEFAULT_BAUD, DEFAULT_HZ,
    AUTO_WHEEL_RADIUS_CM, AUTO_WHEEL_TRACK_CM,
    AUTO_TRUCK_LENGTH_CM, AUTO_TRAILER_LENGTH_CM,
    AUTO_START_YAW_RAD,
    AUTO_HITCH_HARD_LIMIT_DEG, AUTO_HITCH_RELEASE_DEG,
    AUTO_HITCH_RECOVERY_RPM, AUTO_MIN_EFFECTIVE_RPM,
    HITCH_REAL_ZERO, HITCH_REAL_NEG_WORKING, HITCH_REAL_POS_WORKING,
    HITCH_MEAS_ZERO, HITCH_MEAS_NEG_WORKING, HITCH_MEAS_POS_WORKING,
    MARKER_SIZE_CM, WORKSPACE_BOX_SIDE_INSET_CM, WORKSPACE_BOX_VERTICAL_EXTEND_CM,
    CAMERA_FOCUS_RELOCK_FRAMES,
)
from .widgets import AutoStateView, HitchGauge, TelemetryCard, StatusBadge, DashboardHeader
from .stylesheet import get_blue_stylesheet
from truck_n_trailer.control.mpc import MPCConfig, TruckTrailerMPC
from truck_n_trailer.parking.body_to_wheels import BodyToWheels
from truck_n_trailer.uart import UartPacketSender, validate_sender_config

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None

try:
    from truck_n_trailer.vision.camera import open_configured_camera
except Exception:
    open_configured_camera = None

try:
    from truck_n_trailer.vision.config import (
        ARUCO_DICT,
        ARUCO_PARAMS,
        TRACKING_MARKER_IDS,
        REFERENCE_MARKER_IDS,
    )
    from truck_n_trailer.vision.detect import load_calibration, marker_pose_world, draw_world_plane
except Exception:
    ARUCO_DICT = None
    ARUCO_PARAMS = None
    TRACKING_MARKER_IDS = {}
    REFERENCE_MARKER_IDS = {}
    load_calibration = None
    marker_pose_world = None
    draw_world_plane = None

try:
    from truck_n_trailer.vision import overlay as _overlay
except Exception:
    _overlay = None


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
        self.resize(1280, 800)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setStyleSheet(get_blue_stylesheet())

        self.header = DashboardHeader(self)

        self.sender: UartPacketSender | None = None
        self.current_motion = "STOP"
        self.manual_distance_cm = 0.0
        self.latest_hitch_raw_deg: float | None = None
        self.latest_hitch_display_deg = 0.0
        self.hitch_meas_neg_working = HITCH_MEAS_NEG_WORKING
        self.hitch_meas_zero = HITCH_MEAS_ZERO
        self.hitch_meas_pos_working = HITCH_MEAS_POS_WORKING
        self.auto_running = False
        self.auto_mpc = self._build_auto_mpc()
        self.auto_q = self.auto_mpc.cfg.q0.copy().astype(float)
        self.auto_u_guess = np.zeros((2, self.auto_mpc.cfg.N), dtype=float)
        self.auto_wheels = BodyToWheels(
            wheel_track_cm=AUTO_WHEEL_TRACK_CM,
            wheel_radius_cm=AUTO_WHEEL_RADIUS_CM,
            rpm_limit=120.0,
        )
        self.auto_solver_fail_count = 0
        self.auto_hitch_recovery_active = False
        self.auto_last_meas_rpms: tuple[float, float] | None = None
        self.auto_last_meas_hitch_deg: float | None = None

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
        self.auto_stop_btn = QPushButton("Stop")
        self.auto_status_label = StatusBadge("MPC: Idle")
        self.auto_state_view = AutoStateView(
            truck_len_cm=AUTO_TRUCK_LENGTH_CM,
            trailer_len_cm=AUTO_TRAILER_LENGTH_CM,
        )
        self.hitch_angle_gauge = HitchGauge("POT")
        self.hitch_vision_gauge = HitchGauge("VISION")
        self.calibrate_btn = QPushButton("Calibration")
        self.manual_rpm_value = TelemetryCard("0.0", "RPM")
        self.manual_speed_value = TelemetryCard("0.00", "Speed cm/s")
        self.manual_distance_value = TelemetryCard("0.00", "Distance cm")
        self.auto_rpm_value = TelemetryCard("0.0", "RPM")
        self.auto_speed_value = TelemetryCard("0.00", "Speed cm/s")
        self.auto_distance_value = TelemetryCard("0.00", "Distance cm")

        self.camera_cap = None
        self.camera_detector = None
        self.camera_K = None
        self.camera_dist = None
        self.camera_H = None
        self.camera_H_inv = None
        self.camera_frame_counter = 0
        self.reference_marker_corners_cache: dict[int, np.ndarray] = {}
        self.auto_vision_q: np.ndarray | None = None
        self.auto_goal_xy: np.ndarray | None = None
        self.auto_prev_vision_t: float | None = None
        self.auto_prev_pivot_xy: np.ndarray | None = None
        self.auto_prev_truck_heading: float | None = None
        self.latest_hitch_vision_deg: float | None = None
        self.auto_vel_ema_cm_s = 0.0
        self.auto_omega_ema_rad_s = 0.0
        self.auto_pred_path_xy_cm: list[np.ndarray] = []
        self.auto_axis_origin_px: np.ndarray | None = None
        self.auto_axis_x_hat_px: np.ndarray | None = None
        self.auto_axis_y_hat_px: np.ndarray | None = None
        self.auto_axis_px_per_cm: float | None = None
        self.camera_source_box = QComboBox()
        self.camera_refresh_btn = QPushButton("Refresh")
        self.camera_connect_btn = QPushButton("Connect")
        self.camera_disconnect_btn = QPushButton("Disconnect")
        self.camera_status_label = QLabel("Camera: Disconnected")
        self.camera_view = QLabel("No camera feed")
        self.camera_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_view.setMinimumSize(480, 320)
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
        root.addLayout(left_col, stretch=3)
        root.addLayout(right_col, stretch=2)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        mode_row.addWidget(self.mode_manual_radio)
        mode_row.addWidget(self.mode_auto_radio)
        mode_row.addStretch()
        left_col.addLayout(mode_row)

        conn_group = QGroupBox("Connection")
        conn_form = QFormLayout()
        conn_group.setLayout(conn_form)
        conn_form.addRow("Port", self.port_box)

        conn_buttons = QHBoxLayout()
        conn_buttons.addWidget(self.refresh_btn)
        conn_buttons.addWidget(self.connect_btn)
        conn_buttons.addWidget(self.disconnect_btn)
        conn_form.addRow(conn_buttons)

        left_col.addWidget(conn_group)

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
        manual_telemetry_row = QHBoxLayout(manual_telemetry_group)
        manual_telemetry_row.setSpacing(6)
        manual_telemetry_row.addWidget(self.manual_rpm_value)
        manual_telemetry_row.addWidget(self.manual_speed_value)
        manual_telemetry_row.addWidget(self.manual_distance_value)
        manual_layout.addWidget(manual_telemetry_group)

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
        auto_btns = QHBoxLayout()
        auto_btns.addWidget(self.auto_start_btn)
        auto_btns.addWidget(self.auto_stop_btn)
        auto_layout.addLayout(auto_btns)
        auto_layout.addWidget(self.auto_status_label)
        auto_layout.addWidget(self.auto_state_view, stretch=1)

        auto_telemetry_group = QGroupBox("Telemetry")
        auto_telemetry_row = QHBoxLayout(auto_telemetry_group)
        auto_telemetry_row.setSpacing(6)
        auto_telemetry_row.addWidget(self.auto_rpm_value)
        auto_telemetry_row.addWidget(self.auto_speed_value)
        auto_telemetry_row.addWidget(self.auto_distance_value)
        auto_layout.addWidget(auto_telemetry_group)

        cam_group = QGroupBox("Camera Connection")
        cam_form = QFormLayout(cam_group)
        cam_form.addRow("Source", self.camera_source_box)
        cam_buttons = QHBoxLayout()
        cam_buttons.addWidget(self.camera_refresh_btn)
        cam_buttons.addWidget(self.camera_connect_btn)
        cam_buttons.addWidget(self.camera_disconnect_btn)
        cam_form.addRow(cam_buttons)

        self.mode_stack.addWidget(manual_page)
        self.mode_stack.addWidget(auto_page)
        left_col.addWidget(self.mode_stack, stretch=1)
        right_col.addWidget(cam_group)
        right_col.addWidget(self.camera_status_label)
        right_col.addWidget(self.camera_view, stretch=1)

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

    def _build_auto_mpc(self) -> TruckTrailerMPC:
        q0 = np.array([20.0, 155.0, AUTO_START_YAW_RAD, AUTO_START_YAW_RAD, 0.0, 0.0], dtype=float)
        q_des = np.array([20.0, 20.0, AUTO_START_YAW_RAD, AUTO_START_YAW_RAD, 0.0, 0.0], dtype=float)
        cfg = MPCConfig(
            L=AUTO_TRUCK_LENGTH_CM,
            d=AUTO_TRAILER_LENGTH_CM,
            dt=1.0 / DEFAULT_HZ,
            N=30,
            max_steps=500,
            target_tol=2.0,
            angle_tol=0.15,
            a_min=-10.0,
            a_max=10.0,
            v_min=-20.0,
            v_max=20.0,
            max_jackknife_angle=math.radians(45.0),
            q0=q0,
            q_des=q_des,
            w_theta_t=20.0,
            w_theta_l=40.0,
            w_v=15.0,
            w_omega=15.0,
            w_pos_stage=0.05,
        )
        return TruckTrailerMPC(cfg)

    def _auto_reset_state(self) -> None:
        self.auto_q = self.auto_mpc.cfg.q0.copy().astype(float)
        self.auto_u_guess = np.zeros((2, self.auto_mpc.cfg.N), dtype=float)
        self.auto_wheels.reset(v=float(self.auto_q[4]), omega=float(self.auto_q[5]))
        self.auto_solver_fail_count = 0
        self.auto_hitch_recovery_active = False
        self.auto_last_meas_rpms = None
        self.auto_last_meas_hitch_deg = None
        self.auto_state_view.set_goal(self.auto_mpc.cfg.q_des)
        self.auto_state_view.reset_path(self.auto_q)
        self.auto_prev_vision_t = None
        self.auto_prev_pivot_xy = None
        self.auto_prev_truck_heading = None
        self.auto_vel_ema_cm_s = 0.0
        self.auto_omega_ema_rad_s = 0.0
        self.auto_pred_path_xy_cm = []

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _auto_reached_goal(self) -> bool:
        cfg = self.auto_mpc.cfg
        pos_err = float(np.linalg.norm(self.auto_q[:2] - cfg.q_des[:2]))
        ang_t = abs(self._wrap_angle(float(self.auto_q[2] - cfg.q_des[2])))
        ang_l = abs(self._wrap_angle(float(self.auto_q[3] - cfg.q_des[3])))
        return pos_err <= cfg.target_tol and ang_t <= cfg.angle_tol and ang_l <= cfg.angle_tol

    def _simulate_step(self, q: np.ndarray, u: np.ndarray, dt: float, d: float) -> np.ndarray:
        x, y, theta_t, theta_l, v, omega = [float(val) for val in q]
        a, alpha = [float(val) for val in u]
        q_next = np.array(
            [
                x + dt * v * math.cos(theta_t),
                y + dt * v * math.sin(theta_t),
                theta_t + dt * omega,
                theta_l + dt * (v / d) * math.sin(theta_t - theta_l),
                v + dt * a,
                omega + dt * alpha,
            ],
            dtype=float,
        )
        q_next[2] = self._wrap_angle(float(q_next[2]))
        q_next[3] = self._wrap_angle(float(q_next[3]))
        return q_next

    def _auto_tick(self) -> tuple[float, float]:
        if self.auto_vision_q is None or self.auto_goal_xy is None:
            self.auto_status_label.setText("MPC: Waiting for vision state")
            self.auto_pred_path_xy_cm = []
            return 0.0, 0.0

        self.auto_q = self.auto_vision_q.copy()
        q_des = self.auto_mpc.cfg.q_des.copy()
        q_des[0] = float(self.auto_goal_xy[0])
        q_des[1] = float(self.auto_goal_xy[1])
        q_des[2] = float(self.auto_q[2])
        q_des[3] = float(self.auto_q[3])
        q_des[4] = 0.0
        q_des[5] = 0.0
        self.auto_mpc.cfg.q_des = q_des
        self.auto_state_view.set_goal(self.auto_mpc.cfg.q_des)

        if self._auto_reached_goal():
            self.auto_running = False
            self.auto_status_label.setText("MPC: Finished")
            self._update_button_states()
            self.auto_pred_path_xy_cm = []
            return 0.0, 0.0

        cfg = self.auto_mpc.cfg
        if float(self.auto_q[4]) <= cfg.v_min + 1e-6:
            self.auto_u_guess[0, :] = np.maximum(self.auto_u_guess[0, :], 0.0)
        elif float(self.auto_q[4]) >= cfg.v_max - 1e-6:
            self.auto_u_guess[0, :] = np.minimum(self.auto_u_guess[0, :], 0.0)

        u_opt = self.auto_mpc.solve(self.auto_q, self.auto_u_guess)
        if u_opt is None:
            retry_guess = np.zeros_like(self.auto_u_guess)
            u_opt = self.auto_mpc.solve(self.auto_q, retry_guess)
            if u_opt is None:
                self.auto_solver_fail_count += 1
                fallback_u = np.array([0.0, 0.0], dtype=float)
                self.auto_q = self._simulate_step(self.auto_q, fallback_u, cfg.dt, cfg.d)
                self.auto_state_view.set_state(self.auto_q)
                self.auto_u_guess[:, :] = 0.0
                rpm_l, rpm_r = self.auto_wheels.convert(a=0.0, alpha=0.0, dt=cfg.dt)
                self.auto_status_label.setText(
                    f"MPC: Recovering ({self.auto_solver_fail_count})"
                )
                self.auto_pred_path_xy_cm = []
                print(
                    f"[AUTO] solver failed at q=({self.auto_q[0]:.2f},{self.auto_q[1]:.2f},"
                    f"{self.auto_q[2]:.2f},{self.auto_q[3]:.2f},{self.auto_q[4]:.2f},{self.auto_q[5]:.2f}); "
                    "using fallback u=(0,0)"
                )
                return rpm_l, rpm_r
            self.auto_u_guess = retry_guess

        u0 = u_opt[:, 0]
        self.auto_solver_fail_count = 0
        self.auto_status_label.setText("MPC: Running")
        rpm_l, rpm_r = self.auto_wheels.convert(a=float(u0[0]), alpha=float(u0[1]), dt=cfg.dt)
        pred_steps = max(1, min(u_opt.shape[1], int(round(3.0 / cfg.dt))))
        q_pred = self.auto_q.copy()
        pred_xy_cm: list[np.ndarray] = [q_pred[:2].copy()]
        for k in range(pred_steps):
            q_pred = self._simulate_step(q_pred, u_opt[:, k], cfg.dt, cfg.d)
            pred_xy_cm.append(q_pred[:2].copy())
        self.auto_pred_path_xy_cm = pred_xy_cm
        self.auto_u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])
        self.auto_state_view.set_state(self.auto_q)
        self.auto_state_view.set_pred_path(
            [(float(p[0]), float(p[1])) for p in pred_xy_cm]
        )
        print(
            f"[AUTO] q=({self.auto_q[0]:.2f},{self.auto_q[1]:.2f},"
            f"{self.auto_q[2]:.2f},{self.auto_q[3]:.2f}) "
            f"u=({float(u0[0]):.2f},{float(u0[1]):.2f}) "
            f"rpm=({rpm_l:.2f},{rpm_r:.2f})"
        )
        return rpm_l, rpm_r

    def _is_manual_mode(self) -> bool:
        return self.mode_stack.currentIndex() == 0

    def _on_mode_changed(self, mode_id: int) -> None:
        self.mode_stack.setCurrentIndex(mode_id)
        self.header.set_mode("MANUAL" if mode_id == 0 else "AUTOMATIC")
        if mode_id == 0:
            self.auto_running = False
            self._reset_manual_stats()
            self.auto_status_label.setText("MPC: Idle")
            self._update_button_states()
        else:
            self._set_motion("STOP")
            self.auto_state_view.set_goal(self.auto_mpc.cfg.q_des)
            self.auto_state_view.reset_path(self.auto_q)
            self.auto_status_label.setText("MPC: Idle")
            self._update_button_states()

    def _auto_start(self) -> None:
        if self.sender is None:
            return
        if self.auto_vision_q is None or self.auto_goal_xy is None:
            self.auto_status_label.setText("MPC: Need vision lock (truck, trailer, refs)")
            return
        self._auto_reset_state()
        self.auto_q = self.auto_vision_q.copy()
        self.auto_state_view.reset_path(self.auto_q)
        self._reset_manual_stats()
        self.auto_running = True
        self.auto_status_label.setText("MPC: Running")
        self._update_button_states()

    def _auto_stop(self) -> None:
        self.auto_running = False
        self.auto_hitch_recovery_active = False
        self.auto_wheels.reset(v=0.0, omega=0.0)
        self.auto_pred_path_xy_cm = []
        self.auto_status_label.setText("MPC: Stopped")
        self._update_button_states()

    def _update_speed_label(self, value: int) -> None:
        self.speed_value.setText(f"{value} RPM")

    def _linearize_hitch_output(self, angle_deg: float) -> float:
        a = float(angle_deg)
        if a <= self.hitch_meas_zero:
            denom = self.hitch_meas_neg_working - self.hitch_meas_zero
            if abs(denom) < 1e-9:
                return a
            t = (a - self.hitch_meas_zero) / denom
            return HITCH_REAL_ZERO + t * (HITCH_REAL_NEG_WORKING - HITCH_REAL_ZERO)
        denom = self.hitch_meas_pos_working - self.hitch_meas_zero
        if abs(denom) < 1e-9:
            return a
        t = (a - self.hitch_meas_zero) / denom
        return HITCH_REAL_ZERO + t * (HITCH_REAL_POS_WORKING - HITCH_REAL_ZERO)

    def _update_hitch_angle_display(self, raw_adc: float) -> None:
        corrected = self._linearize_hitch_output(raw_adc)
        clamped = max(-180.0, min(180.0, float(corrected)))
        self.latest_hitch_display_deg = clamped
        self.hitch_angle_gauge.setValue(clamped)
        self.hitch_angle_gauge.setConnected(True)

    def _update_hitch_vision_display(self, angle_deg: float | None) -> None:
        self.latest_hitch_vision_deg = None if angle_deg is None else float(angle_deg)
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
        return raw  # Return raw ADC directly

    def _run_hitch_calibration(self) -> None:
        if self.sender is None:
            QMessageBox.warning(self, "Calibration", "Connect to the ESP first.")
            return

        steps = [
            (
                f"Move hitch to {HITCH_REAL_NEG_WORKING:+.1f} deg "
                "(truck rotated clockwise/right relative to trailer), then click OK."
            ),
            f"Move hitch to {HITCH_REAL_ZERO:+.1f} deg, then click OK.",
            (
                f"Move hitch to {HITCH_REAL_POS_WORKING:+.1f} deg "
                "(truck rotated counterclockwise/left relative to trailer), then click OK."
            ),
            (
                f"Move hitch back to {HITCH_REAL_ZERO:+.1f} deg, then click OK. "
                "This second zero reading is used to reduce hysteresis."
            ),
        ]
        captured: list[float] = []
        for message in steps:
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
        zero = 0.5 * (zero_first + zero_return)
        if not (neg < zero < pos):
            QMessageBox.warning(
                self,
                "Calibration",
                "Captured points are not ordered. Please repeat calibration.",
            )
            return

        self.hitch_meas_neg_working = neg
        self.hitch_meas_zero = zero
        self.hitch_meas_pos_working = pos

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
        self.manual_distance_cm = 0.0
        self.manual_rpm_value.setText("0.0")
        self.manual_speed_value.setText("0.00")
        self.manual_distance_value.setText("0.00")
        self.auto_rpm_value.setText("0.0")
        self.auto_speed_value.setText("0.00")
        self.auto_distance_value.setText("0.00")

    def _update_manual_stats(self, rpm_l: float, rpm_r: float) -> None:
        avg_rpm = (float(rpm_l) + float(rpm_r)) * 0.5
        cm_s = (avg_rpm * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0
        self.manual_distance_cm += abs(cm_s) * (1.0 / DEFAULT_HZ)

        self.manual_rpm_value.setText(f"{avg_rpm:.1f}")
        self.manual_speed_value.setText(f"{cm_s:.2f}")
        self.manual_distance_value.setText(f"{self.manual_distance_cm:.2f}")
        self.auto_rpm_value.setText(f"{avg_rpm:.1f}")
        self.auto_speed_value.setText(f"{cm_s:.2f}")
        self.auto_distance_value.setText(f"{self.manual_distance_cm:.2f}")

    def _auto_update_state_from_measurements(
        self,
        motor_rpms: tuple[float, float],
        hitch_deg_display: float,
    ) -> None:
        cfg = self.auto_mpc.cfg
        dt = cfg.dt
        rpm_l, rpm_r = motor_rpms

        v_l = (float(rpm_l) * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0
        v_r = (float(rpm_r) * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0
        v = 0.5 * (v_l + v_r)
        omega = (v_r - v_l) / AUTO_WHEEL_TRACK_CM

        theta_t = float(self.auto_q[2]) + dt * omega
        theta_t = self._wrap_angle(theta_t)
        x = float(self.auto_q[0]) + dt * v * math.cos(theta_t)
        y = float(self.auto_q[1]) + dt * v * math.sin(theta_t)

        hitch_rad = math.radians(float(hitch_deg_display))
        theta_l = self._wrap_angle(theta_t - hitch_rad)
        self.auto_q = np.array([x, y, theta_t, theta_l, v, omega], dtype=float)

    def _body_twist_to_wheel_rpm(self, v_cm_s: float, omega_rad_s: float) -> tuple[float, float]:
        v_l = float(v_cm_s) - (float(omega_rad_s) * AUTO_WHEEL_TRACK_CM) / 2.0
        v_r = float(v_cm_s) + (float(omega_rad_s) * AUTO_WHEEL_TRACK_CM) / 2.0
        circumference = 2.0 * math.pi * AUTO_WHEEL_RADIUS_CM
        rpm_l = (v_l / circumference) * 60.0
        rpm_r = (v_r / circumference) * 60.0
        return rpm_l, rpm_r

    # ── cv2 overlay helpers (logic lives in vision/overlay.py) ──────────── #

    def _draw_vehicle_box(self, frame, corners, heading_rad,
                          front_extra_cm, rear_extra_cm, side_extra_cm, color) -> None:
        if _overlay is not None:
            _overlay.draw_vehicle_box(frame, corners, front_extra_cm, rear_extra_cm,
                                      side_extra_cm, color, MARKER_SIZE_CM)

    def _draw_truck_pivot_x(self, frame, corners) -> None:
        if _overlay is not None:
            _overlay.draw_truck_pivot_x(frame, corners, MARKER_SIZE_CM)

    def _draw_workspace_box_from_reference_markers(self, frame, id_to_corners) -> None:
        if _overlay is not None:
            _overlay.draw_workspace_box(
                frame, id_to_corners, self.camera_H, self.camera_H_inv,
                WORKSPACE_BOX_SIDE_INSET_CM, WORKSPACE_BOX_VERTICAL_EXTEND_CM, MARKER_SIZE_CM,
            )

    def _draw_auto_prediction_path(self, frame) -> None:
        if _overlay is not None:
            _overlay.draw_prediction_path(
                frame, self.auto_pred_path_xy_cm, self.auto_axis_origin_px,
                self.auto_axis_x_hat_px, self.auto_axis_y_hat_px, self.auto_axis_px_per_cm,
            )

    # ── connection / port helpers ────────────────────────────────────────── #

    def _car_like_recovery_rpms(self) -> tuple[float, float]:
        base_v = -((AUTO_HITCH_RECOVERY_RPM * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0)
        desired_omega = -1.0 if self.latest_hitch_display_deg > 0.0 else 1.0
        max_omega = (2.0 * abs(base_v) / AUTO_WHEEL_TRACK_CM) * 0.9
        omega = desired_omega * max_omega
        return self._body_twist_to_wheel_rpm(base_v, omega)

    def _boost_auto_rpms(self, rpm_l: float, rpm_r: float) -> tuple[float, float]:
        peak = max(abs(rpm_l), abs(rpm_r))
        if peak < 1e-6:
            return 0.0, 0.0
        if peak >= AUTO_MIN_EFFECTIVE_RPM:
            return rpm_l, rpm_r
        scale = AUTO_MIN_EFFECTIVE_RPM / peak
        return rpm_l * scale, rpm_r * scale

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
        if cv2 is None:
            self.camera_source_box.clear()
            self.camera_status_label.setText("Camera: OpenCV not installed")
            return
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

    def _apply_camera_focus_lock(self, cap) -> None:
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_FOCUS, 20)
        except Exception:
            pass

    def _init_vision_detector(self) -> None:
        self.camera_detector = None
        self.camera_K = None
        self.camera_dist = None
        self.camera_H = None
        self.camera_H_inv = None
        if cv2 is None or ARUCO_DICT is None or ARUCO_PARAMS is None:
            return
        try:
            self.camera_detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
        except Exception:
            self.camera_detector = None
            return

        if load_calibration is None:
            return
        try:
            K, dist, H = load_calibration()
            if K is not None and dist is not None and H is not None:
                self.camera_K = K
                self.camera_dist = dist
                self.camera_H = H
                self.camera_H_inv = np.linalg.inv(H)
        except Exception:
            self.camera_K = None
            self.camera_dist = None
            self.camera_H = None
            self.camera_H_inv = None

    def _connect_camera(self) -> None:
        if cv2 is None:
            self.camera_status_label.setText("Camera: OpenCV not installed")
            return
        if self.camera_cap is not None:
            return
        source = self.camera_source_box.currentData()
        if source == "configured" and open_configured_camera is not None:
            cap = open_configured_camera()
        elif source == "configured":
            self.camera_status_label.setText("Camera: vision module unavailable")
            return
        else:
            cap = cv2.VideoCapture(int(source))
            if cap is not None and not cap.isOpened():
                cap.release()
                cap = None

        if cap is None:
            self.camera_status_label.setText("Camera: Failed to connect")
            return

        self.camera_cap = cap
        self._apply_camera_focus_lock(cap)
        self.camera_frame_counter = 0
        self.reference_marker_corners_cache.clear()
        self._init_vision_detector()
        self.camera_timer.start(33)
        self.camera_status_label.setText("Camera: Connected")
        self._set_camera_connected(True)

    def _disconnect_camera(self) -> None:
        if self.camera_cap is None:
            return
        self.camera_timer.stop()
        self.camera_cap.release()
        self.camera_cap = None
        self.camera_detector = None
        self.camera_K = None
        self.camera_dist = None
        self.camera_H = None
        self.camera_H_inv = None
        self.camera_frame_counter = 0
        self.reference_marker_corners_cache.clear()
        self.camera_view.setText("No camera feed")
        self.camera_view.setPixmap(QPixmap())
        self.camera_status_label.setText("Camera: Disconnected")
        self._update_hitch_vision_display(None)
        self._set_camera_connected(False)

    def _tick_camera(self) -> None:
        if cv2 is None:
            return
        cap = self.camera_cap
        if cap is None:
            return
        ok, frame = cap.read()
        if not ok:
            self.camera_status_label.setText("Camera: Frame read failed")
            self._disconnect_camera()
            return
        self.camera_frame_counter += 1
        if self.camera_frame_counter % CAMERA_FOCUS_RELOCK_FRAMES == 0:
            self._apply_camera_focus_lock(cap)
        view = frame
        if self.camera_K is not None and self.camera_dist is not None:
            try:
                view = cv2.undistort(view, self.camera_K, self.camera_dist)
            except Exception:
                pass

        if self.camera_H_inv is not None and draw_world_plane is not None:
            try:
                draw_world_plane(view, self.camera_H_inv)
            except Exception:
                pass

        found_count = 0
        hitch_vision_deg: float | None = None
        if self.camera_detector is not None:
            try:
                corners_list, ids, _ = self.camera_detector.detectMarkers(view)
            except Exception:
                corners_list, ids = None, None
            if ids is not None:
                id_to_corners = {int(ids[i][0]): corners_list[i][0] for i in range(len(ids))}
                required_ref_ids = (10, 11, 12, 13)
                for marker_id in required_ref_ids:
                    corners = id_to_corners.get(marker_id)
                    if corners is not None:
                        self.reference_marker_corners_cache[marker_id] = corners.copy()

                box_id_to_corners = dict(id_to_corners)
                for marker_id in required_ref_ids:
                    if marker_id not in box_id_to_corners:
                        cached = self.reference_marker_corners_cache.get(marker_id)
                        if cached is not None:
                            box_id_to_corners[marker_id] = cached
                ref_px_lengths: list[float] = []
                for marker_id in required_ref_ids:
                    c = box_id_to_corners.get(marker_id)
                    if c is None:
                        continue
                    ref_px_lengths.extend(
                        [
                            float(np.linalg.norm(c[1] - c[0])),
                            float(np.linalg.norm(c[2] - c[1])),
                            float(np.linalg.norm(c[3] - c[2])),
                            float(np.linalg.norm(c[0] - c[3])),
                        ]
                    )
                if ref_px_lengths:
                    self.auto_axis_px_per_cm = (sum(ref_px_lengths) / len(ref_px_lengths)) / MARKER_SIZE_CM
                else:
                    self.auto_axis_px_per_cm = None

                self._draw_workspace_box_from_reference_markers(view, box_id_to_corners)
                truck_corners = id_to_corners.get(0)
                trailer_corners = id_to_corners.get(1)
                if truck_corners is not None and trailer_corners is not None:
                    try:
                        if self.camera_H is not None and marker_pose_world is not None:
                            truck_pos_world, truck_heading_world = marker_pose_world(truck_corners, self.camera_H)
                            trailer_pos_world, trailer_heading_world = marker_pose_world(trailer_corners, self.camera_H)
                            hitch_vision_deg = float(
                                np.degrees(self._wrap_angle(float(truck_heading_world - trailer_heading_world)))
                            )

                            axis_tr = box_id_to_corners.get(11)
                            axis_br = box_id_to_corners.get(12)
                            if axis_tr is not None and axis_br is not None:
                                tr_center_px = axis_tr.mean(axis=0).astype(np.float32)
                                br_center_px = axis_br.mean(axis=0).astype(np.float32)
                                axis_pts_world = cv2.perspectiveTransform(
                                    np.array([[br_center_px, tr_center_px]], dtype=np.float32),
                                    self.camera_H,
                                )[0]
                                origin = axis_pts_world[0].astype(float)
                                y_vec = axis_pts_world[1].astype(float) - origin
                                y_norm = float(np.linalg.norm(y_vec))
                                if y_norm > 1e-6:
                                    y_hat = y_vec / y_norm
                                    x_hat = np.array([y_hat[1], -y_hat[0]], dtype=float)
                                    self.auto_axis_origin_px = br_center_px.astype(float)
                                    self.auto_axis_y_hat_px = y_hat.astype(float)
                                    self.auto_axis_x_hat_px = x_hat.astype(float)
                                    self.auto_axis_px_per_cm = None

                                    ref_ids = (10, 11, 12, 13)
                                    if all(mid in box_id_to_corners for mid in ref_ids):
                                        r_tl = box_id_to_corners[10][2].astype(np.float32)
                                        r_tr = box_id_to_corners[11][3].astype(np.float32)
                                        r_br = box_id_to_corners[12][0].astype(np.float32)
                                        r_bl = box_id_to_corners[13][1].astype(np.float32)
                                        ref_poly_px = np.array([[r_tl, r_tr, r_br, r_bl]], dtype=np.float32)
                                        ref_poly_world = cv2.perspectiveTransform(ref_poly_px, self.camera_H)[0]

                                        def _unit(v: np.ndarray) -> np.ndarray:
                                            n = float(np.linalg.norm(v))
                                            if n < 1e-6:
                                                return np.zeros(2, dtype=np.float32)
                                            return (v / n).astype(np.float32)

                                        inset_side = WORKSPACE_BOX_SIDE_INSET_CM
                                        extend_vertical = WORKSPACE_BOX_VERTICAL_EXTEND_CM
                                        w_tl, w_tr, w_br, w_bl = ref_poly_world
                                        in_tl = w_tl + inset_side * _unit(w_tr - w_tl) - extend_vertical * _unit(w_bl - w_tl)
                                        in_tr = w_tr + inset_side * _unit(w_tl - w_tr) - extend_vertical * _unit(w_br - w_tr)
                                        in_br = w_br + inset_side * _unit(w_bl - w_br) - extend_vertical * _unit(w_tr - w_br)
                                        in_bl = w_bl + inset_side * _unit(w_br - w_bl) - extend_vertical * _unit(w_tl - w_bl)
                                        box_center_world = (
                                            np.array([in_tl, in_tr, in_br, in_bl], dtype=np.float32).mean(axis=0)
                                        ).astype(float)
                                        rel_goal = box_center_world - origin
                                        self.auto_goal_xy = np.array(
                                            [float(np.dot(rel_goal, x_hat)), float(np.dot(rel_goal, y_hat))],
                                            dtype=float,
                                        )

                                    truck_center_world = np.array(truck_pos_world, dtype=float)
                                    trailer_center_world = np.array(trailer_pos_world, dtype=float)

                                    truck_center_px = truck_corners.mean(axis=0).astype(np.float32)
                                    truck_top_mid_px = ((truck_corners[0] + truck_corners[1]) / 2.0).astype(
                                        np.float32
                                    )
                                    truck_pts_world = cv2.perspectiveTransform(
                                        np.array([[truck_center_px, truck_top_mid_px]], dtype=np.float32),
                                        self.camera_H,
                                    )[0]
                                    truck_fwd_world = truck_pts_world[1].astype(float) - truck_pts_world[0].astype(float)
                                    truck_fwd_norm = float(np.linalg.norm(truck_fwd_world))
                                    if truck_fwd_norm > 1e-6:
                                        truck_fwd_world /= truck_fwd_norm
                                        back_offset_cm = MARKER_SIZE_CM * 0.5 + 4.0
                                        pivot_world = truck_center_world - truck_fwd_world * back_offset_cm
                                        rel_pivot = pivot_world - origin
                                        pivot_xy = np.array(
                                            [float(np.dot(rel_pivot, x_hat)), float(np.dot(rel_pivot, y_hat))],
                                            dtype=float,
                                        )

                                        truck_fwd_local = np.array(
                                            [
                                                float(np.dot(truck_fwd_world, x_hat)),
                                                float(np.dot(truck_fwd_world, y_hat)),
                                            ],
                                            dtype=float,
                                        )
                                        theta_t = float(
                                            self._wrap_angle(math.atan2(truck_fwd_local[1], truck_fwd_local[0]))
                                        )

                                        trailer_top_mid_px = (
                                            (trailer_corners[0] + trailer_corners[1]) / 2.0
                                        ).astype(np.float32)
                                        trailer_pts_world = cv2.perspectiveTransform(
                                            np.array([[trailer_center_world.astype(np.float32), trailer_top_mid_px]], dtype=np.float32),
                                            self.camera_H,
                                        )[0]
                                        trailer_fwd_world = (
                                            trailer_pts_world[1].astype(float) - trailer_pts_world[0].astype(float)
                                        )
                                        trailer_norm = float(np.linalg.norm(trailer_fwd_world))
                                        if trailer_norm > 1e-6:
                                            trailer_fwd_world /= trailer_norm
                                            trailer_fwd_local = np.array(
                                                [
                                                    float(np.dot(trailer_fwd_world, x_hat)),
                                                    float(np.dot(trailer_fwd_world, y_hat)),
                                                ],
                                                dtype=float,
                                            )
                                            theta_l = float(
                                                self._wrap_angle(
                                                    math.atan2(trailer_fwd_local[1], trailer_fwd_local[0])
                                                )
                                            )

                                            now = time.monotonic()
                                            v_cm_s = 0.0
                                            omega_rad_s = 0.0
                                            if (
                                                self.auto_prev_vision_t is not None
                                                and self.auto_prev_pivot_xy is not None
                                                and self.auto_prev_truck_heading is not None
                                            ):
                                                dt = now - self.auto_prev_vision_t
                                                if 1e-3 < dt < 0.5:
                                                    dxy = pivot_xy - self.auto_prev_pivot_xy
                                                    prev_theta = float(self.auto_prev_truck_heading)
                                                    v_raw = float(
                                                        (dxy[0] * math.cos(prev_theta) + dxy[1] * math.sin(prev_theta))
                                                        / dt
                                                    )
                                                    omega_raw = float(
                                                        self._wrap_angle(theta_t - prev_theta) / dt
                                                    )
                                                    alpha = 0.35
                                                    self.auto_vel_ema_cm_s = (
                                                        alpha * v_raw + (1.0 - alpha) * self.auto_vel_ema_cm_s
                                                    )
                                                    self.auto_omega_ema_rad_s = (
                                                        alpha * omega_raw
                                                        + (1.0 - alpha) * self.auto_omega_ema_rad_s
                                                    )
                                                    v_cm_s = self.auto_vel_ema_cm_s
                                                    omega_rad_s = self.auto_omega_ema_rad_s
                                            self.auto_prev_vision_t = now
                                            self.auto_prev_pivot_xy = pivot_xy.copy()
                                            self.auto_prev_truck_heading = theta_t
                                            self.auto_vision_q = np.array(
                                                [
                                                    float(pivot_xy[0]),
                                                    float(pivot_xy[1]),
                                                    float(theta_t),
                                                    float(theta_l),
                                                    float(v_cm_s),
                                                    float(omega_rad_s),
                                                ],
                                                dtype=float,
                                            )
                                        else:
                                            self.auto_vision_q = None
                                    else:
                                        self.auto_vision_q = None
                                else:
                                    self.auto_vision_q = None
                            else:
                                self.auto_vision_q = None
                        else:
                            axis_tr = box_id_to_corners.get(11)
                            axis_br = box_id_to_corners.get(12)
                            if axis_tr is None or axis_br is None:
                                self.auto_vision_q = None
                            else:
                                px_lengths: list[float] = []
                                for marker_id in required_ref_ids:
                                    c = box_id_to_corners.get(marker_id)
                                    if c is None:
                                        continue
                                    px_lengths.extend(
                                        [
                                            float(np.linalg.norm(c[1] - c[0])),
                                            float(np.linalg.norm(c[2] - c[1])),
                                            float(np.linalg.norm(c[3] - c[2])),
                                            float(np.linalg.norm(c[0] - c[3])),
                                        ]
                                    )
                                if not px_lengths:
                                    self.auto_vision_q = None
                                else:
                                    px_per_cm = (sum(px_lengths) / len(px_lengths)) / MARKER_SIZE_CM
                                    if px_per_cm <= 1e-6:
                                        self.auto_vision_q = None
                                    else:
                                        tr_center_px = axis_tr.mean(axis=0).astype(float)
                                        br_center_px = axis_br.mean(axis=0).astype(float)
                                        y_vec_px = tr_center_px - br_center_px
                                        y_norm = float(np.linalg.norm(y_vec_px))
                                        if y_norm <= 1e-6:
                                            self.auto_vision_q = None
                                        else:
                                            y_hat_px = y_vec_px / y_norm
                                            x_hat_px = np.array([y_hat_px[1], -y_hat_px[0]], dtype=float)
                                            origin_px = br_center_px
                                            self.auto_axis_origin_px = origin_px.astype(float)
                                            self.auto_axis_x_hat_px = x_hat_px.astype(float)
                                            self.auto_axis_y_hat_px = y_hat_px.astype(float)
                                            self.auto_axis_px_per_cm = float(px_per_cm)

                                            def _unit(v: np.ndarray) -> np.ndarray:
                                                n = float(np.linalg.norm(v))
                                                if n < 1e-6:
                                                    return np.zeros(2, dtype=float)
                                                return v / n

                                            def _to_local_cm(pt_px: np.ndarray) -> np.ndarray:
                                                rel = pt_px.astype(float) - origin_px
                                                return np.array(
                                                    [
                                                        float(np.dot(rel, x_hat_px) / px_per_cm),
                                                        float(np.dot(rel, y_hat_px) / px_per_cm),
                                                    ],
                                                    dtype=float,
                                                )

                                            if all(mid in box_id_to_corners for mid in required_ref_ids):
                                                p_tl = box_id_to_corners[10][2].astype(float)
                                                p_tr = box_id_to_corners[11][3].astype(float)
                                                p_br = box_id_to_corners[12][0].astype(float)
                                                p_bl = box_id_to_corners[13][1].astype(float)
                                                inset_side_px = WORKSPACE_BOX_SIDE_INSET_CM * px_per_cm
                                                extend_vertical_px = WORKSPACE_BOX_VERTICAL_EXTEND_CM * px_per_cm
                                                in_tl = p_tl + inset_side_px * _unit(p_tr - p_tl) - extend_vertical_px * _unit(p_bl - p_tl)
                                                in_tr = p_tr + inset_side_px * _unit(p_tl - p_tr) - extend_vertical_px * _unit(p_br - p_tr)
                                                in_br = p_br + inset_side_px * _unit(p_bl - p_br) - extend_vertical_px * _unit(p_tr - p_br)
                                                in_bl = p_bl + inset_side_px * _unit(p_br - p_bl) - extend_vertical_px * _unit(p_tl - p_bl)
                                                box_center_px = np.array([in_tl, in_tr, in_br, in_bl], dtype=float).mean(axis=0)
                                                self.auto_goal_xy = _to_local_cm(box_center_px)

                                            truck_center_px = truck_corners.mean(axis=0).astype(float)
                                            truck_top_mid_px = ((truck_corners[0] + truck_corners[1]) / 2.0).astype(float)
                                            trailer_center_px = trailer_corners.mean(axis=0).astype(float)
                                            trailer_top_mid_px = ((trailer_corners[0] + trailer_corners[1]) / 2.0).astype(float)

                                            truck_fwd_px = truck_top_mid_px - truck_center_px
                                            trailer_fwd_px = trailer_top_mid_px - trailer_center_px
                                            tn = float(np.linalg.norm(truck_fwd_px))
                                            ln = float(np.linalg.norm(trailer_fwd_px))
                                            if tn <= 1e-6 or ln <= 1e-6:
                                                self.auto_vision_q = None
                                            else:
                                                truck_fwd_px /= tn
                                                trailer_fwd_px /= ln
                                                theta_t = float(
                                                    self._wrap_angle(
                                                        math.atan2(
                                                            float(np.dot(truck_fwd_px, y_hat_px)),
                                                            float(np.dot(truck_fwd_px, x_hat_px)),
                                                        )
                                                    )
                                                )
                                                theta_l = float(
                                                    self._wrap_angle(
                                                        math.atan2(
                                                            float(np.dot(trailer_fwd_px, y_hat_px)),
                                                            float(np.dot(trailer_fwd_px, x_hat_px)),
                                                        )
                                                    )
                                                )
                                                hitch_vision_deg = float(
                                                    np.degrees(self._wrap_angle(theta_t - theta_l))
                                                )

                                                back_offset_px = (MARKER_SIZE_CM * 0.5 + 4.0) * px_per_cm
                                                pivot_px = truck_center_px - truck_fwd_px * back_offset_px
                                                pivot_xy = _to_local_cm(pivot_px)

                                                now = time.monotonic()
                                                v_cm_s = 0.0
                                                omega_rad_s = 0.0
                                                if (
                                                    self.auto_prev_vision_t is not None
                                                    and self.auto_prev_pivot_xy is not None
                                                    and self.auto_prev_truck_heading is not None
                                                ):
                                                    dt = now - self.auto_prev_vision_t
                                                    if 1e-3 < dt < 0.5:
                                                        dxy = pivot_xy - self.auto_prev_pivot_xy
                                                        prev_theta = float(self.auto_prev_truck_heading)
                                                        v_raw = float(
                                                            (dxy[0] * math.cos(prev_theta) + dxy[1] * math.sin(prev_theta))
                                                            / dt
                                                        )
                                                        omega_raw = float(self._wrap_angle(theta_t - prev_theta) / dt)
                                                        alpha = 0.35
                                                        self.auto_vel_ema_cm_s = (
                                                            alpha * v_raw + (1.0 - alpha) * self.auto_vel_ema_cm_s
                                                        )
                                                        self.auto_omega_ema_rad_s = (
                                                            alpha * omega_raw
                                                            + (1.0 - alpha) * self.auto_omega_ema_rad_s
                                                        )
                                                        v_cm_s = self.auto_vel_ema_cm_s
                                                        omega_rad_s = self.auto_omega_ema_rad_s

                                                self.auto_prev_vision_t = now
                                                self.auto_prev_pivot_xy = pivot_xy.copy()
                                                self.auto_prev_truck_heading = theta_t
                                                self.auto_vision_q = np.array(
                                                    [
                                                        float(pivot_xy[0]),
                                                        float(pivot_xy[1]),
                                                        float(theta_t),
                                                        float(theta_l),
                                                        float(v_cm_s),
                                                        float(omega_rad_s),
                                                    ],
                                                    dtype=float,
                                                )
                    except Exception:
                        hitch_vision_deg = None
                        self.auto_vision_q = None
                else:
                    self.auto_vision_q = None
                all_names = {}
                all_names.update(REFERENCE_MARKER_IDS)
                all_names.update(TRACKING_MARKER_IDS)
                for marker_id, corners in id_to_corners.items():
                    pts = corners.astype(int)
                    cv2.polylines(view, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                    center = corners.mean(axis=0).astype(int)
                    label = all_names.get(marker_id, f"id {marker_id}")
                    text = f"{label} ({marker_id})"
                    cv2.putText(
                        view,
                        text,
                        (int(pts[0][0]), int(pts[0][1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.circle(view, tuple(center), 4, (0, 255, 255), -1)
                    if label == "truck":
                        self._draw_vehicle_box(
                            view, corners, 0.0,
                            front_extra_cm=7.0, rear_extra_cm=5.0, side_extra_cm=2.0,
                            color=(255, 120, 0),
                        )
                        self._draw_truck_pivot_x(view, corners)
                    elif label == "trailer":
                        self._draw_vehicle_box(
                            view, corners, 0.0,
                            front_extra_cm=9.0, rear_extra_cm=7.0, side_extra_cm=2.0,
                            color=(180, 0, 255),
                        )
                    if self.camera_H is not None and marker_pose_world is not None:
                        try:
                            world_pos, heading = marker_pose_world(corners, self.camera_H)
                            deg = float(np.degrees(heading))
                            pose_text = f"({world_pos[0]:.1f},{world_pos[1]:.1f}) {deg:.1f}deg"
                            cv2.putText(
                                view,
                                pose_text,
                                (int(pts[0][0]), int(pts[0][1]) + 14),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (255, 255, 0),
                                2,
                                cv2.LINE_AA,
                            )
                        except Exception:
                            pass
                    found_count += 1
            else:
                self.auto_vision_q = None
                self.auto_axis_origin_px = None
                self.auto_axis_x_hat_px = None
                self.auto_axis_y_hat_px = None
                self.auto_axis_px_per_cm = None
        self._draw_auto_prediction_path(view)
        self._update_hitch_vision_display(hitch_vision_deg)
        self.camera_status_label.setText(f"Camera: Connected | Markers: {found_count}")
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

    def _update_button_states(self) -> None:
        connected = self.sender is not None
        for btn in self.drive_buttons:
            btn.setEnabled(connected)
        self.auto_start_btn.setEnabled(connected and not self.auto_running)
        self.auto_stop_btn.setEnabled(connected and self.auto_running)

    def _set_connected(self, connected: bool) -> None:
        if not connected:
            self.auto_running = False
            self.auto_status_label.setText("MPC: Idle")
            self._update_hitch_angle_display(0.0)
            self._reset_manual_stats()
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self._update_button_states()
        self.header.set_connected(connected)

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
        self._reset_manual_stats()
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

        pot_angle_deg = sender.read_pot_angle_deg()
        if pot_angle_deg is not None:
            self.latest_hitch_raw_deg = pot_angle_deg
            self._update_hitch_angle_display(pot_angle_deg)
        motor_rpms = sender.read_motor_rpms()
        if motor_rpms is not None:
            self._update_manual_stats(motor_rpms[0], motor_rpms[1])

        if self.mode_stack.currentIndex() == 1:
            if self.auto_running:
                hitch_abs = abs(self.latest_hitch_vision_deg) if self.latest_hitch_vision_deg is not None else 0.0
                if self.auto_hitch_recovery_active and hitch_abs <= AUTO_HITCH_RELEASE_DEG:
                    self.auto_hitch_recovery_active = False
                    self.auto_status_label.setText("MPC: Running")

                if (not self.auto_hitch_recovery_active) and hitch_abs > AUTO_HITCH_HARD_LIMIT_DEG:
                    self.auto_hitch_recovery_active = True

                if self.auto_hitch_recovery_active:
                    rpm_l, rpm_r = self._car_like_recovery_rpms()
                    self.auto_status_label.setText(
                        f"MPC: Jackknife recovery ({self.latest_hitch_display_deg:.1f} deg)"
                    )
                else:
                    rpm_l, rpm_r = self._auto_tick()
                rpm_l, rpm_r = self._boost_auto_rpms(rpm_l, rpm_r)
            else:
                rpm_l, rpm_r = 0.0, 0.0
        else:
            base_rpm = float(self.speed_slider.value())
            rpm_l, rpm_r = motion_targets(self.current_motion, base_rpm)
        try:
            sender.send_targets(rpm_l, rpm_r)
        except Exception:
            self._disconnect()

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
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
        elif key == Qt.Key.Key_Space:
            self._set_motion("STOP")

    def keyReleaseEvent(self, event) -> None:  # type: ignore[override]
        if not self._is_manual_mode():
            super().keyReleaseEvent(event)
            return
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right):
            self._set_motion("STOP")

    def focusOutEvent(self, event) -> None:  # type: ignore[override]
        if self._is_manual_mode():
            self._set_motion("STOP")
        super().focusOutEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._disconnect_camera()
        self._disconnect()
        super().closeEvent(event)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simple PyQt UART drive GUI")
    parser.add_argument("--port", default="", help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--rpm", type=float, default=30.0, help="Initial base RPM")
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
