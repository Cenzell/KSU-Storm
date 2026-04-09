import os
import sys
import time
import logging
import math
from pathlib import Path
import pygame
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QIcon
from PyQt6 import uic

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parents[1]
LIB_DIR = PROJECT_ROOT / "lib"
UI_DIR = BASE_DIR / "ui"
UI_FILE = UI_DIR / "driver_station.ui"

for path in (LIB_DIR, UI_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

import comm
import fms as fms_module
from auto_routines import AUTO_ROUTINES, routine_description, routine_keys, routine_label, routine_payload
from driver_ui import DriverUIHelpers

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
GAMEPAD_POLL_RATE_MS = 20
JOYSTICK_THRESHOLD = 0.01  # Minimum change to send update
MAX_LINEAR_SPEED_MPS = 1.2
MAX_ANGULAR_SPEED_DPS = 180.0
EXPECTED_POSE_HORIZON_S = 0.35
SLOW_DRIVE_SCALE = 0.2
AXIS_DEADZONE = 0.03

FACE_BUTTON_COLORS = {
    0: "green",   # A
    1: "red",     # B
    2: "blue",    # X
    3: "purple",  # Y
}


class AppWindow(DriverUIHelpers, QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(str(UI_FILE), self)
        self.setup_tabs()

        self.joystick = None
        self.init_pygame_and_joystick()

        # Connection manager (ZMQ-based)
        self.conn_manager = comm.ConnectionManager()
        self.conn_manager.signals.connection_status.connect(self.update_connection_status)
        self.conn_manager.start()

        # Background command worker – all ZMQ sends happen here, never on the UI thread
        self.cmd_worker = comm.CommandWorker(self.conn_manager)
        self.cmd_worker.start()

        # FMS poller
        self.fms_required_voltage: float = 0.0  # current required voltage (0 = no match)
        self.fms_required_rpm: float = 0.0      # current required RPM     (0 = no match)
        self.fms_poller = fms_module.FMSPoller()
        self.fms_poller.signals.connection_changed.connect(self._handle_fms_connection)
        self.fms_poller.signals.match_update.connect(self._handle_fms_match_update)
        self.fms_poller.signals.voltage_required.connect(self._on_fms_voltage_required)
        self.fms_poller.signals.rpm_required.connect(self._on_fms_rpm_required)
        self.fms_poller.start()

        # Jumpstart cooldown timer (§3.3.5 — 30 s between jumpstarts)
        self._jumpstart_cooldown_remaining: int = 0
        self._jumpstart_cooldown_timer = QTimer(self)
        self._jumpstart_cooldown_timer.setInterval(1000)  # tick every second
        self._jumpstart_cooldown_timer.timeout.connect(self._tick_jumpstart_cooldown)

        # Telemetry receiver
        self.telemetry_receiver = comm.TelemetryReceiver(self.conn_manager)
        self.telemetry_receiver.start()
        
        # Connect signals
        self.conn_manager.signals.ping_response.connect(self.handle_ping_response)
        self.conn_manager.signals.telemetry_update.connect(self.handle_telemetry)
        
        # Gamepad polling timer
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.poll_gamepad)
        self.gamepad_timer.start(GAMEPAD_POLL_RATE_MS)

        # Match timer
        self.match_timer = QTimer()
        self.match_timer.timeout.connect(self.update_match_time)
        self.match_time_seconds = 0
        self.match_running = False
        self.auto_duration = 30  # 30 seconds for auto
        self.teleop_duration = 210  # 3 minutes 30 seconds (210 seconds) for teleop
        self.match_start_time = 0

        # State tracking
        self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}
        self.last_sent_joystick_values = self.joystick_values.copy()
        self.current_mode = "STOPPED"
        self.current_alliance = "RED"
        self.current_pose = {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
        self.expected_pose = self.current_pose.copy()
        self.selected_auto_key = routine_keys()[0] if routine_keys() else ""
        self.current_auto_status = {"active": False, "status": "idle"}
        self._last_diagnostic_telemetry_time = 0.0
        self._last_diagnostic_controls = None

        # Add field view to odometry panel
        self.setup_field_view()
        self.setup_main_camera_view()
        self.setup_camera_stream()
        start_x, start_y, start_theta = self._alliance_start_pose()
        self.current_pose = {
            "x": start_x,
            "y": start_y,
            "theta_deg": start_theta,
        }
        self.expected_pose = self.current_pose.copy()
        
        # Keyboard control state
        self.keyboard_enabled = True
        self.keys_pressed = set()
        self.keyboard_speed = 0.7  # Default keyboard speed (0.0 to 1.0)
        
        # Connect mode buttons
        self.btn_auto.clicked.connect(self.set_auto_mode)
        self.btn_teleop.clicked.connect(self.set_teleop_mode)
        self.btn_rst.clicked.connect(self.reset_robot)
        if hasattr(self, 'btn_auto_3'):
            self.btn_auto_3.clicked.connect(self.toggle_alliance)
        if hasattr(self, 'jumpstart_btn'):
            self.jumpstart_btn.clicked.connect(self.trigger_jumpstart)
        if hasattr(self, 'pushButton'):
            self.pushButton.clicked.connect(self.reset_odometry)
        if hasattr(self, 'odo_tab_reset_button'):
            self.odo_tab_reset_button.clicked.connect(self.reset_odometry)
        if hasattr(self, 'btn_odo_optical'):
            self.btn_odo_optical.clicked.connect(lambda: self.set_odometry_mode("OPTICAL"))
        if hasattr(self, 'btn_odo_motor'):
            self.btn_odo_motor.clicked.connect(lambda: self.set_odometry_mode("MOTOR"))
        if hasattr(self, 'btn_odo_hybrid'):
            self.btn_odo_hybrid.clicked.connect(lambda: self.set_odometry_mode("HYBRID"))
        if hasattr(self, 'odo_tab_optical_button'):
            self.odo_tab_optical_button.clicked.connect(lambda: self.set_odometry_mode("OPTICAL"))
        if hasattr(self, 'odo_tab_motor_button'):
            self.odo_tab_motor_button.clicked.connect(lambda: self.set_odometry_mode("MOTOR"))
        if hasattr(self, 'odo_tab_hybrid_button'):
            self.odo_tab_hybrid_button.clicked.connect(lambda: self.set_odometry_mode("HYBRID"))
        if hasattr(self, "auto_routine_selector"):
            for key in routine_keys():
                self.auto_routine_selector.addItem(routine_label(key), key)
            self.auto_routine_selector.currentIndexChanged.connect(self.on_auto_routine_changed)
        if hasattr(self, "auto_run_selected_button"):
            self.auto_run_selected_button.clicked.connect(self.run_selected_auto)
        if hasattr(self, "auto_cancel_button"):
            self.auto_cancel_button.clicked.connect(self.cancel_auto)

        # Elevator PID controls
        if hasattr(self, "elev_set_gains_btn"):
            self.elev_set_gains_btn.clicked.connect(self.send_elevator_gains)
        if hasattr(self, "elev_go_btn"):
            self.elev_go_btn.clicked.connect(self.send_elevator_setpoint)
        if hasattr(self, "elev_disable_btn"):
            self.elev_disable_btn.clicked.connect(self.disable_elevator_pid)
        if hasattr(self, "elev_preset_buttons"):
            for label, btn in self.elev_preset_buttons.items():
                ticks = btn.property("elev_ticks")
                btn.clicked.connect(lambda checked, t=ticks: self._send_elevator_preset(t))
        if hasattr(self, "elev_manual_send_btn"):
            self.elev_manual_send_btn.clicked.connect(self.send_elevator_manual)
        if hasattr(self, "elev_manual_stop_btn"):
            self.elev_manual_stop_btn.clicked.connect(self.stop_elevator_manual)

        # Arm PID controls
        if hasattr(self, "arm_set_gains_btn"):
            self.arm_set_gains_btn.clicked.connect(self.send_arm_gains)
        if hasattr(self, "arm_go_btn"):
            self.arm_go_btn.clicked.connect(self.send_arm_setpoint)
        if hasattr(self, "arm_disable_btn"):
            self.arm_disable_btn.clicked.connect(self.disable_arm_pid)
        if hasattr(self, "arm_preset_buttons"):
            for label, btn in self.arm_preset_buttons.items():
                deg = btn.property("arm_degrees")
                btn.clicked.connect(lambda checked, d=deg: self._send_arm_preset(d))
        if hasattr(self, "arm_manual_send_btn"):
            self.arm_manual_send_btn.clicked.connect(self.send_arm_manual)
        if hasattr(self, "arm_manual_stop_btn"):
            self.arm_manual_stop_btn.clicked.connect(self.stop_arm_manual)
        
        # Setup keyboard speed slider if it exists in UI
        if hasattr(self, 'keyboard_speed_slider'):
            self.keyboard_speed_slider.setMinimum(0)
            self.keyboard_speed_slider.setMaximum(100)
            self.keyboard_speed_slider.setValue(100)
            self.keyboard_speed_slider.valueChanged.connect(self.update_keyboard_speed)
            self.keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")
        
        logger.info("Driver station initialized")
        logger.info("Keyboard controls: WASD=move, QE=rotate, Shift=speed boost, Space=stop")
        self.append_diagnostic("connection", "Connection manager started")
        self.append_diagnostic("telemetry", "Telemetry receiver started")
        self._update_alliance_button()
        self._update_odometry_context_labels()
        self._sync_settings_tab_controls()
        self._update_network_status_labels(False, "N/A")
        self._refresh_auto_selection_ui()
        self._update_auto_status_labels({"active": False, "status": "idle"})

    def _alliance_start_pose(self):
        half_robot = 18.0 * 0.0254 / 2.0
        start_y = half_robot
        start_theta = 90.0
        if self.current_alliance == "BLUE":
            return self.field_widget.field_width_m - half_robot, start_y, start_theta
        return half_robot, start_y, start_theta

    def _update_alliance_button(self):
        if not hasattr(self, "btn_auto_3"):
            color = "red" if self.current_alliance == "RED" else "blue"
        else:
            color = "red" if self.current_alliance == "RED" else "blue"
            self.btn_auto_3.setText(f"Alliance: {self.current_alliance.title()}")
            self.btn_auto_3.setStyleSheet(f"background: {color}; color: white;")

        if hasattr(self, "odo_tab_alliance_label"):
            self.odo_tab_alliance_label.setText(f"Alliance: {self.current_alliance.title()}")
        if hasattr(self, "settings_alliance_summary"):
            self.settings_alliance_summary.setText(f"Alliance: {self.current_alliance.title()}")

    def _update_odometry_context_labels(self):
        if hasattr(self, "odo_tab_mode_label"):
            self.odo_tab_mode_label.setText(f"Mode: {self.current_mode.title()}")
        if hasattr(self, "odo_tab_odo_mode_label") and hasattr(self, "label_odo_mode"):
            current_text = self.label_odo_mode.text().replace("Odometry Mode: ", "")
            self.odo_tab_odo_mode_label.setText(f"Odometry Mode: {current_text}")
        if hasattr(self, "settings_mode_summary"):
            self.settings_mode_summary.setText(f"Mode: {self.current_mode.title()}")
        if hasattr(self, "settings_odometry_summary") and hasattr(self, "label_odo_mode"):
            current_text = self.label_odo_mode.text().replace("Odometry Mode: ", "")
            self.settings_odometry_summary.setText(f"Odometry Mode: {current_text}")
        if hasattr(self, "settings_auto_summary"):
            self.settings_auto_summary.setText(
                f"Selected Auto: {routine_label(self.selected_auto_key) if self.selected_auto_key else 'None'}"
            )
        if hasattr(self, "odo_tab_field_size_label"):
            self.odo_tab_field_size_label.setText(
                f"Field: {self.field_widget.field_width_m:.3f} m x {self.field_widget.field_height_m:.3f} m"
            )

    def _refresh_auto_selection_ui(self):
        selected_label = routine_label(self.selected_auto_key) if self.selected_auto_key else "None"
        selected_description = routine_description(self.selected_auto_key) if self.selected_auto_key else ""

        if hasattr(self, "auto_routine_selector"):
            index = self.auto_routine_selector.findData(self.selected_auto_key)
            if index >= 0 and self.auto_routine_selector.currentIndex() != index:
                self.auto_routine_selector.blockSignals(True)
                self.auto_routine_selector.setCurrentIndex(index)
                self.auto_routine_selector.blockSignals(False)

        if hasattr(self, "auto_routine_description_label"):
            self.auto_routine_description_label.setText(
                f"Selected Auto: {selected_label}"
                + (f" | {selected_description}" if selected_description else "")
            )

        if hasattr(self, "settings_auto_summary"):
            self.settings_auto_summary.setText(f"Selected Auto: {selected_label}")

    def _update_auto_status_labels(self, auto_status):
        auto_status = auto_status or {"active": False, "status": "idle"}
        self.current_auto_status = dict(auto_status)
        status_text = str(auto_status.get("status", "idle")).replace("_", " ").title()
        routine_name = auto_status.get("routine_name")
        if not routine_name and self.selected_auto_key:
            routine_name = routine_label(self.selected_auto_key)
        step_name = auto_status.get("step_name")
        step_index = int(auto_status.get("step_index", 0))
        step_count = int(auto_status.get("step_count", 0))
        last_error = auto_status.get("last_error")

        if hasattr(self, "auto_status_label"):
            self.auto_status_label.setText(f"Auto Status: {status_text}")

        detail_parts = []
        if routine_name:
            detail_parts.append(f"Routine {routine_name}")
        if step_count > 0:
            detail_parts.append(f"Step {step_index + 1}/{step_count}")
        if step_name:
            detail_parts.append(str(step_name))
        if last_error:
            detail_parts.append(f"Note: {last_error}")
        detail_text = " | ".join(detail_parts) if detail_parts else "No routine running"

        if hasattr(self, "auto_status_detail_label"):
            self.auto_status_detail_label.setText(f"Auto Detail: {detail_text}")
        if hasattr(self, "settings_auto_status_summary"):
            self.settings_auto_status_summary.setText(f"Auto Status: {status_text}")

    def on_auto_routine_changed(self, index):
        if not hasattr(self, "auto_routine_selector"):
            return
        selected_key = self.auto_routine_selector.itemData(index)
        if not selected_key:
            return
        self.selected_auto_key = str(selected_key)
        self._refresh_auto_selection_ui()
        self.append_diagnostic("controls", f"Selected auto routine: {routine_label(self.selected_auto_key)}")

    def run_selected_auto(self):
        if not self.selected_auto_key:
            self.append_diagnostic("controls", "No auto routine selected")
            return False

        if self.conn_manager.get_client() is None:
            self.append_diagnostic("controls", "Cannot run auto: robot not connected")
            return False

        try:
            payload = routine_payload(self.selected_auto_key)
        except KeyError as exc:
            logger.error("Unknown auto routine: %s", exc)
            self.append_diagnostic("controls", str(exc))
            return False

        # Update UI immediately; robot confirms via telemetry.
        self.current_mode = "AUTO"
        self.robot_status.setText("Autonomous")
        self._update_odometry_context_labels()
        self._update_auto_status_labels({"active": True, "status": "running"})
        self.start_match_timer()
        self.append_diagnostic("controls", f"Auto routine requested: {routine_label(self.selected_auto_key)}")
        self.cmd_worker.enqueue("auto_run", routine=payload)
        return True

    def cancel_auto(self):
        if self.conn_manager.get_client() is None:
            self.append_diagnostic("controls", "Cannot cancel auto: robot not connected")
            return False

        self._update_auto_status_labels({"active": False, "status": "cancelled"})
        self.append_diagnostic("controls", "Auto routine cancel requested")
        self.cmd_worker.enqueue("auto_cancel")
        return True

    def _sync_settings_tab_controls(self):
        checkbox_pairs = [
            ("slow_drive", "settings_slow_drive_checkbox"),
            ("disable_drive", "settings_disable_drive_checkbox"),
            ("only_drive", "settings_only_drive_checkbox"),
            ("disable_vision", "settings_disable_vision_checkbox"),
            ("check_odo", "settings_check_odo_checkbox"),
            ("end_after_teleop", "settings_end_after_teleop_checkbox"),
        ]

        for source_name, target_name in checkbox_pairs:
            source = getattr(self, source_name, None)
            target = getattr(self, target_name, None)
            if source is None or target is None:
                continue

            target.blockSignals(True)
            target.setChecked(source.isChecked())
            target.blockSignals(False)
            target.toggled.connect(source.setChecked)
            source.toggled.connect(target.setChecked)

        if hasattr(self, "settings_keyboard_speed_slider"):
            self.settings_keyboard_speed_slider.blockSignals(True)
            self.settings_keyboard_speed_slider.setValue(int(self.keyboard_speed * 100))
            self.settings_keyboard_speed_slider.blockSignals(False)
            self.settings_keyboard_speed_slider.valueChanged.connect(self.update_keyboard_speed)
        if hasattr(self, "settings_keyboard_speed_label"):
            self.settings_keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")

    def _update_network_status_labels(self, is_connected, address):
        if hasattr(self, "network_status_label"):
            self.network_status_label.setText(
                "Status: Connected" if is_connected else "Status: Disconnected"
            )
        if hasattr(self, "network_active_address_label"):
            self.network_active_address_label.setText(f"Active Address: {address}")

    def _update_network_ping_label(self, ping_ms):
        if hasattr(self, "network_ping_label"):
            self.network_ping_label.setText(f"Last Ping: {ping_ms:.1f} ms")

    def _update_network_telemetry_labels(self, data):
        if hasattr(self, "network_robot_mode_label"):
            self.network_robot_mode_label.setText(f"Robot Mode: {data.get('mode', self.current_mode)}")
        if hasattr(self, "network_last_telemetry_label"):
            self.network_last_telemetry_label.setText(
                f"Last Telemetry: {time.strftime('%H:%M:%S')}"
            )

    # ── FMS handlers ──────────────────────────────────────────────────────────

    def _handle_fms_connection(self, reachable: bool) -> None:
        """Called when FMS reachability changes."""
        if not hasattr(self, "fms_status_label"):
            return
        if reachable:
            self.fms_status_label.setText("FMS: Connected")
            self.fms_status_label.setStyleSheet("font-size: 12px; color: rgb(100, 220, 100);")
        else:
            self.fms_status_label.setText("FMS: Not connected")
            self.fms_status_label.setStyleSheet("font-size: 12px; color: rgb(180, 180, 180);")
            if hasattr(self, "fms_match_label"):
                self.fms_match_label.setText("Match: --")
            if hasattr(self, "fms_time_label"):
                self.fms_time_label.setText("FMS Time: --")
            if hasattr(self, "fms_voltage_label"):
                self.fms_voltage_label.setText("--  V")
            if hasattr(self, "fms_rpm_label"):
                self.fms_rpm_label.setText("--  RPM")
        self.append_diagnostic("connection", f"FMS reachable: {reachable}")

    def _handle_fms_match_update(self, data) -> None:
        """Called each poll cycle with fresh FMS data, or None if no active match."""
        if data is None:
            if hasattr(self, "fms_match_label"):
                self.fms_match_label.setText("Match: No active match")
            if hasattr(self, "fms_time_label"):
                self.fms_time_label.setText("FMS Time: --")
            if hasattr(self, "fms_voltage_label"):
                self.fms_voltage_label.setText("--  V")
            if hasattr(self, "fms_rpm_label"):
                self.fms_rpm_label.setText("--  RPM")
            return

        # data is an fms_module.FMSMatchData instance
        status = "Active" if data.match_active else "Upcoming"
        if hasattr(self, "fms_match_label"):
            self.fms_match_label.setText(f"Match: {status}")

        mins = int(data.time_remaining_s) // 60
        secs = int(data.time_remaining_s) % 60
        if hasattr(self, "fms_time_label"):
            self.fms_time_label.setText(f"FMS Time: {mins}:{secs:02d}")

        voltage = data.required_voltage
        if hasattr(self, "fms_voltage_label"):
            if voltage > 0:
                self.fms_voltage_label.setText(f"{voltage:.0f}  V")
            else:
                self.fms_voltage_label.setText("--  V")

        rpm = data.required_rpm
        if hasattr(self, "fms_rpm_label"):
            if rpm > 0:
                self.fms_rpm_label.setText(f"{rpm:.0f}  RPM")
            else:
                self.fms_rpm_label.setText("--  RPM")

    def _on_fms_voltage_required(self, voltage: float) -> None:
        """Called only when the required voltage value actually changes.

        This is the integration point for the voltage circuit.  When the
        circuit hardware and wiring specs are available, send the voltage
        command here (e.g. via SerialBridge).

        Args:
            voltage: Required voltage in volts (2 / 4 / 6 / 8 / 10), or 0
                     when no match is active.
        """
        self.fms_required_voltage = voltage
        logger.info("FMS voltage required: %sV", voltage)
        self.append_diagnostic("connection", f"FMS voltage required: {voltage}V")

        # ── future circuit integration ──────────────────────────────────────
        # Fill in once circuit specs/wiring are available:
        #
        #   if voltage > 0 and hasattr(self, "serial_bridge"):
        #       self.serial_bridge.send_voltage(voltage)
        # ───────────────────────────────────────────────────────────────────

    def _on_fms_rpm_required(self, rpm: float) -> None:
        """Called only when the required grid frequency (RPM) changes.

        Operators use this to know how fast to spin the Charging Wheel
        (§3.3.4 Generate Electricity: 1 KJ per 2s at correct RPM ±5).

        Args:
            rpm: Required RPM (20 / 30 / 40 / 50), or 0 when no match active.
        """
        self.fms_required_rpm = rpm
        logger.info("FMS RPM required: %s RPM", rpm)
        self.append_diagnostic("connection", f"FMS grid frequency: {rpm} RPM")

    # ── Jumpstart cooldown ────────────────────────────────────────────────────

    JUMPSTART_COOLDOWN_S = 30

    def trigger_jumpstart(self) -> None:
        """Start a 30-second cooldown after a Jumpstart Grid attempt (§3.3.5)."""
        if self._jumpstart_cooldown_remaining > 0:
            self.append_diagnostic("controls",
                f"Jumpstart blocked — cooldown {self._jumpstart_cooldown_remaining}s remaining")
            return

        self._jumpstart_cooldown_remaining = self.JUMPSTART_COOLDOWN_S
        self._update_jumpstart_ui()
        self._jumpstart_cooldown_timer.start()
        self.append_diagnostic("controls", "Jumpstart triggered — 30 s cooldown started")

    def _tick_jumpstart_cooldown(self) -> None:
        self._jumpstart_cooldown_remaining -= 1
        if self._jumpstart_cooldown_remaining <= 0:
            self._jumpstart_cooldown_remaining = 0
            self._jumpstart_cooldown_timer.stop()
            self._end_jumpstart_cooldown()
        else:
            self._update_jumpstart_ui()

    def _update_jumpstart_ui(self) -> None:
        remaining = self._jumpstart_cooldown_remaining
        if hasattr(self, "jumpstart_cooldown_label"):
            self.jumpstart_cooldown_label.setText(f"COOLDOWN  {remaining} s")
        if hasattr(self, "voltage_stack"):
            self.voltage_stack.setCurrentIndex(1)  # show cooldown overlay
        if hasattr(self, "jumpstart_btn"):
            self.jumpstart_btn.setEnabled(False)
            self.jumpstart_btn.setStyleSheet(
                "font-weight: bold; font-size: 13px;"
                "background: rgb(80, 40, 40); color: rgb(160, 100, 100); border-radius: 4px;"
            )

    def _end_jumpstart_cooldown(self) -> None:
        if hasattr(self, "voltage_stack"):
            self.voltage_stack.setCurrentIndex(0)  # restore voltage display
        if hasattr(self, "jumpstart_btn"):
            self.jumpstart_btn.setEnabled(True)
            self.jumpstart_btn.setStyleSheet(
                "font-weight: bold; font-size: 13px;"
                "background: rgb(60, 130, 60); color: white; border-radius: 4px;"
            )
        self.append_diagnostic("controls", "Jumpstart cooldown expired — ready")

    # ── end FMS handlers ──────────────────────────────────────────────────────

    def set_alliance(self, alliance):
        alliance = str(alliance).upper()
        if alliance not in {"RED", "BLUE"}:
            return False

        self.current_alliance = alliance
        self._update_alliance_button()
        self.append_diagnostic("controls", f"Alliance set to {alliance}")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("alliance", alliance=alliance)
        return True

    def toggle_alliance(self):
        new_alliance = "BLUE" if self.current_alliance == "RED" else "RED"
        if self.set_alliance(new_alliance):
            self.reset_odometry()

    def update_odometry_labels(self, x_m, y_m, theta_deg):
        if hasattr(self, 'label_3'):
            self.label_3.setText(f"X: {x_m:.2f} m")
        if hasattr(self, 'label_2'):
            self.label_2.setText(f"Y: {y_m:.2f} m")
        if hasattr(self, 'label_4'):
            self.label_4.setText(f"Theta: {theta_deg:.1f} deg")
        if hasattr(self, 'odo_tab_x_label'):
            self.odo_tab_x_label.setText(f"X: {x_m:.2f} m")
        if hasattr(self, 'odo_tab_y_label'):
            self.odo_tab_y_label.setText(f"Y: {y_m:.2f} m")
        if hasattr(self, 'odo_tab_theta_label'):
            self.odo_tab_theta_label.setText(f"Theta: {theta_deg:.1f} deg")

    def update_mechanism_encoder_labels(self, encoders):
        values = list(encoders) if isinstance(encoders, list) else []

        def encoder_value(index):
            if index >= len(values):
                return 0
            try:
                return int(values[index])
            except Exception:
                return 0

        elevator_left = encoder_value(4)
        elevator_right = encoder_value(5)
        arm_motor = encoder_value(6)

        if hasattr(self, "odo_tab_elevator_left_encoder_label"):
            self.odo_tab_elevator_left_encoder_label.setText(f"Elevator Left: {elevator_left}")
        if hasattr(self, "odo_tab_elevator_right_encoder_label"):
            self.odo_tab_elevator_right_encoder_label.setText(f"Elevator Right: {elevator_right}")
        if hasattr(self, "odo_tab_arm_motor_encoder_label"):
            self.odo_tab_arm_motor_encoder_label.setText(f"Arm Motor: {arm_motor}")

    def update_expected_pose(self):
        """Project a short-horizon expected pose from current command inputs."""
        base_x = float(self.current_pose["x"])
        base_y = float(self.current_pose["y"])
        base_theta_deg = float(self.current_pose["theta_deg"])

        if self.current_mode != "TELEOP":
            self.expected_pose = {"x": base_x, "y": base_y, "theta_deg": base_theta_deg}
            self.field_widget.set_expected_pose(base_x, base_y, base_theta_deg)
            if hasattr(self, "odometry_field_widget"):
                self.odometry_field_widget.set_expected_pose(base_x, base_y, base_theta_deg)
            if hasattr(self, "odo_tab_expected_label"):
                self.odo_tab_expected_label.setText(
                    f"Expected: X {base_x:.2f} m | Y {base_y:.2f} m | Theta {base_theta_deg:.1f} deg"
                )
            return

        lx = float(self.joystick_values.get("lx", 0.0))
        ly = float(self.joystick_values.get("ly", 0.0))
        rx = float(self.joystick_values.get("rx", 0.0))

        v_forward = ly * MAX_LINEAR_SPEED_MPS
        v_strafe = lx * MAX_LINEAR_SPEED_MPS
        omega_deg = rx * MAX_ANGULAR_SPEED_DPS

        theta_rad = math.radians(base_theta_deg)
        v_field_x = (v_forward * math.cos(theta_rad)) - (v_strafe * math.sin(theta_rad))
        v_field_y = (v_forward * math.sin(theta_rad)) + (v_strafe * math.cos(theta_rad))

        expected_x = base_x + (v_field_x * EXPECTED_POSE_HORIZON_S)
        expected_y = base_y + (v_field_y * EXPECTED_POSE_HORIZON_S)
        expected_theta_deg = (base_theta_deg + (omega_deg * EXPECTED_POSE_HORIZON_S)) % 360.0

        expected_x = max(0.0, min(self.field_widget.field_width_m, expected_x))
        expected_y = max(0.0, min(self.field_widget.field_height_m, expected_y))

        self.expected_pose = {"x": expected_x, "y": expected_y, "theta_deg": expected_theta_deg}
        self.field_widget.set_expected_pose(expected_x, expected_y, expected_theta_deg)
        if hasattr(self, "odometry_field_widget"):
            self.odometry_field_widget.set_expected_pose(expected_x, expected_y, expected_theta_deg)
        if hasattr(self, "odo_tab_expected_label"):
            self.odo_tab_expected_label.setText(
                f"Expected: X {expected_x:.2f} m | Y {expected_y:.2f} m | Theta {expected_theta_deg:.1f} deg"
            )

    def set_odometry_mode(self, mode):
        """Set the odometry source mode on the robot."""
        if hasattr(self, 'label_odo_mode'):
            self.label_odo_mode.setText(f"Odometry Mode: {mode.title()}")
        if hasattr(self, 'odo_tab_odo_mode_label'):
            self.odo_tab_odo_mode_label.setText(f"Odometry Mode: {mode.title()}")
        self.append_diagnostic("telemetry", f"Odometry mode requested: {mode}")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("odometry_mode", mode=mode)

    def send_elevator_gains(self):
        kp = self.elev_kp_spin.value()
        ki = self.elev_ki_spin.value()
        kd = self.elev_kd_spin.value()
        max_out = self.elev_max_out_spin.value()
        decel_zone = self.elev_decel_zone_spin.value()
        self.append_diagnostic("telemetry",
            f"Elevator PID gains: kP={kp} kI={ki} kD={kd} maxOut={max_out} decelZone={decel_zone}")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("elevator_pid", kp=kp, ki=ki, kd=kd,
                                    max_output=max_out, decel_zone=decel_zone)

    def send_elevator_setpoint(self):
        ticks = self.elev_setpoint_spin.value()
        self._send_elevator_preset(ticks)

    def _send_elevator_preset(self, ticks: int):
        self.elev_setpoint_spin.setValue(ticks)
        self.append_diagnostic("telemetry", f"Elevator setpoint: {ticks} ticks")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("elevator_setpoint", setpoint=float(ticks))
        if hasattr(self, "elev_setpoint_label"):
            self.elev_setpoint_label.setText(f"Setpoint: {ticks}")
        if hasattr(self, "elev_status_label"):
            self.elev_status_label.setText("PID: Active")

    def send_elevator_manual(self):
        left = self.elev_manual_left_slider.value() / 100.0
        right = self.elev_manual_right_slider.value() / 100.0
        self.append_diagnostic("telemetry", f"Elevator manual: L={left:.2f} R={right:.2f}")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("elevator_manual", left=left, right=right)
        if hasattr(self, "elev_status_label"):
            self.elev_status_label.setText("PID: Inactive (manual)")

    def stop_elevator_manual(self):
        if hasattr(self, "elev_manual_left_slider"):
            self.elev_manual_left_slider.setValue(0)
        if hasattr(self, "elev_manual_right_slider"):
            self.elev_manual_right_slider.setValue(0)
        self.append_diagnostic("telemetry", "Elevator manual stop")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("elevator_manual", left=0.0, right=0.0)

    def disable_elevator_pid(self):
        self.append_diagnostic("telemetry", "Elevator PID disabled")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("elevator_disable")
        if hasattr(self, "elev_status_label"):
            self.elev_status_label.setText("PID: Inactive")

    # ── Arm PID controls ──────────────────────────────────────────────────────

    def send_arm_gains(self):
        kp = self.arm_kp_spin.value()
        ki = self.arm_ki_spin.value()
        kd = self.arm_kd_spin.value()
        max_out = self.arm_max_out_spin.value()
        decel_zone = self.arm_decel_zone_spin.value()
        self.append_diagnostic("telemetry",
            f"Arm PID gains: kP={kp} kI={ki} kD={kd} maxOut={max_out} decelZone={decel_zone}°")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("arm_pid", kp=kp, ki=ki, kd=kd,
                                    max_output=max_out, decel_zone_deg=decel_zone)

    def send_arm_setpoint(self):
        deg = self.arm_setpoint_spin.value()
        self._send_arm_preset(deg)

    def _send_arm_preset(self, degrees: float):
        self.arm_setpoint_spin.setValue(degrees)
        self.append_diagnostic("telemetry", f"Arm setpoint: {degrees:.1f}°")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("arm_setpoint", degrees=float(degrees))
        if hasattr(self, "arm_setpoint_label"):
            self.arm_setpoint_label.setText(f"Setpoint: {degrees:.1f}°")
        if hasattr(self, "arm_status_label"):
            self.arm_status_label.setText("PID: Active")

    def send_arm_manual(self):
        speed = self.arm_manual_slider.value() / 100.0
        self.append_diagnostic("telemetry", f"Arm manual: speed={speed:.2f}")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("arm_manual", speed=speed)
        if hasattr(self, "arm_status_label"):
            self.arm_status_label.setText("PID: Inactive (manual)")

    def stop_arm_manual(self):
        if hasattr(self, "arm_manual_slider"):
            self.arm_manual_slider.setValue(0)
        self.append_diagnostic("telemetry", "Arm manual stop")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("arm_manual", speed=0.0)

    def disable_arm_pid(self):
        self.append_diagnostic("telemetry", "Arm PID disabled")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("arm_disable")
        if hasattr(self, "arm_status_label"):
            self.arm_status_label.setText("PID: Inactive")

    def reset_odometry(self):
        """Reset odometry pose on robot and local field widget."""
        logger.info("Odometry reset requested")
        self.append_diagnostic("telemetry", "Odometry reset requested")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("reset_odometry")
        start_x, start_y, start_theta = self._alliance_start_pose()
        self.current_pose = {"x": start_x, "y": start_y, "theta_deg": start_theta}
        self.expected_pose = self.current_pose.copy()
        self.field_widget.set_pose(start_x, start_y, start_theta)
        self.field_widget.set_expected_pose(start_x, start_y, start_theta)
        if hasattr(self, "odometry_field_widget"):
            self.odometry_field_widget.set_pose(start_x, start_y, start_theta)
            self.odometry_field_widget.set_expected_pose(start_x, start_y, start_theta)
        self.update_odometry_labels(start_x, start_y, start_theta)
    
    def set_auto_mode(self):
        """Switch robot to autonomous mode."""
        if self.run_selected_auto():
            logger.info("Started selected autonomous routine")
    
    def set_teleop_mode(self):
        """Switch robot to teleoperated mode (manual start)."""
        if self._set_robot_mode("TELEOP"):
            # Reset timer when manually starting teleop
            self.match_time_seconds = 0
            if not self.match_running:
                self.start_match_timer()
            logger.info("Switched to TELEOP mode (manual)")
            self.append_diagnostic("controls", "Mode changed to TELEOP")
    
    def auto_switch_to_teleop(self):
        """Automatically switch from AUTO to TELEOP after 30 seconds."""
        if self._set_robot_mode("TELEOP"):
            # Reset timer for teleop phase
            self.match_time_seconds = 0
            logger.info("Auto-switched from AUTO to TELEOP at 30 seconds")
            self.append_diagnostic("controls", "Mode auto-switched from AUTO to TELEOP")
    
    def reset_robot(self):
        """Reset robot to stopped state."""
        self.current_mode = "STOPPED"
        self.robot_status.setText("Stopped")
        self._update_auto_status_labels({"active": False, "status": "idle"})
        self.stop_match_timer()
        logger.info("Robot reset")
        self.append_diagnostic("controls", "Robot reset requested")
        if self.conn_manager.get_client() is not None:
            self.cmd_worker.enqueue("reset")

    def _set_robot_mode(self, mode):
        mode = str(mode).upper()
        if self.conn_manager.get_client() is None:
            return False

        # Update UI immediately; robot confirms via telemetry.
        self.current_mode = mode
        if mode == "AUTO":
            self.robot_status.setText("Autonomous")
        elif mode == "TELEOP":
            self.robot_status.setText("Teleoperated")
        else:
            self.robot_status.setText("Stopped")
            self._update_auto_status_labels({"active": False, "status": "idle"})
        self._update_odometry_context_labels()
        self.append_diagnostic("controls", f"Mode requested: {mode}")

        self.cmd_worker.enqueue("mode", mode=mode)
        return True

    def _set_control_mode_label(self, mode_name, color=None):
        if not hasattr(self, "control_mode_label"):
            return
        if color is None:
            self.control_mode_label.setText(f"Control: {mode_name}")
        else:
            self.control_mode_label.setText(f"Control: <b style='color: {color};'>{mode_name}</b>")

    def _set_face_button_style(self, button_index, active):
        labels = {
            0: self.button_a_label,
            1: self.button_b_label,
            2: self.button_x_label,
            3: self.button_y_label,
        }
        label = labels.get(button_index)
        if label is None:
            return
        label.setStyleSheet(f"color: {FACE_BUTTON_COLORS[button_index] if active else 'lightgray'}")

    def _scaled_axes(self, lx, ly, rx, ry):
        if self.slow_drive.isChecked():
            return (
                lx * SLOW_DRIVE_SCALE,
                ly * SLOW_DRIVE_SCALE,
                rx * SLOW_DRIVE_SCALE,
                ry * SLOW_DRIVE_SCALE,
            )
        return lx, ly, rx, ry
    
    def start_match_timer(self):
        """Start the match timer."""
        self.match_time_seconds = 0
        self.match_running = True
        self.match_timer.start(1000)  # Update every second
        self.update_match_time()
        logger.info("Match timer started")
    
    def stop_match_timer(self):
        """Stop the match timer."""
        self.match_running = False
        self.match_timer.stop()
        self.match_time_seconds = 0
        if hasattr(self, 'timer'):
            self.timer.setText("Time")
        logger.info("Match timer stopped")
    
    def update_match_time(self):
        """Update the match timer display."""
        if not self.match_running:
            return
        
        minutes = self.match_time_seconds // 60
        seconds = self.match_time_seconds % 60
        
        # AUTO phase logic (30 seconds)
        if self.current_mode == "AUTO":
            if self.match_time_seconds >= self.auto_duration:
                # Auto switch to teleop
                self.auto_switch_to_teleop()
                return
            
            time_str = f"Time: {minutes}:{seconds:02d} (Auto)"
        
        # TELEOP phase logic (3:30 = 210 seconds)
        elif self.current_mode == "TELEOP":
            if self.match_time_seconds >= self.teleop_duration:
                # Switch to overtime
                overtime_seconds = self.match_time_seconds - self.teleop_duration
                overtime_minutes = overtime_seconds // 60
                overtime_secs = overtime_seconds % 60
                time_str = f"Time: OVERTIME +{overtime_minutes}:{overtime_secs:02d}"
                
                # Optional: Change status color to indicate overtime
                self.robot_status.setStyleSheet("color: red; font-weight: bold;")
            else:
                time_str = f"Time: {minutes}:{seconds:02d} (Teleop)"

                

        # STOPPED or other modes
        else:
            time_str = f"Time: {minutes}:{seconds:02d}"
        
        if hasattr(self, 'timer'):
            self.timer.setText(time_str)
        
        self.match_time_seconds += 1
    
    def handle_ping_response(self, ping_ms):
        """Handle ping response from robot."""
        self.ping_label.setText(f"Ping: {ping_ms:.1f} ms")
        self._update_network_ping_label(ping_ms)
        self.append_diagnostic("connection", f"Ping response: {ping_ms:.1f} ms")
    
    def handle_telemetry(self, data):
        """Handle telemetry data from robot."""
        # Update UI with telemetry data
        # Example: battery, sensor readings, motor status, etc.
        try:
            field = data.get('field', {})
            pose = data.get('pose', {})
            odometry_mode = data.get('odometry_mode')
            alliance = data.get('alliance')
            auto_status = data.get('auto')
            encoders = data.get('encoders', [])

            width_m = float(field.get('width_m', self.field_widget.field_width_m))
            height_m = float(field.get('height_m', self.field_widget.field_height_m))
            x_m = float(pose.get('x', self.current_pose["x"]))
            y_m = float(pose.get('y', self.current_pose["y"]))
            theta_deg = float(pose.get('theta_deg', self.current_pose["theta_deg"]))

            self.field_widget.set_field_size(width_m, height_m)
            self.field_widget.set_pose(x_m, y_m, theta_deg)
            if hasattr(self, "odometry_field_widget"):
                self.odometry_field_widget.set_field_size(width_m, height_m)
                self.odometry_field_widget.set_pose(x_m, y_m, theta_deg)
            self.update_odometry_labels(x_m, y_m, theta_deg)
            self.current_pose = {"x": x_m, "y": y_m, "theta_deg": theta_deg}
            self.update_expected_pose()
            self.update_mechanism_encoder_labels(encoders)

            if odometry_mode and hasattr(self, 'label_odo_mode'):
                self.label_odo_mode.setText(f"Odometry Mode: {str(odometry_mode).title()}")
            if odometry_mode and hasattr(self, 'odo_tab_odo_mode_label'):
                self.odo_tab_odo_mode_label.setText(f"Odometry Mode: {str(odometry_mode).title()}")
            if alliance:
                self.current_alliance = str(alliance).upper()
                self._update_alliance_button()
            if auto_status is not None:
                self._update_auto_status_labels(auto_status)
            self._update_odometry_context_labels()
            self._update_network_telemetry_labels(data)

            elevator = data.get("elevator")
            if elevator is not None:
                self._update_elevator_labels(elevator, encoders)

            arm = data.get("arm")
            if arm is not None:
                self._update_arm_labels(arm)
        except Exception as e:
            logger.error(f"Error parsing telemetry pose: {e}")
            self.append_diagnostic("telemetry", f"Telemetry parse error: {e}")

        logger.debug(f"Telemetry: {data}")
        now = time.time()
        if now - self._last_diagnostic_telemetry_time >= 1.0:
            self.append_diagnostic_json("telemetry", "Telemetry update", data)
            self._last_diagnostic_telemetry_time = now
    
    def _update_elevator_labels(self, elevator: dict, encoders: list):
        active = elevator.get("active", False)
        setpoint = elevator.get("setpoint", 0)
        current = elevator.get("current_pos", 0.0)
        output = elevator.get("output", 0.0)

        if hasattr(self, "elev_status_label"):
            self.elev_status_label.setText("PID: Active" if active else "PID: Inactive")
        if hasattr(self, "elev_position_label"):
            self.elev_position_label.setText(f"Position: {current:.0f}")
        if hasattr(self, "elev_setpoint_label"):
            self.elev_setpoint_label.setText(f"Setpoint: {setpoint:.0f}")
        if hasattr(self, "elev_output_label"):
            self.elev_output_label.setText(f"Output: {output:.3f}")

    def _update_arm_labels(self, arm: dict):
        active = arm.get("active", False)
        current = arm.get("current_deg", 0.0)
        setpoint = arm.get("setpoint_deg", 0.0)
        output = arm.get("output", 0.0)

        if hasattr(self, "arm_status_label"):
            self.arm_status_label.setText("PID: Active" if active else "PID: Inactive")
        if hasattr(self, "arm_position_label"):
            self.arm_position_label.setText(f"Position: {current:.1f}°")
        if hasattr(self, "arm_setpoint_label"):
            self.arm_setpoint_label.setText(f"Setpoint: {setpoint:.1f}°")
        if hasattr(self, "arm_output_label"):
            self.arm_output_label.setText(f"Output: {output:.3f}")

    def update_keyboard_speed(self, value):
        """Update keyboard speed from slider."""
        self.keyboard_speed = value / 100.0
        if hasattr(self, 'keyboard_speed_label'):
            self.keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")
        if hasattr(self, 'settings_keyboard_speed_label'):
            self.settings_keyboard_speed_label.setText(f"Keyboard Speed: {self.keyboard_speed:.0%}")
        if hasattr(self, 'settings_keyboard_speed_slider') and self.settings_keyboard_speed_slider.value() != value:
            self.settings_keyboard_speed_slider.blockSignals(True)
            self.settings_keyboard_speed_slider.setValue(value)
            self.settings_keyboard_speed_slider.blockSignals(False)
        logger.debug(f"Keyboard speed set to {self.keyboard_speed:.0%}")
    
    def keyPressEvent(self, event):
        """Handle keyboard key press events."""
        if not self.keyboard_enabled:
            return
        
        key = event.key()
        
        # Add key to pressed set
        self.keys_pressed.add(key)
        
        # Don't process if auto-repeat
        if event.isAutoRepeat():
            return

        # Log key presses for debugging
        key_names = {
            Qt.Key.Key_W: "W", Qt.Key.Key_A: "A", 
            Qt.Key.Key_S: "S", Qt.Key.Key_D: "D",
            Qt.Key.Key_Q: "Q", Qt.Key.Key_E: "E",
            Qt.Key.Key_Space: "Space", Qt.Key.Key_Shift: "Shift"
        }
        
        if key in key_names:
            logger.debug(f"Key pressed: {key_names[key]}")
    
    def keyReleaseEvent(self, event):
        """Handle keyboard key release events."""
        if not self.keyboard_enabled:
            return
        
        key = event.key()
        
        # Remove key from pressed set
        self.keys_pressed.discard(key)
        
        # Don't process if auto-repeat
        if event.isAutoRepeat():
            return
    
    def calculate_keyboard_input(self):
        """Calculate joystick values from keyboard input."""
        lx = 0.0  # Left/right strafe
        ly = 0.0  # Forward/backward
        rx = 0.0  # Rotation
        
        # Base speed (can be boosted with Shift)
        speed = self.keyboard_speed
        if Qt.Key.Key_Shift in self.keys_pressed:
            speed = 1.0  # Full speed with shift

        if self.slow_drive.isChecked():
            speed *= SLOW_DRIVE_SCALE
        
        # Movement keys
        if Qt.Key.Key_W in self.keys_pressed:
            ly += speed
        if Qt.Key.Key_S in self.keys_pressed:
            ly -= speed
        if Qt.Key.Key_A in self.keys_pressed:
            lx -= speed
        if Qt.Key.Key_D in self.keys_pressed:
            lx += speed
        
        # Rotation keys
        if Qt.Key.Key_Q in self.keys_pressed:
            rx -= speed
        if Qt.Key.Key_E in self.keys_pressed:
            rx += speed
        
        # Emergency stop
        if Qt.Key.Key_Space in self.keys_pressed:
            lx = ly = rx = 0.0
        
        return lx, ly, rx, 0.0  # ry not used for keyboard

    def init_pygame_and_joystick(self):
        """Initialize pygame and detect joystick."""
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.gamepad_label.setText(f"Gamepad: {self.joystick.get_name()}")
                logger.info(f"Found joystick: {self.joystick.get_name()}")
            else:
                self.gamepad_label.setText("Gamepad: Not Found")
                logger.warning("No joystick found")
        except Exception as e:
            logger.error(f"Error initializing pygame/joystick: {e}")
            self.gamepad_label.setText("Gamepad: Error")

    def update_connection_status(self, is_connected, address):
        """Update UI based on connection status."""
        if is_connected:
            self.status_label.setText("Status: <b style='color: green;'>Connected</b>")
            self.address_label.setText(f"Address: {address}")
            self._update_network_status_labels(True, address)
            logger.info(f"Connected to {address}")
            self.append_diagnostic("connection", f"Connected to {address}")
        else:
            self.status_label.setText("Status: <b style='color: red;'>Disconnected</b>")
            self.address_label.setText("Address: N/A")
            self.ping_label.setText("Ping: -- ms")
            self._update_network_status_labels(False, "N/A")
            self.robot_status.setText("Stopped")
            self.current_mode = "STOPPED"
            self._update_auto_status_labels({"active": False, "status": "idle"})
            self._update_odometry_context_labels()
            
            # Reset button colors
            self.button_a_label.setStyleSheet("color: lightgray")
            self.button_b_label.setStyleSheet("color: lightgray")
            self.button_x_label.setStyleSheet("color: lightgray")
            self.button_y_label.setStyleSheet("color: lightgray")
            
            logger.warning("Disconnected from robot")
            self.append_diagnostic("connection", "Disconnected from robot")

    def values_changed_significantly(self, old_values, new_values, threshold=JOYSTICK_THRESHOLD):
        """Check if joystick values changed beyond threshold."""
        return any(abs(old_values[k] - new_values[k]) > threshold for k in old_values)

    def poll_gamepad(self):
        """Poll gamepad state and send updates to robot."""
        # Only check connection status – never block on ZMQ here.
        is_connected = self.conn_manager.get_client() is not None
        if not is_connected:
            return

        try:
            # Check if we have keyboard input
            keyboard_input = self.calculate_keyboard_input()
            has_keyboard_input = any(abs(v) > 0.01 for v in keyboard_input)
            
            # Update control mode indicator
            if has_keyboard_input:
                self._set_control_mode_label("Keyboard", color="blue")
            elif self.joystick is not None:
                self._set_control_mode_label("Gamepad", color="green")
            else:
                self._set_control_mode_label("None")
            
            # Use keyboard input if active, otherwise use joystick
            if has_keyboard_input:
                self.joystick_values['lx'] = keyboard_input[0]
                self.joystick_values['ly'] = keyboard_input[1]
                self.joystick_values['rx'] = keyboard_input[2]
                self.joystick_values['ry'] = keyboard_input[3]
            elif self.joystick is not None:
                # Poll joystick only if no keyboard input
                pygame.event.pump()
                # Read and apply deadzone to joystick axes
                axis_lx = self.joystick.get_axis(0)
                axis_ly = self.joystick.get_axis(1)
                axis_rx = self.joystick.get_axis(2)
                axis_ry = self.joystick.get_axis(4)

                self.joystick_values['lx'] = axis_lx if abs(axis_lx) > AXIS_DEADZONE else 0.0
                self.joystick_values['ly'] = -axis_ly if abs(axis_ly) > AXIS_DEADZONE else 0.0
                self.joystick_values['rx'] = axis_rx if abs(axis_rx) > AXIS_DEADZONE else 0.0
                self.joystick_values['ry'] = -axis_ry if abs(axis_ry) > AXIS_DEADZONE else 0.0

                # Handle button events – enqueue, never block
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        self.cmd_worker.enqueue("button", button_id=event.button, action="DOWN")
                        if event.button in FACE_BUTTON_COLORS:
                            self._set_face_button_style(event.button, active=True)
                    elif event.type == pygame.JOYBUTTONUP:
                        self.cmd_worker.enqueue("button", button_id=event.button, action="UP")
                        if event.button in FACE_BUTTON_COLORS:
                            self._set_face_button_style(event.button, active=False)
            else:
                # No input - zero everything
                self.joystick_values = {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0}

            lx, ly, rx, ry = self._scaled_axes(
                self.joystick_values['lx'],
                self.joystick_values['ly'],
                self.joystick_values['rx'],
                self.joystick_values['ry'],
            )
            self.joystick_values['lx'] = lx
            self.joystick_values['ly'] = ly
            self.joystick_values['rx'] = rx
            self.joystick_values['ry'] = ry

            # Update UI labels
            self.lx_label.setText(f"LX: {self.joystick_values['lx']:.2f}")
            self.ly_label.setText(f"LY: {self.joystick_values['ly']:.2f}")
            self.rx_label.setText(f"RX: {self.joystick_values['rx']:.2f}")
            self.ry_label.setText(f"RY: {self.joystick_values['ry']:.2f}")
            self.update_expected_pose()

            control_snapshot = (
                "mode="
                f"{'keyboard' if has_keyboard_input else 'gamepad' if self.joystick is not None else 'none'} "
                f"lx={self.joystick_values['lx']:.2f} "
                f"ly={self.joystick_values['ly']:.2f} "
                f"rx={self.joystick_values['rx']:.2f} "
                f"ry={self.joystick_values['ry']:.2f}"
            )
            if control_snapshot != self._last_diagnostic_controls:
                self.append_diagnostic("controls", control_snapshot)
                self._last_diagnostic_controls = control_snapshot

            should_send_joystick = (
                self.current_mode == "TELEOP"
                or self.values_changed_significantly(self.last_sent_joystick_values, self.joystick_values)
            )

            # Keep the coprocessor watchdog fed – enqueue so the UI never blocks.
            if should_send_joystick:
                self.cmd_worker.send_joystick(
                    self.joystick_values['lx'],
                    self.joystick_values['ly'],
                    self.joystick_values['rx'],
                    self.joystick_values['ry']
                )
                self.last_sent_joystick_values = self.joystick_values.copy()
                
        except Exception as e:
            logger.error(f"Error polling gamepad: {e}")

    def closeEvent(self, event):
        """Clean up resources on application close."""
        logger.info("Closing application...")
        
        try:
            self.stop_camera_stream()

            # Stop threads
            self.telemetry_receiver.stop()
            self.conn_manager.stop()
            
            # Wait for threads to finish (with timeout)
            self.telemetry_receiver.join(timeout=2)
            self.conn_manager.join(timeout=2)
            
            # Clean up joystick
            if self.joystick:
                self.joystick.quit()
                
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")
        finally:
            pygame.quit()
            event.accept()
            logger.info("Application closed")


def main():
    os.system('cls' if os.name == 'nt' else 'clear')
    app = QApplication(sys.argv)
    icon_path = os.path.join(os.path.dirname(__file__), "app_icon.png")
    if os.path.exists(icon_path):
        app.setWindowIcon(QIcon(icon_path))
    window = AppWindow()
    if os.path.exists(icon_path):
        window.setWindowIcon(QIcon(icon_path))
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
