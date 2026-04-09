"""Field Management System (FMS) API client.

Polls the FMS REST API in a background thread and emits Qt signals
when match data or connection state changes.

Future circuit integration
--------------------------
When the voltage circuit is ready, hook into the ``voltage_required``
signal or override ``on_voltage_required`` in a subclass.  The voltage
will always be one of {2, 4, 6, 8, 10} volts, or 0 when no match is
active.
"""

from __future__ import annotations

import json
import logging
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal

logger = logging.getLogger(__name__)

# ── configuration ─────────────────────────────────────────────────────────────

FMS_BASE_URL = "http://10.0.0.1:5173/api/v1"
FMS_POLL_INTERVAL_S = 1.0
FMS_TIMEOUT_S = 2.0

# Grid Voltage: randomly chosen per match from {2, 4, 6, 8, 10} V  (§3.3.5)
VALID_VOLTAGES = frozenset({2, 4, 6, 8, 10})
# Grid Frequency: randomly chosen per match from {20, 30, 40, 50} RPM  (§3.3.4)
VALID_RPMS = frozenset({20, 30, 40, 50})


# ── data model ────────────────────────────────────────────────────────────────

@dataclass
class FMSMatchData:
    """Parsed snapshot of one FMS poll cycle."""
    required_voltage: float   # always one of VALID_VOLTAGES (or 0 = no match)
    required_rpm: float       # always one of VALID_RPMS    (or 0 = no match)
    time_remaining_s: float   # seconds; 240 when match has not started
    match_active: bool        # False when no live/upcoming match


# ── signals ───────────────────────────────────────────────────────────────────

class FMSSignals(QObject):
    """Qt signals emitted from the FMS polling thread."""
    # Emitted each poll with fresh data, or None when the FMS returns 400.
    match_update = pyqtSignal(object)
    # Emitted when reachability changes (True = reachable, False = unreachable).
    connection_changed = pyqtSignal(bool)
    # Emitted only when the required voltage value actually changes.
    voltage_required = pyqtSignal(float)
    # Emitted only when the required RPM value actually changes.
    rpm_required = pyqtSignal(float)


# ── poller ────────────────────────────────────────────────────────────────────

class FMSPoller(threading.Thread):
    """Background thread that polls /match and /time every second."""

    def __init__(self, base_url: str = FMS_BASE_URL) -> None:
        super().__init__(daemon=True, name="FMSPoller")
        self.base_url = base_url.rstrip("/")
        self.signals = FMSSignals()

        self._running = True
        self._last_reachable: Optional[bool] = None
        self._last_voltage: Optional[float] = None
        self._last_rpm: Optional[float] = None

    # ── public ────────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._running = False

    # ── internal ──────────────────────────────────────────────────────────

    def run(self) -> None:
        logger.info("FMS poller started – polling %s", self.base_url)
        while self._running:
            self._poll_once()
            time.sleep(FMS_POLL_INTERVAL_S)

    def _fetch(self, path: str) -> Optional[tuple[int, dict]]:
        """Return (status_code, body_dict) or None on network error."""
        url = f"{self.base_url}{path}"
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=FMS_TIMEOUT_S) as resp:
                body = json.loads(resp.read().decode())
                return resp.status, body
        except urllib.error.HTTPError as exc:
            return exc.code, {}
        except Exception:
            return None

    def _poll_once(self) -> None:
        match_result = self._fetch("/match")
        time_result = self._fetch("/time")

        # Network unreachable
        if match_result is None or time_result is None:
            self._set_reachable(False)
            self.signals.match_update.emit(None)
            return

        self._set_reachable(True)

        match_status, match_body = match_result
        time_status, time_body = time_result

        if match_status != 200 or time_status != 200:
            # No active or upcoming match
            self.signals.match_update.emit(None)
            self._notify_voltage(0.0)
            return

        raw_voltage = float(match_body.get("requiredVoltage", 0))
        voltage = self._clamp_voltage(raw_voltage)
        raw_rpm = float(match_body.get("requiredRpm", 0))
        rpm = self._clamp_rpm(raw_rpm)
        time_remaining = float(time_body.get("time", 240))
        match_active = time_remaining < 240.0

        data = FMSMatchData(
            required_voltage=voltage,
            required_rpm=rpm,
            time_remaining_s=time_remaining,
            match_active=match_active,
        )
        self.signals.match_update.emit(data)
        self._notify_voltage(voltage)
        self._notify_rpm(rpm)

    def _set_reachable(self, reachable: bool) -> None:
        if self._last_reachable != reachable:
            self._last_reachable = reachable
            self.signals.connection_changed.emit(reachable)
            logger.info("FMS reachable: %s", reachable)

    def _notify_voltage(self, voltage: float) -> None:
        if self._last_voltage != voltage:
            self._last_voltage = voltage
            self.signals.voltage_required.emit(voltage)
            logger.info("FMS required voltage changed: %sV", voltage)

    def _notify_rpm(self, rpm: float) -> None:
        if self._last_rpm != rpm:
            self._last_rpm = rpm
            self.signals.rpm_required.emit(rpm)
            logger.info("FMS required RPM changed: %s RPM", rpm)

    @staticmethod
    def _clamp_voltage(raw: float) -> float:
        """Snap raw voltage to the nearest valid step {2,4,6,8,10}."""
        if raw in VALID_VOLTAGES:
            return raw
        return min(VALID_VOLTAGES, key=lambda v: abs(v - raw))

    @staticmethod
    def _clamp_rpm(raw: float) -> float:
        """Snap raw RPM to the nearest valid step {20,30,40,50}."""
        if raw in VALID_RPMS:
            return raw
        return min(VALID_RPMS, key=lambda v: abs(v - raw))
