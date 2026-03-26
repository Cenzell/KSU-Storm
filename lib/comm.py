"""Robot communications helpers using ZeroMQ."""

from __future__ import annotations

import threading
import time
from typing import Optional

import zmq
from PyQt6.QtCore import QObject, pyqtSignal

# Configuration
ROBOT_ADDRESSES = [
    "10.42.0.85",
    "10.42.0.3",
    "10.42.0.2",
    "127.0.0.1",
    "10.222.255.253"
]
COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
PING_INTERVAL_S = 1
HEARTBEAT_TIMEOUT_S = 2.0
COMMAND_TIMEOUT_MS = 2000
TELEMETRY_TIMEOUT_MS = 100

class WorkerSignals(QObject):
    """Signals for communication with Qt GUI thread."""
    connection_status = pyqtSignal(bool, str)
    ping_response = pyqtSignal(float)
    telemetry_update = pyqtSignal(dict)


class RobotClient:
    """Client that manages command (REQ/REP) and telemetry (SUB) sockets."""

    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip
        self.context = zmq.Context()
        self.signals = WorkerSignals()

        self.command_socket = self.context.socket(zmq.REQ)
        self.command_socket.connect(f"tcp://{robot_ip}:{COMMAND_PORT}")
        self.command_socket.setsockopt(zmq.RCVTIMEO, COMMAND_TIMEOUT_MS)
        self.command_socket.setsockopt(zmq.LINGER, 0)

        self.telemetry_socket = self.context.socket(zmq.SUB)
        self.telemetry_socket.connect(f"tcp://{robot_ip}:{TELEMETRY_PORT}")
        self.telemetry_socket.subscribe("")
        self.telemetry_socket.setsockopt(zmq.RCVTIMEO, TELEMETRY_TIMEOUT_MS)
        self.telemetry_socket.setsockopt(zmq.LINGER, 0)

        self.connected = False
        self.running = True
        self.last_ping_time = 0
        self.ping_sent_time = None

        print(f"[RobotClient] Initialized connection to {robot_ip}")

    def _set_connected(self, connected: bool) -> None:
        if self.connected == connected:
            return
        self.connected = connected
        self.signals.connection_status.emit(connected, self.robot_ip if connected else "")

    def send_command(self, command_type: str, **kwargs) -> Optional[dict]:
        """Send a command to the robot and wait for a response."""
        try:
            command = {"type": command_type, "timestamp": time.time(), **kwargs}
            self.command_socket.send_json(command)
            response = self.command_socket.recv_json()

            self._set_connected(True)
            return response
        except zmq.Again:
            self._set_connected(False)
            return None
        except Exception as e:
            print(f"[RobotClient] Command error: {e}")
            self._set_connected(False)
            return None

    def send_joystick(self, lx: float, ly: float, rx: float, ry: float) -> Optional[dict]:
        return self.send_command("joystick", lx=lx, ly=ly, rx=rx, ry=ry)

    def send_button(self, button_id: int, action: str) -> Optional[dict]:
        return self.send_command("button", button_id=button_id, action=action)

    def set_mode(self, mode: str) -> Optional[dict]:
        return self.send_command("mode", mode=mode)

    def reset_robot(self) -> Optional[dict]:
        return self.send_command("reset")

    def send_ping(self) -> Optional[dict]:
        self.ping_sent_time = time.time()
        return self.send_command("ping")

    def receive_telemetry(self) -> Optional[dict]:
        """Try to receive telemetry (non-blocking)."""
        try:
            data = self.telemetry_socket.recv_json(flags=zmq.NOBLOCK)

            self._set_connected(True)
            self.signals.telemetry_update.emit(data)
            return data
        except zmq.Again:
            return None
        except Exception as e:
            print(f"[RobotClient] Telemetry error: {e}")
            return None

    def cleanup(self) -> None:
        """Clean up sockets and terminate context."""
        self.running = False
        self.command_socket.close(0)
        self.telemetry_socket.close(0)
        self.context.term()


class ConnectionManager(threading.Thread):
    """Manage connection attempts across candidate robot addresses."""

    def __init__(self):
        super().__init__()
        self.signals = WorkerSignals()
        self.client: Optional[RobotClient] = None
        self.lock = threading.Lock()
        self.running = True
        self.current_address_idx = 0
        self.daemon = True

    def _advance_address(self) -> None:
        self.current_address_idx = (self.current_address_idx + 1) % len(ROBOT_ADDRESSES)

    def run(self) -> None:
        print("[ConnectionManager] Starting...")

        while self.running:
            with self.lock:
                if self.client is None or not self.client.connected:
                    address = ROBOT_ADDRESSES[self.current_address_idx]
                    print(f"[ConnectionManager] Attempting {address}...")

                    try:
                        if self.client:
                            self.client.cleanup()

                        self.client = RobotClient(address)
                        self.client.signals = self.signals

                        response = self.client.send_ping()
                        if response and response.get("status") == "success":
                            print(f"[ConnectionManager] ✅ Connected to {address}")
                            self.signals.connection_status.emit(True, f"{address}:{COMMAND_PORT}")
                        else:
                            self._advance_address()
                            self.client = None
                    except Exception as e:
                        print(f"[ConnectionManager] Connection failed: {e}")
                        self._advance_address()
                        self.client = None
                        self.signals.connection_status.emit(False, "")

            time.sleep(1.0 if self.client is None else 0.5)

    def get_client(self) -> Optional[RobotClient]:
        with self.lock:
            return self.client if self.client and self.client.connected else None

    def stop(self) -> None:
        self.running = False
        with self.lock:
            if self.client:
                self.client.cleanup()


class TelemetryReceiver(threading.Thread):
    """Continuously receive telemetry and send periodic pings."""

    def __init__(self, conn_manager):
        super().__init__()
        self.conn_manager = conn_manager
        self.running = True
        self.last_ping_time = 0
        self.daemon = True

    def run(self) -> None:
        print("[TelemetryReceiver] Starting...")

        while self.running:
            client = self.conn_manager.get_client()

            if client:
                client.receive_telemetry()
                if time.time() - self.last_ping_time > PING_INTERVAL_S:
                    ping_start = time.time()
                    response = client.send_ping()

                    if response and response.get("status") == "success":
                        ping_ms = (time.time() - ping_start) * 1000
                        client.signals.ping_response.emit(ping_ms)

                    self.last_ping_time = time.time()
            else:
                time.sleep(0.1)

            time.sleep(0.01)

    def stop(self) -> None:
        self.running = False
