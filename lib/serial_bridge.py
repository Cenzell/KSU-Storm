import json
import logging
import threading
import time
from typing import Any, Dict, Optional

import serial

logger = logging.getLogger(__name__)
logger.info("serial_bridge module loaded from this file")
SERIAL_BRIDGE_DEBUG = True


class SerialBridge:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        read_timeout: float = 0.05,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout

        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        self.last_rx_time = 0.0
        self.last_status: Dict[str, Any] = {}
        self.latest_telemetry: Dict[str, Any] = {
            "encoders": [0, 0, 0, 0, 0, 0, 0],
            "relay": 0,
            "servo_ok": True,
            "mode": "STOPPED",
            "drive_enabled": False,
            "drive_cmd": [0.0, 0.0, 0.0, 0.0],
            "mech_cmd": [0.0, 0.0, 0.0],
            "last_status": {},
            "bridge_connected": False,
        }

    def connect(self) -> None:
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.read_timeout)
        self.running = True
        self.rx_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name="serial-bridge-rx",
        )
        self.rx_thread.start()
        self.send({"type": "ping"})
        logger.info("Serial bridge connected on %s @ %d", self.port, self.baudrate)

    def close(self) -> None:
        self.running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send(self, obj: Dict[str, Any]) -> bool:
        try:
            line = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
        except Exception as e:
            logger.error("JSON encode failed: %s", e)
            raise

        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(line)
                return True
            except Exception as e:
                logger.error("Serial write failed: %s", e)
                return False

    def set_drive_motors(self, motors: list[float]) -> bool:
        if len(motors) != 4:
            raise ValueError("drive motors must have length 4")
        if SERIAL_BRIDGE_DEBUG:
            logger.info("TX drive motors: %s", [round(float(x), 3) for x in motors])
        return self.send({"type": "drive", "motors": [float(x) for x in motors]})

    def set_mech_motors(self, motors: list[float]) -> bool:
        if len(motors) != 3:
            raise ValueError("mechanism motors must have length 3")
        return self.send({"type": "mech", "motors": [float(x) for x in motors]})

    def set_servo(self, channel: int, value: float) -> bool:
        return self.send({"type": "servo", "channel": int(channel), "value": float(value)})

    def set_relay(self, on: bool) -> bool:
        return self.send({"type": "relay", "on": bool(on)})

    def set_led(self, r: int, g: int, b: int, w: int = 0) -> bool:
        return self.send({
            "type": "led",
            "r": int(r),
            "g": int(g),
            "b": int(b),
            "w": int(w),
        })

    def set_mode(self, mode: str) -> bool:
        return self.send({"type": "mode", "mode": str(mode).upper()})

    def reset(self) -> bool:
        return self.send({"type": "reset"})

    def get_latest_telemetry(self) -> Dict[str, Any]:
        connected = (time.time() - self.last_rx_time) < 1.0
        data = dict(self.latest_telemetry)
        data["last_status"] = dict(self.last_status)
        data["bridge_connected"] = connected
        return data

    def _handle_message(self, msg: Dict[str, Any]) -> None:
        self.last_rx_time = time.time()
        msg_type = str(msg.get("type", "")).lower()

        if msg_type == "telemetry":
            self.latest_telemetry.update(msg)
            if SERIAL_BRIDGE_DEBUG:
                logger.info(
                    "RX telemetry: mode=%s drive_enabled=%s drive_cmd=%s relay=%s encoders=%s",
                    msg.get("mode"),
                    msg.get("drive_enabled"),
                    msg.get("drive_cmd"),
                    msg.get("relay"),
                    msg.get("encoders"),
                )
        elif msg_type in ("ack", "pong", "hello", "fault", "status"):
            self.last_status = msg
            self.latest_telemetry["last_status"] = dict(msg)
            if SERIAL_BRIDGE_DEBUG:
                logger.info("RX status: %s", msg)
            if msg_type == "fault":
                logger.warning("MCU fault: %s", msg)
        else:
            logger.debug("Unhandled MCU message: %s", msg)

    def _read_loop(self) -> None:
        assert self.ser is not None
        while self.running and self.ser is not None:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    logger.warning("Bad serial JSON: %r", line)
                    continue

                if isinstance(msg, dict):
                    self._handle_message(msg)

            except Exception as e:
                logger.error("Serial read failed: %s", e)
                time.sleep(0.1)
