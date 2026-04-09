#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import socketserver
import threading
import time
from dataclasses import dataclass
from http import server
from typing import Any, Callable, Dict, Optional, Tuple, Union

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


TAG_SIZE_METERS = 0.0492125
DEFAULT_WIDTH = int(os.environ.get("KSU_CAMERA_WIDTH", "640"))
DEFAULT_HEIGHT = int(os.environ.get("KSU_CAMERA_HEIGHT", "480"))
DEFAULT_FX = float(os.environ.get("KSU_CAMERA_FX", "700.0"))
DEFAULT_FY = float(os.environ.get("KSU_CAMERA_FY", "700.0"))
DEFAULT_CX = float(os.environ.get("KSU_CAMERA_CX", str(DEFAULT_WIDTH / 2.0)))
DEFAULT_CY = float(os.environ.get("KSU_CAMERA_CY", str(DEFAULT_HEIGHT / 2.0)))
ENABLE_UNDISTORT = os.environ.get("KSU_ENABLE_UNDISTORT", "1").strip().lower() not in ("0", "false", "no")
PORT = int(os.environ.get("KSU_CAMERA_PORT", "8080"))

APRILTAG_FAMILY = "tag36h11"
NTHREADS = 4
QUAD_DECIMATE = 2.0
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
DEBUG = 0
CAMERA_INIT_RETRY_DELAY_S = float(os.environ.get("KSU_CAMERA_INIT_RETRY_DELAY_S", "2.0"))
CAMERA_INIT_MAX_ATTEMPTS = int(os.environ.get("KSU_CAMERA_INIT_MAX_ATTEMPTS", "3"))


def _is_truthy_env(name: str, default: str) -> bool:
    return os.environ.get(name, default).strip().lower() not in ("0", "false", "no")


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default).strip()


@dataclass(frozen=True)
class CameraConfig:
    name: str
    label: str
    backend: str
    selector: str
    width: int
    height: int
    calibration_file: str
    enable_apriltag: bool


class StreamingOutput:
    def __init__(self) -> None:
        self.frame: Optional[bytes] = None
        self.condition = threading.Condition()

    def write(self, frame_bytes: bytes) -> None:
        with self.condition:
            self.frame = frame_bytes
            self.condition.notify_all()


class CameraWorker:
    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self.output = StreamingOutput()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name=f"camera-{self.config.name}",
        )
        self.thread.start()

    def _run(self) -> None:
        configure_opencv_logging()

        detector = build_detector() if self.config.enable_apriltag else None
        calibration = load_calibration(self.config.calibration_file)
        if calibration:
            camera_params = [
                calibration["fx"],
                calibration["fy"],
                calibration["cx"],
                calibration["cy"],
            ]
        else:
            camera_params = [
                DEFAULT_FX,
                DEFAULT_FY,
                DEFAULT_CX,
                DEFAULT_CY,
            ]
            print(f"[{self.config.name}] Using fallback intrinsics")

        attempts = 0
        close_source: Optional[Callable[[], None]] = None

        while True:
            try:
                backend, capture_rgb_frame, close_source = create_frame_source(self.config)
                print(f"[{self.config.name}] Camera backend: {backend}")
                if backend == "opencv":
                    print(f"[{self.config.name}] OpenCV source: {self.config.selector}")
                break
            except Exception as exc:
                attempts += 1
                if attempts >= max(1, CAMERA_INIT_MAX_ATTEMPTS):
                    print(f"[{self.config.name}] Camera disabled: {exc}")
                    return
                print(
                    f"[{self.config.name}] Camera init failed: {exc}. "
                    f"Retrying in {CAMERA_INIT_RETRY_DELAY_S:.1f}s..."
                )
                time.sleep(CAMERA_INIT_RETRY_DELAY_S)

        last_print_time = 0.0

        try:
            while True:
                frame = capture_rgb_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                if calibration and calibration["dist_coeffs"] is not None and ENABLE_UNDISTORT:
                    frame = cv2.undistort(
                        frame,
                        calibration["camera_matrix"],
                        calibration["dist_coeffs"],
                    )

                if detector is not None:
                    frame, last_print_time = annotate_apriltags(
                        frame,
                        detector,
                        camera_params,
                        self.config.name,
                        last_print_time,
                    )

                cv2.putText(
                    frame,
                    self.config.label,
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                )

                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    self.output.write(jpeg.tobytes())
        finally:
            if close_source is not None:
                close_source()


def default_camera_configs() -> Dict[str, CameraConfig]:
    shared_calibration = _env("KSU_CAMERA_CALIBRATION_FILE", "camera_calibration.json")
    return {
        # USB camera, OpenCV index 0.
        "front_left": CameraConfig(
            name="front_left",
            label=_env("KSU_CAMERA_FRONT_LEFT_LABEL", "Front Left"),
            backend=_env("KSU_CAMERA_FRONT_LEFT_BACKEND", "opencv"),
            selector=_env("KSU_CAMERA_FRONT_LEFT_SELECTOR", "0"),
            width=int(_env("KSU_CAMERA_FRONT_LEFT_WIDTH", str(DEFAULT_WIDTH))),
            height=int(_env("KSU_CAMERA_FRONT_LEFT_HEIGHT", str(DEFAULT_HEIGHT))),
            calibration_file=_env(
                "KSU_CAMERA_FRONT_LEFT_CALIBRATION_FILE",
                shared_calibration,
            ),
            enable_apriltag=_is_truthy_env("KSU_CAMERA_FRONT_LEFT_ENABLE_APRILTAG", "1"),
        ),
        # USB camera, OpenCV index 1.
        "front_right": CameraConfig(
            name="front_right",
            label=_env("KSU_CAMERA_FRONT_RIGHT_LABEL", "Front Right"),
            backend=_env("KSU_CAMERA_FRONT_RIGHT_BACKEND", "opencv"),
            selector=_env("KSU_CAMERA_FRONT_RIGHT_SELECTOR", "1"),
            width=int(_env("KSU_CAMERA_FRONT_RIGHT_WIDTH", str(DEFAULT_WIDTH))),
            height=int(_env("KSU_CAMERA_FRONT_RIGHT_HEIGHT", str(DEFAULT_HEIGHT))),
            calibration_file=_env(
                "KSU_CAMERA_FRONT_RIGHT_CALIBRATION_FILE",
                shared_calibration,
            ),
            enable_apriltag=_is_truthy_env("KSU_CAMERA_FRONT_RIGHT_ENABLE_APRILTAG", "1"),
        ),
        # Primary driver view — Pi camera on CSI connector 0.
        # AprilTag annotation disabled: overlays are distracting for the driver.
        "driver": CameraConfig(
            name="driver",
            label=_env("KSU_CAMERA_DRIVER_LABEL", "Driver"),
            backend=_env("KSU_CAMERA_DRIVER_BACKEND", "picamera2"),
            selector=_env("KSU_CAMERA_DRIVER_SELECTOR", "0"),
            width=int(_env("KSU_CAMERA_DRIVER_WIDTH", str(DEFAULT_WIDTH))),
            height=int(_env("KSU_CAMERA_DRIVER_HEIGHT", str(DEFAULT_HEIGHT))),
            calibration_file=_env("KSU_CAMERA_DRIVER_CALIBRATION_FILE", shared_calibration),
            enable_apriltag=_is_truthy_env("KSU_CAMERA_DRIVER_ENABLE_APRILTAG", "0"),
        ),
    }


CAMERA_WORKERS = {
    name: CameraWorker(config)
    for name, config in default_camera_configs().items()
}


def parse_opencv_source(raw: str) -> Union[int, str]:
    if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
        return int(raw)
    return raw


def resolve_camera_backend(config: CameraConfig) -> str:
    backend = config.backend.lower()
    if backend in {"picamera2", "opencv"}:
        return backend
    return "picamera2" if Picamera2 is not None else "opencv"


def _create_picamera(config: CameraConfig):
    if Picamera2 is None:
        raise RuntimeError("Picamera2 is not installed")

    camera_index = int(config.selector)
    camera_info = Picamera2.global_camera_info()
    if len(camera_info) <= camera_index:
        raise RuntimeError(f"Camera index {camera_index} not found")

    try:
        picam2 = Picamera2(camera_num=camera_index)
    except TypeError:
        picam2 = Picamera2(camera_index)

    video_config = picam2.create_video_configuration(
        main={"size": (config.width, config.height), "format": "RGB888"}
    )
    picam2.configure(video_config)
    picam2.start()
    time.sleep(1.0)

    def capture_rgb_frame():
        return picam2.capture_array()

    def close_source():
        try:
            picam2.stop()
        except Exception:
            pass

    return capture_rgb_frame, close_source


def _create_opencv_capture(config: CameraConfig):
    source = parse_opencv_source(config.selector)
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open OpenCV camera source: {source}")

    def capture_rgb_frame():
        ok, frame_bgr = cap.read()
        if not ok:
            return None
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    def close_source():
        cap.release()

    return capture_rgb_frame, close_source


def create_frame_source(config: CameraConfig):
    backend = resolve_camera_backend(config)
    if backend == "picamera2":
        try:
            capture_rgb_frame, close_source = _create_picamera(config)
            return backend, capture_rgb_frame, close_source
        except Exception as exc:
            if config.backend.lower() == "auto":
                print(f"[{config.name}] Picamera2 init failed ({exc}), falling back to OpenCV")
            else:
                raise

    capture_rgb_frame, close_source = _create_opencv_capture(config)
    return "opencv", capture_rgb_frame, close_source


def configure_opencv_logging() -> None:
    try:
        cv2.setLogLevel(0)
    except AttributeError:
        try:
            cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
        except Exception:
            pass
    except Exception:
        pass


def load_calibration(path: str) -> Optional[Dict[str, Any]]:
    if not path or not os.path.exists(path):
        return None

    try:
        with open(path, "r", encoding="utf-8") as file_obj:
            data = json.load(file_obj)

        matrix = data.get("camera_matrix")
        if not matrix:
            print(f"Calibration file missing camera_matrix: {path}")
            return None

        dist = data.get("dist_coeffs", [])
        image_size = data.get("image_size")

        camera_matrix = np.array(matrix, dtype=np.float64)
        dist_coeffs = np.array(dist, dtype=np.float64) if dist else None

        return {
            "fx": float(camera_matrix[0][0]),
            "fy": float(camera_matrix[1][1]),
            "cx": float(camera_matrix[0][2]),
            "cy": float(camera_matrix[1][2]),
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
            "image_size": image_size,
        }
    except Exception as exc:
        print(f"Failed to load calibration file {path}: {exc}")
        return None


def build_detector():
    if Detector is None:
        print("AprilTag disabled: pupil_apriltags is not installed")
        return None

    return Detector(
        families=APRILTAG_FAMILY,
        nthreads=NTHREADS,
        quad_decimate=QUAD_DECIMATE,
        quad_sigma=QUAD_SIGMA,
        refine_edges=REFINE_EDGES,
        decode_sharpening=DECODE_SHARPENING,
        debug=DEBUG,
    )


def rotation_matrix_to_euler_zyx(rotation_matrix: Any) -> Tuple[float, float, float]:
    sy = math.sqrt(
        rotation_matrix[0][0] * rotation_matrix[0][0]
        + rotation_matrix[1][0] * rotation_matrix[1][0]
    )
    singular = sy < 1e-6

    if not singular:
        yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
        pitch = math.atan2(-rotation_matrix[2][0], sy)
        roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
    else:
        yaw = math.atan2(-rotation_matrix[0][1], rotation_matrix[1][1])
        pitch = math.atan2(-rotation_matrix[2][0], sy)
        roll = 0.0

    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def annotate_apriltags(frame, detector, camera_params, camera_name: str, last_print_time: float):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    results = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=TAG_SIZE_METERS,
    )

    now = time.time()
    printed_this_frame = False

    for result in results:
        corners = result.corners.astype(int)
        center = result.center.astype(int)

        for index in range(4):
            p1 = tuple(corners[index])
            p2 = tuple(corners[(index + 1) % 4])
            cv2.line(frame, p1, p2, (0, 255, 0), 2)

        cv2.circle(frame, tuple(center), 5, (255, 0, 0), -1)

        tag_id = result.tag_id
        z_m = float(result.pose_t[2][0])
        yaw_deg, pitch_deg, roll_deg = rotation_matrix_to_euler_zyx(result.pose_R)

        cv2.putText(
            frame,
            f"ID {tag_id}",
            (center[0] + 10, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Z {z_m:.2f} m",
            (center[0] + 10, center[1] + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Yaw {yaw_deg:.1f}",
            (center[0] + 10, center[1] + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        if now - last_print_time > 0.2:
            print(
                f"[{camera_name}] Tag {tag_id} | "
                f"X={float(result.pose_t[0][0]):.3f} m  "
                f"Y={float(result.pose_t[1][0]):.3f} m  "
                f"Z={z_m:.3f} m  "
                f"Yaw={yaw_deg:.1f}  Pitch={pitch_deg:.1f}  Roll={roll_deg:.1f}"
            )
            printed_this_frame = True

    if printed_this_frame:
        last_print_time = now

    return frame, last_print_time


def render_root_page() -> bytes:
    cards = []
    for worker in CAMERA_WORKERS.values():
        cards.append(
            f"""
            <section style="background:#1a1f23;padding:12px;border-radius:10px;">
                <h2 style="margin:0 0 10px 0;">{worker.config.label}</h2>
                <img src="/{worker.config.name}/stream.mjpg" width="480" height="360" style="width:100%;height:auto;border-radius:8px;" />
            </section>
            """
        )

    html = f"""\
<html>
<head>
    <title>KSU Storm Multi-Camera</title>
</head>
<body style="background:#111;color:#eee;font-family:sans-serif;">
    <h1>KSU Storm Multi-Camera Stream</h1>
    <p>Driver cam (CSI connector 0): picamera2. USB cameras: front left + front right.</p>
    <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;">
        {''.join(cards)}
    </div>
</body>
</html>
"""
    return html.encode("utf-8")


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            content = render_root_page()
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
            return

        parts = [part for part in self.path.strip("/").split("/") if part]
        if len(parts) == 2 and parts[1] == "stream.mjpg":
            worker = CAMERA_WORKERS.get(parts[0])
            if worker is None:
                self.send_error(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()

            try:
                while True:
                    with worker.output.condition:
                        worker.output.condition.wait()
                        frame = worker.output.frame
                    if frame is None:
                        continue

                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except BrokenPipeError:
                pass
            except ConnectionResetError:
                pass
            except Exception as exc:
                print(f"Streaming client for {worker.config.name} disconnected: {exc}")
            return

        self.send_error(404)
        self.end_headers()

    def log_message(self, format_str, *args):
        return


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    for worker in CAMERA_WORKERS.values():
        worker.start()

    address = ("", PORT)
    httpd = StreamingServer(address, StreamingHandler)

    print(f"Server running on port {PORT}")
    print("Open this in a browser on your computer:")
    print(f"  http://<pi-ip-address>:{PORT}")
    for worker in CAMERA_WORKERS.values():
        print(f"  http://<pi-ip-address>:{PORT}/{worker.config.name}/stream.mjpg")

    httpd.serve_forever()


if __name__ == "__main__":
    main()
