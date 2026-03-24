#!/usr/bin/env python3
import math
import os
import socketserver
import threading
import time
from http import server

import cv2

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


# =========================
# User settings
# =========================

# Physical tag size: 2 inches = 0.0508 meters
TAG_SIZE_METERS = 0.0508

# Camera resolution for detection/streaming
WIDTH = 640
HEIGHT = 480

# Replace these with real calibrated values for your camera at this resolution
FX = 700.0
FY = 700.0
CX = WIDTH / 2.0
CY = HEIGHT / 2.0

# MJPEG server port
PORT = int(os.environ.get("KSU_CAMERA_PORT", "8080"))

# Camera backend selection:
#   auto      -> Picamera2 when available, otherwise OpenCV VideoCapture
#   picamera2 -> force RPi camera path
#   opencv    -> force OpenCV camera path
CAMERA_BACKEND = os.environ.get("KSU_CAMERA_BACKEND", "auto").strip().lower()

# OpenCV camera source (device index or URL), used for laptop/dev and as fallback.
# Examples:
#   KSU_OPENCV_CAMERA_SOURCE=0
#   KSU_OPENCV_CAMERA_SOURCE=1
#   KSU_OPENCV_CAMERA_SOURCE=http://192.168.1.10:8080/stream.mjpg
OPENCV_CAMERA_SOURCE = os.environ.get("KSU_OPENCV_CAMERA_SOURCE", "0").strip()

# Enable/disable AprilTag processing at runtime
ENABLE_APRILTAG = os.environ.get("KSU_ENABLE_APRILTAG", "1").strip().lower() not in ("0", "false", "no")

# Detector settings
APRILTAG_FAMILY = "tag36h11"
NTHREADS = 4
QUAD_DECIMATE = 2.0
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
DEBUG = 0


# =========================
# Shared frame buffer
# =========================

class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, frame_bytes):
        with self.condition:
            self.frame = frame_bytes
            self.condition.notify_all()


output = StreamingOutput()


# =========================
# Camera helpers
# =========================

def parse_opencv_source(raw):
    # Numeric value means device index, otherwise treat as URL/path.
    if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
        return int(raw)
    return raw


def resolve_camera_backend():
    if CAMERA_BACKEND == "picamera2":
        return "picamera2"
    if CAMERA_BACKEND == "opencv":
        return "opencv"
    # auto
    return "picamera2" if Picamera2 is not None else "opencv"


def create_frame_source():
    backend = resolve_camera_backend()

    if backend == "picamera2":
        if Picamera2 is None:
            raise RuntimeError("Picamera2 is not installed but KSU_CAMERA_BACKEND=picamera2")

        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)

        def capture_rgb_frame():
            return picam2.capture_array()  # Already RGB

        def close_source():
            try:
                picam2.stop()
            except Exception:
                pass

        return backend, capture_rgb_frame, close_source

    source = parse_opencv_source(OPENCV_CAMERA_SOURCE)
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open OpenCV camera source: {source}")

    def capture_rgb_frame():
        ok, frame_bgr = cap.read()
        if not ok:
            return None
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    def close_source():
        cap.release()

    return backend, capture_rgb_frame, close_source


# =========================
# AprilTag / camera thread
# =========================

def rotation_matrix_to_euler_zyx(R):
    """
    Returns yaw, pitch, roll in degrees using ZYX convention.
    yaw   = rotation about Z
    pitch = rotation about Y
    roll  = rotation about X
    """
    sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
    singular = sy < 1e-6

    if not singular:
        yaw = math.atan2(R[1][0], R[0][0])
        pitch = math.atan2(-R[2][0], sy)
        roll = math.atan2(R[2][1], R[2][2])
    else:
        yaw = math.atan2(-R[0][1], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        roll = 0.0

    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def camera_worker():
    detector = None
    if ENABLE_APRILTAG:
        if Detector is None:
            print("AprilTag disabled: pupil_apriltags is not installed")
        else:
            detector = Detector(
                families=APRILTAG_FAMILY,
                nthreads=NTHREADS,
                quad_decimate=QUAD_DECIMATE,
                quad_sigma=QUAD_SIGMA,
                refine_edges=REFINE_EDGES,
                decode_sharpening=DECODE_SHARPENING,
                debug=DEBUG,
            )

    backend, capture_rgb_frame, close_source = create_frame_source()
    print(f"Camera backend: {backend}")
    if backend == "opencv":
        print(f"OpenCV source: {OPENCV_CAMERA_SOURCE}")

    last_print_time = 0.0

    try:
        while True:
            frame = capture_rgb_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            if detector is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                results = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=[FX, FY, CX, CY],
                    tag_size=TAG_SIZE_METERS,
                )

                now = time.time()
                printed_this_frame = False

                for r in results:
                    corners = r.corners.astype(int)
                    center = r.center.astype(int)

                    for i in range(4):
                        p1 = tuple(corners[i])
                        p2 = tuple(corners[(i + 1) % 4])
                        cv2.line(frame, p1, p2, (0, 255, 0), 2)

                    cv2.circle(frame, tuple(center), 5, (255, 0, 0), -1)

                    tag_id = r.tag_id
                    z_m = float(r.pose_t[2][0])
                    yaw_deg, pitch_deg, roll_deg = rotation_matrix_to_euler_zyx(r.pose_R)

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
                            f"Tag {tag_id} | "
                            f"X={float(r.pose_t[0][0]):.3f} m  "
                            f"Y={float(r.pose_t[1][0]):.3f} m  "
                            f"Z={z_m:.3f} m  "
                            f"Yaw={yaw_deg:.1f}  Pitch={pitch_deg:.1f}  Roll={roll_deg:.1f}"
                        )
                        printed_this_frame = True

                if printed_this_frame:
                    last_print_time = now

            # Convert RGB -> BGR for OpenCV JPEG encoding
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                output.write(jpeg.tobytes())
    finally:
        close_source()


# =========================
# MJPEG web server
# =========================

PAGE = """\
<html>
<head>
    <title>Pi AprilTag Stream</title>
</head>
<body style="background:#111;color:#eee;font-family:sans-serif;">
    <h1>Pi AprilTag Stream</h1>
    <p>MJPEG stream with AprilTag overlays</p>
    <img src="/stream.mjpg" width="640" height="480" />
</body>
</html>
"""


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)

        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()

            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame

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
            except Exception as e:
                print("Streaming client disconnected:", e)

        else:
            self.send_error(404)
            self.end_headers()

    def log_message(self, format_str, *args):
        return


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


# =========================
# Main
# =========================

def main():
    t = threading.Thread(target=camera_worker, daemon=True)
    t.start()

    address = ("", PORT)
    httpd = StreamingServer(address, StreamingHandler)

    print(f"Server running on port {PORT}")
    print("Open this in a browser on your computer:")
    print(f"  http://<pi-ip-address>:{PORT}")
    print("Example:")
    print(f"  http://192.168.1.50:{PORT}")

    httpd.serve_forever()


if __name__ == "__main__":
    main()
