#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import time

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


def parse_source(raw: str):
    if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
        return int(raw)
    return raw


def create_capture(backend: str, source: str, width: int, height: int):
    backend = backend.lower().strip()
    if backend == "auto":
        backend = "picamera2" if Picamera2 is not None else "opencv"

    if backend == "picamera2":
        if Picamera2 is None:
            raise RuntimeError("Picamera2 is not installed. Use --backend opencv or install picamera2.")

        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (width, height), "format": "RGB888"})
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)

        def read_frame_bgr():
            frame_rgb = picam2.capture_array()
            return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        def close():
            try:
                picam2.stop()
            except Exception:
                pass

        return "picamera2", read_frame_bgr, close

    cap = cv2.VideoCapture(parse_source(source))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open OpenCV source: {source}")

    def read_frame_bgr():
        ok, frame = cap.read()
        if not ok:
            return None
        return frame

    def close():
        cap.release()

    return "opencv", read_frame_bgr, close


def compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist) -> float:
    total_error = 0.0
    total_points = 0

    for obj, img, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, mtx, dist)
        error = cv2.norm(img, projected, cv2.NORM_L2)
        total_error += error * error
        total_points += len(obj)

    if total_points == 0:
        return 0.0
    return float(np.sqrt(total_error / total_points))


def main():
    parser = argparse.ArgumentParser(description="Camera calibration using a checkerboard")
    parser.add_argument("--backend", default="auto", choices=["auto", "opencv", "picamera2"], help="Camera backend")
    parser.add_argument("--source", default="0", help="OpenCV source index/URL/path when backend=opencv")
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=480, help="Capture height")
    parser.add_argument("--cols", type=int, default=9, help="Checkerboard inner corners along width")
    parser.add_argument("--rows", type=int, default=6, help="Checkerboard inner corners along height")
    parser.add_argument("--square-size", type=float, default=0.0245, help="Checker square size in meters")
    parser.add_argument("--samples", type=int, default=25, help="Number of successful captures needed")
    parser.add_argument("--output", default="camera_calibration.json", help="Output JSON file")
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= float(args.square_size)

    objpoints = []
    imgpoints = []

    backend_used, read_frame_bgr, close_capture = create_capture(
        args.backend,
        args.source,
        args.width,
        args.height,
    )

    print("Calibration capture started")
    print(f"Backend: {backend_used}")
    print(f"Resolution: {args.width}x{args.height}")
    print(f"Checkerboard: cols={args.cols}, rows={args.rows}, square_size={args.square_size}m")
    print(f"Target captures: {args.samples}")
    print("Controls: Space=save frame (when corners found), q=quit")

    image_size = None

    try:
        while len(objpoints) < args.samples:
            frame = read_frame_bgr()
            if frame is None:
                cv2.waitKey(10)
                continue

            image_size = (frame.shape[1], frame.shape[0])
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(
                gray,
                pattern_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            preview = frame.copy()
            message = f"Samples: {len(objpoints)}/{args.samples}"

            if found:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(preview, pattern_size, corners2, found)
                message += " | checkerboard detected"
            else:
                corners2 = None
                message += " | move/tilt checkerboard"

            cv2.putText(preview, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Calibration", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") and found and corners2 is not None:
                objpoints.append(objp.copy())
                imgpoints.append(corners2)
                print(f"Captured sample {len(objpoints)}/{args.samples}")

        if len(objpoints) < 6:
            raise RuntimeError(f"Not enough valid samples ({len(objpoints)}). Need at least 6.")

        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints,
            imgpoints,
            image_size,
            None,
            None,
        )

        mean_error = compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs)

        result = {
            "timestamp": time.time(),
            "backend": backend_used,
            "image_size": [int(image_size[0]), int(image_size[1])],
            "checkerboard": {
                "cols": int(args.cols),
                "rows": int(args.rows),
                "square_size_m": float(args.square_size),
            },
            "samples": int(len(objpoints)),
            "rms_reprojection_error": float(rms),
            "mean_reprojection_error": float(mean_error),
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.tolist(),
        }

        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)

        print("Calibration complete")
        print(f"Saved: {os.path.abspath(args.output)}")
        print(f"RMS reprojection error: {rms:.4f}")
        print(f"Mean reprojection error: {mean_error:.4f}")
        print("Update camera.py calibration path if needed via KSU_CAMERA_CALIBRATION_FILE")

    finally:
        close_capture()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
