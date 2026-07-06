"""
Face tracker for an ESP32-S3 Sense IP camera setup.

The ESP32 hosts the camera stream and a small HTTP motor endpoint.
This laptop script reads the IP camera stream, detects faces, estimates
face distance, shows the GUI overlays, and sends drive commands back to
the ESP.

Typical usage:
    python face-tracker.py --camera-url http://192.168.4.1:8080/stream --move-url http://192.168.4.1:8080/move
"""

import argparse
import json
import math
import os
import time
import urllib.request
from urllib.parse import urlencode

import cv2
import numpy as np


CAM_INDEX = 0
PRINT_INTERVAL_SEC = 0.5
SCORE_THRESHOLD = 0.7
ACTIVE_FACE_LOST_FRAMES = 8
MATCH_DISTANCE_MULTIPLIER = 1.8
SERIAL_DEADZONE = 0.04
DEFAULT_FACE_WIDTH_MM = 160.0
DEFAULT_TARGET_DISTANCE_CM = 60.0
DEFAULT_FORWARD_GAIN = 0.8
DEFAULT_TURN_GAIN = 0.9
DEFAULT_MAX_SPEED = 0.8

DISTANCE_CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "distance_calibration.json",
)
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_detection_yunet.onnx")
MODEL_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/"
    "face_detection_yunet_2023mar.onnx"
)


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print(f"Face detection model not found, downloading to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Download complete.")


def open_video_source(source):
    if isinstance(source, str) and hasattr(cv2, "CAP_FFMPEG"):
        capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
        if capture.isOpened():
            return capture

    return cv2.VideoCapture(source)


def load_distance_calibration(calibration_path):
    if not calibration_path or not os.path.exists(calibration_path):
        return None

    try:
        with open(calibration_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        focal_length_px = float(payload["focal_length_px"])
        if focal_length_px <= 0:
            return None
        print(f"Loaded distance calibration from {calibration_path}: focal_length_px={focal_length_px:.2f}")
        return focal_length_px
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"Could not load distance calibration from {calibration_path}: {exc}")
        return None


def save_distance_calibration(calibration_path, focal_length_px):
    payload = {
        "focal_length_px": float(focal_length_px),
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(calibration_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    print(f"Saved distance calibration to {calibration_path}: {payload}")


def estimate_distance_mm(face, focal_length_px, face_width_mm):
    if focal_length_px is None:
        return None
    if face["w"] <= 0 or face_width_mm <= 0:
        return None
    return (face_width_mm * focal_length_px) / face["w"]


def compute_focal_length_px(face_width_px, known_distance_mm, face_width_mm):
    if face_width_px <= 0 or known_distance_mm <= 0 or face_width_mm <= 0:
        return None
    return (face_width_px * known_distance_mm) / face_width_mm


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def compute_drive_command(face_present, offset, distance_mm, target_distance_cm, forward_gain, turn_gain, max_speed):
    if not face_present:
        return 0.0, 0.0

    distance_cm = None if distance_mm is None else distance_mm / 10.0
    forward = 0.0
    if distance_cm is not None and target_distance_cm > 0:
        distance_error = (distance_cm - target_distance_cm) / target_distance_cm
        forward = clamp(distance_error * forward_gain, -1.0, 1.0) * max_speed

    turn = clamp(offset * turn_gain, -1.0, 1.0) * max_speed
    left = clamp(forward - turn, -max_speed, max_speed)
    right = clamp(forward + turn, -max_speed, max_speed)
    return left, right


def send_move_command(move_url, payload):
    if not move_url:
        return

    query = urlencode({key: str(value) for key, value in payload.items() if value is not None})
    separator = "&" if "?" in move_url else "?"
    request = urllib.request.Request(f"{move_url}{separator}{query}", method="GET")

    try:
        with urllib.request.urlopen(request, timeout=0.5) as response:
            response.read()
    except OSError as exc:
        print(f"Could not send motion command to {move_url}: {exc}")


def build_detector(frame):
    height, width = frame.shape[:2]
    return cv2.FaceDetectorYN.create(
        MODEL_PATH,
        "",
        (width, height),
        score_threshold=SCORE_THRESHOLD,
    )


def extract_detections(faces):
    detections = []
    if faces is None:
        return detections

    for face in faces:
        x, y, face_width, face_height = face[0:4].astype(int)
        score = float(face[-1])
        center_x = x + face_width // 2
        center_y = y + face_height // 2
        detections.append(
            {
                "x": center_x,
                "y": center_y,
                "w": int(face_width),
                "h": int(face_height),
                "score": round(score, 2),
                "rect": (x, y, face_width, face_height),
            }
        )

    return detections


def match_active_face(active_face, detections):
    if active_face is None or not detections:
        return None, detections

    active_x = active_face["x"]
    active_y = active_face["y"]
    active_size = max(active_face["w"], active_face["h"])

    best_index = None
    best_distance = None

    for index, detection in enumerate(detections):
        distance = math.hypot(detection["x"] - active_x, detection["y"] - active_y)
        distance_limit = max(60.0, MATCH_DISTANCE_MULTIPLIER * max(active_size, detection["w"], detection["h"]))
        if distance <= distance_limit and (best_distance is None or distance < best_distance):
            best_index = index
            best_distance = distance

    if best_index is None:
        return None, detections

    matched_face = detections[best_index]
    remaining = [detection for index, detection in enumerate(detections) if index != best_index]
    return matched_face, remaining


def pick_initial_face(detections):
    if not detections:
        return None
    return detections[0]


def face_offset(face, frame_width):
    offset = (face["x"] - frame_width / 2) / (frame_width / 2)
    if abs(offset) < SERIAL_DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, offset))


def draw_face(frame, face, color):
    x, y, face_width, face_height = face["rect"]
    cv2.rectangle(frame, (x, y), (x + face_width, y + face_height), color, 2)
    cv2.circle(frame, (face["x"], face["y"]), 4, (0, 0, 255), -1)
    cv2.putText(
        frame,
        f"({face['x']},{face['y']})",
        (x, y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
    )


def draw_status_panel(frame, active_face, offset, distance_mm, calibration_hint, drive_left, drive_right, command_mode):
    panel = frame.copy()
    panel_height = 200
    cv2.rectangle(panel, (0, 0), (460, panel_height), (25, 25, 25), -1)

    if active_face is None:
        active_text = "Active face: none"
        offset_text = "Servo offset: n/a"
        distance_text = "Distance: n/a"
        drive_text = "Drive: stop"
    else:
        active_text = f"Active face: ({active_face['x']}, {active_face['y']})"
        offset_text = f"Servo offset: {offset:+.3f}"
        distance_text = f"Distance: {distance_mm / 10.0:.1f} cm" if distance_mm is not None else "Distance: n/a"
        drive_text = f"Drive: L {drive_left:+.2f}  R {drive_right:+.2f}"

    cv2.putText(panel, active_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(panel, offset_text, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 220, 120), 2)
    cv2.putText(panel, distance_text, (12, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 180, 255), 2)
    cv2.putText(panel, drive_text, (12, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 170, 120), 2)
    cv2.putText(panel, f"Command mode: {command_mode}", (12, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    if calibration_hint:
        cv2.putText(panel, calibration_hint, (12, 168), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return panel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera-url",
        default=None,
        help="IP camera MJPEG stream URL, for example http://192.168.4.1:8080/stream.",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=CAM_INDEX,
        help="Local webcam index used if --camera-url is not set.",
    )
    parser.add_argument(
        "--move-url",
        default=None,
        help="Motion endpoint on the ESP32, for example http://192.168.4.1:8080/move.",
    )
    parser.add_argument(
        "--face-width-mm",
        type=float,
        default=DEFAULT_FACE_WIDTH_MM,
        help="Approximate real-world face width used for distance estimation in millimeters.",
    )
    parser.add_argument(
        "--distance-focal-length-px",
        type=float,
        default=None,
        help="Manual focal length in pixels for distance estimation. Overrides saved calibration.",
    )
    parser.add_argument(
        "--distance-calibration-file",
        default=DISTANCE_CALIBRATION_FILE,
        help="Path to save/load distance calibration JSON.",
    )
    parser.add_argument(
        "--calibrate-distance-mm",
        type=float,
        default=None,
        help="Enable distance calibration mode. Stand at this known distance and press 'c' to save the calibration.",
    )
    parser.add_argument(
        "--target-distance-cm",
        type=float,
        default=DEFAULT_TARGET_DISTANCE_CM,
        help="Desired face distance used to translate distance estimates into forward/backward motion.",
    )
    parser.add_argument(
        "--forward-gain",
        type=float,
        default=DEFAULT_FORWARD_GAIN,
        help="How strongly the laptop commands forward/backward motion based on distance error.",
    )
    parser.add_argument(
        "--turn-gain",
        type=float,
        default=DEFAULT_TURN_GAIN,
        help="How strongly the laptop commands left/right turning based on face offset.",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=DEFAULT_MAX_SPEED,
        help="Maximum absolute motor command sent to the ESP32, between 0 and 1.",
    )
    args = parser.parse_args()

    ensure_model()

    capture_source = args.camera_url if args.camera_url else args.camera_index
    cap = open_video_source(capture_source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {capture_source}")

    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Could not read from the camera source")

    detector = build_detector(frame)

    focal_length_px = args.distance_focal_length_px
    if focal_length_px is None:
        focal_length_px = load_distance_calibration(args.distance_calibration_file)

    if args.move_url:
        print(f"HTTP motion endpoint ready: {args.move_url}")
    else:
        print("No --move-url set, so the script will only show overlays and print commands.")

    if args.calibrate_distance_mm is not None:
        print(
            "Distance calibration mode enabled. Stand at the known distance, lock onto your face, "
            "then press 'c' to save the calibration."
        )
    elif focal_length_px is not None:
        print(f"Distance overlay enabled using focal_length_px={focal_length_px:.2f}")
    else:
        print("Distance overlay disabled until you calibrate or provide --distance-focal-length-px.")

    last_print = 0.0
    active_face = None
    lost_frames = 0
    last_sent_state = None
    distance_mm = None
    calibration_hint = None
    drive_left = 0.0
    drive_right = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)
        frame_height, frame_width = frame.shape[:2]
        detector.setInputSize((frame_width, frame_height))

        _, faces = detector.detect(frame)
        detections = extract_detections(faces)
        visible_detections = detections

        if active_face is None:
            active_face = pick_initial_face(visible_detections)
            if active_face is not None:
                visible_detections = visible_detections[1:]
                lost_frames = 0
            else:
                lost_frames += 1
        else:
            matched_face, remaining = match_active_face(active_face, visible_detections)
            if matched_face is not None:
                active_face = matched_face
                visible_detections = remaining
                lost_frames = 0
            else:
                lost_frames += 1
                if lost_frames >= ACTIVE_FACE_LOST_FRAMES:
                    active_face = pick_initial_face(visible_detections)
                    if active_face is not None:
                        visible_detections = visible_detections[1:]
                    lost_frames = 0

        if active_face is not None:
            offset = face_offset(active_face, frame_width)
            distance_mm = estimate_distance_mm(active_face, focal_length_px, args.face_width_mm)
            draw_face(frame, active_face, (0, 255, 0))
        else:
            offset = 0.0
            distance_mm = None

        drive_left, drive_right = compute_drive_command(
            active_face is not None,
            offset,
            distance_mm,
            args.target_distance_cm,
            args.forward_gain,
            args.turn_gain,
            max(0.0, min(1.0, args.max_speed)),
        )

        for detection in visible_detections:
            draw_face(frame, detection, (255, 0, 0))

        detected_count = (1 if active_face is not None else 0) + len(visible_detections)
        cv2.putText(
            frame,
            f"Faces detected: {detected_count}",
            (10, frame_height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

        calibration_hint = f"Press 'c' @ {args.calibrate_distance_mm:.0f} mm" if args.calibrate_distance_mm is not None else None

        frame = draw_status_panel(
            frame,
            active_face,
            offset,
            distance_mm,
            calibration_hint,
            drive_left,
            drive_right,
            "HTTP" if args.move_url else "local",
        )

        now = time.time()
        if now - last_print >= PRINT_INTERVAL_SEC:
            if active_face is not None:
                print(
                    f"[{time.strftime('%H:%M:%S')}] face center=({active_face['x']}, {active_face['y']}) "
                    f"size={active_face['w']}x{active_face['h']} offset={offset:+.3f} "
                    f"drive=({drive_left:+.2f},{drive_right:+.2f})"
                )
            else:
                print(f"[{time.strftime('%H:%M:%S')}] No face locked")
            last_print = now

        current_state = (round(drive_left, 3), round(drive_right, 3), active_face is not None)
        if args.move_url and current_state != last_sent_state:
            payload = {
                "left": round(drive_left, 3),
                "right": round(drive_right, 3),
                "present": 1 if active_face is not None else 0,
                "offset": round(offset, 3),
                "distance_cm": None if distance_mm is None else round(distance_mm / 10.0, 2),
            }
            send_move_command(args.move_url, payload)
            last_sent_state = current_state

        cv2.imshow("Face Tracker", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("c") and args.calibrate_distance_mm is not None and active_face is not None:
            calibration = compute_focal_length_px(active_face["w"], args.calibrate_distance_mm, args.face_width_mm)
            if calibration is not None:
                focal_length_px = calibration
                save_distance_calibration(args.distance_calibration_file, focal_length_px)
                calibration_hint = "Calibration saved"
            else:
                print("Could not calibrate distance from the current face.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
