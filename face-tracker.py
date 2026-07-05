"""
face_tracker_ui.py
Detects faces with OpenCV's YuNet detector, locks onto one active face,
and sends a simple serial command for an RP2040 servo controller.

The PC does the face detection. The RP2040 running MicroPython only reads
serial commands and moves a servo.

USAGE:
    python face-tracker.py --port auto
    python face-tracker.py --port COM5

Press 'q' to quit.
"""

import argparse
import json
import math
import os
import time
import urllib.request

import cv2
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


CAM_INDEX = 0
PRINT_INTERVAL_SEC = 0.5
SERIAL_BAUDRATE = 115200
SCORE_THRESHOLD = 0.7
ACTIVE_FACE_LOST_FRAMES = 8
MATCH_DISTANCE_MULTIPLIER = 1.8
SERIAL_DEADZONE = 0.04
DEFAULT_FACE_WIDTH_MM = 160.0
DISTANCE_CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "distance_calibration.json")

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_detection_yunet.onnx")
MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print(f"Face detection model not found, downloading to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Download complete.")


def open_serial_port(port_name, baudrate):
    if port_name in (None, "", "none", "off"):
        return None

    if serial is None or list_ports is None:
        print("pyserial is not installed, so serial output is disabled. Install it with: pip install pyserial")
        return None

    if port_name == "auto":
        ports = [port.device for port in list_ports.comports()]
        if len(ports) == 1:
            port_name = ports[0]
            print(f"Auto-selected serial port: {port_name}")
        elif len(ports) == 0:
            print("No serial ports found. Run with --port COMx once the RP2040 is connected.")
            return None
        else:
            print("Multiple serial ports found:")
            for port in ports:
                print(f"  {port}")
            print("Re-run with --port COMx to choose the RP2040 board.")
            return None

    try:
        connection = serial.Serial(port_name, baudrate=baudrate, timeout=0)
        time.sleep(1.5)
        return connection
    except serial.SerialException as exc:
        print(f"Could not open serial port {port_name}: {exc}")
        return None


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


def send_face_command(serial_port, face_present, offset):
    if serial_port is None:
        return
    message = f"FACE {1 if face_present else 0} {offset:.3f}\n"
    serial_port.write(message.encode("ascii"))


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


def draw_status_panel(frame, active_face, offset, distance_mm, calibration_hint):
    panel = frame.copy()
    panel_height = 150
    cv2.rectangle(panel, (0, 0), (420, panel_height), (25, 25, 25), -1)

    if active_face is None:
        active_text = "Active face: none"
        offset_text = "Servo offset: n/a"
        distance_text = "Distance: n/a"
    else:
        active_text = f"Active face: ({active_face['x']}, {active_face['y']})"
        offset_text = f"Servo offset: {offset:+.3f}"
        distance_text = f"Distance: {distance_mm / 10.0:.1f} cm" if distance_mm is not None else "Distance: n/a"

    cv2.putText(panel, active_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(panel, offset_text, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 220, 120), 2)
    cv2.putText(panel, distance_text, (12, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 180, 255), 2)
    if calibration_hint:
        cv2.putText(panel, calibration_hint, (12, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return panel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="auto",
        help="Serial port for the RP2040, for example COM5. Use auto to select the only connected board.",
    )
    parser.add_argument("--baud", type=int, default=SERIAL_BAUDRATE, help="Serial baud rate")
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
        help="Enable distance calibration mode. Stand at this known distance and press 'c' to save calibration.",
    )
    args = parser.parse_args()

    ensure_model()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Could not read from webcam")

    detector = build_detector(frame)
    serial_port = open_serial_port(args.port, args.baud)
    if serial_port is not None:
        print("Serial link ready. Sending FACE <present> <offset> commands.")

    focal_length_px = args.distance_focal_length_px
    if focal_length_px is None:
        focal_length_px = load_distance_calibration(args.distance_calibration_file)

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
            current_state = (True, round(offset, 3))
            draw_face(frame, active_face, (0, 255, 0))

            distance_mm = estimate_distance_mm(active_face, focal_length_px, args.face_width_mm)
        else:
            offset = 0.0
            current_state = (False, 0.0)
            distance_mm = None

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

        if args.calibrate_distance_mm is not None:
            calibration_hint = f"Press 'c' @ {args.calibrate_distance_mm:.0f} mm"
        else:
            calibration_hint = None

        status_panel = draw_status_panel(frame, active_face, offset, distance_mm, calibration_hint)
        frame = status_panel

        now = time.time()
        if now - last_print >= PRINT_INTERVAL_SEC:
            if active_face is not None:
                print(
                    f"[{time.strftime('%H:%M:%S')}] locked face center=({active_face['x']}, {active_face['y']}) "
                    f"size={active_face['w']}x{active_face['h']} offset={offset:+.3f}"
                )
            else:
                print(f"[{time.strftime('%H:%M:%S')}] No face locked")
            last_print = now

        if current_state != last_sent_state:
            send_face_command(serial_port, current_state[0], current_state[1])
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
    if serial_port is not None:
        serial_port.close()


if __name__ == "__main__":
    main()