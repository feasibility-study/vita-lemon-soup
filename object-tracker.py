"""
object_tracker_ui.py
Tracks a single colored object via the webcam and draws a bounding box
around it. No serial/ESP32 required - this is just for testing detection.

USAGE:
    python object_tracker_ui.py               # normal run
    python object_tracker_ui.py --calibrate   # opens HSV calibration trackbars

Press 'q' to quit.
"""

import argparse
import cv2
import numpy as np

# ---------------- SETTINGS ----------------
CAM_INDEX = 0

# Run with --calibrate to find the right values for object/lighting.
HSV_LOWER = np.array([121, 0, 125])
HSV_UPPER = np.array([179, 255, 199])

MIN_CONTOUR_AREA = 500
# -------------------------------------------------


def nothing(_):
    pass


def make_calibration_window():
    cv2.namedWindow("Calibration")
    cv2.createTrackbar("H min", "Calibration", int(HSV_LOWER[0]), 179, nothing)
    cv2.createTrackbar("H max", "Calibration", int(HSV_UPPER[0]), 179, nothing)
    cv2.createTrackbar("S min", "Calibration", int(HSV_LOWER[1]), 255, nothing)
    cv2.createTrackbar("S max", "Calibration", int(HSV_UPPER[1]), 255, nothing)
    cv2.createTrackbar("V min", "Calibration", int(HSV_LOWER[2]), 255, nothing)
    cv2.createTrackbar("V max", "Calibration", int(HSV_UPPER[2]), 255, nothing)


def read_calibration():
    lower = np.array([
        cv2.getTrackbarPos("H min", "Calibration"),
        cv2.getTrackbarPos("S min", "Calibration"),
        cv2.getTrackbarPos("V min", "Calibration"),
    ])
    upper = np.array([
        cv2.getTrackbarPos("H max", "Calibration"),
        cv2.getTrackbarPos("S max", "Calibration"),
        cv2.getTrackbarPos("V max", "Calibration"),
    ])
    return lower, upper


def detect_object(frame, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_CONTOUR_AREA:
        return None, mask

    x, y, w, h = cv2.boundingRect(c)
    return (x, y, w, h), mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true", help="show HSV calibration trackbars")
    args = parser.parse_args()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    if args.calibrate:
        make_calibration_window()

    hsv_lower, hsv_upper = HSV_LOWER, HSV_UPPER

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)

        if args.calibrate:
            hsv_lower, hsv_upper = read_calibration()

        result, mask = detect_object(frame, hsv_lower, hsv_upper)

        if result:
            x, y, w, h = result
            cx, cy = x + w // 2, y + h // 2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"({cx},{cy})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "No object detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Object Tracker", frame)
        if args.calibrate:
            cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()