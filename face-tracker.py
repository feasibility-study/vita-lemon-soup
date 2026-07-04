"""
face_tracker_ui.py
Detects ALL faces visible to the webcam using OpenCV's YuNet DNN face
detector, draws a bounding box around each, and prints a list of detected
faces with their coordinates to the console.

Requires OpenCV 4.5.4+ (works fine on OpenCV 5.x, unlike the old Haar
cascade approach which was moved to opencv-contrib in OpenCV 5.0).

On first run this downloads a small (~340KB) face detection model file
into the same folder as this script.

USAGE:
    python face_tracker_ui.py

Press 'q' to quit.
"""

import os
import time
import urllib.request
import cv2
import numpy as np

CAM_INDEX = 0
PRINT_INTERVAL_SEC = 0.5   # how often to print the coordinate list to console
SCORE_THRESHOLD = 0.7      # confidence cutoff for a detection to count as a face

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_detection_yunet.onnx")
MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print(f"Face detection model not found, downloading to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Download complete.")


def main():
    ensure_model()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Could not read from webcam")
    h, w = frame.shape[:2]

    detector = cv2.FaceDetectorYN.create(
        MODEL_PATH, "", (w, h),
        score_threshold=SCORE_THRESHOLD,
    )

    last_print = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        detector.setInputSize((w, h))

        _, faces = detector.detect(frame)

        objects = []
        if faces is not None:
            for face in faces:
                x, y, fw, fh = face[0:4].astype(int)
                score = float(face[-1])
                cx, cy = x + fw // 2, y + fh // 2
                objects.append({"x": cx, "y": cy, "w": int(fw), "h": int(fh), "score": round(score, 2)})

                cv2.rectangle(frame, (x, y), (x + fw, y + fh), (255, 0, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, f"({cx},{cy})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.putText(frame, f"Faces detected: {len(objects)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        now = time.time()
        if now - last_print >= PRINT_INTERVAL_SEC:
            if objects:
                print(f"[{time.strftime('%H:%M:%S')}] Detected {len(objects)} face(s):")
                for i, obj in enumerate(objects):
                    print(f"  #{i}: center=({obj['x']}, {obj['y']}) size={obj['w']}x{obj['h']} score={obj['score']}")
            else:
                print(f"[{time.strftime('%H:%M:%S')}] No faces detected")
            last_print = now

        cv2.imshow("Face Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()