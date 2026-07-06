import cv2
import os
import sys
import numpy as np
import argparse
import pickle
import time
from pathlib import Path

from deepface import DeepFace
from deepface.commons.logger import Logger

Logger.disable = True

BASE_DIR = Path(__file__).parent
DATASET_DIR = BASE_DIR / "dataset"
DATABASE_DIR = BASE_DIR / "database"
CASCADE_PATH = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
EMBEDDINGS_PATH = DATABASE_DIR / "embeddings.pkl"

CAMERA_URL = "http://192.168.4.1:81/stream"
SAMPLE_COUNT = 30
MODEL_NAME = "Facenet"
DETECTOR_BACKEND = "opencv"
DISTANCE_METRIC = "cosine"
DISTANCE_THRESHOLD = 0.4


def get_frame(cap):
    ret, frame = cap.read()
    if not ret:
        return None
    return frame


def detect_faces(gray):
    cascade = cv2.CascadeClassifier(CASCADE_PATH)
    faces = cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(80, 80)
    )
    return faces


def load_database():
    if EMBEDDINGS_PATH.exists():
        with open(EMBEDDINGS_PATH, "rb") as f:
            return pickle.load(f)
    return {}


def save_database(db):
    DATABASE_DIR.mkdir(parents=True, exist_ok=True)
    with open(EMBEDDINGS_PATH, "wb") as f:
        pickle.dump(db, f)


def get_embedding(face_img_rgb):
    try:
        result = DeepFace.represent(
            img_path=face_img_rgb,
            model_name=MODEL_NAME,
            detector_backend="skip",
            enforce_detection=False,
        )
        if result and len(result) > 0:
            return np.array(result[0]["embedding"])
    except Exception as e:
        print(f"  embedding failed: {e}")
    return None


def find_best_match(embedding, database):
    best_name = "Unknown"
    best_dist = float("inf")

    for name, stored_embeddings in database.items():
        for stored in stored_embeddings:
            dist = np.linalg.norm(embedding - stored)
            if dist < best_dist:
                best_dist = dist
                best_name = name

    if best_dist > DISTANCE_THRESHOLD:
        return "Unknown", best_dist
    return best_name, best_dist


def add_face(name, camera_url=CAMERA_URL):
    cap = cv2.VideoCapture(camera_url)
    if not cap.isOpened():
        print(f"Error: Cannot open stream at {camera_url}")
        return

    person_dir = DATASET_DIR / name
    person_dir.mkdir(parents=True, exist_ok=True)

    existing = len(list(person_dir.glob("*.jpg")))
    count = 0

    print(f"Capturing {SAMPLE_COUNT} samples for '{name}'...")
    print("Press 'q' to quit, 's' to save current frame manually.")

    while count < SAMPLE_COUNT:
        frame = get_frame(cap)
        if frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detect_faces(gray)

        for x, y, w, h in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            face_bgr = frame[y : y + h, x : x + w]
            face_resized = cv2.resize(face_bgr, (160, 160))

            fname = person_dir / f"{existing + count + 1:04d}.jpg"
            cv2.imwrite(str(fname), face_resized)
            count += 1
            cv2.putText(
                frame,
                f"Captured {count}/{SAMPLE_COUNT}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            if count >= SAMPLE_COUNT:
                break

        cv2.imshow("Add Face - press 'q' to quit", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Captured {count} samples for '{name}'.")
    print("Run '--train' to compute embeddings and add to database.")


def train():
    people = sorted([d for d in DATASET_DIR.iterdir() if d.is_dir()])
    if not people:
        print("No dataset directories found. Use '--add NAME' first.")
        return

    database = {}
    for person_dir in people:
        name = person_dir.name
        image_files = sorted(person_dir.glob("*.jpg"))
        if not image_files:
            print(f"  No samples for '{name}', skipping.")
            continue

        embeddings = []
        for fpath in image_files:
            img_bgr = cv2.imread(str(fpath))
            if img_bgr is None:
                continue
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            emb = get_embedding(img_rgb)
            if emb is not None:
                embeddings.append(emb)
            print(f"  {name}/{fpath.name}: embedded")

        if embeddings:
            database[name] = embeddings
            print(f"  -> {name}: {len(embeddings)} embedding(s)")
        else:
            print(f"  -> {name}: no valid embeddings, skipping")

    if not database:
        print("No embeddings computed.")
        return

    save_database(database)
    print(f"\nDatabase saved with {len(database)} person(s).")


def list_faces():
    database = load_database()
    if database:
        print("Known faces in database:")
        for name, embeddings in database.items():
            print(f"  {name}: {len(embeddings)} embedding(s)")
    else:
        print("No trained database found.")

    people = sorted([d.name for d in DATASET_DIR.iterdir() if d.is_dir()])
    if people:
        print("\nDataset directories:")
        for p in people:
            count = len(list((DATASET_DIR / p).glob("*.jpg")))
            print(f"  {p}: {count} sample(s)")
    else:
        print("\nNo dataset directories.")


def live(camera_url=CAMERA_URL):
    database = load_database()
    if not database:
        print("No database found. Run '--train' first.")
        return

    print(f"Database loaded: {len(database)} person(s)")
    print(f"Model: {MODEL_NAME}")

    cap = cv2.VideoCapture(camera_url)
    if not cap.isOpened():
        print(f"Error: Cannot open stream at {camera_url}")
        return

    print("Live recognition started. Press 'q' to quit.")

    frame_count = 0
    process_every = 3

    while True:
        frame = get_frame(cap)
        if frame is None:
            continue

        frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detect_faces(gray)

        if frame_count % process_every == 0:
            for x, y, w, h in faces:
                face_bgr = frame[y : y + h, x : x + w]
                face_rgb = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2RGB)
                face_resized = cv2.resize(face_rgb, (160, 160))

                emb = get_embedding(face_resized)
                if emb is not None:
                    name, dist = find_best_match(emb, database)
                else:
                    name, dist = "Error", 0

                color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                label = f"{name} ({dist:.3f})" if name != "Unknown" else "Unknown"
                cv2.putText(
                    frame,
                    label,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

        cv2.imshow("Face Recognition (DeepFace) - press 'q' to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Face recognition (DeepFace) using ESP32-CAM stream"
    )
    parser.add_argument("--add", metavar="NAME", help="Capture face samples for a new person")
    parser.add_argument("--train", action="store_true", help="Compute embeddings from dataset samples")
    parser.add_argument("--live", action="store_true", help="Start live face recognition")
    parser.add_argument("--list", action="store_true", help="List known faces")
    parser.add_argument("--url", default=CAMERA_URL, help=f"Camera stream URL (default: {CAMERA_URL})")
    parser.add_argument("--model", default=MODEL_NAME, help=f"DeepFace model (default: {MODEL_NAME})")
    parser.add_argument("--threshold", type=float, default=DISTANCE_THRESHOLD,
                        help=f"Distance threshold (default: {DISTANCE_THRESHOLD})")

    args = parser.parse_args()

    globals()["MODEL_NAME"] = args.model
    globals()["DISTANCE_THRESHOLD"] = args.threshold

    if args.add:
        add_face(args.add, args.url)
    elif args.train:
        train()
    elif args.live:
        live(args.url)
    elif args.list:
        list_faces()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
