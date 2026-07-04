"""
object_tracker_ui.py
Tracks a single colored object via the webcam and draws a bounding box
around it. No serial/ESP32 required - this is just for testing detection.

USAGE:
    python object_tracker_ui.py               # normal run (loads saved config if present)
    python object_tracker_ui.py --calibrate   # opens HSV slider window with a Save button

CONTROLS (either mode):
    r  - pause the feed and drag a box around the object with your mouse
         (Enter/Space to confirm, Esc to cancel). The HSV range is computed
         automatically from the pixels inside the box.
    q  - quit

In calibrate mode, sliders update to match the box you selected, so you
can fine-tune afterward, then click "Save Config" to write the values to
color_config.json. Future runs auto-load that file.
"""

import argparse
import json
import os
import cv2
import numpy as np

# ---------------- USER SETTINGS ----------------
CAM_INDEX = 0
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "color_config.json")

# Fallback HSV range used only if no config file exists yet.
DEFAULT_HSV_LOWER = np.array([5, 150, 150])
DEFAULT_HSV_UPPER = np.array([15, 255, 255])

MIN_CONTOUR_AREA = 500
HSV_PADDING = (6, 25, 25)          # extra margin added to H, S, V after sampling a box

# --- ROI auto-calibration tuning ---
CORE_S_MIN = 60          # pixels below this saturation are treated as glare/shadow/background, not the object's color
CORE_V_MIN = 60          # pixels below this brightness are treated as shadow, not the object's color
IQR_PERCENTILES = (10, 90)   # trims outlier pixels within the "core" set (still-present specular highlights, etc.)
MAX_MATCH_FRACTION = 0.40    # if the computed range matches more than this fraction of the live frame, reject it
# -------------------------------------------------

BUTTON_RECT = (20, 20, 160, 50)  # x, y, w, h within the Calibration window
save_flash_frames = 0  # counts down after a save, to show "Saved!" briefly


def nothing(_):
    pass


def load_config():
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                cfg = json.load(f)
            lower = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]])
            upper = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]])
            print(f"Loaded calibration from {CONFIG_PATH}")
            return lower, upper
        except (json.JSONDecodeError, KeyError) as e:
            print(f"Config file was invalid ({e}), using defaults.")
    return DEFAULT_HSV_LOWER.copy(), DEFAULT_HSV_UPPER.copy()


def save_config(lower, upper):
    cfg = {
        "h_min": int(lower[0]), "h_max": int(upper[0]),
        "s_min": int(lower[1]), "s_max": int(upper[1]),
        "v_min": int(lower[2]), "v_max": int(upper[2]),
    }
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"Saved calibration to {CONFIG_PATH}: {cfg}")


def make_calibration_window(lower, upper):
    cv2.namedWindow("Calibration")
    cv2.createTrackbar("H min", "Calibration", int(lower[0]), 179, nothing)
    cv2.createTrackbar("H max", "Calibration", int(upper[0]), 179, nothing)
    cv2.createTrackbar("S min", "Calibration", int(lower[1]), 255, nothing)
    cv2.createTrackbar("S max", "Calibration", int(upper[1]), 255, nothing)
    cv2.createTrackbar("V min", "Calibration", int(lower[2]), 255, nothing)
    cv2.createTrackbar("V max", "Calibration", int(upper[2]), 255, nothing)


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


def set_calibration_trackbars(lower, upper):
    cv2.setTrackbarPos("H min", "Calibration", int(lower[0]))
    cv2.setTrackbarPos("H max", "Calibration", int(upper[0]))
    cv2.setTrackbarPos("S min", "Calibration", int(lower[1]))
    cv2.setTrackbarPos("S max", "Calibration", int(upper[1]))
    cv2.setTrackbarPos("V min", "Calibration", int(lower[2]))
    cv2.setTrackbarPos("V max", "Calibration", int(upper[2]))


def draw_calibration_panel(hsv_lower, hsv_upper):
    """Renders a small panel below the trackbars with a clickable Save button."""
    global save_flash_frames
    panel = np.full((110, 300, 3), 40, dtype=np.uint8)

    bx, by, bw, bh = BUTTON_RECT
    color = (0, 200, 0) if save_flash_frames <= 0 else (0, 255, 255)
    cv2.rectangle(panel, (bx, by), (bx + bw, by + bh), color, -1)
    label = "Saved!" if save_flash_frames > 0 else "Save Config"
    text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
    tx = bx + (bw - text_size[0]) // 2
    ty = by + (bh + text_size[1]) // 2
    cv2.putText(panel, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    values_text = f"H:{hsv_lower[0]}-{hsv_upper[0]}  S:{hsv_lower[1]}-{hsv_upper[1]}  V:{hsv_lower[2]}-{hsv_upper[2]}"
    cv2.putText(panel, values_text, (10, by + bh + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    if save_flash_frames > 0:
        save_flash_frames -= 1

    return panel


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        bx, by, bw, bh = BUTTON_RECT
        if bx <= x <= bx + bw and by <= y <= by + bh:
            lower, upper = read_calibration()
            save_config(lower, upper)
            global save_flash_frames
            save_flash_frames = 15  # ~0.5s at typical loop speed


def hsv_range_from_roi(frame, roi):
    """Given a frame and an (x, y, w, h) box, compute a robust HSV range
    from the pixels inside it. Glare/shadow/near-gray pixels (which have
    unreliable hue) are excluded from the color estimate when possible."""
    x, y, w, h = roi
    sub = frame[y:y + h, x:x + w]
    hsv_sub = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(int)

    s_ch, v_ch = hsv_sub[:, 1], hsv_sub[:, 2]
    core_mask = (s_ch >= CORE_S_MIN) & (v_ch >= CORE_V_MIN)
    core = hsv_sub[core_mask]

    if len(core) < 0.1 * len(hsv_sub):
        print("Note: selected area has low color saturation (white/gray/black object, "
              "or too much glare) - color-based tracking works best on bright, "
              "saturated colors. Using the full selection anyway.")
        core = hsv_sub

    lo_pct, hi_pct = IQR_PERCENTILES
    h_lo, h_hi = np.percentile(core[:, 0], [lo_pct, hi_pct])
    s_lo, s_hi = np.percentile(core[:, 1], [lo_pct, hi_pct])
    v_lo, v_hi = np.percentile(core[:, 2], [lo_pct, hi_pct])

    if h_hi - h_lo > 100:
        print("Warning: the selected object's hue spans a very wide range (this "
              "commonly happens with RED objects, since red wraps around both "
              "ends of the hue scale). Try --calibrate mode and adjust the H "
              "sliders manually for red objects.")

    ph, ps, pv = HSV_PADDING
    lower = np.array([max(0, h_lo - ph), max(0, s_lo - ps), max(0, v_lo - pv)], dtype=int)
    upper = np.array([min(179, h_hi + ph), min(255, s_hi + ps), min(255, v_hi + pv)], dtype=int)
    return lower, upper


def range_match_fraction(frame, lower, upper):
    """What fraction of the whole frame would this HSV range mark as 'object'?"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    return np.count_nonzero(mask) / mask.size


def select_object_roi(frame):
    """Lets the user drag a box around the object on a frozen frame.
    Returns (x, y, w, h) or None if cancelled."""
    roi = cv2.selectROI("Drag a box around the object, then press Enter", frame, showCrosshair=True)
    cv2.destroyWindow("Drag a box around the object, then press Enter")
    x, y, w, h = roi
    if w == 0 or h == 0:
        print("Selection cancelled.")
        return None
    return (x, y, w, h)


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
    parser.add_argument("--calibrate", action="store_true", help="show HSV slider window with Save button")
    args = parser.parse_args()

    hsv_lower, hsv_upper = load_config()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    if args.calibrate:
        make_calibration_window(hsv_lower, hsv_upper)
        cv2.setMouseCallback("Calibration", on_mouse)

    print("Press 'r' to draw a box around an object and auto-calibrate. Press 'q' to quit.")

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

        cv2.putText(frame, "Press 'r' to select object", (10, frame.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Object Tracker", frame)
        if args.calibrate:
            cv2.imshow("Mask", mask)
            cv2.imshow("Calibration", draw_calibration_panel(hsv_lower, hsv_upper))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("r"):
            roi = select_object_roi(frame)
            if roi is not None:
                new_lower, new_upper = hsv_range_from_roi(frame, roi)
                frac = range_match_fraction(frame, new_lower, new_upper)

                if frac > MAX_MATCH_FRACTION:
                    print(f"Rejected: that range matched {frac * 100:.0f}% of the current frame "
                          f"(too broad to be useful). Keeping previous calibration.")
                    print("Try again with a smaller box tightly inside the object, avoiding "
                          "edges, glare, and shadowed areas - and prefer a solid, brightly "
                          "colored object over a patterned or pale one.")
                else:
                    hsv_lower, hsv_upper = new_lower, new_upper
                    print(f"Calibrated from selection: lower={hsv_lower.tolist()} "
                          f"upper={hsv_upper.tolist()} (matches {frac * 100:.0f}% of frame)")
                    if args.calibrate:
                        set_calibration_trackbars(hsv_lower, hsv_upper)
                    else:
                        save_config(hsv_lower, hsv_upper)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()