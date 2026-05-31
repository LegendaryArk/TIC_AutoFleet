#!/usr/bin/env python3
"""
Overhead-camera homography-based robot localization.

Tag assignment (tag36h11 family):
  IDs 0-3  — field corner markers (define the homography)
  IDs 4-5  — robot tags (tracked for pose)

Detected robot poses are sent to central-arbiter.py as
'vision_telemetry' messages over TCP, updating x_cm / y_cm /
theta_deg for each robot while leaving all other telemetry
fields (motor speeds, ultrasonic, etc.) untouched.

Usage:
    python3 vision_localizer.py [--no-arbiter]

Press  q  in the OpenCV window to quit.
Press  r  to force-recompute the homography on the next frame.
"""

import argparse
import json
import math
import queue
import socket
import threading
import time

import cv2
import numpy as np
from pupil_apriltags import Detector

# ---------------------------------------------------------------------------
# Config — adjust these to match your physical setup
# ---------------------------------------------------------------------------

ARBITER_HOST = "0.0.0.0"
ARBITER_PORT = 9000

CAMERA_INDEX = 4          # cv2.VideoCapture index
FIELD_SIZE_CM = 400.0     # arena is square

# Physical size of all tags (corner + robot) in metres.
# Set to the printed edge length of the black-bordered square.
TAG_SIZE_M = 0.10

# Orbbec Astra Pro Plus FHD RGB stream (1920×1080).
# Scaled proportionally from the 720p factory defaults (fx≈1050, cx=640, cy=360).
# Run cv2.calibrateCamera with a checkerboard for best heading accuracy.
CAM_PARAMS = [1598.0, 1598.0, 960.0, 540.0]
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

# How often (seconds) to push a vision_telemetry correction per robot.
# Detection still runs every frame for the debug display.
VISION_CORRECTION_INTERVAL_S = 3.0

# Corner EMA smoothing — filters vibration jitter.
# 0.1 = heavy smoothing (slow to adapt), 0.5 = light smoothing (fast to adapt).
CORNER_SMOOTH_ALPHA = 0.2
CORNER_MAX_AGE_S    = 5.0   # drop a smoothed corner if unseen for this long

# Corner tag IDs → field position (x_cm, y_cm).
# Origin is bottom-left of the field.
CORNER_TAG_POSITIONS: dict[int, tuple[float, float]] = {
    0: (0.0, 0.0),
    1: (FIELD_SIZE_CM, 0.0),
    2: (FIELD_SIZE_CM, FIELD_SIZE_CM),
    3: (0.0, FIELD_SIZE_CM),
}

# Robot tag IDs → robot_id string sent to the arbiter.
ROBOT_TAGS: dict[int, str] = {
    4: "robot_A",
    5: "robot_B",
}

# Display
FIELD_MAP_PX = 600        # side length of the field-map panel in pixels
CM_PER_PX = FIELD_SIZE_CM / FIELD_MAP_PX

ROBOT_COLORS = {
    "robot_A": (0, 215, 255),   # gold
    "robot_B": (255, 180, 0),   # cyan-ish
}
CORNER_COLOR  = (0, 220, 0)     # green
FIELD_BORDER_COLOR = (220, 100, 0)  # blue
ARROW_LEN_PX = 30

# ---------------------------------------------------------------------------
# Homography helpers
# ---------------------------------------------------------------------------

def compute_homography_smooth(corner_smooth: dict) -> np.ndarray | None:
    """Compute H (image → field cm) from EMA-smoothed corner positions."""
    src, dst = [], []
    for tag_id, entry in corner_smooth.items():
        if tag_id in CORNER_TAG_POSITIONS:
            src.append(entry["pos"])
            dst.append(list(CORNER_TAG_POSITIONS[tag_id]))
    if len(src) < 4:
        return None
    H, _ = cv2.findHomography(
        np.array(src, dtype=np.float32),
        np.array(dst, dtype=np.float32),
        cv2.RANSAC,
    )
    return H


def apply_homography(H: np.ndarray, pt: tuple[float, float]) -> tuple[float, float]:
    """Project a single image-space point through H → field cm."""
    src = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, H)
    return float(dst[0, 0, 0]), float(dst[0, 0, 1])


def localize_robot(tag, H: np.ndarray) -> tuple[float, float, float]:
    """
    Return (x_cm, y_cm, theta_deg) of a robot tag in field coordinates.

    Position  — homography applied to the tag centre.
    Heading   — angle of the tag's top-edge direction after homography,
                defined as the vector from bottom-edge midpoint to
                top-edge midpoint (corners[0..1] → corners[2..3]).
    """
    corners = tag.corners   # shape (4, 2): BL, BR, TR, TL in image space

    center_img = tag.center
    bottom_mid = (corners[0] + corners[1]) / 2.0
    top_mid    = (corners[2] + corners[3]) / 2.0

    cx_f, cy_f = apply_homography(H, center_img)
    bx_f, by_f = apply_homography(H, bottom_mid)
    tx_f, ty_f = apply_homography(H, top_mid)

    dx = tx_f - bx_f
    dy = ty_f - by_f
    theta_deg = math.degrees(math.atan2(dy, dx))

    return cx_f, cy_f, theta_deg


# ---------------------------------------------------------------------------
# TCP sender
# ---------------------------------------------------------------------------

class VisionSender:
    """
    Background thread that keeps a TCP connection to the arbiter and
    drains an outbound message queue.  Reconnects on any failure.
    """

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self._q: queue.Queue[dict] = queue.Queue()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="vision-sender")
        self._thread.start()

    def send(self, msg: dict) -> None:
        self._q.put(msg)

    def stop(self) -> None:
        self._stop_evt.set()

    def _worker(self) -> None:
        while not self._stop_evt.is_set():
            conn = self._connect()
            if conn is None:
                time.sleep(2.0)
                continue
            print(f"[VISION] Connected to arbiter at {self._host}:{self._port}")
            try:
                # Drain any stale queue items accumulated while disconnected
                while not self._q.empty():
                    try:
                        self._q.get_nowait()
                    except queue.Empty:
                        break

                while not self._stop_evt.is_set():
                    try:
                        msg = self._q.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    data = (json.dumps(msg) + "\n").encode("utf-8")
                    conn.sendall(data)
            except Exception as exc:
                print(f"[VISION] Send error: {exc}. Reconnecting…")
            finally:
                try:
                    conn.close()
                except Exception:
                    pass

    def _connect(self) -> socket.socket | None:
        try:
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.settimeout(4.0)
            conn.connect((self._host, self._port))
            conn.settimeout(None)
            conn.sendall((json.dumps({
                "type": "hello",
                "name": "vision_system",
            }) + "\n").encode("utf-8"))
            return conn
        except Exception as exc:
            print(f"[VISION] Cannot connect to arbiter: {exc}")
            return None


# ---------------------------------------------------------------------------
# Debug visualisation helpers
# ---------------------------------------------------------------------------

def _field_to_map_px(x_cm: float, y_cm: float) -> tuple[int, int]:
    """Convert field cm coords to pixel coords in the field-map panel."""
    px = int(x_cm / CM_PER_PX)
    # Flip Y so that (0,0) is bottom-left in display
    py = FIELD_MAP_PX - 1 - int(y_cm / CM_PER_PX)
    return px, py


def build_field_map(
    robot_poses: dict[str, tuple[float, float, float]],
    H_valid: bool,
    last_fix_t: dict[str, float] | None = None,
) -> np.ndarray:
    """
    Build a 600×600 px top-down field map showing robot positions.
    robot_poses: {robot_id: (x_cm, y_cm, theta_deg)}
    last_fix_t:  {robot_id: monotonic time of last sent correction}
    """
    img = np.full((FIELD_MAP_PX, FIELD_MAP_PX, 3), 30, dtype=np.uint8)

    # Grid lines every 10 cm
    step_px = int(10.0 / CM_PER_PX)
    for i in range(0, FIELD_MAP_PX, step_px):
        cv2.line(img, (i, 0), (i, FIELD_MAP_PX - 1), (60, 60, 60), 1)
        cv2.line(img, (0, i), (FIELD_MAP_PX - 1, i), (60, 60, 60), 1)

    # Field border
    cv2.rectangle(img, (0, 0), (FIELD_MAP_PX - 1, FIELD_MAP_PX - 1), FIELD_BORDER_COLOR, 2)

    # Corner markers
    for tag_id, (fx, fy) in CORNER_TAG_POSITIONS.items():
        px, py = _field_to_map_px(fx, fy)
        cv2.circle(img, (px, py), 8, CORNER_COLOR, -1)
        cv2.putText(img, str(tag_id), (px + 6, py - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, CORNER_COLOR, 1)

    # Homography status
    status_txt = "H: OK" if H_valid else "H: waiting for 4 corners"
    status_col = (0, 200, 0) if H_valid else (0, 80, 200)
    cv2.putText(img, status_txt, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_col, 1)

    # Robot glyphs
    for robot_id, (x_cm, y_cm, theta_deg) in robot_poses.items():
        color = ROBOT_COLORS.get(robot_id, (200, 200, 200))
        px, py = _field_to_map_px(x_cm, y_cm)

        # Circle body
        cv2.circle(img, (px, py), 10, color, 2)

        # Heading arrow
        theta_rad = math.radians(theta_deg)
        ex = int(px + ARROW_LEN_PX * math.cos(theta_rad))
        ey = int(py - ARROW_LEN_PX * math.sin(theta_rad))  # flip Y for display
        cv2.arrowedLine(img, (px, py), (ex, ey), color, 2, tipLength=0.3)

        label = f"{robot_id} ({x_cm:.0f},{y_cm:.0f}) {theta_deg:.0f}deg"
        cv2.putText(img, label, (px + 12, py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        if last_fix_t is not None and robot_id in last_fix_t:
            elapsed = time.monotonic() - last_fix_t[robot_id]
            fix_label = f"fix: {elapsed:.0f}s ago"
            cv2.putText(img, fix_label, (px + 12, py + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    return img


def draw_debug_frame(
    frame: np.ndarray,
    detections,
    H: np.ndarray | None,
    robot_poses: dict[str, tuple[float, float, float]],
    corner_smooth: dict | None = None,
) -> np.ndarray:
    """Return an annotated copy of frame."""
    out = frame.copy()

    for det in detections:
        corners = det.corners.astype(int)
        tag_id = det.tag_id

        if tag_id in CORNER_TAG_POSITIONS:
            color = CORNER_COLOR
            label = f"C{tag_id} {CORNER_TAG_POSITIONS[tag_id]}"
        elif tag_id in ROBOT_TAGS:
            color = ROBOT_COLORS.get(ROBOT_TAGS[tag_id], (200, 200, 200))
            robot_id = ROBOT_TAGS[tag_id]
            if robot_id in robot_poses:
                x, y, th = robot_poses[robot_id]
                label = f"{robot_id} ({x:.0f},{y:.0f}) {th:.0f}°"
            else:
                label = ROBOT_TAGS[tag_id]
        else:
            color = (180, 180, 180)
            label = f"ID {tag_id}"

        # Draw tag quad
        cv2.polylines(out, [corners.reshape(-1, 1, 2)], True, color, 2)

        # Label near top-left corner
        cv2.putText(out, label,
                    (corners[3][0], corners[3][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Center dot
        cx, cy = int(det.center[0]), int(det.center[1])
        cv2.circle(out, (cx, cy), 4, color, -1)

        # Heading arrow for robot tags
        if tag_id in ROBOT_TAGS and H is not None and ROBOT_TAGS[tag_id] in robot_poses:
            _, _, theta_deg = robot_poses[ROBOT_TAGS[tag_id]]
            # Draw arrow in image space (purely cosmetic; uses field theta)
            # Convert the heading back through inverse H to get image direction
            H_inv = np.linalg.inv(H)
            fx, fy = robot_poses[ROBOT_TAGS[tag_id]][:2]
            tip_f = np.array([[[fx + 15.0 * math.cos(math.radians(theta_deg)),
                                 fy + 15.0 * math.sin(math.radians(theta_deg))]]], dtype=np.float32)
            tip_img = cv2.perspectiveTransform(tip_f, H_inv)
            tx, ty = int(tip_img[0, 0, 0]), int(tip_img[0, 0, 1])
            cv2.arrowedLine(out, (cx, cy), (tx, ty), color, 2, tipLength=0.3)

    # Field boundary — draw using smoothed corner positions directly when available
    # (avoids H_inv numerical noise; falls back to H_inv reprojection otherwise)
    if corner_smooth and len(corner_smooth) == 4:
        ordered_ids = [0, 1, 2, 3]
        if all(tid in corner_smooth for tid in ordered_ids):
            pts = np.array([[corner_smooth[tid]["pos"].astype(int)]
                            for tid in ordered_ids], dtype=np.int32)
            cv2.polylines(out, [pts], True, FIELD_BORDER_COLOR, 2)
    elif H is not None:
        H_inv = np.linalg.inv(H)
        field_corners_cm = np.array([
            [[0.0, 0.0]],
            [[FIELD_SIZE_CM, 0.0]],
            [[FIELD_SIZE_CM, FIELD_SIZE_CM]],
            [[0.0, FIELD_SIZE_CM]],
        ], dtype=np.float32)
        img_corners = cv2.perspectiveTransform(field_corners_cm, H_inv)
        cv2.polylines(out, [img_corners.astype(int)], True, FIELD_BORDER_COLOR, 2)

    return out


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="AprilTag overhead localizer")
    parser.add_argument("--no-arbiter", action="store_true",
                        help="Run in display-only mode (no TCP connection)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[VISION] Cannot open camera index {CAMERA_INDEX}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[VISION] Camera opened at {actual_w}×{actual_h} "
          f"(requested {FRAME_WIDTH}×{FRAME_HEIGHT})")
    if (actual_w, actual_h) != (FRAME_WIDTH, FRAME_HEIGHT):
        print("[VISION] WARNING: resolution mismatch — CAM_PARAMS intrinsics "
              "may not match. Update FRAME_WIDTH/HEIGHT and CAM_PARAMS if needed.")

    # Discard the first few frames; cameras often return blank ones on init.
    for _ in range(5):
        cap.read()

    detector = Detector(families="tag36h11")

    sender: VisionSender | None = None
    if not args.no_arbiter:
        sender = VisionSender(ARBITER_HOST, ARBITER_PORT)

    H: np.ndarray | None = None
    force_recalibrate = False
    robot_poses: dict[str, tuple[float, float, float]] = {}
    last_send_t: dict[str, float] = {}
    # EMA-smoothed corner positions: {tag_id: {"pos": np.ndarray, "last_seen": float}}
    corner_smooth: dict[int, dict] = {}

    print("[VISION] Running. Press 'q' to quit, 'r' to recalibrate.")

    _diag_done = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[VISION] Frame capture failed")
            break

        if not _diag_done:
            print(f"[VISION] First live frame: shape={frame.shape}  mean={frame.mean():.1f}")
            _diag_done = True

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAM_PARAMS,
            tag_size=TAG_SIZE_M,
        )

        # Update per-corner EMA positions and recompute H every frame
        now_t = time.monotonic()

        if force_recalibrate:
            corner_smooth.clear()
            force_recalibrate = False
            print("[VISION] Corner positions cleared — rebuilding.")

        for det in detections:
            if det.tag_id in CORNER_TAG_POSITIONS:
                raw = np.array(det.center, dtype=np.float64)
                if det.tag_id in corner_smooth:
                    corner_smooth[det.tag_id]["pos"] = (
                        CORNER_SMOOTH_ALPHA * raw
                        + (1.0 - CORNER_SMOOTH_ALPHA) * corner_smooth[det.tag_id]["pos"]
                    )
                else:
                    corner_smooth[det.tag_id]["pos"] = raw.copy()
                corner_smooth[det.tag_id]["last_seen"] = now_t

        # Expire stale corners
        corner_smooth = {k: v for k, v in corner_smooth.items()
                         if now_t - v["last_seen"] < CORNER_MAX_AGE_S}

        if len(corner_smooth) >= 4:
            new_H = compute_homography_smooth(corner_smooth)
            if new_H is not None:
                H = new_H

        # Localise visible robot tags
        if H is not None:
            for det in detections:
                if det.tag_id in ROBOT_TAGS:
                    robot_id = ROBOT_TAGS[det.tag_id]
                    try:
                        x_cm, y_cm, theta_deg = localize_robot(det, H)
                    except Exception as exc:
                        print(f"[VISION] Localisation error for {robot_id}: {exc}")
                        continue

                    # Clamp to field bounds (handles slight homography overshoot)
                    x_cm = max(0.0, min(FIELD_SIZE_CM, x_cm))
                    y_cm = max(0.0, min(FIELD_SIZE_CM, y_cm))

                    robot_poses[robot_id] = (x_cm, y_cm, theta_deg)

                    # Rate-limited send
                    now = time.monotonic()
                    if sender and (now - last_send_t.get(robot_id, 0.0) >= VISION_CORRECTION_INTERVAL_S):
                        sender.send({
                            "type": "vision_telemetry",
                            "robot_id": robot_id,
                            "x_cm": round(x_cm, 2),
                            "y_cm": round(y_cm, 2),
                            "theta_deg": round(theta_deg, 2),
                            "t_ms": int(time.time() * 1000),
                        })
                        last_send_t[robot_id] = now

        # Build display
        try:
            annotated = draw_debug_frame(frame, detections, H, robot_poses, corner_smooth)
        except Exception as exc:
            print(f"[VISION] draw_debug_frame error: {exc}")
            annotated = frame.copy()

        field_map = build_field_map(robot_poses, H is not None, last_send_t)

        # Scale both panels to 720px tall, then place side by side
        h_target = 720
        w_cam = int(annotated.shape[1] * h_target / annotated.shape[0])
        w_map = int(field_map.shape[1] * h_target / field_map.shape[0])
        cam_scaled = cv2.resize(annotated, (w_cam, h_target))
        map_scaled = cv2.resize(field_map, (w_map, h_target))

        combined = np.hstack([cam_scaled, map_scaled])
        cv2.imshow("Vision Localizer", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("r"):
            force_recalibrate = True
            print("[VISION] Recalibration requested.")

    if sender:
        sender.stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
