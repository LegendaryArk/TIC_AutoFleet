"""
AprilTag-based localization and fixed-coordinate pickup controller.

Runs on the Raspberry Pi with an Orbbec FHD 1080p camera.
Uses 4 AprilTags placed on the field as landmarks to compute the
robot's global pose, then navigates to known object coordinates
and picks them up with the gripper.

Communicates with PRIZM via serial or ESP32 WiFi bridge using
the drive_power and toggle_gripper JSON commands.

Usage:
    python vision_servo.py --env ../robot_a.env
    python vision_servo.py --env ../robot_b.env --targets targets.json --show
"""

import argparse
import json
import math
import os
import time
from enum import Enum, auto

import cv2
import numpy as np
from dotenv import load_dotenv
from pupil_apriltags import Detector

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="AprilTag localization pickup controller")
parser.add_argument("--env", default="../.env", help="Path to env file")
parser.add_argument("--targets", default="targets.json",
                    help="Path to JSON file with pickup coordinates")
parser.add_argument("--camera", type=int, default=0, help="Camera device index")
parser.add_argument("--show", action="store_true",
                    help="Display camera feed with overlays (requires display)")
args = parser.parse_args()

load_dotenv(args.env)

ROBOT_ID = os.getenv("ROBOT_ID", "robot_A")
USE_WIFI_BRIDGE = os.getenv("USE_WIFI_BRIDGE", "false").lower() == "true"
SERIAL_PORT = os.getenv("LOCAL_SERIAL_PORT")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "115200"))
ESP32_HOST = os.getenv("ESP32_HOST")
ESP32_PORT = int(os.getenv("ESP32_PORT", "81"))

# ---------------------------------------------------------------------------
# AprilTag + camera config
# ---------------------------------------------------------------------------
TAG_SIZE_M = 0.16  # physical tag size in meters (16 cm)
TAG_FAMILY = "tag36h11"

# Orbbec / Astra Pro Plus intrinsics (fx, fy, cx, cy)
CAM_PARAMS = [1050.0, 1050.0, 640.0, 360.0]

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Known global positions and orientations of each field AprilTag.
# heading_deg: the direction the tag face is pointing in the global frame
# (0 = +X, 90 = +Y, etc.)
#
# >>> EDIT THESE to match your actual field layout <<<
FIELD_TAGS = {
    0: {"x_cm": 0.0,   "y_cm": 0.0,   "heading_deg": 0.0},
    1: {"x_cm": 400.0, "y_cm": 0.0,   "heading_deg": 90.0},
    2: {"x_cm": 400.0, "y_cm": 400.0, "heading_deg": 180.0},
    3: {"x_cm": 0.0,   "y_cm": 400.0, "heading_deg": 270.0},
}

# ---------------------------------------------------------------------------
# Navigation tuning -- adjust on the real robot
# ---------------------------------------------------------------------------
ARRIVE_THRESHOLD_CM = 5.0       # distance to consider "arrived"
HEADING_THRESHOLD_DEG = 10.0    # heading error to switch from turn to drive
TURN_POWER = 18                 # motor power when turning in place
DRIVE_POWER = 22                # base forward motor power
STEER_GAIN = 12.0               # proportional steering correction while driving
SPIN_SEARCH_POWER = 14          # slow spin when no tags visible
GRIPPER_SETTLE_S = 1.0          # wait after closing gripper
LOCALIZE_FRAMES = 3             # consecutive good frames before trusting pose


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
class State(Enum):
    LOCALIZE = auto()
    NAVIGATE = auto()
    PICKUP = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
# Transport
# ---------------------------------------------------------------------------
class Transport:
    """Sends JSON lines to PRIZM over serial or ESP32 TCP."""

    def __init__(self):
        if USE_WIFI_BRIDGE:
            import socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((ESP32_HOST, ESP32_PORT))
            self._mode = "wifi"
            print(f"[TRANSPORT] WiFi -> {ESP32_HOST}:{ESP32_PORT}")
        else:
            import serial as pyserial
            self.ser = pyserial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            self._mode = "serial"
            print(f"[TRANSPORT] Serial -> {SERIAL_PORT}")

    def send(self, payload: dict) -> None:
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        if self._mode == "serial":
            self.ser.write(line.encode("utf-8"))
            self.ser.flush()
        else:
            self.sock.sendall(line.encode("utf-8"))


def send_drive_power(transport: Transport, left: int, right: int) -> None:
    transport.send({
        "type": "drive_power",
        "robot_id": ROBOT_ID,
        "left": int(max(-100, min(100, left))),
        "right": int(max(-100, min(100, right))),
    })


def send_toggle_gripper(transport: Transport) -> None:
    transport.send({"type": "toggle_gripper", "robot_id": ROBOT_ID})


# ---------------------------------------------------------------------------
# Angle helpers
# ---------------------------------------------------------------------------
def normalize_angle(rad: float) -> float:
    """Wrap angle to (-pi, pi]."""
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    return rad


# ---------------------------------------------------------------------------
# Localization: compute robot pose from detected AprilTags
# ---------------------------------------------------------------------------
def compute_robot_pose(detections: list) -> tuple | None:
    """
    Given a list of pupil_apriltags detections (with pose estimation),
    return (x_cm, y_cm, theta_rad) in the global frame, or None.

    For each visible field tag we know:
      - tag's global position and heading
      - tag's position relative to the camera (pose_t, pose_R)

    pose_t = [x, y, z] of the tag in the *camera* frame (meters):
      x = right, y = down, z = forward

    We project onto the ground plane (x, z) to get the 2D offset,
    then rotate by the tag's known global heading to get the robot's
    global position.
    """
    estimates_x = []
    estimates_y = []
    estimates_theta = []

    for det in detections:
        tag_id = det.tag_id
        if tag_id not in FIELD_TAGS:
            continue

        tag_info = FIELD_TAGS[tag_id]
        tag_x = tag_info["x_cm"]
        tag_y = tag_info["y_cm"]
        tag_heading_rad = math.radians(tag_info["heading_deg"])

        # Tag position in camera frame (meters -> cm)
        cam_x = det.pose_t[0][0] * 100.0  # right
        cam_z = det.pose_t[2][0] * 100.0  # forward

        # Horizontal angle from camera center to tag
        angle_to_tag_cam = math.atan2(cam_x, cam_z)

        # Distance to tag in ground plane
        dist_cm = math.sqrt(cam_x ** 2 + cam_z ** 2)

        # The tag is facing toward the robot. The tag's heading in global
        # frame points outward from whatever surface it's mounted on.
        # The bearing FROM tag TO robot in global frame is the reverse
        # of the tag heading, offset by the camera-frame angle.
        bearing_tag_to_robot = tag_heading_rad + angle_to_tag_cam

        robot_x = tag_x + dist_cm * math.cos(bearing_tag_to_robot)
        robot_y = tag_y + dist_cm * math.sin(bearing_tag_to_robot)

        # Robot heading: the camera looks along +z in the camera frame.
        # In the global frame the camera is pointing FROM robot TOWARD tag.
        # So robot_theta = bearing from robot to tag = opposite of above.
        bearing_robot_to_tag = math.atan2(tag_y - robot_y, tag_x - robot_x)
        # The camera's optical axis is the robot's forward direction,
        # offset by angle_to_tag_cam (how far off-center the tag is).
        robot_theta = normalize_angle(bearing_robot_to_tag - angle_to_tag_cam)

        estimates_x.append(robot_x)
        estimates_y.append(robot_y)
        estimates_theta.append(robot_theta)

    if not estimates_x:
        return None

    avg_x = sum(estimates_x) / len(estimates_x)
    avg_y = sum(estimates_y) / len(estimates_y)

    # Circular mean for angles
    sin_sum = sum(math.sin(t) for t in estimates_theta)
    cos_sum = sum(math.cos(t) for t in estimates_theta)
    avg_theta = math.atan2(sin_sum, cos_sum)

    return (avg_x, avg_y, avg_theta)


# ---------------------------------------------------------------------------
# Load targets
# ---------------------------------------------------------------------------
def load_targets(path: str) -> list[dict]:
    with open(path, "r") as f:
        data = json.load(f)
    targets = data.get("targets", [])
    if not targets:
        raise ValueError(f"No targets found in {path}")
    print(f"[TARGETS] Loaded {len(targets)} pickup targets from {path}")
    for i, t in enumerate(targets):
        print(f"  [{i}] {t.get('label', 'unnamed')} @ ({t['x_cm']}, {t['y_cm']}) cm")
    return targets


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    targets = load_targets(args.targets)

    detector = Detector(families=TAG_FAMILY)

    print(f"[CAMERA] Opening device {args.camera}")
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        return

    transport = Transport()

    # Open gripper to start
    send_toggle_gripper(transport)
    time.sleep(0.3)
    send_toggle_gripper(transport)
    time.sleep(0.5)

    state = State.LOCALIZE
    target_idx = 0
    good_localize_count = 0
    robot_pose = None  # (x_cm, y_cm, theta_rad)

    print("[NAV] Starting AprilTag navigation loop")

    try:
        while state != State.DONE:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=CAM_PARAMS,
                tag_size=TAG_SIZE_M,
            )

            field_detections = [d for d in detections if d.tag_id in FIELD_TAGS]
            pose = compute_robot_pose(field_detections)

            # -----------------------------------------------------------
            # LOCALIZE: spin until we get a stable pose fix
            # -----------------------------------------------------------
            if state == State.LOCALIZE:
                if pose is not None:
                    good_localize_count += 1
                    robot_pose = pose
                    if good_localize_count >= LOCALIZE_FRAMES:
                        rx, ry, rt = robot_pose
                        print(f"[STATE] LOCALIZE -> NAVIGATE  "
                              f"pose=({rx:.1f}, {ry:.1f}) heading={math.degrees(rt):.1f}°")
                        send_drive_power(transport, 0, 0)
                        state = State.NAVIGATE
                else:
                    good_localize_count = 0
                    send_drive_power(transport, -SPIN_SEARCH_POWER, SPIN_SEARCH_POWER)

            # -----------------------------------------------------------
            # NAVIGATE: drive toward current target
            # -----------------------------------------------------------
            elif state == State.NAVIGATE:
                if pose is not None:
                    robot_pose = pose

                if robot_pose is None:
                    send_drive_power(transport, -SPIN_SEARCH_POWER, SPIN_SEARCH_POWER)
                    state = State.LOCALIZE
                    good_localize_count = 0
                    print("[STATE] NAVIGATE -> LOCALIZE (lost all tags)")
                    continue

                rx, ry, rt = robot_pose
                tx = targets[target_idx]["x_cm"]
                ty = targets[target_idx]["y_cm"]

                dx = tx - rx
                dy = ty - ry
                dist = math.sqrt(dx * dx + dy * dy)
                desired_heading = math.atan2(dy, dx)
                heading_err = normalize_angle(desired_heading - rt)
                heading_err_deg = math.degrees(heading_err)

                label = targets[target_idx].get("label", f"target_{target_idx}")
                print(f"[NAV] -> {label}  dist={dist:.1f}cm  "
                      f"heading_err={heading_err_deg:+.1f}°  "
                      f"pose=({rx:.1f},{ry:.1f},{math.degrees(rt):.1f}°)  "
                      f"tags={len(field_detections)}")

                if dist <= ARRIVE_THRESHOLD_CM:
                    send_drive_power(transport, 0, 0)
                    state = State.PICKUP
                    print(f"[STATE] NAVIGATE -> PICKUP (arrived at {label})")
                    continue

                if abs(heading_err_deg) > HEADING_THRESHOLD_DEG:
                    if heading_err > 0:
                        send_drive_power(transport, -TURN_POWER, TURN_POWER)
                    else:
                        send_drive_power(transport, TURN_POWER, -TURN_POWER)
                else:
                    steer = heading_err_deg / HEADING_THRESHOLD_DEG
                    left_power = int(DRIVE_POWER - steer * STEER_GAIN)
                    right_power = int(DRIVE_POWER + steer * STEER_GAIN)
                    left_power = max(-100, min(100, left_power))
                    right_power = max(-100, min(100, right_power))
                    send_drive_power(transport, left_power, right_power)

            # -----------------------------------------------------------
            # PICKUP: stop and close gripper
            # -----------------------------------------------------------
            elif state == State.PICKUP:
                send_drive_power(transport, 0, 0)
                time.sleep(0.3)

                print("[PICKUP] Closing gripper")
                send_toggle_gripper(transport)
                time.sleep(GRIPPER_SETTLE_S)

                target_idx += 1
                if target_idx < len(targets):
                    print(f"[PICKUP] Done. Moving to next target ({target_idx}/{len(targets)})")
                    # Re-open gripper for next pickup
                    send_toggle_gripper(transport)
                    time.sleep(0.5)
                    state = State.NAVIGATE
                else:
                    print(f"[PICKUP] All {len(targets)} targets collected!")
                    state = State.DONE

            # -----------------------------------------------------------
            # Visualization
            # -----------------------------------------------------------
            if args.show:
                for det in detections:
                    corners = det.corners.astype(int)
                    for j in range(4):
                        cv2.line(frame, tuple(corners[j]), tuple(corners[(j + 1) % 4]),
                                 (0, 255, 0), 2)
                    center = tuple(det.center.astype(int))
                    cv2.putText(frame, f"ID:{det.tag_id}", center,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                status = f"State: {state.name}"
                if robot_pose:
                    rx, ry, rt = robot_pose
                    status += f"  Pose: ({rx:.0f},{ry:.0f},{math.degrees(rt):.0f})"
                if target_idx < len(targets):
                    t = targets[target_idx]
                    status += f"  Target: {t.get('label','')} ({t['x_cm']},{t['y_cm']})"
                cv2.putText(frame, status, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("AprilTag Nav", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[NAV] Quit requested")
                    break

    except KeyboardInterrupt:
        print("\n[NAV] Interrupted by user")

    finally:
        print("[NAV] Stopping motors")
        send_drive_power(transport, 0, 0)
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        print("[NAV] Done")


if __name__ == "__main__":
    main()
