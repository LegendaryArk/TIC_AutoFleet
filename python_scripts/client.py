import json
import math
import os
import socket
import threading
import time
import argparse
import serial
from dotenv import load_dotenv

# ----------------------------
# Configuration
# ----------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--env", default="../.env", help="Path to env file")
args = parser.parse_args()
load_dotenv(args.env)

SERVER_HOST = os.getenv("SERVER_HOST_IP_ADDRESS")
SERVER_PORT = int(os.getenv("SERVER_PORT", "9000"))

SERIAL_PORT = os.getenv("LOCAL_SERIAL_PORT")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "115200"))

ROBOT_ID = os.getenv("ROBOT_ID", "robot_A")
CLIENT_NAME = os.getenv("CLIENT_NAME", f"{ROBOT_ID}_laptop")

# Pure pursuit constants (tunable via .env)
WHEEL_DIAMETER_CM    = 10.16
WHEELBASE_CM         = 26.035
PP_LOOKAHEAD_CM      = float(os.getenv("PP_LOOKAHEAD_CM",      "25.0"))
PP_LINEAR_SPEED_CM_S = float(os.getenv("PP_LINEAR_SPEED_CM_S", "15.0"))
PP_GOAL_TOLERANCE_CM = float(os.getenv("PP_GOAL_TOLERANCE_CM",  "5.0"))

# Wifi Connection envs
USE_WIFI_BRIDGE = os.getenv("USE_WIFI_BRIDGE", "false").lower() == "true"

ESP32_HOST = os.getenv("ESP32_HOST")
ESP32_PORT = int(os.getenv("ESP32_PORT", "81"))

# Robot Communication Class
class RobotTransport:
    def __init__(self):
        self.mode = "wifi" if USE_WIFI_BRIDGE else "serial"

        if self.mode == "serial":
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            print(f"[COMMUNICATION] Using SERIAL {SERIAL_PORT}")

        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((ESP32_HOST, ESP32_PORT))
            print(f"[COMMUNICATION] Using WIFI {ESP32_HOST}:{ESP32_PORT}")

    def write(self, message: str):
        if self.mode == "serial":
            self.ser.write(message.encode("utf-8"))
            self.ser.flush()
        else:
            # send raw string to ESP32 websocket-like TCP
            self.sock.sendall((message).encode("utf-8"))

    def readline(self):
        if self.mode == "serial":
            return self.ser.readline().decode("utf-8", errors="replace")
        else:
            try:
                data = self.sock.recv(1024)
                return data.decode("utf-8", errors="replace")
            except:
                return ""

# Pure pursuit controller
class PurePursuitController:
    def __init__(self, waypoints: list[dict], path_id):
        self.waypoints    = waypoints   # [{"x_cm": float, "y_cm": float}, ...]
        self.path_id      = path_id
        self.closest_idx  = 0
        self.done         = False

    def compute_velocity(self, x_cm: float, y_cm: float, theta_deg: float) -> tuple[float, float]:
        """Returns (left_m_s, right_m_s). Sets done=True and returns (0, 0) at goal."""
        if self.done:
            return 0.0, 0.0

        final = self.waypoints[-1]
        if math.hypot(final["x_cm"] - x_cm, final["y_cm"] - y_cm) < PP_GOAL_TOLERANCE_CM:
            self.done = True
            return 0.0, 0.0

        n = len(self.waypoints)

        # Advance closest index: furthest waypoint the robot has already passed
        for i in range(self.closest_idx, n - 1):
            if math.hypot(self.waypoints[i]["x_cm"] - x_cm,
                          self.waypoints[i]["y_cm"] - y_cm) < PP_LOOKAHEAD_CM:
                self.closest_idx = i

        # Walk path segments from closest_idx until arc length >= PP_LOOKAHEAD_CM
        lookahead_x, lookahead_y = final["x_cm"], final["y_cm"]
        arc = 0.0
        for i in range(self.closest_idx, n - 1):
            a, b = self.waypoints[i], self.waypoints[i + 1]
            seg = math.hypot(b["x_cm"] - a["x_cm"], b["y_cm"] - a["y_cm"])
            if arc + seg >= PP_LOOKAHEAD_CM:
                t = (PP_LOOKAHEAD_CM - arc) / seg
                lookahead_x = a["x_cm"] + t * (b["x_cm"] - a["x_cm"])
                lookahead_y = a["y_cm"] + t * (b["y_cm"] - a["y_cm"])
                break
            arc += seg

        # Pure pursuit curvature: κ = 2 sin(α) / L_d
        theta_rad = math.radians(theta_deg)
        alpha = math.atan2(lookahead_y - y_cm, lookahead_x - x_cm) - theta_rad
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi   # normalize to [-π, π]

        kappa = 2.0 * math.sin(alpha) / PP_LOOKAHEAD_CM
        v     = PP_LINEAR_SPEED_CM_S
        omega = v * kappa                           # rad/s
        v_r   = v + omega * (WHEELBASE_CM / 2.0)   # cm/s right wheel
        v_l   = v - omega * (WHEELBASE_CM / 2.0)   # cm/s left wheel

        return v_l / 100.0, v_r / 100.0            # cm/s → m/s


# Module-level pure pursuit state
_pursuit_lock       = threading.Lock()
_pursuit_controller: "PurePursuitController | None" = None


# Command types that should be forwarded from server -> robot serial
FORWARD_TO_ROBOT_TYPES = {
    "path_assignment",
    "pause",
    "resume",
    "stop",
    "toggle_gripper",
}

serial_write_lock = threading.Lock()

DEFAULT_TURN_SPEED = 150
DEFAULT_DRIVE_SPEED = 200
SERIAL_RETRY_COUNTS = {
    "pause": 5,
    "stop": 5,
    "resume": 3,
}
SERIAL_RETRY_DELAY_S = 0.03


def send_json_line_over_socket(sock: socket.socket, payload: dict) -> None:
    message = json.dumps(payload, separators=(",", ":")) + "\n"
    sock.sendall(message.encode("utf-8"))


def compact_payload_for_serial(payload: dict) -> dict:
    if payload.get("type") != "path_assignment":
        return payload

    compact = {
        "type": "path_assignment",
        "robot_id": payload.get("robot_id"),
        "path_id": payload.get("path_id"),
        "replace_existing": bool(payload.get("replace_existing", True)),
    }

    waypoints = []
    for waypoint in payload.get("waypoints", []):
        x_cm = waypoint.get("x_cm")
        y_cm = waypoint.get("y_cm")
        if x_cm is None or y_cm is None:
            continue

        waypoints.append({
            "x_cm": int(round(float(x_cm))),
            "y_cm": int(round(float(y_cm))),
        })

    compact["waypoints"] = waypoints

    motion = payload.get("motion")
    if isinstance(motion, dict):
        turn_speed = motion.get("turn_speed_deg_per_sec")
        drive_speed = motion.get("drive_speed_deg_per_sec")

        if turn_speed is not None:
            turn_speed = int(turn_speed)
        if drive_speed is not None:
            drive_speed = int(drive_speed)

        if turn_speed not in {None, DEFAULT_TURN_SPEED} or drive_speed not in {None, DEFAULT_DRIVE_SPEED}:
            compact["motion"] = {}
            if turn_speed is not None:
                compact["motion"]["turn_speed_deg_per_sec"] = turn_speed
            if drive_speed is not None:
                compact["motion"]["drive_speed_deg_per_sec"] = drive_speed

    return compact


def encode_payload_for_serial(payload: dict) -> str:
    msg_type = payload.get("type")

    if msg_type == "pause":
        return "P\n"
    if msg_type == "resume":
        return "R\n"
    if msg_type == "stop":
        return "S\n"
    if msg_type == "velocity":
        return json.dumps(payload, separators=(",", ":")) + "\n"

    compact_payload = compact_payload_for_serial(payload)
    return json.dumps(compact_payload, separators=(",", ":")) + "\n"


def send_json_line_over_transport(transport: RobotTransport, payload: dict) -> None:
    message = encode_payload_for_serial(payload)
    retry_count = SERIAL_RETRY_COUNTS.get(payload.get("type"), 3)

    with serial_write_lock:
        # For large JSON Commands, stop Robot Telemetry Data from Coming in
        if payload.get("type") == "path_assignment":
            transport.write("S\n")                      # Stop the Robot
            time.sleep(0.8)                             # Delay for Buffer

        for attempt in range(retry_count):
            transport.write(message)
            if attempt + 1 < retry_count:
                time.sleep(SERIAL_RETRY_DELAY_S)

_PURSUIT_SUPPRESS_TYPES = {"path_started", "waypoint_reached", "path_complete", "ack"}


def _start_pure_pursuit(msg: dict, sock: socket.socket) -> None:
    global _pursuit_controller
    waypoints = [
        {"x_cm": float(w["x_cm"]), "y_cm": float(w["y_cm"])}
        for w in msg.get("waypoints", [])
    ]
    with _pursuit_lock:
        _pursuit_controller = PurePursuitController(waypoints, msg.get("path_id"))
    send_json_line_over_socket(sock, {
        "type": "ack",
        "for": "path_assignment",
        "path_id": msg.get("path_id"),
        "robot_id": ROBOT_ID,
    })
    print(f"[PURSUIT] started path_id={msg.get('path_id')} ({len(waypoints)} waypoints)")


def forward_serial_to_server(transport: RobotTransport, sock: socket.socket) -> None:
    """
    Continuously read newline-delimited JSON from serial and forward it to the server.
    When pure pursuit is active, intercepts telemetry to run the control loop and
    suppresses firmware path lifecycle events.
    """
    while True:
        try:
            line = transport.readline().strip()

            if not line:
                continue

            print(f"[SERIAL->TCP raw] {line}")

            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print(f"[WARN] Ignoring non-JSON serial line: {line}")
                continue

            if "robot_id" not in msg:
                msg["robot_id"] = ROBOT_ID

            # Pure pursuit control loop: run on every telemetry tick
            if msg.get("type") == "telemetry":
                with _pursuit_lock:
                    ctrl = _pursuit_controller
                if ctrl is not None:
                    left_m_s, right_m_s = ctrl.compute_velocity(
                        float(msg.get("x_cm",      0)),
                        float(msg.get("y_cm",      0)),
                        float(msg.get("theta_deg", 0)),
                    )
                    vel_cmd = json.dumps({
                        "type":      "velocity",
                        "robot_id":  ROBOT_ID,
                        "left_m_s":  round(left_m_s,  4),
                        "right_m_s": round(right_m_s, 4),
                    }, separators=(",", ":")) + "\n"
                    with serial_write_lock:
                        transport.write(vel_cmd)
                    print(f"[PURSUIT] vel l={left_m_s:.3f} r={right_m_s:.3f} m/s")

                    if ctrl.done:
                        with _pursuit_lock:
                            _pursuit_controller = None
                        send_json_line_over_socket(sock, {
                            "type":     "path_complete",
                            "robot_id": ROBOT_ID,
                            "path_id":  ctrl.path_id,
                            "t_ms":     int(time.time() * 1000),
                            "x_cm":     msg.get("x_cm"),
                            "y_cm":     msg.get("y_cm"),
                            "theta_deg": msg.get("theta_deg"),
                        })
                        print(f"[PURSUIT] path_id={ctrl.path_id} complete")

            # Suppress firmware path lifecycle events while pure pursuit is running
            elif msg.get("type") in _PURSUIT_SUPPRESS_TYPES:
                with _pursuit_lock:
                    active = _pursuit_controller is not None
                if active:
                    print(f"[PURSUIT] suppressed firmware event: {msg.get('type')}")
                    continue

            # Forward all other messages (telemetry, status, etc.) to server
            send_json_line_over_socket(sock, msg)
            print(f"[SERIAL->TCP sent] {msg.get('type')}")

        except Exception as e:
            print(f"[ERROR serial->server] {e}")
            break


def receive_from_server(sock: socket.socket, transport: RobotTransport) -> None:
    """
    Continuously read newline-delimited JSON from server.
    Forward command messages to robot over serial.
    """
    buffer = ""

    while True:
        try:
            data = sock.recv(4096)

            if not data:
                print("Server disconnected")
                break

            buffer += data.decode("utf-8", errors="replace")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                print(f"[TCP->CLIENT raw] {line}")

                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    print(f"[WARN] Ignoring non-JSON server line: {line}")
                    continue

                msg_type = msg.get("type", "")

                # Handle server hello/ack/etc locally
                if msg_type in {"hello_ack", "ack", "heartbeat_ack", "error"}:
                    print(f"[TCP->CLIENT local] {msg}")
                    continue

                # Multi-waypoint path assignments go to pure pursuit controller
                if msg_type == "path_assignment" and len(msg.get("waypoints", [])) > 1:
                    _start_pure_pursuit(msg, sock)
                    print(f"[TCP->PURSUIT] path_assignment intercepted for pure pursuit")

                # Forward other robot commands to serial
                elif msg_type in FORWARD_TO_ROBOT_TYPES:
                    send_json_line_over_transport(transport, msg)
                    print(f"[TCP->SERIAL forwarded] {msg_type} to robot")

                else:
                    print(f"[TCP->CLIENT ignored] unknown/unhandled type: {msg_type}")

        except Exception as e:
            print(f"[ERROR server->client] {e}")
            break


def heartbeat_loop(sock: socket.socket) -> None:
    """
    Optional heartbeat from laptop client to arbiter.
    """
    while True:
        try:
            heartbeat = {
                "type": "heartbeat",
                "robot_id": ROBOT_ID,
                "state": "connected",
                "t_ms": int(time.time() * 1000),
            }
            send_json_line_over_socket(sock, heartbeat)
            time.sleep(2.0)
        except Exception as e:
            print(f"[ERROR heartbeat] {e}")
            break


def main() -> None:
    # Open Communication to Robot (either wired Serial or WiFi)
    transport = RobotTransport()
    
    if not SERVER_HOST:
        print("Missing SERVER_HOST_IP_ADDRESS in .env")
        return

    if not SERIAL_PORT:
        print("Missing LOCAL_SERIAL_PORT in .env")
        return

    try:
        # Connect to central arbiter
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SERVER_HOST, SERVER_PORT))
        print(f"Connected to server {SERVER_HOST}:{SERVER_PORT}")

        # Register this laptop/robot with the arbiter
        hello_msg = {
            "type": "hello",
            "robot_id": ROBOT_ID,
            "client_name": CLIENT_NAME,
            "state": "ready",
        }
        send_json_line_over_socket(sock, hello_msg)
        print(f"[CLIENT->TCP sent] hello for {ROBOT_ID}")

        # Start background receiver for commands from server
        threading.Thread(
            target=receive_from_server,
            args=(sock, transport),
            daemon=True,
        ).start()

        # Optional heartbeat thread
        threading.Thread(
            target=heartbeat_loop,
            args=(sock,),
            daemon=True,
        ).start()

        # Main loop: forward robot telemetry to server
        forward_serial_to_server(transport, sock)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

    except ConnectionRefusedError:
        print("Connection refused. Is the arbiter running?")

    except TimeoutError:
        print("Connection timed out. Check network settings/firewall.")

    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
