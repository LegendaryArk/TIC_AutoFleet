import heapq
import io
import json
import math
from collections import defaultdict
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

_HEADLESS = "--no-gui" in sys.argv

if _HEADLESS:
    class TelemetryGUI:
        def __init__(self, **kw): pass
        def update_robot(self, *a, **kw): pass
        def run(self): threading.Event().wait()
else:
    from gui import TelemetryGUI

from dashboard.server import DashboardServer

HOST = "0.0.0.0"
PORT = 9000
MAX_CLIENTS = 10
GRID_CELL_CM = 10.0
GRID_DIM_CELLS = 40


@dataclass
class ClientSession:
    client_id: int
    conn: socket.socket
    addr: tuple
    name: str = ""
    robot_id: Optional[str] = None
    state: str = "connected"
    last_heartbeat: float = field(default_factory=time.time)
    last_telemetry: Optional[dict] = None
    last_status: Optional[dict] = None
    current_path_id: Optional[str] = None
    current_waypoint_index: Optional[int] = None
    send_lock: threading.Lock = field(default_factory=threading.Lock)
    pending_waypoints: list[dict] = field(default_factory=list)
    pending_motion: Optional[dict] = None
    sequence_path_id: Optional[int] = None
    active_subpath_id: Optional[int] = None
    awaiting_path_ack: bool = False
    awaiting_path_complete: bool = False
    gripper_open: bool = False
    nav_path_waypoints: list[dict] = field(default_factory=list)


clients_lock = threading.Lock()

# keyed by client_id
client_sessions: Dict[int, ClientSession] = {}

# keyed by robot_id
robots_by_id: Dict[str, int] = {}

# AprilTag role registry — keyed by tag_id (int)
# Each entry: {tag_id, role, row, col, x_cm, y_cm}
# Protected by clients_lock
obstacle_tags: Dict[int, dict] = {}
goal_tags: Dict[int, dict] = {}

next_client_id = 1
next_robot_path_id = 1000


# ----------------------------
# Networking helpers
# ----------------------------
def send_json(conn: socket.socket, message: dict) -> None:
    data = json.dumps(message) + "\n"
    conn.sendall(data.encode("utf-8"))


def recv_lines(conn: socket.socket):
    buffer = ""
    while True:
        data = conn.recv(4096)
        if not data:
            break

        buffer += data.decode("utf-8", errors="replace")

        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line:
                yield line


def get_session(client_id: int) -> Optional[ClientSession]:
    with clients_lock:
        return client_sessions.get(client_id)


def get_robot_snapshot() -> dict:
    with clients_lock:
        robots = {}
        for robot_id, client_id in robots_by_id.items():
            session = client_sessions.get(client_id)
            if session is None:
                continue

            telem = session.last_telemetry or {}
            robots[robot_id] = {
                "client_id": session.client_id,
                "name": session.name,
                "state": session.state,
                "path_id": session.current_path_id,
                "waypoint_index": session.current_waypoint_index,
                "last_heartbeat": session.last_heartbeat,
                "last_telemetry": telem,
                "last_status": session.last_status,
                # Flattened telemetry fields for easy canvas access
                "x_cm": telem.get("x_cm"),
                "y_cm": telem.get("y_cm"),
                "theta_deg": telem.get("theta_deg"),
                "front_ultrasonic_cm": telem.get("front_ultrasonic_cm"),
                "left_ultrasonic_cm": telem.get("left_ultrasonic_cm"),
                "left_motor_vel_deg_per_sec": telem.get("left_motor_vel_deg_per_sec"),
                "right_motor_vel_deg_per_sec": telem.get("right_motor_vel_deg_per_sec"),
                # Vision-fused pose (populated by apriltag-localization branch)
                "fused_x": telem.get("fused_x"),
                "fused_y": telem.get("fused_y"),
                "fused_theta": telem.get("fused_theta"),
                # Gripper state (toggled server-side; TODO: report from firmware)
                "gripper_open": session.gripper_open,
                # Planned A* path waypoints for canvas overlay
                "nav_path_waypoints": list(session.nav_path_waypoints),
                # Queued waypoints remaining (coordinated traverse)
                "pending_waypoint_count": len(session.pending_waypoints),
            }

        return {
            "robots": robots,
            "obstacles": list(obstacle_tags.values()),
            "goals": list(goal_tags.values()),
        }


def print_robot_table() -> None:
    with clients_lock:
        print("\n=== Robot Table ===")
        if not client_sessions:
            print("(no clients connected)")
        for client_id, session in client_sessions.items():
            print(
                f"client_id={client_id} "
                f"robot_id={session.robot_id} "
                f"name={session.name!r} "
                f"state={session.state!r} "
                f"path_id={session.current_path_id!r} "
                f"waypoint_index={session.current_waypoint_index!r} "
                f"last_heartbeat={session.last_heartbeat:.1f}"
            )
        print("===================\n")


def send_to_robot(robot_id: str, message: dict) -> bool:
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[SEND] No connected session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            print(f"[SEND] Session disappeared for robot_id={robot_id}")
            return False

    try:
        with session.send_lock:
            send_json(session.conn, message)

        print(f"[SEND] -> {robot_id}: {message}")
        return True

    except Exception as exc:
        print(f"[!] Failed to send to robot {robot_id}: {exc}")
        return False


def next_subpath_id() -> int:
    global next_robot_path_id
    next_robot_path_id += 1
    if next_robot_path_id > 30000:
        next_robot_path_id = 1000
    return next_robot_path_id


def clear_robot_sequence(session: ClientSession) -> None:
    session.pending_waypoints.clear()
    session.pending_motion = None
    session.sequence_path_id = None
    session.active_subpath_id = None
    session.awaiting_path_ack = False
    session.awaiting_path_complete = False


def any_other_robot_busy_unlocked(robot_id: str) -> bool:
    for session in client_sessions.values():
        if session.robot_id and session.robot_id != robot_id and session.awaiting_path_complete:
            return True
    return False


def maybe_dispatch_waiting_sequences() -> bool:
    with clients_lock:
        robot_ids = [
            session.robot_id
            for session in client_sessions.values()
            if session.robot_id and session.pending_waypoints and not session.awaiting_path_complete
        ]

    dispatched = False
    for robot_id in robot_ids:
        if dispatch_next_waypoint(robot_id):
            dispatched = True
            break

    return dispatched


def dispatch_next_waypoint(robot_id: str) -> bool:
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[SEQUENCE] No connected session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            print(f"[SEQUENCE] Session disappeared for robot_id={robot_id}")
            return False

        if any_other_robot_busy_unlocked(robot_id):
            return False

        if session.awaiting_path_ack or session.awaiting_path_complete:
            return False

        if not session.pending_waypoints:
            session.pending_motion = None
            session.sequence_path_id = None
            session.active_subpath_id = None
            return False

        waypoint = session.pending_waypoints.pop(0)
        subpath_id = next_subpath_id()
        message = {
            "type": "path_assignment",
            "robot_id": robot_id,
            "path_id": subpath_id,
            "replace_existing": True,
            "waypoints": [waypoint],
        }
        if session.pending_motion is not None:
            message["motion"] = dict(session.pending_motion)

        session.active_subpath_id = subpath_id
        session.current_path_id = str(subpath_id)
        session.current_waypoint_index = 0
        session.awaiting_path_ack = True
        session.awaiting_path_complete = True

    ok = send_to_robot(robot_id, message)
    if not ok:
        with clients_lock:
            client_id = robots_by_id.get(robot_id)
            session = client_sessions.get(client_id) if client_id is not None else None
            if session is not None:
                session.pending_waypoints.insert(0, waypoint)
                session.active_subpath_id = None
                session.awaiting_path_ack = False
                session.awaiting_path_complete = False
        return False

    print(f"[SEQUENCE] dispatched subpath {subpath_id} to {robot_id} -> {waypoint}")
    return True


def clamp_cell(value: int) -> int:
    return max(0, min(GRID_DIM_CELLS - 1, value))


def pose_to_cell(telemetry: dict) -> tuple[int, int] | None:
    try:
        x_cm = float(telemetry["x_cm"])
        y_cm = float(telemetry["y_cm"])
    except (KeyError, TypeError, ValueError):
        return None

    col = clamp_cell(int(x_cm // GRID_CELL_CM))
    row = clamp_cell(int(y_cm // GRID_CELL_CM))
    return row, col


def cell_center_waypoint(cell: tuple[int, int]) -> dict:
    row, col = cell
    return {
        "x_cm": col * GRID_CELL_CM + GRID_CELL_CM / 2.0,
        "y_cm": row * GRID_CELL_CM + GRID_CELL_CM / 2.0,
    }


def plan_grid_path(
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: set[tuple[int, int]],
) -> list[tuple[int, int]] | None:
    if start == goal:
        return [start]

    def h(cell):
        dr = abs(cell[0] - goal[0])
        dc = abs(cell[1] - goal[1])
        return max(dr, dc)  # Chebyshev — admissible for 8-directional movement

    open_set = [(h(start), 0, start)]
    came_from: dict = {start: None}
    g_score: dict = {start: 0}
    tie = 0

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        row, col = current
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
            neighbor = (row + d_row, col + d_col)
            nr, nc = neighbor
            if not (0 <= nr < GRID_DIM_CELLS and 0 <= nc < GRID_DIM_CELLS):
                continue
            if neighbor in blocked and neighbor != goal:
                continue
            new_g = g_score[current] + 1
            if new_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = new_g
                tie += 1
                heapq.heappush(open_set, (new_g + h(neighbor), tie, neighbor))

    return None


def start_coordinated_traverse(message: dict) -> bool:
    robots = message.get("robots", [])
    if len(robots) != 2:
        print("[COORD] Expected exactly two robots in coordinated_traverse request")
        return False

    robot_one = robots[0]
    robot_two = robots[1]

    robot_one_id = str(robot_one.get("robot_id", "")).strip()
    robot_two_id = str(robot_two.get("robot_id", "")).strip()
    if not robot_one_id or not robot_two_id or robot_one_id == robot_two_id:
        print("[COORD] Need two distinct robot ids for coordinated traverse")
        return False

    with clients_lock:
        client_one_id = robots_by_id.get(robot_one_id)
        client_two_id = robots_by_id.get(robot_two_id)
        session_one = client_sessions.get(client_one_id) if client_one_id is not None else None
        session_two = client_sessions.get(client_two_id) if client_two_id is not None else None

        if session_one is None or session_two is None:
            print("[COORD] One or both robots are not connected")
            return False

        start_one = pose_to_cell(session_one.last_telemetry or {})
        start_two = pose_to_cell(session_two.last_telemetry or {})
        if start_one is None or start_two is None:
            print("[COORD] Need telemetry from both robots before planning a coordinated traverse")
            return False

        goal_one = (clamp_cell(int(robot_one["goal_row"])), clamp_cell(int(robot_one["goal_col"])))
        goal_two = (clamp_cell(int(robot_two["goal_row"])), clamp_cell(int(robot_two["goal_col"])))

        clear_robot_sequence(session_one)
        clear_robot_sequence(session_two)

    if start_one == start_two:
        print("[COORD] Robots are in the same grid cell; refusing to plan until they are separated")
        return False

    if goal_one == goal_two:
        print("[COORD] Robots cannot share the same goal cell")
        return False

    path_one = plan_grid_path(start_one, goal_one, {start_two})
    if path_one is None:
        print(f"[COORD] No path found for {robot_one_id} from {start_one} to {goal_one}")
        return False

    path_two = plan_grid_path(start_two, goal_two, {goal_one})
    if path_two is None:
        print(f"[COORD] No path found for {robot_two_id} from {start_two} to {goal_two}")
        return False

    with clients_lock:
        session_one = client_sessions[robots_by_id[robot_one_id]]
        session_two = client_sessions[robots_by_id[robot_two_id]]

        session_one.pending_waypoints = [cell_center_waypoint(cell) for cell in path_one[1:]]
        session_one.pending_motion = None
        session_one.sequence_path_id = next_subpath_id()
        session_one.active_subpath_id = None
        session_one.awaiting_path_ack = False
        session_one.awaiting_path_complete = False

        session_two.pending_waypoints = [cell_center_waypoint(cell) for cell in path_two[1:]]
        session_two.pending_motion = None
        session_two.sequence_path_id = next_subpath_id()
        session_two.active_subpath_id = None
        session_two.awaiting_path_ack = False
        session_two.awaiting_path_complete = False

    print(f"[COORD] {robot_one_id}: {start_one} -> {goal_one} using {len(path_one) - 1} steps")
    print(f"[COORD] {robot_two_id}: {start_two} -> {goal_two} using {len(path_two) - 1} steps")
    maybe_dispatch_waiting_sequences()
    return True


def queue_robot_path(message: dict) -> bool:
    robot_id = message.get("robot_id")
    if not robot_id:
        return False

    waypoints = []
    for waypoint in message.get("waypoints", []):
        x_cm = waypoint.get("x_cm")
        y_cm = waypoint.get("y_cm")
        if x_cm is None or y_cm is None:
            continue
        waypoints.append({
            "x_cm": float(x_cm),
            "y_cm": float(y_cm),
        })

    if not waypoints:
        print(f"[SEQUENCE] Refusing to queue empty path for {robot_id}")
        return False

    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[SEQUENCE] No connected session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            print(f"[SEQUENCE] Session disappeared for robot_id={robot_id}")
            return False

        session.pending_waypoints = waypoints
        session.pending_motion = dict(message["motion"]) if isinstance(message.get("motion"), dict) else None
        session.sequence_path_id = int(message.get("path_id", next_subpath_id()))
        session.active_subpath_id = None
        session.awaiting_path_ack = False
        session.awaiting_path_complete = False

    maybe_dispatch_waiting_sequences()
    return True


def navigate_to_goal(message: dict) -> bool:
    """
    Plan an A* path from the robot's current position to a goal and dispatch
    the entire path as a single path_assignment (for pure-pursuit client).
    """
    robot_id = message.get("robot_id")
    if not robot_id:
        return False

    goal_x = message.get("goal_x_cm")
    goal_y = message.get("goal_y_cm")
    if goal_x is None or goal_y is None:
        print(f"[NAV] Missing goal_x_cm / goal_y_cm in navigate_to_goal")
        return False

    goal_cell = (
        clamp_cell(int(float(goal_y) // GRID_CELL_CM)),
        clamp_cell(int(float(goal_x) // GRID_CELL_CM)),
    )

    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            print(f"[NAV] No session for robot_id={robot_id}")
            return False

        session = client_sessions.get(client_id)
        if session is None:
            return False

        telem = session.last_telemetry or {}
        start_cell = pose_to_cell(telem)
        if start_cell is None:
            print(f"[NAV] No telemetry position for {robot_id} — cannot plan path")
            return False

        # Blocked cells: other robots' current positions + tagged obstacles
        blocked: set[tuple[int, int]] = set()
        for s in client_sessions.values():
            if s.robot_id and s.robot_id != robot_id and s.last_telemetry:
                c = pose_to_cell(s.last_telemetry)
                if c:
                    blocked.add(c)
        for obs in obstacle_tags.values():
            if obs.get("row") is not None and obs.get("col") is not None:
                blocked.add((obs["row"], obs["col"]))

    path_cells = plan_grid_path(start_cell, goal_cell, blocked)
    if path_cells is None:
        print(f"[NAV] No A* path found for {robot_id}: {start_cell} → {goal_cell}")
        return False

    waypoints = [cell_center_waypoint(c) for c in path_cells[1:]]
    if not waypoints:
        print(f"[NAV] {robot_id} is already at the goal cell")
        return True

    path_id = next_subpath_id()
    nav_msg = {
        "type": "path_assignment",
        "robot_id": robot_id,
        "path_id": path_id,
        "replace_existing": True,
        "waypoints": waypoints,
    }
    if isinstance(message.get("motion"), dict):
        nav_msg["motion"] = message["motion"]

    # Store the full path for canvas overlay, then clear on path_complete
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        session = client_sessions.get(client_id) if client_id is not None else None
        if session is not None:
            clear_robot_sequence(session)
            session.nav_path_waypoints = list(waypoints)
            session.current_path_id = str(path_id)
            session.awaiting_path_complete = True

    ok = send_to_robot(robot_id, nav_msg)
    if ok:
        print(f"[NAV] Dispatched A* path to {robot_id}: {len(waypoints)} waypoints "
              f"({start_cell} → {goal_cell})")
    return ok


def set_tag_role(message: dict) -> bool:
    """
    Assign or clear an AprilTag's role (obstacle | goal | none).

    message = {
        "type": "set_tag_role",
        "tag_id": <int>,
        "role": "obstacle" | "goal" | "none",
        "x_cm": <optional float>,
        "y_cm": <optional float>,
    }

    When x_cm / y_cm are omitted the tag is registered without a known
    position (position will be filled in when vision detects it).
    """
    try:
        tag_id = int(message["tag_id"])
    except (KeyError, TypeError, ValueError):
        return False

    role = str(message.get("role", "none"))
    x_cm = message.get("x_cm")
    y_cm = message.get("y_cm")

    row: Optional[int] = None
    col: Optional[int] = None
    if x_cm is not None and y_cm is not None:
        col = clamp_cell(int(float(x_cm) // GRID_CELL_CM))
        row = clamp_cell(int(float(y_cm) // GRID_CELL_CM))

    tag_info = {
        "tag_id": tag_id,
        "role": role,
        "row": row,
        "col": col,
        "x_cm": float(x_cm) if x_cm is not None else None,
        "y_cm": float(y_cm) if y_cm is not None else None,
    }

    with clients_lock:
        obstacle_tags.pop(tag_id, None)
        goal_tags.pop(tag_id, None)
        if role == "obstacle":
            obstacle_tags[tag_id] = tag_info
        elif role == "goal":
            goal_tags[tag_id] = tag_info

    print(f"[TAG] tag_id={tag_id} → role={role!r}")
    return True


def handle_tag_detected(client_id: int, msg: dict) -> None:
    """
    TODO (apriltag-localization merge): handle tag_detected messages from
    vision_localizer.py when it spots a non-robot, non-corner AprilTag.

    Expected message format:
        {
            "type": "tag_detected",
            "tag_id": <int>,
            "x_cm": <float>,
            "y_cm": <float>,
            "theta_deg": <float>,
        }

    Implementation steps:
    1. Look up tag_id in obstacle_tags / goal_tags.
    2. If found, update the stored x_cm / y_cm / row / col with the
       vision-measured position (smoothed over time if desired).
    3. Broadcast the updated snapshot so the canvas re-renders obstacle
       and goal markers at their detected positions.
    4. Optionally: if a goal tag moves, notify any robot navigating to it.
    """
    # TODO: implement after merging apriltag-localization branch
    robot_label = f"client_{client_id}"
    print(f"[{robot_label}] tag_detected (unhandled): {msg}")


# ----------------------------
# GUI command callback
# ----------------------------
def gui_command_sender(message_obj):
    """
    Called by GUI button handlers and the web dashboard POST /command endpoint.
    message_obj is a dict or a dataclass message object from messages.py.
    """
    try:
        message = message_obj if isinstance(message_obj, dict) else message_obj.to_dict()
        msg_type = message.get("type")
        robot_id = message.get("robot_id")

        if msg_type == "coordinated_traverse":
            ok = start_coordinated_traverse(message)
            if ok:
                print("[GUI SEND] Started coordinated two-robot traverse")
            else:
                print("[GUI SEND] Failed to start coordinated two-robot traverse")
            return

        if msg_type == "navigate_to_goal":
            ok = navigate_to_goal(message)
            if ok:
                print(f"[GUI SEND] A* navigate dispatched for {robot_id}")
            else:
                print(f"[GUI SEND] A* navigate failed for {robot_id}")
            return

        if msg_type == "set_tag_role":
            ok = set_tag_role(message)
            if ok:
                print(f"[GUI SEND] Tag role updated: tag_id={message.get('tag_id')} role={message.get('role')}")
            else:
                print("[GUI SEND] set_tag_role failed")
            return

        if not robot_id:
            print("[GUI SEND] Refusing to send message with no robot_id")
            return

        if msg_type == "path_assignment":
            ok = queue_robot_path(message)
        else:
            if msg_type == "stop":
                with clients_lock:
                    client_id = robots_by_id.get(robot_id)
                    session = client_sessions.get(client_id) if client_id is not None else None
                    if session is not None:
                        clear_robot_sequence(session)
                        session.nav_path_waypoints.clear()

            if msg_type == "toggle_gripper":
                with clients_lock:
                    client_id = robots_by_id.get(robot_id)
                    session = client_sessions.get(client_id) if client_id is not None else None
                    if session is not None:
                        session.gripper_open = not session.gripper_open

            ok = send_to_robot(robot_id, message)

        if ok:
            print(f"[GUI SEND] Sent {msg_type} to {robot_id}")
        else:
            print(f"[GUI SEND] Failed to send {msg_type} to {robot_id}")

    except Exception as exc:
        print(f"[GUI SEND] Error building/sending message: {exc}")


class _TeeWriter(io.TextIOBase):
    """
    Wraps sys.stdout so every print() in the arbiter is forwarded to the
    dashboard log panel in addition to the normal terminal output.
    """

    def __init__(self, original: io.TextIOBase, log_fn) -> None:
        self._orig   = original
        self._log_fn = log_fn
        self._buf    = ""
        self._lock   = threading.Lock()

    def write(self, s: str) -> int:
        with self._lock:
            self._orig.write(s)
            self._orig.flush()
            self._buf += s
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                if line.strip():
                    self._log_fn("arbiter", line)
        return len(s)

    def flush(self) -> None:
        self._orig.flush()


# Initialize GUI
gui = TelemetryGUI(command_sender=gui_command_sender)

# Initialize web dashboard (always starts, regardless of --no-gui)
dashboard = DashboardServer(
    snapshot_fn=get_robot_snapshot,
    command_callback=gui_command_sender,
)
dashboard.start()

# Intercept arbiter stdout → forward every print() to the dashboard log panel
sys.stdout = _TeeWriter(sys.stdout, dashboard.log)


# ----------------------------
# Session / identity helpers
# ----------------------------
def touch_session(client_id: int) -> None:
    with clients_lock:
        session = client_sessions.get(client_id)
        if session:
            session.last_heartbeat = time.time()


def bind_identity_from_message(client_id: int, msg: dict) -> None:
    with clients_lock:
        session = client_sessions[client_id]

        if "name" in msg:
            session.name = str(msg["name"])
        elif "client_name" in msg:
            session.name = str(msg["client_name"])

        if "robot_id" in msg and msg["robot_id"] is not None:
            robot_id = str(msg["robot_id"])
            session.robot_id = robot_id
            robots_by_id[robot_id] = client_id

        if "state" in msg and msg["state"] is not None:
            session.state = str(msg["state"])

        if "path_id" in msg and msg["path_id"] is not None:
            session.current_path_id = str(msg["path_id"])

        if "waypoint_index" in msg and msg["waypoint_index"] is not None:
            try:
                session.current_waypoint_index = int(msg["waypoint_index"])
            except (TypeError, ValueError):
                pass


# ----------------------------
# Message handlers
# ----------------------------
def handle_hello(client_id: int, msg: dict, conn: socket.socket) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    session = get_session(client_id)
    print(
        f"[Client {client_id}] registered "
        f"name={session.name!r}, robot_id={session.robot_id!r}, state={session.state!r}"
    )

    send_json(conn, {
        "type": "ack",
        "for": "hello",
        "client_id": client_id,
        "robot_id": session.robot_id,
    })

    print_robot_table()


def handle_telemetry(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions[client_id]
        session.last_telemetry = msg

    if session.robot_id:
        gui.update_robot(session.robot_id, msg)
        dashboard.notify(session.robot_id, msg)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] telemetry: {msg}")


def handle_status(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions[client_id]
        session.last_status = msg

        # Merge status into GUI-visible state if we have prior telemetry
        merged = dict(session.last_telemetry or {})
        merged.update(msg)

    if session.robot_id:
        gui.update_robot(session.robot_id, merged)
        dashboard.notify(session.robot_id, merged)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] status: {msg}")


def handle_path_event(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions[client_id]

        # Path lifecycle messages should be visible in the GUI the same way
        # status updates are, while preserving the most recent telemetry pose.
        merged = dict(session.last_telemetry or {})
        merged.update(msg)

        event_type = str(msg.get("type", ""))
        if event_type == "path_started":
            merged["state"] = "executing_path"
            session.state = "executing_path"
        elif event_type == "waypoint_reached":
            merged["state"] = "waypoint_reached"
            session.state = "waypoint_reached"
        elif event_type == "path_complete":
            merged["state"] = "idle"
            session.state = "idle"
            session.current_waypoint_index = None
            session.current_path_id = None
            session.awaiting_path_complete = False
            session.active_subpath_id = None
            session.nav_path_waypoints.clear()

        session.last_status = merged

    if session.robot_id:
        gui.update_robot(session.robot_id, merged)
        dashboard.notify(session.robot_id, merged)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] {msg.get('type')}: {msg}")

    if msg.get("type") == "path_complete":
        if session.robot_id:
            dispatch_next_waypoint(session.robot_id)
        maybe_dispatch_waiting_sequences()


def handle_ack(client_id: int, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    with clients_lock:
        session = client_sessions.get(client_id)
        if session is not None and msg.get("for") == "path_assignment":
            ack_path_id = msg.get("path_id")
            if ack_path_id is None or session.active_subpath_id is None:
                session.awaiting_path_ack = False
            else:
                try:
                    if int(ack_path_id) == session.active_subpath_id:
                        session.awaiting_path_ack = False
                except (TypeError, ValueError):
                    session.awaiting_path_ack = False

    robot_label = session.robot_id if session and session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] ack: {msg}")


def handle_heartbeat(client_id: int, conn: socket.socket, msg: dict) -> None:
    touch_session(client_id)
    bind_identity_from_message(client_id, msg)

    session = get_session(client_id)
    send_json(conn, {
        "type": "heartbeat_ack",
        "robot_id": session.robot_id if session else None,
        "server_t": time.time(),
    })


# ----------------------------
# Client thread
# ----------------------------
def handle_client(client_id: int, conn: socket.socket, addr) -> None:
    print(f"[+] Client {client_id} connected from {addr}")

    try:
        send_json(conn, {
            "type": "hello_ack",
            "client_id": client_id,
            "message": "connected",
        })

        for line in recv_lines(conn):
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print(f"[Client {client_id}] Bad JSON: {line}")
                continue

            msg_type = msg.get("type", "")

            if msg_type == "hello":
                handle_hello(client_id, msg, conn)

            elif msg_type == "telemetry":
                handle_telemetry(client_id, msg)

            elif msg_type == "heartbeat":
                handle_heartbeat(client_id, conn, msg)

            elif msg_type == "status":
                handle_status(client_id, msg)

            elif msg_type in {"path_started", "waypoint_reached", "path_complete"}:
                handle_path_event(client_id, msg)

            elif msg_type == "ack":
                handle_ack(client_id, msg)

            elif msg_type == "tag_detected":
                # TODO: implement handle_tag_detected after merging apriltag-localization
                handle_tag_detected(client_id, msg)

            else:
                print(f"[Client {client_id}] unknown message type: {msg_type}")
                send_json(conn, {
                    "type": "error",
                    "message": f"unknown message type: {msg_type}",
                })

    except ConnectionResetError:
        print(f"[!] Client {client_id} connection reset")
    except Exception as exc:
        print(f"[!] Client {client_id} error: {exc}")
    finally:
        with clients_lock:
            session = client_sessions.pop(client_id, None)
            if session and session.robot_id:
                mapped_client_id = robots_by_id.get(session.robot_id)
                if mapped_client_id == client_id:
                    robots_by_id.pop(session.robot_id, None)

        conn.close()
        print(f"[-] Client {client_id} disconnected")
        print_robot_table()


# ----------------------------
# Accept loop
# ----------------------------
def accept_loop(server_sock: socket.socket) -> None:
    global next_client_id

    while True:
        conn, addr = server_sock.accept()

        with clients_lock:
            if len(client_sessions) >= MAX_CLIENTS:
                send_json(conn, {
                    "type": "error",
                    "message": "server full",
                })
                conn.close()
                continue

            client_id = next_client_id
            next_client_id += 1

            client_sessions[client_id] = ClientSession(
                client_id=client_id,
                conn=conn,
                addr=addr,
            )

        thread = threading.Thread(
            target=handle_client,
            args=(client_id, conn, addr),
            daemon=True,
        )
        thread.start()
        print_robot_table()


# ----------------------------
# Server main
# ----------------------------
def server_main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen()
        print(f"Server listening on {HOST}:{PORT}")

        accept_loop(server_sock)


def main() -> None:
    server_thread = threading.Thread(target=server_main, daemon=True)
    server_thread.start()

    gui.run()


if __name__ == "__main__":
    main()
