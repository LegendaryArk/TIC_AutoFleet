import json
from collections import deque
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from gui import TelemetryGUI

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


clients_lock = threading.Lock()

# keyed by client_id
client_sessions: Dict[int, ClientSession] = {}

# keyed by robot_id
robots_by_id: Dict[str, int] = {}

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
        snapshot = {}
        for robot_id, client_id in robots_by_id.items():
            session = client_sessions.get(client_id)
            if session is None:
                continue

            snapshot[robot_id] = {
                "client_id": session.client_id,
                "name": session.name,
                "state": session.state,
                "path_id": session.current_path_id,
                "waypoint_index": session.current_waypoint_index,
                "last_heartbeat": session.last_heartbeat,
                "last_telemetry": session.last_telemetry,
                "last_status": session.last_status,
            }
        return snapshot


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

    queue = deque([start])
    came_from = {start: None}

    while queue:
        row, col = queue.popleft()
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_cell = (row + d_row, col + d_col)
            next_row, next_col = next_cell
            if not (0 <= next_row < GRID_DIM_CELLS and 0 <= next_col < GRID_DIM_CELLS):
                continue
            if next_cell in blocked and next_cell != goal:
                continue
            if next_cell in came_from:
                continue

            came_from[next_cell] = (row, col)
            if next_cell == goal:
                path = [goal]
                current = (row, col)
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            queue.append(next_cell)

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


# ----------------------------
# GUI command callback
# ----------------------------
def gui_command_sender(message_obj):
    """
    Called by GUI button handlers.
    message_obj is one of your dataclass message objects from messages.py.
    """
    try:
        message = message_obj if isinstance(message_obj, dict) else message_obj.to_dict()
        robot_id = message.get("robot_id")

        if message.get("type") == "coordinated_traverse":
            ok = start_coordinated_traverse(message)
            if ok:
                print("[GUI SEND] Started coordinated two-robot traverse")
            else:
                print("[GUI SEND] Failed to start coordinated two-robot traverse")
            return

        if not robot_id:
            print("[GUI SEND] Refusing to send message with no robot_id")
            return

        if message.get("type") == "path_assignment":
            ok = queue_robot_path(message)
        else:
            if message.get("type") == "stop":
                with clients_lock:
                    client_id = robots_by_id.get(robot_id)
                    session = client_sessions.get(client_id) if client_id is not None else None
                    if session is not None:
                        clear_robot_sequence(session)

            ok = send_to_robot(robot_id, message)

        if ok:
            print(f"[GUI SEND] Sent {message.get('type')} to {robot_id}")
        else:
            print(f"[GUI SEND] Failed to send {message.get('type')} to {robot_id}")

    except Exception as exc:
        print(f"[GUI SEND] Error building/sending message: {exc}")


# Initialize GUI
gui = TelemetryGUI(command_sender=gui_command_sender)


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

        session.last_status = merged

    if session.robot_id:
        gui.update_robot(session.robot_id, merged)

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
