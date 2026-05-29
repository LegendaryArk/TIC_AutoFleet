from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Literal, Optional


MessageType = Literal[
    "hello",
    "telemetry",
    "ack",
    "path_assignment",
    "path_started",
    "waypoint_reached",
    "path_complete",
    "status",
    "stop",
    "pause",
    "resume",
    "toggle_gripper",
    "heartbeat",
    "heartbeat_ack",
]


@dataclass
class Message:
    type: MessageType

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


@dataclass
class HelloMessage(Message):
    robot_id: str
    client_name: Optional[str] = None
    state: Optional[str] = None

    def __init__(self, robot_id: str, client_name: Optional[str] = None, state: Optional[str] = None):
        super().__init__("hello")
        self.robot_id = robot_id
        self.client_name = client_name
        self.state = state


@dataclass
class TelemetryMessage(Message):
    robot_id: str
    state: str
    path_id: int
    waypoint_index: int
    t_ms: int
    x_cm: float
    y_cm: float
    theta_deg: float
    front_ultrasonic_cm: Optional[float] = None
    left_ultrasonic_cm: Optional[float] = None

    def __init__(
        self,
        robot_id: str,
        state: str,
        path_id: int,
        waypoint_index: int,
        t_ms: int,
        x_cm: float,
        y_cm: float,
        theta_deg: float,
        front_ultrasonic_cm: Optional[float] = None,
        left_ultrasonic_cm: Optional[float] = None,
    ):
        super().__init__("telemetry")
        self.robot_id = robot_id
        self.state = state
        self.path_id = path_id
        self.waypoint_index = waypoint_index
        self.t_ms = t_ms
        self.x_cm = x_cm
        self.y_cm = y_cm
        self.theta_deg = theta_deg
        self.front_ultrasonic_cm = front_ultrasonic_cm
        self.left_ultrasonic_cm = left_ultrasonic_cm


@dataclass
class AckMessage(Message):
    robot_id: str
    for_type: str
    path_id: Optional[int] = None
    t_ms: Optional[int] = None

    def __init__(self, robot_id: str, for_type: str, path_id: Optional[int] = None, t_ms: Optional[int] = None):
        super().__init__("ack")
        self.robot_id = robot_id
        self.for_type = for_type
        self.path_id = path_id
        self.t_ms = t_ms

    def to_dict(self) -> dict:
        d = super().to_dict()
        d["for"] = d.pop("for_type")
        return d


@dataclass
class Waypoint:
    x_cm: float
    y_cm: float


@dataclass
class MotionSettings:
    turn_speed_deg_per_sec: int
    drive_speed_deg_per_sec: int


@dataclass
class PathAssignmentMessage(Message):
    robot_id: str
    path_id: int
    replace_existing: bool
    waypoints: list[Waypoint]
    motion: Optional[MotionSettings] = None

    def __init__(
        self,
        robot_id: str,
        path_id: int,
        waypoints: list[Waypoint],
        replace_existing: bool = True,
        motion: Optional[MotionSettings] = None,
    ):
        super().__init__("path_assignment")
        self.robot_id = robot_id
        self.path_id = path_id
        self.replace_existing = replace_existing
        self.waypoints = waypoints
        self.motion = motion


@dataclass
class PathStartedMessage(Message):
    robot_id: str
    path_id: int
    t_ms: int

    def __init__(self, robot_id: str, path_id: int, t_ms: int):
        super().__init__("path_started")
        self.robot_id = robot_id
        self.path_id = path_id
        self.t_ms = t_ms


@dataclass
class WaypointReachedMessage(Message):
    robot_id: str
    path_id: int
    waypoint_index: int
    t_ms: int
    x_cm: Optional[float] = None
    y_cm: Optional[float] = None
    theta_deg: Optional[float] = None

    def __init__(
        self,
        robot_id: str,
        path_id: int,
        waypoint_index: int,
        t_ms: int,
        x_cm: Optional[float] = None,
        y_cm: Optional[float] = None,
        theta_deg: Optional[float] = None,
    ):
        super().__init__("waypoint_reached")
        self.robot_id = robot_id
        self.path_id = path_id
        self.waypoint_index = waypoint_index
        self.t_ms = t_ms
        self.x_cm = x_cm
        self.y_cm = y_cm
        self.theta_deg = theta_deg


@dataclass
class PathCompleteMessage(Message):
    robot_id: str
    path_id: int
    t_ms: int
    x_cm: Optional[float] = None
    y_cm: Optional[float] = None
    theta_deg: Optional[float] = None

    def __init__(
        self,
        robot_id: str,
        path_id: int,
        t_ms: int,
        x_cm: Optional[float] = None,
        y_cm: Optional[float] = None,
        theta_deg: Optional[float] = None,
    ):
        super().__init__("path_complete")
        self.robot_id = robot_id
        self.path_id = path_id
        self.t_ms = t_ms
        self.x_cm = x_cm
        self.y_cm = y_cm
        self.theta_deg = theta_deg


@dataclass
class StatusMessage(Message):
    robot_id: str
    state: str
    path_id: Optional[int] = None
    waypoint_index: Optional[int] = None
    reason: Optional[str] = None
    t_ms: Optional[int] = None

    def __init__(
        self,
        robot_id: str,
        state: str,
        path_id: Optional[int] = None,
        waypoint_index: Optional[int] = None,
        reason: Optional[str] = None,
        t_ms: Optional[int] = None,
    ):
        super().__init__("status")
        self.robot_id = robot_id
        self.state = state
        self.path_id = path_id
        self.waypoint_index = waypoint_index
        self.reason = reason
        self.t_ms = t_ms


@dataclass
class StopMessage(Message):
    robot_id: str
    reason: Optional[str] = None

    def __init__(self, robot_id: str, reason: Optional[str] = None):
        super().__init__("stop")
        self.robot_id = robot_id
        self.reason = reason


@dataclass
class PauseMessage(Message):
    robot_id: str
    reason: Optional[str] = None

    def __init__(self, robot_id: str, reason: Optional[str] = None):
        super().__init__("pause")
        self.robot_id = robot_id
        self.reason = reason


@dataclass
class ResumeMessage(Message):
    robot_id: str

    def __init__(self, robot_id: str):
        super().__init__("resume")
        self.robot_id = robot_id


@dataclass
class ToggleGripperMessage(Message):
    robot_id: str

    def __init__(self, robot_id: str):
        super().__init__("toggle_gripper")
        self.robot_id = robot_id


@dataclass
class HeartbeatMessage(Message):
    robot_id: str
    t_ms: Optional[int] = None
    state: Optional[str] = None

    def __init__(self, robot_id: str, t_ms: Optional[int] = None, state: Optional[str] = None):
        super().__init__("heartbeat")
        self.robot_id = robot_id
        self.t_ms = t_ms
        self.state = state


@dataclass
class HeartbeatAckMessage(Message):
    robot_id: str
    server_t: float

    def __init__(self, robot_id: str, server_t: float):
        super().__init__("heartbeat_ack")
        self.robot_id = robot_id
        self.server_t = server_t
