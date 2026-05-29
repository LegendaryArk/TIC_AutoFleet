# Message Protocol Tutorial

The fleet starter code uses newline-delimited JSON. Each message is one JSON object followed by `\n`.

Example:

```json
{"type":"pause","robot_id":"robot_A","reason":"operator_requested"}
```

The newline matters because both the socket code and serial code read complete lines.

## Python Message Classes

`messages.py` defines dataclasses for the common messages used by the GUI and arbiter. Each class inherits from `Message` and can be serialized with `to_dict` or `to_json`.

Example:

```python
from messages import PathAssignmentMessage, Waypoint

message = PathAssignmentMessage(
    robot_id="robot_A",
    path_id=1001,
    waypoints=[Waypoint(x_cm=100, y_cm=120)],
)

payload = message.to_dict()
```

The code does not require every message to be created from a dataclass. `central-arbiter.py` also accepts plain dictionaries for internal commands such as `coordinated_traverse`.

## Core Message Types

### `hello`

Sent from `client.py` to the arbiter after the TCP connection opens.

```json
{"type":"hello","robot_id":"robot_A","client_name":"robot_A_laptop","state":"ready"}
```

The arbiter uses this to bind a TCP client session to a robot ID.

### `telemetry`

Sent from the robot sketch through the client to the arbiter.

```json
{
  "type": "telemetry",
  "robot_id": "robot_A",
  "state": "executing_path",
  "path_id": 1002,
  "waypoint_index": 0,
  "t_ms": 48125,
  "x_cm": 205.4,
  "y_cm": 130.1,
  "theta_deg": 88.7,
  "front_ultrasonic_cm": 42,
  "left_ultrasonic_cm": 97
}
```

The GUI expects coordinates in centimeters and heading in degrees.

### `path_assignment`

Sent from the arbiter to a robot to load a path.

```json
{
  "type": "path_assignment",
  "robot_id": "robot_A",
  "path_id": 1002,
  "replace_existing": true,
  "waypoints": [
    {"x_cm": 210, "y_cm": 130},
    {"x_cm": 240, "y_cm": 130}
  ],
  "motion": {
    "turn_speed_deg_per_sec": 150,
    "drive_speed_deg_per_sec": 200
  }
}
```

`client.py` compacts this before sending it over serial. Waypoint coordinates are rounded to integers because the Arduino sketch stores them in `int16_t` arrays.

### `ack`

Sent by the robot when it accepts a command.

```json
{"type":"ack","robot_id":"robot_A","for":"path_assignment","path_id":1002,"t_ms":48200}
```

In Python, the dataclass uses `for_type` because `for` is a reserved keyword. `AckMessage.to_dict` converts `for_type` back to the JSON field named `for`.

### Path Lifecycle Messages

The robot sketch emits these while executing a path:

- `path_started` when it begins a loaded path.
- `waypoint_reached` when a waypoint is reached.
- `path_complete` when all waypoints are done.

The arbiter uses `path_complete` to dispatch the next queued waypoint.

### Control Messages

The GUI and arbiter can send:

- `pause`
- `resume`
- `stop`
- `toggle_gripper`

For serial delivery, `client.py` encodes pause, resume, and stop as:

- `P\n`
- `R\n`
- `S\n`

The full JSON form is still used over TCP.

### `heartbeat` and `heartbeat_ack`

`client.py` sends a heartbeat every two seconds so the arbiter can refresh connection state.

```json
{"type":"heartbeat","robot_id":"robot_A","state":"connected","t_ms":1760000000000}
```

The arbiter replies with:

```json
{"type":"heartbeat_ack","robot_id":"robot_A","server_t":1760000000.0}
```

## Adding A New Message Type

Use this checklist when adding a message:

1. Add the string literal to `MessageType` in `messages.py`.
2. Add a dataclass if the GUI or Python code will construct the message often.
3. Teach `central-arbiter.py` how to route or handle the new type.
4. Add the type to `FORWARD_TO_ROBOT_TYPES` in `client.py` if the robot firmware should receive it.
5. Add parsing and behavior in the Arduino sketch.
6. Add an acknowledgement or status response if operators need feedback.

Keep new messages line-delimited and avoid deeply nested JSON for Arduino-facing commands. The current firmware uses small manual parsing helpers rather than a full JSON library.

