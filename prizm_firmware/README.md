# Robot Firmware Tutorial

This folder contains three Arduino sketches for PRIZM-based robot testing.

- `testing-bot-controls.ino` is a minimal manual-control sketch.
- `telemetry_and_communicate_to_aribiter.ino` is the full telemetry and waypoint sketch used with the Python arbiter.
- `wireless-bot-controls.ino` is the the full telemetry and waypoint sketch that can be used wirelessly with the Python arbiter when using a Robot with an ESP32 attached.

## Simple Manual-Control Sketch

`testing-bot-controls.ino` is useful before running the full fleet stack. It checks that the motors, serial connection, and gripper servo are wired correctly.

Serial commands:

- `w` drives forward.
- `s` drives backward.
- `a` turns left.
- `d` turns right.
- `1` opens the gripper.
- `2` closes the gripper.
- Any other command stops the motors.

The sketch remembers the most recent command for `50 ms`. If no new command arrives within that timeout, it stops both motors. That timeout acts as a simple dead-man switch so the robot does not keep moving forever after the controller stops sending commands.

Motor direction is encoded in the helper functions:

- `setMotorsForward`
- `setMotorsBackward`
- `setMotorsLeft`
- `setMotorsRight`

If the robot moves backward when you expect forward motion, adjust the signs in those helpers rather than changing the rest of the sketch.

## Full Telemetry And Waypoint Sketch

The full sketch is designed to communicate with `client.py` over serial at `115200` baud.

It does five jobs in its main loop:

1. Read serial commands.
2. Update odometry while motors are moving.
3. Start the next turn or drive primitive for the active waypoint.
4. Refresh ultrasonic sensor readings when serial input is quiet.
5. Print periodic telemetry JSON.

The sketch reads serial multiple times per loop so pause, resume, and stop commands can be handled quickly.

## Robot Identity

At the top of the full sketch:

```cpp
const char* ROBOT_ID = "robot_A";
```

This must match the `ROBOT_ID` configured in the robot laptop's `.env` file. If they do not match, the robot may ignore commands because `jsonTargetsThisRobot` filters commands by robot ID.

## Pose And Odometry

The pose variables are:

```cpp
float x_cm = 200.0;
float y_cm = 100.0;
float theta_rad = PI / 2.0;
```

These are the robot's believed starting position and heading. The code updates them from motor encoder deltas in `updatePoseFromEncoders`.

The most important calibration constants are:

- `WHEEL_DIAMETER_CM`
- `WHEEL_BASE_CM`
- `WHEEL_CIRCUMFERENCE_CM`

If distance or turning is inaccurate, tune these constants and verify motor directions before changing the path logic.

## Path Storage

The full sketch stores paths in fixed-size arrays:

```cpp
const int MAX_WAYPOINTS = 12;
int16_t waypoint_xs[MAX_WAYPOINTS];
int16_t waypoint_ys[MAX_WAYPOINTS];
```

This avoids dynamic memory allocation on the microcontroller. The tradeoff is that incoming paths are limited to `12` waypoints and coordinates are rounded to integers.

The current Python arbiter usually sends one waypoint at a time during sequencing, so this limit is not a problem for the provided workflow.

## Motion State Machine

The sketch breaks path execution into primitive actions:

- `PRIM_TURN` rotates the robot toward the next waypoint.
- `PRIM_DRIVE` drives straight to the waypoint.
- `PRIM_NONE` means no primitive is currently active.

`maybeStartNextPrimitive` decides what to do next:

1. If no path is loaded, do nothing.
2. If paused, do nothing.
3. If already turning or driving, do nothing.
4. If the path has not started yet, send `path_started`.
5. If all waypoints are done, send `path_complete`.
6. If the robot is already close enough to the waypoint, send `waypoint_reached`.
7. If heading error is too large, start a turn.
8. Otherwise, start a straight drive.

`updateActivePrimitive` watches the PRIZM motor-busy flags. When a drive primitive finishes, it sends `waypoint_reached` and advances the waypoint index.

## Command Handling

The full sketch accepts both compact control opcodes and JSON commands.

Compact opcodes:

- `P` pauses.
- `R` resumes.
- `S` stops and clears the active path.

JSON commands:

- `path_assignment`
- `pause`
- `resume`
- `stop`
- `toggle_gripper`

The JSON parsing helpers are intentionally small and simple. They search for specific field names and parse only the values the firmware needs. This keeps dependencies low, but it also means Arduino-facing JSON should stay simple and predictable.

## Telemetry Output

`printPoseJSON` prints the telemetry object read by `client.py`:

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

Telemetry is sent every `250 ms` by default. Ultrasonic readings are refreshed every `500 ms`, but only when serial input is quiet so command processing stays responsive.

## Safe Places To Modify

- Change `ROBOT_ID` per robot.
- Tune wheel and servo constants near the top of the full sketch.
- Adjust `POSITION_TOLERANCE_CM` and `HEADING_TOLERANCE_DEG` to make waypoint completion stricter or more forgiving.
- Increase or decrease `TELEMETRY_PERIOD_MS` if the GUI needs faster or slower updates.
- Add new command handlers near `handleIncomingJson`.

After changing motion behavior, test with the simple sketch first, then run a one-waypoint path from the GUI before trying coordinated traversal.
