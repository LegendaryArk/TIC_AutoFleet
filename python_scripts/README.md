# Python Starter Code Tutorial

The Python side has three jobs:

1. Define the protocol messages.
2. Bridge robot serial communication to the TCP arbiter.
3. Run the central fleet arbiter and operator GUI.

## Files

- `messages.py` contains dataclasses for the shared message format.
- `client.py` runs beside a robot and forwards messages between TCP and serial.
- `central-arbiter.py` runs on the operator computer and coordinates connected robots.
- `gui.py` is the visual dashboard and command panel.
- `requirements.txt` lists the packages used by the Python tools.

## `client.py`: Robot Laptop Bridge

`client.py` is the connector between the PRIZM robot and the central arbiter.

Configuration comes from the `.env` file:

- `SERVER_HOST_IP_ADDRESS` is the operator computer running `central-arbiter.py`.
- `SERVER_PORT` defaults to `9000`.
- `LOCAL_SERIAL_PORT` is the local serial device connected to the robot.
- `SERIAL_BAUD` defaults to `115200`.
- `ROBOT_ID` is the robot identity that appears in the GUI (Make sure this matches the firmware in the PRIZM Robot)
- `CLIENT_NAME` is an optional human-readable laptop name.

To connect the a robot (lets say Robot_A) to the Central Arbiter Server via the client, run: `python client.py --env ../robot_a.env` where `robot_a.env` contains the appropriate values to connect to the robot.

The main flow is:

1.  Open the serial port.
2.  Connect to the arbiter TCP server.
3.  Send a `hello` message.
4.  Start a background thread for messages from the arbiter.
5.  Start a heartbeat thread.
6.  Keep reading serial telemetry and forwarding it to the arbiter.

Two helper functions are important when changing the protocol:

- `send_json_line_over_socket` writes compact newline-delimited JSON to TCP.
- `send_json_line_over_serial` writes either JSON or a compact single-letter command to serial.

Pause, resume, and stop are encoded as `P`, `R`, and `S` on serial. This makes urgent control commands short and allows the robot sketch to handle them with minimal parsing.

## `central-arbiter.py`: Fleet Server

`central-arbiter.py` owns the connected-client table and is the only process that talks directly to all robot laptops.

Key state:

- `client_sessions` maps arbiter-assigned `client_id` values to `ClientSession` records.
- `robots_by_id` maps each `robot_id` to the active `client_id`.
- Each `ClientSession` stores the latest telemetry, latest status, path state, and sequencing flags.

The server starts in `main`:

1. A background thread runs `server_main`.
2. `server_main` opens a TCP socket on `0.0.0.0:9000`.
3. `accept_loop` creates one handler thread per client.
4. The main thread runs the GUI event loop.

Incoming messages are routed in `handle_client`:

- `hello` registers the robot identity.
- `telemetry` updates the session and GUI.
- `status` updates robot state without replacing the latest pose.
- `path_started`, `waypoint_reached`, and `path_complete` update path lifecycle state.
- `ack` clears acknowledgement wait flags for queued path assignments.
- `heartbeat` refreshes connection health and receives a `heartbeat_ack`.

## Path Sequencing

The arbiter deliberately sends one waypoint at a time to a robot during queued paths. This is handled by:

- `queue_robot_path`
- `dispatch_next_waypoint`
- `handle_path_event`
- `maybe_dispatch_waiting_sequences`

When a GUI test path contains several waypoints, `queue_robot_path` stores them in `pending_waypoints`. `dispatch_next_waypoint` pops the next waypoint and sends it as a one-waypoint `path_assignment`. When the robot later emits `path_complete`, the arbiter dispatches the next waypoint.

This pattern is easy to understand and debug, but it is not a full traffic-management system. It is a good place to add reservation logic, priorities, collision checks, or retry handling.

## Coordinated Two-Robot Traverse

The GUI can request a `coordinated_traverse` dictionary instead of one of the dataclass message types.

`start_coordinated_traverse`:

1. Requires exactly two distinct robot IDs.
2. Reads each robot's latest telemetry.
3. Converts each pose into a grid cell.
4. Validates the two goal cells.
5. Runs `plan_grid_path` for each robot.
6. Stores each path as pending one-cell waypoints.
7. Starts dispatching one robot at a time.

`plan_grid_path` is a breadth-first search over a `40 x 40` grid. It moves up, down, left, or right by one cell and avoids blocked cells supplied by the caller.

This is intentionally simple. A stronger solution might add dynamic reservations, predicted travel times, obstacle updates from sensors, or simultaneous movement when paths are safely separated.

## `gui.py`: Operator Interface

`TelemetryGUI` owns the dashboard window. It receives updates through `update_robot`, which pushes telemetry into a thread-safe queue. The Tkinter event loop periodically drains that queue in `_process_queue`.

The GUI has four major responsibilities:

- Display a table of robot ID, state, path ID, waypoint index, pose, and ultrasonic readings.
- Plot robot positions, heading triangles, safety boxes, ultrasonic rays, and recent echo points.
- Send manual commands such as pause, resume, stop, and gripper toggle.
- Build test-path and coordinated-traverse command messages.

When adding buttons, follow the existing pattern:

1. Add the control in `_build_layout`.
2. Implement a `_send_*` method that builds a message.
3. Call `self.command_sender(message)` if a sender was provided.
4. Add any needed message type support in the arbiter, client, and firmware.

## Common Debugging Checks

- If the client prints `Connection refused`, start the arbiter first or check `SERVER_HOST_IP_ADDRESS`.
- If the client prints a serial error, verify `LOCAL_SERIAL_PORT`, USB connection, and baud rate.
- If telemetry reaches the client but not the GUI, inspect the TCP logs in `client.py` and `central-arbiter.py`.
- If commands reach the client but the robot ignores them, check that `robot_id` in `.env` matches `ROBOT_ID` in the Arduino sketch.
- if commands reach the client but the robot ignores them, on ESP32 enabled PRIZM bots you can connect to the ESP32 and read the Serial Monitor logs to debug.
- If path motion looks wrong, recalibrate wheel diameter, wheel base, motor directions, and starting pose in the Arduino sketch.
