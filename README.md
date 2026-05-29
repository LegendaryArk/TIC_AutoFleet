# Autonomous Fleets

## Challenge Description

Modern factories and warehouses increasingly rely on fleets of mobile robots to move material, support assembly, and reduce wasted motion. The challenge in this track is not just making one robot move, but coordinating multiple robots safely and predictably in a shared workspace.

Students are invited to build autonomy, coordination, and fleet-management features for a small robotics testbed. Solutions can focus on robot-to-robot coordination, task allocation, path planning, operator tooling, safety behaviors, or telemetry-driven monitoring.

## Potential Solutions

- Multi-robot path planning and collision avoidance
- Fleet dispatching or centralized task arbitration
- Telemetry dashboards for operators and judges
- Safety logic using ultrasonic or other onboard sensors
- Smarter pickup, drop-off, or gripper control workflows
- Reliability improvements for command handling and robot recovery

## Recommended Roadmap

Teams are encouraged to take the project in any direction. These milestones are not requirements or a scoring checklist; they are meant to give less experienced teams a practical path from "the robot moves" to "we built a fleet behavior." Advanced teams can skip, combine, or replace them with their own plan.

<img src="/assets/fleet-rr.png" width="50%">

### Milestone 1: Make One Robot Reliable

Goal: prove that one robot can be controlled predictably.

Suggested outcomes:

- Upload a working Arduino sketch to one robot
- Confirm forward, backward, left, right, stop, and gripper commands
- Tune motor directions, speed values, and servo positions
- Establish a safe emergency stop or timeout behavior

Good demo: one robot can be manually controlled without drifting into unsafe behavior when commands stop.

### Milestone 2: Connect Robot Telemetry

Goal: make the robot visible to the software stack.

Suggested outcomes:

- Start the central arbiter and robot client
- Send telemetry from the robot to the dashboard
- Display robot ID, state, pose, path progress, and sensor readings
- Verify that pause, resume, stop, and gripper commands travel from the GUI to the robot

Good demo: the dashboard updates while the robot moves, and GUI commands affect the physical robot.

### Milestone 3: Execute A Simple Autonomous Path

Goal: move from manual control to waypoint-based behavior.

Suggested outcomes:

- Send a straight-line path from the GUI
- Send a turn-around or L-shaped path
- Tune odometry constants such as wheel diameter and wheel base
- Report path lifecycle events such as started, waypoint reached, and complete

Good demo: one robot can drive to one or more target points and report progress back to the arbiter.

### Milestone 4: Add Safety And Recovery

Goal: make autonomy more robust when something goes wrong.

Suggested outcomes:

- Use ultrasonic or other sensor data to detect nearby obstacles
- Pause, slow, reroute, or stop when the robot sees a hazard
- Add recovery behavior after a failed command, blocked path, or lost connection
- Make robot state understandable to operators through status messages or dashboard indicators

Good demo: the robot reacts safely to an obstacle or failure instead of blindly continuing.

### Milestone 5: Coordinate Multiple Robots

Goal: demonstrate fleet behavior instead of independent single-robot control.

Suggested outcomes:

- Connect two or more robots with unique robot IDs
- Prevent two robots from being assigned the same space at the same time
- Add centralized task assignment, traffic rules, reservations, or route planning
- Show each robot's current task and progress in the dashboard

Good demo: two robots complete compatible tasks without colliding or blocking each other unnecessarily.

### Milestone 6: Build Your Differentiator

Goal: turn the starter system into your team's own solution.

Possible directions:

- Smarter dispatching: assign jobs based on distance, battery, load, or availability
- Better autonomy: improve localization, path planning, obstacle avoidance, or task execution
- Better operations: add logs, alerts, replay, maps, operator controls, or judge-friendly visualizations
- Better hardware behavior: improve pickup/drop-off workflows, gripper reliability, or mechanical integration
- Better resilience: handle disconnects, bad telemetry, stuck robots, and command retries

Good demo: the project has a clear idea beyond the starter code and shows why that idea improves fleet operation.

## Starter Package Overview

This folder contains starter material for the student hackathon. It is intended as a base package that teams can extend during the event, not as a polished production system.

### Included Components

- `python_scripts/central-arbiter.py`
  Central laptop/server process that accepts robot connections, displays telemetry in a GUI, sends commands, and supports a two-robot coordinated traverse mode.
- `python_scripts/client.py`
  Laptop-side bridge that connects one robot to the central arbiter over TCP and forwards commands to the robot over serial or TCP.
- `python_scripts/gui.py`
  Telemetry dashboard with robot status, arena visualization, manual controls, test-path buttons, and grid-based coordination controls.
- `python_scripts/messages.py`
  Shared message definitions for telemetry, path assignment, acknowledgements, pause/resume/stop, gripper toggle, and heartbeat messages.
- `prizm_firmware/testing-bot-controls/testing-bot-controls.ino`
  Simple Arduino/PRIZM sketch for basic manual movement and gripper testing over serial commands.
- `prizm_firmware/telemetry_and_communicate_to_aribiter/telemetry_and_communicate_to_aribiter.ino`
  More complete robot sketch with odometry, waypoint execution, telemetry publishing, ultrasonic sensor readings, acknowledgements, and command handling.
- `prizm_firmware/wireless-bot-controls/wireless-bot-controls.ino`
  A Port of `telemetry_and_communicate_to_arbiter.ino` that allows you to communicate to the robot wirelessly via an ESP32

## Development Setup

### Arduino Environment

The robot sketches use the TETRIX PRIZM Arduino library. Before uploading either `.ino` file, install the PRIZM ZIP library into your Arduino environment.

Suggested setup:

1. Download or locate the [PRIZM Arduino library ZIP file](https://go.pitsco.com/tetrix-prizm-legacy-resources?#downloads).
2. In the Arduino IDE, go to `Sketch` > `Include Library` > `Add .ZIP Library...`.
3. Select the PRIZM library ZIP file.
4. Restart the Arduino IDE if the library does not appear immediately.

Pitsco's PRIZM coding guide can be used as a reference for the PRIZM library and Arduino setup: [TETRIX PRIZM Coding Essentials](https://asset.pitsco.com/sharedimages/resources/44720_tetrix_prizm_codingessentialssg.pdf).

For board and port configuration:

1. Connect the PRIZM controller to the computer over USB.
2. In the Arduino IDE, select `Tools` > `Board` > `Arduino AVR Boards` > `Arduino Uno`.
3. The PRIZM board emulates an Arduino Uno, so use `Arduino Uno` even though the physical controller is the PRIZM board.
4. Select the matching serial port under `Tools` > `Port`. On Windows this is usually a `COM` port, such as `COM5`.
5. If multiple ports are listed, unplug and reconnect the PRIZM controller to identify which port appears.

### Python Environment

The Python starter code expects a local environment with the packages listed in `python_scripts/requirements.txt`.

Example workflow:

1. Create a virtual environment
2. Install the requirements
3. Copy `.env.example` to `.env`
4. Update the values for your local machine and robot

### Environment Variables

The provided example environment file includes:

- `SERVER_HOST_IP_ADDRESS`
- `SERVER_PORT`
- `LOCAL_SERIAL_PORT`
- `SERIAL_BAUD`
- `ROBOT_ID`
- `CLIENT_NAME`

These values control how each robot laptop connects to the central arbiter and which serial port is used to talk to the robot controller.

## Suggested Workflow

### 1. Load Robot Firmware

Choose the Arduino sketch that matches your use case:

- Use `testing-bot-controls.ino` for simple manual drive and gripper testing
- Use `telemetry_and_communicate_to_aribiter.ino` for telemetry, waypoint following, and arbiter communication
- Use `wireless-bot-controls.ino` for telemetry, waypoint following and arbiter communication wirelessly

Before uploading:

- Install the PRIZM ZIP library in the Arduino IDE
- Select `Arduino Uno` as the board
- Select the serial port for the connected PRIZM controller
- Confirm the sketch includes `#include <PRIZM.h>` without compile errors

After uploading:
- Press 'START' (the green button) on the PRIZM to run the sketch

### 2. Start the Central Arbiter

Run `python_scripts/central-arbiter.py` on the machine acting as the server/operator station.

The arbiter:

- Listens for robot laptop connections on port `9000` by default
- Displays robot telemetry in a Tkinter + Matplotlib dashboard
- Tracks robot state, heartbeat, status, and path progress
- Plans grid-based coordinated traverses for two robots in a `4 m x 4 m` arena represented as `40 x 40` cells at `10 cm` resolution

### 3. Start a Robot Client

Run `python_scripts/client.py` on the laptop connected to a robot controller.

The client:

- Opens the configured serial port
- Connects to the arbiter over TCP
- Registers the robot with a `hello` message
- Forwards robot telemetry upstream
- Relays path, stop, pause, resume, and gripper commands back to the robot

To run the command you have to pass an `.env` file as an argument, for example if you want to connect to robot_a then you run `python client.py --env ../robot_a.env`

### 4. Test and Extend

From the GUI you can already:

- Pause, resume, or stop a robot
- Toggle the gripper
- Send straight-line, turn-around, and L-shaped test paths
- Launch a coordinated two-robot traverse using goal cells in the arena grid

Teams can build on this foundation by improving localization, obstacle handling, scheduling logic, fleet behaviors, operator interfaces, or robot hardware integration.

## Notes for Teams

- The included code assumes a shared communication protocol based on newline-delimited JSON messages
- The more advanced robot sketch publishes telemetry including pose and ultrasonic distances
- The starter package currently provides basic coordination logic and operator tooling, but leaves substantial room for student innovation
- If you change robot geometry, motor behavior, or sensor layout, you should review the constants in the Arduino sketch before testing
