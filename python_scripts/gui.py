import queue
import tkinter as tk
from tkinter import ttk
from collections import defaultdict
import math
import time

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator
from matplotlib.patches import Polygon
from itertools import cycle
from messages import PauseMessage, ResumeMessage, StopMessage, ToggleGripperMessage, PathAssignmentMessage, Waypoint, MotionSettings



class TelemetryGUI:
    """
    Thread-safe telemetry dashboard.

    Server threads call:
        gui.update_robot(robot_id, telemetry_dict)

    GUI runs on main thread and consumes queued updates.
    """

    # Sensor geometry
    MAX_VALID_ULTRASONIC_CM = 200.0
    MIN_VALID_ULTRASONIC_CM = 1.0

    FRONT_SENSOR_FORWARD_OFFSET_CM = 9.5
    FRONT_SENSOR_LATERAL_OFFSET_CM = 0.0
    FRONT_SENSOR_ANGLE_OFFSET_DEG = 0.0

    LEFT_SENSOR_FORWARD_OFFSET_CM = 0.0
    LEFT_SENSOR_LATERAL_OFFSET_CM = 16.0
    LEFT_SENSOR_ANGLE_OFFSET_DEG = 90.0

    # Arena display settings: fixed 4m x 4m = 400cm x 400cm
    ARENA_SIZE_CM = 400
    HALF_ARENA_CM = ARENA_SIZE_CM / 2
    GRID_SPACING_CM = 10
    ECHO_WINDOW_S = 10.0
    GRID_DIM_CELLS = 40
    DEFAULT_TEST_DISTANCE_CM = 30.0
    DEFAULT_TURN_SPEED = 150
    DEFAULT_DRIVE_SPEED = 200

    # Robot safety box
    SAFETY_BOX_SIZE_CM = 40.0
    SAFETY_BOX_HALF_CM = SAFETY_BOX_SIZE_CM / 2.0

    def __init__(self, command_sender=None):
        self.root = tk.Tk()
        self.root.title("Robot Telemetry Dashboard")
        self.root.geometry("1600x900")

        self.command_sender = command_sender
        self.telemetry_queue = queue.Queue()
        self.test_path_counter = int(time.time())

        # latest per-robot state
        self.robot_states = {}

        # per-robot history
        self.robot_history = defaultdict(lambda: {
            "t": [],
            "x": [],
            "y": [],
            "theta": [],
            "front_ultra": [],
            "left_ultra": [],
            "left_motor_vel": [],
            "right_motor_vel": [],
            "front_echo_t": [],
            "front_echo_x": [],
            "front_echo_y": [],
            "left_echo_t": [],
            "left_echo_x": [],
            "left_echo_y": [],
        })

        self.robot_colors = {}
        self.color_cycle = cycle([
            "tab:blue",
            "tab:orange",
            "tab:green",
            "tab:red",
            "tab:purple",
            "tab:brown",
            "tab:pink",
            "tab:gray",
            "tab:olive",
            "tab:cyan",
        ])

        self._build_layout()
        self.root.after(100, self._process_queue)

    # ----------------------------
    # Public API
    # ----------------------------
    def update_robot(self, robot_id: str, telemetry: dict):
        self.telemetry_queue.put((robot_id, telemetry))

    def run(self):
        self.root.mainloop()

    # ----------------------------
    # Layout
    # ----------------------------
    
    def _build_layout(self): 
        mainframe = ttk.Frame(self.root, padding=8)
        mainframe.pack(fill=tk.BOTH, expand=True)

        # Left panel: compact robot table
        left_panel = ttk.Frame(mainframe, width=320)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, expand=False)
        left_panel.pack_propagate(False)

        canvas = tk.Canvas(left_panel)
        scrollbar = ttk.Scrollbar(left_panel, orient="vertical", command=canvas.yview)

        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        robots_frame = ttk.LabelFrame(scrollable_frame, text="Robots")
        robots_frame.pack(fill=tk.X, expand=False, padx=(0, 8), pady=(0, 8))

        columns = ("robot_id", "state", "x", "y", "theta")
        self.tree = ttk.Treeview(robots_frame, columns=columns, show="headings", height=4)

        for col in columns:
            self.tree.heading(col, text=col)

        self.tree.column("robot_id", width=80, anchor="center")
        self.tree.column("state", width=70, anchor="center")
        self.tree.column("x", width=50, anchor="center")
        self.tree.column("y", width=50, anchor="center")
        self.tree.column("theta", width=60, anchor="center")

        self.tree.pack(fill=tk.X, expand=False)

        # Right panel: one large arena plot
        right_panel = ttk.Frame(mainframe)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(10, 8), tight_layout=True)
        self.ax_traj = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create Control Board  
        controls_frame = ttk.LabelFrame(scrollable_frame, text="Test Commands")
        controls_frame.pack(fill=tk.X, expand=False, padx=(0, 8), pady=(8, 0))

        ttk.Label(controls_frame, text="Target Robot:").pack(fill=tk.X, pady=(4, 2))

        self.selected_robot_var = tk.StringVar(value="")
        self.robot_selector = ttk.Combobox(
            controls_frame,
            textvariable=self.selected_robot_var,
            state="readonly",
            values=[],
        )
        self.robot_selector.pack(fill=tk.X, pady=(0, 6))

        self.selected_robot_summary_var = tk.StringVar(
            value="Select a robot to send a test path."
        )
        ttk.Label(
            controls_frame,
            textvariable=self.selected_robot_summary_var,
            wraplength=280,
            justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(0, 6))

        ttk.Button(controls_frame, text="Pause", command=self._send_pause).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="Resume", command=self._send_resume).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="Stop", command=self._send_stop).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="Toggle Gripper", command=self._send_toggle_gripper).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="Send Straight Test", command=self._send_straight_test_path).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="Send 180 Turn Test", command=self._send_turnaround_test_path).pack(fill=tk.X, pady=2)
        ttk.Button(controls_frame, text="Send L Test Path", command=self._send_test_path).pack(fill=tk.X, pady=2)

        coordination_frame = ttk.LabelFrame(scrollable_frame, text="Grid Coordination")
        coordination_frame.pack(fill=tk.X, expand=False, padx=(0, 8), pady=(8, 0))

        ttk.Label(
            coordination_frame,
            text="4m x 4m arena split into 40 x 40 cells at 10 cm resolution.",
            wraplength=280,
            justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(4, 6))

        ttk.Label(coordination_frame, text="Robot 1").pack(fill=tk.X)

        self.grid_robot_one_var = tk.StringVar(value="")
        self.grid_robot_one_selector = ttk.Combobox(
            coordination_frame,
            textvariable=self.grid_robot_one_var,
            state="readonly",
            values=[],
        )
        self.grid_robot_one_selector.pack(fill=tk.X, pady=(0, 4))

        robot_one_goal = ttk.Frame(coordination_frame)
        robot_one_goal.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(robot_one_goal, text="Goal row").grid(row=0, column=0, sticky="w")
        self.grid_robot_one_row_var = tk.StringVar(value="10")
        ttk.Entry(robot_one_goal, textvariable=self.grid_robot_one_row_var, width=6).grid(row=0, column=1, padx=(6, 12))
        ttk.Label(robot_one_goal, text="Goal col").grid(row=0, column=2, sticky="w")
        self.grid_robot_one_col_var = tk.StringVar(value="10")
        ttk.Entry(robot_one_goal, textvariable=self.grid_robot_one_col_var, width=6).grid(row=0, column=3, padx=(6, 0))
        ttk.Label(coordination_frame, text="Robot 2").pack(fill=tk.X)

        self.grid_robot_two_var = tk.StringVar(value="")
        self.grid_robot_two_selector = ttk.Combobox(
            coordination_frame,
            textvariable=self.grid_robot_two_var,
            state="readonly",
            values=[],
        )
        self.grid_robot_two_selector.pack(fill=tk.X, pady=(0, 4))

        robot_two_goal = ttk.Frame(coordination_frame)
        robot_two_goal.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(robot_two_goal, text="Goal row").grid(row=0, column=0, sticky="w")
        self.grid_robot_two_row_var = tk.StringVar(value="20")
        ttk.Entry(robot_two_goal, textvariable=self.grid_robot_two_row_var, width=6).grid(row=0, column=1, padx=(6, 12))
        ttk.Label(robot_two_goal, text="Goal col").grid(row=0, column=2, sticky="w")
        self.grid_robot_two_col_var = tk.StringVar(value="20")
        ttk.Entry(robot_two_goal, textvariable=self.grid_robot_two_col_var, width=6).grid(row=0, column=3, padx=(6, 0))

        self.grid_plan_summary_var = tk.StringVar(value="Select two robots and set destination cells (0-39).")
        ttk.Label(
            coordination_frame,
            textvariable=self.grid_plan_summary_var,
            wraplength=280,
            justify=tk.LEFT,
        ).pack(fill=tk.X, pady=(0, 6))

        ttk.Button(
            coordination_frame,
            text="Start Two-Robot Traverse",
            command=self._send_two_robot_traverse,
        ).pack(fill=tk.X, pady=2)

    def _get_robot_color(self, robot_id: str) -> str:
        if robot_id not in self.robot_colors:
            self.robot_colors[robot_id] = next(self.color_cycle)
        return self.robot_colors[robot_id]

    def _draw_robot_safety_box(self, ax, x: float, y: float, theta_deg: float, label: str, color: str):
        """
        Draw a rotated 40cm x 40cm safety square centered on the robot.
        """
        half = self.SAFETY_BOX_HALF_CM
        theta_rad = math.radians(theta_deg)

        # Square corners in robot-local coordinates
        local_corners = [
            (-half, -half),
            ( half, -half),
            ( half,  half),
            (-half,  half),
        ]

        world_corners = []
        for lx, ly in local_corners:
            wx = x + lx * math.cos(theta_rad) - ly * math.sin(theta_rad)
            wy = y + lx * math.sin(theta_rad) + ly * math.cos(theta_rad)
            world_corners.append((wx, wy))

        patch = Polygon(
            world_corners,
            closed=True,
            fill=False,
            linewidth=1.5,
            linestyle="-",
            alpha=0.8,
            label=label,
            edgecolor=color,
        )
        ax.add_patch(patch)

    # ----------------------------
    # Queue processing
    # ----------------------------
    def _prune_echo_history(self, hist: dict, current_t_s: float):
        cutoff_t_s = current_t_s - self.ECHO_WINDOW_S

        while hist["front_echo_t"] and hist["front_echo_t"][0] < cutoff_t_s:
            hist["front_echo_t"].pop(0)
            hist["front_echo_x"].pop(0)
            hist["front_echo_y"].pop(0)

        while hist["left_echo_t"] and hist["left_echo_t"][0] < cutoff_t_s:
            hist["left_echo_t"].pop(0)
            hist["left_echo_x"].pop(0)
            hist["left_echo_y"].pop(0)

    def _process_queue(self):
        updated = False

        while not self.telemetry_queue.empty():
            robot_id, telemetry = self.telemetry_queue.get()

            self.robot_states[robot_id] = telemetry
            hist = self.robot_history[robot_id]

            t = telemetry.get("t_ms")
            x = telemetry.get("x_cm")
            y = telemetry.get("y_cm")
            theta = telemetry.get("theta_deg")
            front = telemetry.get("front_ultrasonic_cm")
            left = telemetry.get("left_ultrasonic_cm")
            left_vel = telemetry.get("left_motor_vel_deg_per_sec")
            right_vel = telemetry.get("right_motor_vel_deg_per_sec")
            t_s = float(t) / 1000.0 if t is not None else None

            if t is not None:
                hist["t"].append(t_s)
            if x is not None:
                hist["x"].append(float(x))
            if y is not None:
                hist["y"].append(float(y))
            if theta is not None:
                hist["theta"].append(float(theta))
            if front is not None:
                hist["front_ultra"].append(float(front))
            if left is not None:
                hist["left_ultra"].append(float(left))
            if left_vel is not None:
                hist["left_motor_vel"].append(float(left_vel))
            if right_vel is not None:
                hist["right_motor_vel"].append(float(right_vel))

            if x is not None and y is not None and theta is not None:
                x = float(x)
                y = float(y)
                theta = float(theta)

                if front is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(front),
                        self.FRONT_SENSOR_FORWARD_OFFSET_CM,
                        self.FRONT_SENSOR_LATERAL_OFFSET_CM,
                        self.FRONT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["front_echo_t"].append(t_s if t_s is not None else 0.0)
                        hist["front_echo_x"].append(ex)
                        hist["front_echo_y"].append(ey)

                if left is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(left),
                        self.LEFT_SENSOR_FORWARD_OFFSET_CM,
                        self.LEFT_SENSOR_LATERAL_OFFSET_CM,
                        self.LEFT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["left_echo_t"].append(t_s if t_s is not None else 0.0)
                        hist["left_echo_x"].append(ex)
                        hist["left_echo_y"].append(ey)

            if t_s is not None:
                self._prune_echo_history(hist, t_s)

            updated = True

        if updated:
            self._refresh_table()
            self._refresh_robot_selector()
            self._refresh_plot()

        self.root.after(100, self._process_queue)

    # ----------------------------
    # Math helpers
    # ----------------------------
    def _world_sensor_position(
        self,
        x: float,
        y: float,
        theta_deg: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
    ):
        theta_rad = math.radians(theta_deg)

        sensor_x = (
            x
            + forward_offset_cm * math.cos(theta_rad)
            - lateral_offset_cm * math.sin(theta_rad)
        )
        sensor_y = (
            y
            + forward_offset_cm * math.sin(theta_rad)
            + lateral_offset_cm * math.cos(theta_rad)
        )
        return sensor_x, sensor_y

    def _compute_echo_point(
        self,
        x: float,
        y: float,
        theta_deg: float,
        ultrasonic_cm: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        sensor_angle_offset_deg: float,
    ):
        if not (self.MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= self.MAX_VALID_ULTRASONIC_CM):
            return None

        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )

        ray_angle_deg = theta_deg + sensor_angle_offset_deg
        ray_angle_rad = math.radians(ray_angle_deg)

        ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
        ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

        return ex, ey

    def _draw_latest_ray(
        self,
        ax,
        x: float,
        y: float,
        theta_deg: float,
        ultrasonic_cm: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        sensor_angle_offset_deg: float,
        label: str,
        linestyle: str,
        color: str,
    ):
        if not (self.MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= self.MAX_VALID_ULTRASONIC_CM):
            return

        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )

        ray_angle_rad = math.radians(theta_deg + sensor_angle_offset_deg)
        ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
        ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

        ax.plot([sensor_x, ex], [sensor_y, ey], linestyle=linestyle, label=label, color=color)

    # ----------------------------
    # Refresh UI
    # ----------------------------
    def _refresh_table(self):
        self.tree.delete(*self.tree.get_children())

        for robot_id, state in self.robot_states.items():
            self.tree.insert(
                "",
                "end",
                values=(
                    robot_id,
                    state.get("state", ""),
                    round(float(state.get("x_cm", 0.0)), 2),
                    round(float(state.get("y_cm", 0.0)), 2),
                    round(float(state.get("theta_deg", 0.0)), 2),
                ),
            )

    def _refresh_plot(self):
        self.ax_traj.clear()

        self.ax_traj.set_title("Robot Trajectory + Ultrasonic Points")
        self.ax_traj.set_xlabel("x (cm)")
        self.ax_traj.set_ylabel("y (cm)")

        # Arena is 0 → 400 cm
        self.ax_traj.set_xlim(0, self.ARENA_SIZE_CM)
        self.ax_traj.set_ylim(0, self.ARENA_SIZE_CM)

        # Keep physical scale equal
        self.ax_traj.set_aspect("equal", adjustable="box")

        # Major ticks every 50 cm (labels)
        self.ax_traj.xaxis.set_major_locator(MultipleLocator(50))
        self.ax_traj.yaxis.set_major_locator(MultipleLocator(50))

        # Minor ticks every 10 cm (grid squares)
        self.ax_traj.xaxis.set_minor_locator(MultipleLocator(10))
        self.ax_traj.yaxis.set_minor_locator(MultipleLocator(10))

        # Draw grids
        self.ax_traj.grid(which="major", linewidth=1.0)
        self.ax_traj.grid(which="minor", linewidth=0.3)

        for robot_id, hist in self.robot_history.items():
            xs = hist["x"]
            ys = hist["y"]
            color = self._get_robot_color(robot_id)

            if xs and ys:
                self.ax_traj.plot(xs, ys, marker="o", linestyle="-", label=f"{robot_id} path", color=color)

                theta_deg = hist["theta"][-1]
                theta_rad = math.radians(theta_deg)
                arrow_len = 8.0
                dx = arrow_len * math.cos(theta_rad)
                dy = arrow_len * math.sin(theta_rad)

                self.ax_traj.arrow(
                    xs[-1],
                    ys[-1],
                    dx,
                    dy,
                    head_width=3.0,
                    head_length=4.0,
                    length_includes_head=True,
                )

                # Robot center point
                self.ax_traj.scatter(
                    [xs[-1]],
                    [ys[-1]],
                    s=50,
                    label=f"{robot_id} center",
                    zorder=5,
                    color=color,
                )

                # Rotated safety box
                self._draw_robot_safety_box(
                    self.ax_traj,
                    xs[-1],
                    ys[-1],
                    theta_deg,
                    label=f"{robot_id} safety box",
                    color=color,
                )

                if hist["front_ultra"]:
                    self._draw_latest_ray(
                        self.ax_traj,
                        xs[-1],
                        ys[-1],
                        theta_deg,
                        hist["front_ultra"][-1],
                        self.FRONT_SENSOR_FORWARD_OFFSET_CM,
                        self.FRONT_SENSOR_LATERAL_OFFSET_CM,
                        self.FRONT_SENSOR_ANGLE_OFFSET_DEG,
                        label=f"{robot_id} latest front ray",
                        linestyle="--",
                        color=color,
                    )

            if hist["front_echo_x"] and hist["front_echo_y"]:
                self.ax_traj.scatter(
                    hist["front_echo_x"],
                    hist["front_echo_y"],
                    s=20,
                    alpha=0.75,
                    label=f"{robot_id} front hits",
                    color=color,
                )

            if hist["left_echo_x"] and hist["left_echo_y"]:
                self.ax_traj.scatter(
                    hist["left_echo_x"],
                    hist["left_echo_y"],
                    s=20,
                    alpha=0.75,
                    label=f"{robot_id} left hits",
                    color=color,
                )

        handles, labels = self.ax_traj.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            self.ax_traj.legend(unique.values(), unique.keys(), loc="best")

        self.canvas.draw()
    
    # ----------------------------
    # Control Board
    # ----------------------------
    def _get_selected_robot_id(self) -> str | None:
        robot_id = self.selected_robot_var.get().strip()
        if not robot_id:
            return None
        return robot_id

    def _get_selected_robot_state(self) -> dict | None:
        robot_id = self._get_selected_robot_id()
        if robot_id is None:
            return None
        return self.robot_states.get(robot_id)

    def _get_test_distance_cm(self) -> float:
        return self.DEFAULT_TEST_DISTANCE_CM

    def _get_motion_settings(self) -> MotionSettings:
        return MotionSettings(
            turn_speed_deg_per_sec=self.DEFAULT_TURN_SPEED,
            drive_speed_deg_per_sec=self.DEFAULT_DRIVE_SPEED,
        )

    def _next_test_path_id(self) -> int:
        self.test_path_counter += 1
        return self.test_path_counter

    def _send_pause(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = PauseMessage(
                robot_id=robot_id,
                reason="gui_pause_button",
            )
            self.command_sender(msg)

    def _send_resume(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = ResumeMessage(robot_id=robot_id)
            self.command_sender(msg)

    def _send_stop(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = StopMessage(
                robot_id=robot_id,
                reason="gui_stop_button",
            )
            self.command_sender(msg)

    def _send_toggle_gripper(self):
        robot_id = self._get_selected_robot_id()
        if robot_id and self.command_sender:
            msg = ToggleGripperMessage(robot_id=robot_id)
            self.command_sender(msg)

    def _send_straight_test_path(self):
        robot_id = self._get_selected_robot_id()
        robot_state = self._get_selected_robot_state()

        if not robot_id or not self.command_sender or not robot_state:
            return

        x = float(robot_state.get("x_cm", 0.0))
        y = float(robot_state.get("y_cm", 0.0))
        theta_deg = float(robot_state.get("theta_deg", 0.0))
        theta_rad = math.radians(theta_deg)
        distance_cm = self._get_test_distance_cm()

        wp_x = min(max(x + distance_cm * math.cos(theta_rad), 0.0), 400.0)
        wp_y = min(max(y + distance_cm * math.sin(theta_rad), 0.0), 400.0)

        msg = PathAssignmentMessage(
            robot_id=robot_id,
            path_id=self._next_test_path_id(),
            replace_existing=True,
            waypoints=[Waypoint(x_cm=wp_x, y_cm=wp_y)],
            motion=self._get_motion_settings(),
        )

        self.command_sender(msg)

    def _send_turnaround_test_path(self):
        robot_id = self._get_selected_robot_id()
        robot_state = self._get_selected_robot_state()

        if not robot_id or not self.command_sender or not robot_state:
            return

        x = float(robot_state.get("x_cm", 0.0))
        y = float(robot_state.get("y_cm", 0.0))
        theta_deg = float(robot_state.get("theta_deg", 0.0))
        theta_rad = math.radians(theta_deg)
        distance_cm = self._get_test_distance_cm()

        wp_x = min(max(x - distance_cm * math.cos(theta_rad), 0.0), 400.0)
        wp_y = min(max(y - distance_cm * math.sin(theta_rad), 0.0), 400.0)

        msg = PathAssignmentMessage(
            robot_id=robot_id,
            path_id=self._next_test_path_id(),
            replace_existing=True,
            waypoints=[Waypoint(x_cm=wp_x, y_cm=wp_y)],
            motion=self._get_motion_settings(),
        )

        self.command_sender(msg)

    def _send_test_path(self):
        robot_id = self._get_selected_robot_id()
        robot_state = self._get_selected_robot_state()

        if not robot_id or not self.command_sender or not robot_state:
            return

        # Start from the robot's current believed pose and send a simple
        # three-waypoint path in the global arena frame.
        x = float(robot_state.get("x_cm", 0.0))
        y = float(robot_state.get("y_cm", 0.0))

        # Simple "Γ"-shaped test path, clipped to the 0..400 cm arena
        wp1_x = min(max(x + 20.0, 0.0), 400.0)
        wp1_y = min(max(y,         0.0), 400.0)

        wp2_x = min(max(x + 40.0, 0.0), 400.0)
        wp2_y = min(max(y,         0.0), 400.0)

        wp3_x = min(max(x + 40.0, 0.0), 400.0)
        wp3_y = min(max(y + 40.0, 0.0), 400.0)

        msg = PathAssignmentMessage(
            robot_id=robot_id,
            path_id=self._next_test_path_id(),
            replace_existing=True,
            waypoints=[
                Waypoint(x_cm=wp1_x, y_cm=wp1_y),
                Waypoint(x_cm=wp2_x, y_cm=wp2_y),
                Waypoint(x_cm=wp3_x, y_cm=wp3_y),
            ],
            motion=self._get_motion_settings(),
        )

        self.command_sender(msg)

    def _parse_goal_cell(self, row_var: tk.StringVar, col_var: tk.StringVar) -> tuple[int, int] | None:
        try:
            row = int(row_var.get())
            col = int(col_var.get())
        except (TypeError, ValueError):
            return None

        if not (0 <= row < self.GRID_DIM_CELLS and 0 <= col < self.GRID_DIM_CELLS):
            return None

        return row, col

    def _send_two_robot_traverse(self):
        if not self.command_sender:
            return

        robot_one = self.grid_robot_one_var.get().strip()
        robot_two = self.grid_robot_two_var.get().strip()
        if not robot_one or not robot_two or robot_one == robot_two:
            self.grid_plan_summary_var.set("Pick two different robots before starting a coordinated traverse.")
            return

        goal_one = self._parse_goal_cell(self.grid_robot_one_row_var, self.grid_robot_one_col_var)
        goal_two = self._parse_goal_cell(self.grid_robot_two_row_var, self.grid_robot_two_col_var)
        if goal_one is None or goal_two is None:
            self.grid_plan_summary_var.set("Goal cells must be integers from 0 to 39.")
            return

        if goal_one == goal_two:
            self.grid_plan_summary_var.set("Choose different goal cells for the two robots.")
            return

        self.grid_plan_summary_var.set(
            f"Planning coordinated traverse: {robot_one} -> ({goal_one[0]}, {goal_one[1]}), "
            f"{robot_two} -> ({goal_two[0]}, {goal_two[1]})."
        )

        self.command_sender({
            "type": "coordinated_traverse",
            "robots": [
                {"robot_id": robot_one, "goal_row": goal_one[0], "goal_col": goal_one[1]},
                {"robot_id": robot_two, "goal_row": goal_two[0], "goal_col": goal_two[1]},
            ],
        })
    

    def _refresh_robot_selector(self):
        robot_ids = sorted(self.robot_states.keys())
        self.robot_selector["values"] = robot_ids
        self.grid_robot_one_selector["values"] = robot_ids
        self.grid_robot_two_selector["values"] = robot_ids

        current = self.selected_robot_var.get()

        # If current selection disappeared, clear it
        if current and current not in robot_ids:
            self.selected_robot_var.set("")

        # If nothing is selected and robots exist, pick the first one
        if not self.selected_robot_var.get() and robot_ids:
            self.selected_robot_var.set(robot_ids[0])

        if not self.grid_robot_one_var.get() and robot_ids:
            self.grid_robot_one_var.set(robot_ids[0])

        if not self.grid_robot_two_var.get() and len(robot_ids) > 1:
            self.grid_robot_two_var.set(robot_ids[1])
        elif not self.grid_robot_two_var.get() and robot_ids:
            self.grid_robot_two_var.set(robot_ids[0])

        selected_state = self._get_selected_robot_state()
        if selected_state is None:
            self.selected_robot_summary_var.set("Select a robot to send a test path.")
            return

        robot_id = self._get_selected_robot_id()
        state = selected_state.get("state", "unknown")
        path_id = selected_state.get("path_id", "-")
        waypoint_index = selected_state.get("waypoint_index", "-")
        x = float(selected_state.get("x_cm", 0.0))
        y = float(selected_state.get("y_cm", 0.0))
        theta = float(selected_state.get("theta_deg", 0.0))

        left_vel = selected_state.get("left_motor_vel_deg_per_sec")
        right_vel = selected_state.get("right_motor_vel_deg_per_sec")
        vel_str = ""
        if left_vel is not None and right_vel is not None:
            vel_str = f", vel=L:{int(left_vel)} R:{int(right_vel)} deg/s"

        self.selected_robot_summary_var.set(
            f"{robot_id}: state={state}, path={path_id}, waypoint={waypoint_index}, "
            f"pose=({x:.1f}, {y:.1f}, {theta:.1f} deg){vel_str}"
        )
