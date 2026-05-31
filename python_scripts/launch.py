#!/usr/bin/env python3
"""
AutoFleet fleet launcher.

Starts in one command:
  - central-arbiter.py   (TCP server + A* planner + web dashboard on :8080)
  - client.py × N       (one bridge process per robot, auto-restarts on crash)

Dashboard access
─────────────────
  Same local network  →  http://<this-machine-ip>:8080
  Over SSH tunnel     →  add  -L 8080:localhost:8080  to your ssh command,
                         then open  http://localhost:8080  on your laptop
  Serial-only         →  not supported (serial carries bytes, not HTTP);
                         use a USB Ethernet gadget or WiFi instead.

Usage
─────
  # Inline robot config (robot-id:serial-port pairs):
  python3 launch.py robot_A:/dev/ttyUSB0 robot_B:/dev/ttyUSB1

  # WiFi transport:
  python3 launch.py robot_A:wifi:192.168.1.50:81 robot_B:/dev/ttyUSB1

  # JSON config file (default: robots.json next to this script):
  python3 launch.py --config robots.json

Config file format (robots.json)
──────────────────────────────────
  {
    "robots": [
      {
        "robot_id": "robot_A",
        "transport": "serial",        # or "wifi"
        "serial_port": "/dev/ttyUSB0",
        "serial_baud": 115200
      },
      {
        "robot_id": "robot_B",
        "transport": "wifi",
        "esp32_host": "192.168.4.1",
        "esp32_port": 81
      }
    ]
  }
"""

import argparse
import json
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────

SCRIPTS_DIR     = Path(__file__).parent.resolve()
ARBITER_SCRIPT  = SCRIPTS_DIR / "central-arbiter.py"
CLIENT_SCRIPT   = SCRIPTS_DIR / "client.py"
DEFAULT_CONFIG  = SCRIPTS_DIR / "robots.json"

# ── Timing ────────────────────────────────────────────────────────────────────

ARBITER_READY_TIMEOUT = 15   # seconds to wait for arbiter TCP port
ARBITER_TCP_PORT      = 9000
DASHBOARD_PORT        = 8080
CLIENT_RESTART_DELAY  = 3    # seconds between client crash → restart

# ── ANSI colour helpers ────────────────────────────────────────────────────────

_COLOURS = {
    "reset":   "\033[0m",
    "bold":    "\033[1m",
    "red":     "\033[91m",
    "green":   "\033[92m",
    "yellow":  "\033[93m",
    "blue":    "\033[94m",
    "magenta": "\033[95m",
    "cyan":    "\033[96m",
    "white":   "\033[97m",
    "grey":    "\033[90m",
}

_ROBOT_COLOUR_CYCLE = ["cyan", "magenta", "yellow", "green", "blue"]

def _c(colour: str, text: str) -> str:
    if not sys.stdout.isatty():
        return text
    return _COLOURS.get(colour, "") + text + _COLOURS["reset"]

# ── Process registry ──────────────────────────────────────────────────────────

_procs: list[subprocess.Popen] = []
_shutdown = threading.Event()

def _register(proc: subprocess.Popen) -> None:
    _procs.append(proc)

def _terminate_all() -> None:
    for proc in _procs:
        if proc.poll() is None:
            proc.terminate()
    for proc in _procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

# ── Log forwarding to dashboard ───────────────────────────────────────────────

_log_q: "queue.Queue[dict | None]" = queue.Queue()


def _log_sender_loop(url: str) -> None:
    """
    Drain _log_q and POST each entry to the dashboard's /log endpoint.
    Runs in a daemon thread; log loss is acceptable — never blocks callers.
    """
    while True:
        entry = _log_q.get()
        if entry is None:
            return
        try:
            data = json.dumps(entry).encode()
            req  = urllib.request.Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
            )
            urllib.request.urlopen(req, timeout=2)
        except Exception:
            pass


def _start_log_sender() -> None:
    t = threading.Thread(
        target=_log_sender_loop,
        args=(f"http://localhost:{DASHBOARD_PORT}/log",),
        daemon=True,
        name="log-sender",
    )
    t.start()


# ── Output multiplexer ────────────────────────────────────────────────────────

def _stream_output(proc: subprocess.Popen, prefix: str, colour: str) -> None:
    """Read proc stdout line-by-line, emit with a coloured prefix, and forward to dashboard."""
    tag = _c(colour, f"[{prefix}]") + " "
    try:
        for raw in proc.stdout:
            line = raw.rstrip("\n")
            if line:
                print(tag + line, flush=True)
                _log_q.put({"source": prefix, "line": line})
    except Exception:
        pass

# ── Network helpers ───────────────────────────────────────────────────────────

def _local_ip() -> str:
    """Best-effort: return the primary LAN IP of this machine."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "localhost"

def _wait_for_arbiter(timeout: float) -> bool:
    """Block until the arbiter's TCP port is accepting connections."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", ARBITER_TCP_PORT), timeout=1):
                return True
        except OSError:
            time.sleep(0.5)
    return False

# ── Config parsing ────────────────────────────────────────────────────────────

def _parse_inline_robot(spec: str) -> dict:
    """
    Parse a single robot spec from the command line.

    Formats:
      robot_A:/dev/ttyUSB0              (serial, default baud)
      robot_A:/dev/ttyUSB0:115200       (serial, explicit baud)
      robot_A:wifi:192.168.1.10:81      (WiFi)
    """
    parts = spec.split(":")
    robot_id = parts[0]

    if len(parts) >= 2 and parts[1].lower() == "wifi":
        # robot_id:wifi:host:port
        host = parts[2] if len(parts) > 2 else "192.168.4.1"
        port = int(parts[3]) if len(parts) > 3 else 81
        return {
            "robot_id":  robot_id,
            "transport": "wifi",
            "esp32_host": host,
            "esp32_port": port,
        }
    else:
        # robot_id:serial_port[:baud]
        serial_port = parts[1] if len(parts) > 1 else ""
        baud        = int(parts[2]) if len(parts) > 2 else 115200
        return {
            "robot_id":    robot_id,
            "transport":   "serial",
            "serial_port": serial_port,
            "serial_baud": baud,
        }

def _load_config(config_path: Path) -> list[dict]:
    with open(config_path) as f:
        data = json.load(f)
    return data["robots"]

def _write_default_config(path: Path) -> None:
    default = {
        "robots": [
            {
                "robot_id":    "robot_A",
                "transport":   "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baud": 115200,
            },
            {
                "robot_id":    "robot_B",
                "transport":   "serial",
                "serial_port": "/dev/ttyUSB1",
                "serial_baud": 115200,
            },
        ]
    }
    with open(path, "w") as f:
        json.dump(default, f, indent=2)
    print(_c("grey", f"Created default config at {path}"))

# ── Process launchers ─────────────────────────────────────────────────────────

def _launch_arbiter() -> subprocess.Popen:
    proc = subprocess.Popen(
        [sys.executable, str(ARBITER_SCRIPT), "--no-gui"],
        cwd=str(SCRIPTS_DIR),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    _register(proc)
    threading.Thread(
        target=_stream_output,
        args=(proc, "ARBITER", "white"),
        daemon=True,
    ).start()
    return proc


def _build_client_env(robot: dict) -> dict:
    """Build the environment dict for a single robot's client process."""
    env = os.environ.copy()
    env["SERVER_HOST_IP_ADDRESS"] = "127.0.0.1"
    env["SERVER_PORT"]            = str(ARBITER_TCP_PORT)
    env["ROBOT_ID"]               = robot["robot_id"]
    env["CLIENT_NAME"]            = f"{robot['robot_id']}_bridge"

    if robot.get("transport", "serial") == "wifi":
        env["USE_WIFI_BRIDGE"] = "true"
        env["ESP32_HOST"]      = str(robot.get("esp32_host", "192.168.4.1"))
        env["ESP32_PORT"]      = str(robot.get("esp32_port", 81))
    else:
        env["USE_WIFI_BRIDGE"]  = "false"
        env["LOCAL_SERIAL_PORT"] = robot.get("serial_port", "")
        env["SERIAL_BAUD"]       = str(robot.get("serial_baud", 115200))

    return env


def _launch_client(robot: dict, colour: str) -> subprocess.Popen:
    env  = _build_client_env(robot)
    proc = subprocess.Popen(
        # --env /dev/null prevents client.py from loading any .env file
        # that might override the env vars we just set.
        [sys.executable, str(CLIENT_SCRIPT), "--env", "/dev/null"],
        cwd=str(SCRIPTS_DIR),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    _register(proc)
    threading.Thread(
        target=_stream_output,
        args=(proc, robot["robot_id"], colour),
        daemon=True,
    ).start()
    return proc


def _watch_client(robot: dict, colour: str) -> None:
    """
    Monitor a client process and restart it on crash until _shutdown is set.
    Runs in its own daemon thread.
    """
    while not _shutdown.is_set():
        proc = _launch_client(robot, colour)
        exit_code = proc.wait()

        if _shutdown.is_set():
            break

        tag = _c(colour, f"[{robot['robot_id']}]")
        print(
            _c("red", f"{tag} exited (code {exit_code}), "
               f"restarting in {CLIENT_RESTART_DELAY}s…"),
            flush=True,
        )
        time.sleep(CLIENT_RESTART_DELAY)

# ── Dashboard banner ──────────────────────────────────────────────────────────

def _print_dashboard_banner() -> None:
    ip = _local_ip()
    print()
    print(_c("bold", "─" * 58))
    print(_c("bold", "  AutoFleet Dashboard"))
    print(_c("bold", "─" * 58))
    print(f"  {'Same network':12s}  " + _c("cyan", f"http://{ip}:{DASHBOARD_PORT}"))
    print(f"  {'SSH tunnel':12s}  "   + _c("grey",
          f"ssh -L {DASHBOARD_PORT}:localhost:{DASHBOARD_PORT} user@{ip}"))
    print(f"  {'then open':12s}  "    + _c("cyan", f"http://localhost:{DASHBOARD_PORT}"))
    print(_c("bold", "─" * 58))
    print(_c("grey", "  Press Ctrl+C to stop all processes"))
    print(_c("bold", "─" * 58))
    print()

# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="AutoFleet fleet launcher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "robots",
        nargs="*",
        metavar="ROBOT_ID:PORT",
        help="Inline robot specs: robot_A:/dev/ttyUSB0  or  robot_A:wifi:host:port",
    )
    parser.add_argument(
        "--config", "-c",
        metavar="FILE",
        help=f"JSON config file (default: {DEFAULT_CONFIG.name})",
    )
    parser.add_argument(
        "--no-restart",
        action="store_true",
        help="Do not auto-restart robot clients on crash",
    )
    args = parser.parse_args()

    # ── Resolve robot configs ────────────────────────────────────────────────

    if args.robots:
        robots = [_parse_inline_robot(s) for s in args.robots]
    elif args.config:
        robots = _load_config(Path(args.config))
    elif DEFAULT_CONFIG.exists():
        robots = _load_config(DEFAULT_CONFIG)
    else:
        _write_default_config(DEFAULT_CONFIG)
        print(_c("yellow",
            f"No robot config supplied. Edit {DEFAULT_CONFIG} then re-run."))
        sys.exit(0)

    if not robots:
        print(_c("red", "Error: no robots defined."))
        sys.exit(1)

    # ── Signal handling ──────────────────────────────────────────────────────

    def _on_signal(sig, _frame):
        print(_c("yellow", "\nShutting down…"), flush=True)
        _shutdown.set()
        _terminate_all()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # ── Launch arbiter ───────────────────────────────────────────────────────

    print(_c("bold", "\nStarting arbiter…"), flush=True)
    arbiter = _launch_arbiter()

    print(_c("grey", f"Waiting for arbiter on TCP :{ARBITER_TCP_PORT}…"), flush=True)
    if not _wait_for_arbiter(ARBITER_READY_TIMEOUT):
        print(_c("red",
            f"Arbiter did not become ready within {ARBITER_READY_TIMEOUT}s."))
        _terminate_all()
        sys.exit(1)

    print(_c("green", "Arbiter ready."), flush=True)

    # ── Start log forwarder (sends client stdout to dashboard /log) ───────────
    _start_log_sender()

    # ── Launch robot clients ─────────────────────────────────────────────────

    for i, robot in enumerate(robots):
        colour = _ROBOT_COLOUR_CYCLE[i % len(_ROBOT_COLOUR_CYCLE)]
        robot_id = robot["robot_id"]
        transport = robot.get("transport", "serial")

        if transport == "wifi":
            detail = f"wifi {robot.get('esp32_host')}:{robot.get('esp32_port')}"
        else:
            detail = f"serial {robot.get('serial_port')} @ {robot.get('serial_baud')} baud"

        print(_c(colour, f"Starting client for {robot_id}  ({detail})"), flush=True)

        if args.no_restart:
            _launch_client(robot, colour)
        else:
            threading.Thread(
                target=_watch_client,
                args=(robot, colour),
                daemon=True,
            ).start()

    # ── Dashboard banner ─────────────────────────────────────────────────────

    _print_dashboard_banner()

    # ── Keep-alive: monitor arbiter ──────────────────────────────────────────

    try:
        while True:
            time.sleep(1)
            if arbiter.poll() is not None:
                print(_c("red", "\nArbiter exited unexpectedly — shutting down."),
                      flush=True)
                _shutdown.set()
                _terminate_all()
                sys.exit(1)
    except KeyboardInterrupt:
        _on_signal(signal.SIGINT, None)


if __name__ == "__main__":
    main()
