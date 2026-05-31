import asyncio
import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, Optional

import httpx
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

_STATIC_DIR = Path(__file__).parent / "static"

VISION_MJPEG_URL = "http://localhost:8081/stream"


class LogBuffer:
    """Thread-safe ring buffer of recent log entries (source + line + timestamp)."""

    MAX = 2000

    def __init__(self) -> None:
        self._buf: deque[dict] = deque(maxlen=self.MAX)
        self._lock = threading.Lock()

    def append(self, source: str, line: str) -> dict:
        entry = {"ts": time.time(), "source": source, "line": line}
        with self._lock:
            self._buf.append(entry)
        return entry

    def snapshot(self) -> list[dict]:
        with self._lock:
            return list(self._buf)


class _WebSocketManager:
    def __init__(self):
        self._clients: list[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._clients.append(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients = [c for c in self._clients if c is not ws]

    async def broadcast(self, data: str) -> None:
        async with self._lock:
            clients = list(self._clients)
        for ws in clients:
            try:
                await ws.send_text(data)
            except Exception:
                pass


class DashboardServer:
    """
    Lightweight FastAPI server that bridges the central arbiter to a browser dashboard.

    Usage in central-arbiter.py:
        dashboard = DashboardServer(
            snapshot_fn=get_robot_snapshot,
            command_callback=gui_command_sender,
        )
        dashboard.start()
        # then wherever gui.update_robot() is called:
        dashboard.notify(robot_id, msg)
    """

    def __init__(
        self,
        snapshot_fn: Callable[[], dict],
        command_callback: Callable[[dict], None],
        host: str = "0.0.0.0",
        port: int = 8080,
    ) -> None:
        self._snapshot_fn = snapshot_fn
        self._command_callback = command_callback
        self._host = host
        self._port = port

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_manager = _WebSocketManager()
        self._log_buf = LogBuffer()
        self._log_ws  = _WebSocketManager()
        self._app = self._build_app()

    # ------------------------------------------------------------------
    # Public interface called from arbiter threads
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start uvicorn in a background daemon thread."""
        t = threading.Thread(target=self._run, daemon=True, name="dashboard-server")
        t.start()

    def log(self, source: str, line: str) -> None:
        """
        Thread-safe. Buffer the log entry and broadcast to all /ws/logs subscribers.
        Safe to call before start() — entries accumulate in the buffer and are
        sent to any browser that connects later.
        """
        entry = self._log_buf.append(source, line)
        if self._loop is None or self._loop.is_closed():
            return
        payload = json.dumps({"type": "log_entry", **entry})
        asyncio.run_coroutine_threadsafe(
            self._log_ws.broadcast(payload), self._loop
        )

    def notify(self, robot_id: str, msg: dict) -> None:
        """
        Called by the arbiter whenever robot state changes.
        Schedules a WebSocket broadcast from the event loop thread.
        """
        if self._loop is None or self._loop.is_closed():
            return
        snapshot_data = self._snapshot_fn()
        # snapshot_data = {"robots": {...}, "obstacles": [...], "goals": [...]}
        payload = json.dumps({"type": "robot_snapshot", **snapshot_data})
        asyncio.run_coroutine_threadsafe(
            self._ws_manager.broadcast(payload), self._loop
        )

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        config = uvicorn.Config(
            self._app,
            host=self._host,
            port=self._port,
            log_level="warning",
            loop="asyncio",
        )
        server = uvicorn.Server(config)
        print(f"[Dashboard] Serving on http://{self._host}:{self._port}")
        self._loop.run_until_complete(server.serve())

    def _build_app(self) -> FastAPI:
        app = FastAPI()
        ws_manager = self._ws_manager

        @app.get("/", response_class=HTMLResponse)
        async def index():
            return (_STATIC_DIR / "index.html").read_text()

        app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")

        @app.websocket("/ws")
        async def websocket_endpoint(ws: WebSocket):
            await ws_manager.connect(ws)
            # Send current snapshot immediately on connect
            snapshot_data = self._snapshot_fn()
            await ws.send_text(
                json.dumps({"type": "robot_snapshot", **snapshot_data})
            )
            try:
                while True:
                    # Keep the connection alive; browser sends pings
                    await ws.receive_text()
            except WebSocketDisconnect:
                pass
            finally:
                await ws_manager.disconnect(ws)

        command_callback = self._command_callback

        @app.post("/command")
        async def command(request_body: dict):
            try:
                command_callback(request_body)
                return {"ok": True}
            except Exception as exc:
                return {"ok": False, "error": str(exc)}

        @app.get("/camera")
        async def camera():
            """
            Proxy the MJPEG stream from vision_localizer.py (port 8081).

            We open the upstream connection and read the response headers
            BEFORE returning a StreamingResponse so that a ConnectError
            (camera offline) is caught here and returned as a clean 503,
            rather than crashing the ASGI handler mid-stream.
            """
            client = httpx.AsyncClient(timeout=httpx.Timeout(3.0, read=None))
            try:
                upstream = await client.send(
                    client.build_request("GET", VISION_MJPEG_URL),
                    stream=True,
                )
            except Exception as exc:
                await client.aclose()
                return Response(content=f"Camera offline: {exc}", status_code=503)

            async def _proxy():
                try:
                    async for chunk in upstream.aiter_bytes():
                        yield chunk
                except Exception:
                    pass  # camera disconnected mid-stream; stop the generator cleanly
                finally:
                    await upstream.aclose()
                    await client.aclose()

            return StreamingResponse(
                _proxy(),
                media_type=upstream.headers.get(
                    "content-type",
                    "multipart/x-mixed-replace; boundary=frame",
                ),
            )

        log_buf = self._log_buf
        log_ws  = self._log_ws

        @app.websocket("/ws/logs")
        async def log_ws_endpoint(ws: WebSocket):
            await log_ws.connect(ws)
            # Replay the full buffer so the panel isn't blank on first load
            for entry in log_buf.snapshot():
                try:
                    await ws.send_text(json.dumps({"type": "log_entry", **entry}))
                except Exception:
                    break
            try:
                while True:
                    await ws.receive_text()
            except WebSocketDisconnect:
                pass
            finally:
                await log_ws.disconnect(ws)

        @app.post("/log")
        async def inject_log(body: dict):
            """
            External processes (e.g. launch.py) POST log lines here.
            Body: {"source": "<name>", "line": "<text>"}
            """
            source = str(body.get("source", "?"))
            line   = str(body.get("line",   ""))
            if line:
                self.log(source, line)
            return {"ok": True}

        return app
