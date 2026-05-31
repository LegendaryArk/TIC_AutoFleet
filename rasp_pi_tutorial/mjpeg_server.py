"""
Minimal MJPEG streaming server for OpenCV frames.

Usage in vision_localizer.py:
    from mjpeg_server import MjpegServer

    _stream = MjpegServer(port=8081)
    _stream.start()

    # inside the main loop, after building the combined debug frame:
    _stream.push_frame(combined)

The dashboard proxies this stream at GET /camera.
Accessible directly at http://<rpi-ip>:8081/stream
"""

import io
import socket
import threading
import time
from typing import Optional

import cv2
import numpy as np

_BOUNDARY = b"frame"
_RESPONSE_HEADER = (
    b"HTTP/1.1 200 OK\r\n"
    b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    b"Cache-Control: no-cache\r\n"
    b"Connection: close\r\n"
    b"\r\n"
)


class MjpegServer:
    def __init__(self, port: int = 8081, jpeg_quality: int = 75):
        self._port = port
        self._jpeg_quality = jpeg_quality
        self._lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._clients: list[socket.socket] = []
        self._running = False

    def start(self) -> None:
        self._running = True
        t = threading.Thread(target=self._accept_loop, daemon=True, name="mjpeg-server")
        t.start()

    def push_frame(self, frame: np.ndarray) -> None:
        """Encode an OpenCV BGR frame as JPEG and broadcast to all clients."""
        ok, buf = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        )
        if not ok:
            return

        jpeg = buf.tobytes()
        part = (
            b"--" + _BOUNDARY + b"\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n"
            b"\r\n"
        ) + jpeg + b"\r\n"

        with self._lock:
            self._latest_jpeg = jpeg
            dead = []
            for conn in self._clients:
                try:
                    conn.sendall(part)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self._clients.remove(conn)
                try:
                    conn.close()
                except OSError:
                    pass

    def _accept_loop(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", self._port))
            srv.listen(8)
            print(f"[MJPEG] Streaming on http://0.0.0.0:{self._port}/stream")

            while self._running:
                try:
                    srv.settimeout(1.0)
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                t = threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True,
                )
                t.start()

    def _handle_client(self, conn: socket.socket, addr) -> None:
        try:
            # Drain the HTTP request (ignore it)
            conn.settimeout(2.0)
            try:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk or b"\r\n\r\n" in chunk:
                        break
            except socket.timeout:
                pass

            conn.settimeout(None)
            conn.sendall(_RESPONSE_HEADER)

            # Send the most recent frame immediately so the client isn't blank
            with self._lock:
                latest = self._latest_jpeg

            if latest is not None:
                part = (
                    b"--" + _BOUNDARY + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(latest)).encode() + b"\r\n"
                    b"\r\n"
                ) + latest + b"\r\n"
                conn.sendall(part)

            with self._lock:
                self._clients.append(conn)

        except OSError:
            try:
                conn.close()
            except OSError:
                pass
