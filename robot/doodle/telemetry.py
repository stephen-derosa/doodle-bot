"""Localhost telemetry tap: whoever owns the bus broadcasts what it reads.

The serial bus has exactly one master at a time (see `FeetechBus.open`), so a
viewer such as `doodle twin` cannot poll the servos while another command is
running. Instead every `Arm.read_ticks` fires the ticks it just read at a UDP
port on localhost, fire-and-forget. Nothing listens? The datagram is dropped by
the kernel for free. Something listens? It sees the arm exactly as the owning
command does, with no second bus master and no shared file.

One datagram per read at up to 50 Hz is a few microseconds of work; the cost is
paid by the sender, and there is deliberately no acknowledgement or retry.
"""
from __future__ import annotations

import json
import os
import socket
import time

DEFAULT_PORT = int(os.environ.get("DOODLE_TAP_PORT", "8770"))
_ADDR = "127.0.0.1"


class Tap:
    """Sender side. Safe to construct even if nothing will ever listen."""

    def __init__(self, port: int = DEFAULT_PORT):
        self.port = port
        self._sock: socket.socket | None = None
        self.sent = 0
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setblocking(False)
        except OSError:
            self._sock = None

    def send(self, ticks, pen_down: bool | None = None, tag: str = "") -> None:
        if self._sock is None:
            return
        msg = {"t": time.time(), "ticks": [float(v) for v in ticks], "pen_down": pen_down, "tag": tag}
        try:
            self._sock.sendto(json.dumps(msg).encode(), (_ADDR, self.port))
            self.sent += 1
        except OSError:
            pass  # no listener, or a transient; telemetry must never disturb control

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class Listener:
    """Receiver side, used by the twin."""

    def __init__(self, port: int = DEFAULT_PORT, timeout: float = 1.0):
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((_ADDR, port))
        self._sock.settimeout(timeout)

    def recv(self) -> dict | None:
        """Next datagram as a dict, or None after `timeout` seconds of silence."""
        try:
            data, _ = self._sock.recvfrom(4096)
        except socket.timeout:
            return None
        try:
            return json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            return None

    def close(self) -> None:
        self._sock.close()
