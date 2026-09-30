"""Digital twin: serve the arm's live pose, the paper and the canvas to a browser.

No physics. The page just draws whatever the model believes: joint angles from
servo ticks, link positions from the same forward kinematics that plans every
stroke, and the paper/canvas from the calibrated frame. That is the point --
"see what the system sees" -- so a wrong picture here means a wrong model, not
a wrong renderer.

Three state sources:

* the bus, polled at a fixed rate, for an idle or hand-moved arm;
* the localhost telemetry tap, for watching an arm that another command is
  driving -- that command owns the serial port (exclusively) and broadcasts
  every read it makes, so the twin never becomes a second bus master;
* a `doodle draw` CSV log, tailed live or replayed.

The server is the standard library's threaded HTTP server: one sampler thread
owns the state, any number of browsers subscribe to it over server-sent events.
Everything heavy (WebGL) happens in the viewer's browser, not on the Pi.
"""
from __future__ import annotations

import csv
import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from .arm import Arm
from .config import Calibration, Config
from .kinematics import SO101Kinematics
from .telemetry import DEFAULT_PORT, Listener

STATIC = Path(__file__).parent / "static"
PEN_DOWN_MM = 2.5   # tip within this of the paper plane counts as "down" when no log says


# --- state -------------------------------------------------------------------------------------
def chain_points(kin: SO101Kinematics, q) -> list[list[float]]:
    """World positions of the arm's skeleton, base to pen tip.

    [base, top of the pan column, lift axis, elbow, wrist_flex, roll origin, tip]
    """
    q = np.asarray(q, float)
    p0, p1, p2, p3, _ = kin.planar_points(q)
    c, s = math.cos(q[0]), math.sin(q[0])

    def world(rz):
        return [float(rz[0] * c), float(rz[0] * s), float(rz[1])]

    pts = [[0.0, 0.0, 0.0], [0.0, 0.0, float(kin.g.lift_z)], world(p0), world(p1), world(p2), world(p3)]
    pts.append([float(v) for v in kin.fk(q)])      # exact tip, lateral offset included
    return pts


def _rect(frame, corners_uv) -> list[list[float]]:
    return [[float(v) for v in frame.to_world(u, w)] for u, w in corners_uv]


def twin_state(arm: Arm, kin: SO101Kinematics, cfg: Config, cal: Calibration, ticks,
               alive: dict[str, bool] | None = None, source: str = "bus",
               pen_down: bool | None = None, error: str | None = None, t_rel: float | None = None) -> dict:
    """Everything the page needs for one frame, as plain JSON-able data."""
    ticks = np.asarray(ticks, float)
    q = arm.ticks_to_q(ticks)
    chain = chain_points(kin, q)
    tip = chain[-1]
    P = cfg.paper
    paper_corners = _rect(cal.paper, [(0, 0), (P.width, 0), (P.width, P.height), (0, P.height)])
    u0, v0 = P.canvas_origin()
    canvas_corners = _rect(cal.paper, [(u0, v0), (u0 + P.canvas_width, v0),
                                       (u0 + P.canvas_width, v0 + P.canvas_height), (u0, v0 + P.canvas_height)])
    paper_z = float(cal.paper.origin[2])
    if pen_down is None and cal.paper.calibrated:
        pen_down = bool(tip[2] <= paper_z + PEN_DOWN_MM)
    return {
        "t": time.time(),
        "t_rel": t_rel,
        "source": source,
        "error": error,
        "joints": [
            {
                "name": n, "id": int(i), "ticks": float(tk), "deg": float(math.degrees(qq)),
                "min_deg": float(math.degrees(lo)), "max_deg": float(math.degrees(hi)),
                "alive": None if alive is None else bool(alive.get(n, False)),
            }
            for n, i, tk, qq, lo, hi in zip(arm.names, arm.ids, ticks, q, arm.min_rad, arm.max_rad)
        ],
        "chain": chain,
        "tip": tip,
        "pen_dir": [float(v) for v in kin.pen_direction(q)],
        "pen_down": pen_down,
        "paper": {"calibrated": bool(cal.paper.calibrated), "corners": paper_corners, "z": paper_z,
                  "width": float(P.width), "height": float(P.height)},
        "canvas": {"corners": canvas_corners, "width": float(P.canvas_width), "height": float(P.canvas_height)},
        "geometry": {"lift_z": float(kin.g.lift_z), "lift_r": float(kin.g.lift_r)},
        "joints_calibrated": bool(cal.joints_calibrated),
    }


# --- state providers ------------------------------------------------------------------------------
class StateStore:
    """Latest frame plus a condition variable so SSE writers can block until it changes."""

    def __init__(self):
        self._cond = threading.Condition()
        self._seq = 0
        self._payload = b"{}"

    def publish(self, state: dict) -> None:
        with self._cond:
            self._seq += 1
            state["seq"] = self._seq
            self._payload = json.dumps(state).encode()
            self._cond.notify_all()

    def latest(self) -> tuple[int, bytes]:
        with self._cond:
            return self._seq, self._payload

    def wait_for_change(self, seen: int, timeout: float) -> None:
        with self._cond:
            if self._seq == seen:
                self._cond.wait(timeout)


class BusSampler(threading.Thread):
    """Poll the servos at `rate` Hz and publish a frame each time.

    `demo` animates the fake arm so the page can be exercised with no hardware.
    """

    def __init__(self, store: StateStore, arm: Arm, kin, cfg, cal, rate: float = 20.0, demo: bool = False):
        super().__init__(daemon=True, name="twin-sampler")
        self.store, self.arm, self.kin, self.cfg, self.cal = store, arm, kin, cfg, cal
        self.period = 1.0 / rate
        self.demo = demo
        self.stop = threading.Event()
        self.alive = None

    def run(self) -> None:
        try:
            self.alive = self.arm.ping_all()
        except Exception as e:  # noqa: BLE001 - keep serving with the error shown
            self.alive = None
            self.store.publish(twin_state(self.arm, self.kin, self.cfg, self.cal,
                                          self.arm.zero, source="bus", error=f"ping failed: {e}"))
        t0 = time.monotonic()
        while not self.stop.is_set():
            t = time.monotonic() - t0
            try:
                if self.demo:
                    self._animate(t)
                ticks = self.arm.read_ticks()
                state = twin_state(self.arm, self.kin, self.cfg, self.cal, ticks, alive=self.alive,
                                   source="demo" if self.demo else "bus")
            except Exception as e:  # noqa: BLE001 - a bus hiccup must not kill the twin
                state = twin_state(self.arm, self.kin, self.cfg, self.cal, self.arm.zero,
                                   alive=self.alive, source="bus", error=str(e))
            self.store.publish(state)
            self.stop.wait(self.period)

    def _animate(self, t: float) -> None:
        """Gentle sinusoids inside each joint's limits, written straight into the fake bus."""
        lo, hi = self.arm.min_rad, self.arm.max_rad
        mid, amp = (lo + hi) / 2, np.minimum((hi - lo) / 2 * 0.6, math.radians(35))
        q = mid + amp * np.sin(t * np.array([0.30, 0.45, 0.55, 0.70, 0.25]) + np.arange(5))
        for i, tk in zip(self.arm.ids, self.arm.q_to_ticks(q)):
            self.arm.bus.mem[i]["present_position"] = int(round(tk))


class LogFollower(threading.Thread):
    """Tail (or replay) a `doodle draw` CSV and publish its actual-or-commanded ticks.

    Watching a drawing this way keeps the twin off the serial bus while the
    executor owns it. `pace` sleeps to the log's own timestamps so a finished
    file replays in real time; a live file is simply followed as rows appear.
    """

    def __init__(self, store: StateStore, arm: Arm, kin, cfg, cal, path: Path, pace: bool = True):
        super().__init__(daemon=True, name="twin-log")
        self.store, self.arm, self.kin, self.cfg, self.cal = store, arm, kin, cfg, cal
        self.path, self.pace = Path(path), pace
        self.stop = threading.Event()

    def run(self) -> None:
        with open(self.path, newline="") as f:
            header = None
            t_wall0 = t_log0 = None
            while not self.stop.is_set():
                line = f.readline()
                if not line:
                    self.stop.wait(0.05)
                    continue
                if not line.endswith("\n"):          # partial row still being written
                    f.seek(f.tell() - len(line))
                    self.stop.wait(0.05)
                    continue
                row = next(csv.reader([line]))
                if header is None:
                    header = row
                    continue
                rec = dict(zip(header, row))
                ticks = self._ticks(rec)
                if ticks is None:
                    continue
                t_rel = float(rec.get("t", 0.0))
                if self.pace:
                    now = time.monotonic()
                    if t_wall0 is None:
                        t_wall0, t_log0 = now, t_rel
                    due = t_wall0 + (t_rel - t_log0)
                    if due > now:
                        self.stop.wait(due - now)
                pen = rec.get("pen_down")
                self.store.publish(twin_state(
                    self.arm, self.kin, self.cfg, self.cal, ticks, source=f"log:{self.path.name}",
                    pen_down=None if pen in (None, "") else bool(int(pen)), t_rel=t_rel))

    def _ticks(self, rec: dict) -> np.ndarray | None:
        act = [rec.get(f"act_{n}", "") for n in self.arm.names]
        if all(v not in ("", None) for v in act):
            return np.array([float(v) for v in act])
        cmd = [rec.get(f"cmd_{n}", "") for n in self.arm.names]
        if all(v not in ("", None) for v in cmd):
            return np.array([float(v) for v in cmd])
        return None


class TapSource(threading.Thread):
    """Publish frames from the localhost telemetry tap instead of the bus.

    This is how the twin watches an arm that another command is driving: that
    command owns the serial port and broadcasts every read it makes, and this
    thread just listens. No second bus master, so no collisions.
    """

    def __init__(self, store: StateStore, arm: Arm, kin, cfg, cal, port: int = DEFAULT_PORT,
                 stale_after: float = 3.0):
        super().__init__(daemon=True, name="twin-tap")
        self.store, self.arm, self.kin, self.cfg, self.cal = store, arm, kin, cfg, cal
        self.port, self.stale_after = port, stale_after
        self.stop = threading.Event()

    def run(self) -> None:
        lis = Listener(self.port, timeout=0.5)
        last_ticks, last_seen = None, None
        self.store.publish(twin_state(self.arm, self.kin, self.cfg, self.cal, self.arm.zero, source="live",
                                      error=f"waiting for telemetry on udp:{self.port} -- run any doodle "
                                            f"command that reads the arm"))
        try:
            while not self.stop.is_set():
                msg = lis.recv()
                if msg is not None and "ticks" in msg and len(msg["ticks"]) == len(self.arm.ids):
                    last_ticks, last_seen = msg["ticks"], time.monotonic()
                    self.store.publish(twin_state(self.arm, self.kin, self.cfg, self.cal, last_ticks,
                                                  source="live", pen_down=msg.get("pen_down")))
                elif last_ticks is not None and time.monotonic() - last_seen > self.stale_after:
                    self.store.publish(twin_state(self.arm, self.kin, self.cfg, self.cal, last_ticks, source="live",
                                                  error=f"no telemetry for {time.monotonic() - last_seen:.0f} s"))
        finally:
            lis.close()


# --- http --------------------------------------------------------------------------------------
def make_handler(store: StateStore, static_dir: Path = STATIC, quiet: bool = True):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # noqa: N802 - stdlib name
            if not quiet:
                super().log_message(fmt, *args)

        def _send(self, code: int, body: bytes, ctype: str, extra: dict | None = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802 - stdlib name
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html", "/twin.html"):
                self._send(200, (static_dir / "twin.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/three.min.js":
                self._send(200, (static_dir / "three.min.js").read_bytes(), "application/javascript")
            elif path == "/state":
                self._send(200, store.latest()[1], "application/json")
            elif path == "/events":
                self._events()
            else:
                self._send(404, b"not found", "text/plain")

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            seen = -1
            try:
                while True:
                    seq, payload = store.latest()
                    if seq != seen:
                        self.wfile.write(b"data: " + payload + b"\n\n")
                        self.wfile.flush()
                        seen = seq
                    else:
                        store.wait_for_change(seen, timeout=1.0)
                        if store.latest()[0] == seen:       # keep proxies from closing an idle stream
                            self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def lan_addresses() -> list[str]:
    """Best-effort list of this host's non-loopback IPv4 addresses."""
    addrs = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(info[4][0])
    except socket.gaierror:
        pass
    try:  # the interface that routes outward, even if the hostname does not resolve to it
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        addrs.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(a for a in addrs if not a.startswith("127."))


def serve(store: StateStore, host: str = "0.0.0.0", port: int = 8765) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_handler(store))
    server.daemon_threads = True
    return server
