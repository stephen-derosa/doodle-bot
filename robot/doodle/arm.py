"""Named-joint view of the servo bus with the safety checks in one place.

Everything above this layer thinks in URDF joint angles (radians). Everything
below thinks in servo ticks. ``Arm`` is the only converter, and the only code
that is allowed to send a goal position.
"""
from __future__ import annotations

import math
import time

import numpy as np

from .config import Calibration, Config
from .servo import TICKS_PER_REV, FakeBus, FeetechBus
from .telemetry import Tap

RAD_PER_TICK = 2 * math.pi / TICKS_PER_REV


class SafetyError(RuntimeError):
    pass


class Arm:
    def __init__(self, bus: FeetechBus, cfg: Config, cal: Calibration):
        self.bus, self.cfg, self.cal = bus, cfg, cal
        self.ids = cfg.ids
        self.names = cfg.joint_names
        self.zero = np.asarray(cal.zero_ticks, float)
        self.dir = np.asarray(cal.direction, float)
        self.min_ticks = np.array([j.min_ticks for j in cfg.joints])
        self.max_ticks = np.array([j.max_ticks for j in cfg.joints])
        self.min_rad = np.array([j.min_rad for j in cfg.joints])
        self.max_rad = np.array([j.max_rad for j in cfg.joints])
        # Measured travel wins. Limits stored in radians are relative to q, and
        # q is defined by the zero ticks, so any recalibration silently moves
        # them; the tick window is the same physical travel whatever q means.
        for i, j in enumerate(cfg.joints):
            if not j.travel_measured:
                continue
            a = self.dir[i] * (j.min_ticks - self.zero[i]) * RAD_PER_TICK
            b = self.dir[i] * (j.max_ticks - self.zero[i]) * RAD_PER_TICK
            self.min_rad[i], self.max_rad[i] = min(a, b), max(a, b)
        self._last_cmd_ticks: np.ndarray | None = None
        # Whoever owns the bus broadcasts what it reads, so a viewer never has to.
        self.tap = Tap()

    # -- conversions ----------------------------------------------------------
    def ticks_to_q(self, ticks) -> np.ndarray:
        return self.dir * (np.asarray(ticks, float) - self.zero) * RAD_PER_TICK

    def q_to_ticks(self, q) -> np.ndarray:
        return self.zero + self.dir * np.asarray(q, float) / RAD_PER_TICK

    def check_q(self, q, where: str = "") -> None:
        q = np.asarray(q, float)
        if not np.all(np.isfinite(q)):
            raise SafetyError(f"non-finite joint target {q} {where}")
        for i in range(len(q)):
            if not (self.min_rad[i] - 1e-6 <= q[i] <= self.max_rad[i] + 1e-6):
                raise SafetyError(
                    f"{self.names[i]} = {math.degrees(q[i]):.1f} deg outside "
                    f"[{math.degrees(self.min_rad[i]):.0f}, {math.degrees(self.max_rad[i]):.0f}] {where}")
        t = self.q_to_ticks(q)
        bad = (t < self.min_ticks) | (t > self.max_ticks)
        if bad.any():
            i = int(np.argmax(bad))
            raise SafetyError(f"{self.names[i]} tick target {t[i]:.0f} outside "
                              f"[{self.min_ticks[i]}, {self.max_ticks[i]}] {where}")

    # -- bus i/o ----------------------------------------------------------------
    def ping_all(self) -> dict[str, bool]:
        return {n: self.bus.ping(i) for n, i in zip(self.names, self.ids)}

    def read_ticks(self) -> np.ndarray:
        d = self.bus.sync_read("present_position", self.ids)
        ticks = np.array([d[i] for i in self.ids], float)
        self.tap.send(ticks)
        return ticks

    def read_q(self) -> np.ndarray:
        return self.ticks_to_q(self.read_ticks())

    def read_status(self) -> dict[str, dict[str, float]]:
        out: dict[str, dict[str, float]] = {}
        v = self.bus.sync_read("present_voltage", self.ids)
        t = self.bus.sync_read("present_temperature", self.ids)
        ld = self.bus.sync_read("present_load", self.ids)
        for n, i in zip(self.names, self.ids):
            out[n] = {"voltage": v[i] / 10.0, "temperature": t[i], "load": ld[i]}
        return out

    def torque(self, on: bool) -> None:
        self.bus.torque_all(self.ids, on)
        if on:
            # a freshly enabled servo must not jump: make its goal its present position
            self._last_cmd_ticks = self.read_ticks()
            self.bus.sync_write("goal_position", {i: int(round(t)) for i, t in zip(self.ids, self._last_cmd_ticks)})

    def configure(self) -> None:
        """Write the drawing-friendly servo tuning. Torque must be off."""
        s = self.cfg.servo
        for i in self.ids:
            self.bus.write(i, "operating_mode", 0)          # position mode
            self.bus.write(i, "p_gain", s.p_gain)
            self.bus.write(i, "i_gain", s.i_gain)
            self.bus.write(i, "d_gain", s.d_gain)
            self.bus.write(i, "acceleration", s.acceleration)
            self.bus.write(i, "goal_velocity", s.goal_velocity)
            self.bus.write(i, "torque_limit", s.torque_limit)
            self.bus.write(i, "return_delay", s.return_delay)

    def command_q(self, q, where: str = "") -> np.ndarray:
        """Send one joint-space setpoint. Returns the ticks that were sent."""
        self.check_q(q, where)
        t = np.round(self.q_to_ticks(q)).astype(int)
        self.bus.sync_write("goal_position", {i: int(v) for i, v in zip(self.ids, t)})
        self._last_cmd_ticks = t.astype(float)
        return t

    def move_to_q(self, q_target, speed_deg_s: float | None = None, rate_hz: float | None = None,
                  on_step=None) -> None:
        """Blocking joint-space move from wherever the arm is, at a bounded joint speed."""
        speed = math.radians(speed_deg_s or self.cfg.motion.approach_speed_deg_s)
        rate = rate_hz or self.cfg.motion.rate_hz
        q_target = np.asarray(q_target, float)
        self.check_q(q_target, "(move_to target)")
        q0 = self.read_q()
        dist = float(np.max(np.abs(q_target - q0)))
        n = max(1, int(math.ceil(dist / speed * rate)))
        t0 = time.monotonic()
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)  # smoothstep
            self.command_q(q0 + (q_target - q0) * s, "(move_to)")
            if on_step:
                on_step(k, n)
            nxt = t0 + k / rate
            dt = nxt - time.monotonic()
            if dt > 0:
                time.sleep(dt)

    def settled(self, tol_ticks: float = 12.0) -> bool:
        if self._last_cmd_ticks is None:
            return True
        return bool(np.all(np.abs(self.read_ticks() - self._last_cmd_ticks) <= tol_ticks))

    def wait_settled(self, timeout: float = 3.0, tol_ticks: float = 12.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.settled(tol_ticks):
                return True
            time.sleep(0.02)
        return False


def open_arm(cfg: Config, cal: Calibration, dry_run: bool = False) -> Arm:
    if dry_run:
        bus = FakeBus(cfg.ids, start_ticks=2048)
        # start the fake arm at the park pose so dry runs look like real ones
        arm = Arm(bus, cfg, cal)
        park = np.radians(cfg.park_pose_deg)
        for i, t in zip(cfg.ids, arm.q_to_ticks(park)):
            bus.mem[i]["present_position"] = int(round(t))
            bus.mem[i]["goal_position"] = int(round(t))
        return arm
    bus = FeetechBus(cfg.port, cfg.baud).open()
    return Arm(bus, cfg, cal)
