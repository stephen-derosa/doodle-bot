"""Stream a planned trajectory to the arm, watching it the whole time.

Safety behaviour:

* every joint target passes ``Arm.check_q`` (limits in rad *and* ticks)
* the approach from wherever the arm is to the first sample is simulated
  with FK first; if the pen would dip below the paper an intermediate hover
  waypoint is inserted
* while streaming, present positions are read back; a tracking error above
  ``abort_ticks`` (stall/collision) or an over-temperature aborts the run
* Ctrl-C or an abort lifts the pen straight up, then holds position
* the arm ends in the park pose with torque still on (an SO-101 collapses
  under gravity when torque is released); ``--torque-off`` releases it
"""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .arm import Arm, SafetyError
from .config import Calibration, Config
from .kinematics import SO101Kinematics
from .planner import Trajectory, approach_is_safe


@dataclass
class RunReport:
    ok: bool
    reason: str = ""
    duration_s: float = 0.0
    samples_sent: int = 0
    mean_period_ms: float = 0.0
    max_period_ms: float = 0.0
    late_ticks: int = 0
    max_track_err_ticks: float = 0.0
    rms_track_err_ticks: float = 0.0
    log_path: str | None = None
    extra: dict = field(default_factory=dict)


class Executor:
    def __init__(self, arm: Arm, cfg: Config, cal: Calibration, kin: SO101Kinematics | None = None,
                 realtime: bool = True, log_path: Path | str | None = None, verbose: bool = True,
                 abort_ticks: float = 250.0, warn_ticks: float = 60.0, max_temp_c: float = 65.0,
                 read_every: int = 1):
        self.arm, self.cfg, self.cal = arm, cfg, cal
        self.kin = kin or SO101Kinematics.from_config(cfg, cal=cal)
        if cal.tool_along is not None:
            self.kin.tool.along = cal.tool_along
        self.realtime, self.verbose = realtime, verbose
        self.log_path = Path(log_path) if log_path else None
        self.abort_ticks, self.warn_ticks, self.max_temp_c = abort_ticks, warn_ticks, max_temp_c
        self.read_every = max(1, read_every)
        self.prepared = False

    def say(self, *a):
        if self.verbose:
            print(*a, flush=True)

    # -- setup --------------------------------------------------------------
    def prepare(self) -> None:
        alive = self.arm.ping_all()
        missing = [n for n, ok in alive.items() if not ok]
        if missing:
            raise SafetyError(f"servos not answering: {missing}")
        self.arm.torque(False)
        self.arm.configure()
        self.arm.torque(True)
        st = self.arm.read_status()
        low = [n for n, s in st.items() if s["voltage"] < 10.5]
        if low:
            raise SafetyError(f"supply voltage low on {low}: {st}")
        self.prepared = True
        self.say("servos ready:", ", ".join(f"{n} {s['voltage']:.1f}V {s['temperature']:.0f}C" for n, s in st.items()))

    def paper_z(self) -> float:
        return float(self.cal.paper.origin[2])

    def approach(self, q_target, speed_deg_s: float | None = None) -> None:
        """Joint-space move that is guaranteed not to drag the pen through the paper."""
        q_now = self.arm.read_q()
        floor = self.paper_z() - 0.5
        ok, mz = approach_is_safe(self.kin, q_now, q_target, floor)
        if not ok:
            # hover: same xy as the target, well above the paper, then descend
            xyz = self.kin.fk(q_target)
            hover = self.kin.ik(xyz + np.array([0, 0, 40.0]), self.cfg.motion.pen_tilt_options_deg)
            if hover.ok and approach_is_safe(self.kin, q_now, hover.q, floor)[0]:
                self.say(f"approach would dip to z={mz:.1f}; going via hover waypoint")
                self.arm.move_to_q(hover.q, speed_deg_s)
            else:
                # last resort: lift straight up from the current position first
                here = self.kin.fk(q_now)
                up = self.kin.ik(here + np.array([0, 0, 40.0]), self.cfg.motion.pen_tilt_options_deg)
                if not up.ok:
                    raise SafetyError(f"cannot find a safe approach (min z {mz:.1f} mm)")
                self.say("approach would dip below the paper; lifting first")
                self.arm.move_to_q(up.q, speed_deg_s)
        self.arm.move_to_q(q_target, speed_deg_s)
        self.arm.wait_settled()

    # -- the main loop ------------------------------------------------------------
    def run(self, traj: Trajectory) -> RunReport:
        if not self.prepared:
            self.prepare()
        dt = 1.0 / self.cfg.motion.rate_hz
        self.say(f"approaching start pose ({traj.duration():.1f} s drawing, {len(traj.t)} samples)")
        self.approach(traj.q[0])

        log_f = log_w = None
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log_f = open(self.log_path, "w", newline="")
            log_w = csv.writer(log_f)
            log_w.writerow(["t", "pen_down", *[f"cmd_{n}" for n in self.arm.names], *[f"act_{n}" for n in self.arm.names],
                            "x", "y", "z"])
        periods: list[float] = []
        errs: list[float] = []
        late = 0
        k = 0
        rep = RunReport(ok=True)
        t_start = time.monotonic()
        last = t_start
        try:
            while True:
                t_rel = k * dt
                if t_rel > traj.t[-1]:
                    break
                q = traj.q_at(t_rel)
                cmd = self.arm.command_q(q, f"(t={t_rel:.2f}s)")
                act = None
                if k % self.read_every == 0:
                    act = self.arm.read_ticks()
                    err = np.abs(act - cmd)
                    errs.append(float(err.max()))
                    if err.max() > self.abort_ticks and t_rel > 0.5:
                        i = int(err.argmax())
                        raise SafetyError(f"{self.arm.names[i]} lagging {err.max():.0f} ticks (stall or collision?)")
                    if k % (25 * self.read_every) == 0 and k > 0:
                        temps = self.arm.bus.sync_read("present_temperature", self.arm.ids)
                        hot = {n: t for n, t in zip(self.arm.names, temps.values()) if t > self.max_temp_c}
                        if hot:
                            raise SafetyError(f"over temperature: {hot}")
                if log_w:
                    idx = min(int(np.searchsorted(traj.t, t_rel)), len(traj.t) - 1)
                    log_w.writerow([f"{t_rel:.3f}", int(traj.pen_down[idx]), *cmd.tolist(),
                                    *([""] * 5 if act is None else [int(v) for v in act]), *np.round(traj.xyz[idx], 2)])
                    if k % 25 == 0:
                        log_f.flush()       # lets `doodle twin --follow` tail the drawing live
                if self.verbose and k % int(5 / dt) == 0 and k > 0:
                    self.say(f"  t={t_rel:5.1f}/{traj.t[-1]:.1f}s  track err {errs[-1] if errs else 0:.0f} ticks")
                k += 1
                if self.realtime:
                    nxt = t_start + k * dt
                    now = time.monotonic()
                    if nxt > now:
                        time.sleep(nxt - now)
                    else:
                        late += 1
                    now = time.monotonic()
                    periods.append(now - last)
                    last = now
        except KeyboardInterrupt:
            rep = RunReport(ok=False, reason="interrupted by user")
            self.abort("Ctrl-C")
        except SafetyError as e:
            rep = RunReport(ok=False, reason=str(e))
            self.abort(str(e))
        finally:
            if log_f:
                log_f.close()
        if rep.ok:
            self.arm.wait_settled()
        rep.duration_s = time.monotonic() - t_start
        rep.samples_sent = k
        if periods:
            rep.mean_period_ms = 1000 * float(np.mean(periods))
            rep.max_period_ms = 1000 * float(np.max(periods))
        rep.late_ticks = late
        if errs:
            e = np.array(errs)
            rep.max_track_err_ticks = float(e.max())
            rep.rms_track_err_ticks = float(np.sqrt(np.mean(e ** 2)))
        rep.log_path = str(self.log_path) if self.log_path else None
        return rep

    # -- recovery -------------------------------------------------------------------
    def abort(self, why: str) -> None:
        self.say(f"\nABORT: {why} -> lifting pen")
        try:
            q_now = self.arm.read_q()
            here = self.kin.fk(q_now)
            up = self.kin.ik(here + np.array([0, 0, self.cfg.motion.pen_up_z + 15.0]),
                             self.cfg.motion.pen_tilt_options_deg)
            if up.ok:
                self.arm.move_to_q(up.q, speed_deg_s=15.0)
            else:
                # can't solve IK from here: at least stop where we are
                self.arm.command_q(q_now, "(abort hold)")
        except Exception as e:  # never let recovery itself throw past the caller
            self.say(f"  (recovery move failed: {e})")

    def park(self, torque_off: bool = False) -> None:
        park = np.radians(self.cfg.park_pose_deg)
        self.say("parking")
        self.approach(park)
        if torque_off:
            self.arm.torque(False)
            self.say("torque released")
