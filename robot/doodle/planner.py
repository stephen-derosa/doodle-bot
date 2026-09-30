"""Drawing -> toolpath -> timed joint trajectory.

Pipeline for ``plan()``:

1. scale/centre the drawing into the canvas (uniform scale, keeps aspect)
2. order strokes greedily to shorten pen-up travel (strokes may be reversed)
3. build 3D segments in canvas coordinates: travel at ``pen_up_z``, plunge,
   draw at ``-pen_press``, lift, with dwells at each pen transition
4. time-parameterise every segment with an acceleration-limited profile and
   corner slow-down (classic forward/backward look-ahead pass)
5. sample at the control rate, map canvas -> paper -> world, solve IK
6. check joint speeds and dilate time if any joint would be too fast

Anything unreachable raises ``PlanError`` *before* the arm moves.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import Calibration, Config
from .kinematics import SO101Kinematics
from .shapes import Drawing, Stroke


class PlanError(RuntimeError):
    pass


@dataclass
class Segment:
    pts: np.ndarray      # (N, 3) canvas u, v, z(mm above paper plane)
    speed: float         # mm/s cruise
    pen_down: bool
    dwell_after: float = 0.0


@dataclass
class Trajectory:
    t: np.ndarray                 # (N,) seconds
    q: np.ndarray                 # (N, 5) rad
    xyz: np.ndarray               # (N, 3) world mm
    pen_down: np.ndarray          # (N,) bool
    tilt_deg: np.ndarray          # (N,)
    segments: list[Segment] = field(default_factory=list)
    canvas_uv0: tuple[float, float] = (0.0, 0.0)
    stats: dict = field(default_factory=dict)

    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    def q_at(self, tt: float) -> np.ndarray:
        """Linear interpolation of the joint vector at time tt (clamped)."""
        tt = min(max(tt, self.t[0]), self.t[-1])
        i = int(np.searchsorted(self.t, tt, side="right")) - 1
        i = min(max(i, 0), len(self.t) - 2)
        dt = self.t[i + 1] - self.t[i]
        a = 0.0 if dt <= 0 else (tt - self.t[i]) / dt
        return self.q[i] + (self.q[i + 1] - self.q[i]) * a

    def preview_segments(self):
        return [(s.pts[:, :2], s.pen_down) for s in self.segments]


# --- stroke ordering ---------------------------------------------------------------
def order_strokes(strokes: list[Stroke], start=(0.0, 0.0)) -> list[Stroke]:
    remaining = [s for s in strokes if len(s.points) > 0]
    out: list[Stroke] = []
    pos = np.asarray(start, float)
    while remaining:
        best, best_d, rev = None, math.inf, False
        for k, s in enumerate(remaining):
            d0 = float(np.linalg.norm(s.points[0] - pos))
            d1 = float(np.linalg.norm(s.points[-1] - pos))
            if d0 < best_d:
                best, best_d, rev = k, d0, False
            if d1 < best_d:
                best, best_d, rev = k, d1, True
        s = remaining.pop(best)  # type: ignore[arg-type]
        pts = s.points[::-1].copy() if rev else s.points.copy()
        out.append(Stroke(pts))
        pos = pts[-1]
    return out


def densify(pts: np.ndarray, max_step: float) -> np.ndarray:
    """Insert points so no segment is longer than max_step (keeps corners)."""
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        L = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(L / max_step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)


# --- velocity profile ----------------------------------------------------------------
def time_parameterise(pts: np.ndarray, v_max: float, accel: float, v_corner_min: float,
                      v_start: float = 0.0, v_end: float = 0.0) -> np.ndarray:
    """Return the time at each vertex for a polyline traversed with speed
    limits: v_max on straights, slower through corners, |accel| bounded."""
    n = len(pts)
    if n == 1:
        return np.zeros(1)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    if n == 2 and v_start == 0.0 and v_end == 0.0:
        # a bare two-vertex move: triangular/trapezoidal profile by formula
        L = float(seg[0])
        if L <= 1e-9:
            return np.zeros(2)
        v_peak = min(v_max, math.sqrt(accel * L))
        return np.array([0.0, L / v_peak + v_peak / accel])
    v = np.full(n, v_max, float)
    # corner limits from the turning angle at each interior vertex
    for i in range(1, n - 1):
        a, b = pts[i] - pts[i - 1], pts[i + 1] - pts[i]
        la, lb = np.linalg.norm(a), np.linalg.norm(b)
        if la < 1e-9 or lb < 1e-9:
            v[i] = v_corner_min
            continue
        cosang = float(np.clip(np.dot(a, b) / (la * lb), -1, 1))
        ang = math.acos(cosang)                      # 0 = straight
        # allow full speed up to ~5 deg, corner_min at >= 90 deg, linear between
        f = 1.0 - min(1.0, max(0.0, (math.degrees(ang) - 5.0) / 85.0))
        v[i] = max(v_corner_min, v_max * f)
    v[0], v[-1] = v_start, v_end
    # forward pass (acceleration), backward pass (deceleration)
    for i in range(1, n):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * accel * seg[i - 1]))
    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * accel * seg[i]))
    t = np.zeros(n)
    for i in range(1, n):
        vm = (v[i - 1] + v[i]) / 2
        t[i] = t[i - 1] + (seg[i - 1] / vm if vm > 1e-9 else 0.0)
    return t


def sample_polyline(pts: np.ndarray, t_vertex: np.ndarray, dt: float, t0: float):
    """Sample position along a time-parameterised polyline at a fixed step."""
    T = t_vertex[-1]
    if T <= 0:
        return np.array([t0]), pts[:1].copy()
    ts = np.arange(0.0, T, dt)
    out = np.empty((len(ts), pts.shape[1]))
    for k in range(pts.shape[1]):
        out[:, k] = np.interp(ts, t_vertex, pts[:, k])
    return ts + t0, out


# --- the planner ------------------------------------------------------------------------
def build_segments(drawing: Drawing, cfg: Config) -> list[Segment]:
    m = cfg.motion
    P = cfg.paper
    fitted = drawing.fit_to(P.canvas_width, P.canvas_height, P.margin)
    strokes = order_strokes(fitted.strokes, start=(0.0, P.canvas_height))
    segs: list[Segment] = []
    zu, zd = m.pen_up_z, -m.pen_press
    for s in strokes:
        pts = densify(s.points, 2.0)
        first, last = pts[0], pts[-1]
        prev_xy = segs[-1].pts[-1, :2] if segs else first
        # travel (pen up) to above the stroke start
        travel = np.array([[*prev_xy, zu], [*first, zu]])
        if np.linalg.norm(travel[0] - travel[1]) > 1e-6:
            segs.append(Segment(travel, m.travel_speed, False))
        # plunge
        segs.append(Segment(np.array([[*first, zu], [*first, zd]]), m.plunge_speed, False, dwell_after=m.dwell_s))
        # draw
        segs.append(Segment(np.column_stack([pts, np.full(len(pts), zd)]), m.draw_speed, True, dwell_after=m.dwell_s))
        # lift
        segs.append(Segment(np.array([[*last, zd], [*last, zu]]), m.plunge_speed, False))
    return segs


def plan(drawing: Drawing, cfg: Config, cal: Calibration, kin: SO101Kinematics | None = None) -> Trajectory:
    m = cfg.motion
    kin = kin or SO101Kinematics.from_config(cfg, cal=cal)
    if cal.tool_along is not None:
        kin.tool.along = cal.tool_along
    segs = build_segments(drawing, cfg)
    dt = 1.0 / m.rate_hz
    u0, v0 = cfg.paper.canvas_origin()
    frame = cal.paper

    T, UVZ, DOWN = [], [], []
    t_cursor = 0.0
    for s in segs:
        s.pts = densify(s.pts, 1.0)   # vertices every <=1 mm so the profile can ramp
        tv = time_parameterise(s.pts, s.speed, m.accel, m.corner_speed)
        ts, ps = sample_polyline(s.pts, tv, dt, t_cursor)
        T.append(ts)
        UVZ.append(ps)
        DOWN.append(np.full(len(ts), s.pen_down))
        t_cursor = t_cursor + max(tv[-1], dt)
        # end point + dwell
        n_dwell = int(round(s.dwell_after / dt))
        T.append(t_cursor + np.arange(n_dwell + 1) * dt)
        UVZ.append(np.repeat(s.pts[-1:], n_dwell + 1, axis=0))
        DOWN.append(np.full(n_dwell + 1, s.pen_down))
        t_cursor += (n_dwell + 1) * dt
    t = np.concatenate(T)
    uvz = np.vstack(UVZ)
    down = np.concatenate(DOWN)

    xyz = np.array([frame.to_world(u0 + u, v0 + v, z) for u, v, z in uvz])
    q = np.empty((len(xyz), 5))
    tilt = np.empty(len(xyz))
    failures: list[str] = []
    for i, p in enumerate(xyz):
        res = kin.ik(p, m.pen_tilt_options_deg)
        if not res.ok:
            if len(failures) < 5:
                failures.append(f"canvas ({uvz[i,0]:.1f}, {uvz[i,1]:.1f}, z{uvz[i,2]:+.1f}) -> world "
                                f"({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}): {res.reason}")
            q[i] = np.nan
            continue
        q[i], tilt[i] = res.q, res.tilt_deg
    if failures:
        n_bad = int(np.isnan(q[:, 0]).sum())
        raise PlanError(f"{n_bad}/{len(q)} samples unreachable. First few:\n  " + "\n  ".join(failures)
                        + "\nMove the paper / canvas or shrink the canvas (see `doodle layout`).")

    # joint speed check -> dilate time if necessary
    dq = np.abs(np.diff(q, axis=0)) / np.maximum(np.diff(t)[:, None], 1e-9)
    max_speed = float(np.degrees(dq.max())) if len(dq) else 0.0
    dilation = 1.0
    if max_speed > m.max_joint_speed_deg_s:
        dilation = max_speed / m.max_joint_speed_deg_s
        t = t * dilation
    ink = sum(float(np.sum(np.linalg.norm(np.diff(s.pts[:, :2], axis=0), axis=1))) for s in segs if s.pen_down)
    travel = sum(float(np.sum(np.linalg.norm(np.diff(s.pts[:, :2], axis=0), axis=1))) for s in segs if not s.pen_down)
    stats = {
        "samples": int(len(t)), "duration_s": float(t[-1]), "ink_mm": ink, "travel_mm": travel,
        "strokes": int(sum(1 for s in segs if s.pen_down)),
        "max_joint_speed_deg_s": max_speed / dilation, "time_dilation": dilation,
        "tilted_samples": int(np.sum(np.abs(tilt) > 1e-9)), "max_tilt_deg": float(np.abs(tilt).max()),
        "canvas_uv0": (u0, v0),
    }
    return Trajectory(t, q, xyz, down, tilt, segs, (u0, v0), stats)


def approach_is_safe(kin: SO101Kinematics, q_from, q_to, floor_z: float, steps: int = 40) -> tuple[bool, float]:
    """Would a straight joint-space move dip the pen below floor_z? Returns (ok, min z)."""
    zs = [kin.fk(q_from + (np.asarray(q_to) - np.asarray(q_from)) * (k / steps))[2] for k in range(steps + 1)]
    mz = float(min(zs))
    return mz >= floor_z, mz
