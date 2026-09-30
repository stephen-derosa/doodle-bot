"""Turning servo ticks into a trustworthy pen position.

Three stages, each optional after the first:

1. **Reference pose** (`doodle calib pose`): put the arm by hand into the
   "L pose" - shoulder_pan centred, upper arm plumb, forearm horizontal,
   pen pointing straight down - and record the ticks. That fixes the zero
   offset of every joint to within a couple of degrees.

2. **Paper frame** (`doodle calib paper`): with torque off, touch the pen
   tip to three corners of the box drawn on the page. FK of the recorded
   ticks gives those corners in the world frame, and stepping back by the
   box's offset gives the paper's origin and axes. The box is used rather
   than the sheet because all four of its corners are marked and within
   reach, where the sheet's far corners are ~350 mm out. The measured
   corner-to-corner distances versus the drawn box are printed as a sanity
   check on the kinematic model.

3. **Refinement** (`doodle calib refine`): touch a grid of known points on
   the paper and fit the joint offsets and the tool length so FK matches.
   A small Levenberg-Marquardt on top of numpy; no scipy on the Pi.
"""
from __future__ import annotations

import copy
import itertools
import math
from dataclasses import dataclass, field, replace

import numpy as np

from .arm import RAD_PER_TICK
from .servo import TICKS_PER_REV
from .config import Calibration, Config, PaperFrame, Tool
from .kinematics import PEN_DOWN_ANGLE, SO101Kinematics


def _roll_axis_at_l_pose(pen_angle_deg: float) -> float:
    """Roll axis angle when the pen points straight down.

    The L pose fixes the *pen*, not the roll axis. A pen that hangs square off
    the roll horn (pen_angle_deg = -90) is vertical while the roll axis is level.
    """
    return PEN_DOWN_ANGLE - math.radians(pen_angle_deg)


def reference_pose_q(cfg: Config) -> np.ndarray:
    """URDF joint angles of the L pose (pan centred, upper arm vertical,
    forearm horizontal, pen straight down, roll at its drawing value)."""
    A = np.radians([cfg.geometry.zero_angle1_deg, cfg.geometry.zero_angle2_deg, cfg.geometry.zero_angle3_deg])
    phi1, phi2, phi3 = math.pi / 2, 0.0, _roll_axis_at_l_pose(cfg.tool.pen_angle_deg)
    q1 = A[0] - phi1
    q2 = (A[1] - A[0]) - (phi2 - phi1)
    q3 = (A[2] - A[1]) - (phi3 - phi2)
    return np.array([0.0, q1, q2, q3, cfg.tool.roll_rad])


REFERENCE_POSE_HELP = """\
Put the arm in the L pose by hand (torque is off):
  * shoulder_pan  - arm pointing straight ahead, centred over the base
  * upper arm     - vertical (plumb / check with a square)
  * forearm       - horizontal, pointing forward
  * pen           - pointing straight down
  * wrist_roll    - pen holder centred so the pen sits in the arm plane
Then press Enter to record the servo positions."""


_IN_W = 28                  # interior width of the drawn sheet, in characters
_MARGIN = 11                # left gutter, leaves room for the vertical dimension
_CELL_ASPECT = 2.0          # a terminal row is roughly twice as tall as it is wide


def _sheet_rows(width: float, height: float) -> int:
    """Interior height, in rows, that renders `width` x `height` true to shape.

    Terminal cells are about twice as tall as they are wide, so drawing a
    portrait sheet needs proportionally fewer rows than columns. Deriving this
    from the configured size keeps the picture honest: the robot stands at the
    `width` edge, so a letter sheet in portrait must look portrait.
    """
    if width <= 0 or height <= 0:
        return 9
    return max(4, min(26, int(round(_IN_W * (height / width) / _CELL_ASPECT))))


def _hdim(span: int, label: str) -> str:
    """`<----- label ----->` filling exactly `span` characters."""
    lab = f" {label} "
    fill = max(0, span - 2 - len(lab))
    left = fill // 2
    return "<" + "-" * left + lab + "-" * (fill - left) + ">"


def _sheet_art(width: float, height: float, canvas):
    """Blank sheet with the drawn canvas box dotted in.

    Returns the character grid plus the mm -> cell mappers and the box bounds,
    so every calibration diagram places its marks the same way.
    """
    u0, v0, cw, ch = canvas
    in_h = _sheet_rows(width, height)
    rows = [list("+" + "-" * _IN_W + "+")]
    rows += [list("|" + " " * _IN_W + "|") for _ in range(in_h)]
    rows += [list("+" + "-" * _IN_W + "+")]

    def col(u):
        return min(max(1 + int(round(u / width * (_IN_W - 1))), 1), _IN_W) if width else 1

    def row(v):
        return min(max(in_h - int(round(v / height * (in_h - 1))), 1), in_h) if height else in_h

    cl, cr, cb, ct = col(u0), col(u0 + cw), row(v0), row(v0 + ch)
    for c in range(cl, cr + 1):
        rows[ct][c] = rows[cb][c] = "."
    for r in range(ct, cb + 1):
        rows[r][cl] = rows[r][cr] = "."
    return rows, col, row, (cl, cr, cb, ct)


def _finish(out, rows, extra_caption):
    """Common footer: robot marker, legend, caption."""
    pad = " " * _MARGIN
    out.append(pad + " " * max(0, _IN_W // 2 - 5) + "^^^ robot ^^^")
    out.append("")
    out.append(pad + "dotted box = the drawn canvas; solid box = the sheet")
    for line in extra_caption:
        out.append(pad + line)
    return "\n".join(out)


def paper_diagram(point: str, width: float, height: float, canvas) -> str:
    """Plan view of the sheet with the canvas box drawn and `point` marked X.

    `canvas` is (u0, v0, canvas_w, canvas_h) in mm. The three touch points are
    corners of that box rather than of the sheet: they are drawn in sharpie, so
    they are unambiguous to touch, and they all sit inside the arm's reach
    where the sheet's own far corners do not.

    The robot stands at the near (bottom) edge, so 'origin' is the box's
    near-left corner, 'x' its near-right corner and 'y' its far-left corner.
    """
    u0, v0, cw, ch = canvas
    rows, _, _, (cl, cr, cb, ct) = _sheet_art(width, height, canvas)
    spot = {"origin": (cb, cl), "x": (cb, cr), "y": (ct, cl)}[point]
    rows[spot[0]][spot[1]] = "X"

    out = []
    pad = " " * _MARGIN
    if point == "x":
        out.append(pad + " " * cl + _hdim(cr - cl + 1, f"{cw:.0f} mm"))
    body = [pad + "".join(r) for r in rows]
    if point == "y":
        label, mid = f"{ch:.0f} mm", (ct + cb) // 2
        for r in range(ct, cb + 1):
            gutter = {ct: "^", cb: "v", mid: f"| {label}"}.get(r, "|")
            body[r] = gutter.ljust(_MARGIN) + "".join(rows[r])
    out += body
    caption = {
        "origin": "X = box corner nearest the robot, on its LEFT  (paper origin + %.0f, %.0f mm)" % (u0, v0),
        "x": f"X = box corner nearest the robot, on its RIGHT: {cw:.0f} mm from the first",
        "y": f"X = far corner up the LEFT side of the box: {ch:.0f} mm from the first",
    }[point]
    return _finish(out, rows, [caption])


def grid_diagram(uv, all_uv, width: float, height: float, canvas,
                 label: str = "", index: int | None = None, total: int | None = None) -> str:
    """Plan view with every grid mark as 'o' and the one to touch now as X.

    Showing the whole grid, not just the active point, is what makes the
    sequence followable: the marks are all identical pencil dots on the page,
    so the picture has to say *which* one is next.
    """
    rows, col, row, _ = _sheet_art(width, height, canvas)
    for u, v in all_uv:
        rows[row(v)][col(u)] = "o"
    rows[row(uv[1])][col(uv[0])] = "X"

    pad = " " * _MARGIN
    out = [pad + "".join(r) for r in rows]
    head = f"X = point {label}" if label else "X = this point"
    if index is not None and total is not None:
        head += f"  ({index} of {total})"
    return _finish(out, rows, [
        head + f" at paper u={uv[0]:.0f} mm, v={uv[1]:.0f} mm",
        "o = the other grid marks, not yet touched",
    ])


def geometry_from_pose(q_chain, pen_angle_deg: float = 0.0) -> np.ndarray:
    """Inverse of `reference_pose_q`: link angles at q=0 implied by the L pose.

    A (the `zero_angle*` geometry) and the zero ticks are two parameterisations
    of the same single measurement, so one has to be pinned to solve the other.
    `reference_pose_q` pins A to the nominal design and solves the zero ticks;
    this solves A instead, for the caller that pins the zero ticks to the
    servos' own encoder centres. That makes the config describe the arm as
    actually built rather than as designed.

    Args:
        q_chain: measured [shoulder_lift, elbow_flex, wrist_flex] at the L pose.
        pen_angle_deg: the tool's pen angle from the roll axis (`Tool.pen_angle_deg`).

    Returns:
        [A0, A1, A2] in radians.
    """
    phi1, phi2, phi3 = math.pi / 2, 0.0, _roll_axis_at_l_pose(pen_angle_deg)
    q1, q2, q3 = (float(v) for v in q_chain)
    a0 = q1 + phi1
    a1 = q2 + a0 + phi2 - phi1
    a2 = q3 + a1 + phi3 - phi2
    return np.array([a0, a1, a2])


def zero_ticks_from_pose(ticks, q_pose, direction) -> np.ndarray:
    """zero = ticks - dir * q / rad_per_tick, so that ticks_to_q(ticks) == q_pose."""
    ticks = np.asarray(ticks, float)
    return ticks - np.asarray(direction, float) * np.asarray(q_pose, float) / RAD_PER_TICK


# --- paper frame --------------------------------------------------------------
@dataclass
class TouchPoint:
    label: str
    uv: tuple[float, float]         # nominal position on the paper (mm)
    ticks: list[float]              # servo ticks when the pen touched it
    world: list[float] = field(default_factory=list)  # FK result (filled in)


def paper_frame_from_corners(kin: SO101Kinematics, ticks_to_q, touches: list[TouchPoint],
                             width: float, height: float,
                             origin_uv: tuple[float, float] = (0.0, 0.0)) -> tuple[PaperFrame, dict]:
    """Solve the paper frame from three touched corners of a known rectangle.

    touches must carry the labels 'origin', 'x' and 'y'. They are the corners of
    a rectangle `width` x `height` whose 'origin' corner sits at `origin_uv` in
    paper coordinates, so the rectangle need not be the sheet itself: measuring
    the drawn canvas box instead keeps every touch on a real, marked corner well
    inside the arm's reach, where the sheet's own far corners are not.
    """
    pts = {}
    for t in touches:
        w = kin.fk(ticks_to_q(t.ticks))
        t.world = [float(v) for v in w]
        pts[t.label] = w
    o, px, py = pts["origin"], pts["x"], pts["y"]
    xv = px - o
    yv = py - o
    x_len, y_len = float(np.linalg.norm(xv)), float(np.linalg.norm(yv))
    x_axis = xv / x_len
    y_raw = yv - np.dot(yv, x_axis) * x_axis
    y_axis = y_raw / np.linalg.norm(y_raw)
    z_mean = float(np.mean([o[2], px[2], py[2]]))
    # Step back from the touched corner to the sheet's (0, 0).
    origin = o - origin_uv[0] * x_axis - origin_uv[1] * y_axis
    origin[2] = z_mean
    frame = PaperFrame(origin=[float(v) for v in origin], x_axis=[float(v) for v in x_axis],
                       y_axis=[float(v) for v in y_axis], calibrated=True)
    report = {
        "x_len_measured": x_len, "x_len_nominal": width, "x_err": x_len - width,
        "y_len_measured": y_len, "y_len_nominal": height, "y_err": y_len - height,
        "corner_angle_deg": math.degrees(math.acos(np.clip(np.dot(xv, yv) / (x_len * y_len), -1, 1))),
        "z_spread": float(max(o[2], px[2], py[2]) - min(o[2], px[2], py[2])),
        "z_mean": z_mean,
        "normal_z": float(np.cross(x_axis, y_axis)[2]),
    }
    return frame, report


# --- diagnosis ----------------------------------------------------------------
NOMINAL_BOX = None  # filled per call from the config


def box_metrics(pts) -> dict:
    """Width, height, corner angle and z statistics of three touched corners."""
    o, px, py = (np.asarray(p, float) for p in pts)
    xv, yv = px - o, py - o
    w, h = float(np.linalg.norm(xv)), float(np.linalg.norm(yv))
    ang = math.degrees(math.acos(np.clip(xv @ yv / max(w * h, 1e-9), -1, 1)))
    zs = [float(o[2]), float(px[2]), float(py[2])]
    return {"width": w, "height": h, "angle_deg": ang, "z_mean": float(np.mean(zs)),
            "z_spread": float(max(zs) - min(zs))}


def diagnose_signs(cfg: Config, cal: Calibration, tool_candidates=None) -> list[dict]:
    """Which joint-direction signs make the stored corner touches form the drawn box?

    Uses only evidence already on disk: the servo ticks at the L pose and at the
    three touched box corners. For every combination of pitch-joint signs the
    link geometry is re-solved from the L pose (a sign change moves it too), the
    touches are pushed through FK, and the result is scored against the box the
    user actually drew. A clear winner names the wrong sign; no winner says the
    error is elsewhere (lengths, tool, or the L pose itself).

    Returns hypotheses sorted best-first; each has 'flips', 'tool', metrics and
    a 'score' where 0 is a perfect box lying flat on the table.
    """
    if cal.pose_ticks is None or len(cal.paper_touches) < 3:
        raise ValueError("need cal.pose_ticks and three cal.paper_touches; re-run `calib pose` and `calib paper`")
    P = cfg.paper
    nominal = (P.canvas_width, P.canvas_height)
    by_label = {t["label"]: t for t in cal.paper_touches}
    ticks = [np.asarray(by_label[k]["ticks"], float) for k in ("origin", "x", "y")]
    pose = np.asarray(cal.pose_ticks, float)
    zero = np.asarray(cal.zero_ticks, float)
    tools = list(tool_candidates) if tool_candidates else [cfg.tool.along]
    out = []
    for flips in itertools.product([1, -1], repeat=3):
        d = np.asarray(cal.direction, float).copy()
        d[1:4] = flips
        # the geometry the L pose implies under these signs
        c = copy.deepcopy(cfg)
        A = geometry_from_pose(d[1:4] * (pose[1:4] - zero[1:4]) * RAD_PER_TICK, cfg.tool.pen_angle_deg)
        c.geometry.zero_angle1_deg, c.geometry.zero_angle2_deg, c.geometry.zero_angle3_deg = (
            math.degrees(v) for v in A)
        for tool in tools:
            c.tool.along = tool
            kin = SO101Kinematics.from_config(c)
            pts = [kin.fk(d * (t - zero) * RAD_PER_TICK) for t in ticks]
            m = box_metrics(pts)
            score = (abs(m["width"] - nominal[0]) / nominal[0] + abs(m["height"] - nominal[1]) / nominal[1]
                     + abs(m["angle_deg"] - 90.0) / 90.0 + m["z_spread"] / 50.0)
            out.append({"flips": {"shoulder_lift": flips[0], "elbow_flex": flips[1], "wrist_flex": flips[2]},
                        "tool": tool, **m, "score": float(score),
                        "geometry_deg": [math.degrees(v) for v in A]})
    out.sort(key=lambda h: h["score"])
    return out


# --- refinement fit ------------------------------------------------------------
def levenberg_marquardt(residual_fn, x0, iters=60, lam=1e-2, eps=1e-6, verbose=False, tol=1e-6):
    x = np.asarray(x0, float).copy()
    r = residual_fn(x)
    cost = float(r @ r)
    for it in range(iters):
        J = np.empty((len(r), len(x)))
        for j in range(len(x)):
            dx = np.zeros_like(x)
            dx[j] = eps
            J[:, j] = (residual_fn(x + dx) - residual_fn(x - dx)) / (2 * eps)
        A = J.T @ J
        g = J.T @ r
        improved = False
        for _ in range(12):
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            xn = x + step
            rn = residual_fn(xn)
            cn = float(rn @ rn)
            if cn < cost:
                gain = (cost - cn) / max(cost, 1e-30)
                x, r, cost = xn, rn, cn
                improved = gain > tol
                lam = max(lam / 3, 1e-9)
                break
            lam *= 4
        if verbose:
            print(f"  iter {it:2d} rms={math.sqrt(cost / len(r)):.4f} lam={lam:.1e}")
        if not improved or math.sqrt(cost / len(r)) < 1e-4:
            break
    return x, math.sqrt(cost / len(r))


@dataclass
class RefineResult:
    zero_ticks: np.ndarray
    tool_along: float
    paper: PaperFrame
    rms_before: float
    rms_after: float
    residuals: np.ndarray


MAX_TICK_SHIFT = 200.0      # ~17 deg; a joint zero from the L pose is not wronger than this
TOOL_BOUND_FRAC = 0.5       # the pen length is measured, so allow +-50% and no more


def refine_from_grid(cfg: Config, cal: Calibration, touches: list[TouchPoint],
                     fit_tool: bool = True, verbose: bool = False,
                     prior_mm_per_tick: float = 0.005,
                     tool_prior_mm_per_mm: float = 0.05) -> RefineResult:
    """Fit the zero offsets of shoulder_lift, elbow_flex and wrist_flex, the
    tool length, and the paper pose (x, y, z, yaw) so that FK(ticks) hits the
    nominal grid points. Needs >= 5 well-spread points; 9 (a 3x3 grid) is
    comfortable.

    The shoulder_pan offset is deliberately *not* fitted: a pan error is a
    rigid rotation of every point about the pan axis, which the paper pose
    absorbs exactly, so it is unobservable here and harmless for drawing.

    The wrist offset and the tool length are nearly interchangeable if every
    touch is made with the pen at the same angle. That degeneracy is real, so
    the fit is kept from sliding along it three ways: vary the pen tilt between
    touches; weak priors nudge the offsets back toward the reference pose
    (``prior_mm_per_tick``: 200 ticks costs 1 mm) and the tool length back
    toward its measured value (``tool_prior_mm_per_mm``: 20 mm costs 1 mm); and
    both are hard-bounded, which is what actually guarantees a physical answer.
    The priors stay weak on purpose so a genuine few-mm correction still lands;
    the bounds, not the priors, are what stop an unconstrained solve from
    walking the tool length out to metres while the residual barely moves."""
    if len(touches) < 5:
        raise ValueError("need at least 5 touch points to refine")
    direction = np.asarray(cal.direction, float)
    zero0 = np.asarray(cal.zero_ticks, float)
    tool0 = cal.tool_along if cal.tool_along is not None else cfg.tool.along
    T = np.array([t.ticks for t in touches], float)
    UV = np.array([t.uv for t in touches], float)

    # Anchor the bound and the prior to the *measured* pen length in the config,
    # never to the previous fit: anchoring to the fit lets each run drift from
    # the last, so repeated refines compound instead of converging.
    tool_ref = cfg.tool.along
    tool_lo = max(5.0, (1.0 - TOOL_BOUND_FRAC) * tool_ref)
    tool_hi = min(400.0, (1.0 + TOOL_BOUND_FRAC) * tool_ref)

    def unpack(x):
        # Clamping here is what keeps the solve physical: the degenerate
        # wrist-offset/tool-length direction is otherwise free to run away.
        # Bound the resulting zero against the servo centre, the fixed anchor
        # `calib pose` pins the chain joints to -- not against the previous fit,
        # or repeated refines drift a little further each time.
        dz = np.array([0.0, x[0], x[1], x[2]])
        centre = TICKS_PER_REV / 2
        lo, hi = centre - MAX_TICK_SHIFT - zero0[:4], centre + MAX_TICK_SHIFT - zero0[:4]
        dz[1:] = np.clip(dz[1:], lo[1:], hi[1:])
        tool_along = float(np.clip(x[3], tool_lo, tool_hi)) if fit_tool else tool0
        ox, oy, oz, yaw = x[4:8] if fit_tool else x[3:7]
        return dz, tool_along, ox, oy, oz, yaw

    def make_kin(tool_along):
        return SO101Kinematics.from_config(cfg, replace(cfg.tool, along=tool_along))

    def world_targets(ox, oy, oz, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([ox + UV[:, 0] * c - UV[:, 1] * s, oy + UV[:, 0] * s + UV[:, 1] * c,
                         np.full(len(UV), oz)], axis=1)

    def residual(x):
        dz, tool_along, ox, oy, oz, yaw = unpack(x)
        zero = zero0.copy()
        zero[:4] += dz
        kin = make_kin(tool_along)
        pts = np.array([kin.fk(direction * (t - zero) * RAD_PER_TICK) for t in T])
        geo = (pts - world_targets(ox, oy, oz, yaw)).ravel()
        priors = [prior_mm_per_tick * dz[1:]]
        if fit_tool:
            priors.append([tool_prior_mm_per_mm * (tool_along - tool_ref)])
        return np.concatenate([geo] + priors)

    # initial paper pose from the current model: fit a rigid 2D transform uv -> fk(xy)
    kin0 = make_kin(tool0)
    P0 = np.array([kin0.fk(direction * (t - zero0) * RAD_PER_TICK) for t in T])
    cu, cp = UV.mean(0), P0[:, :2].mean(0)
    H = (UV - cu).T @ (P0[:, :2] - cp)
    yaw0 = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    c, s = math.cos(yaw0), math.sin(yaw0)
    o0 = cp - np.array([cu[0] * c - cu[1] * s, cu[0] * s + cu[1] * c])
    x0 = [0, 0, 0] + ([float(np.clip(tool0, tool_lo, tool_hi))] if fit_tool else []) + [o0[0], o0[1], float(P0[:, 2].mean()), yaw0]
    n_geo = 3 * len(T)
    r0 = residual(np.array(x0, float))[:n_geo]
    rms_before = math.sqrt(float(r0 @ r0) / n_geo)
    x, _ = levenberg_marquardt(residual, x0, verbose=verbose)
    r1 = residual(x)[:n_geo]
    rms_after = math.sqrt(float(r1 @ r1) / n_geo)
    dz, tool_along, ox, oy, oz, yaw = unpack(x)
    zero = zero0.copy()
    zero[:4] += dz
    c, s = math.cos(yaw), math.sin(yaw)
    paper = PaperFrame(origin=[float(ox), float(oy), float(oz)], x_axis=[c, s, 0.0], y_axis=[-s, c, 0.0], calibrated=True)
    return RefineResult(zero, float(tool_along), paper, rms_before, rms_after, r1.reshape(-1, 3))


def grid_points(width: float, height: float, n: int = 3, inset: float = 15.0,
                u0: float = 0.0, v0: float = 0.0) -> list[tuple[str, tuple[float, float]]]:
    """n x n touch points inside a width x height box whose corner is at (u0, v0)
    on the paper. Use the canvas box, not the sheet: the far end of a letter
    sheet is beyond the arm's reach."""
    us = u0 + np.linspace(inset, width - inset, n)
    vs = v0 + np.linspace(inset, height - inset, n)
    return [(f"g{j}{i}", (float(u), float(v))) for j, v in enumerate(vs) for i, u in enumerate(us)]
