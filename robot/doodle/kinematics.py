"""SO-101 kinematics for pen plotting.

World frame: origin on the table directly under the shoulder_pan axis, z up,
x pointing "forward" (the direction the arm faces at pan = 0).

The three pitch joints (shoulder_lift, elbow_flex, wrist_flex) share parallel
axes, so once shoulder_pan has chosen the vertical plane the arm is a 3-link
planar chain in (r, z). The pen sits on the wrist_roll axis, which the URDF
puts back in the plane through the pan axis (the 18 mm lateral offset of the
shoulder/elbow/wrist servos cancels at the wrist_roll frame), so
``pan = atan2(y, x)`` exactly for an on-axis pen.

Angle convention: ``q`` is the URDF joint angle (radians), except that pan
(``q[0]``) is positive counter-clockwise viewed from above (the URDF's pan
axis points down, so its sign is flipped; see ``URDF_SIGN``). In-plane absolute
link angles ``phi`` are measured from +r towards +z, and the URDF's pitch
joints rotate the *opposite* way, so ``phi_i = phi_i(0) - q_i`` cumulatively.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .arm import joint_limits_rad
from .config import Calibration, Config, Geometry, Tool

PEN_DOWN_ANGLE = -math.pi / 2  # phi3 for a pen pointing straight at the table


@dataclass
class IKResult:
    q: np.ndarray            # 5 joint angles (rad), URDF convention
    ok: bool
    tilt_deg: float = 0.0    # pen tilt actually used (0 = vertical)
    reason: str = ""


class SO101Kinematics:
    def __init__(self, geom: Geometry, tool: Tool, limits: list[tuple[float, float]]):
        self.g = geom
        self.tool = tool
        self.limits = limits  # [(min, max)] * 5 in rad
        self.A = np.radians([geom.zero_angle1_deg, geom.zero_angle2_deg, geom.zero_angle3_deg])

    @classmethod
    def from_config(cls, cfg: Config, tool: Tool | None = None,
                    cal: Calibration | None = None) -> "SO101Kinematics":
        """Pass `cal` to plan against the measured travel the arm enforces;
        without it the config's radian limits are used as they stand."""
        if cal is None:
            limits = [(j.min_rad, j.max_rad) for j in cfg.joints]
        else:
            limits = list(zip(*joint_limits_rad(cfg, cal)))
        return cls(cfg.geometry, tool or cfg.tool, limits)

    # -- forward ------------------------------------------------------------
    def link_angles(self, q) -> tuple[float, float, float]:
        q = np.asarray(q, float)
        phi1 = self.A[0] - q[1]
        phi2 = phi1 + (self.A[1] - self.A[0]) - q[2]
        phi3 = phi2 + (self.A[2] - self.A[1]) - q[3]
        return phi1, phi2, phi3

    def planar_points(self, q):
        """Joint positions in the arm plane: lift, elbow, wrist_flex, roll origin, tip (r,z)."""
        g = self.g
        phi1, phi2, phi3 = self.link_angles(q)
        p0 = np.array([g.lift_r, g.lift_z])
        u = lambda a: np.array([math.cos(a), math.sin(a)])
        n = lambda a: np.array([-math.sin(a), math.cos(a)])
        p1 = p0 + g.a1 * u(phi1)
        p2 = p1 + g.a2 * u(phi2)
        p3 = p2 + g.a3 * u(phi3)
        roll = q[4] - self.tool.roll_rad
        perp_inplane = self.tool.perp * math.cos(roll) - self.tool.lateral * math.sin(roll)
        tip = p3 + self.tool.along * u(phi3) + perp_inplane * n(phi3)
        return p0, p1, p2, p3, tip

    def tip_lateral(self, q) -> float:
        roll = q[4] - self.tool.roll_rad
        return self.tool.perp * math.sin(roll) + self.tool.lateral * math.cos(roll)

    def fk(self, q) -> np.ndarray:
        """Pen tip position in the world frame (mm)."""
        q = np.asarray(q, float)
        *_, tip = self.planar_points(q)
        r, z = tip
        lat = self.tip_lateral(q)
        c, s = math.cos(q[0]), math.sin(q[0])
        return np.array([r * c - lat * s, r * s + lat * c, z])

    @property
    def pen_offset(self) -> float:
        """Pen body angle minus roll axis angle, in the arm plane (rad)."""
        return math.radians(self.tool.pen_angle_deg)

    def pen_direction(self, q) -> np.ndarray:
        phi3 = self.link_angles(q)[2] + self.pen_offset
        c, s = math.cos(q[0]), math.sin(q[0])
        return np.array([math.cos(phi3) * c, math.cos(phi3) * s, math.sin(phi3)])

    # -- inverse ------------------------------------------------------------
    def ik_planar(self, r: float, z: float, phi3: float) -> tuple[np.ndarray | None, str]:
        """Solve q1..q3 (lift, elbow, wrist_flex) for tip (r,z) with pen angle phi3.
        Elbow-up branch only (the one the SO-101 physically uses)."""
        g = self.g
        u = np.array([math.cos(phi3), math.sin(phi3)])
        n = np.array([-math.sin(phi3), math.cos(phi3)])
        perp_inplane = self.tool.perp  # roll assumed at tool.roll_rad while drawing
        wrist = np.array([r, z]) - (g.a3 + self.tool.along) * u - perp_inplane * n
        d = wrist - np.array([g.lift_r, g.lift_z])
        D = float(np.hypot(*d))
        if D > g.a1 + g.a2 - 1e-6:
            return None, f"out of reach (need {D:.1f} mm, max {g.a1 + g.a2:.1f})"
        if D < abs(g.a1 - g.a2) + 1e-6:
            return None, "too close to the shoulder"
        cos_beta = (g.a1 ** 2 + g.a2 ** 2 - D ** 2) / (2 * g.a1 * g.a2)
        beta = math.acos(max(-1.0, min(1.0, cos_beta)))   # interior elbow angle
        rel = -(math.pi - beta)                              # link2 relative to link1, elbow up
        gamma = math.atan2(d[1], d[0])
        cos_alpha = (g.a1 ** 2 + D ** 2 - g.a2 ** 2) / (2 * g.a1 * D)
        alpha = math.acos(max(-1.0, min(1.0, cos_alpha)))
        phi1 = gamma + alpha
        phi2 = phi1 + rel
        q1 = self.A[0] - phi1
        q2 = (self.A[1] - self.A[0]) - rel
        q3 = (self.A[2] - self.A[1]) - (phi3 - phi2)
        return np.array([q1, q2, q3]), ""

    def within_limits(self, q) -> tuple[bool, str]:
        for i, (lo, hi) in enumerate(self.limits):
            if not (lo - 1e-9 <= q[i] <= hi + 1e-9):
                return False, f"joint {i} = {math.degrees(q[i]):.1f} deg outside [{math.degrees(lo):.0f}, {math.degrees(hi):.0f}]"
        return True, ""

    def ik(self, xyz, tilt_options_deg=(0.0,)) -> IKResult:
        """Joint angles that put the pen tip at world xyz with the pen (nearly)
        vertical. Tries each tilt in order and returns the first that is
        reachable and inside the joint limits."""
        x, y, z = (float(v) for v in xyz)
        rho = math.hypot(x, y)
        lat = self.tip_lateral([0, 0, 0, 0, self.tool.roll_rad])
        if rho <= abs(lat):
            return IKResult(np.zeros(5), False, 0.0, "target on the pan axis")
        r = math.sqrt(rho * rho - lat * lat)
        pan = math.atan2(y, x) - math.atan2(lat, r)
        reasons = []
        for tilt in tilt_options_deg:
            # tilt is of the pen; ik_planar wants the roll axis angle
            phi3 = PEN_DOWN_ANGLE + math.radians(tilt) - self.pen_offset
            q123, why = self.ik_planar(r, z, phi3)
            if q123 is None:
                reasons.append(f"tilt {tilt:+.0f}: {why}")
                continue
            q = np.array([pan, *q123, self.tool.roll_rad])
            ok, why = self.within_limits(q)
            if ok:
                return IKResult(q, True, tilt, "")
            reasons.append(f"tilt {tilt:+.0f}: {why}")
        return IKResult(np.zeros(5), False, 0.0, "; ".join(reasons))

    # -- workspace ----------------------------------------------------------
    def reachable_mask(self, xs, ys, z: float, tilt_options_deg=(0.0,)) -> np.ndarray:
        """Boolean grid of which (x,y) on the plane z are drawable."""
        m = np.zeros((len(ys), len(xs)), bool)
        for j, y in enumerate(ys):
            for i, x in enumerate(xs):
                m[j, i] = self.ik((x, y, z), tilt_options_deg).ok
        return m

    def radial_reach(self, z: float, tilt_options_deg=(0.0,), step=1.0) -> tuple[float, float]:
        """(r_min, r_max) along the x axis at height z where the pen can be placed."""
        rs = np.arange(20.0, 400.0, step)
        ok = np.array([self.ik((r, 0.0, z), tilt_options_deg).ok for r in rs])
        if not ok.any():
            return (math.nan, math.nan)
        idx = np.where(ok)[0]
        # take the first contiguous block
        end = idx[0]
        while end + 1 < len(ok) and ok[end + 1]:
            end += 1
        return float(rs[idx[0]]), float(rs[end])


# --- reference implementation straight from the URDF, for tests ----------------
_URDF_JOINTS = [
    ("shoulder_pan", (0.0388353, 0, 0.0624), (3.14159, 0, -3.14159)),
    ("shoulder_lift", (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0)),
    ("elbow_flex", (-0.11257, -0.028, 0), (0, 0, 1.5708)),
    ("wrist_flex", (-0.1349, 0.0052, 0), (0, 0, -1.5708)),
    ("wrist_roll", (0, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159)),
]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def urdf_fk(q, tool_in_roll_frame=(-0.0079, -0.000218121, -0.0981274)) -> np.ndarray:
    """Full 3D FK from the URDF numbers. Returns the tool point in the *URDF
    base_link* frame in mm. Default tool point is the gripper_frame origin."""
    M = np.eye(4)
    for (name, xyz, rpy), qi in zip(_URDF_JOINTS, q):
        T = np.eye(4)
        T[:3, :3] = _rpy(*rpy)
        T[:3, 3] = xyz
        M = M @ T
        R = np.eye(4)
        c, s = math.cos(qi), math.sin(qi)
        R[:2, :2] = [[c, -s], [s, c]]
        M = M @ R
    p = M @ np.array([*tool_in_roll_frame, 1.0])
    return p[:3] * 1000.0


URDF_BASE_TO_PAN_X = 38.8353  # mm; world frame = URDF base_link shifted by this
# Our q[0] (pan) is positive counter-clockwise seen from above; the URDF's
# shoulder_pan axis points down, so its sign is opposite. Pitch joints agree.
URDF_SIGN = np.array([-1.0, 1.0, 1.0, 1.0, 1.0])
