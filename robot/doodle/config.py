"""Configuration and calibration files.

Two YAML files live in ``config/``:

* ``doodle.yaml``      static: port, joints, geometry, motion limits, paper.
* ``calibration.yaml`` measured: per-joint zero ticks, directions, paper frame.

Everything is plain dataclasses so the rest of the code never touches YAML.
Units: millimetres, radians, seconds, servo ticks (4096/rev) unless a field
name says otherwise (``*_deg``).
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT / "config"
DEFAULT_CONFIG = CONFIG_DIR / "doodle.yaml"
DEFAULT_CALIB = CONFIG_DIR / "calibration.yaml"

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@dataclass
class JointConfig:
    name: str
    id: int
    # URDF soft limits, radians. Tightened here if the pen holder collides.
    min_rad: float
    max_rad: float
    # Servo tick window that is ever allowed to be commanded (hard safety).
    # When `travel_measured` is set these came from `doodle calib travel`
    # sweeping the joint to its stops, and they -- not min_rad/max_rad -- are
    # the source of truth: ticks are the servo's own absolute encoder, so they
    # survive every recalibration that redefines q.
    min_ticks: int = 100
    max_ticks: int = 3995
    travel_measured: bool = False


@dataclass
class Geometry:
    """SO-101 planar geometry, derived from the official URDF (see docs/PLAN.md)."""
    lift_r: float = 30.4        # lift axis horizontal offset from the pan axis
    lift_z: float = 116.6       # lift axis height above the table (base bottom = 0)
    a1: float = 116.0           # shoulder_lift -> elbow_flex
    a2: float = 135.0           # elbow_flex -> wrist_flex
    a3: float = 61.1            # wrist_flex -> wrist_roll axis origin
    # absolute in-plane angle of each link at URDF q = 0, degrees from horizontal
    zero_angle1_deg: float = 76.04
    zero_angle2_deg: float = 2.21
    zero_angle3_deg: float = 0.0


@dataclass
class Tool:
    """Pen tip relative to the wrist_roll output frame."""
    along: float = 100.0    # mm along the roll axis (out of the holder) to the pen tip
    perp: float = 0.0       # mm off-axis, in the arm plane when roll = roll_rad (+ = 90 deg CCW of the roll axis)
    lateral: float = 0.0    # mm off-axis, perpendicular to the arm plane
    roll_rad: float = 0.0   # wrist_roll joint angle held while drawing
    # Angle of the pen body from the roll axis, in the arm plane. 0 = the pen
    # continues the roll axis (a gripper-style mount). -90 = the pen hangs
    # square below a roll axis that points forward, as on a bracket clamped to
    # the side of the roll horn: then the pen is vertical with the roll axis
    # level, and its tip is `perp` = -(tip drop below the axis) mm off it.
    pen_angle_deg: float = 0.0


@dataclass
class Motion:
    rate_hz: float = 50.0            # setpoint streaming rate
    draw_speed: float = 30.0         # mm/s along the paper while the pen is down
    travel_speed: float = 80.0       # mm/s while the pen is up
    accel: float = 200.0             # mm/s^2 tip acceleration for the S-profile
    corner_speed: float = 8.0        # mm/s minimum speed through sharp corners
    pen_up_z: float = 8.0            # lift above the paper between strokes
    pen_press: float = 1.5           # push below the fitted paper plane for pressure
    plunge_speed: float = 15.0       # mm/s for pen up/down moves
    dwell_s: float = 0.15            # settle time after pen down / before pen up
    max_joint_speed_deg_s: float = 90.0
    approach_speed_deg_s: float = 20.0   # joint-space speed for moving to the start pose
    pen_tilt_options_deg: tuple = (0.0, -5.0, 5.0, -10.0, 10.0, -15.0, 15.0)


@dataclass
class ServoTuning:
    p_gain: int = 16          # lerobot uses 16 to avoid shakiness (default 32)
    i_gain: int = 0
    d_gain: int = 32
    acceleration: int = 30    # x100 ticks/s^2 ramp inside the servo
    goal_velocity: int = 1500  # ticks/s cap; streaming setpoints do the real shaping
    torque_limit: int = 700   # of 1000, so a collision stalls rather than strips gears
    return_delay: int = 0


@dataclass
class Paper:
    """Physical sheet. Canvas is the sub-rectangle we actually draw in."""
    width: float = 215.9      # letter, portrait, mm
    height: float = 279.4
    canvas_width: float = 150.0
    canvas_height: float = 100.0
    margin: float = 5.0       # keep strokes this far inside the canvas
    # canvas bottom-left corner on the paper; None = centred on the sheet
    canvas_u0: float | None = None
    canvas_v0: float | None = 30.0   # 30 mm from the near edge keeps the whole canvas in easy reach

    def canvas_origin(self) -> tuple[float, float]:
        u0 = (self.width - self.canvas_width) / 2 if self.canvas_u0 is None else self.canvas_u0
        v0 = (self.height - self.canvas_height) / 2 if self.canvas_v0 is None else self.canvas_v0
        return u0, v0

    def canvas_corners(self) -> list[tuple[str, float, float]]:
        """Clockwise from the robot's near-left: name and paper (u, v) mm."""
        u0, v0 = self.canvas_origin()
        w, h = self.canvas_width, self.canvas_height
        return [
            ("near left", u0, v0),
            ("near right", u0 + w, v0),
            ("far right", u0 + w, v0 + h),
            ("far left", u0, v0 + h),
        ]


@dataclass
class Config:
    port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61034319-if00"
    baud: int = 1_000_000
    joints: list[JointConfig] = field(default_factory=lambda: [
        JointConfig("shoulder_pan", 1, -1.91986, 1.91986),
        JointConfig("shoulder_lift", 2, -1.74533, 1.74533),
        JointConfig("elbow_flex", 3, -1.69, 1.69),
        JointConfig("wrist_flex", 4, -1.65806, 1.65806),
        JointConfig("wrist_roll", 5, -2.74385, 2.84121),
    ])
    geometry: Geometry = field(default_factory=Geometry)
    tool: Tool = field(default_factory=Tool)
    motion: Motion = field(default_factory=Motion)
    servo: ServoTuning = field(default_factory=ServoTuning)
    paper: Paper = field(default_factory=Paper)
    # Joint-space pose (deg, URDF convention) to park in with torque still on.
    park_pose_deg: list[float] = field(default_factory=lambda: [0.0, -95.0, 95.0, 0.0, 0.0])

    @property
    def ids(self) -> list[int]:
        return [j.id for j in self.joints]

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.joints]

    def joint(self, name: str) -> JointConfig:
        return next(j for j in self.joints if j.name == name)


@dataclass
class PaperFrame:
    """Where the paper is in the world frame (pan axis at the table = origin).

    The defaults describe a letter sheet in portrait, centred in front of the
    robot with its near edge 100 mm from the pan axis: paper u runs to the
    robot's right (world -y), paper v runs away from the robot (world +x).
    They are only a placeholder for previews and dry runs; ``calibrated``
    stays False until `doodle calib paper` measures the real sheet."""
    origin: list[float] = field(default_factory=lambda: [100.0, 108.0, 0.0])  # paper (0,0) corner
    x_axis: list[float] = field(default_factory=lambda: [0.0, -1.0, 0.0])    # unit, along paper width
    y_axis: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])     # unit, along paper height
    calibrated: bool = False

    def to_world(self, u: float, v: float, w: float = 0.0):
        import numpy as np
        o, x, y = (np.asarray(a, float) for a in (self.origin, self.x_axis, self.y_axis))
        z = np.cross(x, y)
        return o + u * x + v * y + w * z


@dataclass
class Calibration:
    # ticks that correspond to URDF q = 0 for each joint, in config joint order
    zero_ticks: list[float] = field(default_factory=lambda: [2048.0] * 5)
    # +1 if increasing ticks == increasing URDF q, else -1
    direction: list[int] = field(default_factory=lambda: [1, 1, 1, 1, 1])
    joints_calibrated: bool = False
    paper: PaperFrame = field(default_factory=PaperFrame)
    tool_along: float | None = None   # refined tool length, overrides Config.tool.along
    # Raw evidence the calibration was derived from, kept so it can be re-examined
    # offline (`doodle calib diagnose`) without touching the arm again.
    pose_ticks: list[float] | None = None                 # servo ticks at the recorded L pose
    paper_touches: list[dict] = field(default_factory=list)  # [{label, uv, ticks}] from `calib paper`
    notes: str = ""


# --- (de)serialisation helpers -------------------------------------------------
def _from_dict(cls, d: dict[str, Any]):
    kw = {}
    for f in fields(cls):
        if f.name not in d:
            continue
        v = d[f.name]
        t = f.type if not isinstance(f.type, str) else None
        if f.name == "joints":
            v = [JointConfig(**j) for j in v]
        elif f.name == "geometry":
            v = _from_dict(Geometry, v)
        elif f.name == "tool":
            v = _from_dict(Tool, v)
        elif f.name == "motion":
            v = _from_dict(Motion, v)
        elif f.name == "servo":
            v = _from_dict(ServoTuning, v)
        elif f.name == "paper" and cls is Config:
            v = _from_dict(Paper, v)
        elif f.name == "paper" and cls is Calibration:
            v = _from_dict(PaperFrame, v)
        elif f.name == "pen_tilt_options_deg":
            v = tuple(v)
        kw[f.name] = v
    return cls(**kw)


def _to_plain(obj):
    if is_dataclass(obj):
        return {k: _to_plain(v) for k, v in asdict(obj).items()}
    if isinstance(obj, (list, tuple)):
        return [_to_plain(v) for v in obj]
    if isinstance(obj, dict):
        return {k: _to_plain(v) for k, v in obj.items()}
    if hasattr(obj, "item"):  # numpy scalar
        return obj.item()
    return obj


def load_config(path: Path | str | None = None) -> Config:
    path = Path(path) if path else DEFAULT_CONFIG
    if not path.exists():
        return Config()
    with open(path) as f:
        return _from_dict(Config, yaml.safe_load(f) or {})


def save_config(cfg: Config, path: Path | str | None = None) -> Path:
    path = Path(path) if path else DEFAULT_CONFIG
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(_to_plain(cfg), f, sort_keys=False)
    return path


def load_calibration(path: Path | str | None = None) -> Calibration:
    path = Path(path) if path else DEFAULT_CALIB
    if not path.exists():
        return Calibration()
    with open(path) as f:
        return _from_dict(Calibration, yaml.safe_load(f) or {})


def save_calibration(cal: Calibration, path: Path | str | None = None) -> Path:
    path = Path(path) if path else DEFAULT_CALIB
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(_to_plain(cal), f, sort_keys=False)
    return path


def deg(rad: float) -> float:
    return math.degrees(rad)


def rad(deg_: float) -> float:
    return math.radians(deg_)
