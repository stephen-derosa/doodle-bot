"""PNG previews (Pillow only - matplotlib is not installed on the Pi).

* ``render_drawing``  the strokes as they will appear on the canvas
* ``render_toolpath`` pen-down (black) and pen-up travel (grey dashes) moves
* ``render_layout``   top view of the robot: reachable zone, paper, canvas
* ``render_joints``   joint angle traces of a planned trajectory
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .config import Calibration, Config
from .kinematics import SO101Kinematics
from .shapes import Drawing


def _canvas(w_mm: float, h_mm: float, px_per_mm: float, pad: int = 20):
    W, H = int(w_mm * px_per_mm) + 2 * pad, int(h_mm * px_per_mm) + 2 * pad
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    def to_px(u, v):  # paper coords (v up) -> image coords (y down)
        return pad + u * px_per_mm, H - pad - v * px_per_mm
    return img, d, to_px


def render_drawing(drawing: Drawing, width: float, height: float, path: Path | str,
                   px_per_mm: float = 4.0) -> Path:
    img, d, to_px = _canvas(width, height, px_per_mm)
    d.rectangle([to_px(0, height), to_px(width, 0)], outline=(180, 180, 180), width=1)
    for s in drawing.strokes:
        pts = [to_px(*p) for p in s.points]
        if len(pts) >= 2:
            d.line(pts, fill="black", width=max(1, int(0.6 * px_per_mm)), joint="curve")
        else:
            d.ellipse([pts[0][0] - 1, pts[0][1] - 1, pts[0][0] + 1, pts[0][1] + 1], fill="black")
    d.text((6, 4), f"{drawing.name}  canvas {width:.0f}x{height:.0f} mm  ink {drawing.total_length():.0f} mm", fill=(90, 90, 90))
    path = Path(path)
    img.save(path)
    return path


def render_toolpath(segments, width: float, height: float, path: Path | str, px_per_mm: float = 4.0,
                    title: str = "") -> Path:
    """segments: iterable of (uv_points (N,2), pen_down: bool) in canvas mm."""
    img, d, to_px = _canvas(width, height, px_per_mm)
    d.rectangle([to_px(0, height), to_px(width, 0)], outline=(180, 180, 180), width=1)
    ink = travel = 0.0
    for pts, down in segments:
        pts = np.asarray(pts, float)
        if len(pts) < 2:
            continue
        L = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        px = [to_px(*p) for p in pts]
        if down:
            ink += L
            d.line(px, fill="black", width=max(1, int(0.6 * px_per_mm)), joint="curve")
        else:
            travel += L
            # dashed grey
            for a, b in zip(px[:-1], px[1:]):
                seg = math.hypot(b[0] - a[0], b[1] - a[1])
                n = max(1, int(seg / 6))
                for k in range(0, n, 2):
                    t0, t1 = k / n, min((k + 1) / n, 1)
                    d.line([(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0),
                            (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)], fill=(160, 160, 200), width=1)
            r = 3
            d.ellipse([px[-1][0] - r, px[-1][1] - r, px[-1][0] + r, px[-1][1] + r], outline=(200, 60, 60))
    d.text((6, 4), f"{title}  ink {ink:.0f} mm, travel {travel:.0f} mm", fill=(90, 90, 90))
    path = Path(path)
    img.save(path)
    return path


def render_layout(cfg: Config, cal: Calibration, path: Path | str, px_per_mm: float = 2.0,
                  canvas_uv0=(0.0, 0.0), z: float | None = None, tilt_options=None) -> Path:
    """Top-down view in the world frame: reachable pen positions, paper, canvas."""
    kin = SO101Kinematics.from_config(cfg, cal=cal)
    if cal.tool_along is not None:
        kin.tool.along = cal.tool_along
    z = cal.paper.origin[2] if z is None else z
    tilt_options = tilt_options or cfg.motion.pen_tilt_options_deg
    xs = np.arange(-320, 321, 5.0)
    ys = np.arange(-320, 321, 5.0)
    W = H = int(640 * px_per_mm) + 40
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    def to_px(x, y):
        return 20 + (x + 320) * px_per_mm, H - 20 - (y + 320) * px_per_mm

    strict = kin.reachable_mask(xs, ys, z, (0.0,))
    loose = kin.reachable_mask(xs, ys, z, tilt_options)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if loose[j, i]:
                col = (200, 235, 200) if strict[j, i] else (235, 235, 190)
                a, b = to_px(x - 2.5, y + 2.5), to_px(x + 2.5, y - 2.5)
                d.rectangle([a, b], fill=col)
    # base + pan axis
    d.ellipse([*to_px(-45, 45), *to_px(45, -45)], outline=(80, 80, 80), width=2)
    d.line([to_px(0, 0), to_px(60, 0)], fill=(80, 80, 80), width=2)
    d.text(to_px(-40, -50), "base (x forward)", fill=(80, 80, 80))
    # paper + canvas
    P = cfg.paper
    pf = cal.paper
    corners = [pf.to_world(*uv) for uv in [(0, 0), (P.width, 0), (P.width, P.height), (0, P.height)]]
    d.polygon([to_px(c[0], c[1]) for c in corners], outline=(60, 60, 220))
    u0, v0 = canvas_uv0
    cc = [pf.to_world(*uv) for uv in [(u0, v0), (u0 + P.canvas_width, v0), (u0 + P.canvas_width, v0 + P.canvas_height), (u0, v0 + P.canvas_height)]]
    d.polygon([to_px(c[0], c[1]) for c in cc], outline=(220, 60, 60))
    d.text((6, 4), f"green: pen vertical reachable  yellow: needs tilt  blue: paper  red: canvas  (z={z:.1f})", fill=(60, 60, 60))
    path = Path(path)
    img.save(path)
    return path


def render_joints(t: np.ndarray, q: np.ndarray, names: list[str], path: Path | str,
                  pen_down: np.ndarray | None = None) -> Path:
    W, H = 1200, 500
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    qd = np.degrees(q)
    lo, hi = float(qd.min()) - 5, float(qd.max()) + 5
    pad = 50
    colors = [(200, 40, 40), (40, 140, 40), (40, 40, 200), (180, 120, 0), (120, 40, 160)]

    def to_px(tt, v):
        return pad + (tt - t[0]) / max(t[-1] - t[0], 1e-9) * (W - 2 * pad), H - pad - (v - lo) / (hi - lo) * (H - 2 * pad)
    if pen_down is not None:
        for i in range(len(t) - 1):
            if pen_down[i]:
                d.line([to_px(t[i], lo), to_px(t[i], hi)], fill=(235, 235, 235))
    for v in range(int(lo // 30) * 30, int(hi) + 30, 30):
        if lo <= v <= hi:
            d.line([to_px(t[0], v), to_px(t[-1], v)], fill=(220, 220, 220))
            d.text((4, to_px(t[0], v)[1] - 6), f"{v}", fill=(120, 120, 120))
    for k, name in enumerate(names):
        pts = [to_px(tt, v) for tt, v in zip(t, qd[:, k])]
        d.line(pts, fill=colors[k % len(colors)], width=2)
        d.text((pad + 10 + 170 * k, 8), name, fill=colors[k % len(colors)])
    d.text((W - 200, H - 30), f"t = {t[-1]:.1f} s", fill=(80, 80, 80))
    path = Path(path)
    img.save(path)
    return path
