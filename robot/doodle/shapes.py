"""Drawings are lists of strokes; strokes are polylines in millimetres on the
paper (u to the right, v up, origin bottom-left of the *canvas*).

The generators here are the test assets. Anything else (SVG, traced images)
only has to produce the same ``Drawing`` structure to be drawable.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Stroke:
    points: np.ndarray  # (N, 2)

    def length(self) -> float:
        if len(self.points) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(self.points, axis=0), axis=1)))


@dataclass
class Drawing:
    name: str
    strokes: list[Stroke] = field(default_factory=list)

    def bounds(self) -> tuple[float, float, float, float]:
        pts = np.vstack([s.points for s in self.strokes if len(s.points)])
        return float(pts[:, 0].min()), float(pts[:, 1].min()), float(pts[:, 0].max()), float(pts[:, 1].max())

    def total_length(self) -> float:
        return sum(s.length() for s in self.strokes)

    def transformed(self, scale: float = 1.0, offset=(0.0, 0.0)) -> "Drawing":
        off = np.asarray(offset, float)
        return Drawing(self.name, [Stroke(s.points * scale + off) for s in self.strokes])

    def fit_to(self, width: float, height: float, margin: float = 0.0) -> "Drawing":
        """Uniformly scale and centre inside a width x height box."""
        x0, y0, x1, y1 = self.bounds()
        w, h = max(x1 - x0, 1e-9), max(y1 - y0, 1e-9)
        s = min((width - 2 * margin) / w, (height - 2 * margin) / h)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        return self.transformed(s, (width / 2 - cx * s, height / 2 - cy * s))

    def to_dict(self) -> dict:
        return {"name": self.name, "units": "mm",
                "strokes": [np.round(s.points, 4).tolist() for s in self.strokes]}

    @classmethod
    def from_dict(cls, d: dict) -> "Drawing":
        return cls(d["name"], [Stroke(np.asarray(p, float).reshape(-1, 2)) for p in d["strokes"]])

    def save(self, path: Path | str) -> Path:
        path = Path(path)
        path.write_text(json.dumps(self.to_dict(), indent=1))
        return path

    @classmethod
    def load(cls, path: Path | str) -> "Drawing":
        return cls.from_dict(json.loads(Path(path).read_text()))

    def to_svg(self, stroke_width: float = 0.8) -> str:
        x0, y0, x1, y1 = self.bounds()
        pad = 5
        w, h = x1 - x0 + 2 * pad, y1 - y0 + 2 * pad
        paths = []
        for s in self.strokes:
            d = " ".join(f"{'M' if i == 0 else 'L'}{p[0]-x0+pad:.3f},{y1-p[1]+pad:.3f}" for i, p in enumerate(s.points))
            paths.append(f'<path d="{d}" fill="none" stroke="black" stroke-width="{stroke_width}" stroke-linecap="round" stroke-linejoin="round"/>')
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" viewBox="0 0 {w:.3f} {h:.3f}">'
                + "".join(paths) + "</svg>")


# --- generators (all centred on the origin, sized in mm) -------------------------
def _arc_points(radius: float, a0: float, a1: float, spacing: float) -> np.ndarray:
    n = max(8, int(math.ceil(abs(a1 - a0) * radius / spacing)))
    a = np.linspace(a0, a1, n + 1)
    return np.stack([radius * np.cos(a), radius * np.sin(a)], axis=1)


def circle(diameter: float = 80.0, spacing: float = 1.0) -> Drawing:
    pts = _arc_points(diameter / 2, 0.0, 2 * math.pi, spacing)
    pts[-1] = pts[0]
    return Drawing("circle", [Stroke(pts)])


def square(side: float = 80.0) -> Drawing:
    h = side / 2
    pts = np.array([[-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]])
    return Drawing("square", [Stroke(pts)])


def star(points: int = 5, outer_radius: float = 45.0, inner_ratio: float = 0.4) -> Drawing:
    pts = []
    for k in range(2 * points):
        r = outer_radius if k % 2 == 0 else outer_radius * inner_ratio
        a = math.pi / 2 + k * math.pi / points
        pts.append([r * math.cos(a), r * math.sin(a)])
    pts.append(pts[0])
    return Drawing("star", [Stroke(np.array(pts))])


def spiral(turns: float = 3.5, outer_radius: float = 45.0, spacing: float = 1.0) -> Drawing:
    """Archimedean spiral from the centre outwards, ~constant point spacing."""
    b = outer_radius / (2 * math.pi * turns)
    theta_max = 2 * math.pi * turns
    # arc length s ~ b*theta^2/2  ->  theta = sqrt(2 s / b)
    s_max = b * theta_max ** 2 / 2
    n = max(50, int(s_max / spacing))
    s = np.linspace(0, s_max, n)
    theta = np.sqrt(2 * s / b)
    r = b * theta
    return Drawing("spiral", [Stroke(np.stack([r * np.cos(theta), r * np.sin(theta)], axis=1))])


def test_grid(width: float = 150.0, height: float = 100.0, cells: int = 3) -> Drawing:
    """Ruled grid: the best asset for checking scale, squareness and repeatability."""
    strokes = []
    for i in range(cells + 1):
        x = -width / 2 + i * width / cells
        strokes.append(Stroke(np.array([[x, -height / 2], [x, height / 2]])))
    for j in range(cells + 1):
        y = -height / 2 + j * height / cells
        strokes.append(Stroke(np.array([[-width / 2, y], [width / 2, y]])))
    return Drawing("grid", strokes)


GENERATORS = {
    "circle": circle,
    "square": square,
    "star": star,
    "spiral": spiral,
    "grid": test_grid,
}


def make(name: str) -> Drawing:
    if name not in GENERATORS:
        raise KeyError(f"unknown shape {name!r}; choose from {sorted(GENERATORS)}")
    return GENERATORS[name]()
