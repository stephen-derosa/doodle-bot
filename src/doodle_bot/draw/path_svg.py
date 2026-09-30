"""Centerline SVG serialization and conversion to plotter movements."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from math import hypot
from pathlib import Path
from typing import Sequence

from svgelements import Move as SvgMove
from svgelements import Path as SvgPath
from svgelements import Shape, SVG

from .motion import Move, paths_to_moves
from .path_gcode import write_moves_gcode


_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def write_paths_svg(
    path: Path,
    paths: Sequence[Sequence[tuple[float, float]]],
    *,
    width: float,
    height: float,
    plot_y_max: float,
) -> None:
    """Write ordered bottom-left paths as visually equivalent SVG polylines."""

    if width <= 0 or height <= 0 or plot_y_max < 0:
        raise ValueError("SVG dimensions must be positive")
    path.parent.mkdir(parents=True, exist_ok=True)
    root = ET.Element(
        "svg",
        {
            "xmlns": "http://www.w3.org/2000/svg",
            "viewBox": f"0 0 {width:.6f} {height:.6f}",
            "width": f"{width:.6f}",
            "height": f"{height:.6f}",
            "data-plot-y-max": f"{plot_y_max:.6f}",
        },
    )
    group = ET.SubElement(
        root,
        "g",
        {
            "fill": "none",
            "stroke": "black",
            "stroke-width": "1",
            "stroke-linecap": "round",
            "stroke-linejoin": "round",
        },
    )
    for points in paths:
        if not points:
            continue
        svg_points = " ".join(
            f"{x:.6f},{plot_y_max - y:.6f}" for x, y in points
        )
        ET.SubElement(group, "polyline", {"points": svg_points})
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _numbers(value: str) -> list[float]:
    return [float(number) for number in re.findall(_NUMBER, value)]


def read_svg_canvas_size(path: Path) -> tuple[float, float]:
    """Return the SVG's rendered viewport width and height."""

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        raise ValueError(f"invalid SVG XML: {error}") from error
    view_box = _numbers(root.attrib.get("viewBox", ""))
    if len(view_box) != 4 or view_box[2] <= 0 or view_box[3] <= 0:
        raise ValueError("SVG requires a positive four-number viewBox")
    try:
        document = SVG.parse(path, reify=True)
        width, height = float(document.width), float(document.height)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"invalid SVG geometry: {error}") from error
    if width <= 0 or height <= 0:
        raise ValueError("SVG viewport dimensions must be positive")
    return width, height


def _point(segment, t: float) -> tuple[float, float]:
    point = segment.point(t)
    return float(point.x), float(point.y)


def _distance_from_line(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = hypot(dx, dy)
    if length <= 1e-12:
        return hypot(point[0] - start[0], point[1] - start[1])
    return abs(dy * point[0] - dx * point[1] + end[0] * start[1] - end[1] * start[0]) / length


def _flatten_segment(segment, tolerance: float) -> list[tuple[float, float]]:
    """Adaptively flatten one transformed SVG segment within an error bound."""

    start, end = _point(segment, 0.0), _point(segment, 1.0)
    points = [start]

    def subdivide(
        t0: float,
        first: tuple[float, float],
        t1: float,
        last: tuple[float, float],
        depth: int,
    ) -> None:
        quarter_t = t0 + (t1 - t0) * 0.25
        middle_t = (t0 + t1) / 2
        three_quarter_t = t0 + (t1 - t0) * 0.75
        samples = (
            _point(segment, quarter_t),
            _point(segment, middle_t),
            _point(segment, three_quarter_t),
        )
        error = max(_distance_from_line(point, first, last) for point in samples)
        if error <= tolerance or depth >= 20:
            points.append(last)
            return
        middle = samples[1]
        subdivide(t0, first, middle_t, middle, depth + 1)
        subdivide(middle_t, middle, t1, last, depth + 1)

    subdivide(0.0, start, 1.0, end, 0)
    return points


def _travel_distance(paths: Sequence[Sequence[tuple[float, float]]]) -> float:
    current = (0.0, 0.0)
    total = 0.0
    for path in paths:
        total += hypot(path[0][0] - current[0], path[0][1] - current[1])
        current = path[-1]
    return total


def optimize_paths(
    paths: Sequence[Sequence[tuple[float, float]]],
) -> list[list[tuple[float, float]]]:
    """Orient and order strokes to minimize non-drawing pen travel."""

    remaining = [list(path) for path in paths if path]
    ordered: list[list[tuple[float, float]]] = []
    current = (0.0, 0.0)
    while remaining:
        best_index = 0
        best_reverse = False
        best_distance = float("inf")
        for index, path in enumerate(remaining):
            for reverse, endpoint in ((False, path[0]), (True, path[-1])):
                distance = hypot(endpoint[0] - current[0], endpoint[1] - current[1])
                if distance < best_distance:
                    best_index, best_reverse, best_distance = index, reverse, distance
        selected = remaining.pop(best_index)
        if best_reverse:
            selected.reverse()
        ordered.append(selected)
        current = selected[-1]

    # Open-route 2-opt: reverse ranges (and each stroke) while travel improves.
    current_distance = _travel_distance(ordered)
    while len(ordered) > 1:
        best_distance = current_distance
        best_route: list[list[tuple[float, float]]] | None = None
        for start in range(len(ordered)):
            for end in range(start, len(ordered)):
                candidate = ordered[:start] + [
                    list(reversed(path)) for path in reversed(ordered[start:end + 1])
                ] + ordered[end + 1:]
                candidate_distance = _travel_distance(candidate)
                if candidate_distance < best_distance - 1e-9:
                    best_distance = candidate_distance
                    best_route = candidate
        if best_route is None:
            break
        ordered, current_distance = best_route, best_distance
    return ordered


def join_close_paths(
    paths: Sequence[Sequence[tuple[float, float]]],
    max_gap: float,
) -> list[list[tuple[float, float]]]:
    """Join consecutive routed strokes separated by at most ``max_gap``."""

    if max_gap < 0:
        raise ValueError("join distance must not be negative")
    joined: list[list[tuple[float, float]]] = []
    for source in paths:
        path = list(source)
        if not path:
            continue
        if joined and hypot(
            path[0][0] - joined[-1][-1][0],
            path[0][1] - joined[-1][-1][1],
        ) <= max_gap:
            if path[0] != joined[-1][-1]:
                joined[-1].append(path[0])
            joined[-1].extend(path[1:])
        else:
            joined.append(path)
    return joined


def read_paths_svg(
    path: Path,
    *,
    curve_tolerance: float = 0.25,
    optimize: bool = True,
    join_distance: float = 0.0,
) -> list[list[tuple[float, float]]]:
    """Read and flatten visible SVG geometry into bottom-left plot paths."""

    if curve_tolerance <= 0:
        raise ValueError("curve_tolerance must be positive")
    if join_distance < 0:
        raise ValueError("join distance must not be negative")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        raise ValueError(f"invalid SVG XML: {error}") from error
    view_box = _numbers(root.attrib.get("viewBox", ""))
    if len(view_box) != 4 or view_box[2] <= 0 or view_box[3] <= 0:
        raise ValueError("SVG requires a positive four-number viewBox")
    try:
        document = SVG.parse(path, reify=True)
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"invalid SVG geometry: {error}") from error
    canvas_width, canvas_height = float(document.width), float(document.height)
    if canvas_width <= 0 or canvas_height <= 0:
        raise ValueError("SVG viewport dimensions must be positive")
    plot_y_max = float(root.attrib.get("data-plot-y-max", canvas_height))

    svg_paths: list[list[tuple[float, float]]] = []
    for element in document.elements():
        values = getattr(element, "values", {})
        if values.get("visibility") in {"hidden", "collapse"} or values.get("display") == "none":
            continue
        if isinstance(element, SvgPath):
            geometry = SvgPath(element)
        elif isinstance(element, Shape):
            geometry = SvgPath(element)
        else:
            continue
        geometry.reify()
        for index in range(geometry.count_subpaths()):
            subpath = geometry.subpath(index)
            points: list[tuple[float, float]] = []
            for segment in subpath:
                if isinstance(segment, SvgMove):
                    point = (float(segment.end.x), float(segment.end.y))
                    if not points:
                        points.append(point)
                    continue
                flattened = _flatten_segment(segment, curve_tolerance)
                if points and flattened[0] == points[-1]:
                    points.extend(flattened[1:])
                else:
                    points.extend(flattened)
            if points:
                svg_paths.append(
                    [(x, plot_y_max - y) for x, y in points]
                )

    if not svg_paths:
        raise ValueError("SVG contains no drawable vector geometry")
    paths = optimize_paths(svg_paths) if optimize else svg_paths
    return join_close_paths(paths, join_distance) if join_distance > 0 else paths


def svg_to_gcode(
    input_path: Path,
    output_path: Path,
    *,
    max_step: float = 1.0,
    pen_up_z: float = 1.0,
    pen_down_z: float = 0.0,
    join_distance: float = 0.0,
) -> tuple[Move, ...]:
    """Convert optimized, adaptively flattened SVG geometry to G-code."""

    if not input_path.is_file():
        raise ValueError(f"Input SVG does not exist: {input_path}")
    if max_step <= 0:
        raise ValueError("max_step must be positive")
    moves = paths_to_moves(
        read_paths_svg(
            input_path,
            curve_tolerance=max_step / 4,
            join_distance=join_distance,
        ),
        max_step,
    )
    write_moves_gcode(
        output_path,
        moves,
        pen_up_z=pen_up_z,
        pen_down_z=pen_down_z,
    )
    return tuple(moves)
