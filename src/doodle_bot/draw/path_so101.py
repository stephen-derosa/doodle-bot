"""Convert movement G-code into SO-101 drawing assets.

The SO-101 application consumes the JSON ``Drawing`` format implemented by
``robot/doodle/shapes.py``.  It stores pen-down paths as millimetre polylines;
calibrated joint trajectories are generated later by the robot planner.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

from .motion import Move, paths_to_moves
from .path_gcode import read_moves_gcode


def moves_to_strokes(moves: Iterable[Move]) -> list[list[list[float]]]:
    """Group movement destinations into pen-down SO-101 polylines.

    The final pen-up position is included as the start of a new stroke because
    lowering the pen happens at that position before the first drawing move.
    Consecutive duplicate points are omitted.
    """

    strokes: list[list[list[float]]] = []
    stroke: list[list[float]] | None = None
    current: list[float] | None = None

    for move in moves:
        point = [float(move.x), float(move.y)]
        if not all(math.isfinite(value) for value in point):
            raise ValueError("movement coordinates must be finite")

        if move.pen_down:
            if stroke is None:
                stroke = []
                if current is not None:
                    stroke.append(current)
            if not stroke or point != stroke[-1]:
                stroke.append(point)
        elif stroke is not None:
            strokes.append(stroke)
            stroke = None
        current = point

    if stroke is not None:
        strokes.append(stroke)
    return strokes


def read_so101_strokes(path: Path) -> list[list[tuple[float, float]]]:
    """Read and validate the SO-101 JSON drawing format."""

    try:
        drawing = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(f"unable to read SO-101 JSON: {error}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid SO-101 JSON: {error}") from error
    if not isinstance(drawing, dict):
        raise ValueError("SO-101 JSON root must be an object")
    if drawing.get("units") != "mm":
        raise ValueError("SO-101 JSON units must be 'mm'")
    raw_strokes = drawing.get("strokes")
    if not isinstance(raw_strokes, list) or not raw_strokes:
        raise ValueError("SO-101 JSON requires a non-empty strokes array")

    strokes: list[list[tuple[float, float]]] = []
    for stroke_index, raw_stroke in enumerate(raw_strokes):
        if not isinstance(raw_stroke, list) or not raw_stroke:
            raise ValueError(f"stroke {stroke_index} must be a non-empty array")
        stroke: list[tuple[float, float]] = []
        for point_index, raw_point in enumerate(raw_stroke):
            if (
                not isinstance(raw_point, list)
                or len(raw_point) != 2
                or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in raw_point)
            ):
                raise ValueError(
                    f"stroke {stroke_index} point {point_index} must contain two numbers"
                )
            point = float(raw_point[0]), float(raw_point[1])
            if not all(math.isfinite(value) for value in point):
                raise ValueError(
                    f"stroke {stroke_index} point {point_index} must be finite"
                )
            stroke.append(point)
        strokes.append(stroke)
    return strokes


def read_moves_so101(path: Path, *, max_step: float = 1.0) -> list[Move]:
    """Convert an SO-101 JSON drawing into visualization movements."""

    return paths_to_moves(read_so101_strokes(path), max_step=max_step)


def gcode_to_so101(
    input_path: Path,
    output_path: Path,
    *,
    name: str | None = None,
    pen_up_z: float = 1.0,
    pen_down_z: float = 0.0,
) -> dict[str, object]:
    """Write G-code as a JSON drawing accepted by the SO-101 robot app."""

    if not input_path.is_file():
        raise ValueError(f"Input G-code does not exist: {input_path}")
    drawing_name = name if name is not None else input_path.stem
    if not drawing_name.strip():
        raise ValueError("drawing name must not be empty")

    strokes = moves_to_strokes(
        read_moves_gcode(
            input_path,
            pen_up_z=pen_up_z,
            pen_down_z=pen_down_z,
        )
    )
    if not strokes:
        raise ValueError("G-code contains no pen-down movements")

    drawing: dict[str, object] = {
        "name": drawing_name,
        "units": "mm",
        "strokes": [
            [[round(x, 4), round(y, 4)] for x, y in stroke]
            for stroke in strokes
        ],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(drawing, indent=1) + "\n", encoding="utf-8")
    return drawing


def add_gcode_to_so101_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input", required=True, type=Path, help="source movement G-code")
    parser.add_argument("--output", required=True, type=Path, help="destination SO-101 JSON")
    parser.add_argument("--name", help="drawing name (defaults to the input filename)")
    parser.add_argument("--pen-up-z", type=float, default=1.0, help="Z coordinate for a lifted pen")
    parser.add_argument("--pen-down-z", type=float, default=0.0, help="Z coordinate for a lowered pen")


def run_gcode_to_so101_command(args: argparse.Namespace) -> None:
    try:
        drawing = gcode_to_so101(
            args.input,
            args.output,
            name=args.name,
            pen_up_z=args.pen_up_z,
            pen_down_z=args.pen_down_z,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error
    strokes = drawing["strokes"]
    point_count = sum(len(stroke) for stroke in strokes)
    print(f"Wrote {args.output}: {len(strokes)} stroke(s), {point_count} points")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_gcode_to_so101_arguments(parser)
    run_gcode_to_so101_command(parser.parse_args())


if __name__ == "__main__":
    main()
