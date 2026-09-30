"""Convert centerline SVG files to absolute-position plotter G-code."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter

from .path_svg import svg_to_gcode


def add_svg_to_gcode_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input", required=True, type=Path, help="source centerline SVG")
    parser.add_argument("--output", required=True, type=Path, help="destination G-code file")
    parser.add_argument("--max-step", type=float, default=1.0, help="maximum distance per movement")
    parser.add_argument(
        "--join-distance",
        type=float,
        default=0.0,
        help="draw across gaps up to this many SVG units (default: disabled)",
    )
    parser.add_argument("--pen-up-z", type=float, default=1.0, help="Z coordinate for a lifted pen")
    parser.add_argument("--pen-down-z", type=float, default=0.0, help="Z coordinate for a lowered pen")


def run_svg_to_gcode_command(args: argparse.Namespace) -> None:
    started = perf_counter()
    try:
        moves = svg_to_gcode(
            args.input,
            args.output,
            max_step=args.max_step,
            join_distance=args.join_distance,
            pen_up_z=args.pen_up_z,
            pen_down_z=args.pen_down_z,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error
    draw_moves = sum(move.pen_down for move in moves)
    print(
        f"SVG: {args.input} | moves: {len(moves)} "
        f"({draw_moves} draw, {len(moves) - draw_moves} travel)"
    )
    print(f"Wrote {args.output} in {perf_counter() - started:.3f} seconds")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_svg_to_gcode_arguments(parser)
    run_svg_to_gcode_command(parser.parse_args())


if __name__ == "__main__":
    main()
