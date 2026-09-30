"""One-shot image tracing and interactive rendering workflow."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import shutil
from time import perf_counter

from .path_so101 import gcode_to_so101, read_moves_so101
from .path_svg import read_svg_canvas_size, svg_to_gcode
from .renderer import interactive_plot
from .tracer import DEFAULT_IMAGE, trace_image_to_svg


def add_draw_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input", type=Path, default=DEFAULT_IMAGE, help=f"source image or SVG (default: {DEFAULT_IMAGE})")
    parser.add_argument("--width", type=float, default=400.0, help="plot/robot canvas width in coordinate units")
    parser.add_argument("--max-step", type=float, default=1.0, help="maximum distance per movement")
    parser.add_argument(
        "--join-distance",
        type=float,
        default=0.0,
        help="draw across SVG stroke gaps up to this distance (default: disabled)",
    )
    parser.add_argument("--pixel-width", type=int, default=400, help="processing canvas width in pixels")
    parser.add_argument("--pixel-height", type=int, default=600, help="processing canvas height in pixels")
    parser.add_argument("--pen-up-z", type=float, default=1.0, help="Z coordinate for a lifted pen")
    parser.add_argument("--pen-down-z", type=float, default=0.0, help="Z coordinate for a lowered pen")
    parser.add_argument(
        "--output-root",
        type=Path,
        help="parent for timestamped run folders (default: input directory)",
    )


def create_run_directory(input_path: Path, output_root: Path | None = None) -> Path:
    """Create a unique, timestamped output directory for one image run."""

    root = output_root if output_root is not None else input_path.parent
    timestamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    base = root / f"{input_path.stem}-{timestamp}"
    candidate = base
    index = 2
    while True:
        try:
            candidate.mkdir(parents=True)
            return candidate
        except FileExistsError:
            candidate = Path(f"{base}-{index}")
            index += 1


def run_draw_command(args: argparse.Namespace) -> None:
    started = perf_counter()
    if not args.input.is_file():
        raise SystemExit(f"Input file does not exist: {args.input}")
    run_directory = create_run_directory(args.input, args.output_root)
    stem = args.input.stem
    svg_path = run_directory / f"{stem}.svg"
    gcode_path = run_directory / f"{stem}.gcode"
    so101_path = run_directory / f"{stem}.json"
    try:
        if args.input.suffix.lower() == ".svg":
            shutil.copyfile(args.input, svg_path)
            canvas_width, canvas_height = read_svg_canvas_size(svg_path)
            source_step = "SVG"
        else:
            result = trace_image_to_svg(
                args.input,
                svg_path,
                pixel_width=args.pixel_width,
                pixel_height=args.pixel_height,
                coordinate_width=args.width,
                max_step=args.max_step,
            )
            canvas_width = args.width
            canvas_height = args.width * result.canvas_height / result.canvas_width
            source_step = "Image -> SVG"
        svg_to_gcode(
            svg_path,
            gcode_path,
            max_step=args.max_step,
            join_distance=args.join_distance,
            pen_up_z=args.pen_up_z,
            pen_down_z=args.pen_down_z,
        )
        gcode_to_so101(
            gcode_path,
            so101_path,
            name=stem,
            pen_up_z=args.pen_up_z,
            pen_down_z=args.pen_down_z,
        )
        # Read the final artifact back so the preview consumes the exact
        # SO-101 drawing contract handed to the robot application.
        moves = read_moves_so101(so101_path, max_step=args.max_step)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    print(
        f"Input: {args.input} | {source_step} -> G-code -> SO-101 JSON | "
        f"canvas: {canvas_width:g}x{canvas_height:g} | moves: {len(moves)}\n"
        f"Wrote {run_directory}"
    )
    interactive_plot(moves, canvas_width, canvas_height, started)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_draw_arguments(parser)
    run_draw_command(parser.parse_args())


if __name__ == "__main__":
    main()
