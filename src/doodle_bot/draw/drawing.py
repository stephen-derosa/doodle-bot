"""One-shot image tracing and interactive rendering workflow."""

from __future__ import annotations

import argparse
from pathlib import Path
from tempfile import TemporaryDirectory
from time import perf_counter

from .path_csv import read_moves_csv
from .renderer import interactive_plot
from .tracer import DEFAULT_IMAGE, trace_image_to_csv


def add_draw_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input", type=Path, default=DEFAULT_IMAGE, help=f"source image (default: {DEFAULT_IMAGE})")
    parser.add_argument("--width", type=float, default=400.0, help="plot/robot canvas width in coordinate units")
    parser.add_argument("--max-step", type=float, default=1.0, help="maximum distance per movement")
    parser.add_argument("--pixel-width", type=int, default=400, help="processing canvas width in pixels")
    parser.add_argument("--pixel-height", type=int, default=600, help="processing canvas height in pixels")


def run_draw_command(args: argparse.Namespace) -> None:
    started = perf_counter()
    with TemporaryDirectory(prefix="doodle-bot-") as temporary_directory:
        csv_path = Path(temporary_directory) / "movements.csv"
        try:
            result = trace_image_to_csv(
                args.input,
                csv_path,
                pixel_width=args.pixel_width,
                pixel_height=args.pixel_height,
                coordinate_width=args.width,
                max_step=args.max_step,
            )
            # Intentionally read the generated file back: the GUI consumes the
            # same CSV contract a robot or standalone renderer receives.
            moves = read_moves_csv(csv_path)
        except ValueError as error:
            raise SystemExit(str(error)) from error
        height = args.width * result.canvas_height / result.canvas_width
        print(
            f"Image: {args.input} | internal CSV commands loaded | "
            f"canvas: {result.canvas_width}x{result.canvas_height} | moves: {len(moves)}"
        )
        interactive_plot(moves, args.width, height, started)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_draw_arguments(parser)
    run_draw_command(parser.parse_args())


if __name__ == "__main__":
    main()
