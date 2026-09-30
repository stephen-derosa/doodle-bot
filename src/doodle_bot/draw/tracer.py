"""Convert a source image into robot-friendly XY movements."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Sequence

import numpy as np
from PIL import Image

from .motion import Move, paths_to_moves


DEFAULT_IMAGE = Path("images/char.png")


@dataclass(frozen=True)
class TraceResult:
    canvas_width: int
    canvas_height: int
    stroke_count: int
    moves: tuple[Move, ...]


def _otsu_threshold(gray: np.ndarray) -> int:
    histogram = np.bincount(gray.ravel(), minlength=256).astype(float)
    probabilities = histogram / histogram.sum()
    cumulative_probability = np.cumsum(probabilities)
    cumulative_mean = np.cumsum(probabilities * np.arange(256))
    total_mean = cumulative_mean[-1]
    denominator = cumulative_probability * (1.0 - cumulative_probability)
    variance = np.zeros(256)
    valid = denominator > 0
    variance[valid] = (
        total_mean * cumulative_probability[valid] - cumulative_mean[valid]
    ) ** 2 / denominator[valid]
    return int(np.argmax(variance))


def image_to_mask(
    path: Path,
    pixel_width: int = 400,
    pixel_height: int = 600,
) -> np.ndarray:
    """Fit an image into a centered, fixed-size boolean foreground mask."""

    if pixel_width <= 0 or pixel_height <= 0:
        raise ValueError("pixel_width and pixel_height must be positive")
    with Image.open(path) as source:
        image = source.convert("L")
        scale = min(pixel_width / image.width, pixel_height / image.height)
        fitted_size = (
            max(1, round(image.width * scale)),
            max(1, round(image.height * scale)),
        )
        image = image.resize(fitted_size, Image.Resampling.LANCZOS)
        canvas = Image.new("L", (pixel_width, pixel_height), color=255)
        offset = (
            (pixel_width - image.width) // 2,
            (pixel_height - image.height) // 2,
        )
        canvas.paste(image, offset)
        gray = np.asarray(canvas)
    mask = gray <= _otsu_threshold(gray)
    # Line art normally occupies the minority of the page. This also supports
    # white-on-black drawings without requiring a separate flag.
    if mask.mean() > 0.5:
        mask = ~mask
    return _thin(mask)


def _thin(mask: np.ndarray) -> np.ndarray:
    """Skeletonize a binary image with the Zhang-Suen thinning algorithm."""

    pixels = mask.astype(np.uint8).copy()
    changed = True
    while changed:
        changed = False
        for first_pass in (True, False):
            padded = np.pad(pixels, 1)
            p2 = padded[:-2, 1:-1]
            p3 = padded[:-2, 2:]
            p4 = padded[1:-1, 2:]
            p5 = padded[2:, 2:]
            p6 = padded[2:, 1:-1]
            p7 = padded[2:, :-2]
            p8 = padded[1:-1, :-2]
            p9 = padded[:-2, :-2]
            neighbors = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            transitions = (
                (p2 == 0) & (p3 == 1)
            ).astype(np.uint8)
            for before, after in ((p3, p4), (p4, p5), (p5, p6), (p6, p7),
                                  (p7, p8), (p8, p9), (p9, p2)):
                transitions += ((before == 0) & (after == 1)).astype(np.uint8)
            if first_pass:
                preserve_a = p2 * p4 * p6 == 0
                preserve_b = p4 * p6 * p8 == 0
            else:
                preserve_a = p2 * p4 * p8 == 0
                preserve_b = p2 * p6 * p8 == 0
            remove = (
                (pixels == 1)
                & (neighbors >= 2)
                & (neighbors <= 6)
                & (transitions == 1)
                & preserve_a
                & preserve_b
            )
            if remove.any():
                pixels[remove] = 0
                changed = True
    return pixels.astype(bool)


def mask_to_strokes(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Trace neighboring foreground pixels into pen-down strokes."""

    remaining = {tuple(point) for point in np.argwhere(mask)}
    strokes: list[list[tuple[int, int]]] = []
    offsets = tuple(
        (dy, dx)
        for dy in (-1, 0, 1)
        for dx in (-1, 0, 1)
        if (dy, dx) != (0, 0)
    )

    def available_neighbors(point: tuple[int, int]) -> list[tuple[int, int]]:
        y, x = point
        return [(y + dy, x + dx) for dy, dx in offsets if (y + dy, x + dx) in remaining]

    while remaining:
        endpoints = [point for point in remaining if len(available_neighbors(point)) <= 1]
        current = min(endpoints or remaining)
        stroke: list[tuple[int, int]] = []
        previous: tuple[int, int] | None = None
        while current in remaining:
            stroke.append(current)
            remaining.remove(current)
            candidates = available_neighbors(current)
            if not candidates:
                break
            if previous is None:
                next_point = min(candidates)
            else:
                direction = (current[0] - previous[0], current[1] - previous[1])
                next_point = max(
                    candidates,
                    key=lambda point: (point[0] - current[0]) * direction[0]
                    + (point[1] - current[1]) * direction[1],
                )
            previous, current = current, next_point
        strokes.append(stroke)
    return strokes


def optimize_stroke_order(
    strokes: Sequence[Sequence[tuple[int, int]]],
    image_shape: tuple[int, int],
) -> list[list[tuple[int, int]]]:
    """Order and orient strokes to reduce non-drawing robot travel.

    The tracer discovers strokes in image scan order, which can make the pen
    repeatedly cross the whole page. Nearest-endpoint routing chooses the next
    closest stroke and reverses it when its far end is closer. This is a fast
    approximation of the open travelling-salesperson problem. A subsequent
    2-opt pass removes avoidable crossings between those pen-up connections.
    """

    remaining = [list(stroke) for stroke in strokes if stroke]
    current = (image_shape[0] - 1, 0)  # Pixel corresponding to robot origin.
    ordered: list[list[tuple[int, int]]] = []
    while remaining:
        best_index = 0
        reverse = False
        best_distance = float("inf")
        for index, stroke in enumerate(remaining):
            for should_reverse, endpoint in ((False, stroke[0]), (True, stroke[-1])):
                distance = (endpoint[0] - current[0]) ** 2 + (endpoint[1] - current[1]) ** 2
                if distance < best_distance:
                    best_index = index
                    reverse = should_reverse
                    best_distance = distance
        selected = remaining.pop(best_index)
        if reverse:
            selected.reverse()
        ordered.append(selected)
        current = selected[-1]

    origin = (image_shape[0] - 1, 0)

    def distance(first: tuple[int, int], second: tuple[int, int]) -> float:
        return float(np.hypot(first[0] - second[0], first[1] - second[1]))

    # Reversing a range also reverses every stroke within it. The ink geometry
    # is unchanged, but the two pen-up links at the range boundaries may become
    # shorter. Repeat the best improvement until no crossing can be shortened.
    while len(ordered) > 1:
        best_change = -1e-9
        best_range: tuple[int, int] | None = None
        for start in range(len(ordered)):
            previous = origin if start == 0 else ordered[start - 1][-1]
            for end in range(start, len(ordered)):
                old_distance = distance(previous, ordered[start][0])
                new_distance = distance(previous, ordered[end][-1])
                if end + 1 < len(ordered):
                    following = ordered[end + 1][0]
                    old_distance += distance(ordered[end][-1], following)
                    new_distance += distance(ordered[start][0], following)
                change = new_distance - old_distance
                if change < best_change:
                    best_change = change
                    best_range = (start, end)
        if best_range is None:
            break
        start, end = best_range
        ordered[start:end + 1] = [
            list(reversed(stroke)) for stroke in reversed(ordered[start:end + 1])
        ]
    return ordered


def strokes_to_paths(
    strokes: Sequence[Sequence[tuple[int, int]]],
    image_shape: tuple[int, int],
    width: float = 400.0,
) -> list[list[tuple[float, float]]]:
    """Order pixel strokes and scale them into bottom-left plot coordinates."""

    if width <= 0:
        raise ValueError("width must be positive")
    rows, columns = image_shape
    scale = width / max(columns, 1)

    def robot_point(pixel: tuple[int, int]) -> tuple[float, float]:
        row, column = pixel
        return column * scale, (rows - 1 - row) * scale

    return [
        [robot_point(pixel) for pixel in stroke]
        for stroke in optimize_stroke_order(strokes, image_shape)
        if stroke
    ]


def strokes_to_moves(
    strokes: Sequence[Sequence[tuple[int, int]]],
    image_shape: tuple[int, int],
    width: float = 400.0,
    max_step: float = 1.0,
) -> list[Move]:
    """Scale pixel strokes to robot units and limit every XY movement."""

    return paths_to_moves(strokes_to_paths(strokes, image_shape, width), max_step)


def trace_image_to_svg(
    input_path: Path,
    output_path: Path,
    *,
    pixel_width: int = 400,
    pixel_height: int = 600,
    coordinate_width: float = 400.0,
    max_step: float = 1.0,
) -> TraceResult:
    """Run the line algorithm and write ordered centerline strokes as SVG."""

    from .path_svg import write_paths_svg

    if not input_path.is_file():
        raise ValueError(f"Input image does not exist: {input_path}")
    mask = image_to_mask(input_path, pixel_width, pixel_height)
    strokes = mask_to_strokes(mask)
    paths = strokes_to_paths(strokes, mask.shape, coordinate_width)
    moves = paths_to_moves(paths, max_step)
    coordinate_height = coordinate_width * mask.shape[0] / mask.shape[1]
    plot_y_max = coordinate_width * (mask.shape[0] - 1) / mask.shape[1]
    write_paths_svg(
        output_path,
        paths,
        width=coordinate_width,
        height=coordinate_height,
        plot_y_max=plot_y_max,
    )
    return TraceResult(mask.shape[1], mask.shape[0], len(strokes), tuple(moves))


def add_trace_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input", type=Path, default=DEFAULT_IMAGE, help=f"source image (default: {DEFAULT_IMAGE})")
    parser.add_argument("--output", required=True, type=Path, help="destination centerline SVG")
    parser.add_argument("--width", type=float, default=400.0, help="robot canvas width in coordinate units")
    parser.add_argument("--max-step", type=float, default=1.0, help="maximum distance per movement")
    parser.add_argument("--pixel-width", type=int, default=400, help="processing canvas width in pixels")
    parser.add_argument("--pixel-height", type=int, default=600, help="processing canvas height in pixels")


def run_trace_command(args: argparse.Namespace) -> None:
    started = perf_counter()
    try:
        result = trace_image_to_svg(
            args.input,
            args.output,
            pixel_width=args.pixel_width,
            pixel_height=args.pixel_height,
            coordinate_width=args.width,
            max_step=args.max_step,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error
    draw_moves = sum(move.pen_down for move in result.moves)
    print(
        f"Image: {args.input} | canvas: {result.canvas_width}x{result.canvas_height} px "
        f"| strokes: {result.stroke_count} | moves: {len(result.moves)} "
        f"({draw_moves} draw, {len(result.moves) - draw_moves} travel)"
    )
    print(f"Wrote {args.output} in {perf_counter() - started:.3f} seconds")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_trace_arguments(parser)
    run_trace_command(parser.parse_args())


if __name__ == "__main__":
    main()
