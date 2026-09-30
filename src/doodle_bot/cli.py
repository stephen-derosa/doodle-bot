from __future__ import annotations

import argparse
from pathlib import Path

from .config import load_pipeline
from .draw.drawing import add_draw_arguments, run_draw_command
from .draw.path_so101 import (
    add_gcode_to_so101_arguments,
    run_gcode_to_so101_command,
)
from .draw.tracer import add_trace_arguments, run_trace_command
from .execution import ExecutorRouter, build_executors
from .registry import StageRegistry
from .runner import PipelineRunner
from .draw.renderer import add_render_arguments, run_render_command
from .draw.svg_converter import add_svg_to_gcode_arguments, run_svg_to_gcode_command
from .stages import builtin_stages


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a DoodleBot pipeline.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run")
    run.add_argument("--pipeline", required=True, type=Path)
    run.add_argument("--input", required=True, type=Path)
    run.add_argument("--output", required=True, type=Path)
    trace = subparsers.add_parser("trace", help="Convert an image to centerline SVG.")
    add_trace_arguments(trace)
    svg_to_gcode = subparsers.add_parser("svg-to-gcode", help="Convert centerline SVG to movement G-code.")
    add_svg_to_gcode_arguments(svg_to_gcode)
    gcode_to_so101 = subparsers.add_parser(
        "gcode-to-so101",
        help="Convert movement G-code to an SO-101 drawing JSON file.",
    )
    add_gcode_to_so101_arguments(gcode_to_so101)
    render = subparsers.add_parser("render", help="Render an SO-101 drawing JSON interactively.")
    add_render_arguments(render)
    draw = subparsers.add_parser(
        "draw",
        help="Trace an image through SVG and G-code, then render it interactively.",
    )
    add_draw_arguments(draw)
    args = parser.parse_args()
    if args.command == "trace":
        run_trace_command(args)
        return
    if args.command == "svg-to-gcode":
        run_svg_to_gcode_command(args)
        return
    if args.command == "gcode-to-so101":
        run_gcode_to_so101_command(args)
        return
    if args.command == "draw":
        run_draw_command(args)
        return
    if args.command == "render":
        run_render_command(args)
        return
    registry = StageRegistry()
    for stage in builtin_stages():
        registry.register(stage)
    registry.load_plugins()
    config = load_pipeline(args.pipeline)
    result = PipelineRunner(registry, ExecutorRouter(build_executors(config.executors))).run(config, args.input, args.output)
    print(result.path)


if __name__ == "__main__":
    main()
