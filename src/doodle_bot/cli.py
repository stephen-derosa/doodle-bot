from __future__ import annotations

import argparse
from pathlib import Path

from .config import load_pipeline
from .execution import ExecutorRouter, build_executors
from .registry import StageRegistry
from .runner import PipelineRunner
from .stages import builtin_stages


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a DoodleBot pipeline.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run")
    run.add_argument("--pipeline", required=True, type=Path)
    run.add_argument("--input", required=True, type=Path)
    run.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    registry = StageRegistry()
    for stage in builtin_stages():
        registry.register(stage)
    registry.load_plugins()
    config = load_pipeline(args.pipeline)
    result = PipelineRunner(registry, ExecutorRouter(build_executors(config.executors))).run(config, args.input, args.output)
    print(result.path)


if __name__ == "__main__":
    main()
