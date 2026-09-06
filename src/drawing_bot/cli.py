from __future__ import annotations

import argparse
from pathlib import Path

from .config import load_pipeline
from .registry import StageRegistry
from .runner import PipelineRunner
from .stages import builtin_stages


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a Drawing Bot pipeline.")
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
    result = PipelineRunner(registry).run(load_pipeline(args.pipeline), args.input, args.output)
    print(result.path)


if __name__ == "__main__":
    main()
