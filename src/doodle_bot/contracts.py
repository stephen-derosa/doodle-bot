from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Protocol


@dataclass(frozen=True)
class Artifact:
    """A file plus immutable metadata passed between stages."""

    path: Path
    media_type: str
    lineage: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ModelProfile:
    name: str
    provider: str
    model: str
    revision: str
    options: Mapping[str, Any]
    parameters: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class StageRequest:
    run_dir: Path
    stage_id: str
    options: Mapping[str, Any]
    model: ModelProfile | None


class Stage(Protocol):
    """A replaceable transformation in the pipeline."""

    type_name: str

    def can_handle(self, artifact: Artifact) -> bool: ...

    def run(self, artifact: Artifact, request: StageRequest) -> Artifact: ...
