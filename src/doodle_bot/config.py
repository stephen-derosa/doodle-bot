from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import tomllib

from .contracts import ModelProfile


@dataclass(frozen=True)
class StageConfig:
    id: str
    type: str
    model_profile: str | None
    enabled: bool
    executor: str
    options: dict[str, Any]


@dataclass(frozen=True)
class PipelineConfig:
    name: str
    stages: tuple[StageConfig, ...]
    profiles: dict[str, ModelProfile]
    executors: dict[str, "ExecutorConfig"]


@dataclass(frozen=True)
class ExecutorConfig:
    """A named machine or service allowed to execute pipeline stages."""

    name: str
    kind: str
    url: str | None
    token_env: str | None
    timeout_seconds: int


def load_pipeline(path: Path) -> PipelineConfig:
    with path.open("rb") as handle:
        raw = tomllib.load(handle)
    profiles_path = (path.parent / raw["model_profiles"]).resolve()
    with profiles_path.open("rb") as handle:
        model_raw = tomllib.load(handle)["profiles"]
    profiles = {
        name: ModelProfile(name=name, provider=value["provider"], model=value["model"],
                           revision=value["revision"], options=value.get("options", {}),
                           parameters={key: item for key, item in value.items()
                                       if key not in {"provider", "model", "revision", "options"}})
        for name, value in model_raw.items()
    }
    executors = {
        name: ExecutorConfig(name=name, kind=value["kind"], url=value.get("url"),
                             token_env=value.get("token_env"), timeout_seconds=value.get("timeout_seconds", 120))
        for name, value in raw.get("executors", {"local": {"kind": "local"}}).items()
    }
    stages = tuple(
        StageConfig(id=item["id"], type=item["type"],
                    model_profile=item.get("model_profile"),
                    enabled=item.get("enabled", True), executor=item.get("executor", "local"),
                    options=item.get("options", {}))
        for item in raw["stages"]
    )
    for stage in stages:
        if stage.model_profile and stage.model_profile not in profiles:
            raise ValueError(f"Stage '{stage.id}' names unknown model profile '{stage.model_profile}'.")
        if stage.executor not in executors:
            raise ValueError(f"Stage '{stage.id}' names unknown executor '{stage.executor}'.")
    return PipelineConfig(name=raw["name"], stages=stages, profiles=profiles, executors=executors)
