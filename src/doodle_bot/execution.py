from __future__ import annotations

import base64
import json
import mimetypes
import os
from pathlib import Path
from typing import Protocol
from urllib.request import Request, urlopen

from .config import ExecutorConfig
from .contracts import Artifact, Stage, StageRequest


class Executor(Protocol):
    """Runs a stage locally or delegates it to another machine."""

    def execute(self, stage: Stage, artifact: Artifact, request: StageRequest) -> Artifact: ...


class LocalExecutor:
    def execute(self, stage: Stage, artifact: Artifact, request: StageRequest) -> Artifact:
        return stage.run(artifact, request)


class HttpExecutor:
    """Stateless worker protocol, suitable for a dedicated GPU machine."""

    def __init__(self, config: ExecutorConfig) -> None:
        if not config.url:
            raise ValueError(f"HTTP executor '{config.name}' requires a URL.")
        self.config = config

    def execute(self, stage: Stage, artifact: Artifact, request: StageRequest) -> Artifact:
        payload = {
            "stage_type": stage.type_name,
            "input": {"filename": artifact.path.name, "media_type": artifact.media_type,
                      "data_base64": base64.b64encode(artifact.path.read_bytes()).decode("ascii"),
                      "metadata": artifact.metadata},
            "options": request.options,
            "model": None if request.model is None else {"name": request.model.name, "provider": request.model.provider,
                      "model": request.model.model, "revision": request.model.revision, "options": request.model.options,
                      "parameters": request.model.parameters},
        }
        headers = {"Content-Type": "application/json"}
        if self.config.token_env:
            token = os.environ.get(self.config.token_env)
            if not token:
                raise ValueError(f"Executor '{self.config.name}' requires environment variable {self.config.token_env}.")
            headers["Authorization"] = f"Bearer {token}"
        endpoint = f"{self.config.url.rstrip('/')}/v1/stages/{stage.type_name}"
        with urlopen(Request(endpoint, data=json.dumps(payload).encode(), headers=headers, method="POST"),
                     timeout=self.config.timeout_seconds) as response:
            result = json.load(response)
        media_type = result["media_type"]
        filename = Path(result.get("filename", request.stage_id)).name
        extension = Path(filename).suffix or mimetypes.guess_extension(media_type) or ".bin"
        target = request.run_dir / f"{request.stage_id}{extension}"
        target.write_bytes(base64.b64decode(result["data_base64"]))
        return Artifact(target, media_type, artifact.lineage + (request.stage_id,),
                        {**artifact.metadata, **result.get("metadata", {}), "executor": self.config.name})


class ExecutorRouter:
    def __init__(self, executors: dict[str, Executor]) -> None:
        self.executors = executors

    def get(self, name: str) -> Executor:
        try:
            return self.executors[name]
        except KeyError as error:
            raise KeyError(f"No initialized executor named '{name}'.") from error


def build_executors(configs: dict[str, ExecutorConfig]) -> dict[str, Executor]:
    built: dict[str, Executor] = {}
    for name, config in configs.items():
        if config.kind == "local":
            built[name] = LocalExecutor()
        elif config.kind == "http":
            built[name] = HttpExecutor(config)
        else:
            raise ValueError(f"Unsupported executor kind '{config.kind}' for '{name}'.")
    return built
