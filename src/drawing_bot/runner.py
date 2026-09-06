from __future__ import annotations

import json
import mimetypes
from dataclasses import asdict
from pathlib import Path

from .config import PipelineConfig
from .contracts import Artifact, StageRequest
from .registry import StageRegistry


class PipelineRunner:
    def __init__(self, registry: StageRegistry) -> None:
        self.registry = registry

    def run(self, config: PipelineConfig, source: Path, output_dir: Path) -> Artifact:
        output_dir.mkdir(parents=True, exist_ok=True)
        media_type, _ = mimetypes.guess_type(source.name)
        if not media_type:
            raise ValueError(f"Cannot infer media type for {source}")
        artifact = Artifact(path=source.resolve(), media_type=media_type)
        records: list[dict[str, object]] = []
        for stage_config in config.stages:
            if not stage_config.enabled:
                continue
            stage = self.registry.get(stage_config.type)
            if not stage.can_handle(artifact):
                raise ValueError(f"Stage '{stage_config.id}' ({stage_config.type}) cannot accept {artifact.media_type}. "
                                 "Add or configure a compatible conversion stage first.")
            model = config.profiles.get(stage_config.model_profile) if stage_config.model_profile else None
            artifact = stage.run(artifact, StageRequest(output_dir, stage_config.id, stage_config.options, model))
            records.append({"id": stage_config.id, "type": stage_config.type,
                            "model_profile": stage_config.model_profile, "artifact": str(artifact.path),
                            "media_type": artifact.media_type})
        (output_dir / "provenance.json").write_text(json.dumps({"pipeline": config.name, "stages": records}, indent=2))
        return artifact
