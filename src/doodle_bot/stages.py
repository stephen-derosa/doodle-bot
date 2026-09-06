from __future__ import annotations

import shutil
import base64
from pathlib import Path

from .contracts import Artifact, StageRequest


class _PassThroughImageStage:
    """Temporary adapter boundary for an image-model implementation."""

    accepted_media_types = frozenset({"image/jpeg", "image/png", "image/webp"})

    def can_handle(self, artifact: Artifact) -> bool:
        return artifact.media_type in self.accepted_media_types

    def run(self, artifact: Artifact, request: StageRequest) -> Artifact:
        extension = artifact.path.suffix or ".png"
        target = request.run_dir / f"{request.stage_id}{extension}"
        shutil.copy2(artifact.path, target)
        return Artifact(path=target, media_type=artifact.media_type,
                        lineage=artifact.lineage + (request.stage_id,),
                        metadata={**artifact.metadata, "placeholder": True,
                                  "stage": self.type_name, "model_profile": request.model.name if request.model else None})


class SceneCaricatureStage(_PassThroughImageStage):
    type_name = "image.scene_caricature"


class LineArtStage(_PassThroughImageStage):
    type_name = "image.line_art"


class SvgStage:
    type_name = "vector.svg"

    def can_handle(self, artifact: Artifact) -> bool:
        return artifact.media_type in {"image/jpeg", "image/png", "image/webp", "image/svg+xml"}

    def run(self, artifact: Artifact, request: StageRequest) -> Artifact:
        target = request.run_dir / f"{request.stage_id}.svg"
        if artifact.media_type == "image/svg+xml":
            shutil.copy2(artifact.path, target)
        else:
            encoded = base64.b64encode(artifact.path.read_bytes()).decode("ascii")
            target.write_text(
                "<svg xmlns=\"http://www.w3.org/2000/svg\" "
                "xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 1 1\">"
                f"<image width=\"1\" height=\"1\" xlink:href=\"data:{artifact.media_type};base64,{encoded}\"/>"
                "</svg>"
            )
        return Artifact(path=target, media_type="image/svg+xml", lineage=artifact.lineage + (request.stage_id,),
                        metadata={**artifact.metadata, "stage": self.type_name, "placeholder": True})


def builtin_stages() -> tuple[object, ...]:
    return SceneCaricatureStage(), LineArtStage(), SvgStage()
