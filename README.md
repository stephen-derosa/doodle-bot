# DoodleBot

An extensible, artifact-based pipeline for turning a source photo into a themed
caricature, clean black-and-white line art, SVG, and any later output format.

## Pipeline flow

```text
source photo
  -> scene_caricature (image model)
  -> line_art (line-art model or CV processor)
  -> svg (vectorizer)
  -> export (optional: plotter, PDF, G-code, PNG, ...)
```

Each arrow is a `Stage`. A pipeline config selects a stage implementation and
a versioned model profile for every step. Stages receive one `Artifact` and
return one `Artifact`; the runner writes every result and its provenance to
disk. This keeps intermediate approvals, retries, and model comparisons easy.

## Quick start

The included scene and line-art stages are transparent pass-through adapters;
the SVG stage wraps the resulting raster in an SVG container. They make the
orchestration runnable while real model integrations are added. Their output
metadata is explicitly marked as a placeholder, so it cannot be mistaken for
finished generative or vector artwork.

```bash
python3 -m doodle_bot.cli run \
    --pipeline configs/pipelines/default.toml \
    --input /path/to/photo.jpg \
    --output ./runs/demo
```

The input must be an image. Replace the placeholder stages with real model and
vectorizer implementations before production use.

## Adding a scene effect or replacing a model

Model profiles live in `configs/models.toml`. Add a profile and set the
pipeline stage's `model_profile` to its identifier. The stage receives the
resolved profile, including provider, model id, revision, prompt template and
provider-specific options. No pipeline-runner code changes are needed.

## Adding a final format

Implement `Stage`, register it, then append it to a pipeline config:

```toml
[[stages]]
id = "plotter"
type = "export.plotter_hpgl"
enabled = true
```

The stage's `can_handle` method should accept `image/svg+xml` and return the
new media type, for example `application/vnd.hp-hpgl`. The artifact store will
preserve the generated `.hpgl` file and its provenance automatically.

## Plugin packages

External packages can expose stages through Python entry points:

```toml
[project.entry-points."doodle_bot.stages"]
"my.stage" = "my_package.stages:MyStage"
```

Use a fully qualified stage type such as `my.stage` in a pipeline config.

## Local and remote execution

Every stage chooses an `executor`. `local` is the default; a named HTTP worker
can be selected for GPU-heavy scene generation, upscaling, or vectorization.
The pipeline retains a single artifact lineage even when work moves between
machines. The remote worker receives the source artifact, stage options, and
resolved model profile, then returns its generated artifact to the local run.

```toml
[[stages]]
id = "scene"
type = "image.scene_caricature"
executor = "gpu-worker"
model_profile = "scene_v2"

[executors.gpu-worker]
kind = "http"
url = "https://gpu-worker.example.internal"
token_env = "DOODLEBOT_GPU_WORKER_TOKEN"
timeout_seconds = 180
```

The worker protocol is `POST /v1/stages/{stage-type}`. It receives JSON with
the base64 input artifact, stage options, and model profile, and returns JSON
with `data_base64`, `media_type`, optional `filename`, and `metadata`.
Credentials stay in environment variables, never model or pipeline files.
