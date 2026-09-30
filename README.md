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

## Image-to-SVG-to-G-code drawing

All drawing implementation code lives in `src/doodle_bot/draw/`. The image
algorithm, vector format, G-code conversion, and GUI renderer are separate.
First trace the bundled line art into ordered centerline SVG strokes:

```bash
uv run doodle-bot trace --input images/char.png --output char.svg
```

Convert the SVG to absolute-position robot G-code:

```bash
uv run doodle-bot svg-to-gcode --input char.svg --output char.gcode
```

Convert that G-code into the JSON drawing format consumed by the SO-101 app:

```bash
uv run doodle-bot gcode-to-so101 \
  --input char.gcode \
  --output robot/assets/char.json
cd robot && uv run doodle preview assets/char.json
```

The JSON contains millimetre pen-down polylines. The SO-101 planner fits them
to its configured canvas and performs the calibration-dependent inverse
kinematics when previewing or drawing.

The converter accepts SVG paths and standard vector shapes, including lines,
polylines, polygons, rectangles, circles, ellipses, cubic/quadratic Béziers,
and arcs. It applies nested SVG transforms, adaptively flattens curves, then
orients and reorders strokes with nearest-endpoint routing plus 2-opt to reduce
pen-up travel. G-code movement spacing remains bounded by `--max-step`.
Pass `--join-distance N` to replace pen lifts across gaps of at most `N` SVG
units with short drawn connectors; it is disabled by default because joining
strokes changes the artwork slightly.

Render an existing SO-101 JSON drawing with the interactive GUI:

```bash
uv run doodle-bot render --input robot/assets/char.json
```

For the usual one-shot workflow, `draw` runs all three conversions, retains
their output, reads the final SO-101 JSON back through the plotting adapter,
and starts the GUI. Each invocation creates an `<image>-<timestamp>` folder
beside the input (or below `--output-root`) containing same-stem SVG, G-code,
and SO-101 JSON:

```bash
uv run doodle-bot draw --input images/char.png
# writes images/char-20260930-143900/{char.svg,char.gcode,char.json}
```

An SVG can also be supplied directly. In that case `draw` preserves the input
SVG in the run folder and starts at the SVG-to-G-code step:

```bash
uv run doodle-bot draw --input images/doodle.svg
```

The default input is `images/char.png`, fitted proportionally into a
400×600-pixel portrait canvas with matching 0–400 by 0–600 plot coordinates.
The plot shows pen-down points in black above the orange pen-up travel history,
while every executed coordinate is printed to the console. Red Xs mark pen
lifts and green Xs mark where drawing resumes. Strokes are ordered and reversed
using nearest-endpoint routing followed by 2-opt route improvement to avoid
unnecessary pen-up travel. The console also reports the complete preparation
time from image loading through SVG, G-code, and SO-101 JSON generation. Use
`trace --help`, `svg-to-gcode --help`, `gcode-to-so101 --help`, and
`render --help` for individual stages. Use `draw --help` for the one-shot
workflow.

The generated stream starts with `G90` for absolute positioning. Every
subsequent instruction is a `G1`: XY instructions move the tool and Z
instructions lift or lower the pen. The default is Z1 for up and Z0 for down;
use `--pen-up-z` and `--pen-down-z` to match the target machine.

```gcode
G90
G1 Z1.000000
G1 X120.000000 Y250.000000
G1 X121.000000 Y250.000000
G1 Z0.000000
G1 X121.000000 Y250.000000
G1 X122.000000 Y251.000000
G1 Z1.000000
```

Generated streams always begin and end with the configured pen-up Z state,
leaving the robot in a safe pen-lifted state.

Plot controls:

- Space: play or pause
- Up/Down: switch to paced mode and increase or decrease drawing speed
- Left: restart from a blank plot and immediately replay at 0.1 seconds/step
- H: show or hide orange travel points and red/green pen markers

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

## Whiteboard

The repo also includes a Next.js drawing whiteboard in `app/`. Install
dependencies and start the dev server from the repository root:

```bash
npm install
npm run dev
```

Open [http://localhost:3000](http://localhost:3000). The page is `app/page.tsx`;
the drawing surface lives in `app/whiteboard.tsx`.
