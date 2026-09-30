# Doodle Bot voice agent

A LiveKit voice agent that asks the user what the robot arm should draw. The user can describe it out loud or in text, turn on their camera, or both. It then generates a minimal line sketch with OpenAI (`gpt-image-2.5-flare`, low quality, for speed), writes it to `/tmp/doodle-*.jpg`, and prints the path to stdout.

The pipeline follows the defaults in the [LiveKit voice AI quickstart](https://docs.livekit.io/agents/start/voice-ai.md). It runs on LiveKit Inference: AssemblyAI Universal-3.5 Pro for STT, Gemma 4 31B as the LLM, Fish Audio S2.1 Pro for TTS, and the LiveKit turn detector, with ai-coustics noise cancellation. Camera frames are sampled with the pattern from the [video docs](https://docs.livekit.io/agents/multimodality/vision/video.md).

## Setup

```bash
cp .env.example .env.local   # or: lk app env -w
# fill in LiveKit credentials and OPENAI_API_KEY
uv sync
uv run agent.py download-files
```

## Run

```bash
uv run agent.py console   # talk to it in your terminal (no camera)
uv run agent.py dev       # connect to LiveKit; use the Agent Console with agent name "doodle-bot"
```
