import asyncio
import base64
import os
import tempfile
import time

from dotenv import load_dotenv
from livekit import agents, rtc
from livekit.agents import (
    Agent,
    AgentServer,
    AgentSession,
    RunContext,
    TurnHandlingOptions,
    function_tool,
    get_job_context,
    inference,
    room_io,
)
from livekit.agents.utils.images import EncodeOptions, ResizeOptions, encode
from livekit.plugins import ai_coustics
from openai import AsyncOpenAI

load_dotenv(".env.local")

IMAGE_MODEL = os.getenv("OPENAI_IMAGE_MODEL", "gpt-image-2.5-flare")
OUTPUT_DIR = "/tmp"

SKETCH_PROMPT = (
    "Make a caricature of this person at {ask}. Make it a simple sketch that can a robot "
    "arm with a pen on the end could draw. Keep resolution minimal, assume no more than "
    "5-6 lines which a robot could draw in under 1 minute. Do this as fast as possible."
)

INSTRUCTIONS = """You are Doodle Bot, the friendly voice of a robot arm that draws with a pen.
Your job is to find out what the user would like the robot arm to draw for them.

Start by telling the user they can describe an image they would like the robot arm to draw.
They can describe it in words, turn on their camera so the robot can sketch them, or both.
A typical request is a caricature of the user somewhere, like surfing at the beach or on the moon.

Once the user has described what they want, call the draw_sketch tool right away.
Pass their request as a short scene description, for example "the beach riding a surfboard".
Set use_camera to true unless the user says not to use their camera.
If the request is unclear, ask one short clarifying question before calling the tool.

Your responses are short, spoken aloud, and use no formatting, emojis, asterisks, or other symbols.
You are playful and encouraging."""


class DoodleBot(Agent):
    def __init__(self) -> None:
        super().__init__(instructions=INSTRUCTIONS)
        self._openai = AsyncOpenAI()
        self._latest_frame: rtc.VideoFrame | None = None
        self._video_stream: rtc.VideoStream | None = None
        self._tasks: list[asyncio.Task] = []

    async def on_enter(self) -> None:
        room = get_job_context().room

        # Pick up a camera track that is already published
        for participant in room.remote_participants.values():
            for publication in participant.track_publications.values():
                if publication.track and publication.track.kind == rtc.TrackKind.KIND_VIDEO:
                    self._create_video_stream(publication.track)
                    break

        # Watch for camera tracks published later
        @room.on("track_subscribed")
        def on_track_subscribed(
            track: rtc.Track,
            publication: rtc.RemoteTrackPublication,
            participant: rtc.RemoteParticipant,
        ) -> None:
            if track.kind == rtc.TrackKind.KIND_VIDEO:
                self._create_video_stream(track)

        @room.on("track_unsubscribed")
        def on_track_unsubscribed(
            track: rtc.Track,
            publication: rtc.RemoteTrackPublication,
            participant: rtc.RemoteParticipant,
        ) -> None:
            if track.kind == rtc.TrackKind.KIND_VIDEO:
                self._latest_frame = None

        await self.session.generate_reply(
            instructions="Greet the user and explain that they can describe a picture for the "
            "robot arm to draw, either in words, with their camera, or both."
        )

    def _create_video_stream(self, track: rtc.Track) -> None:
        # Only keep one stream open at a time
        if self._video_stream is not None:
            old = self._video_stream
            self._video_stream = None
            asyncio.create_task(old.aclose())

        self._video_stream = rtc.VideoStream(track)

        async def read_stream(stream: rtc.VideoStream) -> None:
            async for event in stream:
                self._latest_frame = event.frame

        task = asyncio.create_task(read_stream(self._video_stream))
        task.add_done_callback(lambda t: self._tasks.remove(t))
        self._tasks.append(task)

    @function_tool()
    async def draw_sketch(self, ctx: RunContext, description: str, use_camera: bool = True) -> str:
        """Generate a simple line sketch for the robot arm to draw, based on the user's request.

        Call this once the user has described what they want drawn.

        Args:
            description: Where the user is or what they are doing in the drawing, for example
                "the beach riding a surfboard". This completes the sentence
                "Make a caricature of this person at ...".
            use_camera: Whether to attach the user's current camera frame as a reference photo.
        """
        frame = self._latest_frame if use_camera else None
        await ctx.update(
            "Started generating the sketch"
            + (" using the user's camera." if frame else " from the text description only.")
        )

        prompt = SKETCH_PROMPT.format(ask=description.strip())
        async with ctx.with_filler("Still sketching, almost there.", delay=6, interval=10):
            image_bytes = await self._generate_sketch(prompt, frame)

        fd, path = tempfile.mkstemp(prefix=f"doodle-{int(time.time())}-", suffix=".jpg", dir=OUTPUT_DIR)
        with os.fdopen(fd, "wb") as f:
            f.write(image_bytes)
        print(path, flush=True)

        return "The sketch is ready and has been sent to the robot arm."

    async def _generate_sketch(self, prompt: str, frame: rtc.VideoFrame | None) -> bytes:
        options = dict(
            model=IMAGE_MODEL,
            prompt=prompt,
            size="1024x1024",
            quality="low",
            output_format="jpeg",
        )
        if frame is not None:
            photo = encode(
                frame,
                EncodeOptions(
                    format="JPEG",
                    resize_options=ResizeOptions(width=1024, height=1024, strategy="scale_aspect_fit"),
                ),
            )
            result = await self._openai.images.edit(image=("camera.jpg", photo, "image/jpeg"), **options)
        else:
            result = await self._openai.images.generate(**options)
        return base64.b64decode(result.data[0].b64_json)


server = AgentServer()


@server.rtc_session(agent_name="doodle-bot")
async def doodle_bot(ctx: agents.JobContext) -> None:
    session = AgentSession(
        stt=inference.STT(model="assemblyai/universal-3-5-pro", language="en"),
        llm=inference.LLM(model="google/gemma-4-31b-it"),
        tts=inference.TTS(
            model="fishaudio/s2.1-pro",
            voice="fa4c9eb3dccc4806b382b40d61c6b10a",
        ),
        turn_handling=TurnHandlingOptions(
            turn_detection=inference.TurnDetector(),
        ),
    )

    await session.start(
        room=ctx.room,
        agent=DoodleBot(),
        room_options=room_io.RoomOptions(
            audio_input=room_io.AudioInputOptions(
                noise_cancellation=ai_coustics.audio_enhancement(model=ai_coustics.EnhancerModel.QUAIL_VF_S),
            ),
        ),
    )


if __name__ == "__main__":
    agents.cli.run_app(server)
