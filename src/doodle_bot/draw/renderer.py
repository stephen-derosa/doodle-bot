"""Render a DoodleBot movement CSV with interactive playback controls."""

from __future__ import annotations

import argparse
from pathlib import Path
from time import perf_counter
from typing import Sequence

import numpy as np

from .motion import Move
from .path_csv import read_moves_csv


FAST_TIMER_INTERVAL_MS = 1
SLOW_STEP_DELAY = 0.1


def interactive_plot(
    moves: Sequence[Move],
    width: float = 400.0,
    height: float = 600.0,
    preparation_started: float | None = None,
    step_delay: float = SLOW_STEP_DELAY,
) -> None:
    """Play CSV movements with the existing keyboard controls and styling."""

    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(4, 6), dpi=100, constrained_layout=True)
    axes.set_xlabel("X")
    axes.set_ylabel("Y")
    axes.set_xlim(0, width)
    axes.set_ylim(0, height)
    axes.set_xticks(np.linspace(0, width, 5))
    axes.set_yticks(np.linspace(0, height, 7))
    axes.set_aspect("equal", adjustable="box")
    axes.grid(alpha=0.2)
    travel_artist = axes.scatter([], [], s=9, color="tab:orange", zorder=2, label="Pen up")
    draw_artist = axes.scatter([], [], s=9, color="black", zorder=3, label="Pen down")
    lift_artist = axes.scatter([], [], s=48, marker="x", linewidths=1.8,
                               color="red", zorder=4, label="Pen lifted")
    resume_artist = axes.scatter([], [], s=48, marker="x", linewidths=1.8,
                                 color="green", zorder=4, label="Pen resumed")
    axes.legend(loc="upper right")
    state = {
        "index": 0,
        "playing": False,
        "final_lift_added": False,
        "step_delay": max(0.001, step_delay),
        "steps_visible": True,
        "mode": "fast",
    }
    draw_points: list[tuple[float, float]] = []
    travel_points: list[tuple[float, float]] = []
    lift_points: list[tuple[float, float]] = []
    resume_points: list[tuple[float, float]] = []

    def offsets(points: list[tuple[float, float]]) -> np.ndarray:
        return np.asarray(points) if points else np.empty((0, 2))

    def refresh_artists() -> None:
        travel_artist.set_offsets(offsets(travel_points))
        draw_artist.set_offsets(offsets(draw_points))
        lift_artist.set_offsets(offsets(lift_points))
        resume_artist.set_offsets(offsets(resume_points))
        figure.canvas.draw_idle()

    def update_title(status: str) -> None:
        speed = "MAX speed" if state["mode"] == "fast" else f"{1 / state['step_delay']:.1f} steps/s"
        axes.set_title(f"{width:g}×{height:g} — {status} — {speed}")

    def execute_move(index: int) -> str:
        move = moves[index]
        previous = moves[index - 1] if index else None
        if not move.pen_down:
            travel_points.append((move.x, move.y))
            if previous is not None and previous.pen_down:
                lift_points.append((previous.x, previous.y))
        else:
            draw_points.append((move.x, move.y))
            if previous is None or not previous.pen_down:
                resume_points.append((move.x, move.y))
        action = "DRAW" if move.pen_down else "MOVE"
        return f"{index + 1:05d}/{len(moves):05d} {action:4s} X={move.x:8.3f} Y={move.y:8.3f}"

    def advance_playback() -> None:
        if not state["playing"]:
            return
        output: list[str] = []
        if state["mode"] == "fast":
            deadline = perf_counter() + 1 / 60
            while state["index"] < len(moves) and perf_counter() < deadline:
                output.append(execute_move(state["index"]))
                state["index"] += 1
        elif state["index"] < len(moves):
            output.append(execute_move(state["index"]))
            state["index"] += 1
        if output:
            print("\n".join(output), flush=True)
        if state["index"] >= len(moves):
            state["playing"] = False
            timer.stop()
            if moves and moves[-1].pen_down and not state["final_lift_added"]:
                lift_points.append((moves[-1].x, moves[-1].y))
                state["final_lift_added"] = True
            update_title("drawing complete")
            print("Drawing complete.", flush=True)
        refresh_artists()

    timer = figure.canvas.new_timer(interval=FAST_TIMER_INTERVAL_MS)
    timer.add_callback(advance_playback)

    def apply_timer_interval() -> None:
        timer.interval = FAST_TIMER_INTERVAL_MS if state["mode"] == "fast" else max(
            1, round(state["step_delay"] * 1000)
        )

    def toggle_playback() -> None:
        if state["index"] >= len(moves):
            return
        state["playing"] = not state["playing"]
        if state["playing"]:
            apply_timer_interval()
            update_title("playing (SPACE to pause)")
            print("Playback started.", flush=True)
            timer.start()
        else:
            timer.stop()
            update_title("paused (SPACE to resume)")
            print("Playback paused.", flush=True)
        figure.canvas.draw_idle()

    def change_speed(faster: bool) -> None:
        was_playing = state["playing"]
        if was_playing:
            timer.stop()
        state["mode"] = "slow"
        state["step_delay"] = min(2.0, max(0.001, state["step_delay"] * (0.8 if faster else 1.25)))
        apply_timer_interval()
        if was_playing:
            timer.start()
        update_title("playing (SPACE to pause)" if state["playing"] else "paused")
        print(f"Speed: {1 / state['step_delay']:.1f} steps/s ({state['step_delay']:.3f} seconds/step)", flush=True)
        figure.canvas.draw_idle()

    def toggle_step_points() -> None:
        state["steps_visible"] = not state["steps_visible"]
        for artist in (travel_artist, lift_artist, resume_artist):
            artist.set_visible(state["steps_visible"])
        print(f"Step points {'shown' if state['steps_visible'] else 'hidden'}.", flush=True)
        figure.canvas.draw_idle()

    def restart() -> None:
        timer.stop()
        state.update(index=0, playing=True, final_lift_added=False, mode="slow", step_delay=SLOW_STEP_DELAY)
        draw_points.clear()
        travel_points.clear()
        lift_points.clear()
        resume_points.clear()
        apply_timer_interval()
        update_title("slow replay (SPACE to pause)")
        refresh_artists()
        print("Playback restarted from blank in slow mode (0.100 seconds/step).", flush=True)
        timer.start()

    def handle_key(event: object) -> None:
        key = getattr(event, "key", None)
        if key in {" ", "space"}:
            toggle_playback()
        elif key == "up":
            change_speed(faster=True)
        elif key == "down":
            change_speed(faster=False)
        elif key == "left":
            restart()
        elif key in {"h", "H"}:
            toggle_step_points()

    figure.canvas.mpl_connect("key_press_event", handle_key)
    figure.canvas.mpl_connect("close_event", lambda _event: timer.stop())
    update_title("paused")
    figure.canvas.draw()
    if preparation_started is not None:
        print(f"Preparation time (CSV load -> plot ready): {perf_counter() - preparation_started:.3f} seconds")
    print(
        f"Ready: {len(moves)} movements in maximum-speed mode.\n"
        "Controls: SPACE play/pause | LEFT slow restart | UP/DOWN paced speed | H show/hide step points"
    )
    plt.show()


def add_render_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--input", required=True, type=Path, help="movement CSV to render")
    parser.add_argument("--width", type=float, default=400.0, help="plot width in coordinate units")
    parser.add_argument("--height", type=float, default=600.0, help="plot height in coordinate units")


def run_render_command(args: argparse.Namespace) -> None:
    if not args.input.is_file():
        raise SystemExit(f"Input CSV does not exist: {args.input}")
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("width and height must be positive")
    started = perf_counter()
    try:
        moves = read_moves_csv(args.input)
    except ValueError as error:
        raise SystemExit(f"Invalid movement CSV: {error}") from error
    print(f"CSV: {args.input} | canvas: {args.width:g}x{args.height:g} | moves: {len(moves)}")
    interactive_plot(moves, args.width, args.height, started)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_render_arguments(parser)
    run_render_command(parser.parse_args())


if __name__ == "__main__":
    main()
