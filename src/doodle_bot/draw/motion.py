"""Shared robot movement model."""

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class Move:
    """One robot position. ``pen_down`` controls whether this move draws."""

    x: float
    y: float
    pen_down: bool


def _interpolate(
    start: tuple[float, float],
    end: tuple[float, float],
    max_step: float,
) -> Iterable[tuple[float, float]]:
    distance = float(np.hypot(end[0] - start[0], end[1] - start[1]))
    if distance == 0:
        return
    count = max(1, int(np.ceil(distance / max_step)))
    for index in range(1, count + 1):
        fraction = index / count
        yield (
            start[0] + (end[0] - start[0]) * fraction,
            start[1] + (end[1] - start[1]) * fraction,
        )


def paths_to_moves(
    paths: Sequence[Sequence[tuple[float, float]]],
    max_step: float = 1.0,
) -> list[Move]:
    """Convert ordered pen strokes into bounded pen-up and pen-down moves."""

    if max_step <= 0:
        raise ValueError("max_step must be positive")
    moves: list[Move] = []
    current = (0.0, 0.0)
    for path in paths:
        if not path:
            continue
        start = path[0]
        for point in _interpolate(current, start, max_step):
            moves.append(Move(*point, pen_down=False))
        current = start
        moves.append(Move(*current, pen_down=True))
        for target in path[1:]:
            for point in _interpolate(current, target, max_step):
                moves.append(Move(*point, pen_down=True))
            current = target
    return moves
