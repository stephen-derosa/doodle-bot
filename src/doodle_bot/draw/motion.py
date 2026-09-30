"""Shared robot movement model."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Move:
    """One robot position. ``pen_down`` controls whether this move draws."""

    x: float
    y: float
    pen_down: bool
