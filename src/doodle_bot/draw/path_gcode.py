"""Read and write DoodleBot movement G-code files."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from .motion import Move


def write_moves_gcode(
    path: Path,
    moves: Iterable[Move],
    *,
    pen_up_z: float = 1.0,
    pen_down_z: float = 0.0,
) -> None:
    """Write absolute-position G-code using only G1 movement commands."""

    if pen_up_z == pen_down_z:
        raise ValueError("pen-up-z and pen-down-z must differ")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write("G90\n")
        pen_down = False
        output.write(f"G1 Z{pen_up_z:.6f}\n")
        for move in moves:
            if move.pen_down != pen_down:
                pen_down = move.pen_down
                z = pen_down_z if pen_down else pen_up_z
                output.write(f"G1 Z{z:.6f}\n")
            output.write(f"G1 X{move.x:.6f} Y{move.y:.6f}\n")
        if pen_down:
            output.write(f"G1 Z{pen_up_z:.6f}\n")


def read_moves_gcode(
    path: Path,
    *,
    pen_up_z: float = 1.0,
    pen_down_z: float = 0.0,
) -> list[Move]:
    """Read the absolute G1 subset emitted by :func:`write_moves_gcode`."""

    if pen_up_z == pen_down_z:
        raise ValueError("pen-up-z and pen-down-z must differ")
    moves: list[Move] = []
    absolute = False
    pen_down: bool | None = None
    x: float | None = None
    y: float | None = None

    with path.open(encoding="utf-8") as source:
        for line_number, raw_line in enumerate(source, start=1):
            line = raw_line.split(";", 1)[0].strip()
            if not line:
                continue
            fields = line.upper().split()
            command = fields[0]
            if not absolute:
                if command != "G90" or len(fields) != 1:
                    raise ValueError(f"line {line_number}: command stream must begin with G90")
                absolute = True
                continue
            if command != "G1":
                raise ValueError(f"line {line_number}: expected a G1 command")

            values: dict[str, float] = {}
            for field in fields[1:]:
                axis = field[:1]
                if axis not in {"X", "Y", "Z"} or axis in values:
                    raise ValueError(f"line {line_number}: expected unique X, Y, or Z fields")
                try:
                    values[axis] = float(field[1:])
                except ValueError as error:
                    raise ValueError(f"line {line_number}: invalid {axis} value") from error
            if not values:
                raise ValueError(f"line {line_number}: G1 requires an X, Y, or Z value")

            if "Z" in values:
                z = values["Z"]
                if abs(z - pen_down_z) <= 1e-9:
                    pen_down = True
                elif abs(z - pen_up_z) <= 1e-9:
                    pen_down = False
                else:
                    raise ValueError(
                        f"line {line_number}: Z must be {pen_up_z:g} (up) "
                        f"or {pen_down_z:g} (down)"
                    )
            if "X" in values:
                x = values["X"]
            if "Y" in values:
                y = values["Y"]
            if "X" in values or "Y" in values:
                if pen_down is None:
                    raise ValueError(f"line {line_number}: set pen Z before moving in XY")
                if x is None or y is None:
                    raise ValueError(f"line {line_number}: first XY move must set both X and Y")
                moves.append(Move(x, y, pen_down))

    if not absolute:
        raise ValueError("G-code contains no G90 command")
    if not moves:
        raise ValueError("G-code contains no XY movements")
    return moves
