"""Read and write DoodleBot movement CSV files."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

from .motion import Move


def write_moves_csv(path: Path, moves: Iterable[Move]) -> None:
    """Write an ordered robot command stream, starting with the pen up."""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        pen_down = False
        writer.writerow(("pos", "up"))
        for move in moves:
            if move.pen_down != pen_down:
                pen_down = move.pen_down
                writer.writerow(("pos", "down" if pen_down else "up"))
            writer.writerow(("coord", f"{move.x:.6f}", f"{move.y:.6f}"))
        if pen_down:
            writer.writerow(("pos", "up"))


def read_moves_csv(path: Path) -> list[Move]:
    """Read sequential pen-state and coordinate robot commands."""

    moves: list[Move] = []
    pen_down: bool | None = None
    with path.open(newline="", encoding="utf-8") as source:
        for line_number, row in enumerate(csv.reader(source), start=1):
            if not row or all(not field.strip() for field in row):
                continue
            record = row[0].strip().lower()
            if record == "pos":
                if len(row) != 2 or row[1].strip().lower() not in {"up", "down"}:
                    raise ValueError(f"line {line_number}: expected pos,up or pos,down")
                position = row[1].strip().lower()
                if pen_down is None and position != "up":
                    raise ValueError(f"line {line_number}: command stream must begin with pos,up")
                pen_down = position == "down"
            elif record == "coord":
                if len(row) != 3:
                    raise ValueError(f"line {line_number}: expected coord,x,y")
                if pen_down is None:
                    raise ValueError(f"line {line_number}: command stream must begin with pos,up")
                try:
                    coordinate = (float(row[1]), float(row[2]))
                except ValueError as error:
                    raise ValueError(f"line {line_number}: x and y must be numbers") from error
                moves.append(Move(*coordinate, pen_down=pen_down))
            else:
                raise ValueError(f"line {line_number}: expected a pos or coord command")
    if not moves:
        raise ValueError("CSV contains no movements")
    return moves
