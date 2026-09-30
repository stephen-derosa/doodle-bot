import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from doodle_bot.draw.motion import Move
from doodle_bot.draw.path_so101 import (
    gcode_to_so101,
    moves_to_strokes,
    read_moves_so101,
    read_so101_strokes,
)


class PathSo101Tests(unittest.TestCase):
    def test_groups_pen_down_moves_into_strokes(self) -> None:
        moves = [
            Move(1.0, 2.0, False),
            Move(1.0, 2.0, True),
            Move(3.0, 4.0, True),
            Move(8.0, 9.0, False),
            Move(10.0, 11.0, True),
        ]

        self.assertEqual(
            moves_to_strokes(moves),
            [
                [[1.0, 2.0], [3.0, 4.0]],
                [[8.0, 9.0], [10.0, 11.0]],
            ],
        )

    def test_writes_robot_drawing_json(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            input_path = root / "example.gcode"
            output_path = root / "nested" / "example.json"
            input_path.write_text(
                "G90\n"
                "G1 Z1\n"
                "G1 X1 Y2\n"
                "G1 Z0\n"
                "G1 X1 Y2\n"
                "G1 X3.123456 Y4.5\n"
                "G1 Z1\n"
            )

            drawing = gcode_to_so101(input_path, output_path)

            self.assertEqual(
                drawing,
                {
                    "name": "example",
                    "units": "mm",
                    "strokes": [[[1.0, 2.0], [3.1235, 4.5]]],
                },
            )
            self.assertEqual(json.loads(output_path.read_text()), drawing)
            self.assertEqual(
                read_so101_strokes(output_path),
                [[(1.0, 2.0), (3.1235, 4.5)]],
            )
            moves = read_moves_so101(output_path, max_step=10.0)
            self.assertEqual(moves[-1], Move(3.1235, 4.5, True))

    def test_rejects_gcode_without_pen_down_moves(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            input_path = root / "travel.gcode"
            input_path.write_text("G90\nG1 Z1\nG1 X1 Y2\n")

            with self.assertRaisesRegex(ValueError, "no pen-down"):
                gcode_to_so101(input_path, root / "travel.json")

    def test_rejects_invalid_so101_points(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "invalid.json"
            path.write_text('{"units":"mm","strokes":[[[1]]]}\n')

            with self.assertRaisesRegex(ValueError, "must contain two numbers"):
                read_so101_strokes(path)


if __name__ == "__main__":
    unittest.main()
