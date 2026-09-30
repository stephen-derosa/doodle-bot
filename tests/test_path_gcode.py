import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from doodle_bot.draw.motion import Move
from doodle_bot.draw.path_gcode import read_moves_gcode, write_moves_gcode


class PathGcodeTests(unittest.TestCase):
    def test_round_trip_uses_absolute_g1_commands(self) -> None:
        expected = [Move(1.25, 2.5, False), Move(3.0, 4.0, True)]
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "drawing.gcode"
            write_moves_gcode(path, expected)
            content = path.read_text()
            actual = read_moves_gcode(path)
        self.assertEqual(
            content.splitlines(),
            [
                "G90",
                "G1 Z1.000000",
                "G1 X1.250000 Y2.500000",
                "G1 Z0.000000",
                "G1 X3.000000 Y4.000000",
                "G1 Z1.000000",
            ],
        )
        self.assertEqual(actual, expected)

    def test_rejects_stream_without_absolute_positioning(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "invalid.gcode"
            path.write_text("G1 X1 Y2\n")
            with self.assertRaisesRegex(ValueError, "begin with G90"):
                read_moves_gcode(path)

    def test_supports_configurable_pen_heights(self) -> None:
        expected = [Move(2.0, 3.0, True)]
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "drawing.gcode"
            write_moves_gcode(path, expected, pen_up_z=5.0, pen_down_z=-1.0)
            actual = read_moves_gcode(path, pen_up_z=5.0, pen_down_z=-1.0)
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
