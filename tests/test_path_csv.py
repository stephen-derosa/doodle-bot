import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from doodle_bot.draw.motion import Move
from doodle_bot.draw.path_csv import read_moves_csv, write_moves_csv


class PathCsvTests(unittest.TestCase):
    def test_round_trip_uses_state_change_commands(self) -> None:
        expected = [Move(1.25, 2.5, False), Move(3.0, 4.0, True)]
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "drawing.csv"
            write_moves_csv(path, expected)
            content = path.read_text()
            actual = read_moves_csv(path)
        self.assertEqual(
            content.splitlines(),
            [
                "pos,up",
                "coord,1.250000,2.500000",
                "pos,down",
                "coord,3.000000,4.000000",
                "pos,up",
            ],
        )
        self.assertEqual(actual, expected)

    def test_rejects_stream_without_initial_pen_up(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "invalid.csv"
            path.write_text("coord,1,2\n")
            with self.assertRaisesRegex(ValueError, "begin with pos,up"):
                read_moves_csv(path)


if __name__ == "__main__":
    unittest.main()
