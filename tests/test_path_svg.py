import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from doodle_bot.draw.motion import paths_to_moves
from doodle_bot.draw.path_gcode import read_moves_gcode
from doodle_bot.draw.path_svg import (
    join_close_paths,
    optimize_paths,
    read_paths_svg,
    svg_to_gcode,
    write_paths_svg,
)


class PathSvgTests(unittest.TestCase):
    def test_round_trip_preserves_paths_and_bottom_left_coordinates(self) -> None:
        expected = [[(1.0, 2.0), (3.0, 4.0)], [(8.0, 9.0)]]
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "drawing.svg"
            write_paths_svg(path, expected, width=10.0, height=12.0, plot_y_max=11.0)
            actual = read_paths_svg(path)
        self.assertEqual(actual, expected)

    def test_svg_to_gcode_preserves_plotting_moves(self) -> None:
        paths = [[(2.0, 0.0), (4.0, 0.0)], [(4.0, 3.0)]]
        with TemporaryDirectory() as temporary_directory:
            svg_path = Path(temporary_directory) / "drawing.svg"
            gcode_path = Path(temporary_directory) / "drawing.gcode"
            write_paths_svg(svg_path, paths, width=5.0, height=4.0, plot_y_max=3.0)
            svg_to_gcode(svg_path, gcode_path, max_step=1.0)
            actual = read_moves_gcode(gcode_path)
        self.assertEqual(actual, paths_to_moves(paths, max_step=1.0))

    def test_reads_basic_svg_path_commands(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "drawing.svg"
            path.write_text(
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">'
                '<path d="M 1 9 L 2 8 h 1 v -1"/>'
                "</svg>"
            )
            actual = read_paths_svg(path)
        self.assertEqual(actual, [[(1.0, 1.0), (2.0, 2.0), (3.0, 2.0), (3.0, 3.0)]])

    def test_flattens_curves_and_arcs(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "curves.svg"
            path.write_text(
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">'
                '<path d="M0 100 C0 0 100 0 100 100 A20 20 0 0 1 60 100"/>'
                "</svg>"
            )
            actual = read_paths_svg(path, curve_tolerance=0.5, optimize=False)

        self.assertEqual(len(actual), 1)
        self.assertGreater(len(actual[0]), 10)
        self.assertEqual(actual[0][0], (0.0, 0.0))
        self.assertAlmostEqual(actual[0][-1][0], 60.0)
        self.assertAlmostEqual(actual[0][-1][1], 0.0)
        self.assertGreater(max(y for _, y in actual[0]), 70.0)

    def test_applies_group_and_viewport_transforms(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "transformed.svg"
            path.write_text(
                '<svg xmlns="http://www.w3.org/2000/svg" width="200" height="200" '
                'viewBox="0 0 100 100"><g transform="translate(10 20)">'
                '<path d="M0 0 L10 0"/></g></svg>'
            )
            actual = read_paths_svg(path, optimize=False)

        self.assertEqual(actual, [[(20.0, 160.0), (40.0, 160.0)]])

    def test_optimizes_stroke_order_and_direction(self) -> None:
        paths = [
            [(90.0, 0.0), (100.0, 0.0)],
            [(20.0, 0.0), (10.0, 0.0)],
            [(30.0, 0.0), (40.0, 0.0)],
        ]

        self.assertEqual(
            optimize_paths(paths),
            [
                [(10.0, 0.0), (20.0, 0.0)],
                [(30.0, 0.0), (40.0, 0.0)],
                [(90.0, 0.0), (100.0, 0.0)],
            ],
        )

    def test_joins_close_routed_strokes(self) -> None:
        paths = [
            [(0.0, 0.0), (2.0, 0.0)],
            [(3.0, 0.0), (5.0, 0.0)],
            [(10.0, 0.0), (12.0, 0.0)],
        ]

        self.assertEqual(
            join_close_paths(paths, max_gap=1.0),
            [
                [(0.0, 0.0), (2.0, 0.0), (3.0, 0.0), (5.0, 0.0)],
                [(10.0, 0.0), (12.0, 0.0)],
            ],
        )

    def test_rejects_negative_join_distance(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not be negative"):
            join_close_paths([], max_gap=-1.0)


if __name__ == "__main__":
    unittest.main()
