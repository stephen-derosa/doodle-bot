import unittest
from argparse import Namespace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import numpy as np
from PIL import Image

from doodle_bot.draw.drawing import run_draw_command
from doodle_bot.draw.tracer import image_to_mask, mask_to_strokes, optimize_stroke_order, strokes_to_moves


class DrawingTests(unittest.TestCase):
    def test_image_is_fitted_to_requested_canvas(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "line.png"
            image = Image.new("L", (10, 10), color=255)
            image.putpixel((5, 5), 0)
            image.save(path)
            mask = image_to_mask(path, 40, 60)
        self.assertEqual(mask.shape, (60, 40))

    def test_traces_a_connected_line_as_one_stroke(self) -> None:
        mask = np.zeros((5, 5), dtype=bool)
        mask[2, 1:4] = True
        strokes = mask_to_strokes(mask)
        self.assertEqual(len(strokes), 1)
        self.assertEqual(set(strokes[0]), {(2, 1), (2, 2), (2, 3)})

    def test_every_move_is_small(self) -> None:
        moves = strokes_to_moves([[(0, 4), (0, 3)]], (5, 5), width=100, max_step=2)
        previous = (0.0, 0.0)
        for move in moves:
            distance = np.hypot(move.x - previous[0], move.y - previous[1])
            self.assertLessEqual(distance, 2.0 + 1e-9)
            previous = (move.x, move.y)
        self.assertTrue(any(move.pen_down for move in moves))

    def test_default_coordinates_match_pixel_canvas(self) -> None:
        moves = strokes_to_moves([[(599, 0), (0, 399)]], (600, 400), max_step=1)
        self.assertEqual((moves[0].x, moves[0].y), (0.0, 0.0))
        self.assertEqual((moves[-1].x, moves[-1].y), (399.0, 599.0))

    def test_strokes_are_ordered_by_nearest_endpoint(self) -> None:
        strokes = [[(0, 8), (0, 9)], [(9, 8), (9, 9)], [(9, 2), (9, 3)]]
        ordered = optimize_stroke_order(strokes, (10, 10))
        self.assertEqual(ordered[0], [(9, 2), (9, 3)])
        self.assertEqual(ordered[1], [(9, 8), (9, 9)])
        self.assertEqual(ordered[2], [(0, 9), (0, 8)])

    def test_draw_persists_all_pipeline_assets_in_timestamped_folder(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            input_path = Path(temporary_directory) / "iteration.png"
            image = Image.new("L", (5, 5), color=255)
            image.putpixel((2, 2), 0)
            image.save(input_path)
            args = Namespace(
                input=input_path,
                width=5.0,
                max_step=1.0,
                pixel_width=5,
                pixel_height=5,
                pen_up_z=1.0,
                pen_down_z=0.0,
                output_root=None,
            )
            with patch("doodle_bot.draw.drawing.interactive_plot"):
                run_draw_command(args)
            run_directories = list(Path(temporary_directory).glob("iteration-*"))
            self.assertEqual(len(run_directories), 1)
            run_directory = run_directories[0]
            self.assertTrue(run_directory.is_dir())
            self.assertTrue((run_directory / "iteration.svg").is_file())
            self.assertEqual(
                (run_directory / "iteration.gcode").read_text().splitlines()[0],
                "G90",
            )
            self.assertTrue((run_directory / "iteration.json").is_file())

    def test_draw_skips_tracing_for_svg_input(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            input_path = root / "sketch.svg"
            source = (
                '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 10">'
                '<polyline points="1,2 3,4"/></svg>'
            )
            input_path.write_text(source)
            args = Namespace(
                input=input_path,
                width=5.0,
                max_step=1.0,
                pixel_width=5,
                pixel_height=5,
                pen_up_z=1.0,
                pen_down_z=0.0,
                output_root=None,
            )

            with (
                patch("doodle_bot.draw.drawing.trace_image_to_svg") as trace,
                patch("doodle_bot.draw.drawing.interactive_plot") as plot,
            ):
                run_draw_command(args)

            trace.assert_not_called()
            plot.assert_called_once()
            self.assertEqual(plot.call_args.args[1:3], (20.0, 10.0))
            run_directory = next(root.glob("sketch-*"))
            self.assertEqual((run_directory / "sketch.svg").read_text(), source)
            self.assertTrue((run_directory / "sketch.gcode").is_file())
            self.assertTrue((run_directory / "sketch.json").is_file())


if __name__ == "__main__":
    unittest.main()
