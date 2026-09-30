import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
from PIL import Image

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


if __name__ == "__main__":
    unittest.main()
