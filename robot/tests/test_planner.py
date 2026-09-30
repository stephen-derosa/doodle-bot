import numpy as np
import pytest

from doodle.config import Calibration, Config, PaperFrame
from doodle.planner import PlanError, build_segments, densify, order_strokes, plan, time_parameterise
from doodle.shapes import GENERATORS, Drawing, Stroke, make


def setup():
    cfg = Config()
    cal = Calibration()
    # letter sheet, portrait, laid in front of the robot: u runs to the robot's right (-y), v away (+x)
    cal.paper = PaperFrame(origin=[100.0, 108.0, 0.0], x_axis=[0.0, -1.0, 0.0], y_axis=[1.0, 0.0, 0.0], calibrated=True)
    cfg.paper.canvas_v0 = 30.0
    return cfg, cal


def test_time_parameterise_two_point_move_has_duration():
    t = time_parameterise(np.array([[0.0, 0.0, 8.0], [0.0, 0.0, -1.5]]), 15.0, 200.0, 8.0)
    assert t[-1] > 0.5   # 9.5 mm at <=15 mm/s with 200 mm/s^2 ramps


def test_time_parameterise_monotone_and_speed_limited():
    pts = densify(np.array([[0, 0], [100, 0], [100, 100]], float), 1.0)
    t = time_parameterise(pts, 30.0, 200.0, 8.0)
    assert np.all(np.diff(t) > 0)
    v = np.linalg.norm(np.diff(pts, axis=0), axis=1) / np.diff(t)
    assert v.max() <= 30.0 + 1e-6
    # the 90 degree corner must be slow: segment average of (v_prev + v_corner)/2 with v_corner = 8
    i = 100
    assert v[i - 1] < 16.0 and v[i] < 16.0


def test_order_strokes_prefers_nearest_and_reverses():
    a = Stroke(np.array([[0.0, 0.0], [10.0, 0.0]]))
    b = Stroke(np.array([[50.0, 0.0], [11.0, 0.0]]))  # closer when reversed
    out = order_strokes([b, a], start=(0.0, 0.0))
    assert np.allclose(out[0].points[0], (0, 0))
    assert np.allclose(out[1].points[0], (11, 0))


def test_segments_start_at_canvas_origin_with_pen_up():
    cfg, _ = setup()
    drawing = Drawing("line", [Stroke(np.array([[10.0, 10.0], [20.0, 10.0]]))])

    segments = build_segments(drawing, cfg)

    assert not segments[0].pen_down
    assert np.allclose(segments[0].pts[0], (0.0, 0.0, cfg.motion.pen_up_z))


@pytest.mark.parametrize("name", sorted(GENERATORS))
def test_plan_all_shapes(name):
    cfg, cal = setup()
    tr = plan(make(name), cfg, cal)
    assert tr.stats["time_dilation"] == 1.0
    assert np.all(np.diff(tr.t) > 0)
    # every pen-down sample sits on the drawing plane, pen-up samples above it
    z = tr.xyz[:, 2]
    assert np.allclose(z[tr.pen_down], -cfg.motion.pen_press, atol=1e-6)
    assert z[~tr.pen_down].max() <= cfg.motion.pen_up_z + 1e-6
    # joint speed bounded
    dq = np.degrees(np.abs(np.diff(tr.q, axis=0)) / np.diff(tr.t)[:, None])
    assert dq.max() <= cfg.motion.max_joint_speed_deg_s + 1e-6
    # tip speed while drawing never exceeds draw_speed
    v = np.linalg.norm(np.diff(tr.xyz, axis=0), axis=1) / np.diff(tr.t)
    down = tr.pen_down[:-1] & tr.pen_down[1:]
    assert v[down].max() <= cfg.motion.draw_speed * 1.05


def test_plan_unreachable_raises():
    cfg, cal = setup()
    cal.paper.origin = [400.0, 108.0, 0.0]
    with pytest.raises(PlanError):
        plan(make("square"), cfg, cal)


def test_trajectory_interpolation():
    cfg, cal = setup()
    tr = plan(make("square"), cfg, cal)
    mid = (tr.t[10] + tr.t[11]) / 2
    assert np.allclose(tr.q_at(mid), (tr.q[10] + tr.q[11]) / 2)
    assert np.allclose(tr.q_at(-1), tr.q[0])
    assert np.allclose(tr.q_at(1e9), tr.q[-1])
