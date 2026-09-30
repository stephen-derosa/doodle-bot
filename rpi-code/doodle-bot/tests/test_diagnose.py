import copy
import math

import numpy as np
import pytest

from doodle import calibration as calib
from doodle.arm import RAD_PER_TICK
from doodle.config import Calibration, Config, PaperFrame
from doodle.kinematics import SO101Kinematics

JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")


def _evidence(true_flips, tool=92.87, dist=90.0):
    """Fabricate what `calib pose` + `calib paper` would record on an arm whose
    real pitch-joint signs are `true_flips`, while the calibration believes +1."""
    cfg = Config()
    cfg.tool.along = tool
    P = cfg.paper
    zero = np.array([2080.0, 2048.0, 2048.0, 2048.0, 1003.0])
    d_real = np.array([1.0, *true_flips, 1.0])

    # The real arm's geometry: pick a plausible L pose (ticks near the servo centres)
    # and let the true signs define what it means.
    pose_ticks = np.array([2080.0, 2099.0, 2013.0, 2073.0, 1003.0])
    real = copy.deepcopy(cfg)
    A = calib.geometry_from_pose(d_real[1:4] * (pose_ticks[1:4] - zero[1:4]) * RAD_PER_TICK)
    real.geometry.zero_angle1_deg, real.geometry.zero_angle2_deg, real.geometry.zero_angle3_deg = (
        math.degrees(v) for v in A)
    kin_real = SO101Kinematics.from_config(real)

    # The recorded calibration believes every sign is +1 and solved its geometry accordingly.
    cal = Calibration(zero_ticks=zero.tolist(), direction=[1, 1, 1, 1, 1], joints_calibrated=True)
    cal.pose_ticks = pose_ticks.tolist()
    A_bel = calib.geometry_from_pose((pose_ticks[1:4] - zero[1:4]) * RAD_PER_TICK)
    cfg.geometry.zero_angle1_deg, cfg.geometry.zero_angle2_deg, cfg.geometry.zero_angle3_deg = (
        math.degrees(v) for v in A_bel)

    frame = PaperFrame(origin=[dist, P.width / 2, 0.0], x_axis=[0, -1, 0], y_axis=[1, 0, 0], calibrated=True)
    u0, v0 = P.canvas_origin()
    touches = []
    for label, (u, v) in [("origin", (u0, v0)), ("x", (u0 + P.canvas_width, v0)), ("y", (u0, v0 + P.canvas_height))]:
        r = kin_real.ik(frame.to_world(u, v), tilt_options_deg=(0.0, -8.0, 8.0))
        assert r.ok, label
        ticks = zero + d_real * r.q / RAD_PER_TICK          # what the real encoders read
        touches.append({"label": label, "uv": [u, v], "ticks": ticks.tolist()})
    cal.paper_touches = touches
    return cfg, cal


@pytest.mark.parametrize("true_flips", [(1, 1, -1), (1, -1, 1), (-1, 1, 1), (1, -1, -1)])
def test_diagnose_names_the_flipped_joint(true_flips):
    cfg, cal = _evidence(true_flips)
    hyps = calib.diagnose_signs(cfg, cal)
    best = hyps[0]
    assert tuple(best["flips"][j] for j in JOINTS) == true_flips
    assert best["score"] < 0.05
    assert best["width"] == pytest.approx(cfg.paper.canvas_width, abs=0.5)
    assert best["height"] == pytest.approx(cfg.paper.canvas_height, abs=0.5)
    assert best["angle_deg"] == pytest.approx(90.0, abs=0.5)
    # and the wrong belief scores clearly worse
    believed = next(h for h in hyps if tuple(h["flips"][j] for j in JOINTS) == (1, 1, 1))
    assert believed["score"] > 5 * best["score"] + 0.1


def test_diagnose_says_when_signs_are_already_right():
    cfg, cal = _evidence((1, 1, 1))
    hyps = calib.diagnose_signs(cfg, cal)
    assert tuple(hyps[0]["flips"][j] for j in JOINTS) == (1, 1, 1)
    assert hyps[0]["score"] < 0.05


def test_diagnose_needs_the_evidence():
    cfg, cal = Config(), Calibration()
    with pytest.raises(ValueError, match="calib pose"):
        calib.diagnose_signs(cfg, cal)


def test_box_metrics_of_a_perfect_box():
    m = calib.box_metrics([[0, 0, 0], [101.6, 0, 0], [0, 152.4, 0]])
    assert (m["width"], m["height"], m["angle_deg"], m["z_spread"]) == (101.6, 152.4, 90.0, 0.0)
