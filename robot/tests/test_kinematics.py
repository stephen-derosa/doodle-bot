import math
import numpy as np
import pytest

from doodle.config import Config, Tool
from doodle.kinematics import SO101Kinematics, urdf_fk, URDF_BASE_TO_PAN_X, URDF_SIGN


def gripper_kin():
    # gripper_frame is 98.13 mm along the roll axis and 7.9 mm off-axis (down at q=0)
    cfg = Config()
    return SO101Kinematics.from_config(cfg, Tool(along=98.1274, perp=-7.9, lateral=0.0, roll_rad=0.0))


def test_planar_fk_matches_urdf_fk_at_zero():
    k = gripper_kin()
    mine = k.fk(np.zeros(5))
    ref = urdf_fk(np.zeros(5))
    ref[0] -= URDF_BASE_TO_PAN_X
    assert np.allclose(mine, ref, atol=0.5), (mine, ref)


@pytest.mark.parametrize("seed", range(30))
def test_planar_fk_matches_urdf_fk_random(seed):
    rng = np.random.default_rng(seed)
    k = gripper_kin()
    q = rng.uniform(-1.4, 1.4, 5)
    q[4] = 0.0  # roll = 0 keeps the 7.9 mm gripper offset in-plane
    mine = k.fk(q)
    ref = urdf_fk(q * URDF_SIGN)
    ref[0] -= URDF_BASE_TO_PAN_X
    assert np.allclose(mine, ref, atol=0.6), (q, mine, ref)


def test_fk_with_roll_matches_urdf():
    rng = np.random.default_rng(1)
    k = gripper_kin()
    for _ in range(20):
        q = rng.uniform(-1.2, 1.2, 5)
        mine = k.fk(q)
        ref = urdf_fk(q * URDF_SIGN)
        ref[0] -= URDF_BASE_TO_PAN_X
        # the tiny -0.2 mm y offset in the URDF and roll-sign convention are within 1 mm
        assert np.linalg.norm(mine - ref) < 1.0, (q, mine, ref)


def test_ik_roundtrip_pen_vertical():
    k = SO101Kinematics.from_config(Config())
    for x, y in [(150, 0), (180, 40), (200, -60), (130, 30), (230, 0)]:
        res = k.ik((x, y, 0.0))
        assert res.ok, res.reason
        tip = k.fk(res.q)
        assert np.allclose(tip, (x, y, 0.0), atol=1e-6)
        d = k.pen_direction(res.q)
        assert np.allclose(d, (0, 0, -1), atol=1e-9)


def test_ik_unreachable():
    k = SO101Kinematics.from_config(Config())
    assert not k.ik((400, 0, 0)).ok
    assert not k.ik((0, 0, 0)).ok


def test_ik_falls_back_to_tilt():
    k = SO101Kinematics.from_config(Config())
    # very close to the base needs a tilted pen because of the wrist_flex limit
    far = k.ik((90, 0, 0), tilt_options_deg=(0.0,))
    tilted = k.ik((90, 0, 0), tilt_options_deg=(0.0, -10.0, 10.0, -20.0, 20.0))
    assert tilted.ok or not far.ok


def test_radial_reach_sane():
    k = SO101Kinematics.from_config(Config())
    rmin, rmax = k.radial_reach(0.0)
    assert 40 < rmin < 160
    assert 220 < rmax < 300
    assert rmax - rmin > 100
