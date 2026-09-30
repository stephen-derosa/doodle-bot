import numpy as np

from doodle.arm import open_arm
from doodle.config import Calibration, Config, PaperFrame
from doodle.executor import Executor
from doodle.planner import plan
from doodle.shapes import make


def setup(tmp_path):
    cfg, cal = Config(), Calibration()
    cal.paper = PaperFrame(origin=[100.0, 108.0, 0.0], x_axis=[0.0, -1.0, 0.0], y_axis=[1.0, 0.0, 0.0], calibrated=True)
    cal.joints_calibrated = True
    cfg.paper.canvas_v0 = 30.0
    arm = open_arm(cfg, cal, dry_run=True)
    ex = Executor(arm, cfg, cal, realtime=False, log_path=tmp_path / "log.csv", verbose=False)
    return cfg, cal, arm, ex


def test_dry_run_draws_and_parks(tmp_path):
    cfg, cal, arm, ex = setup(tmp_path)
    tr = plan(make("star"), cfg, cal)
    rep = ex.run(tr)
    assert rep.ok, rep.reason
    assert rep.samples_sent >= len(tr.t) - 1
    assert (tmp_path / "log.csv").exists()
    # ends at the last planned sample
    assert np.allclose(arm.read_q(), tr.q[-1], atol=np.radians(0.1))
    ex.park()
    assert np.allclose(np.degrees(arm.read_q()), cfg.park_pose_deg, atol=0.1)


def test_approach_never_dips_below_paper(tmp_path):
    cfg, cal, arm, ex = setup(tmp_path)
    tr = plan(make("square"), cfg, cal)
    zs = []
    orig = arm.command_q

    def spy(q, where=""):
        zs.append(ex.kin.fk(q)[2])
        return orig(q, where)
    arm.command_q = spy
    ex.prepare()
    ex.approach(tr.q[0])
    assert min(zs) > cal.paper.origin[2] - 0.5


def test_abort_on_tracking_error(tmp_path):
    cfg, cal, arm, ex = setup(tmp_path)
    tr = plan(make("circle"), cfg, cal)
    # make the fake servo for the elbow stop following after 1 s
    real_write = arm.bus.sync_write
    state = {"t": 0}

    def stuck(reg, values):
        state["t"] += 1
        if reg == "goal_position" and state["t"] > 60:
            values = dict(values)
            values.pop(cfg.joint("elbow_flex").id)
            arm.bus.mem[cfg.joint("elbow_flex").id]["present_position"] = 100
        return real_write(reg, values)
    arm.bus.sync_write = stuck
    rep = ex.run(tr)
    assert not rep.ok and "elbow_flex" in rep.reason
