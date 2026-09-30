import argparse
import builtins
import math

import numpy as np
import pytest

from doodle import cli
from doodle.cli import _feasible_nudge
from doodle.arm import open_arm
from doodle.config import Calibration, Config


def _run_check_dirs(monkeypatch, tmp_path, answers):
    """Drive `cmd_calib_check_dirs` with scripted answers against a FakeBus arm.

    The command uses the same `dry_run` flag to pick the bus *and* to skip the
    prompt, so exercising the interactive loop means decoupling the two: patch
    `open_arm` to hand back a fake arm while leaving `dry_run` false.
    """
    cfg, cal = Config(), Calibration()
    cal.direction = [1, 1, 1, 1, 1]
    cal.zero_ticks = [2048.0] * 5
    cal.joints_calibrated = True

    arm = open_arm(cfg, cal, dry_run=True)
    # Park at q=0; the default park pose puts elbow_flex within 8 deg of its limit.
    for i, t in zip(cfg.ids, arm.q_to_ticks(np.zeros(5))):
        arm.bus.mem[i]["present_position"] = int(round(t))
        arm.bus.mem[i]["goal_position"] = int(round(t))

    moves = []
    real_move = arm.move_to_q
    monkeypatch.setattr(arm, "move_to_q",
                        lambda q, **kw: (moves.append(np.copy(q)), real_move(q, **kw))[1])
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(cli.time, "sleep", lambda *_: None)

    it = iter(answers)
    asked = []

    def fake_input(prompt=""):
        ans = next(it)
        asked.append(ans)
        return ans

    monkeypatch.setattr(builtins, "input", fake_input)
    args = argparse.Namespace(dry_run=False, calib=tmp_path / "calib.yaml")
    cli.cmd_calib_check_dirs(args, cfg, cal)
    return cal, moves, asked


def test_check_dirs_r_repeats_the_motion(monkeypatch, tmp_path):
    """'r' replays the nudge and re-asks, without recording an answer."""
    # first joint: repeat twice then accept, remaining four accepted outright
    cal, moves, asked = _run_check_dirs(monkeypatch, tmp_path, ["r", "r", "y", "y", "y", "y", "y"])

    assert asked == ["r", "r", "y", "y", "y", "y", "y"]
    # each joint zeroes then nudges, each replay backs off then nudges again,
    # plus a final re-zero when the loop ends
    assert len(moves) == 2 * 5 + 2 * 2 + 1
    # repeating must not be mistaken for "no" -- no direction should flip
    assert cal.direction == [1, 1, 1, 1, 1]


def test_check_dirs_holds_the_nudge_until_yes_or_no(monkeypatch, tmp_path):
    """The joint under test must not return to zero before an explicit y/n.

    Enter and stray keys re-ask without moving, and a replay backs off only
    halfway, so between the nudge and the verdict no move targets q=0.
    """
    cal, moves, asked = _run_check_dirs(monkeypatch, tmp_path, ["", "x", "r", "n", "y", "y", "y", "y"])

    assert asked == ["", "x", "r", "n", "y", "y", "y", "y"]
    # zero, nudge, then the replay: halfway back, nudge again
    home, q1, back, again = moves[:4]
    assert np.allclose(home, 0.0)
    assert q1[0] > 0 and np.allclose(q1[1:], 0.0)
    assert np.allclose(back, q1 / 2)
    assert np.allclose(again, q1)
    # the next zeroing comes only after the "n"
    assert np.allclose(moves[4], 0.0)
    assert len(moves) == 2 * 5 + 2 + 1
    assert cal.direction == [-1, 1, 1, 1, 1]


def test_check_dirs_n_after_r_still_flips(monkeypatch, tmp_path):
    """'r' is transparent to the answer that follows it."""
    cal, moves, asked = _run_check_dirs(monkeypatch, tmp_path, ["r", "n", "y", "y", "y", "y"])

    assert len(moves) == 2 * (5 + 1) + 1
    assert cal.direction == [-1, 1, 1, 1, 1]


def _fake_arm():
    cfg, cal = Config(), Calibration()
    cal.direction = [1, 1, 1, 1, 1]
    cal.zero_ticks = [2048.0] * 5
    return open_arm(cfg, cal, dry_run=True)


def test_feasible_nudge_uses_full_step_when_there_is_room():
    arm = _fake_arm()
    q1, deg = _feasible_nudge(arm, np.zeros(5), 0)
    assert deg == 8.0
    arm.check_q(q1)


def test_feasible_nudge_shrinks_near_a_limit():
    """wrist_flex sits ~88 deg into a +-95 deg range at the L pose."""
    arm = _fake_arm()
    q0 = np.zeros(5)
    q0[3] = math.radians(87.79)
    q1, deg = _feasible_nudge(arm, q0, 3)
    assert deg == 4.0            # +8 would hit 95.8 and be refused
    arm.check_q(q1)


def test_feasible_nudge_reverses_when_pinned_at_max():
    arm = _fake_arm()
    q0 = np.zeros(5)
    q0[3] = arm.max_rad[3]
    q1, deg = _feasible_nudge(arm, q0, 3)
    assert deg == -8.0
    arm.check_q(q1)


def test_feasible_nudge_gives_up_when_no_room_either_way():
    arm = _fake_arm()
    arm.min_rad[1], arm.max_rad[1] = math.radians(-0.5), math.radians(0.5)
    q1, deg = _feasible_nudge(arm, np.zeros(5), 1)
    assert q1 is None and deg == 0.0


def test_dry_run_never_writes_the_real_config_or_calibration(tmp_path, monkeypatch):
    """`calib pose` solves the link geometry, so it writes the config as well.

    A simulated session must leave both real files alone.
    """
    cfg_path, calib_path = tmp_path / "doodle.yaml", tmp_path / "calibration.yaml"
    cli.save_config(Config(), cfg_path)
    cli.save_calibration(Calibration(), calib_path)
    before = (cfg_path.read_text(), calib_path.read_text())

    monkeypatch.setattr(builtins, "input", lambda *_: "")
    cli.main(["--dry-run", "--config", str(cfg_path), "--calib", str(calib_path), "calib", "pose"])

    assert (cfg_path.read_text(), calib_path.read_text()) == before
    assert (tmp_path / "doodle.dry-run.yaml").exists()
    assert (tmp_path / "calibration.dry-run.yaml").exists()


def test_home_ticks_do_not_depend_on_direction():
    """`q_to_ticks(0) == zero_ticks` for either sign.

    This is what lets `check-dirs` re-zero instead of undoing each nudge: the
    home pose is the same physical place even when a direction sign is wrong,
    and flipping one needs no zero_ticks re-derivation.
    """
    cfg, cal = Config(), Calibration()
    cal.zero_ticks = [2048.0, 2100.0, 1900.0, 2048.0, 1000.0]
    cal.joints_calibrated = True
    for direction in ([1, 1, 1, 1, 1], [-1, -1, -1, -1, -1], [-1, 1, -1, 1, -1]):
        cal.direction = list(direction)
        arm = open_arm(cfg, cal, dry_run=True)
        assert np.allclose(arm.q_to_ticks(np.zeros(5)), cal.zero_ticks)


def test_check_dirs_refuses_a_big_swing_to_zero(monkeypatch, tmp_path, capsys):
    """A stale calibration puts q=0 far from the L pose; declining must abort."""
    cfg, cal = Config(), Calibration()
    cal.direction = [1, 1, 1, 1, 1]
    cal.zero_ticks = [2048.0] * 5
    cal.joints_calibrated = True
    arm = open_arm(cfg, cal, dry_run=True)
    # park wrist_flex ~88 deg away from q=0, as the pre-geometry calibration did
    ticks = arm.q_to_ticks(np.zeros(5))
    arm.bus.mem[cfg.ids[3]]["present_position"] = int(ticks[3] + 1000)

    moves = []
    monkeypatch.setattr(arm, "move_to_q", lambda q, **kw: moves.append(np.copy(q)))
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(builtins, "input", lambda *_: "n")

    rc = cli.cmd_calib_check_dirs(
        argparse.Namespace(dry_run=False, calib=tmp_path / "c.yaml"), cfg, cal)

    assert rc == 1
    assert moves == []                       # nothing moved
    assert "WARNING: zeroing moves the joints" in capsys.readouterr().out


def test_calib_corners_returns_to_zero_between_touches(monkeypatch, tmp_path):
    """near-left, near-right, far-right, far-left; q=0 after every touch."""
    from doodle.executor import Executor

    cfg, cal = Config(), Calibration()
    cal.direction = [1] * 5
    cal.zero_ticks = [2048.0] * 5
    cal.joints_calibrated = True
    cal.paper.calibrated = True
    arm = open_arm(cfg, cal, dry_run=True)
    kin = cli.kin_for(cfg, cal)

    targets = []

    def spy(self, q, speed_deg_s=None):
        targets.append(np.copy(q))
        self.arm.command_q(q)

    monkeypatch.setattr(Executor, "approach", spy)
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(cli.time, "sleep", lambda *_: None)

    args = argparse.Namespace(dry_run=False, speed=None, hold=0.0)
    assert cli.cmd_calib_corners(args, cfg, cal) == 0

    q0 = np.zeros(5)
    zu, zd = cfg.motion.pen_up_z, -cfg.motion.pen_press
    expected = [q0]
    for _, u, v in cfg.paper.canvas_corners():
        hover = kin.ik(cal.paper.to_world(u, v, zu), cfg.motion.pen_tilt_options_deg).q
        touch = kin.ik(cal.paper.to_world(u, v, zd), cfg.motion.pen_tilt_options_deg).q
        expected.extend([hover, touch, hover, q0])
    assert len(targets) == len(expected)
    for got, want in zip(targets, expected):
        assert np.allclose(got, want, atol=1e-9)
    assert np.allclose(arm.read_q(), q0, atol=np.radians(0.1))


def test_calib_corners_refuses_uncalibrated_hardware():
    cfg, cal = Config(), Calibration()
    with pytest.raises(SystemExit, match="calib pose"):
        cli.cmd_calib_corners(argparse.Namespace(dry_run=False, speed=None, hold=0.0), cfg, cal)


def _limits_deg(cfg):
    return [(round(math.degrees(j.min_rad), 2), round(math.degrees(j.max_rad), 2)) for j in cfg.joints]


def _tick_window(cfg, cal):
    """Joint travel in ticks -- the physically invariant form of the limits."""
    from doodle.arm import RAD_PER_TICK
    out = []
    for j, z, d in zip(cfg.joints, cal.zero_ticks, cal.direction):
        a, b = z + d * j.min_rad / RAD_PER_TICK, z + d * j.max_rad / RAD_PER_TICK
        out.append((round(min(a, b), 1), round(max(a, b), 1)))
    return out


def test_calib_pose_shifts_limits_so_travel_is_unchanged(monkeypatch, tmp_path):
    """Redefining q must not change which physical positions are allowed.

    Regression: `calib pose` moved q=0 to the servo centre but left the joint
    limits on the old convention, so the arm was refused travel it has
    (wrist_flex = -131 deg) while other poses were wrongly permitted.
    """
    cfg, cal = Config(), Calibration()
    cal.direction = [1, 1, 1, 1, 1]
    cal.zero_ticks = [2115.0, 1800.0, 2300.0, 1500.0, 998.0]
    cal.joints_calibrated = True
    arm = open_arm(cfg, cal, dry_run=True)
    for i, t in zip(cfg.ids, [2099.0, 2013.0, 2073.0, 2100.0, 998.0]):
        arm.bus.mem[i]["present_position"] = int(t)

    before = _tick_window(cfg, cal)
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(builtins, "input", lambda *_: "")
    cli.cmd_calib_pose(argparse.Namespace(dry_run=False, calib=tmp_path / "c.yaml",
                                          config=tmp_path / "d.yaml"), cfg, cal)

    assert cal.zero_ticks[1:4] == [2048.0] * 3          # chain pinned to the servo centre
    assert _limits_deg(cfg) != _limits_deg(Config())    # limits moved with q
    after = _tick_window(cfg, cal)
    for (a0, b0), (a1, b1) in zip(before, after):       # but the travel did not
        assert abs(a0 - a1) < 0.5 and abs(b0 - b1) < 0.5


def test_calib_pose_limit_shift_is_idempotent(monkeypatch, tmp_path):
    """Running it twice on the same pose must not shift the limits twice."""
    cfg, cal = Config(), Calibration()
    cal.direction = [1, 1, 1, 1, 1]
    cal.zero_ticks = [2115.0, 1800.0, 2300.0, 1500.0, 998.0]
    cal.joints_calibrated = True
    arm = open_arm(cfg, cal, dry_run=True)
    for i, t in zip(cfg.ids, [2099.0, 2013.0, 2073.0, 2100.0, 998.0]):
        arm.bus.mem[i]["present_position"] = int(t)
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(builtins, "input", lambda *_: "")
    args = argparse.Namespace(dry_run=False, calib=tmp_path / "c.yaml", config=tmp_path / "d.yaml")

    cli.cmd_calib_pose(args, cfg, cal)
    once = _limits_deg(cfg)
    cli.cmd_calib_pose(args, cfg, cal)
    assert _limits_deg(cfg) == once


def test_measured_travel_makes_limits_immune_to_recalibration():
    """The whole point of measuring travel in ticks.

    Radian limits are relative to q, and q is defined by zero_ticks, so every
    recalibration silently moves them. A measured tick window is the same
    physical travel whatever the zeros become.
    """
    cfg = Config()
    for j in cfg.joints:
        j.min_ticks, j.max_ticks, j.travel_measured = 800, 3200, True

    windows = []
    for zeros in ([2048.0] * 5, [1930.4, 2256.2, 2648.0, 1500.0, 998.0]):
        cal = Calibration(zero_ticks=list(zeros), direction=[1, 1, 1, 1, 1], joints_calibrated=True)
        arm = open_arm(cfg, cal, dry_run=True)
        # map the derived radian limits back to ticks
        windows.append([(round(arm.q_to_ticks(arm.min_rad)[i], 1),
                         round(arm.q_to_ticks(arm.max_rad)[i], 1)) for i in range(5)])
    assert windows[0] == windows[1]
    assert all(w == (800.0, 3200.0) for w in windows[0])


def test_calib_travel_records_all_joints_together(monkeypatch, tmp_path, capsys):
    """One Enter records every joint; the live display lists them all at once."""
    cfg, cal = Config(), Calibration()
    cal.direction = [1] * 5
    cal.zero_ticks = [2048.0] * 5
    arm = open_arm(cfg, cal, dry_run=True)
    start = [2048, 2100, 1900, 2000, 1800]
    extents = [(800, 3300), (900, 3100), (700, 3000), (850, 3200), (100, 4000)]
    for sid, t in zip(cfg.ids, start):
        arm.bus.mem[sid]["present_position"] = t

    reads = {"n": 0}
    orig = arm.read_ticks

    def fake_read():
        reads["n"] += 1
        if reads["n"] == 2:
            for sid, (lo, _) in zip(cfg.ids, extents):
                arm.bus.mem[sid]["present_position"] = lo
        elif reads["n"] >= 3:
            for sid, (_, hi) in zip(cfg.ids, extents):
                arm.bus.mem[sid]["present_position"] = hi
        return orig()

    monkeypatch.setattr(arm, "read_ticks", fake_read)
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(cli, "_pending_enter", lambda: reads["n"] >= 3)
    monkeypatch.setattr(cli.time, "sleep", lambda *_: None)
    monkeypatch.setattr(cli.sys.stdin, "readline", lambda: "\n")

    args = argparse.Namespace(dry_run=False, margin=15, min_span=200, config=tmp_path / "d.yaml")
    assert cli.cmd_calib_travel(args, cfg, cal) == 0

    for j, (lo, hi) in zip(cfg.joints, extents):
        assert j.travel_measured
        assert j.min_ticks == lo + 15
        assert j.max_ticks == hi - 15

    out = capsys.readouterr().out
    assert "sweep all joints to min and max positions, then press Enter" in out
    for j in cfg.joints:
        assert j.name in out


def test_unmeasured_joints_keep_their_urdf_limits():
    cfg = Config()                      # travel_measured defaults to False
    cal = Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    arm = open_arm(cfg, cal, dry_run=True)
    assert np.allclose(arm.min_rad, [j.min_rad for j in cfg.joints])
    assert np.allclose(arm.max_rad, [j.max_rad for j in cfg.joints])


def test_calib_travel_opens_the_bus_once_and_saves_before_displaying(monkeypatch, tmp_path):
    """Regression: a second open_arm() to re-derive the limits tripped our own
    exclusive lock *after* the sweep and *before* save_config, losing the sweep."""
    cfg, cal = Config(), Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    real_open = cli.open_arm
    opens = []

    def counting_open(*a, **k):
        opens.append(1)
        return real_open(*a, **k)

    monkeypatch.setattr(cli, "open_arm", counting_open)
    cfg_path = tmp_path / "d.yaml"
    args = argparse.Namespace(dry_run=True, margin=15, min_span=200, config=cfg_path)
    rc = cli.cmd_calib_travel(args, cfg, cal)

    assert rc == 0 and len(opens) == 1
    assert cfg_path.exists()
    saved = cli.load_config(cfg_path)
    assert all(j.travel_measured for j in saved.joints)
    # dry-run synthesises +-900 ticks around the fake arm's pose, trimmed by the margin
    assert all(j.max_ticks - j.min_ticks == 1800 - 2 * 15 for j in saved.joints)


def test_calib_pose_reports_measured_travel_and_names_the_bad_park_joint(monkeypatch, tmp_path, capsys):
    """The printed limits must be the measured travel, not stale radian fields.

    Regression: with travel measured, `calib pose` printed wrist_flex as
    [-181, 9] deg (the URDF radian fields, shifted) while the arm itself
    enforced the measured [-101, 84], and warned about the park pose without
    saying which joint was out.
    """
    cfg, cal = Config(), Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    for j in cfg.joints:
        j.min_ticks, j.max_ticks, j.travel_measured = 1040, 3000, True   # about [-88.6, 83.7] deg
        j.min_rad, j.max_rad = -3.16, 0.16                               # stale, deliberately wrong
    arm = open_arm(cfg, cal, dry_run=True)
    for i in cfg.ids:
        arm.bus.mem[i]["present_position"] = 2048
    monkeypatch.setattr(cli, "open_arm", lambda *a, **k: arm)
    monkeypatch.setattr(builtins, "input", lambda *_: "")
    cli.cmd_calib_pose(argparse.Namespace(dry_run=False, calib=tmp_path / "c.yaml",
                                          config=tmp_path / "d.yaml"), cfg, cal)
    out = capsys.readouterr().out
    assert "in [-89, 84]" in out and "in [-181" not in out
    # the default park pose folds shoulder_lift to -95 deg, past the -88.6 stop
    assert "shoulder_lift -95.0 not in [-88.6, 83.7]" in out
    assert "joint limits shifted" not in out            # measured windows never shift


def test_ik_plans_against_the_measured_travel():
    cfg, cal = Config(), Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    for j in cfg.joints:
        j.min_ticks, j.max_ticks, j.travel_measured = 1040, 3000, True
        j.min_rad, j.max_rad = -0.01, 0.01                              # stale radian fields
    lo, hi = cli.joint_limits_rad(cfg, cal)
    assert np.allclose(cli.kin_for(cfg, cal).limits, list(zip(lo, hi)))
    assert np.allclose(open_arm(cfg, cal, dry_run=True).min_rad, lo)
