"""The ``doodle`` command. Run ``doodle -h`` for the list of subcommands."""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

from . import calibration as calib
from .arm import RAD_PER_TICK, Arm, SafetyError, joint_limits_rad, open_arm
from .config import (Calibration, Config, ROOT, load_calibration, load_config, save_calibration,
                     save_config)
from .kinematics import SO101Kinematics
from .planner import PlanError, plan
from .preview import render_drawing, render_joints, render_layout, render_toolpath
from .servo import BAUD_INDEX, TICKS_PER_REV, BusBusy, FeetechBus, ServoError, ServoTimeout
from .shapes import GENERATORS, Drawing, make

ASSETS = ROOT / "assets"
LOGS = ROOT / "logs"


def fmt_q(q) -> str:
    return " ".join(f"{math.degrees(v):7.2f}" for v in q)


def load_drawing(spec: str) -> Drawing:
    if spec in GENERATORS:
        return make(spec)
    p = Path(spec)
    if p.suffix == ".json" and p.exists():
        return Drawing.load(p)
    p2 = ASSETS / f"{spec}.json"
    if p2.exists():
        return Drawing.load(p2)
    raise SystemExit(f"unknown drawing {spec!r}: use one of {sorted(GENERATORS)} or a .json path")


def kin_for(cfg: Config, cal: Calibration) -> SO101Kinematics:
    k = SO101Kinematics.from_config(cfg, cal=cal)
    if cal.tool_along is not None:
        k.tool.along = cal.tool_along
    return k


# --- bus level ----------------------------------------------------------------------------
def cmd_scan(a, cfg, cal):
    bauds = [cfg.baud] if not a.all_bauds else list(BAUD_INDEX)
    for baud in bauds:
        with FeetechBus(cfg.port, baud, timeout=0.02) as bus:
            found = bus.scan(range(0, a.max_id + 1), retries=2)
        if found:
            print(f"{baud} baud: servos {found}")
            names = {j.id: j.name for j in cfg.joints}
            for i in found:
                print(f"  ID {i:3d}  {names.get(i, '(unassigned / not a doodle joint)')}")
            missing = [j for j in cfg.joints if j.id not in found]
            if missing:
                print("  missing:", ", ".join(f"{j.name} (ID {j.id})" for j in missing))
            return 0
    print("no servos answered" + ("" if a.all_bauds else f" at {cfg.baud} baud (try --all-bauds)"))
    return 1


def cmd_assign_ids(a, cfg, cal):
    print("Servo ID assignment. Feetech servos ship as ID 1, so this must be done with\n"
          "exactly ONE servo connected to the bus at a time.\n")
    todo = [j for j in cfg.joints if a.joint in (None, j.name)]
    with FeetechBus(cfg.port, cfg.baud, timeout=0.03) as bus:
        for j in todo:
            input(f"--> Connect ONLY the '{j.name}' servo (target ID {j.id}) and press Enter... ")
            found = bus.scan(range(0, 21), retries=2)
            if len(found) != 1:
                print(f"    expected exactly one servo, found {found}. Skipping {j.name}.")
                continue
            old = found[0]
            if old == j.id:
                print(f"    already ID {j.id}")
                continue
            bus.set_id(old, j.id)
            print(f"    ID {old} -> {j.id}  ({j.name})  model={bus.read(j.id, 'model')}")
    print("\nReconnect all servos, then run `doodle scan` and `doodle status`.")
    return 0


def _status_snapshot(arm, cfg, cal, alive) -> str:
    lines = ["servo   id alive   ticks    deg(q)   volt  temp  load  torque"]
    ticks = arm.read_ticks() if all(alive.values()) else None
    q = arm.ticks_to_q(ticks) if ticks is not None else None
    st = arm.read_status() if ticks is not None else {}
    te = arm.bus.sync_read("torque_enable", arm.ids) if ticks is not None else {}
    for k, (n, i) in enumerate(zip(arm.names, arm.ids)):
        if ticks is None:
            lines.append(f"{n:14s} {i:2d} {'yes' if alive[n] else 'NO ':5s}")
            continue
        s = st[n]
        lines.append(f"{n:14s} {i:2d} {'yes':5s} {ticks[k]:6.0f} {math.degrees(q[k]):9.2f} {s['voltage']:6.1f} {s['temperature']:5.0f} {s['load']:5.0f}  {'on' if te[i] else 'off'}")
    if ticks is not None:
        kin = kin_for(cfg, cal)
        tip = kin.fk(q)
        lines.append(f"\npen tip (world, mm): x={tip[0]:.1f} y={tip[1]:.1f} z={tip[2]:.1f}   pen dir z={kin.pen_direction(q)[2]:+.2f}")
        if not cal.joints_calibrated:
            lines.append("NOTE: joints are not calibrated; angles above are meaningless until `doodle calib pose`")
    return "\n".join(lines)


def cmd_status(a, cfg, cal):
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        alive = arm.ping_all()
        if not a.watch:
            print(_status_snapshot(arm, cfg, cal, alive))
            return 0
        period = 1.0 / a.rate
        try:
            while True:
                t0 = time.monotonic()
                out = _status_snapshot(arm, cfg, cal, alive)
                print("\x1b[H\x1b[2J" + out, flush=True)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            pass
    finally:
        arm.bus.close()
    return 0


def cmd_torque(a, cfg, cal):
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(a.state == "on")
        print(f"torque {a.state}")
    finally:
        arm.bus.close()
    return 0


def cmd_configure(a, cfg, cal):
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(False)
        arm.configure()
        s = cfg.servo
        print(f"wrote P={s.p_gain} I={s.i_gain} D={s.d_gain} accel={s.acceleration} "
              f"goal_velocity={s.goal_velocity} torque_limit={s.torque_limit} to {arm.ids}")
    finally:
        arm.bus.close()
    return 0


def cmd_info(a, cfg, cal):
    with FeetechBus(cfg.port, cfg.baud) as bus:
        for k, v in bus.info(a.id).items():
            print(f"  {k:22s} {v}")
    return 0


# --- calibration -------------------------------------------------------------------------------
def cmd_calib_pose(a, cfg, cal):
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(False)
        print(calib.REFERENCE_POSE_HELP)
        if not a.dry_run:
            input()
        ticks = arm.read_ticks()
        d = np.asarray(cal.direction, float)
        zero = calib.zero_ticks_from_pose(ticks, calib.reference_pose_q(cfg), d)

        # Pin the three in-plane joints to their servo's own encoder centre and
        # let the measured pose determine the link geometry, rather than
        # trusting the nominal design angles and bending the zeros to fit. This
        # is what keeps each joint's travel centred on the servo centre: with
        # the design defaults the L pose put wrist_flex at 87.8 deg of a +-95
        # deg range, leaving 7 deg of headroom.
        centre = TICKS_PER_REV / 2
        zero[1:4] = centre
        q_chain = d[1:4] * (np.asarray(ticks, float)[1:4] - centre) * RAD_PER_TICK
        q_ref_old = calib.reference_pose_q(cfg)
        old_A = [cfg.geometry.zero_angle1_deg, cfg.geometry.zero_angle2_deg, cfg.geometry.zero_angle3_deg]
        new_A = [float(math.degrees(v)) for v in calib.geometry_from_pose(q_chain, cfg.tool.pen_angle_deg)]
        (cfg.geometry.zero_angle1_deg,
         cfg.geometry.zero_angle2_deg,
         cfg.geometry.zero_angle3_deg) = new_A

        cal.joints_calibrated = True

        # q is defined by zero_ticks alone, so moving a zero redefines every
        # angle expressed in q -- the joint limits included. Shift them by the
        # same amount or the arm is fenced off from travel it physically has,
        # and allowed into travel it does not. Deriving the shift from the zero
        # change (rather than from the reference pose) keeps it correct even
        # when the geometry is unchanged but the zeros are reset.
        shift = -np.asarray(cal.direction, float) * (zero - np.asarray(cal.zero_ticks, float)) * RAD_PER_TICK
        old_lim = [(math.degrees(j.min_rad), math.degrees(j.max_rad)) for j in cfg.joints]
        for j, d in zip(cfg.joints, shift):
            j.min_rad += float(d)
            j.max_rad += float(d)
        cal.zero_ticks = [float(v) for v in zero]
        cal.pose_ticks = [float(v) for v in ticks]          # evidence for `calib diagnose`

        print("ticks at L pose:", ticks.astype(int).tolist())
        print("zero ticks     :", [round(v) for v in cal.zero_ticks])
        print("link angles at q=0 (deg):")
        for i, (o, n) in enumerate(zip(old_A, new_A), start=1):
            print(f"  zero_angle{i}_deg  {o:8.2f} -> {n:8.2f}")

        # Only radian limits move with q. A measured tick window is the same
        # physical travel whatever the zeros are, so listing it as "shifted"
        # would only suggest a change that did not happen.
        moved = [(j, lim, d) for j, lim, d in zip(cfg.joints, old_lim, np.degrees(shift))
                 if abs(d) > 0.05 and not j.travel_measured]
        if moved:
            print("\njoint limits shifted with the new q (deg):")
            for j, (lo, hi), d in moved:
                print(f"  {j.name:14s} [{lo:7.1f},{hi:7.1f}] -> "
                      f"[{math.degrees(j.min_rad):7.1f},{math.degrees(j.max_rad):7.1f}]  ({d:+.2f})")

        lo_all, hi_all = (np.degrees(v) for v in joint_limits_rad(cfg, cal))
        print("\nL pose now sits at:")
        q_ref = np.degrees(calib.reference_pose_q(cfg))
        for j, q, lo, hi in zip(cfg.joints, q_ref, lo_all, hi_all):
            src = "measured" if j.travel_measured else "URDF default"
            print(f"  {j.name:14s} {q:7.2f} deg in [{lo:.0f}, {hi:.0f}]"
                  f"  (headroom {min(q - lo, hi - q):.1f}, {src})")

        pc = save_config(cfg, a.config)
        p = save_calibration(cal, a.calib)
        print(f"\nsaved {p}\nsaved {pc}")
        # New zeros redefine q, so every angle already stored in the config now
        # points somewhere else physically. park_pose_deg is the one that bites.
        shift_deg = np.degrees(shift)
        if np.abs(shift_deg).max() > 0.5:
            park_equiv = np.asarray(cfg.park_pose_deg, float) + shift_deg
            print("\nNOTE: q was redefined, so park_pose_deg now means a different pose."
                  f"\n  per-joint shift : {np.round(shift_deg, 2).tolist()}"
                  f"\n  same pose as before: {np.round(park_equiv, 2).tolist()}"
                  f"\n  currently in config: {cfg.park_pose_deg}")
        park = np.asarray(cfg.park_pose_deg, float)
        outside = [f"{j.name} {v:.1f} not in [{lo:.1f}, {hi:.1f}]"
                   for j, v, lo, hi in zip(cfg.joints, park, lo_all, hi_all) if not lo <= v <= hi]
        if outside:
            print("\nWARNING: park_pose_deg in the config is outside the joint travel ("
                  + "; ".join(outside) + "). The L pose is not affected. "
                  "Update park_pose_deg before running `doodle draw`.")
        print("Next: `doodle calib check-dirs`, then `doodle calib paper`.")
    finally:
        arm.bus.close()
    return 0


def _feasible_nudge(arm, q0, k, step_deg=15.0, min_deg=2.0):
    """Largest safe nudge of joint `k` away from q0, positive for preference.

    A joint can sit close enough to an end stop that the nominal step does not
    fit: at the L pose `wrist_flex` is ~88 deg into a +-95 deg range, because
    pointing the pen straight down folds the wrist almost fully. Falling back to
    a smaller step, then to the opposite direction, keeps the direction check
    possible everywhere instead of refusing on safety.

    Returns (q1, signed_degrees) or (None, 0.0) if nothing fits.
    """
    for sign in (1.0, -1.0):
        for deg in (step_deg, step_deg / 2, min_deg):
            q1 = np.copy(q0)
            q1[k] += sign * math.radians(deg)
            try:
                arm.check_q(q1, "(direction check)")
            except SafetyError:
                continue
            return q1, sign * deg
    return None, 0.0


def _travel_status_line(name, now, lo, hi, name_width):
    span = hi - lo
    return (f"{name:<{name_width}}:  now {now:6.0f}   min {lo:6.0f}   max {hi:6.0f}   "
            f"span {span:5.0f} ticks ({span * 360.0 / TICKS_PER_REV:5.1f} deg)")


def _print_travel_status(names, now, lo, hi, *, replace):
    """Live list of every joint's sweep. `replace` rewinds the previous block."""
    w = max(len(n) for n in names)
    block = "\n".join(_travel_status_line(n, t, a, b, w) for n, t, a, b in zip(names, now, lo, hi))
    if replace:
        sys.stdout.write(f"\x1b[{len(names)}A\r")
    sys.stdout.write(block + "\n")
    sys.stdout.flush()


def cmd_calib_travel(a, cfg, cal):
    """Sweep every joint by hand to its stops and record the tick windows.

    Measuring the travel beats trusting the URDF: it captures this build (the
    pen holder, and wherever each servo horn happens to be clamped) rather than
    the nominal arm, and it is recorded in ticks, so later calibration steps
    that move the zeros cannot silently invalidate it.
    """
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(False)
        print("Torque is off. Move every joint slowly to each end of its travel,\n"
              f"then press Enter. Stop at the mechanical stop -- {a.margin} ticks\n"
              "are trimmed off each end automatically.\n")
        now = np.asarray(arm.read_ticks(), dtype=float)
        lo = now.copy()
        hi = now.copy()
        print("sweep all joints to min and max positions, then press Enter")
        if a.dry_run:
            lo = lo - 900
            hi = hi + 900
        _print_travel_status(arm.names, now, lo, hi, replace=False)
        if not a.dry_run:
            while not _pending_enter():
                now = np.asarray(arm.read_ticks(), dtype=float)
                lo = np.minimum(lo, now)
                hi = np.maximum(hi, now)
                _print_travel_status(arm.names, now, lo, hi, replace=True)
                time.sleep(0.05)
            sys.stdin.readline()
        print()
        for k, j in enumerate(cfg.joints):
            span = float(hi[k] - lo[k])
            if span < a.min_span:
                print(f"  {j.name}: only {span:.0f} ticks of travel seen; "
                      f"keeping [{j.min_ticks}, {j.max_ticks}]")
                continue
            j.min_ticks = int(round(float(lo[k]) + a.margin))
            j.max_ticks = int(round(float(hi[k]) - a.margin))
            j.travel_measured = True
            print(f"  {j.name}: travel [{j.min_ticks}, {j.max_ticks}] ticks "
                  f"({(j.max_ticks - j.min_ticks) * 360.0 / TICKS_PER_REV:.1f} deg)")

        # Save first: a sweep is minutes of hand work and must not be lost to
        # anything that happens while merely displaying it.
        p = save_config(cfg, a.config)
        print(f"\nsaved {p}")

        print("\nmeasured travel, as joint angles under the current zeros:")
        # Re-derive the radian limits on the bus we already hold. Opening the
        # port again in the same process trips our own exclusive lock.
        view = Arm(arm.bus, cfg, cal)
        for j, lo_r, hi_r in zip(cfg.joints, view.min_rad, view.max_rad):
            flag = "measured" if j.travel_measured else "URDF default"
            print(f"  {j.name:14s} [{math.degrees(lo_r):7.1f},{math.degrees(hi_r):7.1f}] deg  "
                  f"ticks [{j.min_ticks:4d},{j.max_ticks:4d}]  ({flag})")
    finally:
        arm.bus.close()
    return 0


def _pending_enter() -> bool:
    """True once a line is waiting on stdin, so the sweep can poll while idle."""
    import select
    return bool(select.select([sys.stdin], [], [], 0)[0])


def cmd_calib_check_dirs(a, cfg, cal):
    """Nudge each joint and ask whether it moved the way the model expects."""
    expect = {
        "shoulder_pan": "the arm swings LEFT (counter-clockwise seen from above)",
        "shoulder_lift": "the upper arm tilts FORWARD / down toward the paper",
        "elbow_flex": "the forearm folds DOWN toward the table (elbow closes)",
        "wrist_flex": "the pen tip swings DOWN / back under the forearm",
        "wrist_roll": "the pen holder rotates counter-clockwise seen from the pen tip",
    }
    if not cal.joints_calibrated and not a.dry_run:
        raise SystemExit("run `doodle calib pose` first")
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(True)
        # Re-zero before each joint instead of undoing the nudge afterwards.
        # `q_to_ticks(0)` is `zero` whatever the direction sign, so home is the
        # same physical pose even when a sign is wrong -- which is exactly what
        # is under test here -- and a flip leaves it untouched. That also means
        # no zero_ticks need re-deriving when a direction is inverted.
        home = np.zeros(len(arm.names))
        arm.check_q(home, "(zero pose)")
        # A calibration written before the geometry solve puts q=0 a long way
        # from the L pose, so zeroing would fling a joint across its range.
        swing = np.degrees(np.abs(arm.read_q() - home))
        if swing.max() > 20.0 and not a.dry_run:
            print(f"WARNING: zeroing moves the joints by {np.round(swing, 1).tolist()} deg.")
            print("  q=0 should sit close to the L pose. If it does not, this calibration\n"
                  "  predates `doodle calib pose` solving the link geometry -- re-run it.")
            if input("  continue anyway? [y/N] ").strip().lower() != "y":
                return 1
        changed = False
        for k, n in enumerate(arm.names):
            q1, deg = _feasible_nudge(arm, home, k)
            if q1 is None:
                print(f"\n{n}: SKIPPED, no safe nudge from zero "
                      f"within [{math.degrees(arm.min_rad[k]):.0f}, {math.degrees(arm.max_rad[k]):.0f}]; "
                      f"direction left at {cal.direction[k]:+d}")
                continue
            # Nudging the other way inverts the motion the model predicts.
            described = expect[n] if deg > 0 else f"the OPPOSITE of: {expect[n]}"
            print(f"\n{n}: zeroing, then moving {deg:+.0f} deg. Expected: {described}")
            arm.move_to_q(home, speed_deg_s=15)
            time.sleep(0.4)
            arm.move_to_q(q1, speed_deg_s=15)
            # Hold the nudge until the answer is an explicit yes or no, so the
            # joint can be inspected for as long as it takes. Enter is not an
            # answer, and a repeat backs off only halfway before replaying the
            # motion, so the joint never returns to zero before the verdict.
            ans = "y" if a.dry_run else ""
            while ans not in ("y", "n"):
                ans = input("  Did it move as expected? [y/n/r=repeat] ").strip().lower()[:1]
                if ans == "r":
                    print("  repeating...")
                    arm.move_to_q(home + (q1 - home) / 2, speed_deg_s=15)
                    time.sleep(0.4)
                    arm.move_to_q(q1, speed_deg_s=15)
                elif ans not in ("y", "n"):
                    print("  answer y or n (r repeats the motion)")
            if ans == "n":
                cal.direction[k] = -cal.direction[k]
                arm.dir[k] = cal.direction[k]
                changed = True
                print(f"  flipped direction of {n} -> {cal.direction[k]}")
        arm.move_to_q(home, speed_deg_s=15)
        p = save_calibration(cal, a.calib)
        print(f"\nsaved {p}" + (" (directions changed; re-run `doodle calib pose` to be safe)" if changed else " (no changes)"))
    finally:
        arm.bus.close()
    return 0


def _touch(arm: Arm, label: str, dry_run: bool, fake_q=None) -> np.ndarray:
    if dry_run:
        return arm.q_to_ticks(fake_q)
    input(f"  touch the pen tip to '{label}' and press Enter... ")
    return arm.read_ticks()


def cmd_calib_paper(a, cfg, cal):
    if not cal.joints_calibrated and not a.dry_run:
        raise SystemExit("run `doodle calib pose` first")
    kin = kin_for(cfg, cal)
    P = cfg.paper
    # Measure the drawn canvas box, not the sheet. Every corner of it is marked
    # in sharpie and within reach, where the sheet's far corners are ~350 mm out
    # and unreachable -- which is why this used to settle for a ruler mark.
    u0, v0 = P.canvas_origin()
    cw, ch = P.canvas_width, P.canvas_height
    canvas = (u0, v0, cw, ch)
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(False)
        print("Torque is off. Guide the arm by hand so the pen tip rests on the corner\n"
              "marked X in each diagram, keeping the pen roughly vertical. All three are\n"
              "corners of the box you drew on the page.")
        specs = [("origin", (u0, v0)), ("x", (u0 + cw, v0)), ("y", (u0, v0 + ch))]
        fakes = None
        if a.dry_run:  # synthesise the default sheet placement
            f0 = cal.paper
            fakes = {lbl: kin.ik(f0.to_world(*uv)).q for lbl, uv in specs}
        touches = []
        for lbl, uv in specs:
            print("\n" + calib.paper_diagram(lbl, P.width, P.height, canvas))
            touches.append(calib.TouchPoint(lbl, uv, list(_touch(arm, lbl, a.dry_run, fakes and fakes[lbl]))))
        frame, rep = calib.paper_frame_from_corners(kin, arm.ticks_to_q, touches, cw, ch,
                                                    origin_uv=(u0, v0))
        print(f"\nmeasured box width  {rep['x_len_measured']:.1f} mm (nominal {cw}, err {rep['x_err']:+.1f})")
        print(f"measured box height {rep['y_len_measured']:.1f} mm (nominal {ch}, err {rep['y_err']:+.1f})")
        print(f"corner angle {rep['corner_angle_deg']:.1f} deg, z spread {rep['z_spread']:.1f} mm, z mean {rep['z_mean']:.1f}")
        if abs(rep["x_err"]) > 5 or abs(rep["y_err"]) > 5:
            print("WARNING: >5 mm scale error -> kinematic model is off; consider `doodle calib refine`")
        cal.paper = frame
        cal.paper_touches = [{"label": t.label, "uv": [float(v) for v in t.uv], "ticks": [float(v) for v in t.ticks]}
                             for t in touches]              # evidence for `calib diagnose`
        if abs(rep["x_err"]) > 5 or abs(rep["y_err"]) > 5:
            print("Run `doodle calib diagnose` to test whether a joint direction sign explains this.")
        p = save_calibration(cal, a.calib)
        print(f"saved {p}; run `doodle layout` to see the canvas placement")
    finally:
        arm.bus.close()
    return 0


def cmd_calib_diagnose(a, cfg, cal):
    """Explain a bad `calib paper` result from the evidence already on disk."""
    P = cfg.paper
    try:
        hyps = calib.diagnose_signs(cfg, cal, tool_candidates=a.tool)
    except ValueError as e:
        raise SystemExit(str(e))
    names = ("shoulder_lift", "elbow_flex", "wrist_flex")
    believed = dict(zip(names, cal.direction[1:4]))

    def show(h, tag=""):
        flips = ", ".join(f"{k}={'+' if v > 0 else '-'}" for k, v in h["flips"].items())
        print(f"  {tag:8s} {h['width']:6.1f} x {h['height']:6.1f} mm  angle {h['angle_deg']:5.1f}  "
              f"z {h['z_mean']:7.1f} (spread {h['z_spread']:4.1f})  tool {h['tool']:6.1f}  score {h['score']:.3f}"
              f"   [{flips}]")

    print(f"drawn box: {P.canvas_width:.1f} x {P.canvas_height:.1f} mm at 90 deg, flat on the paper\n")
    cur = [h for h in hyps if h["flips"] == believed]
    if cur:
        print("the current signs read the three touches as:")
        show(min(cur, key=lambda h: h["score"]), "current")
    print("\nbest hypotheses (score 0 = perfect box):")
    for h in hyps[:5]:
        show(h)
    best = hyps[0]
    changed = [k for k, v in best["flips"].items() if v != believed[k]]
    print()
    if best["score"] < 0.15 and changed:
        print(f"=> flipping {', '.join(changed)} turns the touches into a {best['width']:.1f} x {best['height']:.1f} mm "
              f"box: that sign is almost certainly wrong. Fix it with `doodle calib check-dirs` (answer 'n' for that "
              f"joint), then re-run `calib pose` and `calib paper`.")
    elif best["score"] < 0.15:
        print("=> the current signs already give a good box; the earlier error came from something since changed.")
    else:
        print("=> no sign combination makes a good box, so this is not a direction sign. Suspect the L pose "
              "(re-do `calib pose` with a square and a plumb line), the tool length, or the link lengths.")
    return 0


def cmd_calib_refine(a, cfg, cal):
    kin = kin_for(cfg, cal)
    P = cfg.paper
    u0, v0 = P.canvas_origin()
    pts = calib.grid_points(P.canvas_width, P.canvas_height, a.grid, inset=a.inset, u0=u0, v0=v0)
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        arm.torque(False)
        canvas = (u0, v0, P.canvas_width, P.canvas_height)
        all_uv = [uv for _, uv in pts]
        print(f"Torque is off. Mark a {a.grid}x{a.grid} grid on the sheet first, evenly spaced\n"
              f"{a.inset} mm inside the box you drew, then guide the pen tip to the mark shown\n"
              "as X in each diagram.\n"
              "Vary how you hold the arm: lean the pen forward on some points and back on\n"
              "others, otherwise the wrist offset and the tool length cannot be separated.")
        touches = []
        for i, (lbl, uv) in enumerate(pts, start=1):
            fake = kin.ik(cal.paper.to_world(*uv), (0.0, -10.0, 10.0)[len(touches) % 3:] + (0.0,)).q if a.dry_run else None
            print("\n" + calib.grid_diagram(uv, all_uv, P.width, P.height, canvas,
                                            label=lbl, index=i, total=len(pts)))
            touches.append(calib.TouchPoint(f"{lbl} u={uv[0]:.0f} v={uv[1]:.0f}", uv,
                                            list(_touch(arm, lbl, a.dry_run, fake))))
        res = calib.refine_from_grid(cfg, cal, touches, fit_tool=not a.no_tool, verbose=True)
        print(f"\nrms error before {res.rms_before:.2f} mm -> after {res.rms_after:.2f} mm")
        print("zero tick change:", np.round(res.zero_ticks - np.asarray(cal.zero_ticks), 1).tolist())
        print(f"tool length: {cal.tool_along or cfg.tool.along:.1f} -> {res.tool_along:.1f} mm")
        if res.rms_after > 3.0:
            print("WARNING: residual still >3 mm; check the touch points and the direction signs")

        # A fit can be numerically fine and still physically impossible. Saving
        # one poisons every later run: IK then demands joint angles the arm
        # cannot reach, and the next refine starts from the bad value.
        problems = []
        lo, hi = (1 - calib.TOOL_BOUND_FRAC) * cfg.tool.along, (1 + calib.TOOL_BOUND_FRAC) * cfg.tool.along
        if not lo <= res.tool_along <= hi:
            problems.append(f"tool length {res.tool_along:.1f} mm outside [{lo:.1f}, {hi:.1f}] "
                            f"(measured pen is {cfg.tool.along:.1f} mm)")
        bad = [(n, z) for n, z in zip(cfg.joint_names, res.zero_ticks) if not 0 <= z < TICKS_PER_REV]
        for n, z in bad:
            problems.append(f"{n} zero {z:.0f} outside the encoder range [0, {TICKS_PER_REV})")
        if res.rms_after > res.rms_before + 1e-9:
            problems.append(f"fit made things worse ({res.rms_before:.2f} -> {res.rms_after:.2f} mm)")
        if min(abs(res.tool_along - lo), abs(res.tool_along - hi)) < 0.05:
            problems.append(f"tool length {res.tool_along:.1f} mm sits ON its bound [{lo:.1f}, {hi:.1f}]: the fit is "
                            f"using the pen length to compensate for an error elsewhere (a direction sign or the "
                            f"L pose) -- run `doodle calib diagnose`")
        if problems:
            print("\nREFUSED to save, the fit is not physical:")
            for pr in problems:
                print(f"  - {pr}")
            print("Nothing was written. Re-run `doodle calib paper`, then refine again with the\n"
                  "pen tilt varied between points; if it keeps happening, check `tool.along`\n"
                  "in the config against the real pen.")
            return 1

        if not a.no_save:
            # Moving a zero redefines q for that joint, so the limits -- which
            # describe fixed physical travel -- have to move with it. Equivalent
            # to holding the joint's tick window constant, which is what the
            # travel actually is.
            dzero = np.asarray(res.zero_ticks, float) - np.asarray(cal.zero_ticks, float)
            dq = -np.asarray(cal.direction, float) * dzero * RAD_PER_TICK
            if np.abs(np.degrees(dq)).max() > 0.05:
                print("joint limits shifted with the new zeros (deg):")
                for j, d in zip(cfg.joints, dq):
                    if abs(d) > 1e-9:
                        lo, hi = math.degrees(j.min_rad), math.degrees(j.max_rad)
                        j.min_rad += float(d)
                        j.max_rad += float(d)
                        print(f"  {j.name:14s} [{lo:7.1f},{hi:7.1f}] -> "
                              f"[{math.degrees(j.min_rad):7.1f},{math.degrees(j.max_rad):7.1f}]  ({math.degrees(d):+.2f})")
            cal.zero_ticks = [float(v) for v in res.zero_ticks]
            cal.tool_along = res.tool_along
            cal.paper = res.paper
            p = save_calibration(cal, a.calib)
            pc = save_config(cfg, a.config)
            print(f"saved {p}\nsaved {pc}")
    finally:
        arm.bus.close()
    return 0


def cmd_calib_show(a, cfg, cal):
    print(f"joints calibrated: {cal.joints_calibrated}")
    for n, z, d in zip(cfg.joint_names, cal.zero_ticks, cal.direction):
        print(f"  {n:14s} zero={z:7.1f} ticks  dir={d:+d}")
    print(f"tool length: {cal.tool_along if cal.tool_along is not None else cfg.tool.along} mm"
          + ("" if cal.tool_along is not None else " (config default, not refined)"))
    pf = cal.paper
    print(f"paper calibrated: {pf.calibrated}  origin={np.round(pf.origin, 1).tolist()} "
          f"x={np.round(pf.x_axis, 3).tolist()} y={np.round(pf.y_axis, 3).tolist()}")
    return 0


def _ik_paper_uv(kin, cfg, cal, u, v, w, where):
    """Solve IK for a paper (u, v, w) point, or raise SystemExit."""
    xyz = cal.paper.to_world(u, v, w)
    res = kin.ik(xyz, cfg.motion.pen_tilt_options_deg)
    if not res.ok:
        raise SystemExit(f"unreachable {where}: {res.reason}")
    return res.q


def cmd_calib_corners(a, cfg, cal):
    """Drive the pen onto each canvas corner, returning to q=0 between them.

    A visual check that the paper frame and IK agree with the drawn box: the
    tip should land on the marks. Hovering above each corner before the
    descent keeps the joint-space move off the page.
    """
    from .executor import Executor
    if not a.dry_run and not (cal.joints_calibrated and cal.paper.calibrated):
        raise SystemExit("run `doodle calib pose` and `doodle calib paper` first")
    kin = kin_for(cfg, cal)
    q0 = np.zeros(5)
    zu, zd = cfg.motion.pen_up_z, -cfg.motion.pen_press
    corners = []
    for name, u, v in cfg.paper.canvas_corners():
        hover = _ik_paper_uv(kin, cfg, cal, u, v, zu, f"{name} hover")
        touch = _ik_paper_uv(kin, cfg, cal, u, v, zd, f"{name} touch")
        corners.append((name, u, v, hover, touch))

    arm = open_arm(cfg, cal, a.dry_run)
    ex = None
    try:
        arm.check_q(q0, "(zero)")
        arm.torque(True)
        ex = Executor(arm, cfg, cal, kin, verbose=True)
        print("Touching each canvas corner, returning to q=0 between them:")
        for name, u, v, _, _ in corners:
            print(f"  {name:10s}  paper u={u:.1f} v={v:.1f} mm")
        ex.approach(q0, a.speed)
        for name, u, v, hover, touch in corners:
            print(f"{name}: hover, touch, back to zero")
            ex.approach(hover, a.speed)
            ex.approach(touch, a.speed)
            if a.hold > 0:
                time.sleep(a.hold)
            ex.approach(hover, a.speed)
            ex.approach(q0, a.speed)
        print("q (deg):", fmt_q(arm.read_q()), " tip:", np.round(kin.fk(arm.read_q()), 1).tolist())
    except KeyboardInterrupt:
        if ex is not None:
            ex.abort("Ctrl-C")
        raise
    except SafetyError as e:
        if ex is not None:
            ex.abort(str(e))
        raise SystemExit(f"refused: {e}")
    finally:
        arm.bus.close()
    return 0


# --- motion ------------------------------------------------------------------------------------
def cmd_jog(a, cfg, cal):
    arm = open_arm(cfg, cal, a.dry_run)
    kin = kin_for(cfg, cal)
    try:
        arm.torque(True)
        q = arm.read_q()
        if a.joint in arm.names:
            q[arm.names.index(a.joint)] += math.radians(a.amount)
        elif a.joint in ("x", "y", "z"):
            tip = kin.fk(q)
            tip["xyz".index(a.joint)] += a.amount
            res = kin.ik(tip, cfg.motion.pen_tilt_options_deg)
            if not res.ok:
                raise SystemExit(f"unreachable: {res.reason}")
            q = res.q
        else:
            raise SystemExit(f"joint must be one of {arm.names} or x/y/z")
        arm.move_to_q(q, speed_deg_s=a.speed)
        arm.wait_settled()
        q = arm.read_q()
        print("q (deg):", fmt_q(q), " tip:", np.round(kin.fk(q), 1).tolist())
    except SafetyError as e:
        raise SystemExit(f"refused: {e}")
    finally:
        arm.bus.close()
    return 0


def cmd_goto(a, cfg, cal):
    arm = open_arm(cfg, cal, a.dry_run)
    kin = kin_for(cfg, cal)
    try:
        arm.torque(True)
        if a.where == "park":
            q = np.radians(cfg.park_pose_deg)
        elif a.where == "lpose":
            q = calib.reference_pose_q(cfg)
        else:
            u, v, w = (float(s) for s in a.where.split(","))
            u0, v0 = cfg.paper.canvas_origin()
            res = kin.ik(cal.paper.to_world(u0 + u, v0 + v, w), cfg.motion.pen_tilt_options_deg)
            if not res.ok:
                raise SystemExit(f"unreachable: {res.reason}")
            q = res.q
        from .executor import Executor
        ex = Executor(arm, cfg, cal, kin, verbose=True)
        ex.approach(q, a.speed)
        print("q (deg):", fmt_q(arm.read_q()), " tip:", np.round(kin.fk(arm.read_q()), 1).tolist())
    except SafetyError as e:
        raise SystemExit(f"refused: {e}")
    finally:
        arm.bus.close()
    return 0


def cmd_shapes(a, cfg, cal):
    ASSETS.mkdir(exist_ok=True)
    P = cfg.paper
    for name in (a.names or sorted(GENERATORS)):
        d = make(name)
        d.save(ASSETS / f"{name}.json")
        (ASSETS / f"{name}.svg").write_text(d.to_svg())
        fitted = d.fit_to(P.canvas_width, P.canvas_height, P.margin)
        render_drawing(fitted, P.canvas_width, P.canvas_height, ASSETS / f"{name}.png")
        print(f"{name:8s} {len(d.strokes)} stroke(s), {d.total_length():.0f} mm  -> assets/{name}.{{json,svg,png}}")
    return 0


def _plan_or_exit(spec, cfg, cal):
    drawing = load_drawing(spec)
    try:
        return drawing, plan(drawing, cfg, cal)
    except PlanError as e:
        raise SystemExit(f"cannot plan {drawing.name!r}: {e}")


def print_stats(traj):
    s = traj.stats
    print(f"  {s['strokes']} strokes, ink {s['ink_mm']:.0f} mm, travel {s['travel_mm']:.0f} mm, "
          f"{s['duration_s']:.1f} s, {s['samples']} samples, max joint speed {s['max_joint_speed_deg_s']:.0f} deg/s"
          + (f", time stretched x{s['time_dilation']:.2f}" if s['time_dilation'] > 1.001 else "")
          + (f", {s['tilted_samples']} samples need pen tilt (max {s['max_tilt_deg']:.0f} deg)" if s['tilted_samples'] else ""))


def cmd_preview(a, cfg, cal):
    drawing, traj = _plan_or_exit(a.drawing, cfg, cal)
    out = Path(a.out or ASSETS / "preview")
    out.mkdir(parents=True, exist_ok=True)
    P = cfg.paper
    p1 = render_toolpath(traj.preview_segments(), P.canvas_width, P.canvas_height, out / f"{drawing.name}_toolpath.png", title=drawing.name)
    p2 = render_joints(traj.t, traj.q, cfg.joint_names, out / f"{drawing.name}_joints.png", traj.pen_down)
    p3 = render_layout(cfg, cal, out / "layout.png", canvas_uv0=traj.canvas_uv0)
    print(f"planned {drawing.name}:")
    print_stats(traj)
    print(f"  wrote {p1}\n        {p2}\n        {p3}")
    return 0


def cmd_layout(a, cfg, cal):
    ASSETS.mkdir(exist_ok=True)
    kin = kin_for(cfg, cal)
    z = cal.paper.origin[2]
    rmin, rmax = kin.radial_reach(z)
    rmin2, rmax2 = kin.radial_reach(z, cfg.motion.pen_tilt_options_deg)
    print(f"at paper height z={z:.1f}: pen-vertical reach {rmin:.0f}..{rmax:.0f} mm from the pan axis; "
          f"with tilt {rmin2:.0f}..{rmax2:.0f} mm")
    u0, v0 = cfg.paper.canvas_origin()
    corners = [cal.paper.to_world(u0 + u, v0 + v) for u, v in [(0, 0), (cfg.paper.canvas_width, 0),
               (cfg.paper.canvas_width, cfg.paper.canvas_height), (0, cfg.paper.canvas_height)]]
    bad = [c for c in corners if not kin.ik(c, cfg.motion.pen_tilt_options_deg).ok]
    print(f"canvas {cfg.paper.canvas_width:.0f}x{cfg.paper.canvas_height:.0f} at paper ({u0:.1f}, {v0:.1f}); "
          f"corners reachable: {len(corners) - len(bad)}/4")
    p = render_layout(cfg, cal, ASSETS / "layout.png", canvas_uv0=(u0, v0))
    print(f"wrote {p}")
    return 0 if not bad else 1


def cmd_draw(a, cfg, cal):
    from .executor import Executor
    if not a.dry_run and not (cal.joints_calibrated and cal.paper.calibrated):
        raise SystemExit("refusing to draw: joints or paper not calibrated (see `doodle calib show`)")
    if a.speed:
        cfg.motion.draw_speed = a.speed
    drawing, traj = _plan_or_exit(a.drawing, cfg, cal)
    print(f"plan for {drawing.name}:")
    print_stats(traj)
    if a.plan_only:
        return 0
    LOGS.mkdir(exist_ok=True)
    log = LOGS / f"{drawing.name}_{time.strftime('%Y%m%d_%H%M%S')}{'_dry' if a.dry_run else ''}.csv"
    arm = open_arm(cfg, cal, a.dry_run)
    try:
        ex = Executor(arm, cfg, cal, realtime=not a.fast, log_path=log)
        for rep_i in range(a.repeat):
            if a.repeat > 1:
                print(f"--- pass {rep_i + 1}/{a.repeat} ---")
            rep = ex.run(traj)
            print(f"{'done' if rep.ok else 'STOPPED'}: {rep.reason or 'ok'}  {rep.duration_s:.1f}s, {rep.samples_sent} setpoints, "
                  f"period {rep.mean_period_ms:.1f}ms (max {rep.max_period_ms:.1f}, late {rep.late_ticks}), "
                  f"tracking err max {rep.max_track_err_ticks:.0f} rms {rep.rms_track_err_ticks:.1f} ticks")
            if not rep.ok:
                break
        if not a.no_park:
            ex.park(torque_off=a.torque_off)
        print(f"log: {log}")
    finally:
        arm.bus.close()
    return 0



# --- digital twin --------------------------------------------------------------------------------
def cmd_twin(a, cfg, cal):
    """Serve a live 3-D view of the arm, paper and canvas to browsers on the LAN."""
    from . import twin as tw

    kin = kin_for(cfg, cal)
    store = tw.StateStore()
    def live():
        # The fake arm only converts ticks; the state comes from whichever
        # command owns the bus and broadcasts its reads on the telemetry tap.
        fake = open_arm(cfg, cal, dry_run=True)
        return fake, tw.TapSource(store, fake, kin, cfg, cal), "listening to the live telemetry tap"

    if a.follow:
        # A drawing log is the state source, so the twin never touches the bus.
        arm = open_arm(cfg, cal, dry_run=True)
        src = tw.LogFollower(store, arm, kin, cfg, cal, Path(a.follow), pace=not a.no_pace)
        what = f"following {a.follow}"
    elif a.live:
        arm, src, what = live()
    else:
        try:
            arm = open_arm(cfg, cal, a.dry_run)
        except BusBusy as e:
            # Another command owns the serial port. Never fight it: two masters on
            # a half-duplex bus corrupt each other's replies. Watch its telemetry.
            print(f"bus busy: {e}\n-> falling back to the live telemetry tap", flush=True)
            arm, src, what = live()
        else:
            src = tw.BusSampler(store, arm, kin, cfg, cal, rate=a.rate, demo=a.dry_run)
            what = "demo (simulated arm)" if a.dry_run else f"polling the bus at {a.rate:g} Hz"
    server = tw.serve(store, a.host, a.port)
    port = server.server_address[1]
    src.start()
    print(f"doodle twin: {what}")
    print("open in a browser on this network:")
    for ip in (tw.lan_addresses() or ["127.0.0.1"]):
        print(f"  http://{ip}:{port}/")
    print("Ctrl-C to stop.", flush=True)      # stdout is block-buffered when redirected
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        src.stop.set()
        server.server_close()
        arm.bus.close()
    return 0

def cmd_init_config(a, cfg, cal):
    p = save_config(cfg, a.config)
    print(f"wrote {p}")
    if not Path(a.calib).exists():
        print(f"wrote {save_calibration(cal, a.calib)}")
    return 0


# --- parser -------------------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="doodle", description="SO-101 doodle robot")
    p.add_argument("--config", default=str(ROOT / "config" / "doodle.yaml"))
    p.add_argument("--calib", default=str(ROOT / "config" / "calibration.yaml"))
    p.add_argument("--dry-run", action="store_true", help="use a simulated bus (no hardware)")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="list servos on the bus"); s.add_argument("--all-bauds", action="store_true"); s.add_argument("--max-id", type=int, default=20); s.set_defaults(f=cmd_scan)
    s = sub.add_parser("assign-ids", help="renumber servos one at a time"); s.add_argument("--joint"); s.set_defaults(f=cmd_assign_ids)
    s = sub.add_parser("info", help="dump one servo's registers"); s.add_argument("id", type=int); s.set_defaults(f=cmd_info)
    s = sub.add_parser("status", help="positions, voltage, temperature, pen tip"); s.add_argument("--watch", action="store_true", help="keep printing at --rate Hz until Ctrl-C"); s.add_argument("--rate", type=float, default=30.0, help="refresh rate in Hz when --watch is set"); s.set_defaults(f=cmd_status)
    s = sub.add_parser("torque", help="torque on/off for all joints"); s.add_argument("state", choices=["on", "off"]); s.set_defaults(f=cmd_torque)
    s = sub.add_parser("configure", help="write servo gains / limits from config"); s.set_defaults(f=cmd_configure)

    c = sub.add_parser("calib", help="calibration steps").add_subparsers(dest="sub", required=True)
    c.add_parser("pose", help="record the L reference pose").set_defaults(f=cmd_calib_pose)
    c.add_parser("check-dirs", help="verify joint direction signs").set_defaults(f=cmd_calib_check_dirs)
    s = c.add_parser("paper", help="touch three corners of the drawn canvas box"); s.set_defaults(f=cmd_calib_paper)
    s = c.add_parser("corners", help="drive each canvas corner, returning to q=0 between them")
    s.add_argument("--speed", type=float, default=None, help="joint-space deg/s for each move")
    s.add_argument("--hold", type=float, default=0.5, help="seconds to rest on each corner")
    s.set_defaults(f=cmd_calib_corners)
    s = c.add_parser("travel", help="sweep all joints to their stops and record the tick windows"); s.add_argument("--margin", type=int, default=15, help="ticks trimmed off each measured end"); s.add_argument("--min-span", type=int, default=200, help="ignore a sweep smaller than this"); s.set_defaults(f=cmd_calib_travel)
    s = c.add_parser("diagnose", help="test whether a joint direction sign explains a bad paper calibration"); s.add_argument("--tool", type=float, nargs="*", help="also try these tool lengths (mm)"); s.set_defaults(f=cmd_calib_diagnose)
    s = c.add_parser("refine", help="touch a grid and fit offsets + tool length"); s.add_argument("--grid", type=int, default=3); s.add_argument("--inset", type=float, default=20.0); s.add_argument("--no-tool", action="store_true"); s.add_argument("--no-save", action="store_true"); s.set_defaults(f=cmd_calib_refine)
    c.add_parser("show", help="print the calibration").set_defaults(f=cmd_calib_show)

    s = sub.add_parser("jog", help="move one joint (deg) or the tip along x/y/z (mm)"); s.add_argument("joint"); s.add_argument("amount", type=float); s.add_argument("--speed", type=float, default=15.0); s.set_defaults(f=cmd_jog)
    s = sub.add_parser("goto", help="park | lpose | u,v,z (canvas mm)"); s.add_argument("where"); s.add_argument("--speed", type=float, default=None); s.set_defaults(f=cmd_goto)
    s = sub.add_parser("shapes", help="generate the test assets"); s.add_argument("names", nargs="*"); s.set_defaults(f=cmd_shapes)
    s = sub.add_parser("preview", help="plan and render without moving"); s.add_argument("drawing"); s.add_argument("--out"); s.set_defaults(f=cmd_preview)
    s = sub.add_parser("layout", help="reachable area vs paper/canvas"); s.set_defaults(f=cmd_layout)
    s = sub.add_parser("draw", help="plan and draw"); s.add_argument("drawing"); s.add_argument("--speed", type=float, help="pen-down mm/s"); s.add_argument("--repeat", type=int, default=1); s.add_argument("--plan-only", action="store_true"); s.add_argument("--fast", action="store_true", help="dry-run without real-time pacing"); s.add_argument("--no-park", action="store_true"); s.add_argument("--torque-off", action="store_true"); s.set_defaults(f=cmd_draw)
    s = sub.add_parser("twin", help="serve a live 3-D digital twin to browsers on the LAN"); s.add_argument("--host", default="0.0.0.0"); s.add_argument("--port", type=int, default=8765); s.add_argument("--rate", type=float, default=20.0, help="bus polling rate, Hz"); s.add_argument("--follow", metavar="CSV", help="tail/replay a `doodle draw` log instead of polling the bus"); s.add_argument("--live", action="store_true", help="listen to the telemetry tap of whichever command owns the bus (automatic when the port is busy)"); s.add_argument("--no-pace", action="store_true", help="with --follow, do not replay at the log's own speed"); s.set_defaults(f=cmd_twin)
    s = sub.add_parser("init-config", help="write config/doodle.yaml with defaults"); s.set_defaults(f=cmd_init_config)
    return p


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    cfg = load_config(a.config)
    cal = load_calibration(a.calib)
    if a.dry_run:
        # never let a simulated session overwrite the real calibration or config
        # (`calib pose` solves the link geometry, so it writes the config too)
        dry = Path(a.calib).with_suffix(".dry-run.yaml")
        save_calibration(cal, dry)
        a.calib = str(dry)
        dry_cfg = Path(a.config).with_suffix(".dry-run.yaml")
        save_config(cfg, dry_cfg)
        a.config = str(dry_cfg)
    try:
        return a.f(a, cfg, cal) or 0
    except ServoError as e:            # timeouts, a busy port, a vanished USB adapter
        print(f"bus error: {e}", file=sys.stderr)
        return 2
    except SafetyError as e:
        print(f"refused (safety): {e}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
