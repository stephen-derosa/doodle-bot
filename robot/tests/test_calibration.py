import pytest
import math
import numpy as np

from doodle import calibration as calib
from doodle.arm import RAD_PER_TICK
from doodle.servo import TICKS_PER_REV
from doodle.config import Calibration, Config, PaperFrame, Tool
from doodle.kinematics import SO101Kinematics


def test_reference_pose_geometry():
    cfg = Config()
    k = SO101Kinematics.from_config(cfg)
    q = calib.reference_pose_q(cfg)
    phi1, phi2, phi3 = k.link_angles(q)
    assert math.isclose(phi1, math.pi / 2, abs_tol=1e-9)   # upper arm vertical
    assert math.isclose(phi2, 0.0, abs_tol=1e-9)           # forearm horizontal
    assert math.isclose(phi3, -math.pi / 2, abs_tol=1e-9)  # pen straight down


def test_zero_ticks_roundtrip():
    cfg = Config()
    q = calib.reference_pose_q(cfg)
    direction = np.array([1, -1, 1, -1, 1])
    true_zero = np.array([2000, 2100, 1900, 2048, 2048.0])
    ticks = true_zero + direction * q / RAD_PER_TICK
    zero = calib.zero_ticks_from_pose(ticks, q, direction)
    assert np.allclose(zero, true_zero)


def test_paper_frame_from_corners():
    cfg = Config()
    k = SO101Kinematics.from_config(cfg)
    frame_true = PaperFrame(origin=[100, 108, 0], x_axis=[0, -1, 0], y_axis=[1, 0, 0])
    W, H = 150.0, 100.0
    touches = []
    for lbl, uv in [("origin", (0, 0)), ("x", (W, 0)), ("y", (0, H))]:
        q = k.ik(frame_true.to_world(*uv)).q
        touches.append(calib.TouchPoint(lbl, uv, list(q / RAD_PER_TICK + 2048)))
    ticks_to_q = lambda t: (np.asarray(t) - 2048) * RAD_PER_TICK
    frame, rep = calib.paper_frame_from_corners(k, ticks_to_q, touches, W, H)
    assert np.allclose(frame.origin, frame_true.origin, atol=1e-6)
    assert np.allclose(frame.x_axis, frame_true.x_axis, atol=1e-9)
    assert np.allclose(frame.y_axis, frame_true.y_axis, atol=1e-9)
    assert abs(rep["x_err"]) < 1e-6 and abs(rep["y_err"]) < 1e-6


def test_refine_recovers_offsets_and_tool_length():
    """Simulate a robot whose true zero offsets and tool length differ from the
    calibration file, touch a 3x3 grid, and check the fit recovers them."""
    cfg = Config()
    cfg.tool.along = 100.0
    true_tool = 104.5
    true_zero = np.array([2048.0, 2048.0, 2048.0, 2048.0, 2048.0]) + np.array([12.0, -20.0, 15.0, -9.0, 0.0])
    direction = np.array([1, 1, 1, 1, 1])
    true_kin = SO101Kinematics.from_config(cfg, Tool(along=true_tool))
    frame_true = PaperFrame(origin=[110, 100, 0], x_axis=[math.cos(-1.55), math.sin(-1.55), 0],
                            y_axis=[-math.sin(-1.55), math.cos(-1.55), 0])
    cal = Calibration(zero_ticks=[2048.0] * 5, direction=list(direction), joints_calibrated=True)
    touches = []
    rng = np.random.default_rng(0)
    tilts = [-12, 0, 12, 8, -8, 0, 12, -12, 5]   # a human never holds the pen at one angle
    for (lbl, uv), tilt in zip(calib.grid_points(150, 100, 3, inset=15), tilts):
        q = true_kin.ik(frame_true.to_world(*uv), tilt_options_deg=(tilt,)).q
        ticks = true_zero + direction * q / RAD_PER_TICK + rng.normal(0, 0.3, 5)  # 0.3 tick noise
        touches.append(calib.TouchPoint(lbl, uv, list(ticks)))
    res = calib.refine_from_grid(cfg, cal, touches, fit_tool=True)
    assert res.rms_before > 0.25         # the uncorrected model is visibly wrong
    assert res.rms_after < 0.15          # and the fit is at the noise floor
    # pan is unobservable (absorbed by the paper pose) and stays untouched
    assert res.zero_ticks[0] == cal.zero_ticks[0]
    assert np.allclose(res.zero_ticks[1:4], true_zero[1:4], atol=2.0)
    assert abs(res.tool_along - true_tool) < 0.7
    # The fitted paper pose is rotated about the pan axis relative to the true
    # one (it absorbs the 12-tick pan error), which is fine as long as the
    # model + fitted frame agree on where every touched point is.
    assert abs(res.zero_ticks[0] - true_zero[0]) > 5
    assert np.abs(res.residuals).max() < 0.5


def test_geometry_from_pose_inverts_reference_pose_q():
    """A -> q_pose -> A must round-trip, since the two parameterise one measurement."""
    cfg = Config()
    cfg.geometry.zero_angle1_deg = 76.04
    cfg.geometry.zero_angle2_deg = 2.21
    cfg.geometry.zero_angle3_deg = 0.0
    q = calib.reference_pose_q(cfg)
    A = np.degrees(calib.geometry_from_pose(q[1:4]))
    assert np.allclose(A, [76.04, 2.21, 0.0], atol=1e-9)


def test_geometry_from_pose_centres_the_chain_on_the_servo_centre():
    """Pinning zeros to the encoder centre puts the L pose near q=0.

    With the nominal design angles the L pose left wrist_flex at ~88 deg of a
    +-95 deg range; solving A from the measurement instead is what recovers the
    travel.
    """
    cfg = Config()
    ticks = np.array([2180.0, 2083.0, 2060.0, 2064.0, 998.0])   # measured at the L pose
    direction = np.ones(5)
    centre = TICKS_PER_REV / 2
    q_chain = direction[1:4] * (ticks[1:4] - centre) * RAD_PER_TICK

    A = calib.geometry_from_pose(q_chain)
    (cfg.geometry.zero_angle1_deg,
     cfg.geometry.zero_angle2_deg,
     cfg.geometry.zero_angle3_deg) = (float(math.degrees(v)) for v in A)

    q_ref = calib.reference_pose_q(cfg)
    assert np.allclose(q_ref[1:4], q_chain, atol=1e-9)
    # every chain joint now sits within a couple of degrees of its servo centre
    assert np.abs(np.degrees(q_ref[1:4])).max() < 5.0


# --- paper diagram -------------------------------------------------------------
CANVAS = (50.8, 30.0, 101.6, 152.4)      # u0, v0, w, h  -- a 4 x 6 in box
SHEET = (203.2, 266.7)                   # 8 x 10.5 in, short edge at the robot


def _drawing(point, width=SHEET[0], height=SHEET[1], canvas=CANVAS):
    """The sheet art only, with the caption split off."""
    parts = calib.paper_diagram(point, width, height, canvas).split("\n\n")
    return parts[0], parts[1]


def test_paper_diagram_marks_exactly_one_point():
    for point in ("origin", "x", "y"):
        art, caption = _drawing(point)
        assert art.count("X") == 1
        assert "X = " in caption


def test_paper_diagram_marks_canvas_corners_not_sheet_corners():
    """Every touch point must be a corner of the drawn box, inside the sheet.

    The sheet's own far corners are ~350 mm out and unreachable, which is why
    this used to ask for a ruler mark instead of a third corner.
    """
    for point in ("origin", "x", "y"):
        art, _ = _drawing(point)
        rows = art.splitlines()
        r = [i for i, line in enumerate(rows) if "X" in line][0]
        line = rows[r]
        c = line.index("X")
        # the X sits on the dotted canvas box, never on the solid sheet border
        assert line[c - 1] in ". |" and rows[r].count("X") == 1
        assert 0 < r < len(rows) - 2          # not the sheet's top/bottom border
        assert "." in line                    # the canvas box is drawn on this row


def test_paper_diagram_corners_are_the_three_distinct_box_corners():
    spots = {}
    for point in ("origin", "x", "y"):
        art, _ = _drawing(point)
        rows = art.splitlines()
        # the 'x' diagram carries a dimension bar above the sheet, so count rows
        # from the sheet's top border rather than from the top of the art
        base = next(i for i, line in enumerate(rows) if line.strip().startswith("+"))
        r = [i for i, line in enumerate(rows) if "X" in line][0]
        spots[point] = (r - base, rows[r].index("X"))
    assert len({*spots.values()}) == 3
    assert spots["origin"][0] == spots["x"][0]      # both on the near edge
    assert spots["origin"][1] < spots["x"][1]       # origin left of x
    assert spots["y"][1] == spots["origin"][1]      # y straight up from origin
    assert spots["y"][0] < spots["origin"][0]       # and further from the robot


def test_paper_diagram_dimensions_are_the_canvas_not_the_sheet():
    art, _ = _drawing("x")
    assert "102 mm" in art and "203 mm" not in art      # box width, not sheet width
    art, _ = _drawing("y")
    assert "152 mm" in art and "267 mm" not in art      # box height, not sheet height


def test_paper_diagram_canvas_position_follows_the_offset():
    """Moving the box up the page must move the drawn box up too."""
    def near_row(v0):
        art, _ = _drawing("origin", canvas=(50.8, v0, 101.6, 152.4))
        return [i for i, line in enumerate(art.splitlines()) if "X" in line][0]

    assert near_row(0.0) > near_row(40.0) > near_row(80.0)


def test_paper_diagram_keeps_the_box_inside_the_sheet():
    """An oversized or badly offset box must still be drawn within the border."""
    for canvas in [(0.0, 0.0, 203.2, 266.7), (0.0, 0.0, 999.0, 999.0), (200.0, 260.0, 50.0, 50.0)]:
        for point in ("origin", "x", "y"):
            art, _ = _drawing(point, canvas=canvas)
            rows = [r for r in art.splitlines() if r.strip().startswith(("+", "|"))]
            assert all(r.strip()[0] in "+|" and r.strip()[-1] in "+|" for r in rows)
            assert art.count("X") == 1


def _drawn_aspect(width, height):
    """Height/width of the drawn box, correcting for non-square character cells."""
    art, _ = _drawing("origin", width=width, height=height, canvas=(0.0, 0.0, width / 2, height / 2))
    box = [r for r in art.splitlines() if r.strip().startswith(("+", "|"))]
    cols = len(box[0].strip())
    return (len(box) * calib._CELL_ASPECT) / cols


def test_paper_diagram_is_drawn_true_to_the_sheet_shape():
    """The robot stands at the `width` edge, so a portrait sheet must look portrait.

    Regression: the box was a fixed 46x9, drawing letter portrait at an aspect
    of 0.46 when the sheet is 1.29 -- effectively rotated 90 deg, implying the
    robot was on the long edge.
    """
    for width, height in [(203.2, 266.7), (210.0, 297.0), (279.4, 215.9), (200.0, 200.0)]:
        assert _drawn_aspect(width, height) == pytest.approx(height / width, rel=0.15)


def test_paper_diagram_puts_the_short_edge_at_the_robot_for_portrait():
    art, _ = _drawing("origin")
    box = [r for r in art.splitlines() if r.strip().startswith(("+", "|"))]
    assert len(box) * calib._CELL_ASPECT > len(box[0].strip())


class _IdentityKin:
    """FK that hands back whatever 'ticks' were recorded, so touches are world points."""
    tool = Tool()

    def fk(self, q):
        return np.asarray(q, float)


def test_paper_frame_from_canvas_corners_recovers_the_sheet_origin():
    """Touching the box must still yield the frame of the sheet it sits on.

    The three touches are the box's corners at (u0, v0), so the solve has to
    step back by that offset to report the paper's own (0, 0).
    """
    true_o = np.array([70.0, 101.6, -48.3])
    x_axis, y_axis = np.array([0.0, -1.0, 0.0]), np.array([1.0, 0.0, 0.0])
    u0, v0, cw, ch = CANVAS

    def world(u, v):
        return true_o + u * x_axis + v * y_axis

    touches = [
        calib.TouchPoint("origin", (u0, v0), list(world(u0, v0))),
        calib.TouchPoint("x", (u0 + cw, v0), list(world(u0 + cw, v0))),
        calib.TouchPoint("y", (u0, v0 + ch), list(world(u0, v0 + ch))),
    ]
    frame, rep = calib.paper_frame_from_corners(
        _IdentityKin(), lambda t: t, touches, cw, ch, origin_uv=(u0, v0))

    assert np.allclose(frame.origin, true_o, atol=1e-6)
    assert np.allclose(frame.x_axis, x_axis, atol=1e-9)
    assert np.allclose(frame.y_axis, y_axis, atol=1e-9)
    # the scale check now reports the box, which is what was measured
    assert rep["x_len_measured"] == pytest.approx(cw)
    assert rep["y_len_measured"] == pytest.approx(ch)


def test_paper_frame_origin_uv_defaults_to_no_offset():
    o = np.array([70.0, 101.6, -48.3])
    x_axis, y_axis = np.array([0.0, -1.0, 0.0]), np.array([1.0, 0.0, 0.0])
    touches = [
        calib.TouchPoint("origin", (0, 0), list(o)),
        calib.TouchPoint("x", (100, 0), list(o + 100 * x_axis)),
        calib.TouchPoint("y", (0, 50), list(o + 50 * y_axis)),
    ]
    frame, _ = calib.paper_frame_from_corners(_IdentityKin(), lambda t: t, touches, 100.0, 50.0)
    assert np.allclose(frame.origin, o, atol=1e-6)


def _grid_uv(n=3, inset=20.0):
    u0, v0, cw, ch = CANVAS
    return [uv for _, uv in calib.grid_points(cw, ch, n, inset=inset, u0=u0, v0=v0)]


def _grid_art(uv, all_uv=None, **kw):
    all_uv = _grid_uv() if all_uv is None else all_uv
    parts = calib.grid_diagram(uv, all_uv, SHEET[0], SHEET[1], CANVAS, **kw).split("\n\n")
    return parts[0], parts[1]


def _sheet_lines(art):
    """Only the drawn sheet, excluding the '^^^ robot ^^^' marker below it."""
    return [r for r in art.splitlines() if r.strip().startswith(("+", "|"))]


def test_grid_diagram_marks_one_point_and_shows_the_rest():
    all_uv = _grid_uv()
    art, caption = _grid_art(all_uv[0])
    sheet = "\n".join(_sheet_lines(art))
    assert sheet.count("X") == 1
    assert sheet.count("o") == len(all_uv) - 1      # the active one is drawn as X
    assert "o = the other grid marks" in caption


def test_grid_diagram_moves_the_x_between_points():
    all_uv = _grid_uv()

    def spot(uv):
        art, _ = _grid_art(uv)
        rows = art.splitlines()
        r = [i for i, line in enumerate(rows) if "X" in line][0]
        return r, rows[r].index("X")

    spots = [spot(uv) for uv in all_uv]
    assert len(set(spots)) == len(all_uv)           # every grid point is distinct
    # first row of the grid is nearest the robot, so it draws lowest
    assert spots[0][0] > spots[-1][0]
    # within a row, u increases to the right
    assert spots[0][1] < spots[2][1]


def test_grid_diagram_keeps_every_mark_inside_the_canvas_box():
    all_uv = _grid_uv()
    for uv in all_uv:
        art, _ = _grid_art(uv)
        for line in _sheet_lines(art):
            for ch in "Xo":
                if ch in line:
                    body = line.strip()
                    # a mark never lands on the sheet border itself
                    assert body[0] in "+|" and body[-1] in "+|"
                    assert body[1:-1].count(ch) == line.count(ch)


def test_grid_diagram_caption_carries_progress_and_coordinates():
    all_uv = _grid_uv()
    _, caption = _grid_art(all_uv[4], label="g11", index=5, total=9)
    assert "g11" in caption and "5 of 9" in caption
    assert f"u={all_uv[4][0]:.0f}" in caption and f"v={all_uv[4][1]:.0f}" in caption


def test_grid_and_paper_diagrams_share_the_same_sheet_art():
    """Both flows must draw the identical sheet, so the picture reads the same."""
    paper_art, _ = _drawing("origin")
    grid_art, _ = _grid_art(_grid_uv()[0])
    # paper marks a box corner (X replaces a dot); grid marks inside it (X/o replace blanks)
    paper = [r.replace("X", ".") for r in _sheet_lines(paper_art)]
    grid = [r.replace("X", " ").replace("o", " ") for r in _sheet_lines(grid_art)]
    assert paper == grid


def _degenerate_touches(cfg, cal, tool_true=82.55, tilt=0.0):
    """A 3x3 grid touched at one constant pen tilt -- the degenerate case.

    Wrist offset and tool length are then nearly interchangeable, which is the
    direction an unbounded solve runs away along.
    """
    true_kin = SO101Kinematics.from_config(cfg, Tool(along=tool_true))
    frame = PaperFrame(origin=[110, 100, 0], x_axis=[0, -1, 0], y_axis=[1, 0, 0])
    direction = np.ones(5)
    rng = np.random.default_rng(1)
    touches = []
    for lbl, uv in calib.grid_points(100, 150, 3, inset=20):
        q = true_kin.ik(frame.to_world(*uv), tilt_options_deg=(tilt,)).q
        ticks = 2048.0 + direction * q / RAD_PER_TICK + rng.normal(0, 0.5, 5)
        touches.append(calib.TouchPoint(lbl, uv, list(ticks)))
    return touches


def test_refine_cannot_run_the_tool_length_away():
    """Bounds must hold even from an absurd starting point.

    Regression: an unbounded solve reported `tool length: 1562.2 -> 1386.2 mm`
    for an 82.55 mm pen, and saving it made IK demand elbow_flex = -133 deg.
    """
    cfg = Config()
    cfg.tool.along = 82.55
    cal = Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    cal.tool_along = 1562.2                      # a previous run's runaway value
    res = calib.refine_from_grid(cfg, cal, _degenerate_touches(cfg, cal), fit_tool=True)

    lo = (1 - calib.TOOL_BOUND_FRAC) * cfg.tool.along
    hi = (1 + calib.TOOL_BOUND_FRAC) * cfg.tool.along
    assert lo <= res.tool_along <= hi
    assert all(0 <= z < 4096 for z in res.zero_ticks)


def test_refine_bounds_do_not_follow_the_previous_fit():
    """Anchoring to cfg.tool.along stops repeated runs from compounding."""
    cfg = Config()
    cfg.tool.along = 82.55
    cal = Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    touches = _degenerate_touches(cfg, cal)

    first = calib.refine_from_grid(cfg, cal, touches, fit_tool=True)
    cal.tool_along = first.tool_along            # feed the fit back in, as the CLI does
    second = calib.refine_from_grid(cfg, cal, touches, fit_tool=True)
    assert abs(second.tool_along - first.tool_along) < 5.0


def test_levenberg_marquardt_stops_once_the_gain_is_negligible():
    """Regression: it ground out 33 iterations with rms stuck at 3.8595."""
    calls = []

    def residual(x):
        calls.append(1)
        return np.array([x[0] - 1.0, 0.5 * (x[1] - 2.0)])

    x, rms = calib.levenberg_marquardt(residual, [0.0, 0.0], iters=60)
    # stopping on negligible gain trades the last digits for not grinding
    assert np.allclose(x, [1.0, 2.0], atol=1e-3)
    assert len(calls) < 60 * 5          # converged early rather than burning every iteration


def test_l_pose_levels_the_roll_axis_for_a_side_mounted_pen():
    """The L pose fixes the pen straight down; for a side mount that is a level roll axis."""
    cfg = Config()
    cfg.tool.pen_angle_deg = -90.0
    k = SO101Kinematics.from_config(cfg)
    q = calib.reference_pose_q(cfg)
    phi1, phi2, phi3 = k.link_angles(q)
    assert math.isclose(phi3, 0.0, abs_tol=1e-9)
    assert np.allclose(k.pen_direction(q), (0, 0, -1), atol=1e-9)
    A = np.degrees(calib.geometry_from_pose(q[1:4], cfg.tool.pen_angle_deg))
    assert np.allclose(A, [cfg.geometry.zero_angle1_deg, cfg.geometry.zero_angle2_deg,
                           cfg.geometry.zero_angle3_deg], atol=1e-9)
