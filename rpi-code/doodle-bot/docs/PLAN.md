# Doodle-bot: plan and architecture

Goal: an SO-101 arm (five servos; the gripper is replaced by the pen holder in
`models/simple_motor_to_pen.*`) draws simple black-and-white sketches with a
Sharpie fine tip, first on a letter sheet, later on a 4x6 notepad, in a way
that is **repeatable, steady and controlled**.

## What "repeatable, steady, controlled" requires

| Need | Component | Where |
|---|---|---|
| Talk to five servos reliably, without lerobot's six-motor assumption | raw Feetech STS3215 driver: ping/read/write, sync read/write, ID + baud tools, fake bus for dry runs | `doodle/servo.py` |
| One place that converts angles to ticks and refuses anything unsafe | `Arm`: joint limits in radians *and* ticks, torque-on without a jump, bounded-speed moves, servo tuning | `doodle/arm.py` |
| Know where the pen tip is for any joint state, and the reverse | planar FK/IK derived from the official URDF, pen-vertical constraint with bounded tilt fallback, reach maps | `doodle/kinematics.py` |
| Tie the model to the physical arm and the physical sheet | reference "L pose" for zero offsets, direction check, paper frame from touch-off, grid-fit refinement of offsets + tool length | `doodle/calibration.py` |
| Describe what to draw independently of the robot | `Drawing`/`Stroke` (polylines in mm), JSON/SVG, generators for circle, square, star, spiral, grid | `doodle/shapes.py` |
| Turn a drawing into smooth motion the servos can follow | fit-to-canvas, stroke ordering, pen up/down/dwell, acceleration-limited profile with corner slow-down, 50 Hz sampling, IK, joint-speed check | `doodle/planner.py` |
| Execute with eyes open | approach path checked against the paper plane, setpoint streaming with read-back, tracking-error / temperature abort, Ctrl-C lifts the pen, park with torque held, CSV log | `doodle/executor.py` |
| See before you draw | toolpath PNG, joint-trace PNG, top-down reach/paper/canvas layout | `doodle/preview.py` |
| Operate it | `doodle` CLI, `--dry-run` everywhere | `doodle/cli.py` |

## Kinematic model

World frame: origin on the table under the shoulder_pan axis, x forward, z up.

From the SO-101 URDF (TheRobotStudio `so101_new_calib.urdf`):

| quantity | value |
|---|---|
| shoulder_lift axis | 30.4 mm forward of the pan axis, 116.6 mm above the table |
| upper arm (lift -> elbow) | 116.0 mm, 76.04° from horizontal at URDF zero |
| forearm (elbow -> wrist_flex) | 135.0 mm, 2.21° at zero |
| wrist_flex -> wrist_roll axis | 61.1 mm, along the forearm |
| lateral offset of the arm plane | 18.3 mm at the shoulder, cancelled at the wrist_roll frame, so the pen lies in the plane through the pan axis |
| pen tip | `tool.along` mm out along the roll axis (default 100, refined by calibration) |

The three pitch joints are parallel, so IK is: pan from `atan2(y, x)`, then a
closed-form 3-link planar solve with the pen constrained vertical (or tilted
by a bounded fallback when the wrist limit is hit). Our pan sign is
counter-clockwise from above, opposite to the URDF's; pitch signs match. The
planar model is verified against a full 3D URDF FK to < 0.6 mm in
`tests/test_kinematics.py`.

Usable workspace at table height with the pen vertical: 52 to 277 mm from the
pan axis. A 150 x 100 mm canvas starting 130 mm out fits comfortably; a whole
letter sheet does not, which is why drawings are scaled into a canvas.

## Motion strategy

* Position mode, P gain lowered to 16 (lerobot's value) to avoid shakiness,
  torque limit 70 % so a collision stalls instead of stripping gears.
* Setpoints are streamed at 50 Hz along a time-parameterised path. The tip
  speed is 30 mm/s drawing, 80 mm/s travelling, 200 mm/s² acceleration, and
  slows to 8 mm/s through sharp corners so the servos never see a step.
* The pen is pushed 1.5 mm below the fitted paper plane (servo compliance
  supplies the pressure) and lifted 8 mm between strokes, with 150 ms dwells
  at every pen transition.
* Joint speed is capped at 90°/s; if a plan would exceed it the whole
  timeline is stretched rather than clipped.
* Present positions are read back every cycle. A lag above 250 ticks (~22°)
  or a servo above 65 °C aborts; the abort lifts the pen straight up.

## Calibration procedure (needs the hardware)

1. `doodle assign-ids` - one servo on the bus at a time (factory ID is 1).
2. `doodle scan`, `doodle status`, `doodle configure`.
3. `doodle calib pose` - hold the arm in the L pose (upper arm plumb, forearm
   horizontal, pen down) and record. Gives zero offsets to a few degrees.
4. `doodle calib check-dirs` - each joint nudges +8° and you confirm the
   direction; wrong ones are flipped in the calibration file.
5. `doodle calib paper` - torque off, touch the two near corners and a ruler
   mark on the left edge. Prints the measured vs nominal distances as a check.
6. `doodle layout` - confirm the canvas is inside the reachable zone.
7. `doodle draw grid` - the ruled grid is the best repeatability test:
   measure the squares, run it twice on the same sheet, look for doubling.
8. Optional `doodle calib refine` - touch a 3x3 grid of marks inside the
   canvas with varied pen tilt; fits the lift/elbow/wrist zero offsets, the
   tool length and the paper pose (Levenberg-Marquardt in numpy). The pan
   offset is not fitted because the paper pose absorbs it exactly. Verified
   in simulation to recover offsets to ~2 ticks and the tool length to
   <0.7 mm from 0.3-tick touch noise.

## Verified without hardware

* 57 tests: packet encoding, FK vs URDF, IK round trips, planner speed and
  z-level invariants for every shape, calibration recovery, executor abort
  and approach safety.
* Every CLI command runs against the fake bus with `--dry-run`; dry runs
  write to `config/calibration.dry-run.yaml` and never touch the real file.

## Not done yet / next steps

* Servo IDs are still all 1 (`doodle scan` shows only ID 1); steps 1-7 above.
* Tool length (`tool.along`) is a guess until `calib paper`/`refine` run;
  the pen holder STEP shows a 15 mm pen bore and a 20 mm horn face, but how
  far the Sharpie protrudes is up to how it is clamped.
* Servo units for `goal_velocity`/`acceleration` follow the Feetech SDK
  convention (ticks/s, x100 ticks/s²); tune on hardware.
* Input formats beyond the generators: an SVG importer and a
  photo -> edge -> polyline tracer both only need to produce a `Drawing`.
* 4x6 notepad: set `paper.width/height` to 101.6 x 152.4 and a canvas of
  about 90 x 140 in `config/doodle.yaml`; the notepad's far corners are in
  reach so `calib paper --edge-mark 152.4` can use the real corner.
