# doodle-bot

An SO-101 arm on a Raspberry Pi 4 (`rover`) that draws simple sketches with a
Sharpie. The gripper is replaced by a pen holder (`models/`), so the arm has
five servos. `docs/PLAN.md` explains the design; `notes/hardware-status.md`
has the bus diagnosis history.

## Status

Software is complete and tested against a simulated bus. The servos are still
at their factory ID (all five answer as ID 1), so the hardware bring-up steps
below have not been run yet.

## Setup

```bash
source .venv/bin/activate          # Python 3.12, numpy, pyserial, pyyaml, pillow, lerobot
uv pip install -e .                # installs the `doodle` command
python -m pytest -q                # 57 tests, no hardware needed
```

Config lives in `config/doodle.yaml` (port, joints, geometry, speeds, paper)
and `config/calibration.yaml` (measured offsets and paper frame). Recreate
the defaults with `doodle init-config`.

## Bring-up

```bash
doodle assign-ids                 # one servo connected at a time, sets IDs 1-5
doodle scan                       # all five joints should be listed
doodle status                     # ticks, voltage, temperature, pen-tip estimate
doodle configure                  # P/I/D, acceleration, torque limit from config
doodle calib travel               # sweep all joints to their stops, record real travel
doodle calib pose                 # hold the L pose, record zero offsets
doodle calib check-dirs           # confirm each joint's direction sign
doodle calib paper                # touch three corners of the drawn box
doodle calib diagnose             # if paper reports a scale error: is a joint direction sign wrong?
doodle calib corners              # drive each canvas corner, back to q=0 between
doodle layout                     # reach vs paper vs canvas -> assets/layout.png
doodle draw grid                  # first real drawing; measure it
```

`doodle calib refine` (touch a 3x3 grid) tightens the offsets and the tool
length once the basics work. Both `pose` and `paper` keep the raw servo
ticks they were derived from in `calibration.yaml`, so `calib diagnose` can
re-examine a bad result offline: it re-solves the geometry under every
combination of joint direction signs and reports which one turns the three
touched corners into the box you actually drew.

## Digital twin

A live 3-D view of what the model believes -- joint angles from the servo
ticks, the arm skeleton from the same forward kinematics that plans every
stroke, and the paper and canvas from the calibrated frame. No physics: if the
picture is wrong, the model is wrong, which is exactly what it is for.

```bash
doodle twin                              # poll the bus, serve on :8765 to the LAN
doodle twin --follow logs/<run>.csv      # replay a drawing log at its own speed
doodle --dry-run twin                    # simulated arm, no hardware
```

Runs on the Pi; open the printed `http://<pi-ip>:8765/` from any machine on
the network. Rendering is done by the browser (Three.js, vendored so it works
offline), the Pi only reads servos and streams JSON over server-sent events.

The serial bus has exactly one master. The port is opened with an exclusive
lock, so a second command fails immediately with `bus error: ... already open
in another process` instead of the two silently corrupting each other's
replies (which looked like `no reply from [1, 2, 5]` and pyserial's "multiple
access on port?"). To watch the arm while *another* command drives it, the
twin listens to a localhost telemetry tap instead: every command that reads
the servos broadcasts each read over UDP, and `doodle twin` falls back to that
automatically when the port is busy (`--live` forces it). `--follow` replays a
saved log; logs store ticks, converted with the *current* calibration, so a
log recorded before a recalibration will not replay where it was drawn.

## Drawing

```bash
doodle shapes                     # assets/{circle,square,star,spiral,grid}.{json,svg,png}
doodle preview star               # plan only -> assets/preview/*.png
doodle draw star                  # plan, draw, park (torque stays on)
doodle draw circle --speed 20 --repeat 2
doodle --dry-run draw spiral --fast   # whole pipeline on the fake bus
```

Ctrl-C during a draw lifts the pen and holds. `doodle torque off` releases
the arm (it will sag). `doodle goto park|lpose|u,v,z` and
`doodle jog <joint> <deg>` / `doodle jog x|y|z <mm>` move it deliberately.

Every draw writes `logs/<shape>_<timestamp>.csv` with commanded and actual
ticks per cycle for repeatability analysis.

## Low-level diagnostics

`tools/busdiag.py` predates the package and still works for raw bus poking:

```bash
python tools/busdiag.py scan
python tools/busdiag.py health 1
```
