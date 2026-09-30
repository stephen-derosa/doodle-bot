# Arm bring-up status

## Host

| | |
|---|---|
| Board | **Raspberry Pi 4 Model B Rev 1.5**, 7.6 GiB RAM, aarch64 |
| OS | Debian 13 (trixie), kernel 6.18.33-rpi |
| Hostname | rover |

Note: this is a Raspberry Pi, **not** a Jetson — there is no NVIDIA GPU and no
CUDA. `torch` is therefore installed as the CPU-only aarch64 build. On-device
policy *training* is not realistic here; teleoperation, data recording and
lightweight inference are.

## Serial link

USB-serial adapter enumerates fine:

- `1a86:55d3` QinHeng CH343 "USB Single Serial", serial `5B61034319`
- bound to the generic `cdc_acm` driver, appears as `/dev/ttyACM0`
- stable symlink: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61034319-if00`
- user `rover` is in `dialout`, so no permission work was needed
- kernel reports a clean `1000000` baud 8N1

## Arm configuration (2026-09-11)

The arm is a full SO-101 **minus the gripper**: the gripper assembly is
replaced by the pen holder in `models/simple_motor_to_pen.*` (15 mm bore for
a Sharpie fine tip, mounts on the wrist_roll horn). That leaves **five**
servos, IDs 1-5 in lerobot's numbering. lerobot's `SO101Follower` refuses to
connect without a sixth motor, so the `doodle` package talks to the bus
directly (`doodle/servo.py`) and `doodle assign-ids` replaces
`lerobot-setup-motors`.

## Status: the servos have not been assigned IDs yet

The five servos are daisy-chained but still at **factory defaults**, which for
the Feetech STS3215 means every one of them is **ID 1 at 1,000,000 baud**.
Re-checked 2026-09-11 with `doodle scan`: still only ID 1 answers.

That single fact explains everything observed on the bus:

- **Only "ID 1" ever answers.** There is no ID 2-5 to find — not because those
  servos are missing, but because they have never been renumbered.
- **Replies come back corrupted.** Addressing ID 1 makes all six servos
  transmit at once on a half-duplex bus. They are not clock-synchronised, so
  their frames overlap a few microseconds apart and garble each other. Only
  when they happen to align closely enough does a frame decode.
- **Host -> servo transmission is perfect** (every ping got *some* reply) while
  **servo -> host is corrupted** — exactly the asymmetry bus contention
  produces, since only the reply direction has six drivers fighting.
- **Lower bus speeds help** (first-attempt ping 31% at 1 Mbaud, 78% at 500k,
  89% at 57,600): the fixed inter-servo timing skew is a smaller fraction of a
  bit period at lower speed, so overlapping frames line up better.

Power is fine — 12.6 V, 33 °C read back from the bus. Firmware 3.10,
model 777 (STS3215).

There is **no evidence of damaged wiring**, and cables should not be replaced
on the strength of these measurements.

### What was ruled out

- **Power** — 12.6 V at the servo.
- **Permissions / driver / baud config** — kernel reports a clean 1000000 8N1,
  and the servo baud register agrees.
- **Half-duplex turnaround timing** — setting the return-delay register
  (addr 7) to 20, 100 and 250 changed nothing (32% -> 30% -> 29% -> 28%).

### An attempted test that did *not* discriminate

If contention were the cause, registers holding *identical* values across all
six servos might still decode cleanly while per-servo values never would.
Measured over 40 single-attempt reads at 1 Mbaud:

| register | identical across servos? | clean decodes |
|---|---|---|
| model | yes | 4/40 |
| id | yes | 8/40 |
| baud_index | yes | 8/40 |
| torque_enable | yes | 8/40 |
| max_angle_limit | yes | 9/40 |
| present_position | no | 0/40 |
| present_speed | no (≈0 when idle) | 7/40 |
| temperature | no | 1/40 |
| present_load | no (≈0 when idle) | 2/40 |

Directionally consistent (`present_position`, the most variable register, is
the only 0/40) but far too noisy to call proof: identical-value registers did
not decode cleanly either. Because the servos are not bit-synchronised, even
identical payloads collide. The ID-collision explanation rests on the servos
being unconfigured, not on this table.

## Definitive test, which is also the fix

Assigning IDs requires one servo on the bus at a time regardless, so the test
and the repair are the same operation. With only a single servo connected,
`python tools/busdiag.py health 1` should jump to ~100%. If it does **not**,
that servo or its cable really is faulty.

## Next step: assign the joint IDs

`doodle assign-ids` walks through the five joints, prompting you to connect
one servo at a time. Run it from a real terminal:

```bash
source .venv/bin/activate
doodle assign-ids
```

| ID | Joint |
|---|---|
| 1 | shoulder_pan |
| 2 | shoulder_lift |
| 3 | elbow_flex |
| 4 | wrist_flex |
| 5 | wrist_roll |

Afterwards `doodle scan` should list all five with none missing, and
`python tools/busdiag.py health 1` should read ~100% at the full 1 Mbaud.
Then follow the bring-up sequence in `README.md` / `docs/PLAN.md`.

## Left as found

The servos are at stock settings: 1,000,000 baud, return-delay 0. During
diagnosis the bus speed was temporarily lowered to 57,600 and the return-delay
register exercised; both were restored.

## Software verified working

- `lerobot` 0.3.2 — `SO101Follower` / `SO101FollowerConfig` / `FeetechMotorsBus`
  all import; the `lerobot-*` CLI entry points are on the venv PATH.
- `torch` 2.14.0+cpu — CPU matmul verified, 4 threads.
- `opencv` 5.0.0 (via `opencv-python-headless`).
- `ffmpeg` 7.1.3 with `libsvtav1`, `libx264` and `libx265` — a system package,
  already present, covering lerobot's default dataset video codec.
- Zero NVIDIA/CUDA packages are installed.
