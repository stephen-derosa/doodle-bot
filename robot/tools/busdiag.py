#!/usr/bin/env python
"""Low-level Feetech STS bus diagnostics for the SO-101 arm.

Independent of lerobot: talks the raw Feetech protocol over the serial bus so
it still works when lerobot itself refuses to connect (e.g. missing motors).

Usage:
    python tools/busdiag.py scan            # find every servo on the bus
    python tools/busdiag.py info [ID]       # dump one servo's registers
    python tools/busdiag.py health [ID]     # measure link reliability
    python tools/busdiag.py setbaud ID RATE # change a servo's bus speed
"""
import argparse
import sys
import time

from feetech_raw import Bus

PORT = "/dev/ttyACM0"
BAUD_INDEX = {1_000_000: 0, 500_000: 1, 250_000: 2, 128_000: 3, 115_200: 4, 57_600: 6}
SO101_JOINTS = {
    1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
    4: "wrist_flex", 5: "wrist_roll", 6: "gripper",
}
REGS = [
    ("firmware_major", 0, 1), ("firmware_minor", 1, 1), ("model", 3, 2),
    ("id", 5, 1), ("baud_index", 6, 1), ("return_delay", 7, 1),
    ("min_angle_limit", 9, 2), ("max_angle_limit", 11, 2),
    ("torque_enable", 40, 1), ("goal_position", 42, 2),
    ("present_position", 56, 2), ("present_speed", 58, 2),
    ("present_load", 60, 2), ("voltage", 62, 1), ("temperature", 63, 1),
]


def open_bus(baud):
    """Open the bus, auto-detecting speed when none is given."""
    if baud:
        return Bus(PORT, baud), baud
    for cand in BAUD_INDEX:
        b = Bus(PORT, cand)
        if any(b.ping(i, tries=6) for i in range(1, 7)):
            return b, cand
        b.close()
    return None, None


def cmd_scan(args):
    b, baud = open_bus(args.baud)
    if b is None:
        print("No servo answered at any baud rate.")
        return 1
    print(f"bus speed: {baud} baud\n")
    found = []
    for sid in range(0, args.max_id + 1):
        if b.ping(sid, tries=args.retries):
            found.append(sid)
            print(f"  ID {sid:>3}  {SO101_JOINTS.get(sid, '(not an SO-101 joint id)')}")
    print(f"\n{len(found)} servo(s) found: {found}")
    missing = sorted(set(SO101_JOINTS) - set(found))
    if missing:
        print("MISSING SO-101 joints:")
        for m in missing:
            print(f"  ID {m}  {SO101_JOINTS[m]}")
    b.close()
    return 0


def cmd_info(args):
    b, baud = open_bus(args.baud)
    if b is None:
        print("No servo answered at any baud rate.")
        return 1
    print(f"bus speed: {baud} baud")
    print(f"=== servo ID {args.id} ({SO101_JOINTS.get(args.id, 'unknown joint')}) ===")
    if not b.ping(args.id, tries=10):
        print("  not responding")
        b.close()
        return 1
    for name, addr, ln in REGS:
        v = b.read(args.id, addr, ln, tries=10)
        if v is None:
            print(f"  {name:18s} <no valid reply>")
        elif name == "voltage":
            print(f"  {name:18s} {v / 10.0} V")
        elif name == "model":
            print(f"  {name:18s} {v}{' (STS3215)' if v == 777 else ''}")
        else:
            print(f"  {name:18s} {v}")
    b.close()
    return 0


def cmd_health(args):
    """Quantify link quality: a healthy bus answers ~100% of first attempts."""
    b, baud = open_bus(args.baud)
    if b is None:
        print("No servo answered at any baud rate.")
        return 1
    n = args.count
    pings = sum(1 for _ in range(n) if b.ping(args.id, tries=1))
    reads = sum(1 for _ in range(n) if b.read(args.id, 56, 2, tries=1) is not None)
    print(f"bus speed: {baud} baud, servo ID {args.id}, {n} attempts each")
    print(f"  ping           {pings:>4}/{n}  ({100 * pings // n}%)")
    print(f"  position read  {reads:>4}/{n}  ({100 * reads // n}%)")
    if pings < n or reads < n:
        print("\n  Packets are being lost or corrupted. A healthy bus is ~100%.")
        print("  Suspect wiring: cable, connector crimps, daisy-chain links.")
    b.close()
    return 0


def cmd_setbaud(args):
    if args.rate not in BAUD_INDEX:
        print(f"rate must be one of {sorted(BAUD_INDEX)}")
        return 1
    b, baud = open_bus(args.baud)
    if b is None:
        print("No servo answered at any baud rate.")
        return 1
    print(f"servo ID {args.id}: {baud} -> {args.rate} baud")
    b.write(args.id, 55, 0, 1)                        # unlock EEPROM
    b.write(args.id, 6, BAUD_INDEX[args.rate], 1)     # baud index
    b.close()
    time.sleep(0.4)
    nb = Bus(PORT, args.rate)
    if nb.ping(args.id, tries=15):
        nb.write(args.id, 55, 1, 1)                   # relock EEPROM
        print(f"  confirmed: servo answers at {args.rate} baud")
        nb.close()
        return 0
    nb.close()
    print("  servo did not answer at the new rate; run 'scan' to locate it")
    return 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--baud", type=int, help="force a bus speed instead of auto-detecting")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="find every servo on the bus")
    s.add_argument("--max-id", type=int, default=20)
    s.add_argument("--retries", type=int, default=8)
    s.set_defaults(func=cmd_scan)

    s = sub.add_parser("info", help="dump one servo's registers")
    s.add_argument("id", nargs="?", type=int, default=1)
    s.set_defaults(func=cmd_info)

    s = sub.add_parser("health", help="measure link reliability")
    s.add_argument("id", nargs="?", type=int, default=1)
    s.add_argument("--count", type=int, default=100)
    s.set_defaults(func=cmd_health)

    s = sub.add_parser("setbaud", help="change a servo's bus speed")
    s.add_argument("id", type=int)
    s.add_argument("rate", type=int)
    s.set_defaults(func=cmd_setbaud)

    args = p.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
