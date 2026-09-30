import numpy as np
import pytest

from doodle.arm import Arm, SafetyError, open_arm
from doodle.config import Calibration, Config
from doodle.servo import (BAUD_INDEX, INSTR_SYNC_READ, FakeBus, FeetechBus, checksum,
                          decode_signbit, encode_signbit)


def test_checksum_and_signbit():
    # ping packet for id 1: FF FF 01 02 01 FB
    assert checksum(bytes([1, 2, 1])) == 0xFB
    assert decode_signbit(encode_signbit(-300)) == -300
    assert decode_signbit(encode_signbit(300)) == 300
    assert BAUD_INDEX[1_000_000] == 0


def test_fake_bus_sync_roundtrip():
    bus = FakeBus([1, 2, 3])
    bus.torque_all([1, 2, 3], True)
    bus.sync_write("goal_position", {1: 100, 2: 200, 3: 300})
    assert bus.sync_read("present_position", [1, 2, 3]) == {1: 100, 2: 200, 3: 300}
    bus.set_id(3, 5)
    assert bus.ping(5) and not bus.ping(3)


def test_arm_conversions_and_limits():
    cfg, cal = Config(), Calibration()
    cal.direction = [1, -1, 1, 1, 1]
    cal.zero_ticks = [2048, 2000, 2100, 2048, 2048]
    arm = open_arm(cfg, cal, dry_run=True)
    q = np.radians([10, -20, 30, 40, 0])
    assert np.allclose(arm.ticks_to_q(arm.q_to_ticks(q)), q)
    arm.check_q(q)
    with pytest.raises(SafetyError):
        arm.check_q(np.radians([0, 0, 0, 120, 0]))      # wrist beyond URDF limit
    with pytest.raises(SafetyError):
        arm.check_q(np.array([np.nan, 0, 0, 0, 0]))
    cfg.joints[0].max_ticks = 2100
    arm = open_arm(cfg, cal, dry_run=True)
    with pytest.raises(SafetyError):
        arm.check_q(np.radians([20, 0, 0, 0, 0]))         # inside rad limit, outside tick window


def test_move_to_and_torque_no_jump():
    cfg, cal = Config(), Calibration()
    arm = open_arm(cfg, cal, dry_run=True)
    start = arm.read_ticks()
    arm.torque(True)
    assert np.allclose(arm.read_ticks(), start)          # enabling torque must not move anything
    arm.move_to_q(np.radians([5, -90, 90, 5, 0]), speed_deg_s=1e6, rate_hz=1000)
    assert np.allclose(np.degrees(arm.read_q()), [5, -90, 90, 5, 0], atol=0.1)


class _BurstSerial:
    """Serial stub that answers a SYNC READ the way the hardware does.

    Every addressed servo replies back-to-back, so the whole burst is sitting
    in the buffer at once and a single `read()` returns all of it.
    """

    def __init__(self, positions):
        self.positions = positions
        self.out = bytearray()

    def write(self, pkt):
        if pkt[4] != INSTR_SYNC_READ:        # only SYNC READ is modelled
            return
        ln = pkt[6]
        for sid in pkt[7:-1]:
            body = bytes([sid, ln + 2, 0]) + int(self.positions[sid]).to_bytes(ln, "little")
            self.out += b"\xff\xff" + body + bytes([checksum(body)])

    def read(self, n):
        chunk, self.out = bytes(self.out[:n]), bytearray(self.out[n:])
        return chunk

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.out.clear()


def test_sync_read_parses_whole_reply_burst():
    """Every servo must be recovered from a single request.

    Regression: `sync_read` used to parse one servo per `read()`, so the first
    servo's parse swallowed the entire burst and discarded the rest. Each retry
    then recovered exactly one more servo, making `retries=3` on a five-servo
    arm fail with "no reply from [4, 5]" on a perfectly healthy bus.
    """
    positions = {1: 2094, 2: 2053, 3: 2006, 4: 2038, 5: 995}
    bus = FeetechBus("/dev/null")
    bus.ser = _BurstSerial(positions)

    assert bus.sync_read("present_position", [1, 2, 3, 4, 5], retries=1) == positions
    assert bus.stats["timeouts"] == 0
    assert bus.stats["tx"] == 1          # one request packet, not one per servo
