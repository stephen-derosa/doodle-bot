"""Raw Feetech STS3215 bus driver.

Deliberately independent of lerobot: lerobot's SO101Follower refuses to connect
unless all six motors answer, and this arm has five (the gripper motor is
replaced by a passive pen holder). Everything here is plain pyserial and the
documented Feetech half-duplex packet protocol.

Packet:  FF FF ID LEN INSTR PARAMS... CHK      CHK = ~(ID+LEN+INSTR+PARAMS) & 0xFF
Status:  FF FF ID LEN ERR   PARAMS... CHK
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass

import serial

# --- control table (STS3215) -------------------------------------------------
# name: (address, length)
REG = {
    "firmware_major": (0, 1), "firmware_minor": (1, 1), "model": (3, 2),
    "id": (5, 1), "baud_index": (6, 1), "return_delay": (7, 1),
    "response_level": (8, 1),
    "min_position_limit": (9, 2), "max_position_limit": (11, 2),
    "max_temperature": (13, 1), "max_voltage": (14, 1), "min_voltage": (15, 1),
    "max_torque": (16, 2), "phase": (18, 1),
    "p_gain": (21, 1), "d_gain": (22, 1), "i_gain": (23, 1),
    "min_startup_force": (24, 2), "cw_dead_zone": (26, 1), "ccw_dead_zone": (27, 1),
    "protection_current": (28, 2), "angular_resolution": (30, 1),
    "homing_offset": (31, 2), "operating_mode": (33, 1),
    "protective_torque": (34, 1), "protection_time": (35, 1), "overload_torque": (36, 1),
    "torque_enable": (40, 1), "acceleration": (41, 1),
    "goal_position": (42, 2), "goal_time": (44, 2), "goal_velocity": (46, 2),
    "torque_limit": (48, 2), "lock": (55, 1),
    "present_position": (56, 2), "present_velocity": (58, 2), "present_load": (60, 2),
    "present_voltage": (62, 1), "present_temperature": (63, 1),
    "status": (65, 1), "moving": (66, 1), "present_current": (69, 2),
}
BAUD_INDEX = {1_000_000: 0, 500_000: 1, 250_000: 2, 128_000: 3,
              115_200: 4, 57_600: 5, 38_400: 6, 19_200: 7}
TICKS_PER_REV = 4096
MODEL_STS3215 = 777
BROADCAST_ID = 0xFE

INSTR_PING, INSTR_READ, INSTR_WRITE, INSTR_SYNC_READ, INSTR_SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
# Some registers are "signed" via bit 15 (magnitude + sign bit), not two's complement.
SIGN_BIT_REGS = {"present_velocity", "present_load", "goal_velocity", "homing_offset", "present_current"}


class ServoError(RuntimeError):
    pass


class ServoTimeout(ServoError):
    pass


class BusBusy(ServoError):
    """The serial port is held by another process."""


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def decode_signbit(v: int, nbits: int = 15) -> int:
    return -(v & ((1 << nbits) - 1)) if v & (1 << nbits) else v


def encode_signbit(v: int, nbits: int = 15) -> int:
    return (abs(v) & ((1 << nbits) - 1)) | ((1 << nbits) if v < 0 else 0)


@dataclass
class Status:
    sid: int
    error: int
    params: bytes


def other_openers(port: str) -> list[tuple[int, str]]:
    """Other processes with `port` open, as (pid, command line). Linux /proc only.

    Same-user processes are visible; anything we cannot inspect is skipped.
    """
    try:
        dev = os.stat(port).st_rdev
    except OSError:
        return []
    me = os.getpid()
    found = []
    try:
        pids = [int(d) for d in os.listdir("/proc") if d.isdigit()]
    except OSError:
        return []
    for pid in pids:
        if pid == me:
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                try:
                    if os.stat(f"/proc/{pid}/fd/{fd}").st_rdev == dev:
                        with open(f"/proc/{pid}/cmdline", "rb") as f:
                            cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                        found.append((pid, cmd[:80] or "?"))
                        break
                except OSError:
                    continue
        except OSError:
            continue
    return found


class FeetechBus:
    """One serial port with any number of STS servos daisy-chained on it."""

    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.05, retries: int = 3):
        self.port, self.baud, self.timeout, self.retries = port, baud, timeout, retries
        self.ser: serial.Serial | None = None
        self.stats = {"tx": 0, "rx_ok": 0, "rx_bad": 0, "timeouts": 0}

    # -- lifecycle ----------------------------------------------------------
    def open(self) -> "FeetechBus":
        # One master per bus. A second process on the same half-duplex line
        # interleaves its requests with ours and each side reads the other's
        # replies, which shows up as scattered "no reply from [1, 2, 5]" drops
        # and pyserial's "multiple access on port?" exception. Failing here,
        # loudly and naming the culprit, is far better. Three layers, because
        # each alone has a hole:
        #   * /proc scan   - catches a holder that took no lock at all (an older
        #                    build, minicom, a lerobot script);
        #   * flock        - pyserial's advisory lock between cooperating openers;
        #   * TIOCEXCL     - kernel refuses any later open() while we hold it.
        others = other_openers(self.port)
        if others:
            who = "; ".join(f"pid {pid} ({cmd})" for pid, cmd in others)
            raise BusBusy(f"{self.port} is already held by {who}. Only one process may drive the bus: "
                          f"stop it, or watch it instead with `doodle twin --live`.")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.5, exclusive=True)
        except serial.SerialException as e:
            if any(w in str(e).lower() for w in ("exclusive", "busy", "lock")):
                # The /proc scan above found nobody else, so the lock is ours: this
                # process already has the port open. Say so, rather than sending the
                # user hunting for a phantom second process.
                raise BusBusy(f"{self.port} is already open in this same process (opened twice in one "
                              f"command, which is a bug in that command -- please report it).") from e
            raise ServoError(f"cannot open {self.port}: {e}") from e
        try:
            import fcntl
            import termios
            fcntl.ioctl(self.ser.fileno(), termios.TIOCEXCL)
        except (ImportError, OSError, AttributeError):
            pass  # not a real tty (tests use ptys/pipes) or not Linux; the other layers still apply
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        return self

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # -- packet plumbing ----------------------------------------------------
    def _send(self, sid: int, instr: int, params: bytes = b"") -> None:
        assert self.ser is not None, "bus not open"
        body = bytes([sid, len(params) + 2, instr]) + params
        pkt = b"\xff\xff" + body + bytes([checksum(body)])
        try:
            self.ser.reset_input_buffer()
            self.ser.write(pkt)
            self.ser.flush()
        except (serial.SerialException, OSError) as e:
            raise ServoError(self._io_error(e)) from e
        self.stats["tx"] += 1

    def _read(self, n: int) -> bytes:
        """Serial read that turns a vanished device into a ServoError, not a traceback."""
        try:
            return self.ser.read(n)
        except (serial.SerialException, OSError) as e:
            raise ServoError(self._io_error(e)) from e

    def _io_error(self, e: Exception) -> str:
        return (f"serial I/O failed on {self.port}: {e}. Either the USB adapter dropped "
                f"(check `dmesg`), or another process is using the port.")

    def _recv(self, sid: int, nparams: int, timeout: float | None = None) -> Status:
        """Wait for one well-formed status packet from `sid`."""
        assert self.ser is not None
        need = 6 + nparams
        buf = bytearray()
        deadline = time.monotonic() + (timeout or self.timeout)
        while time.monotonic() < deadline:
            chunk = self._read(128)
            if chunk:
                buf += chunk
                for i in range(len(buf) - need + 1):
                    if (buf[i] == 0xFF and buf[i + 1] == 0xFF and buf[i + 2] == sid
                            and buf[i + 3] == nparams + 2):
                        frame = bytes(buf[i:i + need])
                        if checksum(frame[2:-1]) == frame[-1]:
                            self.stats["rx_ok"] += 1
                            return Status(sid, frame[4], frame[5:-1])
                        self.stats["rx_bad"] += 1
            else:
                time.sleep(0.0005)
        self.stats["timeouts"] += 1
        raise ServoTimeout(f"servo {sid}: no valid reply ({len(buf)} bytes seen)")

    def _recv_many(self, sids: list[int], nparams: int,
                   timeout: float | None = None) -> dict[int, Status]:
        """Collect status packets from several servos out of one reply burst.

        A SYNC READ makes every addressed servo answer back-to-back, so all the
        replies land in the receive buffer together -- often inside a single
        `read()`. They therefore have to be parsed out of one accumulating
        buffer; calling `_recv` per servo would let the first call swallow the
        whole burst and discard every reply but its own.
        """
        assert self.ser is not None
        need = 6 + nparams
        want = set(sids)
        found: dict[int, Status] = {}
        buf = bytearray()
        deadline = time.monotonic() + (timeout or self.timeout)
        while want and time.monotonic() < deadline:
            chunk = self._read(128)
            if not chunk:
                time.sleep(0.0005)
                continue
            buf += chunk
            i = 0
            while i + need <= len(buf):
                if (buf[i] == 0xFF and buf[i + 1] == 0xFF and buf[i + 2] in want
                        and buf[i + 3] == nparams + 2):
                    frame = bytes(buf[i:i + need])
                    if checksum(frame[2:-1]) == frame[-1]:
                        found[frame[2]] = Status(frame[2], frame[4], frame[5:-1])
                        want.discard(frame[2])
                        self.stats["rx_ok"] += 1
                        i += need
                        continue
                    self.stats["rx_bad"] += 1
                i += 1
            del buf[:max(0, len(buf) - need + 1)]
        self.stats["timeouts"] += len(want)
        return found

    def _txn(self, sid: int, instr: int, params: bytes, nparams: int, retries: int | None = None) -> Status:
        last: Exception | None = None
        for _ in range(retries if retries is not None else self.retries):
            self._send(sid, instr, params)
            try:
                return self._recv(sid, nparams)
            except ServoTimeout as e:
                last = e
        raise last  # type: ignore[misc]

    # -- primitives -----------------------------------------------------------
    def ping(self, sid: int, retries: int | None = None) -> bool:
        try:
            self._txn(sid, INSTR_PING, b"", 0, retries)
            return True
        except ServoTimeout:
            return False

    def read_raw(self, sid: int, addr: int, length: int, retries: int | None = None) -> int:
        st = self._txn(sid, INSTR_READ, bytes([addr, length]), length, retries)
        return int.from_bytes(st.params, "little")

    def write_raw(self, sid: int, addr: int, value: int, length: int, retries: int | None = None) -> None:
        params = bytes([addr]) + int(value).to_bytes(length, "little")
        if sid == BROADCAST_ID:
            self._send(sid, INSTR_WRITE, params)
            return
        self._txn(sid, INSTR_WRITE, params, 0, retries)

    def read(self, sid: int, reg: str, **kw) -> int:
        addr, ln = REG[reg]
        v = self.read_raw(sid, addr, ln, **kw)
        return decode_signbit(v) if reg in SIGN_BIT_REGS else v

    def write(self, sid: int, reg: str, value: int, **kw) -> None:
        addr, ln = REG[reg]
        if reg in SIGN_BIT_REGS:
            value = encode_signbit(int(value))
        self.write_raw(sid, addr, int(value), ln, **kw)

    def sync_write(self, reg: str, values: dict[int, int]) -> None:
        """Write the same register on many servos in one packet (no replies)."""
        addr, ln = REG[reg]
        params = bytearray([addr, ln])
        for sid, v in values.items():
            if reg in SIGN_BIT_REGS:
                v = encode_signbit(int(v))
            params += bytes([sid]) + int(v).to_bytes(ln, "little")
        self._send(BROADCAST_ID, INSTR_SYNC_WRITE, bytes(params))

    def sync_write_regs(self, values: dict[int, dict[str, int]]) -> None:
        """Write a *contiguous* register block (e.g. acceleration..goal_velocity) per servo."""
        # Normalise: every servo must provide the same register names.
        names = list(next(iter(values.values())).keys())
        addrs = [REG[n][0] for n in names]
        lens = [REG[n][1] for n in names]
        start = addrs[0]
        for a, l_, nxt in zip(addrs, lens, addrs[1:] + [None]):
            if nxt is not None and a + l_ != nxt:
                raise ValueError(f"registers {names} are not contiguous")
        total = sum(lens)
        params = bytearray([start, total])
        for sid, regs in values.items():
            params += bytes([sid])
            for n in names:
                v = int(regs[n])
                if n in SIGN_BIT_REGS:
                    v = encode_signbit(v)
                params += v.to_bytes(REG[n][1], "little")
        self._send(BROADCAST_ID, INSTR_SYNC_WRITE, bytes(params))

    def sync_read(self, reg: str, sids: list[int], retries: int | None = None) -> dict[int, int]:
        """Read one register from many servos with a single request packet."""
        addr, ln = REG[reg]
        out: dict[int, int] = {}
        for _ in range(retries if retries is not None else self.retries):
            missing = [s for s in sids if s not in out]
            if not missing:
                break
            self._send(BROADCAST_ID, INSTR_SYNC_READ, bytes([addr, ln]) + bytes(missing))
            for sid, st in self._recv_many(missing, ln).items():
                v = int.from_bytes(st.params, "little")
                out[sid] = decode_signbit(v) if reg in SIGN_BIT_REGS else v
        if len(out) != len(sids):
            raise ServoTimeout(f"sync_read {reg}: no reply from {sorted(set(sids) - set(out))}")
        return out

    # -- conveniences -------------------------------------------------------------
    def scan(self, ids=range(0, 21), retries: int = 3) -> list[int]:
        return [i for i in ids if self.ping(i, retries)]

    def torque(self, sid: int, on: bool) -> None:
        self.write(sid, "torque_enable", 1 if on else 0)

    def torque_all(self, sids: list[int], on: bool) -> None:
        self.sync_write("torque_enable", {s: 1 if on else 0 for s in sids})

    def eeprom_unlock(self, sid: int) -> None:
        self.write(sid, "lock", 0)

    def eeprom_lock(self, sid: int) -> None:
        self.write(sid, "lock", 1)

    def set_id(self, old: int, new: int) -> None:
        """Renumber a servo. Only valid with ONE servo on the bus if old == 1."""
        if not 0 <= new <= 253:
            raise ValueError("id must be 0..253")
        self.eeprom_unlock(old)
        self.write(old, "id", new)
        time.sleep(0.05)
        if not self.ping(new, retries=5):
            raise ServoError(f"servo did not answer at new id {new}")
        self.eeprom_lock(new)

    def set_baud(self, sid: int, baud: int) -> None:
        self.eeprom_unlock(sid)
        self.write(sid, "baud_index", BAUD_INDEX[baud])
        # lock must be re-applied after reopening at the new speed by the caller

    def info(self, sid: int) -> dict[str, int | float]:
        keys = ["firmware_major", "firmware_minor", "model", "id", "baud_index", "return_delay",
                "min_position_limit", "max_position_limit", "p_gain", "d_gain", "i_gain",
                "homing_offset", "operating_mode", "torque_enable", "acceleration",
                "goal_position", "goal_velocity", "torque_limit", "present_position",
                "present_velocity", "present_load", "present_voltage", "present_temperature", "moving"]
        d: dict[str, int | float] = {k: self.read(sid, k) for k in keys}
        d["present_voltage"] = d["present_voltage"] / 10.0
        return d


class FakeBus(FeetechBus):
    """In-memory stand-in for tests and --dry-run. Servos track goals instantly."""

    def __init__(self, ids: list[int], start_ticks: int = 2048):
        super().__init__(port="fake", baud=0)
        self.mem: dict[int, dict[str, int]] = {}
        for i in ids:
            self.mem[i] = {k: 0 for k in REG}
            self.mem[i].update(id=i, model=MODEL_STS3215, firmware_major=3, firmware_minor=10,
                               present_position=start_ticks, goal_position=start_ticks,
                               present_voltage=124, present_temperature=33, p_gain=32, d_gain=32,
                               max_position_limit=4095, torque_limit=1000, lock=1)

    def open(self):
        return self

    def close(self):
        pass

    def ping(self, sid, retries=None):
        return sid in self.mem

    def read(self, sid, reg, **kw):
        if sid not in self.mem:
            raise ServoTimeout(f"servo {sid}: not present (fake)")
        return self.mem[sid][reg]

    def write(self, sid, reg, value, **kw):
        if sid == BROADCAST_ID:
            for m in self.mem.values():
                m[reg] = int(value)
            return
        if sid not in self.mem:
            raise ServoTimeout(f"servo {sid}: not present (fake)")
        self.mem[sid][reg] = int(value)
        if reg == "goal_position" and self.mem[sid]["torque_enable"]:
            self.mem[sid]["present_position"] = int(value)
        if reg == "id":
            self.mem[int(value)] = self.mem.pop(sid)

    def read_raw(self, sid, addr, length, retries=None):
        for k, (a, l_) in REG.items():
            if a == addr and l_ == length:
                return self.read(sid, k)
        raise KeyError(addr)

    def write_raw(self, sid, addr, value, length, retries=None):
        for k, (a, l_) in REG.items():
            if a == addr and l_ == length:
                return self.write(sid, k, value)
        raise KeyError(addr)

    def sync_write(self, reg, values):
        for sid, v in values.items():
            self.write(sid, reg, v)

    def sync_write_regs(self, values):
        for sid, regs in values.items():
            for k, v in regs.items():
                self.write(sid, k, v)

    def sync_read(self, reg, sids, retries=None):
        return {s: self.read(s, reg) for s in sids}
