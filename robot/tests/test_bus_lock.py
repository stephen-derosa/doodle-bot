import json
import os
import pty
import socket
import time

import pytest
import serial

from doodle import cli, telemetry, twin
from doodle.arm import open_arm
from doodle.config import Calibration, Config
from doodle.kinematics import SO101Kinematics
from doodle.servo import BusBusy, FeetechBus, ServoError


def _pty_port():
    master, slave = pty.openpty()
    return master, slave, os.ttyname(slave)


def test_second_opener_of_the_same_port_is_refused():
    """Regression: `doodle twin` polling the bus while `calib travel` ran made
    both see each other's replies ('no reply from [1, 2, 5]', 'multiple access
    on port?'). The second opener must now fail immediately and say why."""
    master, slave, path = _pty_port()
    first = FeetechBus(path).open()
    try:
        with pytest.raises(BusBusy) as e:
            FeetechBus(path).open()
        # both openers are this process, so the message must blame us, not a phantom
        assert "this same process" in str(e.value)
        assert isinstance(e.value, ServoError)          # main() reports it as a bus error
    finally:
        first.close()
        os.close(master); os.close(slave)
    # and the port is usable again once the owner lets go
    master, slave, path = _pty_port()
    FeetechBus(path).open().close()
    os.close(master); os.close(slave)


def test_serial_io_failure_surfaces_as_a_servo_error_not_a_traceback():
    """A vanished adapter used to escape as SerialException from deep inside a read."""
    bus = FeetechBus("/dev/null")

    class DeadSerial:
        def reset_input_buffer(self): pass
        def write(self, b): return len(b)
        def flush(self): pass
        def read(self, n):
            raise serial.SerialException("device reports readiness to read but returned no data")

    bus.ser = DeadSerial()
    with pytest.raises(ServoError) as e:
        bus.sync_read("present_position", [1, 2, 3])
    assert "USB adapter dropped" in str(e.value) and "another process" in str(e.value)
    assert not isinstance(e.value, serial.SerialException)


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    return port


def test_every_tick_read_is_broadcast_on_the_tap():
    port = _free_udp_port()
    lis = telemetry.Listener(port, timeout=2.0)
    cfg, cal = Config(), Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    arm = open_arm(cfg, cal, dry_run=True)
    arm.tap = telemetry.Tap(port)
    for i, t in zip(cfg.ids, [2100, 2000, 2200, 1900, 2048]):
        arm.bus.mem[i]["present_position"] = t
    try:
        ticks = arm.read_ticks()
        msg = lis.recv()
    finally:
        lis.close(); arm.tap.close()
    assert msg is not None and msg["ticks"] == ticks.tolist() == [2100.0, 2000.0, 2200.0, 1900.0, 2048.0]
    assert arm.tap.sent == 1


def test_tap_send_is_harmless_with_no_listener():
    tap = telemetry.Tap(_free_udp_port())
    for _ in range(50):
        tap.send([1, 2, 3, 4, 5])          # kernel drops these; must never raise or block
    tap.close()


def test_twin_live_source_renders_what_the_bus_owner_reads():
    port = _free_udp_port()
    cfg, cal = Config(), Calibration(zero_ticks=[2048.0] * 5, direction=[1] * 5, joints_calibrated=True)
    fake = open_arm(cfg, cal, dry_run=True)
    kin = SO101Kinematics.from_config(cfg)
    store = twin.StateStore()
    src = twin.TapSource(store, fake, kin, cfg, cal, port=port)
    src.start()
    time.sleep(0.2)                                    # listener bound, "waiting" frame published
    seq0, payload0 = store.latest()
    assert json.loads(payload0)["error"].startswith("waiting for telemetry")

    owner = open_arm(cfg, cal, dry_run=True)           # stands in for the command that owns the bus
    owner.tap = telemetry.Tap(port)
    for i, t in zip(cfg.ids, [2148, 2000, 2100, 2048, 2048]):
        owner.bus.mem[i]["present_position"] = t
    owner.read_ticks()
    deadline = time.time() + 3
    while time.time() < deadline and store.latest()[0] == seq0:
        time.sleep(0.02)
    src.stop.set(); src.join(timeout=2); owner.tap.close()
    frame = json.loads(store.latest()[1])
    assert frame["source"] == "live" and frame["error"] is None
    assert [j["ticks"] for j in frame["joints"]] == [2148.0, 2000.0, 2100.0, 2048.0, 2048.0]


def test_main_reports_a_busy_port_as_a_bus_error(monkeypatch, tmp_path, capsys):
    def busy(*a, **k):
        raise BusBusy("/dev/ttyACM0 is already open in another process")

    monkeypatch.setattr(cli, "open_arm", busy)
    cfg_path, cal_path = tmp_path / "d.yaml", tmp_path / "c.yaml"
    cli.save_config(Config(), cfg_path); cli.save_calibration(Calibration(), cal_path)
    rc = cli.main(["--config", str(cfg_path), "--calib", str(cal_path), "status"])
    err = capsys.readouterr().err
    assert rc == 2
    assert "bus error" in err and "another process" in err


def test_a_holder_that_took_no_lock_is_still_detected_and_named():
    """Regression for the real incident: the running `doodle twin` predated the
    lock, so pyserial's advisory flock let a second command open the port and
    corrupt the bus. The /proc scan catches a holder regardless of its code."""
    from doodle.servo import other_openers
    master, slave, path = _pty_port()
    raw = os.open(path, os.O_RDWR | os.O_NOCTTY)       # an opener that knows nothing about locks
    os.set_inheritable(raw, True)                       # os.open is close-on-exec by default (PEP 446)
    try:
        # our own process holds it, and other_openers() excludes ourselves; so exercise the
        # scan by asking about a device we hold from a child process instead
        pid = os.fork()
        if pid == 0:                                    # child: hold the port, then wait to be killed
            os.execv("/bin/sleep", ["sleep", "30"])
        time.sleep(0.2)
        holders = other_openers(path)
        assert any(p == pid for p, _ in holders), holders
        assert "sleep" in dict(holders)[pid]
        with pytest.raises(BusBusy) as e:
            FeetechBus(path).open()
        assert f"pid {pid}" in str(e.value) and "sleep" in str(e.value)
    finally:
        try:
            os.kill(pid, 9); os.waitpid(pid, 0)
        except (OSError, NameError):
            pass
        os.close(raw); os.close(master); os.close(slave)
    # nobody left holding it -> opens fine
    master, slave, path = _pty_port()
    FeetechBus(path).open().close()
    os.close(master); os.close(slave)


def test_opening_the_port_twice_in_one_process_names_this_process():
    """Regression: `calib travel` reopened the bus to display limits and then told
    the user another process held the port, when no such process existed."""
    master, slave, path = _pty_port()
    first = FeetechBus(path).open()
    try:
        with pytest.raises(BusBusy) as e:
            FeetechBus(path).open()
        assert "this same process" in str(e.value) and "another process" not in str(e.value)
    finally:
        first.close(); os.close(master); os.close(slave)
