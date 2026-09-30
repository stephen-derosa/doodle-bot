import json
import threading
import time
import urllib.request
from http.client import HTTPConnection

import numpy as np

from doodle import twin
from doodle.arm import open_arm
from doodle.config import Calibration, Config, PaperFrame
from doodle.kinematics import SO101Kinematics


def _rig():
    cfg, cal = Config(), Calibration()
    cal.direction = [1, 1, 1, 1, 1]
    cal.zero_ticks = [2048.0] * 5
    cal.joints_calibrated = True
    cal.paper = PaperFrame(origin=[70.0, cfg.paper.width / 2, -5.0], x_axis=[0, -1, 0], y_axis=[1, 0, 0],
                           calibrated=True)
    arm = open_arm(cfg, cal, dry_run=True)
    kin = SO101Kinematics.from_config(cfg)
    return cfg, cal, arm, kin


# --- state ---------------------------------------------------------------------------------------
def test_chain_ends_at_fk_tip_and_starts_at_the_base():
    """The skeleton must agree with the FK every stroke is planned with."""
    cfg, cal, arm, kin = _rig()
    q = np.radians([20.0, -10.0, 30.0, 40.0, 5.0])
    chain = twin.chain_points(kin, q)
    assert chain[0] == [0.0, 0.0, 0.0]
    assert np.allclose(chain[1], [0, 0, cfg.geometry.lift_z])
    assert np.allclose(chain[-1], kin.fk(q))
    assert len(chain) == 7
    # every point lies in the pan plane: atan2(y, x) == pan for all off-axis points
    for p in chain[2:]:
        r = np.hypot(p[0], p[1])
        if r > 1e-6:
            assert abs(np.arctan2(p[1], p[0]) - q[0]) < 1e-9


def test_twin_state_is_json_and_reflects_ticks_paper_and_canvas():
    cfg, cal, arm, kin = _rig()
    ticks = np.array([2148.0, 2000.0, 2100.0, 2048.0, 2048.0])
    s = twin.twin_state(arm, kin, cfg, cal, ticks, alive={n: True for n in arm.names}, source="bus")
    json.dumps(s)                                                    # plain data only
    assert [j["ticks"] for j in s["joints"]] == ticks.tolist()
    assert np.allclose([j["deg"] for j in s["joints"]], np.degrees(arm.ticks_to_q(ticks)))
    assert all(j["alive"] for j in s["joints"])
    # paper corners are the configured sheet placed by the calibrated frame
    P = cfg.paper
    assert np.allclose(s["paper"]["corners"][1], cal.paper.to_world(P.width, 0))
    assert np.allclose(s["paper"]["corners"][3], cal.paper.to_world(0, P.height))
    u0, v0 = P.canvas_origin()
    assert np.allclose(s["canvas"]["corners"][0], cal.paper.to_world(u0, v0))
    assert s["paper"]["calibrated"] is True
    assert s["tip"] == s["chain"][-1]


def test_pen_down_is_inferred_from_height_only_when_paper_is_calibrated():
    cfg, cal, arm, kin = _rig()
    # find a pose whose tip is on the paper plane, and one well above it
    on = kin.ik(cal.paper.to_world(60.0, 60.0)).q
    up = kin.ik(cal.paper.to_world(60.0, 60.0, 30.0)).q
    s_on = twin.twin_state(arm, kin, cfg, cal, arm.q_to_ticks(on))
    s_up = twin.twin_state(arm, kin, cfg, cal, arm.q_to_ticks(up))
    assert s_on["pen_down"] is True and s_up["pen_down"] is False
    cal.paper.calibrated = False
    assert twin.twin_state(arm, kin, cfg, cal, arm.q_to_ticks(on))["pen_down"] is None
    # an explicit value (from a log) wins over the heuristic
    assert twin.twin_state(arm, kin, cfg, cal, arm.q_to_ticks(up), pen_down=True)["pen_down"] is True


# --- log follower --------------------------------------------------------------------------------
def test_log_follower_prefers_actual_ticks_and_falls_back_to_commanded(tmp_path):
    cfg, cal, arm, kin = _rig()
    names = arm.names
    log = tmp_path / "run.csv"
    with open(log, "w") as f:
        f.write("t,pen_down," + ",".join(f"cmd_{n}" for n in names) + "," + ",".join(f"act_{n}" for n in names) + ",x,y,z\n")
        f.write("0.000,0," + ",".join(["2000"] * 5) + "," + ",".join(["2010"] * 5) + ",0,0,0\n")   # has act
        f.write("0.020,1," + ",".join(["2100"] * 5) + "," + ",".join([""] * 5) + ",0,0,0\n")       # cmd only
    # StateStore is latest-wins on purpose (a viewer wants the newest frame),
    # so record every publish rather than polling and racing the follower.
    frames = []

    class Recording(twin.StateStore):
        def publish(self, state):
            frames.append(json.loads(json.dumps(state)))
            super().publish(state)

    fol = twin.LogFollower(Recording(), arm, kin, cfg, cal, log, pace=False)
    fol.start()
    deadline = time.time() + 3
    while time.time() < deadline and len(frames) < 2:
        time.sleep(0.02)
    fol.stop.set()
    assert len(frames) == 2
    assert [j["ticks"] for j in frames[0]["joints"]] == [2010.0] * 5 and frames[0]["pen_down"] is False
    assert [j["ticks"] for j in frames[1]["joints"]] == [2100.0] * 5 and frames[1]["pen_down"] is True
    assert frames[1]["source"].startswith("log:") and frames[1]["t_rel"] == 0.02


# --- http ----------------------------------------------------------------------------------------
def _serve(store):
    server = twin.serve(store, "127.0.0.1", 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, server.server_address[1]


def test_server_serves_page_script_and_state():
    cfg, cal, arm, kin = _rig()
    store = twin.StateStore()
    store.publish(twin.twin_state(arm, kin, cfg, cal, [2048.0] * 5))
    server, port = _serve(store)
    try:
        page = urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5).read().decode()
        assert "<title>doodle twin</title>" in page and "/events" in page and "/three.min.js" in page
        js = urllib.request.urlopen(f"http://127.0.0.1:{port}/three.min.js", timeout=5).read()
        assert len(js) > 100_000 and b"three" in js.lower()[:400]
        state = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/state", timeout=5).read())
        assert state["seq"] == 1 and len(state["chain"]) == 7
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/nope", timeout=5)
        except urllib.error.HTTPError as e:
            assert e.code == 404
        else:
            raise AssertionError("expected 404")
    finally:
        server.shutdown(); server.server_close()


def test_events_stream_pushes_each_published_frame():
    cfg, cal, arm, kin = _rig()
    store = twin.StateStore()
    store.publish(twin.twin_state(arm, kin, cfg, cal, [2048.0] * 5))
    server, port = _serve(store)
    try:
        conn = HTTPConnection("127.0.0.1", port, timeout=5)
        conn.request("GET", "/events")
        resp = conn.getresponse()
        assert resp.status == 200 and resp.getheader("Content-Type").startswith("text/event-stream")

        def read_event():
            while True:
                line = resp.fp.readline()
                if line.startswith(b"data: "):
                    return json.loads(line[6:])

        first = read_event()
        assert first["seq"] == 1
        store.publish(twin.twin_state(arm, kin, cfg, cal, [2148.0] * 5))
        second = read_event()
        assert second["seq"] == 2 and second["joints"][0]["ticks"] == 2148.0
        conn.close()
    finally:
        server.shutdown(); server.server_close()


def test_bus_sampler_publishes_frames_and_demo_moves_the_arm():
    cfg, cal, arm, kin = _rig()
    store = twin.StateStore()
    s = twin.BusSampler(store, arm, kin, cfg, cal, rate=50.0, demo=True)
    s.start()
    time.sleep(0.4)
    s.stop.set(); s.join(timeout=2)
    seq, payload = store.latest()
    frame = json.loads(payload)
    assert seq >= 5 and frame["source"] == "demo" and frame["error"] is None
    # demo motion stays inside the limits it was given
    for j in frame["joints"]:
        assert j["min_deg"] - 1e-6 <= j["deg"] <= j["max_deg"] + 1e-6
