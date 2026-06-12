"""Homing REST endpoints over a mock CAN backend (never touches hardware).

The app normally loads the repo config.yaml (which may be a real socketcan
backend). These tests monkeypatch CONFIG_PATH to a temp mock config so the
TestClient drives an in-process MockBus, not the robot."""
import textwrap
import time

import pytest
from fastapi.testclient import TestClient


@pytest.fixture
def client(tmp_path, monkeypatch):
    import backend.api as api
    cfg = tmp_path / "config.yaml"
    cfg.write_text(textwrap.dedent("""
        can: { backend: mock }
        require_home_before_move: true
        axes:
          - { can_id: 1, name: J1, gear_ratio: 1.0, pulses_per_rev: 3200, soft_limit_min: -180, soft_limit_max: 180 }
          - { can_id: 2, name: J2, gear_ratio: 1.0, pulses_per_rev: 3200, soft_limit_min: -180, soft_limit_max: 180 }
    """))
    monkeypatch.setattr(api, "CONFIG_PATH", cfg)
    with TestClient(api.app) as c:
        yield c


def _wait_homed(client, joint, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if client.get("/api/state").json()["axes"][joint]["is_homed"]:
            return True
        time.sleep(0.02)
    return False


def test_move_blocked_until_homed(client):
    r = client.post("/api/move", json={"can_id": 1, "degrees": 10.0, "speed_pct": 0.5})
    assert r.status_code == 409

    assert client.post("/api/home", json={"can_id": 1}).status_code == 200
    assert _wait_homed(client, "J1")

    r = client.post("/api/move", json={"can_id": 1, "degrees": 10.0, "speed_pct": 0.5})
    assert r.status_code == 200


def test_move_accepts_duration_field_in_direct_mode(client):
    # The ROS-only duration_s field is accepted (and ignored) on the direct path.
    assert client.post("/api/home", json={"can_id": 1}).status_code == 200
    assert _wait_homed(client, "J1")
    r = client.post("/api/move", json={"can_id": 1, "degrees": 10.0, "duration_s": 5.0})
    assert r.status_code == 200


class _FakeRos:
    """Minimal stand-in for the in-process ROS client (no rclpy needed)."""
    def __init__(self):
        self.moves = []

    def homed(self):
        return {"J1": True, "J2": True}

    def move_to_joints(self, joints_deg, duration_s=3.0):
        self.moves.append((joints_deg, duration_s))
        return {"accepted": True, "error_code": 0}

    def close(self):  # called by the app lifespan teardown
        pass


def test_move_routes_to_ros_in_ros_mode(client, monkeypatch):
    # With a ROS client present, an absolute move becomes a single-joint trajectory
    # goal to the bridge instead of touching the (dry-run) local Motion.
    import backend.api as api
    fake = _FakeRos()
    monkeypatch.setattr(api.state, "ros", fake)
    r = client.post("/api/move", json={"can_id": 1, "degrees": 25.0})
    assert r.status_code == 200
    assert r.json()["via"] == "ros"
    assert fake.moves == [({"J1": 25.0}, 1.0)]  # name resolved, default duration_s


def test_move_blocked_until_homed_in_ros_mode(client, monkeypatch):
    import backend.api as api

    class _Unhomed(_FakeRos):
        def homed(self):
            return {"J1": False, "J2": False}

    monkeypatch.setattr(api.state, "ros", _Unhomed())
    r = client.post("/api/move", json={"can_id": 1, "degrees": 25.0})
    assert r.status_code == 409  # _require_ros_homed gate


def test_home_all_endpoint(client):
    assert client.post("/api/home/all").status_code == 200


def test_state_exposes_homing_fields(client):
    j1 = client.get("/api/state").json()["axes"]["J1"]
    assert "is_homed" in j1 and "homing" in j1 and "home_enabled" in j1


def test_io_read_endpoint(client):
    r = client.post("/api/io", json={"can_id": 1})
    assert r.status_code == 200
    io = r.json()["io"]
    assert "in_1" in io and "home_switch" in io


def test_home_dir_sets_config(client):
    assert client.post("/api/home/dir", json={"can_id": 1, "ccw": True}).status_code == 200
    assert client.get("/api/state").json()["axes"]["J1"]["home_dir"] == 1
    assert client.post("/api/home/dir", json={"can_id": 1, "ccw": False}).status_code == 200
    assert client.get("/api/state").json()["axes"]["J1"]["home_dir"] == 0


def test_gear_ratio_endpoint_updates_config(client):
    r = client.post("/api/gear_ratio", json={"can_id": 1, "gear_ratio": 7.5})
    assert r.status_code == 200
    j1 = next(a for a in client.get("/api/config").json()["axes"] if a["name"] == "J1")
    assert j1["gear_ratio"] == 7.5


def test_gear_ratio_endpoint_rejects_nonpositive(client):
    # pydantic Field(gt=0) rejects 0 before it reaches Motion.
    assert client.post("/api/gear_ratio", json={"can_id": 1, "gear_ratio": 0}).status_code == 422


def test_joint_zero_endpoint_calibrates_offset(client):
    # Home J1 (counter zeroed at the switch), then declare the pose to be 45°:
    # the offset absorbs the difference and the reported angle follows.
    assert client.post("/api/home", json={"can_id": 1}).status_code == 200
    assert _wait_homed(client, "J1")
    r = client.post("/api/joint_zero", json={"can_id": 1, "angle_deg": 45.0})
    assert r.status_code == 200
    assert r.json()["home_offset_deg"] == pytest.approx(45.0, abs=0.2)
    deg = client.get("/api/state").json()["axes"]["J1"]["degrees"]
    assert deg == pytest.approx(45.0, abs=0.2)
    j1 = next(a for a in client.get("/api/config").json()["axes"] if a["name"] == "J1")
    assert j1["home_offset_deg"] == pytest.approx(45.0, abs=0.2)


def test_joint_zero_routes_to_ros_in_ros_mode(client, monkeypatch):
    # In ROS mode the bridge owns the conversion: the offset is computed from its
    # published joint angle and pushed as a bridge param, mirrored locally.
    import backend.api as api

    class _Ros(_FakeRos):
        def __init__(self):
            super().__init__()
            self.offsets = []

        def joint_states(self):
            return {"name": ["J1", "J2"], "position_deg": [10.0, 0.0]}

        def set_home_offset(self, joint_name, offset_deg):
            self.offsets.append((joint_name, offset_deg))
            return {"success": True, "message": "ok"}

    fake = _Ros()
    monkeypatch.setattr(api.state, "ros", fake)
    r = client.post("/api/joint_zero", json={"can_id": 1, "angle_deg": 90.0})
    assert r.status_code == 200
    body = r.json()
    assert body["via"] == "ros"
    assert body["home_offset_deg"] == pytest.approx(80.0)  # 0 + (90 - 10)
    assert fake.offsets == [("J1", 80.0)]
    j1 = next(a for a in client.get("/api/config").json()["axes"] if a["name"] == "J1")
    assert j1["home_offset_deg"] == pytest.approx(80.0)


def test_root_serves_cache_busted_appjs(client):
    r = client.get("/")
    assert r.status_code == 200
    assert "app.js?v=" in r.text
    assert r.headers.get("cache-control") == "no-store"
