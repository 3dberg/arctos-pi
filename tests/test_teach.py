"""Tests for teach/record (Phase 4)."""
import json

import pytest

from backend.can_bus import MockBus
from backend.config import AppConfig, GripperConfig
from backend.gripper import Gripper
from backend.motion import Motion
from backend.teach import TeachError, TeachRecorder, Waypoint


def _setup(tmp_path, *, gripper_enabled=False):
    cfg = AppConfig.default_six_axis()
    bus = MockBus()
    motion = Motion(cfg, bus)
    gcfg = GripperConfig(enabled=gripper_enabled, can_id=0x07,
                         open_position=0, close_position=255, default_position=64)
    gripper = Gripper(gcfg, bus)
    rec = TeachRecorder(motion=motion, gripper=gripper, programs_dir=tmp_path)
    return rec, motion, gripper, bus


def test_capture_snapshots_current_joints(tmp_path):
    rec, motion, _, _ = _setup(tmp_path)
    motion._state[1].pulses = 800  # 90° at default 1:1, 3200 pulses/rev not configured
    wp = rec.capture(dwell_ms=250, speed_pct=0.4)
    assert set(wp.joints.keys()) == {f"J{i}" for i in range(1, 7)}
    assert wp.dwell_ms == 250
    assert wp.speed_pct == 0.4
    assert rec.dirty is True
    assert len(rec.waypoints) == 1


def test_capture_includes_gripper_only_when_enabled(tmp_path):
    rec_off, *_ = _setup(tmp_path, gripper_enabled=False)
    wp1 = rec_off.capture()
    assert wp1.gripper is None

    rec_on, _, gripper, _ = _setup(tmp_path / "on", gripper_enabled=True)
    gripper._position = 123
    wp2 = rec_on.capture()
    assert wp2.gripper == 123


def test_capture_validates_inputs(tmp_path):
    rec, *_ = _setup(tmp_path)
    with pytest.raises(TeachError):
        rec.capture(dwell_ms=-5)
    with pytest.raises(TeachError):
        rec.capture(speed_pct=0.0)
    with pytest.raises(TeachError):
        rec.capture(speed_pct=1.5)


def test_delete_and_index_check(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture(); rec.capture(); rec.capture()
    rec.delete(1)
    assert len(rec.waypoints) == 2
    with pytest.raises(TeachError):
        rec.delete(99)


def test_reorder(tmp_path):
    rec, *_ = _setup(tmp_path)
    a = rec.capture(dwell_ms=100)
    b = rec.capture(dwell_ms=200)
    c = rec.capture(dwell_ms=300)
    rec.reorder(0, 2)  # a -> end
    assert [w.dwell_ms for w in rec.waypoints] == [200, 300, 100]
    with pytest.raises(TeachError):
        rec.reorder(0, 99)


def test_update_partial(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture(dwell_ms=100, speed_pct=0.3)
    rec.update(0, dwell_ms=500)
    assert rec.waypoints[0].dwell_ms == 500
    assert rec.waypoints[0].speed_pct == 0.3   # unchanged
    rec.update(0, speed_pct=0.7, gripper=128)
    assert rec.waypoints[0].speed_pct == 0.7
    assert rec.waypoints[0].gripper == 128
    with pytest.raises(TeachError):
        rec.update(0, gripper=999)
    with pytest.raises(TeachError):
        rec.update(0, dwell_ms=10**9)


def test_save_load_roundtrip(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture(dwell_ms=100, speed_pct=0.4)
    rec.capture(dwell_ms=250, speed_pct=0.6)
    path = rec.save("pick-place")
    assert path.exists()
    on_disk = json.loads(path.read_text())
    assert on_disk["version"] == 1
    assert on_disk["name"] == "pick-place"
    assert len(on_disk["waypoints"]) == 2
    assert rec.dirty is False
    assert rec.loaded_name == "pick-place"

    # Mutate then load — load should overwrite
    rec.capture()
    assert rec.dirty is True
    rec.load("pick-place")
    assert len(rec.waypoints) == 2
    assert rec.dirty is False
    assert rec.loaded_name == "pick-place"


def test_list_programs_sorted(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture()
    rec.save("alpha")
    rec.save("beta")
    rec.save("0001-warmup")
    assert rec.list_programs() == ["0001-warmup", "alpha", "beta"]


def test_invalid_program_names_rejected(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture()
    for bad in ["../escape", "with space", "slash/inside", "", ".hidden", ".."]:
        with pytest.raises(TeachError):
            rec.save(bad)


def test_load_unknown_program(tmp_path):
    rec, *_ = _setup(tmp_path)
    with pytest.raises(TeachError):
        rec.load("nope")


def test_load_rejects_unknown_schema_version(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec._ensure_dir()
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"name": "bad", "version": 99, "waypoints": []}))
    with pytest.raises(TeachError):
        rec.load("bad")


def test_clear_resets_state(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture(); rec.save("x"); rec.capture()
    assert rec.loaded_name == "x"
    rec.clear()
    assert rec.waypoints == []
    assert rec.loaded_name is None
    assert rec.dirty is False


def test_delete_program(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture()
    rec.save("temp")
    assert "temp" in rec.list_programs()
    rec.delete_program("temp")
    assert "temp" not in rec.list_programs()
    assert rec.loaded_name is None
    with pytest.raises(TeachError):
        rec.delete_program("temp")  # already gone


def test_state_dict_shape(tmp_path):
    rec, *_ = _setup(tmp_path)
    rec.capture()
    s = rec.state_dict()
    assert s["count"] == 1
    assert s["loaded_name"] is None
    assert s["dirty"] is True
    assert isinstance(s["waypoints"], list)
    assert "joints" in s["waypoints"][0]


def test_waypoint_dict_roundtrip():
    wp = Waypoint(joints={"J1": 12.5, "J2": -30.0}, dwell_ms=300, speed_pct=0.4, gripper=200)
    d = wp.to_dict()
    assert d["gripper"] == 200
    assert "t_ms" not in d
    rt = Waypoint.from_dict(d)
    assert rt.joints == wp.joints
    assert rt.dwell_ms == 300 and rt.speed_pct == 0.4 and rt.gripper == 200


def test_loaded_name_uses_filename_not_json_field(tmp_path):
    rec, *_ = _setup(tmp_path)
    # Hand-craft a file whose internal "name" disagrees with the filename
    rec._ensure_dir()
    (tmp_path / "filename.json").write_text(json.dumps({
        "name": "json-internal-name", "version": 1,
        "waypoints": [{"joints": {"J1": 0.0}, "dwell_ms": 0, "speed_pct": 0.5}],
    }))
    rec.load("filename")
    assert rec.loaded_name == "filename"
