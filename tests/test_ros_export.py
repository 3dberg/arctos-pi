"""Tests for the ROS2/MoveIt joint export (single source of truth)."""
import math

import yaml

from backend.config import AppConfig, AxisConfig
from backend.ros_export import joint_limits, joints_spec, main, write_artifacts


def _cfg():
    return AppConfig(
        axes=[
            AxisConfig(can_id=1, name="J1", soft_limit_min=-180, soft_limit_max=180,
                       max_vel_deg_s=90, max_acc_deg_s2=180),
            AxisConfig(can_id=2, name="J2", soft_limit_min=-90, soft_limit_max=45,
                       max_vel_deg_s=60, max_acc_deg_s2=120),
        ],
        robot_type="arctos",
    )


def test_joints_spec_converts_degrees_to_radians():
    spec = joints_spec(_cfg())
    assert spec["robot_type"] == "arctos"
    j1 = spec["joints"]["J1"]
    assert j1["can_id"] == 1
    assert math.isclose(j1["lower"], math.radians(-180), abs_tol=1e-6)
    assert math.isclose(j1["upper"], math.radians(180), abs_tol=1e-6)
    assert math.isclose(j1["velocity"], math.radians(90), abs_tol=1e-6)


def test_joints_spec_preserves_asymmetric_limits():
    j2 = joints_spec(_cfg())["joints"]["J2"]
    assert math.isclose(j2["lower"], math.radians(-90), abs_tol=1e-6)
    assert math.isclose(j2["upper"], math.radians(45), abs_tol=1e-6)


def test_joint_limits_moveit_format():
    limits = joint_limits(_cfg())["joint_limits"]
    assert set(limits) == {"J1", "J2"}
    j1 = limits["J1"]
    assert j1["has_velocity_limits"] is True
    assert j1["has_acceleration_limits"] is True
    assert math.isclose(j1["max_velocity"], math.radians(90), abs_tol=1e-6)
    assert math.isclose(j1["max_acceleration"], math.radians(180), abs_tol=1e-6)


def test_write_artifacts_roundtrips(tmp_path):
    paths = write_artifacts(_cfg(), tmp_path)
    names = {p.name for p in paths}
    assert names == {"arctos_joints.yaml", "joint_limits.yaml"}
    loaded = yaml.safe_load((tmp_path / "arctos_joints.yaml").read_text())
    assert loaded["robot_type"] == "arctos"
    assert "J1" in loaded["joints"]


def test_main_cli_writes_files(tmp_path):
    # No --config file present -> falls back to default 6-axis config.
    out = tmp_path / "out"
    rc = main(["--config", str(tmp_path / "missing.yaml"), "--out", str(out)])
    assert rc == 0
    spec = yaml.safe_load((out / "arctos_joints.yaml").read_text())
    assert len(spec["joints"]) == 6  # default_six_axis
