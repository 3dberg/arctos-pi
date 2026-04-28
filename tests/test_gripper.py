"""Tests for the CAN gripper controller."""
import pytest

from backend.can_bus import MockBus
from backend.config import GripperConfig
from backend.gripper import Gripper


def _gripper(**overrides):
    cfg = GripperConfig(enabled=True, can_id=0x07, open_position=0, close_position=255,
                        default_position=0)
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return Gripper(cfg, MockBus())


def test_set_position_sends_single_byte_frame():
    g = _gripper()
    g.set_position(128)
    assert len(g.bus.sent) == 1
    f = g.bus.sent[0]
    assert f.arbitration_id == 0x07
    assert f.data == bytes([128])
    assert g.position == 128


def test_position_clamped_to_byte_range():
    g = _gripper()
    g.set_position(-50)
    assert g.bus.sent[-1].data == bytes([0])
    g.set_position(999)
    assert g.bus.sent[-1].data == bytes([255])
    assert g.position == 255


def test_open_close_use_configured_endpoints():
    g = _gripper(open_position=10, close_position=200)
    g.open()
    assert g.bus.sent[-1].data == bytes([10])
    assert g.position == 10
    g.close()
    assert g.bus.sent[-1].data == bytes([200])
    assert g.position == 200


def test_disabled_gripper_refuses_commands():
    cfg = GripperConfig(enabled=False)
    g = Gripper(cfg, MockBus())
    with pytest.raises(RuntimeError):
        g.set_position(100)
    assert g.bus.sent == []


def test_state_dict_shape():
    g = _gripper(open_position=5, close_position=250)
    g.set_position(64)
    s = g.state_dict()
    assert s["enabled"] is True
    assert s["can_id"] == 0x07
    assert s["position"] == 64
    assert s["open_position"] == 5
    assert s["close_position"] == 250


def test_custom_can_id():
    g = _gripper(can_id=0x1A)
    g.set_position(42)
    assert g.bus.sent[-1].arbitration_id == 0x1A


def test_default_position_initialized():
    g = _gripper(default_position=77)
    assert g.position == 77
    # Default position is NOT auto-sent on construction (caller decides).
    assert g.bus.sent == []
