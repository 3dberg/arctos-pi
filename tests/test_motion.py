"""Tests for motion coordinator + mock bus."""
from backend.can_bus import MockBus, DryRunBus, Frame
from backend.config import AppConfig, AxisConfig
from backend.motion import Motion, LimitViolation


def _cfg():
    cfg = AppConfig.default_six_axis()
    for ax in cfg.axes:
        ax.gear_ratio = 1.0
        ax.pulses_per_rev = 3200
        ax.max_speed = 1000
        ax.soft_limit_min = -180.0
        ax.soft_limit_max = 180.0
    return cfg


def test_jog_start_sends_speed_frame():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.jog_start(can_id=1, direction=1, speed_pct=0.5)
    assert len(bus.sent) == 1
    f = bus.sent[0]
    assert f.arbitration_id == 1
    assert f.data[0] == 0xF6  # speed-mode cmd
    # speed = 500 → hi nibble 0x1, lo byte 0xF4, dir CW → 0x81
    assert f.data[1] == 0x81
    assert f.data[2] == 500 & 0xFF


def test_jog_stop_sends_zero_speed():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.jog_stop(can_id=2)
    assert bus.sent[-1].data[0] == 0xF6
    # speed bytes zero
    assert bus.sent[-1].data[1] & 0x0F == 0
    assert bus.sent[-1].data[2] == 0


def test_jog_invert_flips_direction():
    cfg = _cfg()
    cfg.axis_by_id(1).invert = True
    bus = MockBus()
    m = Motion(cfg, bus)
    m.jog_start(can_id=1, direction=1, speed_pct=0.5)
    # CW(1) XOR invert → CCW, dir bit should be 0
    assert bus.sent[0].data[1] & 0x80 == 0


def test_negative_speed_pct_inverts_direction():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.jog_start(can_id=1, direction=1, speed_pct=-0.5)
    assert bus.sent[0].data[1] & 0x80 == 0  # CW flipped to CCW


def test_move_respects_soft_limits():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    try:
        m.move_to_degrees(1, 999.0)
    except LimitViolation:
        pass
    else:
        raise AssertionError("expected LimitViolation")


def test_move_all_is_atomic():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    try:
        m.move_all_degrees({1: 10.0, 2: 500.0})  # axis 2 out of range
    except LimitViolation:
        pass
    else:
        raise AssertionError("expected LimitViolation")
    # No frames sent because 2nd axis violated
    assert bus.sent == []


def test_move_updates_optimistic_state():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.move_to_degrees(1, 90.0)  # 90° at 3200 pulses/rev, gear 1.0 → 800 pulses
    assert m._state[1].pulses == 800


def test_emergency_stop_sends_to_all():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.emergency_stop()
    assert len(bus.sent) == 6
    assert all(f.data[0] == 0xF7 for f in bus.sent)


def test_dry_run_blocks_flash_writes():
    bus = DryRunBus()
    m = Motion(_cfg(), bus)
    try:
        m.set_microsteps(1, 32)
    except PermissionError:
        pass
    else:
        raise AssertionError("expected PermissionError in dry_run")
    # With the override flag it should go through
    m.allow_flash_writes = True
    m.set_microsteps(1, 32)
    assert bus.sent[-1].data[0] == 0x84


def test_set_microsteps_scales_pulses_per_rev():
    bus = MockBus()
    cfg = _cfg()
    ax = cfg.axis_by_id(1)
    ax.default_microsteps = 16
    ax.pulses_per_rev = 3200
    m = Motion(cfg, bus)
    m.set_microsteps(1, 32)
    assert ax.pulses_per_rev == 6400
    assert ax.default_microsteps == 32


def test_mock_auto_responds_to_read_pulses():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    bus._virtual_pulses[1] = 1234
    m.request_all_positions()
    # Mock auto-responds synchronously via inject → motion updates state
    assert m._state[1].pulses == 1234


def test_state_dict_shape():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    s = m.state_dict()
    assert set(s.keys()) == {f"J{i}" for i in range(1, 7)}
    assert s["J1"]["can_id"] == 1
    assert "degrees" in s["J1"]
