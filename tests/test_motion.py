"""Tests for motion coordinator + mock bus."""
import time

import pytest

from backend import mks
from backend.can_bus import MockBus, DryRunBus, Frame
from backend.config import AppConfig, AxisConfig, WristDifferential
from backend.motion import HomingError, Motion, LimitViolation, NotHomedError


def _wait(pred, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    return False


def _cfg():
    cfg = AppConfig.default_six_axis()
    # These tests exercise raw move mechanics, not homing; disable the
    # home-before-move gate (its own behavior is covered in the homing tests).
    cfg.require_home_before_move = False
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


def test_set_work_mode_sends_cmd_82():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.set_work_mode(can_id=3, mode=5)  # SR_vFOC
    f = bus.sent[-1]
    assert f.arbitration_id == 3
    assert f.data[0] == 0x82
    assert f.data[1] == 5


def test_set_gear_ratio_changes_conversion():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.set_gear_ratio(1, 2.0)
    assert m.cfg.axis_by_id(1).gear_ratio == 2.0
    # 90° at 3200 pulses/rev, gear 2.0 → 1600 pulses (vs 800 at gear 1.0).
    m.move_to_degrees(1, 90.0)
    assert m._state[1].pulses == 1600


def test_set_gear_ratio_is_software_only_in_dry_run():
    bus = DryRunBus()
    m = Motion(_cfg(), bus)
    # Pure conversion factor: no flash/CAN write, so it must NOT raise in dry_run
    # (unlike set_microsteps) and must not emit a frame.
    m.set_gear_ratio(1, 5.0)
    assert m.cfg.axis_by_id(1).gear_ratio == 5.0
    assert bus.sent == []


def test_set_gear_ratio_rejects_nonpositive():
    m = Motion(_cfg(), MockBus())
    with pytest.raises(ValueError):
        m.set_gear_ratio(1, 0)


def test_set_home_offset_is_software_only():
    bus = DryRunBus()
    m = Motion(_cfg(), bus)
    m.set_home_offset(1, 12.5)
    assert m.cfg.axis_by_id(1).home_offset_deg == 12.5
    assert bus.sent == []


def test_calibrate_joint_zero_shifts_reported_angle():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m.move_to_degrees(1, 30.0)
    m.request_all_positions()
    assert m.state_dict()["J1"]["degrees"] == pytest.approx(30.0, abs=0.2)
    # Declare "this pose is really 90°": offset absorbs the 60° discrepancy.
    off = m.calibrate_joint_zero(1, 90.0)
    assert off == pytest.approx(60.0, abs=0.2)
    assert m.state_dict()["J1"]["degrees"] == pytest.approx(90.0, abs=0.2)
    # Absolute moves share the calibrated frame: going to 90° is now a no-op.
    n = len(bus.sent)
    m.move_to_degrees(1, 90.0)
    fd = [f for f in bus.sent[n:] if f.data[0] == 0xFD]
    assert len(fd) == 1 and int.from_bytes(fd[0].data[4:7], "big") == 0


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


def test_poll_skips_axes_being_homed():
    # An axis mid-home is already polled by its home worker; the periodic
    # pollers must skip it so they don't double the bus traffic during the
    # busiest moment (the CAN-overload fix).
    bus = MockBus()
    m = Motion(_cfg(), bus)
    m._state[2].homing_in_progress = True
    bus.sent.clear()
    m.request_all_positions()
    m.request_all_io()
    polled = {f.arbitration_id for f in bus.sent}
    assert 2 not in polled
    assert {1, 3, 4, 5, 6} <= polled


def test_state_dict_shape():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    s = m.state_dict()
    assert set(s.keys()) == {f"J{i}" for i in range(1, 7)}
    assert s["J1"]["can_id"] == 1
    assert "degrees" in s["J1"]
    assert "is_homed" in s["J1"] and "homing" in s["J1"]


# ---- homing ----

def _home_cfg():
    cfg = _cfg()
    cfg.require_home_before_move = True   # the gate is the point of these tests
    return cfg


def test_home_axis_completes_and_applies_offset():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_offset_deg = 90.0
    bus = MockBus()
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed)
    st = m.state_dict()["J1"]
    assert st["is_homed"] is True
    assert st["homing"] is False
    # MockBus zeroes the counter at the switch; reported angle == the offset.
    assert m._state[1].home_pulse_zero == 0
    assert abs(st["degrees"] - 90.0) < 1e-6


def test_move_blocked_until_homed():
    bus = MockBus()
    m = Motion(_home_cfg(), bus)
    with pytest.raises(NotHomedError):
        m.move_to_degrees(1, 10.0)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed)
    m.move_to_degrees(1, 10.0)  # now allowed
    assert any(f.data[0] == 0xFD for f in bus.sent)


def test_move_all_atomic_homed_gate():
    bus = MockBus()
    m = Motion(_home_cfg(), bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed)
    bus.sent.clear()
    # axis 2 still un-homed -> whole gesture rejected, no move frames sent
    with pytest.raises(NotHomedError):
        m.move_all_degrees({1: 10.0, 2: 10.0})
    assert not any(f.data[0] in (0xFD, 0xFE) for f in bus.sent)


def test_home_enabled_false_is_exempt():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_enabled = False
    bus = MockBus()
    m = Motion(cfg, bus)
    m.move_to_degrees(1, 10.0)  # no homing required for this axis
    assert any(f.data[0] == 0xFD for f in bus.sent)


class _StuckBus(MockBus):
    """GoHome never reports success and the axis sits at a fixed pulse count
    (no motion, switch never trips), so a seek just waits — used to test that
    an in-flight home can be canceled."""
    def _maybe_respond(self, frame):
        cmd = frame.data[0]
        can_id = frame.arbitration_id
        if cmd == 0x91:
            self._respond_status(can_id, 0x91, 1)  # Start, never Success
        elif cmd == 0x90:
            self._respond_status(can_id, 0x90, 1)
        elif cmd == 0x31:
            pulses = self._virtual_pulses.get(can_id, 0)
            body = bytes([0x31]) + pulses.to_bytes(4, "big", signed=True)
            crc = (can_id + sum(body)) & 0xFF
            self.inject(Frame(can_id, body + bytes([crc])))


class _RunawayBus(MockBus):
    """GoHome never succeeds and the axis keeps TRAVELING (position climbs each
    read) without the switch tripping, so the seek-travel bound must abort it."""
    def _maybe_respond(self, frame):
        cmd = frame.data[0] if frame.data else None
        cid = frame.arbitration_id
        if cmd == 0x91:
            self._respond_status(cid, 0x91, 1)  # Start, never Success
            return
        if cmd == 0x31:
            self._virtual_pulses[cid] = self._virtual_pulses.get(cid, 0) + 100000
        super()._maybe_respond(frame)


def test_home_seek_max_abort_estops():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_seek_max_deg = 10.0   # tiny bound; the seek travels far past it
    bus = _RunawayBus()
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: not m._state[1].homing_in_progress)
    assert m._state[1].is_homed is False
    assert m._state[1].home_error and "seek exceeded" in m._state[1].home_error
    assert any(f.arbitration_id == 1 and f.data[0] == 0xF7 for f in bus.sent)  # e-stopped


def test_estop_cancels_in_flight_homing():
    bus = _StuckBus()
    bus._virtual_pulses[1] = 50  # under the seek bound, so it just keeps waiting
    m = Motion(_home_cfg(), bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].homing_in_progress)
    m.emergency_stop()
    assert _wait(lambda: not m._state[1].homing_in_progress)
    assert m._state[1].is_homed is False


def test_request_all_io_and_home_switch():
    cfg = _cfg()  # home_trig_low defaults True (reverse logic)
    bus = MockBus()
    m = Motion(cfg, bus)
    m.request_all_io()
    assert sum(1 for f in bus.sent if f.data[0] == 0x34) == len(cfg.axes)
    # MockBus default IO has IN_1 high (0x01); under active-low that's NOT tripped.
    assert m._state[1].last_io == {"in_1": True, "in_2": False, "out_1": False, "out_2": False}
    assert m.home_switch(1) is False
    assert m.state_dict()["J1"]["home_switch"] is False
    # IN_1 low under active-low -> tripped.
    bus._io_status[1] = 0x00
    m.request_all_io()
    assert m.home_switch(1) is True
    # Non-inverting axis: IN_1 high == tripped.
    cfg.axis_by_id(2).home_trig_low = False
    bus._io_status[2] = 0x01
    m.request_all_io()
    assert m.home_switch(2) is True


def test_home_switch_none_before_any_read():
    m = Motion(_cfg(), MockBus())
    assert m.home_switch(1) is None
    assert m.state_dict()["J1"]["home_switch"] is None


def test_on_frame_ignores_short_echo_frames():
    # When can0 drops, our own 2-byte read requests can loop back. They must be
    # ignored (no exception, no state change), not parsed as replies.
    m = Motion(_cfg(), MockBus())
    m._state[1].pulses = 123
    m._on_frame(Frame(1, bytes([0x31, 0x33])))  # echoed read_pulses request for id 1
    assert m._state[1].pulses == 123


class _BackoffBus(MockBus):
    """Reports the home switch active for the first 2 IO reads, then clear —
    simulating the axis moving off the switch during back-off."""
    def _maybe_respond(self, frame):
        if frame.data and frame.data[0] == 0x34:
            n = getattr(self, "_io_reads", 0) + 1
            self._io_reads = n
            self._io_status[frame.arbitration_id] = 0x00 if n <= 2 else 0x01
        super()._maybe_respond(frame)


class _StuckSwitchBus(MockBus):
    """Switch never clears, and the axis keeps reading further travel, so the
    back-off travel bound must abort it."""
    def _maybe_respond(self, frame):
        if frame.data and frame.data[0] == 0x34:
            self._io_status[frame.arbitration_id] = 0x00  # always active
        elif frame.data and frame.data[0] == 0x31:
            cid = frame.arbitration_id
            self._virtual_pulses[cid] = self._virtual_pulses.get(cid, 0) + 100000
        super()._maybe_respond(frame)


def test_home_backoff_clears_switch_then_homes():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_backoff_deg = 30.0
    bus = _BackoffBus()
    bus._io_status[1] = 0x00  # start on the switch
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: not m._state[1].homing_in_progress, timeout=5)
    assert m._state[1].is_homed is True
    assert m._state[1].home_error is None
    # back-off drove the axis (speed-mode frame) then stopped it (speed 0)
    assert any(f.arbitration_id == 1 and f.data[0] == 0xF6 for f in bus.sent)


def test_home_backoff_aborts_when_switch_stuck():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_backoff_deg = 10.0
    bus = _StuckSwitchBus()
    bus._io_status[1] = 0x00  # active and never clears
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: not m._state[1].homing_in_progress, timeout=5)
    assert m._state[1].is_homed is False
    assert m._state[1].home_error and "back-off" in m._state[1].home_error
    # the motor was stopped (speed-mode 0) rather than left running
    assert any(f.arbitration_id == 1 and f.data[0] == 0xF6 and f.data[1] == 0 and f.data[2] == 0
               for f in bus.sent)


# ---- robust completion + deterministic zeroing (lost-frame tolerance) ----

def test_pulses_seq_bumps_on_reply():
    bus = MockBus()
    m = Motion(_cfg(), bus)
    before = m._state[1].pulses_seq
    m.request_all_positions()  # MockBus auto-responds per axis
    assert m._state[1].pulses_seq > before


class _NoSuccessBus(MockBus):
    """GoHome reports only Start (never Success) — the 0x91 SUCCESS frame is
    'lost'. With a fixed position and an active switch this exercises the
    settle-at-switch completion fallback."""
    def _maybe_respond(self, frame):
        if frame.data and frame.data[0] == 0x91:
            self._respond_status(frame.arbitration_id, 0x91, 1)  # Start only
            return
        super()._maybe_respond(frame)


def test_home_completes_by_settle_when_success_frame_lost():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_backoff_deg = 0   # skip back-off; isolate the seek
    bus = _NoSuccessBus()
    bus._io_status[1] = 0x00                  # switch active (home_trig_low default)
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed, timeout=5)
    assert m._state[1].home_error is None
    assert m._state[1].last_home_status == mks.GO_HOME_START  # never saw SUCCESS
    assert m._state[1].home_pulse_zero == 0                   # 0x92 zeroed at the switch


class _DropFirstReadsBus(MockBus):
    """Drops the first `drop` read_pulses (0x31) replies, then behaves normally —
    exercises _read_pulses_fresh's retry-until-a-fresh-reply during post-home
    zeroing (a single sleep-then-trust read would have mis-anchored the zero)."""
    def _maybe_respond(self, frame):
        if frame.data and frame.data[0] == 0x31:
            done = getattr(self, "dropped", 0)
            if done < getattr(self, "drop", 0):
                self.dropped = done + 1
                return  # swallow this reply
        super()._maybe_respond(frame)


def test_home_zeroing_retries_until_fresh_read():
    cfg = _home_cfg()
    bus = _DropFirstReadsBus()
    bus.drop, bus.dropped = 3, 0   # default IO 0x01 -> switch inactive -> back-off skipped
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed, timeout=5)
    assert bus.dropped == 3                      # the read-back retried past the drops
    assert m._state[1].home_pulse_zero == 0
    assert m._state[1].home_error is None


def test_home_fails_cleanly_when_no_readback_after_zeroing():
    cfg = _home_cfg()
    bus = _DropFirstReadsBus()
    bus.drop, bus.dropped = 100, 0   # post-zero read-back never answers
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: not m._state[1].homing_in_progress, timeout=5)
    assert m._state[1].is_homed is False
    assert m._state[1].home_error and "read-back" in m._state[1].home_error


def test_home_use_driver_params_skips_set_home_and_backoff():
    # The axis homes fine from the panel but not the UI: in driver-params mode we
    # must NOT overwrite the driver's flashed home params (no 0x90) and not run
    # the back-off — just trigger GoHome (0x91).
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_use_driver_params = True
    bus = MockBus()
    bus._io_status[1] = 0x00   # switch "active" — back-off WOULD fire if not skipped
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed, timeout=5)
    assert not any(f.arbitration_id == 1 and f.data[0] == 0x90 for f in bus.sent)  # no set_home
    assert not any(f.arbitration_id == 1 and f.data[0] == 0xF6 for f in bus.sent)  # no back-off drive
    assert any(f.arbitration_id == 1 and f.data[0] == 0x91 for f in bus.sent)      # GoHome sent


class _SeekThenStopBus(MockBus):
    """0x91 reports only Start; position advances for the first few reads then
    holds (the axis seeks then stops). The home switch reads INACTIVE throughout
    (miswired/unknown polarity) — completion must come from the motion-settle
    path, not the switch."""
    def _maybe_respond(self, frame):
        cmd = frame.data[0] if frame.data else None
        cid = frame.arbitration_id
        if cmd == 0x91:
            self._respond_status(cid, 0x91, 1)  # Start only; SUCCESS "lost"
            return
        if cmd == 0x92:
            self._zeroed = True
            self._virtual_pulses[cid] = 0
            self._respond_status(cid, 0x92, 1)
            return
        if cmd == 0x31 and not getattr(self, "_zeroed", False):
            n = getattr(self, "_reads", 0) + 1
            self._reads = n
            self._virtual_pulses[cid] = min(n, 5) * 100  # advance, then hold at 500
        super()._maybe_respond(frame)


def test_home_completes_on_motion_settle_without_switch():
    cfg = _home_cfg()
    cfg.axis_by_id(1).home_use_driver_params = True
    bus = _SeekThenStopBus()
    bus._io_status[1] = 0x01   # switch reads NOT active (home_trig_low default)
    m = Motion(cfg, bus)
    m.home_axis(1)
    assert _wait(lambda: m._state[1].is_homed, timeout=5)
    assert m._state[1].home_error is None
    assert m._state[1].home_pulse_zero == 0   # 0x92 zeroed after the seek settled


# ---- differential wrist (J5/J6: two motors -> two joints) ----

def _wrist_cfg(invert=False, motor_b_invert=False):
    """_cfg() (gear 1.0, ppr 3200, no-home gate) + an enabled J5/J6 differential.
    motor_a=5/pitch, motor_b=6/roll — both at gear 1.0 so motor pulses are easy to
    reason about (10° -> round(10/360*3200) = 89 pulses)."""
    cfg = _cfg()
    cfg.axis_by_id(6).invert = motor_b_invert
    cfg.wrist_differential = WristDifferential(
        motor_a=5, motor_b=6, roll_can=6, pitch_can=5, invert=invert)
    return cfg


def _fd(frames):
    """{can_id: frame} for the 0xFD (position-relative) frames sent."""
    return {f.arbitration_id: f for f in frames if f.data[0] == 0xFD}


def _f6(frames):
    """{can_id: frame} for the 0xF6 (speed-mode) frames sent."""
    return {f.arbitration_id: f for f in frames if f.data[0] == 0xF6}


def _dir_bit(f):
    return 1 if (f.data[1] & 0x80) else 0


def _fd_pulses(f):
    return int.from_bytes(f.data[4:7], "big")


def _f6_speed(f):
    return ((f.data[1] & 0x0F) << 8) | f.data[2]


def test_wrist_pure_roll_moves_both_motors_same_dir():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.move_all_degrees({5: 0.0, 6: 10.0})   # roll only
    fd = _fd(bus.sent)
    assert set(fd) == {5, 6}
    assert _fd_pulses(fd[5]) == _fd_pulses(fd[6]) > 0     # equal magnitude
    assert _dir_bit(fd[5]) == _dir_bit(fd[6])             # same direction


def test_wrist_pure_pitch_moves_both_motors_opposite():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.move_all_degrees({5: 10.0, 6: 0.0})   # pitch only
    fd = _fd(bus.sent)
    assert set(fd) == {5, 6}
    assert _fd_pulses(fd[5]) == _fd_pulses(fd[6]) > 0     # equal magnitude
    assert _dir_bit(fd[5]) != _dir_bit(fd[6])             # opposite direction


def test_wrist_combined_move_sends_two_frames_not_four():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.move_all_degrees({5: 5.0, 6: 20.0})   # both joints, asymmetric
    fd = _fd(bus.sent)
    assert len(fd) == 2 and set(fd) == {5, 6}             # not one move per joint
    assert m._state[5].pulses != 0 and m._state[6].pulses != 0  # both optimistic


def test_wrist_roundtrip_decode():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.move_all_degrees({5: 12.0, 6: 30.0})
    m.request_all_positions()               # MockBus echoes virtual pulses
    st = m.state_dict()
    assert st["J5"]["degrees"] == pytest.approx(12.0, abs=0.2)
    assert st["J6"]["degrees"] == pytest.approx(30.0, abs=0.2)


def test_wrist_jog_roll_drives_both_same_dir():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.jog_start(can_id=6, direction=1, speed_pct=0.5)    # J6 = roll
    f6 = _f6(bus.sent)
    assert set(f6) == {5, 6}
    assert _dir_bit(f6[5]) == _dir_bit(f6[6])            # same direction
    assert _f6_speed(f6[5]) == _f6_speed(f6[6]) > 0      # equal speed


def test_wrist_jog_pitch_drives_both_opposite():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.jog_start(can_id=5, direction=1, speed_pct=0.5)    # J5 = pitch
    f6 = _f6(bus.sent)
    assert set(f6) == {5, 6}
    assert _dir_bit(f6[5]) != _dir_bit(f6[6])            # opposite direction
    assert _f6_speed(f6[5]) == _f6_speed(f6[6]) > 0      # equal speed


def test_wrist_jog_stop_stops_both_motors():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.jog_start(can_id=5, direction=1, speed_pct=0.5)
    bus.sent.clear()
    m.jog_stop(5)
    f6 = _f6(bus.sent)
    assert set(f6) == {5, 6}                              # both motors stopped
    assert all(_f6_speed(f) == 0 for f in f6.values())


def test_wrist_jog_respects_motor_invert():
    bus = MockBus()
    m = Motion(_wrist_cfg(motor_b_invert=True), bus)
    m.jog_start(can_id=6, direction=1, speed_pct=0.5)     # roll: same dir in joint space
    f6 = _f6(bus.sent)
    # motor_b's own invert flips its raw direction relative to motor_a
    assert _dir_bit(f6[5]) != _dir_bit(f6[6])


def test_wrist_soft_limit_on_joint_still_enforced():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    with pytest.raises(LimitViolation):
        m.move_all_degrees({5: 999.0, 6: 0.0})
    assert bus.sent == []                                 # atomic: nothing sent


def test_wrist_disabled_is_uncoupled():
    bus = MockBus()
    m = Motion(_cfg(), bus)                               # no wrist section
    m.move_to_degrees(5, 10.0)
    fd = _fd(bus.sent)
    assert set(fd) == {5}                                 # exactly one motor moved


def test_wrist_decode_falls_back_when_not_both_homed():
    cfg = _wrist_cfg()
    cfg.require_home_before_move = True                   # gate on -> decode needs homed
    m = Motion(cfg, MockBus())
    st = m.state_dict()                                   # must not raise mid-home
    assert "J5" in st and "J6" in st


def test_calibrate_joint_zero_on_wrist_joint_leaves_partner_alone():
    bus = MockBus()
    m = Motion(_wrist_cfg(), bus)
    m.move_all_degrees({5: 10.0, 6: 20.0})
    m.request_all_positions()
    off = m.calibrate_joint_zero(5, 0.0)    # pitch is physically at 0 here
    assert off == pytest.approx(-10.0, abs=0.2)
    st = m.state_dict()
    assert st["J5"]["degrees"] == pytest.approx(0.0, abs=0.2)
    assert st["J6"]["degrees"] == pytest.approx(20.0, abs=0.2)  # roll unaffected
