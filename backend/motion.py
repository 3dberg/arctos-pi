"""6-axis motion coordinator.

Sits between the web API and the CAN bus. Responsibilities:
  - Translate degrees-at-output into pulses, honoring gear_ratio + invert.
  - Enforce soft joint limits. Reject commands that would exceed them.
  - Clamp speed/current/microstep values to per-axis ceilings.
  - Expose a stop-all (e-stop) that sends 0xF7 to every configured axis.
  - Track last known pulse position per axis (optimistic; updated from reads).

Dry-run safety: if bus backend is 'dry_run', config writes (current,
microsteps, work mode) that persist to driver flash are refused unless
an explicit allow_flash_writes flag is set.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from . import mks
from .can_bus import CanBus, Frame
from .config import AppConfig, AxisConfig, WristDifferential

log = logging.getLogger(__name__)

# Homing tuning.
_HOME_POLL_S = 0.1          # cadence of the GoHome monitor loop
_HOME_TIMEOUT_S = 60.0      # hard wall-clock bound on a single seek
# Completion-detection fallbacks (the driver's 0x91 SUCCESS frame can be lost):
_FRESH_READ_RETRIES = 5     # re-send a read this many times waiting for a NEW reply
_FRESH_READ_WAIT_S = 0.12   # per-attempt wait; > one 20 Hz poll period + hw latency
_SETTLE_TOL_PULSES = 4      # |Δpulses| between reads treated as "not moving"
_SETTLE_POLLS = 4           # consecutive settled reads to declare the seek done
_ZERO_TOL_PULSES = 8        # |pulses| after 0x92 accepted as "counter zeroed"


class LimitViolation(ValueError):
    pass


class NotHomedError(RuntimeError):
    """Raised when an absolute move is attempted on an un-homed axis."""


class HomingError(RuntimeError):
    """Raised when a homing request can't be started (bad axis / already homing)."""


@dataclass
class AxisState:
    can_id: int
    pulses: int = 0
    pulses_seq: int = 0                 # bumped on every parsed 0x31 reply (read-freshness)
    enabled: bool = False
    last_error: Optional[str] = None
    # ---- homing ----
    is_homed: bool = False
    homing_in_progress: bool = False
    home_pulse_zero: int = 0            # raw driver pulse count recorded at the switch
    last_home_status: Optional[int] = None  # last GoHome (0x91) status seen on the bus
    home_error: Optional[str] = None
    last_io: Optional[dict] = None      # last decoded 0x34 IO status (debug)
    last_io_seq: int = 0                # bumped on every parsed 0x34 reply (read-freshness)


@dataclass
class Motion:
    cfg: AppConfig
    bus: CanBus
    _state: dict[int, AxisState] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    allow_flash_writes: bool = False
    # Homing runs on per-axis background threads (Motion is otherwise fire-and-forget).
    _home_threads: dict[int, threading.Thread] = field(default_factory=dict)
    _home_cancel: dict[int, threading.Event] = field(default_factory=dict)
    _home_all_thread: Optional[threading.Thread] = None
    homing_all_in_progress: bool = False

    def __post_init__(self) -> None:
        for ax in self.cfg.axes:
            self._state[ax.can_id] = AxisState(can_id=ax.can_id)
        # Differential wrist coupling (J5/J6) when configured; None = every joint
        # is 1:1 with its motor (the historical assumption, still true for J1-J4).
        self._wrist: Optional[WristDifferential] = self.cfg.wrist_differential
        self.bus.on_receive(self._on_frame)

    # ---- conversions ----
    #
    # Reported angle = f(raw_pulses - home_pulse_zero) + home_offset_deg, so a
    # homed axis reads its true output-shaft angle: at the switch the driver's
    # counter is ~home_pulse_zero, which maps to home_offset_deg. Before homing,
    # home_pulse_zero is 0 and these are the plain optimistic conversions.

    def _home_zero(self, ax: AxisConfig) -> int:
        st = self._state.get(ax.can_id)
        return st.home_pulse_zero if st is not None else 0

    # The deg<->pulse conversion factors into two halves: a degrees<->motor-turns
    # part (gear_ratio + home offset, per JOINT) and a motor-turns<->raw-pulses
    # part (pulses_per_rev + invert + home zero, per MOTOR). The differential wrist
    # composes joints from motors, so it works in the shared motor-turns middle.

    def _joint_turns(self, ax: AxisConfig, degrees: float) -> float:
        """Output degrees -> motor turns demanded by this joint (offset removed)."""
        return ((degrees - ax.home_offset_deg) / 360.0) * ax.gear_ratio

    def _turns_to_deg(self, ax: AxisConfig, motor_turns: float) -> float:
        return (motor_turns / ax.gear_ratio) * 360.0 + ax.home_offset_deg

    def _turns_to_pulses(self, ax: AxisConfig, motor_turns: float) -> int:
        """Motor turns -> raw driver pulse target for THIS motor (ppr, invert, zero)."""
        pulses = int(round(motor_turns * ax.pulses_per_rev))
        rel = -pulses if ax.invert else pulses
        return rel + self._home_zero(ax)

    def _pulses_to_turns(self, ax: AxisConfig, pulses: int) -> float:
        rel = pulses - self._home_zero(ax)
        if ax.invert:
            rel = -rel
        return rel / ax.pulses_per_rev

    def _deg_to_pulses(self, ax: AxisConfig, degrees: float) -> int:
        return self._turns_to_pulses(ax, self._joint_turns(ax, degrees))

    def _pulses_to_deg(self, ax: AxisConfig, pulses: int) -> float:
        return self._turns_to_deg(ax, self._pulses_to_turns(ax, pulses))

    # ---- differential wrist (J5/J6: two motors -> two joints) ----

    def _wrist_decode(self) -> tuple[float, float]:
        """Decode both wrist motor encoders into (roll_deg, pitch_deg)."""
        w = self._wrist
        a_ax = self.cfg.axis_by_id(w.motor_a)
        b_ax = self.cfg.axis_by_id(w.motor_b)
        turns_a = self._pulses_to_turns(a_ax, self._state[w.motor_a].pulses)
        turns_b = self._pulses_to_turns(b_ax, self._state[w.motor_b].pulses)
        n_roll, n_pitch = w.inverse(turns_a, turns_b)
        return (self._turns_to_deg(self.cfg.axis_by_id(w.roll_can), n_roll),
                self._turns_to_deg(self.cfg.axis_by_id(w.pitch_can), n_pitch))

    def _wrist_targets(self, roll_deg: float, pitch_deg: float) -> dict[int, int]:
        """(roll_deg, pitch_deg) -> raw pulse target per wrist motor (can_id->pulses)."""
        w = self._wrist
        n_roll = self._joint_turns(self.cfg.axis_by_id(w.roll_can), roll_deg)
        n_pitch = self._joint_turns(self.cfg.axis_by_id(w.pitch_can), pitch_deg)
        turns_a, turns_b = w.forward(n_roll, n_pitch)
        return {
            w.motor_a: self._turns_to_pulses(self.cfg.axis_by_id(w.motor_a), turns_a),
            w.motor_b: self._turns_to_pulses(self.cfg.axis_by_id(w.motor_b), turns_b),
        }

    def _wrist_decodable(self) -> bool:
        """Coupled decode needs BOTH motors' home zero to be meaningful. Mid
        home-all one motor is zeroed and the other isn't, which would report wild
        joint angles, so fall back to the raw per-axis read until both are homed
        (or the home gate is off for bench bring-up)."""
        if not self.cfg.require_home_before_move:
            return True
        w = self._wrist
        for mc in (w.motor_a, w.motor_b):
            ax = self.cfg.axis_by_id(mc)
            if ax.home_enabled and not self._state[mc].is_homed:
                return False
        return True

    def _axis_degrees(self, ax: AxisConfig) -> float:
        """Reported output angle for an axis: coupled decode for the wrist pair
        (when both motors are homed), else the plain per-motor conversion."""
        if self._wrist is not None and self._wrist.involves(ax.can_id) and self._wrist_decodable():
            roll_deg, pitch_deg = self._wrist_decode()
            return roll_deg if ax.can_id == self._wrist.roll_can else pitch_deg
        return self._pulses_to_deg(ax, self._state[ax.can_id].pulses)

    def _span_deg(self, ax: AxisConfig, pulse_delta: int) -> float:
        """Magnitude of a pulse delta in output degrees, ignoring offset/zero.
        Used for the homing seek-travel safety check."""
        motor_turns = pulse_delta / ax.pulses_per_rev
        return abs((motor_turns / ax.gear_ratio) * 360.0)

    def state_dict(self) -> dict:
        out = {}
        for ax in self.cfg.axes:
            st = self._state[ax.can_id]
            out[ax.name] = {
                "can_id": ax.can_id,
                "pulses": st.pulses,
                "degrees": round(self._axis_degrees(ax), 3),
                "enabled": st.enabled,
                "last_error": st.last_error,
                "is_homed": st.is_homed,
                "homing": st.homing_in_progress,
                "home_enabled": ax.home_enabled,
                "home_error": st.home_error,
                "home_dir": ax.home_dir,
                "home_switch": self.home_switch(ax.can_id),
            }
        return out

    # ---- lifecycle ----

    def enable_all(self, on: bool = True) -> None:
        for ax in self.cfg.axes:
            self.bus.send(Frame(ax.can_id, mks.enable(ax.can_id, on)))
            self._state[ax.can_id].enabled = on

    def emergency_stop(self) -> None:
        log.warning("E-STOP requested")
        # Cancel any in-flight homing; an axis interrupted mid-seek has an
        # indeterminate position, so it loses its homed reference.
        with self._lock:
            cancels = list(self._home_cancel.values())
        for ev in cancels:
            ev.set()
        for ax in self.cfg.axes:
            st = self._state[ax.can_id]
            if st.homing_in_progress:
                st.is_homed = False
            try:
                self.bus.send(Frame(ax.can_id, mks.emergency_stop(ax.can_id)))
            except Exception:
                log.exception("e-stop send failed for axis %d", ax.can_id)

    # ---- jog (speed mode, hold-to-run) ----

    def jog_start(self, can_id: int, direction: int, speed_pct: float) -> None:
        """speed_pct: -1.0..1.0, sign overrides direction.

        Sends 0xF6 speed-mode. Caller must send jog_stop on release.
        """
        if self._wrist is not None and self._wrist.involves(can_id):
            return self._jog_wrist(can_id, direction, speed_pct)
        ax = self.cfg.axis_by_id(can_id)
        # Compose actual direction (XOR invert)
        pct = max(-1.0, min(1.0, speed_pct))
        if pct < 0:
            direction = 1 - direction
            pct = -pct
        if ax.invert:
            direction = 1 - direction
        speed = int(round(ax.max_speed * pct))
        if speed <= 0:
            return self.jog_stop(can_id)
        self.bus.send(Frame(can_id, mks.speed_mode(can_id, direction, speed, ax.default_acc)))

    def _jog_wrist(self, can_id: int, direction: int, speed_pct: float) -> None:
        """Jog one wrist joint by driving BOTH motors: roll -> same direction,
        pitch -> opposite. Equal driver-unit speed on both motors keeps the motion
        in a single mode regardless of the per-joint gear ratios. Each motor's own
        `invert` and the wrist `invert` (pitch handedness) are folded in."""
        w = self._wrist
        joint_ax = self.cfg.axis_by_id(can_id)  # commanded joint -> speed magnitude
        pct = max(-1.0, min(1.0, speed_pct))
        if pct < 0:
            direction = 1 - direction
            pct = -pct
        sj = 1.0 if direction == 1 else -1.0
        # Express the single-joint jog as a velocity in joint-mode space, then run
        # it through the same forward transform the position path uses.
        n_roll = sj if can_id == w.roll_can else 0.0
        n_pitch = sj if can_id == w.pitch_can else 0.0
        vel_a, vel_b = w.forward(n_roll, n_pitch)
        speed = int(round(joint_ax.max_speed * pct))
        if speed <= 0:
            return self.jog_stop(can_id)
        for motor_can, vel in ((w.motor_a, vel_a), (w.motor_b, vel_b)):
            m_ax = self.cfg.axis_by_id(motor_can)
            raw_dir = 1 if vel > 0 else 0
            if m_ax.invert:
                raw_dir = 1 - raw_dir
            self.bus.send(Frame(motor_can, mks.speed_mode(motor_can, raw_dir, speed, m_ax.default_acc)))

    def jog_stop(self, can_id: int) -> None:
        # Speed=0 in 0xF6 stops. Using e-stop here would latch the driver error.
        # A coupled wrist joint runs BOTH motors, so stopping it must stop both —
        # otherwise releasing the jog leaves one wrist motor spinning.
        if self._wrist is not None and self._wrist.involves(can_id):
            for mc in (self._wrist.motor_a, self._wrist.motor_b):
                self.bus.send(Frame(mc, mks.speed_mode(mc, 0, 0, self.cfg.axis_by_id(mc).default_acc)))
            return
        self.bus.send(Frame(can_id, mks.speed_mode(can_id, 0, 0, self.cfg.axis_by_id(can_id).default_acc)))

    def jog_stop_all(self) -> None:
        for ax in self.cfg.axes:
            self.jog_stop(ax.can_id)

    # ---- point-to-point ----

    def _require_homed(self, ax: AxisConfig) -> None:
        """Block absolute motion on an un-homed axis (its position is unknown).
        No-op when the gate is disabled or the axis has no home sensor."""
        if not self.cfg.require_home_before_move or not ax.home_enabled:
            return
        if not self._state[ax.can_id].is_homed:
            raise NotHomedError(f"{ax.name}: not homed — home the axis before absolute moves")

    def move_to_degrees(self, can_id: int, degrees: float, speed_pct: float = 0.5) -> None:
        ax = self.cfg.axis_by_id(can_id)
        self._require_homed(ax)
        if not (ax.soft_limit_min <= degrees <= ax.soft_limit_max):
            raise LimitViolation(f"{ax.name}: {degrees}° outside [{ax.soft_limit_min}, {ax.soft_limit_max}]")
        # A coupled wrist joint can't move alone: hold the partner at its current
        # decoded angle while the differential moves both motors.
        if self._wrist is not None and self._wrist.involves(can_id):
            self._move_wrist({can_id: degrees}, speed_pct)
            return
        target = self._deg_to_pulses(ax, degrees)
        current = self._state[can_id].pulses
        delta = target - current
        direction = 1 if delta >= 0 else 0
        speed = max(1, int(round(ax.max_speed * max(0.01, min(1.0, speed_pct)))))
        payload = mks.position_relative(can_id, direction, speed, ax.default_acc, abs(delta))
        self.bus.send(Frame(can_id, payload))
        # Optimistic update; real pos will be refreshed from reads.
        self._state[can_id].pulses = target

    def _move_wrist(self, wrist_cmds: dict[int, float], speed_pct: float) -> None:
        """Move one or both wrist joints with the differential. A joint not present
        in `wrist_cmds` holds its current decoded angle. Sends exactly TWO 0xFD
        frames (one per motor) and updates both optimistic positions, so a combined
        J5+J6 trajectory point never fans out to four conflicting commands."""
        w = self._wrist
        roll_deg, pitch_deg = self._wrist_decode()  # current pose -> hold default
        if w.roll_can in wrist_cmds:
            roll_deg = wrist_cmds[w.roll_can]
        if w.pitch_can in wrist_cmds:
            pitch_deg = wrist_cmds[w.pitch_can]
        # Both motors share one move speed; the driver paces each to its own delta.
        max_speed = min(self.cfg.axis_by_id(c).max_speed for c in wrist_cmds)
        speed = max(1, int(round(max_speed * max(0.01, min(1.0, speed_pct)))))
        for can_id, target in self._wrist_targets(roll_deg, pitch_deg).items():
            ax = self.cfg.axis_by_id(can_id)
            delta = target - self._state[can_id].pulses
            direction = 1 if delta >= 0 else 0
            payload = mks.position_relative(can_id, direction, speed, ax.default_acc, abs(delta))
            self.bus.send(Frame(can_id, payload))
            self._state[can_id].pulses = target  # optimistic; refreshed from reads

    def move_all_degrees(self, degrees_per_axis: dict[int, float], speed_pct: float = 0.5) -> None:
        # Validate the whole gesture first (homed + soft limits); abort atomically.
        for can_id, deg in degrees_per_axis.items():
            ax = self.cfg.axis_by_id(can_id)
            self._require_homed(ax)
            if not (ax.soft_limit_min <= deg <= ax.soft_limit_max):
                raise LimitViolation(f"{ax.name}: {deg}° outside limits")
        # Collapse the coupled wrist joints into a single two-motor move so the
        # pair isn't sent once per joint (which would fight over both motors).
        handled: set[int] = set()
        if self._wrist is not None:
            wrist_cmds = {c: d for c, d in degrees_per_axis.items() if self._wrist.involves(c)}
            if wrist_cmds:
                self._move_wrist(wrist_cmds, speed_pct)
                handled = set(wrist_cmds)
        for can_id, deg in degrees_per_axis.items():
            if can_id not in handled:
                self.move_to_degrees(can_id, deg, speed_pct)

    # ---- homing (driver-native GoHome, MKS 0x90/0x91) ----

    def home_axis(self, can_id: int) -> None:
        """Start an asynchronous home of one axis and return immediately. Sends
        the driver's set-home params then GoHome and monitors completion on a
        background thread; watch is_homed / homing / home_error via state_dict.
        Raises HomingError on a bad precondition (unknown / not home_enabled /
        already homing).

        Homing is per-MOTOR (pure motor space — seek the switch, zero the encoder)
        and needs no differential awareness. Note that on the wrist this drives one
        motor alone, which physically rotates BOTH J5 and J6; that's expected. The
        coupled joint decode in state_dict only becomes meaningful once both wrist
        motors are homed (see _wrist_decodable)."""
        ax = self.cfg.axis_by_id(can_id)
        if not ax.home_enabled:
            raise HomingError(f"{ax.name}: home_enabled is false")
        st = self._state[can_id]
        with self._lock:
            if st.homing_in_progress:
                raise HomingError(f"{ax.name}: homing already in progress")
            st.homing_in_progress = True
            st.is_homed = False
            st.home_error = None
            st.last_home_status = None
            cancel = threading.Event()
            self._home_cancel[can_id] = cancel
            t = threading.Thread(target=self._home_worker, args=(can_id, cancel),
                                 daemon=True, name=f"home-{ax.name}")
            self._home_threads[can_id] = t
            t.start()

    def _switch_active(self, can_id: int) -> bool:
        """Fresh read of the home switch (True = at the switch, reverse-logic applied)."""
        self.bus.send(Frame(can_id, mks.read_io_status(can_id)))
        time.sleep(0.05)  # MockBus replies synchronously; hardware needs a beat
        return self.home_switch(can_id) is True

    def _read_pulses_fresh(self, can_id: int, retries: int = _FRESH_READ_RETRIES,
                           per_try_s: float = _FRESH_READ_WAIT_S) -> Optional[int]:
        """Send read_pulses and wait for a NEW 0x31 reply (pulses_seq advances),
        re-sending up to `retries` times. Returns the fresh pulse count, or None
        if no reply ever arrived. Replaces the fragile 'send then sleep 50ms then
        trust st.pulses' pattern that mis-zeroed homing when a reply was lost."""
        st = self._state.get(can_id)
        if st is None:
            return None
        for _ in range(retries + 1):
            start_seq = st.pulses_seq
            self.bus.send(Frame(can_id, mks.read_pulses(can_id)))
            deadline = time.monotonic() + per_try_s
            while time.monotonic() < deadline:
                if st.pulses_seq != start_seq:
                    return st.pulses
                time.sleep(0.005)
        return None

    def _io_active_fresh(self, can_id: int) -> bool:
        """Like _switch_active but seq-gated: returns True only from a freshly
        arrived 0x34 reply (False if no fresh reply or switch not active)."""
        st = self._state.get(can_id)
        if st is None:
            return False
        start_seq = st.last_io_seq
        self.bus.send(Frame(can_id, mks.read_io_status(can_id)))
        deadline = time.monotonic() + _FRESH_READ_WAIT_S
        while time.monotonic() < deadline:
            if st.last_io_seq != start_seq:
                return self.home_switch(can_id) is True
            time.sleep(0.005)
        return False

    def _backoff_off_switch(self, can_id: int, ax: AxisConfig, cancel: threading.Event) -> None:
        """If the axis is sitting on its home switch, drive OPPOSITE the seek
        direction until the switch releases — the MKS GoHome won't seek when it
        starts already triggered. Bounded by home_backoff_deg; always stops the
        motor. Raises HomingError if the switch never clears."""
        if ax.home_backoff_deg <= 0 or not self._switch_active(can_id):
            return
        log.info("%s: home switch active at start; backing off up to %.0f°", ax.name, ax.home_backoff_deg)
        # set_home uses home_dir 0=CW/1=CCW; speed_mode uses 0=CCW/1=CW (inverse
        # encodings), so passing home_dir as the speed_mode direction drives the
        # physical opposite of the seek — away from the switch, into valid travel.
        start = self._state[can_id].pulses
        cleared = False
        self.bus.send(Frame(can_id, mks.speed_mode(can_id, ax.home_dir, ax.home_speed, ax.default_acc)))
        try:
            deadline = time.monotonic() + _HOME_TIMEOUT_S
            while time.monotonic() < deadline:
                if cancel.is_set():
                    return
                self.bus.send(Frame(can_id, mks.read_io_status(can_id)))
                self.bus.send(Frame(can_id, mks.read_pulses(can_id)))
                time.sleep(_HOME_POLL_S / 2)
                if self.home_switch(can_id) is not True:
                    cleared = True
                    break
                if self._span_deg(ax, self._state[can_id].pulses - start) > ax.home_backoff_deg:
                    break
        finally:
            self.jog_stop(can_id)  # never leave the axis running
        if not cleared:
            raise HomingError(
                f"{ax.name}: home switch still active after {ax.home_backoff_deg}° back-off "
                "— check home_dir (seek direction) and the sensor")
        time.sleep(0.3)  # settle off the switch before GoHome seeks back into it

    def _home_worker(self, can_id: int, cancel: threading.Event) -> None:
        ax = self.cfg.axis_by_id(can_id)
        st = self._state[can_id]
        try:
            if not ax.home_use_driver_params:
                # Clear the switch first if we're already on it, so GoHome can seek.
                self._backoff_off_switch(can_id, ax, cancel)
                if cancel.is_set():
                    st.home_error = "canceled"
                    return
            # Anchor the seek-travel guard and motion detection to a fresh read.
            fresh = self._read_pulses_fresh(can_id)
            start_pulses = fresh if fresh is not None else st.pulses
            if not ax.home_use_driver_params:
                trig = 0 if ax.home_trig_low else 1  # active-low switch -> homeTrig=Low(0)
                self.bus.send(Frame(can_id, mks.set_home(can_id, trig, ax.home_dir,
                                                         ax.home_speed, ax.end_limit)))
                time.sleep(0.05)  # let the driver store params before triggering
            # else: trigger the driver's OWN flashed home params (panel-button
            # equivalent). Overwriting them with our 0x90 is what made J3 show
            # "home" on the LCD but not seek.
            self.bus.send(Frame(can_id, mks.go_home(can_id)))

            deadline = time.monotonic() + _HOME_TIMEOUT_S
            settled_count = 0
            last_seen = start_pulses
            moved = False
            completed_by = None
            while True:
                if cancel.is_set():
                    self._stop_one(can_id)
                    st.home_error = "canceled"
                    return
                # Check terminal status before the travel guard so a fast
                # success (e.g. MockBus) isn't masked by the abort check.
                status = st.last_home_status
                if status == mks.GO_HOME_SUCCESS:
                    completed_by = "GoHome success"
                    break
                if status == mks.GO_HOME_FAIL:
                    self._stop_one(can_id)
                    st.home_error = "driver reported GoHome failure"
                    return
                if time.monotonic() > deadline:
                    self._stop_one(can_id)
                    st.home_error = "home timed out"
                    return
                # Poll a fresh position; enforce the seek-travel safety bound.
                cur = self._read_pulses_fresh(can_id, retries=1)
                if cur is None:
                    time.sleep(_HOME_POLL_S)  # no reply this tick; keep waiting
                    continue
                if self._span_deg(ax, cur - start_pulses) > ax.home_seek_max_deg:
                    self._stop_one(can_id)
                    st.home_error = (f"seek exceeded {ax.home_seek_max_deg}° "
                                     "without reaching the switch")
                    return
                # Fallback completion when the 0x91 SUCCESS frame is lost: the
                # axis SEEKED (moved) and then STOPPED (settled). We do NOT
                # require the home-switch read here — its polarity (home_trig_low)
                # may be miswired, which is exactly why a finished J3 seek wasn't
                # recognized. The switch is kept only as a secondary signal for
                # the rare "already parked on the switch, never moved" case.
                if abs(cur - last_seen) <= _SETTLE_TOL_PULSES:
                    settled_count += 1
                else:
                    settled_count = 0
                    moved = True
                last_seen = cur
                if settled_count >= _SETTLE_POLLS and (moved or self._io_active_fresh(can_id)):
                    completed_by = "moved then settled" if moved else "settled at switch"
                    break
                time.sleep(_HOME_POLL_S)

            # Deterministically zero the driver's counter at the switch (0x92,
            # no motion — the axis is already settled there) and confirm with a
            # fresh read-back, so home_pulse_zero is anchored to a value we
            # actually observed, not a stale or lost read.
            self.bus.send(Frame(can_id, mks.set_axis_zero(can_id)))
            time.sleep(0.05)  # let the driver apply the zero
            val = self._read_pulses_fresh(can_id)
            if val is None:
                self._stop_one(can_id)
                st.home_error = "homed but no pulse read-back after zeroing"
                st.is_homed = False
                return
            if abs(val) > _ZERO_TOL_PULSES:
                log.warning("%s: counter not zeroed after 0x92 (read %d); "
                            "using it as pulse_zero", ax.name, val)
            st.home_pulse_zero = val
            st.is_homed = True
            log.info("%s homed via %s (pulse_zero=%d)", ax.name, completed_by, val)
        except Exception as exc:
            log.exception("homing failed for %s", ax.name)
            st.home_error = str(exc)
            st.is_homed = False
        finally:
            st.homing_in_progress = False
            with self._lock:
                self._home_threads.pop(can_id, None)
                self._home_cancel.pop(can_id, None)

    def _stop_one(self, can_id: int) -> None:
        """Stop a single axis (e-stop) — used to abort a runaway/failed seek."""
        try:
            self.bus.send(Frame(can_id, mks.emergency_stop(can_id)))
        except Exception:
            log.exception("stop failed for axis %d", can_id)

    def home_all(self) -> None:
        """Home every home_enabled axis sequentially in home_order, on a single
        background thread. Returns immediately."""
        with self._lock:
            if self.homing_all_in_progress:
                raise HomingError("home-all already in progress")
            self.homing_all_in_progress = True
            t = threading.Thread(target=self._home_all_worker, daemon=True, name="home-all")
            self._home_all_thread = t
            t.start()

    def _home_all_worker(self) -> None:
        try:
            order = {ax.can_id: i for i, ax in enumerate(self.cfg.axes)}
            axes = sorted((ax for ax in self.cfg.axes if ax.home_enabled),
                          key=lambda a: (a.home_order, order[a.can_id]))
            for ax in axes:
                try:
                    self.home_axis(ax.can_id)
                except HomingError:
                    continue
                t = self._home_threads.get(ax.can_id)
                if t is not None:
                    t.join(timeout=_HOME_TIMEOUT_S + 5.0)
        finally:
            self.homing_all_in_progress = False

    def read_io(self, can_id: int) -> Optional[dict]:
        """Read the driver IO status (0x34) and return the decoded levels plus the
        home-switch state with active-low applied. For the bench wiring check."""
        self.cfg.axis_by_id(can_id)  # validate can_id (raises KeyError if unknown)
        self.bus.send(Frame(can_id, mks.read_io_status(can_id)))
        time.sleep(0.05)  # MockBus replies synchronously; hardware needs a beat
        st = self._state.get(can_id)
        io = dict(st.last_io) if st is not None and st.last_io else None
        if io is not None:
            io["home_switch"] = self.home_switch(can_id)
        return io

    # ---- driver config (persists to flash) ----

    def set_microsteps(self, can_id: int, microsteps: int) -> None:
        self._require_flash_ok()
        self.bus.send(Frame(can_id, mks.set_microsteps(can_id, microsteps)))
        ax = self.cfg.axis_by_id(can_id)
        # Pulses-per-rev scales with microstepping; keep in-memory config coherent.
        if ax.default_microsteps:
            ax.pulses_per_rev = int(ax.pulses_per_rev * microsteps / ax.default_microsteps)
        ax.default_microsteps = microsteps

    def set_current(self, can_id: int, milliamps: int) -> None:
        self._require_flash_ok()
        self.bus.send(Frame(can_id, mks.set_current(can_id, milliamps)))
        self.cfg.axis_by_id(can_id).default_current_ma = milliamps

    def set_work_mode(self, can_id: int, mode: int) -> None:
        self._require_flash_ok()
        self.bus.send(Frame(can_id, mks.set_work_mode(can_id, mode)))

    def set_gear_ratio(self, can_id: int, ratio: float) -> None:
        """Live-adjust an axis's gear ratio (output turns per motor turn) — the
        scale factor in the degrees<->pulses conversion. SOFTWARE-ONLY: it never
        touches the driver or its flash, so it's safe in dry_run and takes effect
        on the next move. A bench tuning aid for matching MoveIt/jog motion to the
        real robot before committing the value to config.yaml."""
        if ratio <= 0:
            raise ValueError(f"gear_ratio must be > 0 (got {ratio})")
        self.cfg.axis_by_id(can_id).gear_ratio = float(ratio)

    def set_home_offset(self, can_id: int, offset_deg: float) -> None:
        """Live-adjust an axis's home offset (the joint angle assigned to the
        home-switch / encoder-zero position). SOFTWARE-ONLY like set_gear_ratio:
        shifts the reported angle and absolute-move targets by the same amount,
        never touches the driver."""
        self.cfg.axis_by_id(can_id).home_offset_deg = float(offset_deg)

    def calibrate_joint_zero(self, can_id: int, true_angle_deg: float = 0.0) -> float:
        """Declare "this joint is physically at true_angle_deg RIGHT NOW" and
        set home_offset_deg so the reported angle matches. The offset enters
        every conversion linearly (including the coupled wrist decode), so
        new_offset = old_offset + (true - reported). Returns the new offset —
        copy it into config.yaml as home_offset_deg to keep it across restarts.
        Most meaningful on a homed axis (the reading is then switch-anchored);
        on an un-homed axis it calibrates against the optimistic position."""
        ax = self.cfg.axis_by_id(can_id)
        reported = self._axis_degrees(ax)
        new_offset = ax.home_offset_deg + (float(true_angle_deg) - reported)
        ax.home_offset_deg = new_offset
        return new_offset

    def _require_flash_ok(self) -> None:
        from .can_bus import DryRunBus
        if isinstance(self.bus, DryRunBus) and not self.allow_flash_writes:
            raise PermissionError(
                "Flash-persisting writes are blocked in dry_run mode. "
                "Set motion.allow_flash_writes=True to override."
            )

    # ---- polling ----

    def request_all_positions(self) -> None:
        # Skip axes mid-home: their home worker is already polling pulses, so a
        # second reader here just adds bus traffic during the busiest moment.
        for ax in self.cfg.axes:
            if self._state[ax.can_id].homing_in_progress:
                continue
            self.bus.send(Frame(ax.can_id, mks.read_pulses(ax.can_id)))

    def request_all_io(self) -> None:
        """Fire-and-forget IO-status reads for every axis; replies update
        _state[].last_io via _on_frame. Used to surface live home-switch state."""
        # Skip axes mid-home (the home worker reads their IO) — see
        # request_all_positions.
        for ax in self.cfg.axes:
            if self._state[ax.can_id].homing_in_progress:
                continue
            self.bus.send(Frame(ax.can_id, mks.read_io_status(ax.can_id)))

    def home_switch(self, can_id: int) -> Optional[bool]:
        """Home-switch state with the axis's reverse-logic applied (True = at the
        switch), from the last cached IO read; None if no read seen yet."""
        st = self._state.get(can_id)
        if st is None or st.last_io is None:
            return None
        ax = self.cfg.axis_by_id(can_id)
        return (not st.last_io["in_1"]) if ax.home_trig_low else st.last_io["in_1"]

    def _on_frame(self, frame: Frame) -> None:
        if len(frame.data) < 3:
            # Real MKS replies are >=3 bytes ([cmd][data..][crc]). Shorter frames
            # are our own 2-byte read requests looped back as the bus drops —
            # ignore them so they don't flood the log with parse errors.
            return
        cmd = frame.data[0]
        can_id = frame.arbitration_id
        st = self._state.get(can_id)
        try:
            if cmd == 0x31:
                pulses = mks.parse_pulses(can_id, frame.data)
                if st is not None:
                    st.pulses = pulses
                    st.pulses_seq += 1
            elif cmd == mks.Cmd.GO_HOME:
                status = mks.parse_status(can_id, frame.data, expected_cmd=cmd)
                if st is not None:
                    st.last_home_status = status
            elif cmd in (mks.Cmd.SET_HOME, mks.Cmd.SET_AXIS_ZERO):
                mks.parse_status(can_id, frame.data, expected_cmd=cmd)  # validate; no state
            elif cmd == mks.Cmd.READ_IO_STATUS:
                if st is not None:
                    st.last_io = mks.parse_io_status(can_id, frame.data)
                    st.last_io_seq += 1
        except Exception:
            log.exception("frame parse failed for %r", frame)
