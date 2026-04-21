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
from dataclasses import dataclass, field
from typing import Optional

from . import mks
from .can_bus import CanBus, Frame
from .config import AppConfig, AxisConfig

log = logging.getLogger(__name__)


class LimitViolation(ValueError):
    pass


@dataclass
class AxisState:
    can_id: int
    pulses: int = 0
    enabled: bool = False
    last_error: Optional[str] = None


@dataclass
class Motion:
    cfg: AppConfig
    bus: CanBus
    _state: dict[int, AxisState] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    allow_flash_writes: bool = False

    def __post_init__(self) -> None:
        for ax in self.cfg.axes:
            self._state[ax.can_id] = AxisState(can_id=ax.can_id)
        self.bus.on_receive(self._on_frame)

    # ---- conversions ----

    def _deg_to_pulses(self, ax: AxisConfig, degrees: float) -> int:
        motor_turns = (degrees / 360.0) * ax.gear_ratio
        pulses = int(round(motor_turns * ax.pulses_per_rev))
        return -pulses if ax.invert else pulses

    def _pulses_to_deg(self, ax: AxisConfig, pulses: int) -> float:
        if ax.invert:
            pulses = -pulses
        motor_turns = pulses / ax.pulses_per_rev
        return (motor_turns / ax.gear_ratio) * 360.0

    def state_dict(self) -> dict:
        out = {}
        for ax in self.cfg.axes:
            st = self._state[ax.can_id]
            out[ax.name] = {
                "can_id": ax.can_id,
                "pulses": st.pulses,
                "degrees": round(self._pulses_to_deg(ax, st.pulses), 3),
                "enabled": st.enabled,
                "last_error": st.last_error,
            }
        return out

    # ---- lifecycle ----

    def enable_all(self, on: bool = True) -> None:
        for ax in self.cfg.axes:
            self.bus.send(Frame(ax.can_id, mks.enable(ax.can_id, on)))
            self._state[ax.can_id].enabled = on

    def emergency_stop(self) -> None:
        log.warning("E-STOP requested")
        for ax in self.cfg.axes:
            try:
                self.bus.send(Frame(ax.can_id, mks.emergency_stop(ax.can_id)))
            except Exception:
                log.exception("e-stop send failed for axis %d", ax.can_id)

    # ---- jog (speed mode, hold-to-run) ----

    def jog_start(self, can_id: int, direction: int, speed_pct: float) -> None:
        """speed_pct: -1.0..1.0, sign overrides direction.

        Sends 0xF6 speed-mode. Caller must send jog_stop on release.
        """
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

    def jog_stop(self, can_id: int) -> None:
        # Speed=0 in 0xF6 stops. Using e-stop here would latch the driver error.
        self.bus.send(Frame(can_id, mks.speed_mode(can_id, 0, 0, self.cfg.axis_by_id(can_id).default_acc)))

    def jog_stop_all(self) -> None:
        for ax in self.cfg.axes:
            self.jog_stop(ax.can_id)

    # ---- point-to-point ----

    def move_to_degrees(self, can_id: int, degrees: float, speed_pct: float = 0.5) -> None:
        ax = self.cfg.axis_by_id(can_id)
        if not (ax.soft_limit_min <= degrees <= ax.soft_limit_max):
            raise LimitViolation(f"{ax.name}: {degrees}° outside [{ax.soft_limit_min}, {ax.soft_limit_max}]")
        target = self._deg_to_pulses(ax, degrees)
        current = self._state[can_id].pulses
        delta = target - current
        direction = 1 if delta >= 0 else 0
        speed = max(1, int(round(ax.max_speed * max(0.01, min(1.0, speed_pct)))))
        payload = mks.position_relative(can_id, direction, speed, ax.default_acc, abs(delta))
        self.bus.send(Frame(can_id, payload))
        # Optimistic update; real pos will be refreshed from reads.
        self._state[can_id].pulses = target

    def move_all_degrees(self, degrees_per_axis: dict[int, float], speed_pct: float = 0.5) -> None:
        # Soft-limit check whole gesture first; abort atomically.
        for can_id, deg in degrees_per_axis.items():
            ax = self.cfg.axis_by_id(can_id)
            if not (ax.soft_limit_min <= deg <= ax.soft_limit_max):
                raise LimitViolation(f"{ax.name}: {deg}° outside limits")
        for can_id, deg in degrees_per_axis.items():
            self.move_to_degrees(can_id, deg, speed_pct)

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

    def _require_flash_ok(self) -> None:
        from .can_bus import DryRunBus
        if isinstance(self.bus, DryRunBus) and not self.allow_flash_writes:
            raise PermissionError(
                "Flash-persisting writes are blocked in dry_run mode. "
                "Set motion.allow_flash_writes=True to override."
            )

    # ---- polling ----

    def request_all_positions(self) -> None:
        for ax in self.cfg.axes:
            self.bus.send(Frame(ax.can_id, mks.read_pulses(ax.can_id)))

    def _on_frame(self, frame: Frame) -> None:
        if not frame.data:
            return
        cmd = frame.data[0]
        try:
            if cmd == 0x31:
                pulses = mks.parse_pulses(frame.arbitration_id, frame.data)
                st = self._state.get(frame.arbitration_id)
                if st is not None:
                    st.pulses = pulses
        except Exception:
            log.exception("frame parse failed for %r", frame)
