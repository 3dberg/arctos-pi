"""Runtime configuration model."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


@dataclass
class AxisConfig:
    can_id: int
    name: str
    gear_ratio: float = 1.0            # output turn per motor turn
    pulses_per_rev: int = 6400          # at current microstep setting (1.8° * 16 = 3200; 32 = 6400)
    invert: bool = False                # flip direction in software
    max_speed: int = 1500               # 0xF6 / 0xFD speed units (0..3000)
    default_acc: int = 2                # 0..255
    soft_limit_min: float = -360.0      # degrees at output shaft
    soft_limit_max: float = 360.0
    default_current_ma: int = 1600      # for SERVO42D clamp at 3000; 57D at 5200
    default_microsteps: int = 16
    # Driver work mode (0x82): 0=CR_OPEN 1=CR_CLOSE 2=CR_vFOC 3=SR_OPEN 4=SR_CLOSE
    # 5=SR_vFOC. None = leave the driver's flashed mode as-is. Set 5 (SR_vFOC) to
    # stop open-loop hum/heat on axes that drifted into a non-FOC mode. Applied
    # ONCE at bridge startup (flash write) only when not None.
    default_work_mode: Optional[int] = None
    hold_current_ma: Optional[int] = None  # optional reduced holding current; None = use default_current_ma
    # ROS2 / MoveIt joint limits at the output shaft. Used only by the ROS2
    # export (backend/ros_export.py); the CAN control path ignores them.
    max_vel_deg_s: float = 90.0         # joint velocity limit, deg/s
    max_acc_deg_s2: float = 180.0       # joint acceleration limit, deg/s^2
    # ---- Homing (driver-native GoHome via MKS 0x90/0x91) ----
    home_enabled: bool = True           # axis has a home sensor; must be homed before absolute moves
    home_dir: int = 0                   # seek direction: 0=CW 1=CCW (RAW driver dir, NOT software-inverted)
    home_speed: int = 200               # seek speed, 0..4095 (12-bit driver units; keep modest)
    home_trig_low: bool = True          # reverse/active-low switch -> driver homeTrig=Low(0)
    home_offset_deg: float = 0.0        # output-shaft angle assigned to the switch position
    home_seek_max_deg: float = 400.0    # safety: abort + e-stop if a seek travels past this
    home_backoff_deg: float = 30.0      # if the switch is already active, back off this far (max) to clear it before GoHome; 0 disables
    home_order: int = 0                 # Home-All sequence (lower runs first; ties keep config order)
    end_limit: bool = True              # 0x90 endLimit param (driver's own limit feature)
    # Replicate the driver's own panel-button home: trigger GoHome (0x91) with
    # the params already flashed on the driver, sending NO 0x90 and skipping the
    # back-off. Use when an axis homes fine from the MKS controller but not from
    # the UI (our home_dir/home_trig/end_limit don't match the driver's setup).
    home_use_driver_params: bool = False


@dataclass(frozen=True)
class WristDifferential:
    """Two motors driving J5/J6 through a differential wrist.

    The SUM of the two motor rotations turns the *roll* joint (both motors spin
    the same way); their DIFFERENCE turns the *pitch* joint (they spin opposite
    ways). All four fields are CAN ids on the bus — on the Arctos the two wrist
    motors ARE the two wrist joints, so {motor_a, motor_b} == {roll_can, pitch_can}.

    `motor_a` is the "+pitch" reference motor: for a positive pitch command it
    advances while `motor_b` retreats. The transform here is purely numeric (motor
    *turns* in / out); per-motor pulses_per_rev, invert and home zero are applied
    by Motion, which owns the live state. `invert` flips the pitch handedness for a
    mirror-imaged build (see config docs for the bench calibration procedure).
    """
    motor_a: int
    motor_b: int
    roll_can: int
    pitch_can: int
    invert: bool = False

    def involves(self, can_id: int) -> bool:
        return can_id in (self.motor_a, self.motor_b)

    def partner(self, can_id: int) -> int:
        """The other wrist joint's can_id."""
        if can_id == self.roll_can:
            return self.pitch_can
        if can_id == self.pitch_can:
            return self.roll_can
        raise KeyError(f"{can_id} is not a wrist joint")

    def forward(self, n_roll: float, n_pitch: float) -> tuple[float, float]:
        """Joint-mode motor turns (roll, pitch) -> (motor_a turns, motor_b turns)."""
        p = -n_pitch if self.invert else n_pitch
        return n_roll + p, n_roll - p

    def inverse(self, turns_a: float, turns_b: float) -> tuple[float, float]:
        """(motor_a turns, motor_b turns) -> joint-mode motor turns (roll, pitch).
        Exact inverse of forward(); determinant is -2, so always invertible."""
        n_roll = (turns_a + turns_b) / 2.0
        p = (turns_a - turns_b) / 2.0
        return n_roll, (-p if self.invert else p)


@dataclass
class CanConfig:
    backend: str = "mock"               # slcan | socketcan | mock | dry_run
    channel: Optional[str] = None       # autodetect when None and backend=slcan; 'can0' etc. for socketcan
    bitrate: int = 500_000


@dataclass
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8000
    heartbeat_ms: int = 200             # WS heartbeat; missed -> stop all motion


@dataclass
class GripperConfig:
    """CAN-attached Arduino-driven servo gripper. Wire format: 1 byte payload
    (0..255) on `can_id`; the MCU maps to servo travel.
    """
    enabled: bool = False
    can_id: int = 0x07
    open_position: int = 0              # raw byte sent for "open"
    close_position: int = 255           # raw byte sent for "close"
    default_position: int = 0           # initial commanded position on boot


@dataclass
class AppConfig:
    can: CanConfig = field(default_factory=CanConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    axes: list[AxisConfig] = field(default_factory=list)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    # Optional differential wrist coupling for J5/J6 (two motors -> two joints).
    # None = no coupling: every joint is treated 1:1 with its motor (J1-J4 always).
    wrist_differential: Optional[WristDifferential] = None
    # Selects the robot model. The ROS2 launch/registry layer maps this to a
    # description / MoveIt bundle (see ros2_ws/src/arctos_robots). The CAN
    # control path is robot-agnostic and only uses `axes`.
    robot_type: str = "arctos"
    # Safety gate: refuse absolute moves (move_to_degrees / MoveIt joint goals /
    # program replay) for any home_enabled axis that has not been homed yet.
    # Manual jog stays available so the operator can reach the switch. Set False
    # for bench bring-up before homing params are tuned.
    require_home_before_move: bool = True

    @staticmethod
    def default_six_axis() -> "AppConfig":
        return AppConfig(
            axes=[AxisConfig(can_id=i, name=f"J{i}") for i in range(1, 7)]
        )

    @classmethod
    def load(cls, path: Path) -> "AppConfig":
        if not path.exists():
            return cls.default_six_axis()
        raw = yaml.safe_load(path.read_text()) or {}
        can = CanConfig(**raw.get("can", {}))
        server = ServerConfig(**raw.get("server", {}))
        axes = [AxisConfig(**a) for a in raw.get("axes", [])]
        gripper = GripperConfig(**raw.get("gripper", {}))
        robot_type = raw.get("robot_type", "arctos")
        require_home = raw.get("require_home_before_move", True)
        if not axes:
            axes = cls.default_six_axis().axes
        wrist = cls._parse_wrist(raw.get("wrist_differential"), axes)
        return cls(can=can, server=server, axes=axes, gripper=gripper,
                   robot_type=robot_type, require_home_before_move=require_home,
                   wrist_differential=wrist)

    @staticmethod
    def _parse_wrist(raw: Optional[dict], axes: list[AxisConfig]) -> Optional[WristDifferential]:
        """Build the wrist coupling from the YAML block, resolving joint names (or
        raw can_ids) to can_ids. Returns None when the section is absent or
        disabled, so an un-coupled robot is byte-for-byte unchanged."""
        if not raw or not raw.get("enabled"):
            return None
        by_name = {ax.name: ax.can_id for ax in axes}
        ids = {ax.can_id for ax in axes}

        def _resolve(key: str) -> int:
            if key not in raw:
                raise ValueError(f"wrist_differential.{key} is required when enabled")
            v = raw[key]
            if isinstance(v, str):
                if v not in by_name:
                    raise ValueError(f"wrist_differential.{key}={v!r} is not a known axis name")
                return by_name[v]
            if int(v) not in ids:
                raise ValueError(f"wrist_differential.{key}={v} is not a known axis can_id")
            return int(v)

        w = WristDifferential(
            motor_a=_resolve("motor_a"),
            motor_b=_resolve("motor_b"),
            roll_can=_resolve("roll_joint"),
            pitch_can=_resolve("pitch_joint"),
            invert=bool(raw.get("invert", False)),
        )
        motors = {w.motor_a, w.motor_b}
        if len(motors) != 2 or {w.roll_can, w.pitch_can} != motors:
            raise ValueError(
                "wrist_differential: motor_a/motor_b must be two distinct axes and "
                "equal {roll_joint, pitch_joint} (the two wrist motors are the two "
                "wrist joints)")
        return w

    def axis_by_id(self, can_id: int) -> AxisConfig:
        for ax in self.axes:
            if ax.can_id == can_id:
                return ax
        raise KeyError(f"no axis with can_id={can_id}")
