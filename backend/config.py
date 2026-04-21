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
class AppConfig:
    can: CanConfig = field(default_factory=CanConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    axes: list[AxisConfig] = field(default_factory=list)

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
        if not axes:
            axes = cls.default_six_axis().axes
        return cls(can=can, server=server, axes=axes)

    def axis_by_id(self, can_id: int) -> AxisConfig:
        for ax in self.axes:
            if ax.can_id == can_id:
                return ax
        raise KeyError(f"no axis with can_id={can_id}")
