"""Teach / record mode — Phase 4.

Captures live joint (+ optional gripper) snapshots into an in-memory
waypoint list, supports list editing, and persists to JSON files under
``~/.arctos/programs/``. Replay belongs to Phase 5 (program queue).

Program JSON schema (version 1):

    {
      "name": "<program name>",
      "version": 1,
      "waypoints": [
        {
          "joints": {"J1": <deg>, "J2": <deg>, ...},
          "dwell_ms": <int>,
          "speed_pct": <float 0..1>,
          "gripper": <int 0..255>     # optional, present when gripper enabled
        },
        ...
      ]
    }

The 10 Hz background polling described in the original scaffold is
deferred — explicit "Capture waypoint" is the primary teach UX. The
data shape leaves room (``t_ms`` on the Waypoint) for adding it later
without breaking saved files.
"""
from __future__ import annotations

import json
import re
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from .gripper import Gripper
from .motion import Motion


SCHEMA_VERSION = 1
DEFAULT_PROGRAMS_DIR = Path.home() / ".arctos" / "programs"
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")


class TeachError(ValueError):
    """Raised for invalid teach mutations (bad index, bad name, etc.)."""


@dataclass
class Waypoint:
    joints: dict[str, float]
    dwell_ms: int = 0
    speed_pct: float = 0.5
    gripper: Optional[int] = None
    t_ms: Optional[int] = None  # reserved for continuous-record mode

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "joints": dict(self.joints),
            "dwell_ms": int(self.dwell_ms),
            "speed_pct": float(self.speed_pct),
        }
        if self.gripper is not None:
            out["gripper"] = int(self.gripper)
        if self.t_ms is not None:
            out["t_ms"] = int(self.t_ms)
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Waypoint":
        joints = data.get("joints") or {}
        if not isinstance(joints, dict):
            raise TeachError("waypoint.joints must be an object")
        return cls(
            joints={str(k): float(v) for k, v in joints.items()},
            dwell_ms=int(data.get("dwell_ms", 0)),
            speed_pct=float(data.get("speed_pct", 0.5)),
            gripper=int(data["gripper"]) if "gripper" in data and data["gripper"] is not None else None,
            t_ms=int(data["t_ms"]) if "t_ms" in data and data["t_ms"] is not None else None,
        )


def _validate_name(name: str) -> str:
    name = name.strip()
    if not name:
        raise TeachError("program name is empty")
    if not _NAME_RE.match(name):
        raise TeachError(
            "program name must match [A-Za-z0-9._-]+ (no spaces, slashes, or dots-only)"
        )
    if name in (".", "..") or name.startswith("."):
        raise TeachError("program name may not start with '.'")
    return name


@dataclass
class TeachRecorder:
    motion: Motion
    gripper: Optional[Gripper] = None
    programs_dir: Path = field(default_factory=lambda: DEFAULT_PROGRAMS_DIR)
    waypoints: list[Waypoint] = field(default_factory=list)
    loaded_name: Optional[str] = None
    dirty: bool = False
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def __post_init__(self) -> None:
        self.programs_dir = Path(self.programs_dir).expanduser()

    # ---- file system ----

    def _ensure_dir(self) -> Path:
        self.programs_dir.mkdir(parents=True, exist_ok=True)
        return self.programs_dir

    def _path_for(self, name: str) -> Path:
        return self._ensure_dir() / f"{_validate_name(name)}.json"

    def list_programs(self) -> list[str]:
        if not self.programs_dir.exists():
            return []
        return sorted(p.stem for p in self.programs_dir.glob("*.json") if p.is_file())

    # ---- snapshot helpers ----

    def _snapshot_joints(self) -> dict[str, float]:
        st = self.motion.state_dict()
        return {name: float(s["degrees"]) for name, s in st.items()}

    def _snapshot_gripper(self) -> Optional[int]:
        if self.gripper is None or not self.gripper.cfg.enabled:
            return None
        return int(self.gripper.position)

    # ---- editing ----

    def capture(self, dwell_ms: int = 0, speed_pct: float = 0.5) -> Waypoint:
        if not 0 <= int(dwell_ms) <= 600_000:
            raise TeachError("dwell_ms must be in [0, 600000]")
        if not 0.0 < float(speed_pct) <= 1.0:
            raise TeachError("speed_pct must be in (0, 1]")
        wp = Waypoint(
            joints=self._snapshot_joints(),
            dwell_ms=int(dwell_ms),
            speed_pct=float(speed_pct),
            gripper=self._snapshot_gripper(),
        )
        with self._lock:
            self.waypoints.append(wp)
            self.dirty = True
        return wp

    def delete(self, index: int) -> Waypoint:
        with self._lock:
            self._check_index(index)
            wp = self.waypoints.pop(index)
            self.dirty = True
            return wp

    def reorder(self, from_index: int, to_index: int) -> None:
        with self._lock:
            self._check_index(from_index)
            if not 0 <= to_index < len(self.waypoints):
                raise TeachError(f"to_index {to_index} out of range")
            wp = self.waypoints.pop(from_index)
            self.waypoints.insert(to_index, wp)
            self.dirty = True

    def update(
        self,
        index: int,
        dwell_ms: Optional[int] = None,
        speed_pct: Optional[float] = None,
        gripper: Optional[int] = None,
    ) -> Waypoint:
        with self._lock:
            self._check_index(index)
            wp = self.waypoints[index]
            if dwell_ms is not None:
                if not 0 <= int(dwell_ms) <= 600_000:
                    raise TeachError("dwell_ms must be in [0, 600000]")
                wp.dwell_ms = int(dwell_ms)
            if speed_pct is not None:
                if not 0.0 < float(speed_pct) <= 1.0:
                    raise TeachError("speed_pct must be in (0, 1]")
                wp.speed_pct = float(speed_pct)
            if gripper is not None:
                if not 0 <= int(gripper) <= 255:
                    raise TeachError("gripper must be in [0, 255]")
                wp.gripper = int(gripper)
            self.dirty = True
            return wp

    def clear(self) -> None:
        with self._lock:
            self.waypoints.clear()
            self.loaded_name = None
            self.dirty = False

    def _check_index(self, index: int) -> None:
        if not 0 <= index < len(self.waypoints):
            raise TeachError(f"waypoint index {index} out of range (have {len(self.waypoints)})")

    # ---- JSON I/O ----

    def to_program_dict(self, name: Optional[str] = None) -> dict[str, Any]:
        return {
            "name": name or self.loaded_name or "untitled",
            "version": SCHEMA_VERSION,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
        }

    def load_program_dict(self, data: dict[str, Any]) -> None:
        if not isinstance(data, dict):
            raise TeachError("program JSON must be an object")
        version = int(data.get("version", 1))
        if version != SCHEMA_VERSION:
            raise TeachError(f"unsupported program schema version: {version}")
        wps_raw = data.get("waypoints") or []
        if not isinstance(wps_raw, list):
            raise TeachError("program.waypoints must be a list")
        wps = [Waypoint.from_dict(w) for w in wps_raw]
        with self._lock:
            self.waypoints = wps
            self.loaded_name = data.get("name") or None
            self.dirty = False

    def save(self, name: str) -> Path:
        path = self._path_for(name)
        path.write_text(json.dumps(self.to_program_dict(name=name), indent=2))
        with self._lock:
            self.loaded_name = _validate_name(name)
            self.dirty = False
        return path

    def load(self, name: str) -> None:
        path = self._path_for(name)
        if not path.exists():
            raise TeachError(f"program '{name}' not found")
        data = json.loads(path.read_text())
        self.load_program_dict(data)
        # _path_for ran the validator, but loaded_name should match the
        # filename specifically (not whatever the JSON's "name" field said).
        self.loaded_name = _validate_name(name)

    def delete_program(self, name: str) -> None:
        path = self._path_for(name)
        if not path.exists():
            raise TeachError(f"program '{name}' not found")
        path.unlink()
        if self.loaded_name == _validate_name(name):
            self.loaded_name = None

    # ---- summary ----

    def state_dict(self) -> dict[str, Any]:
        return {
            "count": len(self.waypoints),
            "loaded_name": self.loaded_name,
            "dirty": self.dirty,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
        }
