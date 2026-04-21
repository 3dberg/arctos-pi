"""Program loading + queue.

STATUS: scaffold for phase 5 — file-format and queue model sketched,
runner implementation deferred until after phase 6 hardware bring-up.

Supported formats:
  * JSON waypoint program (native):
        { "name": str, "version": 1,
          "waypoints": [{ "joints": {"J1": deg, ...}, "dwell_ms": int, "speed_pct": float }, ...] }
  * Legacy CAN-frame .txt (one hex frame per line, arctosgui style)
  * Legacy g-code .tap (G90 + Fxxxx per arctosgui convert.py)

Queue model:
  [ { "program": str, "repeat": int }, ... ]

Runner lifecycle: idle -> running -> paused -> running -> done (or aborted).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class Program:
    name: str
    waypoints: list[dict[str, Any]]
    source: str = "json"   # json | tap | txt

    @classmethod
    def from_json(cls, path: Path) -> "Program":
        data = json.loads(path.read_text())
        return cls(name=data.get("name", path.stem), waypoints=data.get("waypoints", []), source="json")

    def to_json(self, path: Path) -> None:
        path.write_text(json.dumps(
            {"name": self.name, "version": 1, "waypoints": self.waypoints}, indent=2
        ))


@dataclass
class QueueEntry:
    program: str
    repeat: int = 1


@dataclass
class ProgramQueue:
    entries: list[QueueEntry] = field(default_factory=list)
    current_index: int = 0
    current_repeat: int = 0
    status: str = "idle"   # idle | running | paused | done | aborted

    def add(self, name: str, repeat: int = 1) -> None:
        self.entries.append(QueueEntry(program=name, repeat=repeat))

    def clear(self) -> None:
        self.entries.clear()
        self.current_index = 0
        self.current_repeat = 0
        self.status = "idle"


# --- Legacy loaders (sketched; finalize in phase 5) ---

def load_legacy_txt(path: Path) -> list[bytes]:
    """Each line like '01F40064020000006F' -> raw frame bytes (including CRC).
    Returned as list of full frames; sender splits ID byte from data payload.
    """
    frames = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        frames.append(bytes.fromhex(line))
    return frames
