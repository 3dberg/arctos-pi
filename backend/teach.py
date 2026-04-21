"""Teach / record mode.

STATUS: scaffold for phase 4 — API-level stubs present, implementation
deferred until after phase 6 hardware bring-up.

Design:
  - Recorder polls Motion.state_dict() at ~10 Hz while active.
  - "Capture waypoint" takes the current positions, optional dwell_ms and
    speed_pct override, appends to an in-memory waypoint list.
  - save(path) writes a Program JSON file (schema in programs.py).
  - Programs can be replayed by ProgramRunner which translates waypoints
    back into coordinated move_all_degrees calls.

Planned data shape (one waypoint):
    { "t_ms": int, "joints": {"J1": deg, ...}, "dwell_ms": int, "speed_pct": float }
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Waypoint:
    joints: dict[str, float]
    dwell_ms: int = 0
    speed_pct: float = 0.5
    t_ms: Optional[int] = None  # set during live recording for replay timing


@dataclass
class TeachRecorder:
    """Placeholder. In phase 4 this gets wired to Motion + an asyncio task."""
    active: bool = False
    waypoints: list[Waypoint] = field(default_factory=list)

    def start(self) -> None:
        self.active = True
        self.waypoints.clear()

    def stop(self) -> None:
        self.active = False

    def capture(self, joints: dict[str, float], dwell_ms: int = 0, speed_pct: float = 0.5) -> Waypoint:
        wp = Waypoint(joints=joints, dwell_ms=dwell_ms, speed_pct=speed_pct)
        self.waypoints.append(wp)
        return wp

    def clear(self) -> None:
        self.waypoints.clear()
