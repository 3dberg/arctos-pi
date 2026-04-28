"""Gripper controller — CAN-attached Arduino driving a hobby servo.

Wire format (matches the Arduino sketch in the gripper firmware):
  arbitration_id = gripper.can_id (default 0x07)
  data           = [position]      # single unsigned byte, 0..255
                                    # mapped on the MCU to 10°..170° servo travel

The Arduino does not respond, so this module is fire-and-forget. We track the
last commanded position locally so the UI can reflect it without a read-back.
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass

from .can_bus import CanBus, Frame
from .config import GripperConfig

log = logging.getLogger(__name__)


@dataclass
class Gripper:
    cfg: GripperConfig
    bus: CanBus
    _position: int = 0
    _lock: threading.Lock = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._position = int(self.cfg.default_position) & 0xFF

    @property
    def position(self) -> int:
        return self._position

    def set_position(self, position: int) -> int:
        """Send a raw 0..255 position byte. Returns the clamped value sent."""
        if not self.cfg.enabled:
            raise RuntimeError("gripper is disabled in config")
        clamped = max(0, min(255, int(position)))
        self.bus.send(Frame(self.cfg.can_id, bytes([clamped])))
        with self._lock:
            self._position = clamped
        log.debug("gripper position -> %d (can_id=0x%02X)", clamped, self.cfg.can_id)
        return clamped

    def open(self) -> int:
        return self.set_position(self.cfg.open_position)

    def close(self) -> int:
        return self.set_position(self.cfg.close_position)

    def state_dict(self) -> dict:
        return {
            "enabled": self.cfg.enabled,
            "can_id": self.cfg.can_id,
            "position": self._position,
            "open_position": self.cfg.open_position,
            "close_position": self.cfg.close_position,
        }
