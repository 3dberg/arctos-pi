"""CAN bus abstraction for the MKS CANable v1.0 Pro (slcan) adapter.

Three backends, selected by config:
  - slcan: real hardware via python-can, channel = serial device path.
  - mock: in-process fake bus for laptop dev / CI. Records sent frames;
          supplies synthesized encoder-read responses.
  - dry_run: opens no hardware, only logs outgoing frames. Useful for
             a "safe" first boot with wiring powered down.

Frames use standard 11-bit IDs, 500 kbit/s (MKS default).
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Frame:
    """A CAN frame going either direction."""
    arbitration_id: int
    data: bytes

    def __repr__(self) -> str:
        hex_data = " ".join(f"{b:02X}" for b in self.data)
        return f"Frame(id=0x{self.arbitration_id:02X}, data=[{hex_data}])"


class CanBus(Protocol):
    def send(self, frame: Frame) -> None: ...
    def on_receive(self, callback: Callable[[Frame], None]) -> None: ...
    def shutdown(self) -> None: ...


# ---------------- python-can slcan backend ----------------

class SlcanBus:
    """Real hardware via python-can slcan.

    channel: serial device (e.g. /dev/ttyACM0, or /dev/serial/by-id/... for stability)
    bitrate: 500000 for MKS default
    """
    def __init__(self, channel: str, bitrate: int = 500_000):
        try:
            import can  # type: ignore
        except ImportError as e:
            raise RuntimeError("python-can not installed; pip install python-can[serial]") from e
        self._can = can
        self._bus = can.interface.Bus(bustype="slcan", channel=channel, bitrate=bitrate)
        self._callbacks: list[Callable[[Frame], None]] = []
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="can-rx")
        self._rx_thread.start()
        log.info("SlcanBus opened on %s @ %d", channel, bitrate)

    def send(self, frame: Frame) -> None:
        msg = self._can.Message(
            arbitration_id=frame.arbitration_id,
            data=frame.data,
            is_extended_id=False,
        )
        self._bus.send(msg, timeout=0.2)

    def on_receive(self, callback: Callable[[Frame], None]) -> None:
        self._callbacks.append(callback)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            msg = self._bus.recv(timeout=0.1)
            if msg is None:
                continue
            f = Frame(arbitration_id=msg.arbitration_id, data=bytes(msg.data))
            for cb in self._callbacks:
                try:
                    cb(f)
                except Exception:
                    log.exception("receiver callback failed")

    def shutdown(self) -> None:
        self._stop.set()
        self._bus.shutdown()


# ---------------- Mock backend ----------------

@dataclass
class MockBus:
    """In-process fake. Records sent frames; can inject receive frames.

    By default, responds to read commands (0x30, 0x31, 0x33) with synthesized
    payloads so higher layers can be exercised without hardware.
    """
    auto_respond: bool = True
    sent: list[Frame] = field(default_factory=list)
    _callbacks: list[Callable[[Frame], None]] = field(default_factory=list)
    _virtual_pulses: dict[int, int] = field(default_factory=dict)  # can_id -> pulses
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def send(self, frame: Frame) -> None:
        with self._lock:
            self.sent.append(frame)
        log.debug("mock tx %r", frame)
        if self.auto_respond and frame.data:
            self._maybe_respond(frame)

    def _maybe_respond(self, frame: Frame) -> None:
        cmd = frame.data[0]
        can_id = frame.arbitration_id
        if cmd == 0x31:  # read pulses
            pulses = self._virtual_pulses.get(can_id, 0)
            body = bytes([0x31]) + pulses.to_bytes(4, "big", signed=True)
            crc = (can_id + sum(body)) & 0xFF
            self.inject(Frame(can_id, body + bytes([crc])))
        elif cmd == 0x30:  # read encoder carry
            pulses = self._virtual_pulses.get(can_id, 0)
            carry = pulses // 0x10000
            value = pulses & 0xFFFF
            body = bytes([0x30]) + carry.to_bytes(4, "big", signed=True) + value.to_bytes(2, "big")
            crc = (can_id + sum(body)) & 0xFF
            self.inject(Frame(can_id, body + bytes([crc])))
        elif cmd == 0xFD and len(frame.data) >= 7:
            # Simulate relative move effect on virtual position.
            dir_bit = frame.data[1] & 0x80
            pulses = int.from_bytes(frame.data[4:7], "big")
            delta = pulses if dir_bit else -pulses
            self._virtual_pulses[can_id] = self._virtual_pulses.get(can_id, 0) + delta

    def inject(self, frame: Frame) -> None:
        for cb in self._callbacks:
            cb(frame)

    def on_receive(self, callback: Callable[[Frame], None]) -> None:
        self._callbacks.append(callback)

    def shutdown(self) -> None:
        pass


# ---------------- Dry-run backend ----------------

@dataclass
class DryRunBus:
    """Logs outgoing frames, never opens hardware. Safe-mode wiring-off testing."""
    sent: list[Frame] = field(default_factory=list)

    def send(self, frame: Frame) -> None:
        self.sent.append(frame)
        log.info("DRY %r", frame)

    def on_receive(self, callback: Callable[[Frame], None]) -> None:
        pass  # nothing ever arrives

    def shutdown(self) -> None:
        pass


# ---------------- Factory ----------------

def open_bus(backend: str, channel: Optional[str] = None, bitrate: int = 500_000) -> CanBus:
    """backend: 'slcan' | 'mock' | 'dry_run'"""
    if backend == "slcan":
        if not channel:
            raise ValueError("slcan backend requires a channel (e.g. /dev/ttyACM0)")
        return SlcanBus(channel, bitrate)
    if backend == "mock":
        return MockBus()
    if backend == "dry_run":
        return DryRunBus()
    raise ValueError(f"unknown backend: {backend}")


def autodetect_channel() -> Optional[str]:
    """Best-effort find the CANable Pro device on Linux.

    Prefers /dev/serial/by-id paths (stable across reboots). Returns None
    if nothing that looks like a CANable is present.
    """
    import glob, os
    for path in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(path).lower()
        if "canable" in name or "usb2can" in name or "mks" in name:
            return path
    # Fallback: first /dev/ttyACM* present
    acms = sorted(glob.glob("/dev/ttyACM*"))
    return acms[0] if acms else None
