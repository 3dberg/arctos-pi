"""CAN bus abstraction for the MKS CANable v1.0 Pro (slcan) adapter.

Four backends, selected by config:
  - slcan: real hardware via python-can, channel = serial device path.
  - socketcan: real hardware via Linux kernel SocketCAN (e.g. gs_usb firmware
               on CANable Pro), channel = kernel interface name like 'can0'.
  - mock: in-process fake bus for laptop dev / CI. Records sent frames;
          supplies synthesized encoder-read responses.
  - dry_run: opens no hardware, only logs outgoing frames. Useful for
             a "safe" first boot with wiring powered down.

Frames use standard 11-bit IDs, 500 kbit/s (MKS default).
"""
from __future__ import annotations

import errno
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol

log = logging.getLogger(__name__)

# SocketCAN transmit tuning. The kernel CAN tx queue (txqueuelen, ~10 frames by
# default) overruns if we enqueue a burst faster than the 500 kbit bus drains
# it. On hardware that surfaced as the bus "locking up" after a couple of
# commands: several poll loops plus a homing seek all transmit at once, the
# queue fills, python-can raises "Transmit buffer full", and the old code
# misread that as a dead bus and reopened the socket — a reopen storm that
# wedged the interface. We instead serialize sends, pace them with a minimum
# inter-frame gap, and treat a momentarily-full queue as transient
# back-pressure (brief retry, never reopen).
_MIN_TX_GAP_S = 0.001          # minimum spacing between transmits
# python-can select() write timeout per frame. A healthy 500 kbit frame is
# ~0.2 ms on the wire and the kernel tx queue drains a backlog in a few ms, so a
# healthy send never approaches this — it only bites a genuinely stuck queue.
# Kept low (was 0.2) so one congested frame can't hold _tx_lock for hundreds of
# ms and starve the bridge's poll timers (which froze the UI during homing).
# Bump it for slow slcan-over-serial if needed.
_TX_TIMEOUT_S = 0.03
_TX_BACKPRESSURE_RETRIES = 3   # retries when the tx queue is momentarily full
_TX_BACKOFF_S = 0.005          # wait between back-pressure retries
_BACKPRESSURE_ERRNOS = {errno.ENOBUFS, errno.EAGAIN, errno.EWOULDBLOCK}


def _errno_of(exc: BaseException) -> Optional[int]:
    """Best-effort OS errno from an exception or its cause. python-can puts the
    errno on CanOperationError.error_code and chains the original OSError."""
    for attr in ("errno", "error_code"):
        val = getattr(exc, attr, None)
        if isinstance(val, int):
            return val
    cause = exc.__cause__
    return getattr(cause, "errno", None) if cause is not None else None


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
        self._tx_lock = threading.Lock()
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
        # Serialize writes: concurrent threads writing the serial slcan device
        # can interleave bytes and corrupt frames.
        with self._tx_lock:
            self._bus.send(msg, timeout=_TX_TIMEOUT_S)

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


# ---------------- python-can socketcan backend ----------------

class SocketCanBus:
    """Real hardware via Linux kernel SocketCAN.

    Use this when the CANable Pro runs gs_usb firmware (VID 1d50:606f), which
    exposes the adapter as a kernel CAN interface (e.g. can0) rather than a
    serial /dev/ttyACM device.

    Bring the interface up before starting the app:
        sudo ip link set can0 up type can bitrate 500000

    channel: kernel interface name (e.g. 'can0')
    bitrate: informational only on socketcan — the kernel interface must
             already be configured with the correct bitrate via `ip link`.
    """
    def __init__(self, channel: str, bitrate: int = 500_000):
        try:
            import can  # type: ignore
        except ImportError as e:
            raise RuntimeError("python-can not installed; pip install python-can") from e
        self._can = can
        self._channel = channel
        self._bus_lock = threading.Lock()        # guards _open_bus (bus swap)
        self._tx_lock = threading.Lock()         # serializes transmits across threads
        self._last_tx = 0.0                       # monotonic time of the last transmit (pacing)
        self._bus = None  # type: ignore[assignment]
        self._down = False  # True while send is failing; de-spams the reopen warning
        self._open_bus()
        self._callbacks: list[Callable[[Frame], None]] = []
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="can-rx")
        self._rx_thread.start()
        log.info("SocketCanBus opened on %s (kernel bitrate applies)", channel)

    def _open_bus(self) -> None:
        # Caller must hold _bus_lock (or be in __init__).
        # Tear down the OLD bus first into a local, then try to open a new
        # one; only on success do we assign self._bus. If Bus() raises we
        # null self._bus so the next caller will try _open_bus again rather
        # than reusing a half-dead socket (where send() raises ValueError on
        # an fd=-1 socket instead of a recognizable CanError).
        old = self._bus
        self._bus = None
        if old is not None:
            try:
                old.shutdown()
            except Exception:
                pass
        self._bus = self._can.interface.Bus(bustype="socketcan", channel=self._channel)

    def send(self, frame: Frame) -> None:
        msg = self._can.Message(
            arbitration_id=frame.arbitration_id,
            data=frame.data,
            is_extended_id=False,
        )
        # One transmit at a time across every caller thread (the poll timers,
        # the homing worker, trajectory execution), each spaced by _pace(), so
        # a burst can't overrun the kernel tx queue.
        with self._tx_lock:
            self._send_locked(msg)

    def _send_locked(self, msg) -> None:
        for attempt in range(_TX_BACKPRESSURE_RETRIES + 1):
            self._pace()
            try:
                if self._bus is None:
                    raise OSError("CAN bus not open")
                self._bus.send(msg, timeout=_TX_TIMEOUT_S)
                self._mark_up()
                return
            except (self._can.exceptions.CanError, OSError, ValueError) as e:
                if self._is_backpressure(e):
                    # TX queue momentarily full. Reopening the bus here is what
                    # used to wedge it ("send two commands and it stops
                    # responding"); instead let the queue drain and retry the
                    # same frame.
                    if attempt < _TX_BACKPRESSURE_RETRIES:
                        time.sleep(_TX_BACKOFF_S)
                        continue
                    # Retries exhausted. If the controller has gone ERROR/PASSIVE
                    # (e.g. electrical noise from a misbehaving driver), spinning
                    # on timeouts won't help — reopen the socket once (re-applies
                    # the kernel iface state) and replay. Otherwise it's plain
                    # congestion: surface it to the caller (bridge logs+skips,
                    # API returns 503) with no reopen.
                    if self._bus_is_dead():
                        if not self._down:
                            log.warning("CAN %s controller wedged (%s); reopening once",
                                        self._channel, type(e).__name__)
                            self._down = True
                        with self._bus_lock:
                            self._open_bus()
                        self._bus.send(msg, timeout=_TX_TIMEOUT_S)
                        self._last_tx = time.monotonic()
                        self._mark_up()
                        return
                    raise
                # Genuine dead socket — catches every shape of it:
                #   CanError: python-can's typed wrapper around ENETDOWN etc.
                #   OSError: raw errno paths (ENODEV, EBADF) some versions don't
                #     wrap, plus our own "CAN bus not open".
                #   ValueError: socket fd closed to -1 — select() rejects it,
                #     after a prior reopen that itself failed.
                # Bring the bus back and replay the send so the user gets
                # seamless recovery the moment they `ip link set can0 up` (or
                # replug the dongle). Log only the first failure (and
                # "recovered" once it returns) so a downed bus doesn't flood the
                # log at the poll rate.
                if not self._down:
                    log.warning("CAN send failing (%s: %s); reopening %s and retrying "
                                "(further errors suppressed until recovery)",
                                type(e).__name__, e, self._channel)
                    self._down = True
                with self._bus_lock:
                    self._open_bus()
                self._bus.send(msg, timeout=_TX_TIMEOUT_S)
                self._last_tx = time.monotonic()
                self._mark_up()
                return

    def _pace(self) -> None:
        """Hold a minimum gap between transmits so a burst of poll/homing reads
        can't enqueue faster than the bus drains and overrun the kernel tx
        queue. Caller holds _tx_lock."""
        if _MIN_TX_GAP_S <= 0:
            return
        wait = self._last_tx + _MIN_TX_GAP_S - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        self._last_tx = time.monotonic()

    def _is_backpressure(self, e: BaseException) -> bool:
        """True if e means 'tx queue momentarily full' rather than 'bus dead'."""
        timeout_exc = getattr(self._can.exceptions, "CanTimeoutError", None)
        if timeout_exc is not None and isinstance(e, timeout_exc):
            return True
        if _errno_of(e) in _BACKPRESSURE_ERRNOS:
            return True
        # python-can's socketcan raises CanOperationError("Transmit buffer
        # full") with no errno when select() times out on a full tx queue.
        return "buffer full" in str(e).lower()

    def _bus_is_dead(self) -> bool:
        """True if python-can reports the controller in a wedged error state
        (ERROR/PASSIVE / bus-off) that timeouts alone won't clear. Best-effort:
        socketcan exposes bus.state; other backends may not."""
        state = getattr(self._bus, "state", None)
        bus_state = getattr(self._can, "BusState", None)
        if state is None or bus_state is None:
            return False
        dead = {getattr(bus_state, n, None) for n in ("PASSIVE", "ERROR", "BUS_OFF", "ERROR_PASSIVE")}
        dead.discard(None)
        return state in dead

    def _mark_up(self) -> None:
        if self._down:
            log.info("CAN %s recovered", self._channel)
            self._down = False

    def on_receive(self, callback: Callable[[Frame], None]) -> None:
        self._callbacks.append(callback)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._bus is None:
                    time.sleep(0.5)
                    continue
                msg = self._bus.recv(timeout=0.1)
            except (self._can.exceptions.CanError, OSError, ValueError):
                # Same dead-socket cases as send(); just back off. The next
                # send() will reopen the bus and recv() picks up from there.
                time.sleep(0.5)
                continue
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
        try:
            self._bus.shutdown()
        except Exception:
            pass


# ---------------- Mock backend ----------------

@dataclass
class MockBus:
    """In-process fake. Records sent frames; can inject receive frames.

    By default, responds to read commands (0x30, 0x31, 0x34) with synthesized
    payloads, simulates relative moves (0xFD), and fakes the homing handshake
    (0x90/0x91/0x92 — zeroes the virtual position and replies success) so higher
    layers can be exercised without hardware.
    """
    auto_respond: bool = True
    sent: list[Frame] = field(default_factory=list)
    _callbacks: list[Callable[[Frame], None]] = field(default_factory=list)
    _virtual_pulses: dict[int, int] = field(default_factory=dict)  # can_id -> pulses
    _home_params: dict[int, bytes] = field(default_factory=dict)   # can_id -> last 0x90 params
    _io_status: dict[int, int] = field(default_factory=dict)       # can_id -> raw IO byte (bit0=IN_1)
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
        elif cmd == 0x90:  # set home params — record, ack with status=1
            self._home_params[can_id] = bytes(frame.data[1:-1])
            self._respond_status(can_id, 0x90, 1)
        elif cmd == 0x91:  # go home — driver zeroes at the switch, then succeeds
            self._virtual_pulses[can_id] = 0
            self._respond_status(can_id, 0x91, 1)  # Start
            self._respond_status(can_id, 0x91, 2)  # Success
        elif cmd == 0x92:  # set current position as zero
            self._virtual_pulses[can_id] = 0
            self._respond_status(can_id, 0x92, 1)
        elif cmd == 0x34:  # read IO status — default IN_1 tripped (bit0 set)
            bits = self._io_status.get(can_id, 0x01)
            self._respond_status(can_id, 0x34, bits)

    def _respond_status(self, can_id: int, cmd: int, status: int) -> None:
        body = bytes([cmd, status & 0xFF])
        crc = (can_id + sum(body)) & 0xFF
        self.inject(Frame(can_id, body + bytes([crc])))

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
    """backend: 'slcan' | 'socketcan' | 'mock' | 'dry_run'"""
    if backend == "slcan":
        if not channel:
            raise ValueError("slcan backend requires a channel (e.g. /dev/ttyACM0)")
        return SlcanBus(channel, bitrate)
    if backend == "socketcan":
        if not channel:
            raise ValueError("socketcan backend requires a channel (e.g. can0)")
        return SocketCanBus(channel, bitrate)
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
