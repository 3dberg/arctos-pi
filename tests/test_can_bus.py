"""SocketCanBus transmit robustness.

The hardware symptom these guard against: a few commands sent while the poll
loops + a homing seek are running overran the kernel CAN tx queue, python-can
raised "Transmit buffer full", and the old code reopened the socket on every
such error — a reopen storm that wedged the bus. The bus must now tell a
momentarily-full queue (retry, never reopen) apart from a genuinely dead
interface (reopen once + replay).
"""
import errno
import threading
from types import SimpleNamespace

import can
import pytest

from backend.can_bus import SocketCanBus, Frame, _errno_of


def _bp_error():
    # python-can socketcan: select() times out on a full tx queue -> this,
    # with no errno.
    return can.exceptions.CanOperationError("Transmit buffer full")


def _dead_error():
    # python-can socketcan: socket.send on a downed iface -> errno carried in
    # CanOperationError.error_code.
    return can.exceptions.CanOperationError("Failed to transmit: No such device", errno.ENODEV)


class _ScriptedBus:
    """Fake python-can Bus. `script` is consumed one entry per send(): an
    exception to raise, or None to succeed. Once exhausted, send() succeeds."""

    def __init__(self, script=()):
        self._script = list(script)
        self.calls = 0

    def send(self, msg, timeout=None):
        self.calls += 1
        if self._script:
            exc = self._script.pop(0)
            if exc is not None:
                raise exc

    def shutdown(self):
        pass


def _make_bus(initial_bus, reopen_to=None):
    """Build a SocketCanBus without opening hardware. reopen_to: the bus that
    _open_bus() should install (None => reopening is a test failure)."""
    b = SocketCanBus.__new__(SocketCanBus)
    b._channel = "test"
    b._bus_lock = threading.Lock()
    b._tx_lock = threading.Lock()
    b._last_tx = 0.0
    b._down = False
    b._bus = initial_bus
    reopens = {"n": 0}

    def _bus_factory(*a, **k):
        reopens["n"] += 1
        if reopen_to is None:
            raise AssertionError("reopened the bus when it should not have")
        return reopen_to

    b._can = SimpleNamespace(
        exceptions=can.exceptions,
        BusState=can.BusState,
        Message=lambda **kw: kw,
        interface=SimpleNamespace(Bus=_bus_factory),
    )
    return b, reopens


def test_is_backpressure_classifies_full_queue_not_dead_bus():
    b, _ = _make_bus(_ScriptedBus())
    assert b._is_backpressure(_bp_error()) is True
    assert b._is_backpressure(can.exceptions.CanTimeoutError("x")) is True
    assert b._is_backpressure(OSError(errno.ENOBUFS, "no buffer space")) is True
    # Dead-bus shapes must NOT be treated as back-pressure.
    assert b._is_backpressure(_dead_error()) is False
    assert b._is_backpressure(ValueError("negative fd")) is False
    assert b._is_backpressure(OSError(errno.ENODEV, "no device")) is False


def test_errno_of_reads_error_code_and_cause():
    assert _errno_of(_dead_error()) == errno.ENODEV
    assert _errno_of(OSError(errno.ENOBUFS, "x")) == errno.ENOBUFS
    cause = OSError(errno.ENODEV, "x")
    wrapped = RuntimeError("wrap")
    wrapped.__cause__ = cause
    assert _errno_of(wrapped) == errno.ENODEV


def test_backpressure_retries_then_succeeds_without_reopen():
    # Two full-queue errors, then it drains. Same socket throughout: no reopen.
    scripted = _ScriptedBus([_bp_error(), _bp_error(), None])
    b, reopens = _make_bus(scripted)  # reopen_to=None => reopen asserts
    b.send(Frame(1, bytes([0x31, 0x01])))
    assert scripted.calls == 3
    assert reopens["n"] == 0
    assert b._down is False  # never entered the dead-bus path


def test_persistent_backpressure_raises_and_does_not_reopen():
    # Queue never clears -> surface the error to the caller, still no reopen.
    always_full = _ScriptedBus([_bp_error()] * 20)
    b, reopens = _make_bus(always_full)
    with pytest.raises(can.exceptions.CanError):
        b.send(Frame(1, bytes([0x31, 0x01])))
    assert reopens["n"] == 0


def test_dead_bus_reopens_once_and_replays():
    dead = _ScriptedBus([_dead_error()])
    fresh = _ScriptedBus()  # the reopened socket sends fine
    b, reopens = _make_bus(dead, reopen_to=fresh)
    b.send(Frame(1, bytes([0x31, 0x01])))
    assert reopens["n"] == 1          # reopened exactly once
    assert fresh.calls == 1           # replayed on the new socket
    assert b._down is False           # _mark_up cleared it after recovery


def test_persistent_backpressure_with_wedged_controller_reopens_once():
    # Buffer-full forever AND the controller reports a wedged error state
    # (electrical noise from a misbehaving driver): timeouts won't clear it, so
    # reopen once + replay rather than spinning. A healthy controller (no state)
    # must NOT reopen — covered by test_persistent_backpressure_raises_*.
    wedged = _ScriptedBus([_bp_error()] * 20)
    wedged.state = can.BusState.ERROR
    fresh = _ScriptedBus()
    b, reopens = _make_bus(wedged, reopen_to=fresh)
    b.send(Frame(1, bytes([0x31, 0x01])))
    assert reopens["n"] == 1
    assert fresh.calls == 1
