"""MKS SERVO42D / SERVO57D CAN protocol (firmware V1.0.6).

Frame layout: [CAN_ID][CMD][PARAMS...][CRC]
CRC is the low byte of the sum of CAN_ID + CMD + PARAMS.
All multi-byte integers are big-endian. 11-bit standard IDs, 500 kbit/s.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class Cmd(IntEnum):
    READ_ENCODER_CARRY = 0x30   # returns int32 carry + uint16 within-turn value
    READ_PULSES = 0x31          # returns int32 pulses received
    READ_SHAFT_ANGLE = 0x33     # returns int32 angle, 0..0xFFFF = one rev
    READ_IO_STATUS = 0x34       # returns 1 status byte: bit0=IN_1 bit1=IN_2 bit2=OUT_1 bit3=OUT_2
    READ_SHAFT_ERROR = 0x39
    READ_EN_PIN = 0x3A
    READ_LOCK_STATUS = 0x3E

    SET_WORK_MODE = 0x82        # 0=CR_OPEN 1=CR_CLOSE 2=CR_vFOC 3..5=SR_*
    SET_CURRENT = 0x83          # mA / 100? see note: value is in 100mA units on 57D
    SET_SUBDIVISION = 0x84      # microsteps, 1..256 (0=256)
    SET_EN_ACTIVE = 0x85
    SET_MOTOR_DIR = 0x86
    SET_HOME = 0x90             # set home params: homeTrig, homeDir, homeSpeed, endLimit
    GO_HOME = 0x91             # trigger driver autonomous home seek; status 0=fail 1=start 2=success
    SET_AXIS_ZERO = 0x92       # set current position as zero (no motion); like go_home without moving

    ENABLE_MOTOR = 0xF3         # param: 1=enable 0=disable
    EMERGENCY_STOP = 0xF7
    SPEED_MODE = 0xF6           # hold-to-run speed: dir+speed+acc
    POSITION_REL_PULSES = 0xFD  # relative by pulses (preferred for jog-to)
    POSITION_ABS_PULSES = 0xFE  # absolute by pulses


def _crc(data: bytes) -> int:
    return sum(data) & 0xFF


def _frame(can_id: int, cmd: int, params: bytes = b"") -> bytes:
    """Build a full frame with trailing CRC. CAN_ID here is the 11-bit arbitration ID."""
    if not 1 <= can_id <= 0x7FF:
        raise ValueError(f"can_id out of range: {can_id}")
    body = bytes([can_id & 0xFF, cmd]) + params
    return body + bytes([_crc(body)])


def _payload(can_id: int, cmd: int, params: bytes = b"") -> bytes:
    """Data payload for python-can Message.data (cmd + params + crc).

    python-can Message.arbitration_id carries the CAN ID separately, so the
    data bus payload must NOT include the ID byte. CRC is still computed over
    ID + CMD + PARAMS per MKS spec.
    """
    full = _frame(can_id, cmd, params)
    return full[1:]  # strip the ID byte, keep CMD..CRC


# ---------- Motion commands ----------

def enable(can_id: int, on: bool = True) -> bytes:
    return _payload(can_id, Cmd.ENABLE_MOTOR, bytes([1 if on else 0]))


def emergency_stop(can_id: int) -> bytes:
    return _payload(can_id, Cmd.EMERGENCY_STOP)


def speed_mode(can_id: int, direction: int, speed: int, acc: int = 2) -> bytes:
    """Hold-to-run speed mode (CMD 0xF6).

    direction: 0 = CCW, 1 = CW (MKS convention; flip via invert flag upstream)
    speed: 0..3000 (RPM-ish, driver-defined)
    acc: 0..255, 0 = instant. 2 is a gentle default.
    Byte layout: [dir<<7 | speed_high_4bits][speed_low_8bits][acc]
    """
    if not 0 <= speed <= 0x0FFF:
        raise ValueError("speed must be 0..4095")
    if not 0 <= acc <= 0xFF:
        raise ValueError("acc must be 0..255")
    dir_bit = 0x80 if direction else 0x00
    hi = dir_bit | ((speed >> 8) & 0x0F)
    lo = speed & 0xFF
    return _payload(can_id, Cmd.SPEED_MODE, bytes([hi, lo, acc]))


def position_relative(can_id: int, direction: int, speed: int, acc: int, pulses: int) -> bytes:
    """Relative move by pulses (CMD 0xFD).

    direction: 0=CCW 1=CW
    speed: 0..3000
    acc: 0..255
    pulses: 0..0xFFFFFF (sign is carried by direction bit)
    """
    if not 0 <= pulses <= 0xFFFFFF:
        raise ValueError("pulses must fit in 24 bits unsigned")
    dir_bit = 0x80 if direction else 0x00
    hi = dir_bit | ((speed >> 8) & 0x0F)
    lo = speed & 0xFF
    params = bytes([hi, lo, acc & 0xFF]) + pulses.to_bytes(3, "big")
    return _payload(can_id, Cmd.POSITION_REL_PULSES, params)


def position_absolute(can_id: int, speed: int, acc: int, abs_pulses: int) -> bytes:
    """Absolute position by pulses (CMD 0xFE). abs_pulses is signed 24-bit."""
    if not -(1 << 23) <= abs_pulses < (1 << 23):
        raise ValueError("abs_pulses must fit in signed 24 bits")
    hi = (speed >> 8) & 0x0F
    lo = speed & 0xFF
    twos = abs_pulses & 0xFFFFFF
    params = bytes([hi, lo, acc & 0xFF]) + twos.to_bytes(3, "big")
    return _payload(can_id, Cmd.POSITION_ABS_PULSES, params)


# ---------- Config commands ----------

def set_microsteps(can_id: int, microsteps: int) -> bytes:
    """Microsteps 1..256. Firmware encodes 256 as 0."""
    if not 1 <= microsteps <= 256:
        raise ValueError("microsteps must be 1..256")
    value = 0 if microsteps == 256 else microsteps
    return _payload(can_id, Cmd.SET_SUBDIVISION, bytes([value]))


def set_current(can_id: int, milliamps: int) -> bytes:
    """Set running current. SERVO42D max 3000 mA, SERVO57D max 5200 mA.
    Caller is responsible for clamping to the correct ceiling for their driver.
    Value transmitted is mA / 100 in a single byte per V1.0.6 spec.
    """
    if not 0 <= milliamps <= 25500:
        raise ValueError("milliamps out of byte-encodable range")
    return _payload(can_id, Cmd.SET_CURRENT, bytes([milliamps // 100]))


def set_work_mode(can_id: int, mode: int) -> bytes:
    """0=CR_OPEN 1=CR_CLOSE 2=CR_vFOC 3=SR_OPEN 4=SR_CLOSE 5=SR_vFOC."""
    if not 0 <= mode <= 5:
        raise ValueError("mode must be 0..5")
    return _payload(can_id, Cmd.SET_WORK_MODE, bytes([mode]))


def set_motor_direction(can_id: int, reverse: bool) -> bytes:
    return _payload(can_id, Cmd.SET_MOTOR_DIR, bytes([1 if reverse else 0]))


# ---------- Homing commands ----------

def set_home(can_id: int, home_trig: int, home_dir: int, home_speed: int,
             end_limit: bool) -> bytes:
    """Set the driver's home/endstop parameters (CMD 0x90).

    Byte layout: [homeTrig][homeDir][speed_high_4bits][speed_low_8bits][endLimit]
      home_trig: endstop active level — 0=Low, 1=High. A reverse-logic
                 (active-low / normally-closed) home switch uses 0.
      home_dir:  seek direction — 0=CW, 1=CCW. This is the RAW driver
                 direction; do NOT pre-apply the software `invert` flag.
      home_speed: 0..0x0FFF, same 12-bit encoding as speed_mode.
      end_limit: enable the driver's own limit feature (separate from the
                 software seek bound enforced in motion.py).
    Persists to driver flash; first use of end_limit requires a go_home after.
    """
    if home_trig not in (0, 1):
        raise ValueError("home_trig must be 0 (Low) or 1 (High)")
    if home_dir not in (0, 1):
        raise ValueError("home_dir must be 0 (CW) or 1 (CCW)")
    if not 0 <= home_speed <= 0x0FFF:
        raise ValueError("home_speed must be 0..4095")
    params = bytes([
        home_trig,
        home_dir,
        (home_speed >> 8) & 0x0F,
        home_speed & 0xFF,
        1 if end_limit else 0,
    ])
    return _payload(can_id, Cmd.SET_HOME, params)


def go_home(can_id: int) -> bytes:
    """Trigger the driver's autonomous home seek (CMD 0x91). Two-phase reply:
    an immediate status=1 (Start), then a later status=2 (Success) / 0 (Fail)."""
    return _payload(can_id, Cmd.GO_HOME)


def set_axis_zero(can_id: int) -> bytes:
    """Set the current position as the axis zero without moving (CMD 0x92)."""
    return _payload(can_id, Cmd.SET_AXIS_ZERO)


def read_io_status(can_id: int) -> bytes:
    """Request the IO port status (CMD 0x34). Reply decoded by parse_io_status."""
    return _payload(can_id, Cmd.READ_IO_STATUS)


# ---------- Read commands ----------

def read_encoder_carry(can_id: int) -> bytes:
    return _payload(can_id, Cmd.READ_ENCODER_CARRY)


def read_pulses(can_id: int) -> bytes:
    return _payload(can_id, Cmd.READ_PULSES)


def read_shaft_angle(can_id: int) -> bytes:
    return _payload(can_id, Cmd.READ_SHAFT_ANGLE)


# ---------- Response parsing ----------

@dataclass
class EncoderCarry:
    carry: int      # signed int32, full-turn counter
    value: int      # 0..0xFFFF within-turn position


def parse_encoder_carry(can_id: int, payload: bytes) -> EncoderCarry:
    """Payload from a 0x30 response: [CMD][int32 carry][uint16 value][CRC]. 8 bytes."""
    _check(can_id, payload, expected_cmd=Cmd.READ_ENCODER_CARRY, expected_len=8)
    carry = int.from_bytes(payload[1:5], "big", signed=True)
    value = int.from_bytes(payload[5:7], "big", signed=False)
    return EncoderCarry(carry=carry, value=value)


def parse_pulses(can_id: int, payload: bytes) -> int:
    """Payload from 0x31: [CMD][int pulses][CRC].

    Width depends on driver firmware revision:
      - older (V1.0.4/5): int32 → 6 bytes total
      - newer (≥V1.0.6 on SERVO42D/57D): int48 → 8 bytes total
    Accept both; caller gets a signed Python int either way.
    """
    if len(payload) == 6:
        _check(can_id, payload, expected_cmd=Cmd.READ_PULSES, expected_len=6)
        return int.from_bytes(payload[1:5], "big", signed=True)
    if len(payload) == 8:
        _check(can_id, payload, expected_cmd=Cmd.READ_PULSES, expected_len=8)
        return int.from_bytes(payload[1:7], "big", signed=True)
    raise ValueError(
        f"unexpected READ_PULSES reply length: {len(payload)} "
        f"(want 6 for int32-pulses firmware or 8 for int48-pulses firmware)"
    )


def parse_status(can_id: int, payload: bytes, expected_cmd: int) -> int:
    """Generic single-byte status reply: [CMD][status][CRC] (3 bytes).

    Used by GO_HOME (0x91: 0=fail 1=start 2=success), SET_HOME (0x90) and
    SET_AXIS_ZERO (0x92), which all reply with a 1-byte status."""
    _check(can_id, payload, expected_cmd=expected_cmd, expected_len=3)
    return payload[1]


# Status values returned by GO_HOME (CMD 0x91).
GO_HOME_FAIL = 0
GO_HOME_START = 1
GO_HOME_SUCCESS = 2


def parse_io_status(can_id: int, payload: bytes) -> dict:
    """Decode a 0x34 IO-status reply: [CMD][status][CRC] (3 bytes). Returns the
    RAW electrical levels; active-low interpretation belongs to the caller.
    Bit map per MKS spec: bit0=IN_1, bit1=IN_2, bit2=OUT_1, bit3=OUT_2."""
    _check(can_id, payload, expected_cmd=Cmd.READ_IO_STATUS, expected_len=3)
    bits = payload[1]
    return {
        "in_1": bool(bits & 0x01),
        "in_2": bool(bits & 0x02),
        "out_1": bool(bits & 0x04),
        "out_2": bool(bits & 0x08),
    }


def _check(can_id: int, payload: bytes, expected_cmd: int, expected_len: int) -> None:
    if len(payload) != expected_len:
        raise ValueError(f"expected {expected_len} bytes, got {len(payload)}")
    if payload[0] != expected_cmd:
        raise ValueError(f"cmd mismatch: want 0x{expected_cmd:02X}, got 0x{payload[0]:02X}")
    # CRC check: sum of (can_id + cmd + params) low byte == payload[-1]
    want = (can_id + sum(payload[:-1])) & 0xFF
    if want != payload[-1]:
        raise ValueError(f"CRC mismatch: want 0x{want:02X}, got 0x{payload[-1]:02X}")
