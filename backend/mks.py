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
    READ_SHAFT_ERROR = 0x39
    READ_EN_PIN = 0x3A
    READ_LOCK_STATUS = 0x3E

    SET_WORK_MODE = 0x82        # 0=CR_OPEN 1=CR_CLOSE 2=CR_vFOC 3..5=SR_*
    SET_CURRENT = 0x83          # mA / 100? see note: value is in 100mA units on 57D
    SET_SUBDIVISION = 0x84      # microsteps, 1..256 (0=256)
    SET_EN_ACTIVE = 0x85
    SET_MOTOR_DIR = 0x86
    SET_ZERO_MODE = 0x90

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
    """Payload from 0x31: [CMD][int32 pulses][CRC]. 6 bytes."""
    _check(can_id, payload, expected_cmd=Cmd.READ_PULSES, expected_len=6)
    return int.from_bytes(payload[1:5], "big", signed=True)


def _check(can_id: int, payload: bytes, expected_cmd: int, expected_len: int) -> None:
    if len(payload) != expected_len:
        raise ValueError(f"expected {expected_len} bytes, got {len(payload)}")
    if payload[0] != expected_cmd:
        raise ValueError(f"cmd mismatch: want 0x{expected_cmd:02X}, got 0x{payload[0]:02X}")
    # CRC check: sum of (can_id + cmd + params) low byte == payload[-1]
    want = (can_id + sum(payload[:-1])) & 0xFF
    if want != payload[-1]:
        raise ValueError(f"CRC mismatch: want 0x{want:02X}, got 0x{payload[-1]:02X}")
