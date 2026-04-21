"""Tests for MKS protocol frame encoding.

Frame convention: python-can's Message.data contains [CMD..CRC], while
Message.arbitration_id carries the CAN ID. CRC is computed over ID+CMD+PARAMS.
"""
from backend import mks


def _crc_over(can_id: int, body_without_id: bytes) -> int:
    return (can_id + sum(body_without_id[:-1])) & 0xFF


def test_crc_basic():
    # Reference from arctosgui roscan.py style: 01 F4 00 64 02 00 00 00 -> CRC = sum & 0xFF
    data = bytes.fromhex("01F40064020000 00".replace(" ", ""))
    assert mks._crc(data) == sum(data) & 0xFF


def test_enable_frame():
    payload = mks.enable(1, True)
    assert payload[0] == 0xF3
    assert payload[1] == 1
    assert payload[-1] == _crc_over(1, payload)


def test_emergency_stop():
    payload = mks.emergency_stop(3)
    assert payload == bytes([0xF7, (3 + 0xF7) & 0xFF])


def test_speed_mode_cw():
    payload = mks.speed_mode(can_id=1, direction=1, speed=600, acc=2)
    # hi byte: 0x80 | (600>>8)=2  → 0x82
    # lo byte: 600 & 0xFF = 0x58
    assert payload[0] == 0xF6
    assert payload[1] == 0x82
    assert payload[2] == 0x58
    assert payload[3] == 0x02
    assert payload[-1] == _crc_over(1, payload)


def test_speed_mode_ccw():
    payload = mks.speed_mode(can_id=2, direction=0, speed=100, acc=5)
    assert payload[1] == 0x00
    assert payload[2] == 0x64
    assert payload[3] == 0x05
    assert payload[-1] == _crc_over(2, payload)


def test_position_relative_encoding():
    payload = mks.position_relative(can_id=1, direction=1, speed=500, acc=2, pulses=0x010000)
    assert payload[0] == 0xFD
    # hi: 0x80 | (500>>8)=1 → 0x81
    assert payload[1] == 0x81
    assert payload[2] == 500 & 0xFF
    assert payload[3] == 2
    assert payload[4:7] == bytes([0x01, 0x00, 0x00])
    assert payload[-1] == _crc_over(1, payload)


def test_position_relative_rejects_oversized():
    import pytest
    with pytest.raises(ValueError):
        mks.position_relative(1, 1, 500, 2, pulses=1 << 24)


def test_position_absolute_twos_complement():
    payload = mks.position_absolute(can_id=1, speed=500, acc=2, abs_pulses=-1)
    # -1 in 24-bit two's complement = 0xFFFFFF
    assert payload[4:7] == bytes([0xFF, 0xFF, 0xFF])


def test_microsteps_valid():
    p16 = mks.set_microsteps(1, 16)
    assert p16[0] == 0x84
    assert p16[1] == 16
    # 256 → 0 per firmware convention
    p256 = mks.set_microsteps(1, 256)
    assert p256[1] == 0


def test_microsteps_range():
    import pytest
    with pytest.raises(ValueError):
        mks.set_microsteps(1, 0)
    with pytest.raises(ValueError):
        mks.set_microsteps(1, 257)


def test_set_current_encoding():
    # 1600 mA → 16
    p = mks.set_current(1, 1600)
    assert p[0] == 0x83
    assert p[1] == 16
    assert p[-1] == _crc_over(1, p)


def test_parse_pulses_roundtrip():
    # Synthesize a valid response: CMD=0x31, pulses=12345 signed int32, CRC over ID+all
    can_id = 1
    body = bytes([0x31]) + (12345).to_bytes(4, "big", signed=True)
    crc = (can_id + sum(body)) & 0xFF
    payload = body + bytes([crc])
    assert mks.parse_pulses(can_id, payload) == 12345


def test_parse_encoder_carry_roundtrip():
    can_id = 2
    body = bytes([0x30]) + (-5).to_bytes(4, "big", signed=True) + (0x1234).to_bytes(2, "big")
    crc = (can_id + sum(body)) & 0xFF
    payload = body + bytes([crc])
    result = mks.parse_encoder_carry(can_id, payload)
    assert result.carry == -5
    assert result.value == 0x1234


def test_parse_rejects_bad_crc():
    import pytest
    can_id = 1
    bad = bytes([0x31, 0, 0, 0, 1, 0xAA])  # deliberately wrong CRC
    with pytest.raises(ValueError):
        mks.parse_pulses(can_id, bad)
