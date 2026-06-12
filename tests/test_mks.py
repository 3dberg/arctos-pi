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


def test_parse_pulses_int48_firmware():
    # Newer MKS firmware returns int48 pulses (8-byte payload).
    # Real capture from axis 3: 31 FF FF FF F9 BF 04 ED → -409852
    can_id = 3
    payload = bytes.fromhex("31FFFFFFF9BF04ED")
    assert mks.parse_pulses(can_id, payload) == -409852

    # Positive case round-trip with a large int48 value.
    can_id = 5
    pulses = 0xE48E5
    body = bytes([0x31]) + pulses.to_bytes(6, "big", signed=True)
    crc = (can_id + sum(body)) & 0xFF
    payload = body + bytes([crc])
    assert mks.parse_pulses(can_id, payload) == pulses


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


# ---- homing ----

def test_set_home_encoding():
    # active-low switch (trig=0), seek CCW (dir=1), speed 300, endLimit on
    p = mks.set_home(can_id=1, home_trig=0, home_dir=1, home_speed=300, end_limit=True)
    assert p[0] == 0x90
    assert p[1] == 0          # homeTrig=Low
    assert p[2] == 1          # homeDir=CCW
    assert p[3] == (300 >> 8) & 0x0F
    assert p[4] == 300 & 0xFF
    assert p[5] == 1          # endLimit enabled
    assert p[-1] == _crc_over(1, p)


def test_set_home_validates_ranges():
    import pytest
    with pytest.raises(ValueError):
        mks.set_home(1, home_trig=2, home_dir=0, home_speed=100, end_limit=False)
    with pytest.raises(ValueError):
        mks.set_home(1, home_trig=0, home_dir=3, home_speed=100, end_limit=False)
    with pytest.raises(ValueError):
        mks.set_home(1, home_trig=0, home_dir=0, home_speed=5000, end_limit=False)


def test_go_home_and_axis_zero_frames():
    g = mks.go_home(2)
    assert g == bytes([0x91, (2 + 0x91) & 0xFF])
    z = mks.set_axis_zero(4)
    assert z == bytes([0x92, (4 + 0x92) & 0xFF])


def test_read_io_status_frame():
    p = mks.read_io_status(3)
    assert p == bytes([0x34, (3 + 0x34) & 0xFF])


def test_parse_status_roundtrip():
    can_id = 1
    body = bytes([0x91, mks.GO_HOME_SUCCESS])
    crc = (can_id + sum(body)) & 0xFF
    assert mks.parse_status(can_id, body + bytes([crc]), expected_cmd=0x91) == mks.GO_HOME_SUCCESS


def test_parse_status_rejects_wrong_cmd():
    import pytest
    can_id = 1
    body = bytes([0x91, 2])
    payload = body + bytes([(can_id + sum(body)) & 0xFF])
    with pytest.raises(ValueError):
        mks.parse_status(can_id, payload, expected_cmd=0x90)


def test_parse_io_status_bit_decode():
    can_id = 1
    # IN_1 + OUT_1 set -> 0b0101 = 0x05
    body = bytes([0x34, 0x05])
    crc = (can_id + sum(body)) & 0xFF
    io = mks.parse_io_status(can_id, body + bytes([crc]))
    assert io == {"in_1": True, "in_2": False, "out_1": True, "out_2": False}


def test_parse_io_status_rejects_bad_crc():
    import pytest
    with pytest.raises(ValueError):
        mks.parse_io_status(1, bytes([0x34, 0x01, 0xAA]))
