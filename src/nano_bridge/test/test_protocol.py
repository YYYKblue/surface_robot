import binascii
import math
import random
import struct

import pytest

from nano_ros_bridge.protocol import Lines, channels, decode, encode, neutral, servo


def line(body):
    raw = body.encode()
    check = 0
    for byte in raw:
        check ^= byte
    return raw + f"*{check:02X}\r\n".encode()


def test_reference_crc_and_wire_layout():
    assert binascii.crc_hqx(b"123456789", 0xffff) == 0x29b1
    packet = encode(channels(1, 50), 0x12345678, 0xfedcba98)
    assert len(packet) == 39
    assert packet[:4] == b"\xa5\x5a\x02\x19"
    assert struct.unpack("<II", packet[4:12]) == (0x12345678, 0xfedcba98)
    assert packet[12] == 15 and packet[35:37] == b"\0\0"
    packed = int.from_bytes(packet[13:35], "little")
    values = [(((packed >> (11 * i)) & 2047) * 5 + 6994) // 8 for i in range(16)]
    assert values[:6] == [1500, 1500, 1950, 1500, 1500, 1500]
    assert values[9:11] == [1050, 1050]
    assert struct.unpack("<H", packet[-2:])[0] == binascii.crc_hqx(packet[2:37], 0xffff)
    assert encode(neutral(), 1, 0).hex() == (
        "a55a021901000000000000000fea539ffad4a73e8da84f7dead38846d4a73ef5a94f7d00005dd3")


@pytest.mark.parametrize("value", [math.nan, math.inf, -101, 101, True, "20"])
def test_invalid_auxiliary_effort(value):
    with pytest.raises(ValueError):
        servo(value)


def test_auxiliary_inverse_and_explicit_switches():
    for value in range(-100, 101):
        delta = servo(value) - 1500
        actual = 0 if abs(delta) <= 20 else ((abs(delta) - 20) * 100 + 215) // 430
        assert actual == abs(value)
    assert channels(brush_enabled=False, brush_percent=100)[7:11] == [1500, 1500, 1050, 1050]
    assert channels(brush_enabled=True, brush_percent=-20)[7:11] == [1394, 1500, 1950, 1050]


@pytest.mark.parametrize("direction,pair", enumerate([(1500, 1500), (1950, 1500), (1050, 1500),
                                                        (1500, 1050), (1500, 1950)]))
def test_exclusive_direction_encoding(direction, pair):
    for gear, switch in [(25, 1050), (50, 1500), (75, 1950)]:
        for mode in range(3):
            values = channels(direction, gear, mode)
            assert values[:2] == [1500, 1500]
            assert values[2:4] == list(pair)
            assert values[4:6] == [(1050, 1500, 1950)[mode], switch]


@pytest.mark.parametrize("kwargs", [dict(direction=(20, 20, 20, 20)), dict(direction=True),
    dict(direction=5), dict(speed_percent=20), dict(speed_percent=25.), dict(mode=True),
    dict(mode=3), dict(brush_enabled=1)])
def test_old_motor_commands_and_invalid_v2_enums_rejected(kwargs):
    with pytest.raises(ValueError):
        channels(**kwargs)


def test_telemetry_strictness_and_fragment_recovery():
    body = "CTRL,2,400,NANO,1,1,20,OK,BOTH,1,0,0,0,0,1500,1500,1500,1500,25,STOP"
    ctrl = line(body)
    imu = line("IMU,10,400,0,0,9.8,0,0,0,1,0,0,0,0,0,0,0,6")
    stream = Lines()
    data = b"x" * 513 + b"\n" + ctrl + imu
    rng = random.Random(6)
    result = []
    while data:
        size = rng.randint(1, 19)
        result += stream.feed(data[:size])
        data = data[size:]
    assert stream.errors == 1
    assert decode(result[0]).neutral
    assert decode(result[0]).speed_percent == 25
    assert decode(result[1])["status"] == 6
    assert decode(line(body.replace(",OK,", ",DIR_CONFLICT,"))).result == "DIR_CONFLICT"
    for broken in (ctrl[:-5] + b"*00\n", line(body.replace("CTRL,2,", "CTRL,1,")),
                   line(body.rsplit(",", 2)[0]), line(body.replace(",25,STOP", ",20,STOP")),
                   line(body.replace(",STOP", ",DIAGONAL")),
                   line("IMU,10,400,nan,0,9.8,0,0,0,1,0,0,0,0,0,0,0,6"),
                   line(body.replace(",1,1,20,", ",1,2,20,"))):
        with pytest.raises(ValueError):
            decode(broken)


def test_encoder_rejects_bad_channels():
    for values in ([1500] * 15, [1049] * 16, [1951] * 16, [1500.0] * 16):
        with pytest.raises(ValueError):
            encode(values, 1, 0)
    assert neutral()[5] == 1050 and neutral()[9:11] == [1050, 1050]
