import math

import pytest

from f1_wt102_driver.protocol import WitParser, decode_yaw


# WT102 protocol fixtures, not measurements captured from a physical device.
YAW_90 = bytes.fromhex('55 53 00 00 00 00 00 40 00 00 e8')
# Raw channel +500 deg/s at bytes 4..5, calibrated Z -250 deg/s at bytes 6..7.
# Distinct channels catch accidental use of the raw channel in the ROS output.
GYRO_NEGATIVE = bytes.fromhex('55 52 00 00 00 20 00 f0 00 00 b7')


def test_fragmentation_and_concatenation():
    stream = b'noise' + YAW_90 + GYRO_NEGATIVE
    for split in range(len(stream) + 1):
        parser = WitParser()
        assert parser.feed(stream[:split]) + parser.feed(stream[split:]) == [
            YAW_90, GYRO_NEGATIVE]


def test_corruption_and_dropped_bytes_recover():
    parser = WitParser()
    corrupt = YAW_90[:-1] + b'\x00'
    assert parser.feed(corrupt + GYRO_NEGATIVE[:7] + YAW_90) == [YAW_90]
    assert parser.bad_checksums >= 2
    assert len(parser.buffer) < 11
    assert parser.feed(b'\x01' * 10000) == []
    assert len(parser.buffer) == 0


def test_signed_values_and_si_units():
    assert decode_yaw(YAW_90) == ('yaw', math.pi / 2)
    kind, value = decode_yaw(GYRO_NEGATIVE)
    assert kind == 'yaw_rate'
    assert value == pytest.approx(math.radians(-250))


@pytest.mark.parametrize('hex_frame, expected', [
    ('55 53 00 00 00 00 00 c0 00 00 68', -math.pi / 2),
    ('55 53 00 00 00 00 00 80 00 00 28', -math.pi),
    ('55 53 00 00 00 00 ff 7f 00 00 26', 32767 * math.pi / 32768),
    ('55 53 00 00 00 00 00 40 34 12 2e', math.pi / 2),
])
def test_angle_boundaries_and_nonzero_version(hex_frame, expected):
    frame = bytes.fromhex(hex_frame)
    parser = WitParser()
    frames = []
    for byte in frame:
        frames.extend(parser.feed(bytes([byte])))
    assert frames == [frame]
    kind, value = decode_yaw(frames[0])
    assert kind == 'yaw'
    assert value == pytest.approx(expected)


def test_positive_rate_and_unrelated_reply():
    kind, rate = decode_yaw(bytes.fromhex('55 52 00 00 00 00 00 10 00 00 b7'))
    assert kind == 'yaw_rate'
    assert rate == pytest.approx(math.radians(250))
    # Register read replies must not create yaw measurements.
    assert decode_yaw(bytes.fromhex('55 5f 00 00 00 00 00 00 00 00 b4')) is None


@pytest.mark.parametrize('scale', [0, -1, float('nan'), float('inf')])
def test_invalid_scale_rejected(scale):
    with pytest.raises(ValueError):
        decode_yaw(YAW_90, scale)


def test_invalid_frame_rejected():
    with pytest.raises(ValueError):
        decode_yaw(YAW_90[:-1], 1.0)
