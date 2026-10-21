import json
import math

import pytest

from f1_imu_bridge.conversion import ClockDiscontinuity, Converter, UINT32, to_ros_message


BASE = 1_800_000_000_000_000_000
MONO = 100_000_000_000


def packet(sequence=1, ms=1000, delay_ms=0, session="gateway:1", **imu_overrides):
    elapsed = ms - 1000
    imu = {"kind": "IMU", "sequence": sequence, "stm32_ms": ms, "status": 31,
           "acceleration": [0.1, 0.2, 9.81], "angular_velocity": [0, 0, 0.1]}
    imu.update(imu_overrides)
    return {"protocol": "f1-imu-v1", "session": session,
            "received_unix_ns": BASE + (elapsed + delay_ms) * 1_000_000,
            "received_monotonic_ns": MONO + (elapsed + delay_ms) * 1_000_000,
            "imu": imu}


def convert(converter, document, now=None):
    return converter.convert(json.dumps(document).encode(),
                             document["received_unix_ns"] if now is None else now)


def test_imu_fields_and_absent_orientation():
    sample = convert(Converter(), packet())
    message = to_ros_message(sample, "imu_link")
    assert message.header.frame_id == "imu_link"
    assert message.header.stamp.sec == BASE // 10**9
    assert message.linear_acceleration.z == 9.81
    assert message.angular_velocity.z == 0.1
    assert message.orientation_covariance[0] == -1
    assert list(message.angular_velocity_covariance) == [0.0] * 9
    assert list(message.linear_acceleration_covariance) == [0.0] * 9


def test_units_can_be_converted_to_si():
    sample = convert(Converter(acceleration_scale=9.80665,
                               angular_velocity_scale=math.pi / 180),
                     packet(acceleration=[0, 0, 1], angular_velocity=[0, 0, 180]))
    assert sample.acceleration[2] == pytest.approx(9.80665)
    assert sample.angular_velocity[2] == pytest.approx(math.pi)


def test_duplicate_and_missing_sequences_are_counted():
    converter = Converter()
    assert convert(converter, packet())
    assert convert(converter, packet()) is None
    assert convert(converter, packet(sequence=4, ms=1030))
    assert converter.counts["duplicate"] == 1
    assert converter.counts["sequence_gaps"] == 2


def test_device_clock_preserves_sample_spacing_with_variable_latency():
    converter = Converter()
    first = convert(converter, packet())
    second = convert(converter, packet(sequence=2, ms=1010, delay_ms=15))
    assert second.stamp_ns - first.stamp_ns == 10_000_000


def test_uint32_counter_wrap_is_not_a_device_reset():
    converter = Converter()
    first = packet(sequence=UINT32 - 1, ms=UINT32 - 5)
    first["received_unix_ns"], first["received_monotonic_ns"] = BASE, MONO
    before = convert(converter, first)
    after = packet(sequence=0, ms=5)
    after["received_unix_ns"], after["received_monotonic_ns"] = BASE + 10_000_000, MONO + 10_000_000
    assert convert(converter, after).stamp_ns - before.stamp_ns == 10_000_000
    assert converter.counts["device_resets"] == 0


def test_device_reset_reanchors_without_replaying_old_timestamps():
    converter = Converter()
    first = convert(converter, packet(sequence=500, ms=5000))
    reset = packet(sequence=0, ms=0)
    reset["received_unix_ns"] = first.stamp_ns + 20_000_000
    reset["received_monotonic_ns"] = MONO + 4020_000_000
    assert convert(converter, reset).stamp_ns > first.stamp_ns
    assert converter.counts["device_resets"] == 1


def test_reconnect_changes_session():
    converter = Converter()
    convert(converter, packet())
    assert convert(converter, packet(sequence=1, ms=1010, session="gateway:2"))
    assert converter.counts["connection_changes"] == 1


def test_late_packet_does_not_poison_clock_state():
    converter = Converter()
    assert convert(converter, packet(), BASE + 2_000_000_000) is None
    assert converter.counts["stale"] == 1
    assert converter.previous is None
    assert convert(converter, packet())


def test_partial_backwards_counters_are_rejected():
    converter = Converter()
    convert(converter, packet(sequence=10, ms=1000))
    assert convert(converter, packet(sequence=9, ms=1010)) is None
    assert converter.counts["out_of_order"] == 1


@pytest.mark.parametrize("jump_ns", [-1_000_000_000, 1_000_000_000])
def test_system_clock_step_latches_publishing_off(jump_ns):
    converter = Converter()
    convert(converter, packet())
    stepped = packet(sequence=2, ms=1010)
    stepped["received_unix_ns"] += jump_ns
    with pytest.raises(ClockDiscontinuity):
        convert(converter, stepped)
    assert converter.clock_fault
    assert convert(converter, packet(sequence=3, ms=1020)) is None


def test_status_mask_filters_invalid_sensor_state():
    converter = Converter(required_status_mask=3)
    assert convert(converter, packet(status=1)) is None
    assert convert(converter, packet(status=3))


@pytest.mark.parametrize("field,value", [
    ("sequence", True), ("sequence", -1), ("stm32_ms", UINT32),
    ("acceleration", [0, float("nan"), 9.81]),
    ("angular_velocity", [0, 0]), ("status", 32),
])
def test_bad_fields_are_rejected_without_clock_state_changes(field, value):
    converter = Converter()
    assert convert(converter, packet(**{field: value})) is None
    assert converter.previous is None
    assert converter.counts["invalid"] == 1


def test_oversize_and_non_imu_wire_are_rejected():
    converter = Converter()
    for wire in (b"x" * 8193, b"null", b"[]", b"{}", b"not-json"):
        assert converter.convert(wire, BASE) is None
    assert converter.counts["invalid"] == 5
