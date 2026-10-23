"""Serial sample validation and approximate device-to-host timestamp alignment.

No serial, ROS, or network side effects. Alignment uses the lowest observed
host receive latency, not hardware synchronization between IMU and lidar.
"""
from collections import Counter
from dataclasses import dataclass
import math


NS_PER_MS = 1_000_000
UINT32 = 1 << 32


class ClockDiscontinuity(RuntimeError):
    """A system clock step requires restarting the native bridge."""


def integer(value, name, maximum=(1 << 63) - 1):
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError(f"invalid {name}")
    return value


def positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return float(value)


def vector(value, scale, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must have three components")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError(f"non-finite {name}")
    converted = tuple(float(x) * scale for x in value)
    if any(not math.isfinite(x) for x in converted):
        raise ValueError(f"non-finite scaled {name}")
    return converted


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    acceleration: tuple
    angular_velocity: tuple


class Converter:
    def __init__(self, acceleration_scale=1.0, angular_velocity_scale=1.0,
                 required_status_mask=0, max_age_sec=0.5, clock_jump_sec=0.1):
        self.acceleration_scale = positive(acceleration_scale, "acceleration_scale")
        self.angular_velocity_scale = positive(angular_velocity_scale, "angular_velocity_scale")
        self.required_status_mask = integer(required_status_mask, "required_status_mask", 31)
        self.max_age_ns = round(positive(max_age_sec, "max_age_sec") * 1e9)
        self.clock_jump_ns = round(positive(clock_jump_sec, "clock_jump_sec") * 1e9)
        self.counts = Counter()
        self.error = "waiting for serial IMU data"
        self.clock_fault = False
        self.session = None
        self.previous = None
        self.elapsed_ns = 0
        self.offset_mono_ns = None
        self.epoch_offset_ns = None
        self.last_stamp_ns = None
        self.last_receive_ns = None

    def convert(self, imu, now_unix_ns, *, session, received_unix_ns, received_monotonic_ns):
        """Convert one decoded serial sample with its original receive timestamps."""
        if self.clock_fault:
            return None
        try:
            if not isinstance(session, str) or not 1 <= len(session) <= 128:
                raise ValueError("invalid session")
            rx_ns = integer(received_unix_ns, "received_unix_ns", (1 << 31) * 10**9 - 1)
            mono_ns = integer(received_monotonic_ns, "received_monotonic_ns")
            if not isinstance(imu, dict) or imu.get("kind") != "IMU":
                raise ValueError("missing IMU data")
            sequence = integer(imu.get("sequence"), "sequence", UINT32 - 1)
            milliseconds = integer(imu.get("stm32_ms"), "stm32_ms", UINT32 - 1)
            status = integer(imu.get("status"), "status", 31)
            acceleration = vector(imu.get("acceleration"), self.acceleration_scale, "acceleration")
            angular_velocity = vector(imu.get("angular_velocity"), self.angular_velocity_scale,
                                      "angular_velocity")
        except (ValueError, TypeError) as error:
            self.counts["invalid"] += 1
            self.error = str(error)
            return None

        if status & self.required_status_mask != self.required_status_mask:
            self.counts["status_rejected"] += 1
            self.error = "F1 IMU required status bits are missing"
            return None
        age = now_unix_ns - rx_ns
        if age > self.max_age_ns:
            self.counts["stale"] += 1
            self.error = "serial sample is too old"
            return None
        epoch_offset = rx_ns - mono_ns
        if (age < -self.clock_jump_ns or
                (self.epoch_offset_ns is not None and
                 abs(epoch_offset - self.epoch_offset_ns) > self.clock_jump_ns)):
            self.clock_fault = True
            self.counts["clock_fault"] += 1
            self.error = "system clock stepped; restart native bridge"
            raise ClockDiscontinuity(self.error)

        reset = session != self.session
        if not reset and self.previous is not None:
            old_sequence, old_ms, old_mono = self.previous
            if mono_ns < old_mono:
                self.counts["out_of_order"] += 1
                return None
            seq_delta = (sequence - old_sequence) % UINT32
            ms_delta = (milliseconds - old_ms) % UINT32
            if seq_delta == 0:
                self.counts["duplicate"] += 1
                return None
            if seq_delta >= UINT32 // 2 or ms_delta >= UINT32 // 2:
                if (sequence < old_sequence and milliseconds < old_ms and
                        seq_delta >= UINT32 // 2 and ms_delta >= UINT32 // 2):
                    # Serial preserves wire order: both counters moving back
                    # indicate a board reset. A normal uint32 wrap moves forward.
                    reset = True
                    self.counts["device_resets"] += 1
                else:
                    self.counts["out_of_order"] += 1
                    return None
            if not reset:
                self.counts["sequence_gaps"] += seq_delta - 1
                self.elapsed_ns += ms_delta * NS_PER_MS
        if reset:
            if session != self.session and self.session is not None:
                self.counts["connection_changes"] += 1
            self.elapsed_ns = 0
            self.offset_mono_ns = mono_ns
        self.session = session
        self.previous = (sequence, milliseconds, mono_ns)
        self.epoch_offset_ns = epoch_offset
        self.offset_mono_ns = min(self.offset_mono_ns, mono_ns - self.elapsed_ns)
        stamp_ns = self.offset_mono_ns + self.elapsed_ns + epoch_offset
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.counts["nonmonotonic_stamp"] += 1
            self.error = "discarded non-increasing reconstructed timestamp"
            return None
        self.last_stamp_ns = stamp_ns
        self.last_receive_ns = rx_ns
        self.counts["published"] += 1
        self.error = ""
        return Sample(stamp_ns, acceleration, angular_velocity)


def to_ros_message(sample, frame_id):
    from sensor_msgs.msg import Imu

    message = Imu()
    message.header.frame_id = frame_id
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample.stamp_ns, 10**9)
    message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = sample.acceleration
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = sample.angular_velocity
    # Firmware quaternion conventions have not been checked; only publish
    # acceleration and angular velocity, explicitly leaving orientation absent.
    message.orientation_covariance[0] = -1.0
    # Zero acceleration/gyro covariance denotes unknown, not zero measurement noise.
    return message
