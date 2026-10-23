"""Actual C IMU formatter -> native serial decoder/converter, without hardware."""
import ctypes as c
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from nano_ros_bridge.protocol import decode, Lines
from nano_ros_bridge.imu.conversion import Converter, to_ros_message


class Raw(c.Structure):
    _fields_ = [(n, c.c_int16) for n in
                ("acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
                 "roll", "pitch", "yaw", "q0", "q1", "q2", "q3")] + [
                    (n, c.c_uint32) for n in
                    ("frame_count", "time_frame_count", "acc_frame_count",
                     "gyro_frame_count", "angle_frame_count", "quat_frame_count")
                ] + [("sample_ms", c.c_uint32 * 5), ("received_mask", c.c_uint8)]


class ReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ.get("F1_FIRMWARE_DIR", Path(__file__).resolve().parents[4] / "firmware"))
        temporary = tempfile.TemporaryDirectory(prefix="f1_imu_formatter_")
        cls.addClassCleanup(temporary.cleanup)
        library = Path(temporary.name) / "imu_report.so"
        subprocess.run([
            "cc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
            f"-I{root / 'USER'}", f"-I{root / 'SYSTEM/usart4'}",
            str(Path(__file__).with_name("imu_report_harness.c")),
            str(root / "USER/imu_report.c"), "-o", str(library),
        ], check=True, capture_output=True)
        cls.lib = c.CDLL(str(library))
        cls.lib.test_raw_size.restype = c.c_uint
        cls.lib.test_format.argtypes = [c.POINTER(Raw), c.c_uint32, c.c_void_p, c.c_uint16]
        cls.lib.test_format.restype = c.c_uint16
        cls.lib.test_sequence.argtypes = [c.c_uint32]
        cls.lib.test_yaw_limit.argtypes = [c.c_int32]
        assert cls.lib.test_raw_size() == c.sizeof(Raw)

    def setUp(self):
        self.lib.test_init()
        self.raw = Raw()

    def fresh(self, now=100):
        self.raw.received_mask = 31
        self.raw.sample_ms[:] = [now] * 5
        self.raw.angle_frame_count += 1

    def report(self, now):
        buf = c.create_string_buffer(256)
        n = self.lib.test_format(c.byref(self.raw), now, buf, 256)
        self.assertGreater(n, 0)
        self.assertEqual(n, len(buf.value))
        line = buf.value.decode()
        self.assertTrue(line.endswith("\r\n"))
        self.assertLess(n * 10 / 460800, .02)
        return line, decode(buf.value)

    def test_startup_has_no_valid_sensor_fields(self):
        _, event = self.report(0)
        self.assertEqual(event["status"], 0)
        self.assertEqual(event["sequence"], 0)
        self.assertEqual(event["angles_degrees"][:3], [0, 0, 0])

    def test_units_and_quaternion_order(self):
        self.fresh()
        self.raw.acc_z = 2048  # +1g
        self.raw.gyro_x = 16384  # +1000 deg/s
        self.raw.gyro_y = -16384
        self.raw.roll, self.raw.pitch = 16384, -16384
        self.raw.q0, self.raw.q1, self.raw.q2, self.raw.q3 = 16384, -8192, 4096, -2048
        _, event = self.report(100)
        self.assertAlmostEqual(event["acceleration"][2], 9.80665, delta=.0005)
        self.assertAlmostEqual(event["angular_velocity"][0], 1000 * math.pi / 180, delta=.0001)
        self.assertAlmostEqual(event["angular_velocity"][1], -1000 * math.pi / 180, delta=.0001)
        self.assertEqual(event["angles_degrees"][:2], [90, -90])
        self.assertEqual(event["quaternion_wxyz"], [.5, -.25, .125, -.0625])
        self.assertEqual(event["status"], 31)

        converter = Converter(required_status_mask=6)
        sample = converter.convert(event, 1_800_000_000_000_000_000,
                                   session="serial-test", received_unix_ns=1_800_000_000_000_000_000,
                                   received_monotonic_ns=1_000_000_000)
        message = to_ros_message(sample, "imu_link")
        self.assertAlmostEqual(message.linear_acceleration.z, 9.80665, delta=.0005)
        self.assertAlmostEqual(message.angular_velocity.x, 1000 * math.pi / 180, delta=.0001)
        self.assertEqual(message.orientation_covariance[0], -1)

    def test_each_field_expires_independently(self):
        self.fresh(100)
        self.raw.sample_ms[3] = 299
        _, event = self.report(300)
        self.assertEqual(event["status"], 8)
        _, event = self.report(499)
        self.assertEqual(event["status"], 0)

    def test_identical_sensor_values_remain_valid_when_frames_arrive(self):
        for now in range(0, 1000, 20):
            self.fresh(now)
            _, event = self.report(now)
            self.assertEqual(event["status"], 31)
            self.assertEqual(event["sequence"], now // 20)

    def test_yaw_wrap_and_repeated_snapshot(self):
        self.fresh(100)
        self.raw.yaw = round(179 * 32768 / 180)
        self.report(100)
        self.fresh(120)
        self.raw.yaw = round(-179 * 32768 / 180)
        _, event = self.report(120)
        self.assertAlmostEqual(event["angles_degrees"][3], 181, delta=.02)
        _, repeat = self.report(140)
        self.assertEqual(repeat["angles_degrees"][3], event["angles_degrees"][3])
        self.report(320)  # stale: continuity no longer trustworthy
        self.fresh(340)
        self.raw.yaw = 0
        _, recovered = self.report(340)
        self.assertEqual(recovered["angles_degrees"][3], 0)

    def test_time_and_sequence_wrap(self):
        self.fresh(0xFFFFFFF0)
        self.lib.test_sequence(0xFFFFFFFF)
        _, event = self.report(4)
        self.assertEqual(event["status"], 31)
        self.assertEqual(event["sequence"], 0xFFFFFFFF)
        _, event = self.report(24)
        self.assertEqual(event["sequence"], 0)

    def test_boundaries_and_short_destination(self):
        self.fresh()
        for n, t in Raw._fields_:
            if t is c.c_int16:
                setattr(self.raw, n, -32768)
        line, event = self.report(100)
        self.assertEqual(event["angles_degrees"][:3], [-180, -180, -180])
        self.assertEqual(event["quaternion_wxyz"], [-1, -1, -1, -1])
        buf = c.create_string_buffer(b"Z" * 32, 33)
        self.assertEqual(self.lib.test_format(c.byref(self.raw), 120, buf, 8), 0)
        self.assertEqual(buf.raw[8:32], b"Z" * 24)
        self.assertEqual(buf.raw[7], 0)
        with self.assertRaises(ValueError):
            decode(line.replace("IMU,", "IMU,9", 1).encode())

    def test_fragmented_and_consecutive_frames(self):
        self.fresh()
        a, _ = self.report(100)
        b, _ = self.report(120)
        decoder = Lines()
        combined = (a + b).encode()
        result = []
        for start in range(0, len(combined), 7):
            result.extend(decoder.feed(combined[start:start + 7]))
        self.assertEqual([decode(x)["sequence"] for x in result], [0, 1])

    def test_long_running_yaw_saturates_instead_of_overflow(self):
        self.fresh()
        self.report(100)
        self.lib.test_yaw_limit(0x7FFFFFF0)
        self.fresh(120)
        self.raw.yaw = 8192
        _, event = self.report(120)
        self.assertEqual(event["angles_degrees"][3], 21474836.47)


if __name__ == "__main__":
    unittest.main()
