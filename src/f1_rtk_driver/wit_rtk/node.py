"""Read WTRTK-982 serial output and publish independent position and heading."""

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

from f1_rtk_driver.msg import Heading, RawSentence, RtkStatus
from wit_rtk.alignment import EpochPairs, base_coordinates, bow_heading, utc_nanoseconds
from wit_rtk.discovery import ReceiverDiscovery
from wit_rtk.nmea import Fix, parse_sentence


class RtkDriver(Node):
    def __init__(self):
        super().__init__("rtk_driver", namespace="/f1/rtk")
        self.port = self.declare_parameter("port", "auto").value
        self.baudrate = self.declare_parameter("baudrate", 115200).value
        self.stale_timeout = self.declare_parameter("stale_timeout_sec", 1.5).value
        self.reconnect = self.declare_parameter("reconnect_sec", 1.0).value
        self.frame = self.declare_parameter("antenna_frame", "gnss_ant1").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.mounting_deg = self.declare_parameter("baseline_clockwise_from_bow_deg", 42.0).value
        self.antenna_x = self.declare_parameter("antenna_x_m", -0.191).value
        self.antenna_y = self.declare_parameter("antenna_y_m", 0.213).value
        if not self.port or not self.frame or not self.base_frame or self.baudrate <= 0:
            raise ValueError("port, antenna_frame, base_frame and a positive baudrate are required")
        if not all(math.isfinite(v) for v in (self.mounting_deg, self.antenna_x, self.antenna_y)):
            raise ValueError("antenna mounting parameters must be finite")
        if not all(math.isfinite(v) and v > 0 for v in (self.stale_timeout, self.reconnect)):
            raise ValueError("stale_timeout_sec and reconnect_sec must be finite and positive")

        self.fix_pub = self.create_publisher(NavSatFix, "fix", 20)
        self.heading_pub = self.create_publisher(Heading, "heading", 20)
        self.base_fix_pub = self.create_publisher(NavSatFix, "base_fix", 20)
        self.bow_heading_pub = self.create_publisher(Heading, "bow_heading", 20)
        self.raw_pub = self.create_publisher(RawSentence, "raw", 100)
        self.status_pub = self.create_publisher(RtkStatus, "status", 10)
        self.device = None
        self.discovery = ReceiverDiscovery(self.baudrate, self.reconnect) if self.port == "auto" else None
        self.serial_error = ""
        self.retry_at = 0.0
        self.partial = bytearray()
        self.discard_line = False
        self.rejected = 0
        self.fix = None
        self.heading = None
        self.fix_valid = False
        self.heading_valid = False
        self.last_valid_fix = None
        self.last_valid_heading = None
        self.pairs = EpochPairs(self.stale_timeout)
        self.last_base_fix = None
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.025, self.poll, clock=steady)
        self.create_timer(0.2, self.publish_status, clock=steady)

    def header(self):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame)

    def poll(self):
        now = time.monotonic()
        if self.device is None and now < self.retry_at:
            return
        try:
            if self.device is None:
                if self.discovery is None:
                    self.device = serial.Serial(self.port, self.baudrate, timeout=0, exclusive=True)
                else:
                    self.device = self.discovery.poll(now)
                    if self.device is None:
                        self.serial_error = self.discovery.reason
                        return
                self.serial_error = ""
                self.get_logger().info(f"Opened RTK {self.device.port} at {self.baudrate} baud")
            data = self.device.read(4096)
        except (OSError, serial.SerialException) as exc:
            self.disconnect(str(exc), now)
            return
        if data:
            self.consume(data, now)
        self.expire(now)

    def consume(self, data, now):
        # Keep partial lines between reads. Discard an oversized line through its LF.
        parts = data.split(b"\n")
        for index, fragment in enumerate(parts):
            if not self.discard_line:
                if len(self.partial) + len(fragment) > 1024:
                    self.rejected += 1
                    self.discard_line = True
                    self.partial.clear()
                else:
                    self.partial.extend(fragment)
            if index < len(parts) - 1:
                if self.partial:
                    self.process_line(bytes(self.partial).rstrip(b"\r"), now)
                self.partial.clear()
                self.discard_line = False

    def process_line(self, line, now):
        if not line:
            return
        header = self.header()
        self.raw_pub.publish(RawSentence(header=header, sentence=line.decode("ascii", "backslashreplace")))
        try:
            sample = parse_sentence(line)
        except ValueError:
            self.rejected += 1
            return
        if sample is None:
            return
        if isinstance(sample, Fix):
            self.fix = sample
            self.fix_valid = sample.valid
            if sample.valid:
                self.last_valid_fix = now
            fix = self.fix_message(header, sample)
            self.fix_pub.publish(fix)
            self.update_base_fix(fix, sample, now)
        else:
            self.heading = sample
            self.heading_valid = sample.valid
            if sample.valid:
                self.last_valid_heading = now
            bow = self.publish_headings(header, sample)
            self.update_base_fix(bow, sample, now)

    @staticmethod
    def fix_message(header, sample=None):
        msg = NavSatFix(header=header, latitude=math.nan, longitude=math.nan, altitude=math.nan)
        msg.status.status = NavSatStatus.STATUS_NO_FIX
        if sample is not None and sample.valid:
            msg.latitude, msg.longitude, msg.altitude = sample.latitude, sample.longitude, sample.altitude
            # GGA does not identify the constellation or the source of a quality-2 correction.
            msg.status.status = (NavSatStatus.STATUS_GBAS_FIX if sample.quality in (4, 5)
                                 else NavSatStatus.STATUS_FIX)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    def base_header(self, header):
        return Header(stamp=header.stamp, frame_id=self.base_frame)

    def publish_headings(self, header, sample=None):
        msg = Heading(header=header, quality=-1, heading_deg=math.nan)
        if sample is not None:
            msg.utc, msg.quality = sample.utc, sample.quality
            msg.valid, msg.heading_deg = sample.valid, sample.degrees
        self.heading_pub.publish(msg)
        bow = Heading(header=self.base_header(header), utc=msg.utc, quality=msg.quality,
                      valid=msg.valid, heading_deg=bow_heading(msg.heading_deg, self.mounting_deg))
        self.bow_heading_pub.publish(bow)
        return bow

    def update_base_fix(self, message, sample, now):
        epoch = utc_nanoseconds(sample.utc)
        is_fix = isinstance(sample, Fix)
        if not sample.valid or epoch is None:
            self.pairs.invalidate(epoch, now)
            self.invalidate_base_fix(message.header, report=is_fix)
            return
        match = self.pairs.add(epoch, now, message, is_fix)
        if match is None:
            return
        fix, bow, oldest_receipt = match
        latitude, longitude = base_coordinates(fix.latitude, fix.longitude, bow.heading_deg,
                                               self.antenna_x, self.antenna_y)
        self.base_fix_pub.publish(NavSatFix(
            header=self.base_header(fix.header), status=fix.status, latitude=latitude,
            longitude=longitude, altitude=math.nan,
            position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN))
        self.last_base_fix = oldest_receipt

    def invalidate_base_fix(self, header, report=False):
        if report or self.last_base_fix is not None:
            self.base_fix_pub.publish(self.fix_message(self.base_header(header)))
        self.last_base_fix = None

    def expire(self, now):
        if self.fix_valid and (self.device is None or now - self.last_valid_fix >= self.stale_timeout):
            self.fix_valid = False
            self.fix_pub.publish(self.fix_message(self.header()))
        if self.heading_valid and (self.device is None or now - self.last_valid_heading >= self.stale_timeout):
            self.heading_valid = False
            self.publish_headings(self.header())
        if self.last_base_fix is not None and (
                self.device is None or now - self.last_base_fix >= self.stale_timeout):
            self.invalidate_base_fix(self.header())

    def disconnect(self, error, now):
        if self.device is not None:
            self.device.close()
            self.device = None
            self.expire(now)
        self.partial.clear()
        self.discard_line = False
        self.pairs.clear()
        self.fix = self.heading = None
        self.retry_at = now + self.reconnect
        if error != self.serial_error:
            self.get_logger().warning(error)
        self.serial_error = error

    def publish_status(self):
        now = time.monotonic()
        self.expire(now)
        self.status_pub.publish(RtkStatus(
            header=self.header(), connected=self.device is not None, serial_error=self.serial_error,
            fix_quality=self.fix.quality if self.fix else -1,
            heading_quality=self.heading.quality if self.heading else -1,
            fix_valid=self.fix_valid, heading_valid=self.heading_valid,
            fix_age_sec=-1.0 if self.last_valid_fix is None else now - self.last_valid_fix,
            heading_age_sec=-1.0 if self.last_valid_heading is None else now - self.last_valid_heading,
            fix_utc=self.fix.utc if self.fix else "", satellites=self.fix.satellites if self.fix else -1,
            hdop=self.fix.hdop if self.fix else math.nan, rejected_lines=self.rejected))

    def destroy_node(self):
        if self.discovery is not None:
            self.discovery.close()
        if self.device is not None:
            self.device.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtkDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
