"""Fuse a same-epoch vessel fix and true heading as an oriented ENU landmark."""
from dataclasses import dataclass
import math

from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList
from f1_interfaces.msg import Heading, RawSentence
from geometry_msgs.msg import Pose
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from wit_rtk.alignment import utc_nanoseconds
from wit_rtk.nmea import Fix, parse_sentence


ORIGIN_PREFIX = "f1_enu_v1:"
MAX_AGE_NS = 500_000_000


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float

    def compose(self, child):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return PlanarPose(self.x + c * child.x - s * child.y,
                          self.y + s * child.x + c * child.y, self.yaw + child.yaw)

    def inverse(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return PlanarPose(-c * self.x - s * self.y, s * self.x - c * self.y, -self.yaw)

    def message(self):
        msg = Pose()
        msg.position.x, msg.position.y = self.x, self.y
        msg.orientation.z, msg.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        return msg


@dataclass(frozen=True)
class Origin:
    latitude: float
    longitude: float

    def __post_init__(self):
        if not (-90 < self.latitude < 90 and -180 <= self.longitude <= 180):
            raise ValueError("invalid local ENU origin")

    @property
    def id(self):
        return f"{ORIGIN_PREFIX}{self.latitude:.8f}:{self.longitude:.8f}"

    @classmethod
    def from_id(cls, name):
        _, lat, lon = name.split(":")
        origin = cls(float(lat), float(lon))
        if name != origin.id:
            raise ValueError("invalid ENU landmark ID")
        return origin

    def vessel_pose(self, latitude, longitude, heading):
        # Local WGS84 tangent plane for a pool/site. No vertical observation.
        phi = math.radians(self.latitude)
        e2 = 6.6943799901413165e-3
        w = math.sqrt(1 - e2 * math.sin(phi)**2)
        delta_lon = (longitude - self.longitude + 180) % 360 - 180
        east = math.radians(delta_lon) * (6378137.0 / w) * math.cos(phi)
        north = math.radians(latitude - self.latitude) * 6378137.0 * (1 - e2) / w**3
        return PlanarPose(east, north, math.radians(90 - heading))


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


class GnssLandmarks:
    """Works with live driver topics and bags recorded before fusion was added."""

    def __init__(self, node, origin=None, accept_pose=None):
        self.node = node
        self.origin = origin
        self.accept_pose = accept_pose
        self.centers = {}
        self.epochs = {}
        self.headings = {}
        self.last_stamp = 0
        self.last_sample = None
        self.consistent = 0
        self.last_published = 0
        self.last_clock = None

        def setting(name, default):
            return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.position_weight = setting("gnss_translation_weight", 0.5)
        self.heading_weight = setting("gnss_rotation_weight", 8.0)
        rate = setting("gnss_constraint_rate_hz", 2.0)
        if not all(math.isfinite(v) and v > 0 for v in
                   (rate, self.position_weight, self.heading_weight)):
            raise ValueError("GNSS constraint rate and weights must be finite and positive")
        self.period_ns = round(1e9 / rate)
        self.pub = node.create_publisher(LandmarkList, "/f1/rtk/landmarks", 10)
        node.create_subscription(RawSentence, "/f1/rtk/raw", self.raw, 50)
        node.create_subscription(Heading, "/f1/rtk/bow_heading", self.heading, 30)
        node.create_subscription(NavSatFix, "/f1/rtk/base_fix", self.center, 30)

    def raw(self, message):
        line = message.sentence
        if line[3:7] != "GGA,":
            return
        try:
            fix = parse_sentence(line.encode("ascii"))
        except ValueError:
            return
        if not isinstance(fix, Fix):
            return
        # HDOP and satellite count qualify the position, independently of HPR.
        if not (fix.valid and fix.quality in (1, 2, 4, 5) and 0 < fix.hdop <= 3 and fix.satellites >= 5):
            self.invalidate(message.header)
            return
        epoch = utc_nanoseconds(fix.utc)
        if epoch is not None:
            self.remember(self.epochs, stamp_ns(message.header), epoch)

    def heading(self, message):
        if message.header.frame_id != "base_link":
            return
        if not (message.valid and message.quality == 4 and 0 <= message.heading_deg < 360):
            self.invalidate(message.header)
            return
        epoch = utc_nanoseconds(message.utc)
        if epoch is not None:
            self.remember(self.headings, epoch, message)

    def center(self, message):
        if message.header.frame_id != "base_link":
            return
        if (message.status.status == NavSatStatus.STATUS_NO_FIX
                or not -90 < message.latitude < 90 or not -180 <= message.longitude <= 180):
            self.invalidate(message.header)
            return
        self.remember(self.centers, stamp_ns(message.header), message)

    def invalidate(self, header):
        stamp = stamp_ns(header)
        age = self.node.get_clock().now().nanoseconds - stamp
        if stamp >= self.last_stamp and -100_000_000 <= age <= MAX_AGE_NS:
            self.last_stamp = stamp
            self.last_sample = None
            self.consistent = 0

    def remember(self, cache, key, value):
        now = self.node.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            for pending in (self.centers, self.epochs, self.headings):
                pending.clear()
            self.last_published = self.last_stamp = self.consistent = 0
            self.last_sample = None
        self.last_clock = now
        cache[key] = value
        if len(cache) > 32:
            del cache[next(iter(cache))]
        # base_fix carries the GGA receipt stamp. bow_heading carries HPR UTC.
        # The raw GGA connects those two keys even if DDS delivers out of order.
        for stamp, fix in list(self.centers.items()):
            heading = self.headings.get(self.epochs.get(stamp))
            if stamp <= self.last_stamp or now - stamp > MAX_AGE_NS:
                del self.centers[stamp]
            elif heading is not None:
                del self.centers[stamp]
                heading_stamp = stamp_ns(heading.header)
                if (all(-100_000_000 <= now - t <= MAX_AGE_NS for t in (stamp, heading_stamp))
                        and abs(stamp - heading_stamp) <= MAX_AGE_NS):
                    self.observe(fix, heading, stamp)

    def observe(self, fix, heading, stamp):
        origin = self.origin or Origin(fix.latitude, fix.longitude)
        # Compare successive pairs in the same tangent plane even before a datum is accepted.
        current = origin.vessel_pose(fix.latitude, fix.longitude, heading.heading_deg)
        dt = (stamp - self.last_stamp) * 1e-9
        consistent = False
        if self.last_sample is not None and 0 < dt <= 0.5:
            lat, lon, degrees = self.last_sample
            previous = origin.vessel_pose(lat, lon, degrees)
            angle = abs(math.remainder(current.yaw - previous.yaw, 2 * math.pi))
            consistent = (math.hypot(current.x - previous.x, current.y - previous.y) <= 2 + 3 * dt
                          and angle <= math.radians(12 + 120 * dt))
        self.consistent = self.consistent + 1 if consistent else 1
        self.last_stamp = stamp
        self.last_sample = fix.latitude, fix.longitude, heading.heading_deg
        if self.consistent < 3 or stamp - self.last_published < self.period_ns:
            return
        if self.accept_pose is not None and not self.accept_pose(current, fix.header):
            return
        if self.origin is None:
            self.origin = Origin.from_id(origin.id)
            self.node.get_logger().info(f"GNSS landmark {self.origin.id}")
            current = self.origin.vessel_pose(fix.latitude, fix.longitude, heading.heading_deg)
        entry = LandmarkEntry(id=self.origin.id, tracking_from_landmark_transform=current.inverse().message(),
                              translation_weight=self.position_weight, rotation_weight=self.heading_weight)
        self.pub.publish(LandmarkList(header=fix.header, landmarks=[entry]))
        self.last_published = stamp


def main():
    rclpy.init()
    node = Node("gnss_landmarks")
    try:
        GnssLandmarks(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
