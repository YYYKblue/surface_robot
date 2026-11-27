"""Quality-gated ANT1 position observations in a site-local WGS84 tangent plane."""
from dataclasses import dataclass
import math

from f1_interfaces.msg import RawSentence
from geometry_msgs.msg import PointStamped, Pose
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wit_rtk.nmea import parse_sentence, utc_nanoseconds


ORIGIN_PREFIX = "f1_gga_v2:"
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
        if name not in (origin.id, origin.id.replace(ORIGIN_PREFIX, "f1_enu_v1:")):
            raise ValueError("invalid ENU landmark ID")
        return origin

    def project(self, latitude, longitude):
        # Local WGS84 tangent plane for a pool/site. No vertical observation.
        phi = math.radians(self.latitude)
        e2 = 6.6943799901413165e-3
        w = math.sqrt(1 - e2 * math.sin(phi)**2)
        delta_lon = (longitude - self.longitude + 180) % 360 - 180
        east = math.radians(delta_lon) * (6378137.0 / w) * math.cos(phi)
        north = math.radians(latitude - self.latitude) * 6378137.0 * (1 - e2) / w**3
        return east, north


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


class GnssPositions:
    def __init__(self, node, origin=None, map_position=None):
        self.node = node
        self.origin = origin
        self.map_position = map_position
        self.last_clock = None
        self.reset()

        def setting(name, default):
            return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.fixed_only = setting("require_fixed_position", False)
        self.frame = setting("antenna_frame", "gnss_ant1")
        rate = setting("gnss_constraint_rate_hz", 2.0)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("GNSS constraint rate must be finite and positive")
        self.period_ns = round(1e9 / rate)
        self.pub = node.create_publisher(PointStamped, "/f1/rtk/position_constraint", 10)
        node.create_subscription(RawSentence, "/f1/rtk/raw", self.raw, 50)

    def reset(self):
        self.last_stamp = self.last_published = self.consistent = 0
        self.last_epoch = self.previous = None

    def raw(self, message):
        if message.header.frame_id != self.frame or message.sentence[3:7] != "GGA,":
            return
        now = self.node.get_clock().now().nanoseconds
        stamp = stamp_ns(message.header)
        if self.last_clock is not None and now < self.last_clock:
            self.reset()
        self.last_clock = now
        if stamp <= self.last_stamp or not -100_000_000 <= now - stamp <= MAX_AGE_NS:
            return
        try:
            fix = parse_sentence(message.sentence.encode("ascii"))
        except ValueError:
            return
        if fix is None:
            return
        epoch = utc_nanoseconds(fix.utc)
        dt = (stamp - self.last_stamp) * 1e-9
        if (not fix.valid or epoch is None or not -90 < fix.latitude < 90
                or fix.quality not in ((4,) if self.fixed_only else (1, 2, 4, 5))
                or not 0 < fix.hdop <= 3 or fix.satellites < 5):
            self.previous = None
            self.consistent = 0
            self.last_stamp = stamp
            return
        if self.last_epoch is not None and dt < 43200:
            delta = (epoch - self.last_epoch) % (86400 * 10**9)
            if not 0 < delta < 43200 * 10**9:
                return
        self.last_stamp, self.last_epoch = stamp, epoch
        site = self.origin or Origin.from_id(Origin(fix.latitude, fix.longitude).id)
        xy = site.project(fix.latitude, fix.longitude)
        consistent = (self.previous is not None and dt <= .5 and
                      math.dist(xy, site.project(*self.previous)) <= 2 + 3 * dt)
        self.previous = fix.latitude, fix.longitude
        self.consistent = self.consistent + 1 if consistent else 1
        if self.consistent < 3 or stamp - self.last_published < self.period_ns:
            return
        frame = site.id
        if self.map_position is not None:
            projected = self.map_position(xy, frame)
            if projected is None:
                return
            xy, frame = projected
        if self.origin is None:
            self.origin = site
            self.node.get_logger().info(f"GNSS position datum {site.id}")
        observation = PointStamped()
        observation.header.stamp = message.header.stamp
        observation.header.frame_id = frame
        observation.point.x, observation.point.y = xy
        self.pub.publish(observation)
        self.last_published = stamp


def main():
    rclpy.init()
    node = Node("gnss_positions")
    try:
        GnssPositions(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
