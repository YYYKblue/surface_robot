import secrets

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from f1_interfaces.msg import WallObservation

from .perception import Perception, Scan
from .ros_support import configuration, stamp_ns, to_observation_message


class WallPerception(Node):
    def __init__(self, **kwargs):
        super().__init__("pool_perception", **kwargs)
        self.config = configuration(self)
        self.estimator = Perception(self.config)
        self.session_id = secrets.token_hex(12)
        self.sequence = 0
        self.pub = self.create_publisher(WallObservation, "/pool_cleaning/wall_observation", qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(Marker, "/pool_cleaning/wall_marker", 2)
        # Control needs the newest scan, not a queue of old scans when fitting
        # takes longer than the sensor period. Absolute age checks still apply.
        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(LaserScan, "/pavo2_scan", self._scan, latest_scan)
        self.reset_service = self.create_service(Trigger, "/pool_cleaning/reset_observation", self._reset)
        self.previous_clock = 0

    def _reset(self, request, response):
        self.estimator.reset()
        self.session_id = secrets.token_hex(12)
        self.sequence = 0
        response.success, response.message = True, "association reset; active controller must pause"
        return response

    def _scan(self, message):
        now = self.get_clock().now().nanoseconds
        if now < self.previous_clock:
            self.estimator.reset()
            self.session_id = secrets.token_hex(12)
            self.sequence = 0
        self.previous_clock = now
        scan = Scan(stamp_ns(message.header.stamp), message.header.frame_id,
                    message.angle_min, message.angle_increment, tuple(message.ranges),
                    message.range_min, message.range_max, message.time_increment)
        observation = self.estimator.process(scan, now)
        self.sequence += 1
        output = to_observation_message(observation, self.config.base_frame, self.session_id, self.sequence)
        self.pub.publish(output)
        marker = Marker()
        marker.header = output.header
        marker.ns, marker.id = "left_wall", 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD if output.valid else Marker.DELETE
        marker.pose.orientation.w = 1.
        marker.scale.x = .02
        marker.color.g, marker.color.a = 1., 1.
        marker.lifetime.nanosec = 200_000_000
        marker.points = [Point(x=float(x), y=float(y), z=0.) for x, y in (observation.start, observation.end)]
        self.marker_pub.publish(marker)


def main(args=None):
    from .runner import run
    run(WallPerception, args)
