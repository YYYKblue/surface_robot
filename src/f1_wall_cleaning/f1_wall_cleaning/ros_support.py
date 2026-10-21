from dataclasses import fields

from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor
from f1_wall_interfaces.msg import WallObservation

from .config import Config
from .perception import Observation


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def stamp(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def configuration(node):
    default = Config()
    values = {}
    for field in fields(default):
        node.declare_parameter(field.name, getattr(default, field.name), ParameterDescriptor(read_only=True))
        values[field.name] = node.get_parameter(field.name).value
    return Config(**values).validate()


def to_observation_message(observation, frame, session, sequence):
    o = observation
    m = WallObservation()
    m.header.stamp, m.header.frame_id = stamp(o.stamp_ns), frame
    m.session, m.sequence = session, sequence
    m.valid, m.reason = o.valid, o.reason
    m.distance_m, m.tangent_rad, m.residual_m = o.distance, o.angle, o.residual
    m.support_m, m.inliers, m.inlier_ratio = o.support, o.inliers, o.ratio
    m.start_xy, m.end_xy = list(o.start), list(o.end)
    m.front_observed, m.front_clearance_m = o.front_observed, o.front_clearance
    m.front_valid_fraction, m.rigid_collision = o.front_fraction, o.rigid_collision
    m.left_turn_clear, m.right_turn_clear = o.left_turn_clear, o.right_turn_clear
    return m


def from_observation_message(m):
    return Observation(stamp_ns(m.header.stamp), m.valid, m.reason,
                       m.distance_m, m.tangent_rad, m.residual_m, m.support_m,
                       m.inliers, m.inlier_ratio, tuple(m.start_xy), tuple(m.end_xy),
                       m.front_observed, m.front_clearance_m, m.front_valid_fraction,
                       m.rigid_collision, m.left_turn_clear, m.right_turn_clear)
