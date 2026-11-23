from functools import reduce
import math
from operator import xor
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from f1_interfaces.msg import Heading, RawSentence
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from f1_mapping_bringup.gnss_landmarks import GnssLandmarks, Origin, PlanarPose


def inputs(index, heading_epoch=None, heading_quality=4, hdop=1.1):
    utc = f"123000.{index:02}"
    header = Header(stamp=Time(sec=10, nanosec=index * 10_000_000), frame_id="base_link")
    fix = NavSatFix(header=header, latitude=23.0, longitude=114.0, altitude=math.nan)
    body = f"GNGGA,{utc},2300.000,N,11400.000,E,1,12,{hdop},0,M,0,M,,"
    raw = RawSentence(header=header, sentence=f"${body}*{reduce(xor, body.encode()):02X}")
    hpr_header = Header(stamp=Time(sec=10, nanosec=index * 10_000_000 + 1_000_000), frame_id="base_link")
    heading = Heading(header=hpr_header, utc=heading_epoch or utc, heading_deg=0.,
                      valid=True, quality=heading_quality)
    return fix, heading, raw


def adapter():
    node = Mock()
    node.declare_parameter.side_effect = lambda key, default, descriptor: SimpleNamespace(value=default)
    node.get_clock.return_value.now.return_value.nanoseconds = 10_240_000_000
    return GnssLandmarks(node)


def test_join_receipt_stamp_and_utc_in_either_dds_order():
    fusion = adapter()
    for i in (5, 10, 15):
        fix, heading, raw = inputs(i)
        callbacks = [(fusion.center, fix), (fusion.raw, raw), (fusion.heading, heading)]
        for cb, msg in callbacks if i == 10 else reversed(callbacks):
            cb(msg)
    landmark = fusion.pub.publish.call_args.args[0]
    assert landmark.header == fix.header
    entry, = landmark.landmarks
    assert Origin.from_id(entry.id) == Origin(23., 114.)
    assert entry.tracking_from_landmark_transform.orientation.z == pytest.approx(-math.sqrt(.5))
    assert fusion.pub.publish.call_count == 1


@pytest.mark.parametrize("case", ["bad UTC", "float heading", "HDOP", "stale", "receipt mismatch", "no fix"])
def test_unusable_pairs_make_no_constraints(case):
    fusion = adapter()
    for i in (5, 10, 15, 20):
        fix, heading, raw = inputs(i, heading_epoch="123001" if case == "bad UTC" else None,
                                   heading_quality=5 if case == "float heading" else 4,
                                   hdop=4.0 if case == "HDOP" else 1.1)
        if case == "stale":
            fusion.node.get_clock.return_value.now.return_value.nanoseconds += 2_000_000_000
        if case == "receipt mismatch":
            raw.header = Header(stamp=Time(sec=1, nanosec=i))
        if case == "no fix":
            fix.status.status = -1
        fusion.raw(raw)
        fusion.center(fix)
        fusion.heading(heading)
    fusion.pub.publish.assert_not_called()


def test_explicit_heading_loss_requires_three_new_pairs():
    fusion = adapter()
    for i in (5, 10, 15, 20, 25, 30):
        fix, heading, raw = inputs(i, heading_quality=5 if i == 15 else 4)
        fusion.raw(raw)
        fusion.heading(heading)
        fusion.center(fix)
        assert fusion.pub.publish.call_count == (1 if i == 30 else 0)


def test_local_enu_projection_and_pose_inversion():
    origin = Origin(0, 179.99999)
    pose = origin.vessel_pose(.00001, -179.99999, 90.)
    assert pose.x == pytest.approx(2.22639, abs=.001)
    assert pose.y == pytest.approx(1.10574, abs=.001)
    assert pose.yaw == pytest.approx(0.)
    reference = PlanarPose(3., 2., math.pi / 2)
    map_pose = reference.compose(pose)
    assert (map_pose.x, map_pose.y) == pytest.approx((3 - pose.y, 2 + pose.x))
    recovered = reference.inverse().compose(map_pose)
    assert (recovered.x, recovered.y, recovered.yaw) == pytest.approx((pose.x, pose.y, pose.yaw))
