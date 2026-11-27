from functools import reduce
from operator import xor
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from f1_interfaces.msg import RawSentence
from std_msgs.msg import Header

from f1_mapping_bringup.gnss_positions import GnssPositions, Origin


def raw(index, utc=None, quality=5, hdop=1., satellites=12):
    utc = f"123000.{index:02}" if utc is None else utc
    header = Header(stamp=Time(sec=10, nanosec=index * 10_000_000), frame_id="gnss_ant1")
    body = f"GNGGA,{utc},2300.000,N,11400.000,E,{quality},{satellites},{hdop},0,M,0,M,,"
    return RawSentence(header=header, sentence=f"${body}*{reduce(xor, body.encode()):02X}")


def adapter():
    node = Mock()
    node.declare_parameter.side_effect = lambda key, default, descriptor: SimpleNamespace(value=default)
    node.get_clock.return_value.now.return_value.nanoseconds = 10_240_000_000
    return GnssPositions(node)


def test_three_distinct_gga_positions_and_ignored_hpr():
    fusion = adapter()
    for i in (5, 10, 15):
        msg = raw(i)
        other = RawSentence(header=msg.header, sentence="$GNHPR,,,,,,,,*43")
        fusion.raw(other)
        fusion.raw(msg)
    point = fusion.pub.publish.call_args.args[0]
    assert Origin.from_id(point.header.frame_id) == Origin(23., 114.)
    assert point.header.stamp == msg.header.stamp and point.point.x == point.point.y == 0.
    assert fusion.pub.publish.call_count == 1


@pytest.mark.parametrize("changes", [dict(utc=""), dict(utc="120000.00"), dict(quality=0),
                                     dict(hdop=3.5), dict(satellites=4)])
def test_bad_or_duplicate_positions(changes):
    fusion = adapter()
    for i in (5, 10, 15, 20):
        fusion.raw(raw(i, **changes))
    fusion.pub.publish.assert_not_called()


def test_position_loss_requires_three_new_fixes_and_rejects_stale():
    fusion = adapter()
    for i in (5, 10, 15, 20, 25, 30):
        fusion.raw(raw(i, quality=0 if i == 15 else 1))
        assert fusion.pub.publish.call_count == int(i == 30)
    fusion.node.get_clock.return_value.now.return_value.nanoseconds += 2_000_000_000
    fusion.raw(raw(35))
    assert fusion.pub.publish.call_count == 1


def test_dateline_and_legacy_datum():
    assert Origin(0., 179.99999).project(.00001, -179.99999) == pytest.approx((2.22639, 1.10574), abs=.001)
    assert Origin.from_id("f1_enu_v1:23.00000000:114.00000000") == Origin(23., 114.)
