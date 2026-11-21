import math

import pytest

from wit_rtk.alignment import EpochPairs, base_coordinates, bow_heading, utc_nanoseconds


@pytest.mark.parametrize("raw,offset,expected", [(42, 42, 0), (0, 42, 318), (350, -20, 10)])
def test_clockwise_bow_heading(raw, offset, expected):
    assert bow_heading(raw, offset) == expected
    assert math.isnan(bow_heading(math.nan, offset))


# Independent WGS84 direct-geodesic references from PROJ geod.
@pytest.mark.parametrize("heading,lat,lon", [
    (0, 0.000001727346, 0.000001913412),
    (90, -0.000001926307, 0.000001715782),
    (180, -0.000001727346, -0.000001913412),
    (270, 0.000001926307, -0.000001715782),
])
def test_move_from_stern_port_antenna_to_base(heading, lat, lon):
    assert base_coordinates(0, 0, heading, -0.191, 0.213) == pytest.approx(
        (lat, lon), abs=1e-11, rel=0)


@pytest.mark.parametrize("lat,lon,expected", [
    (0, 179.999999, (0, -179.999992016847)),
    (89.999999, 179.999999, (89.999990991292, -96.373179251032)),
])
def test_one_metre_east_near_dateline_and_pole(lat, lon, expected):
    assert base_coordinates(lat, lon, 0, 0, 1) == pytest.approx(expected, abs=2e-8, rel=0)


def test_pairing_order_midnight_missing_and_older_epochs():
    pairs = EpochPairs(1.5)
    before = utc_nanoseconds("235959.95")
    after = utc_nanoseconds("000000")
    assert after == utc_nanoseconds("000000.000") == 0
    assert before == utc_nanoseconds("235959.950")
    assert all(utc_nanoseconds(s) is None for s in ("", "230000.", "123499", "250000", "0"))
    assert pairs.add(before, 1.0, "h1", False) is None
    assert pairs.add(after, 1.1, "h2", False) is None
    assert pairs.add(before, 1.2, "f1", True) == ("f1", "h1", 1.0)
    assert pairs.add(after, 1.3, "f2", True) == ("f2", "h2", 1.1)
    assert pairs.add(before, 1.4, "late", True) is None
    assert pairs.add(after, 1.4, "duplicate", True) is None
    assert pairs.add(after, 3.0, "frozen UTC", True) is None
    assert pairs.add(after, 3.0, "frozen UTC", False) is None
    assert not pairs.fixes


def test_stale_partner_invalid_solution_and_disconnect():
    pairs = EpochPairs(1.0)
    assert pairs.add(1, 0.0, "old", True) is None
    assert pairs.add(1, 1.0, "heading", False) is None
    pairs.invalidate(2, 1.1)
    assert pairs.add(1, 1.2, "fix", True) is None
    assert pairs.add(1, 1.2, "heading", False) is None
    assert pairs.add(3, 1.3, "prior session", True) is None
    pairs.clear()
    assert pairs.add(3, 1.4, "heading", False) is None
    assert pairs.add(3, 1.5, "new fix", True) == ("new fix", "heading", 1.4)
