from functools import reduce
import math
from operator import xor

import pytest

from wit_rtk.nmea import parse_sentence


def sentence(payload):
    body = payload.encode("ascii")
    return b"$" + body + b"*" + f"{reduce(xor, body, 0):02X}".encode()


def test_vendor_example_position_and_heading_have_independent_quality():
    fix = parse_sentence(b"$GNGGA,023541.00,2304.76261531,N,11415.24485118,E,"
                         b"1,18,1.5,44.5917,M,-3.1322,M,,*59")
    heading = parse_sentence(b"$GNHPR,023541.00,101.6490,000.2235,000.0000,4,19,0.00,0999*54")
    assert fix.quality == 1 and heading.quality == 4
    assert fix.latitude == pytest.approx(23 + 4.76261531 / 60)
    assert fix.longitude == pytest.approx(114 + 15.24485118 / 60)
    assert fix.altitude == pytest.approx(41.4595)
    assert (fix.satellites, fix.hdop, heading.degrees) == (18, 1.5, 101.649)
    assert fix.utc == heading.utc == "023541.00"


@pytest.mark.parametrize("quality", [1, 2, 3, 4, 5])
def test_signed_coordinates_and_unknown_height(quality):
    fix = parse_sentence(sentence(f"GPGGA,235959.00,1230.00,S,17930.00,W,{quality},10,1,5,M,,M,,"))
    assert fix.valid and fix.latitude == -12.5 and fix.longitude == -179.5
    assert math.isnan(fix.altitude) and fix.quality == quality


def test_receiver_invalid_and_north_are_distinct():
    fix = parse_sentence(sentence("GNGGA,,,,,,0,00,,,,,,,"))
    assert not fix.valid and fix.quality == 0
    for quality in (0, 1, 5):
        assert not parse_sentence(sentence(f"GPHPR,000000.00,0,0,0,{quality},0,,")).valid
    north = parse_sentence(sentence("GPHPR,000000.00,360.0000,0,0,4,16,,"))
    assert north.valid and north.degrees == 0.0


def test_indoor_receiver_empty_hpr_is_an_invalid_observation():
    heading = parse_sentence(sentence("GNHPR,,,,,,,,"))
    assert heading.quality == -1 and not heading.valid and heading.utc == ""


@pytest.mark.parametrize("line", [
    b"$GNHPR,023541.00,101.6490,000.2235,000.0000,4,19,0.00,0999*55",
    b"$GPGGA,1", sentence("GPGGA,000000.00,,,,,1"),
    sentence("GNGGA,000000.00,9061.00,N,11200.00,E,1,10,1,1,M,1,M,,"),
    sentence("GNGGA,000000.00,2259.00,X,11200.00,E,1,10,1,1,M,1,M,,"),
    sentence("GNGGA,000000.00,2259.00,N,18100.00,E,1,10,1,1,M,1,M,,"),
    sentence("GNHPR,000000.00,nan,0,0,4,20,0,"),
    sentence("GNHPR,000000.00,-1.5,0,0,4,20,0,"),
    sentence("GNHPR,000000.00,1,0,0,-1,20,0,"),
    sentence("GNHPR,000000.00,1,0,0,,20,0,"),
])
def test_bad_target_lines_raise(line):
    with pytest.raises(ValueError):
        parse_sentence(line)


def test_unrelated_sentences_are_ignored():
    assert parse_sentence(sentence("GNRMC,000000.00,V")) is None
    assert parse_sentence(sentence("GNHPR2,000000.00,25,0,0,4")) is None


def test_differential_age_is_distinct_from_observation_age():
    body = "GPGGA,235959.00,1230.00,S,17930.00,W,4,10,1,5,M,2,M,1.2,0042"
    fix = parse_sentence(sentence(body))
    assert fix.valid and fix.quality == 4
    assert fix.differential_age == 1.2 and fix.station_id == "0042"
    for bad in ("", "-1", "nan", "invalid"):
        fix = parse_sentence(sentence(body.replace(",1.2,0042", f",{bad},0042")))
        assert fix.valid and math.isnan(fix.differential_age)
