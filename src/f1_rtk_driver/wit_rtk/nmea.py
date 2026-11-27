"""WTRTK-982 position decoding. No ROS or serial dependencies."""

from dataclasses import dataclass
from functools import reduce
import math
from operator import xor
import re


@dataclass(frozen=True)
class Fix:
    utc: str
    quality: int
    satellites: int
    hdop: float
    latitude: float = math.nan
    longitude: float = math.nan
    altitude: float = math.nan
    differential_age: float = math.nan
    station_id: str = ""

    @property
    def valid(self):
        return math.isfinite(self.latitude) and math.isfinite(self.longitude)


def number(field):
    value = float(field)
    if not math.isfinite(value):
        raise ValueError("non-finite value")
    return value


def coordinate(field, hemisphere, positive, negative, degree_digits, limit):
    if not re.fullmatch(rf"[0-9]{{{degree_digits + 2}}}(?:\.[0-9]+)?", field):
        raise ValueError("invalid degrees/minutes")
    minutes = float(field[degree_digits:])
    value = int(field[:degree_digits]) + minutes / 60.0
    if minutes >= 60 or value > limit or hemisphere not in (positive, negative):
        raise ValueError("invalid coordinate")
    return -value if hemisphere == negative else value


def parse_gga(fields):
    if len(fields) < 13:
        raise ValueError("incomplete GGA")
    quality = quality_code(fields[6])
    satellites = int(fields[7]) if fields[7] else -1
    if satellites < -1 or satellites > 255:
        raise ValueError("invalid satellite count")
    hdop = number(fields[8]) if fields[8] else math.nan
    if hdop < 0:
        raise ValueError("negative HDOP")
    if quality not in (1, 2, 3, 4, 5):
        return Fix(fields[1], quality, satellites, hdop)
    latitude = coordinate(fields[2], fields[3], "N", "S", 2, 90)
    longitude = coordinate(fields[4], fields[5], "E", "W", 3, 180)
    altitude = math.nan
    if fields[9] and fields[11] and fields[10] == fields[12] == "M":
        # NavSatFix expects ellipsoidal height; GGA reports MSL and geoid separation.
        altitude = number(fields[9]) + number(fields[11])
    age = math.nan
    if len(fields) > 13 and fields[13]:
        try:
            age = number(fields[13])
            if age < 0:
                age = math.nan
        except ValueError:
            pass
    station = fields[14] if len(fields) > 14 else ""
    return Fix(fields[1], quality, satellites, hdop, latitude, longitude, altitude, age, station)


def quality_code(field):
    if not re.fullmatch("[0-9]", field):
        raise ValueError("invalid quality code")
    return int(field)


def parse_sentence(line):
    """Return a GGA fix, None for other logs, or ValueError for malformed GGA."""
    if not re.match(rb"^\$[A-Z]{2}GGA,", line):
        return None
    payload, separator, checksum = line[1:].partition(b"*")
    if (not separator or not re.fullmatch(b"[0-9a-fA-F]{2}", checksum)
            or reduce(xor, payload, 0) != int(checksum, 16)):
        raise ValueError("NMEA checksum mismatch")
    fields = payload.decode("ascii").split(",")
    return parse_gga(fields)


def utc_nanoseconds(utc):
    """NMEA time of day, independent of the number of trailing decimal zeros."""
    if not re.fullmatch(r"(?:[01][0-9]|2[0-3])[0-5][0-9][0-5][0-9](?:\.[0-9]{1,9})?", utc):
        return None
    seconds = int(utc[:2]) * 3600 + int(utc[2:4]) * 60 + int(utc[4:6])
    return seconds * 10**9 + int(utc[7:].ljust(9, "0"))
