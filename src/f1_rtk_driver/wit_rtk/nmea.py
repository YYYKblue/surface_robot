"""WTRTK-982 GGA/HPR decoding. No ROS or serial dependencies."""

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

    @property
    def valid(self):
        return math.isfinite(self.latitude) and math.isfinite(self.longitude)


@dataclass(frozen=True)
class Heading:
    utc: str
    quality: int
    degrees: float = math.nan

    @property
    def valid(self):
        return math.isfinite(self.degrees)


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
    return Fix(fields[1], quality, satellites, hdop, latitude, longitude, altitude)


def quality_code(field):
    if not re.fullmatch("[0-9]", field):
        raise ValueError("invalid quality code")
    return int(field)


def parse_sentence(line):
    """Return Fix/Heading, None for unrelated input, or ValueError for a bad target line."""
    if not re.match(rb"^\$[A-Z]{2}(?:GGA|HPR),", line):
        return None
    payload, separator, checksum = line[1:].partition(b"*")
    if (not separator or not re.fullmatch(b"[0-9a-fA-F]{2}", checksum)
            or reduce(xor, payload, 0) != int(checksum, 16)):
        raise ValueError("NMEA checksum mismatch")
    fields = payload.decode("ascii").split(",")
    if fields[0].endswith("GGA"):
        return parse_gga(fields)
    if len(fields) < 6:
        raise ValueError("incomplete HPR")
    # UM982 emits a complete HPR with empty measurement fields before acquiring a solution.
    if len(fields) == 9 and not any(fields[2:]):
        return Heading(fields[1], -1)
    quality = quality_code(fields[5])
    if quality != 4:
        return Heading(fields[1], quality)
    degrees = number(fields[2])
    if not 0 <= degrees <= 360:
        raise ValueError("heading out of range")
    return Heading(fields[1], quality, degrees % 360)
