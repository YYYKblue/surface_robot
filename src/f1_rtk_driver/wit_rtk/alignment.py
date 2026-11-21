"""Planar vessel mounting geometry and same-epoch GGA/HPR pairing."""

import math
import re


def bow_heading(baseline_deg, mounting_deg):
    return (baseline_deg - mounting_deg) % 360.0


def base_coordinates(latitude, longitude, heading_deg, antenna_x, antenna_y):
    """Move from the main antenna to the vessel origin on the WGS84 ellipsoid."""
    heading = math.radians(heading_deg)
    east = -antenna_x * math.sin(heading) + antenna_y * math.cos(heading)
    north = -antenna_x * math.cos(heading) - antenna_y * math.sin(heading)

    # Geographic -> geocentric -> geographic (EPSG method 9602). Work in
    # the antenna meridian so the east component is the Cartesian Y offset.
    a = 6378137.0
    flattening = 1 / 298.257223563
    b = a * (1 - flattening)
    e2 = flattening * (2 - flattening)
    phi = math.radians(latitude)
    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    radius = a / math.sqrt(1 - e2 * sin_phi**2)
    x = radius * cos_phi - north * sin_phi
    z = radius * (1 - e2) * sin_phi + north * cos_phi
    horizontal = math.hypot(x, east)
    theta = math.atan2(z * a, horizontal * b)
    base_lat = math.atan2(z + b * e2 / (1 - e2) * math.sin(theta)**3,
                          horizontal - a * e2 * math.cos(theta)**3)
    base_lon = longitude + math.degrees(math.atan2(east, x))
    return math.degrees(base_lat), (base_lon + 180) % 360 - 180


def utc_nanoseconds(utc):
    """NMEA time of day, independent of the number of trailing decimal zeros."""
    if not re.fullmatch(r"(?:[01][0-9]|2[0-3])[0-5][0-9][0-5][0-9](?:\.[0-9]{1,9})?", utc):
        return None
    seconds = int(utc[:2]) * 3600 + int(utc[2:4]) * 60 + int(utc[4:6])
    return seconds * 10**9 + int(utc[7:].ljust(9, "0"))


class EpochPairs:
    """Bound pending observations, allow either serial order, and emit each epoch once."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.clear()

    def clear(self):
        self.fixes = {}
        self.headings = {}
        self.last_epoch = None

    def is_new_epoch(self, epoch, now):
        # With time-of-day only, wrap forward across midnight. After half a
        # day without a result the date is ambiguous; accept a fresh sequence.
        if self.last_epoch is None:
            return True
        previous_epoch, previous_time = self.last_epoch
        delta = (epoch - previous_epoch) % (86400 * 10**9)
        return now - previous_time >= 43200 or 0 < delta < 43200 * 10**9

    def invalidate(self, epoch, now):
        self.fixes.clear()
        self.headings.clear()
        if epoch is not None and self.is_new_epoch(epoch, now):
            self.last_epoch = (epoch, now)

    def add(self, epoch, now, message, is_fix):
        if not self.is_new_epoch(epoch, now):
            return None
        own, other = (self.fixes, self.headings) if is_fix else (self.headings, self.fixes)
        match = other.pop(epoch, None)
        if match is None or now - match[0] >= self.timeout:
            own[epoch] = (now, message)
            # 128 slots exceed the normal in-flight data at 20 Hz.
            if len(own) > 128:
                del own[next(iter(own))]
            return None
        self.last_epoch = (epoch, now)
        fix, heading = (message, match[1]) if is_fix else (match[1], message)
        return fix, heading, match[0]
