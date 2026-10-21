"""Summarize raw LaserScan validity; never certifies physical accuracy."""
import math
import statistics


class ScanQuality:
    def __init__(self, expected_frame="pavo2s_frame"):
        self.expected_frame = expected_frame
        self.scans = self.points = self.valid = self.nan = self.infinite = self.outside = 0
        self.frames_without_returns = self.metadata_errors = 0
        self.backward_stamps = self.repeated_stamps = self.frame_errors = 0
        self.stale_stamps = self.future_stamps = 0
        self.last_stamp = self.first_arrival = self.last_arrival = None
        self.valid_fractions = []
        self.closest = self.farthest = None
        self.directions = {"right_minus_90_deg": [], "forward_0_deg": [], "left_plus_90_deg": []}
        self.frames = set()
        self.last_geometry = {}

    def add(self, message, arrival_mono, now_unix_ns):
        self.scans += 1
        if self.first_arrival is None:
            self.first_arrival = arrival_mono
        self.last_arrival = arrival_mono
        self.frames.add(message.header.frame_id)
        if message.header.frame_id != self.expected_frame:
            self.frame_errors += 1
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if self.last_stamp is not None:
            self.backward_stamps += int(stamp < self.last_stamp)
            self.repeated_stamps += int(stamp == self.last_stamp)
        self.last_stamp = stamp
        age = (now_unix_ns - stamp) / 1e9
        self.stale_stamps += int(age > 1.0)
        self.future_stamps += int(age < -0.1)
        count = len(message.ranges)
        self.points += count
        metadata = (message.angle_min, message.angle_max, message.angle_increment,
                    message.range_min, message.range_max, message.time_increment, message.scan_time)
        good_geometry = (count > 0 and all(math.isfinite(x) for x in metadata)
                         and message.angle_increment != 0 and 0 <= message.range_min < message.range_max
                         and message.time_increment >= 0 and message.scan_time >= 0)
        if good_geometry:
            predicted_end = message.angle_min + (count - 1) * message.angle_increment
            good_geometry = abs(predicted_end - message.angle_max) <= max(1e-4, abs(message.angle_increment) * 1.5)
        if not good_geometry:
            self.metadata_errors += 1
        good_ranges = []
        valid = 0
        for distance in message.ranges:
            if math.isnan(distance):
                self.nan += 1
            elif not math.isfinite(distance):
                self.infinite += 1
            elif not message.range_min <= distance <= message.range_max:
                self.outside += 1
            else:
                valid += 1
                good_ranges.append(distance)
        self.valid += valid
        self.valid_fractions.append(valid / count if count else 0.0)
        if not valid:
            self.frames_without_returns += 1
        else:
            nearest, farthest = min(good_ranges), max(good_ranges)
            self.closest = nearest if self.closest is None else min(self.closest, nearest)
            self.farthest = farthest if self.farthest is None else max(self.farthest, farthest)
        if good_geometry:
            for name, angle in zip(self.directions, (-math.pi / 2, 0, math.pi / 2)):
                index = (angle - message.angle_min) / message.angle_increment
                if 0 <= index <= count - 1:
                    radius = max(0, int(math.radians(1) / abs(message.angle_increment)))
                    centre = round(index)
                    sector = message.ranges[max(0, centre - radius):min(count, centre + radius + 1)]
                    values = [x for x in sector if math.isfinite(x) and message.range_min <= x <= message.range_max]
                    if values:
                        self.directions[name].append(statistics.median(values))
            self.last_geometry = {"points_per_scan": count,
                                  "angular_span_deg": math.degrees(abs(message.angle_max - message.angle_min)),
                                  "angular_step_deg": math.degrees(message.angle_increment),
                                  "scan_time_sec": message.scan_time,
                                  "ray_span_sec": (count - 1) * message.time_increment,
                                  "declared_range_m": [message.range_min, message.range_max]}

    def report(self):
        duration = 0 if self.first_arrival is None else self.last_arrival - self.first_arrival
        issues = []
        for bad, description in (
            (self.scans == 0, "no scans received"),
            (self.scans > 0 and self.valid == 0, "no finite in-range returns"),
            (self.metadata_errors, "invalid scan geometry metadata"),
            (self.frame_errors, "unexpected sensor frame"),
            (self.backward_stamps, "scan timestamps moved backwards"),
            (self.repeated_stamps, "scan timestamps repeated"),
            (self.stale_stamps, "scan timestamp age exceeds 1 second"),
            (self.future_stamps, "scan timestamps are over 100 ms in the future"),
        ):
            if bad:
                issues.append(description)
        return {
            "scans": self.scans, "frames": sorted(self.frames),
            "arrival_hz": (self.scans - 1) / duration if duration > 0 else None,
            "points": self.points, "valid_points": self.valid,
            "valid_fraction": self.valid / self.points if self.points else 0.0,
            "minimum_scan_valid_fraction": min(self.valid_fractions) if self.valid_fractions else None,
            "nan_points": self.nan, "infinite_points": self.infinite,
            "out_of_range_points": self.outside, "scans_without_valid_returns": self.frames_without_returns,
            "nearest_valid_m": self.closest, "farthest_valid_m": self.farthest,
            "sector_medians_m": {key: statistics.median(values) if values else None
                                 for key, values in self.directions.items()},
            "backward_stamps": self.backward_stamps, "repeated_stamps": self.repeated_stamps,
            "metadata_errors": self.metadata_errors, "geometry": self.last_geometry,
            "issues": issues,
            "interpretation": "Message checks only; compare distances/directions with real objects before claiming accuracy.",
        }
