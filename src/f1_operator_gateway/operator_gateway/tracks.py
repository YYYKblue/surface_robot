"""Bounded, per-run map-frame pose history. Gaps are never joined."""
import math
from collections import deque


class TaskTrack:
    def __init__(self, limit=2000):
        self.limit = limit
        self.identity = None
        self.segments = deque()
        self.count = 0
        self.truncated = False
        self.last_seen = self.last_recorded = None
        self.epoch = None

    def update(self, map_id, run_id, pose, *, recording, epoch=None):
        identity = (map_id, run_id)
        if identity != self.identity:
            self.identity = identity
            self.segments.clear()
            self.count, self.truncated = 0, False
            self.last_seen = self.last_recorded = None
        if not recording or not map_id or not run_id or not pose:
            self.last_seen = self.last_recorded = None
            return
        stamp = pose.get("stamp", {})
        sample = (stamp.get("sec", 0) + stamp.get("nanosec", 0)/1e9,
                  pose["x"], pose["y"])
        if not all(math.isfinite(value) for value in sample) or sample[0] <= 0:
            self.last_seen = self.last_recorded = None
            return
        previous = self.last_seen
        broken = (previous is None or epoch != self.epoch or sample[0] < previous[0]
                  or sample[0]-previous[0] > .6
                  or math.hypot(sample[1]-previous[1], sample[2]-previous[2]) > .35)
        self.last_seen, self.epoch = sample, epoch
        if broken:
            self.segments.append(deque())
            self.last_recorded = None
        recorded = self.last_recorded
        if recorded and (sample[0]-recorded[0] < .2
                         or math.hypot(sample[1]-recorded[1], sample[2]-recorded[2]) < .03):
            return
        self.segments[-1].append([round(sample[1], 4), round(sample[2], 4)])
        self.last_recorded = sample
        self.count += 1
        while self.count > self.limit:
            self.segments[0].popleft()
            self.count -= 1
            self.truncated = True
            if not self.segments[0]:
                self.segments.popleft()

    def snapshot(self):
        return dict(frame_id="map", map_id=self.identity[0] if self.identity else "",
                    run_id=self.identity[1] if self.identity else "",
                    segments=[list(segment) for segment in self.segments],
                    point_count=self.count, truncated=self.truncated)


class SurveyTrack:
    """Timestamped, source-labelled map poses for one survey and its snapshots."""

    def __init__(self):
        self.samples = []
        self.segment = 0
        self.gap = True
        self.trajectory = None

    def update(self, pose, source, trajectory=None):
        if not pose or source not in ("lidar", "new_shore", "gnss_imu"):
            self.gap = True
            return
        if self.trajectory != trajectory:
            self.gap = True
            self.trajectory = trajectory
        stamp = pose["stamp"]["sec"] * 10**9 + pose["stamp"]["nanosec"]
        if self.samples:
            previous = self.samples[-1]
            dt = stamp - previous[0]
            if dt == 0 or (0 < dt < 500_000_000 and not self.gap and previous[4] == source):
                return
            if self.gap or not 0 < dt < 1_500_000_000:
                self.segment += 1
        self.samples.append((stamp, pose["x"], pose["y"], pose["yaw"], source, self.segment))
        self.gap = False

    def display(self):
        # Preserve both endpoints of a long crossing while bounding wire size.
        return survey_segments(self.samples)


def survey_segments(samples, limit=2000):
    stride = max(1, math.ceil(len(samples) / limit))
    points = samples[::stride]
    if points and points[-1] != samples[-1]:
        points.append(samples[-1])
    segments, previous = [], None
    for _, x, y, _, _, segment in points:
        if previous != segment:
            segments.append([])
        segments[-1].append([round(x, 4), round(y, 4)])
        previous = segment
    return segments
