"""Scripted, synthetic feedback; not a dynamics simulator or physical proof."""
import math
from f1_wall_cleaning.perception import Observation
from .config import Config
from .mission import Mission, Pose, Motion, Feedback


def sample(c, now, side="A", along=None, distance=None, speed=0.):
    along = c.end_inset_m if along is None else along
    distance = c.wall_target_m if distance is None else distance
    y = c.width_m - distance if side == "A" else distance
    x, y = c.map_xy(along, y)
    front = (c.length_m - along if side == "A" else along) - c.hull_half_length_m
    wall = Observation(now, valid=True, distance=distance, angle=0., residual=.005,
                       support=1., inliers=40, ratio=.95, front_observed=True,
                       front_clearance=front, front_fraction=1.)
    return Feedback(wall, Pose(now, x, y, c.axis_yaw_rad + (0 if side == "A" else math.pi)),
                    Motion(now, speed, 0., 0.))


def run_demo():
    c, now = Config(), 1_000_000_000
    mission = Mission(c)
    assert mission.start(sample(c, now), now)[0]
    history = [mission.phase]
    along = c.end_inset_m
    for _ in range(1000):
        now += 50_000_000
        if mission.phase == "WAIT_MANUAL_TRANSFER":
            along = c.length_m - c.end_inset_m
            ok, reason = mission.confirm_second_wall(sample(c, now, "B", along), now)
            assert ok, reason
        elif mission.phase.startswith("FOLLOW_"):
            # Script position progress independently of propulsion. This is a
            # state-transition example, explicitly not predicted boat motion.
            along += .02 if mission.side == "A" else -.02
        mission.update(sample(c, now, mission.side, along), now)
        if mission.phase != history[-1]:
            history.append(mission.phase)
        if mission.phase == "FINISHED_SEGMENTS":
            return history
        if mission.phase == "PAUSED":
            raise RuntimeError(mission.reason)
    raise RuntimeError("script did not finish")


def main():
    print("SCRIPTED PREVIEW ONLY — no ROS publishers, hardware, or dynamics model")
    print(" -> ".join(run_demo()))
    print("A/B planned segments only; manual transfer; no brush/contact or full-pool coverage claim")


if __name__ == "__main__":
    main()
