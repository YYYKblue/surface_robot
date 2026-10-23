"""Synthetic feedback exercise, not vessel dynamics or proof of stopping."""
import math
from f1_wall_cleaning.perception import Observation
from .config import Config
from .mission import Mission, Pose, Feedback


def sample(c, now, along=.8, distance=None, angle=0., front_observed=False):
    distance = c.wall_target_m if distance is None else distance
    x, y = c.map_xy(along, c.width_m - distance)
    wall = Observation(now, valid=True, distance=distance, angle=angle, residual=.005,
                       support=1., inliers=40, ratio=.95, front_observed=front_observed,
                       front_clearance=c.length_m - along - c.hull_half_length_m if front_observed else 0.,
                       front_fraction=1. if front_observed else 0., left_turn_clear=True, right_turn_clear=True)
    return Feedback(wall, Pose(now, x, y, c.axis_yaw_rad - angle))


def run_demo():
    c, now = Config(), 1_000_000_000
    mission = Mission(c)
    assert mission.start(sample(c, now), now)[0]
    history = [mission.phase]
    for i in range(80):
        now += 50_000_000
        # Script feedback independently of commands; no assumed propulsion model.
        angle = math.radians(12) if 5 <= i < 18 else 0.
        along = .8 + max(0, i - 20) * .025
        mission.update(sample(c, now, along=along, angle=angle), now)
        if mission.phase != history[-1]:
            history.append(mission.phase)
        if mission.phase == "STOPPED":
            return history
        if mission.phase == "PAUSED":
            raise RuntimeError(mission.reason)
    raise RuntimeError("script did not finish")


def main():
    print("SCRIPTED PREVIEW ONLY — no ROS output or dynamics model")
    print(" -> ".join(run_demo()))
    print("Single side; +/-10 degrees; 40 cm side target; 80 cm front stop; brush 15%")


if __name__ == "__main__":
    main()
