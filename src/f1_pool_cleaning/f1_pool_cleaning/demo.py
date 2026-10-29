"""Synthetic feedback exercise, not vessel dynamics or proof of stopping."""
import math
from f1_pool_cleaning.perception import Observation
from .config import Config
from .mission import Mission, Pose, Feedback

# Scripted-exercise geometry only.  The runtime nodes derive their geometry
# from ``map:=`` through :mod:`f1_pool_cleaning.map_profile`; this synthetic
# rectangle exists so the offline demo and the unit tests stay map-free.  It is
# deliberately not any deployed map's numbers.
SYNTHETIC_GEOMETRY = dict(origin_x_m=-1.0, origin_y_m=-0.7, axis_yaw_rad=0.0,
                          length_m=3.0, width_m=1.6)


def synthetic_config(**overrides):
    """``Config`` with the synthetic demo rectangle, plus explicit overrides."""
    return Config(**{**SYNTHETIC_GEOMETRY, **overrides})


def sample(c, now, along=.8, distance=None, angle=0., front_observed=False):
    distance = c.wall_target_m if distance is None else distance
    x, y = c.map_xy(along, c.width_m - distance)
    wall = Observation(now, valid=True, distance=distance, angle=angle, residual=.005,
                       support=1., inliers=40, ratio=.95, front_observed=front_observed,
                       front_clearance=c.length_m - along - c.hull_half_length_m if front_observed else 0.,
                       front_fraction=1. if front_observed else 0., left_turn_clear=True, right_turn_clear=True)
    return Feedback(wall, Pose(now, x, y, c.axis_yaw_rad - angle))


def run_demo(config=None):
    c, now = (config or synthetic_config()), 1_000_000_000
    mission = Mission(c)
    assert mission.start(sample(c, now), now)[0]
    history = [mission.phase]
    for i in range(110):
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
    print("Stage-2 left brush; +/-10 degrees; 34 cm target (32--36 cm band); 50 cm legacy front stop; brush/side 30%")


if __name__ == "__main__":
    main()
