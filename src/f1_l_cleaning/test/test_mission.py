from dataclasses import replace
import math

from f1_pool_cleaning.demo import synthetic_config
from f1_pool_cleaning.geometry import PoolModel
from f1_pool_cleaning.mission import Feedback, Mission, Pose, command_fields
from f1_pool_cleaning.perception import Observation

from f1_l_cleaning.route import build_l_route


ROUTE = [-1.801431, 0.681948, -10.462431, -0.786552, -11.917931, 10.962448]
NOW = 1_000_000_000


def feedback(config, edge, stamp, along, yaw=None):
    tx, ty = edge.tangent
    nx, ny = edge.left_normal
    pose = Pose(stamp,
                edge.start[0] + along * tx - config.wall_target_m * nx,
                edge.start[1] + along * ty - config.wall_target_m * ny,
                edge.yaw_rad if yaw is None else yaw)
    wall = Observation(stamp, valid=True, distance=config.wall_target_m,
                       angle=0., support=1., inliers=40, ratio=.95)
    return Feedback(wall, pose)


def test_l_transfer_is_reverse_right_push_stop_right_turn_then_reacquire():
    c = synthetic_config().validate()
    route = build_l_route(ROUTE, c)
    mission = Mission(c)
    assert mission.configure_edge(route, route.edges[0])[0]
    now = NOW
    assert mission.start(feedback(c, route.edges[0], now, 1.), now)[0]

    now += 100_000_000
    end_s = route.edges[0].length_m - c.front_stop_m + .01
    effort = mission.update(feedback(c, route.edges[0], now, end_s), now)
    assert mission.phase == "EDGE_STOP"
    assert mission.completed_edges == ["edge_0"]
    assert effort.direction == 0 and not effort.brush_percent and not effort.side_left_percent

    now += 100_000_000
    effort = mission.update(feedback(c, route.edges[0], now, end_s), now)
    assert mission.phase == "MOVE_INWARD" and effort.direction == 0

    now += 100_000_000
    effort = mission.update(feedback(c, route.edges[0], now, end_s), now)
    fields = command_fields(effort, c)
    assert mission.phase == "MOVE_INWARD"
    assert effort.direction == 2 and effort.side_left_percent == -c.side_percent
    assert fields["thrust_enabled"] and fields["thrust_percent"] == -c.side_percent
    assert not fields["brush_enabled"]

    now += round((c.transfer_move_duration_sec + .01) * 1e9)
    pose_feedback = feedback(c, route.edges[0], now, end_s)
    effort = mission.update(pose_feedback, now)
    assert mission.phase == "TURN_TO_NEXT_EDGE" and effort.direction == 0
    now += 100_000_000
    turn_error = math.radians(-25.)
    pose_feedback = replace(pose_feedback, pose=replace(
        pose_feedback.pose, stamp_ns=now, yaw=route.edges[1].yaw_rad - turn_error))
    effort = mission.update(pose_feedback, now)
    assert mission.phase == "TURN_TO_NEXT_EDGE" and effort.direction == 4
    assert not effort.brush_percent and not effort.side_left_percent

    now += 100_000_000
    pose_feedback = replace(pose_feedback, pose=replace(
        pose_feedback.pose, stamp_ns=now,
        yaw=route.edges[1].yaw_rad + math.radians(15)))
    effort = mission.update(pose_feedback, now)
    assert mission.phase == "APPROACH_NEXT_EDGE"
    assert mission.edge is route.edges[1] and effort.direction == 0
    assert mission.reason == "turned to edge_1; re-acquiring target wall"

    now += 100_000_000
    effort = mission.update(feedback(c, route.edges[1], now, .8), now)
    assert mission.phase == "FOLLOW"
    assert effort.brush_percent == c.brush_percent


def test_l_mission_completes_after_second_edge_without_closing_the_route():
    c = synthetic_config().validate()
    route = build_l_route(ROUTE, c)
    mission = Mission(c)
    assert mission.configure_edge(route, route.edges[0])[0]
    now = NOW
    mission.start(feedback(c, route.edges[0], now, 1.), now)

    # Bring the open route to the second edge, modelling one completed transfer.
    edge0 = route.edges[0]
    s = edge0.length_m - c.front_stop_m + .01
    now += 100_000_000
    mission.update(feedback(c, edge0, now, s), now)
    for delta in (.10, .55):
        now += round(delta * 1e9)
        mission.update(feedback(c, edge0, now, s), now)
    now += 100_000_000
    turn = feedback(c, edge0, now, s)
    turn = replace(turn, pose=replace(turn.pose, yaw=route.edges[1].yaw_rad))
    mission.update(turn, now)
    now += 100_000_000
    mission.update(feedback(c, route.edges[1], now, .8), now)
    assert mission.phase == "FOLLOW" and mission.edge.id == "edge_1"

    now += 100_000_000
    end_s = route.edges[1].length_m - c.front_stop_m + .01
    effort = mission.update(feedback(c, route.edges[1], now, end_s), now)
    assert mission.phase == "COMPLETE"
    assert mission.completed_edges == ["edge_0", "edge_1"]
    assert mission.transfer_next_edge is None
    assert effort.direction == 0 and effort.side_left_percent == 0 and effort.brush_percent == 0
    assert PoolModel.next_edge(route, route.edges[1]) is None

