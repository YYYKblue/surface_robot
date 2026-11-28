"""Cross a 36 m gap between two disjoint shores with real Cartographer entry points."""
import json
import math
import subprocess
import time
from functools import reduce
from operator import xor

import rclpy
from cartographer_ros_msgs.srv import TrajectoryQuery, WriteState
from f1_interfaces.msg import RawSentence, SurveyStatus, Wt102Sample
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

from f1_mapping_bringup.gnss_positions import PlanarPose
from f1_localization.gnss_reference import read_reference
from test_gnss_slam import launch, room_scan
from test_open_water import cell


def test_lake_crossing(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "169")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))
    rclpy.init()
    node = rclpy.create_node("lake_crossing")
    clock = node.create_publisher(Clock, "/clock", 10)
    raw = node.create_publisher(RawSentence, "/f1/rtk/raw", 10)
    rates = node.create_publisher(Wt102Sample, "/f1/wt102/sample", qos_profile_sensor_data)
    scans = node.create_publisher(LaserScan, "/pavo2_scan", qos_profile_sensor_data)
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    query = node.create_client(TrajectoryQuery, "/trajectory_query")
    writer = node.create_client(WriteState, "/write_state")
    buf = Buffer()
    listener = TransformListener(buf, node)
    stats, grids, gate = [], [], []
    node.create_subscription(SurveyStatus, "/f1/survey/status", stats.append, 5)
    node.create_subscription(OccupancyGrid, "/map", grids.append, 1)
    node.create_subscription(String, "/pool_cleaning/localization_status", lambda m: gate.append(json.loads(m.data)), 5)
    ms = 100000
    x = y = yaw = 0.
    gga = True
    stream = 1
    args = ["use_sim_time:=true", "use_wt102_heading:=true"]
    path = tmp_path / "lake.pbstream"

    def stamp(t):
        return rclpy.time.Time(nanoseconds=t * 1_000_000).to_msg()

    def step(seconds, shore=None, vx=0., vy=0., turn=0.):
        nonlocal ms, x, y, yaw
        for _ in range(round(seconds*100)):
            ms += 10
            x, y, yaw = x + .01*vx, y + .01*vy, yaw + .01*turn
            clock.publish(Clock(clock=stamp(ms)))
            rates.publish(Wt102Sample(header=Header(stamp=stamp(ms), frame_id="wt102_link"),
                                      stream_id=stream, kind=Wt102Sample.YAW_RATE, value=float(turn)))
            if ms % 20 == 0:
                scan = room_scan(x - (shore or 0.), yaw) if shore is not None else (
                    LaserScan(angle_min=-math.pi, angle_max=math.pi-math.tau/360,
                              angle_increment=math.tau/360, range_min=.1, range_max=15.,
                              ranges=[math.inf]*360))
                scan.time_increment = 0.
                scan.header = Header(stamp=stamp(ms-10), frame_id="pavo2s_frame")
                scans.publish(scan)
                if gga and ms % 100 == 0:
                    ant = PlanarPose(x, y, yaw).compose(PlanarPose(-.191, .213, 0.))
                    utc = f"12{ms//60000%60:02}{ms//1000%60:02}.{ms%1000:03}"
                    body = (f"GNGGA,{utc},23{15+60*ant.y/110750:09.6f},N,"
                            f"114{15+60*ant.x/102523:09.6f},E,1,12,1.1,0,M,0,M,,")
                    raw.publish(RawSentence(header=Header(stamp=stamp(ms), frame_id="gnss_ant1"),
                                            sentence=f"${body}*{reduce(xor,body.encode()):02X}"))
            end = time.monotonic() + .007
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.001)

    def until(check, shore=None, timeout=15.):
        end = time.monotonic() + timeout
        while not check():
            assert time.monotonic() < end, (stats[-1:], gate[-1:])
            step(.1, shore)

    def pose():
        tr = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        return PlanarPose(tr.translation.x, tr.translation.y, 2*math.atan2(tr.rotation.z, tr.rotation.w))

    def call(client, req, shore=None):
        future = client.call_async(req)
        until(future.done, shore)
        result = future.result()
        assert result.status.code == 0, result.status.message
        return result

    try:
        with launch(tmp_path, "f1_mapping_bringup", "lidar_mapping.launch.py", *args):
            until(lambda: stats and stats[-1].source == "lidar" and writer.service_is_ready(), 0.)
            until(lambda: grids and min((c for c in grids[-1].data if c >= 0), default=100) < 17,
                  0., timeout=45.)
            first = pose()
            step(8., 0., vx=.5)
            until(lambda: stats[-1].registered and stats[-1].heading_valid and grids, 0.)
            # Suppress every range, move east while turning the bow toward north.
            step(10., vx=2., turn=.12)
            step(.8)
            assert stats[-1].source == "gnss_imu"
            mid = pose()
            assert abs(mid.x-first.x-x) < .8 and abs(mid.y-first.y) < .6, (mid, first)
            assert abs(mid.yaw-first.yaw-yaw) < .10
            track = call(query, TrajectoryQuery.Request(trajectory_id=0)).trajectory
            assert max(p.pose.position.x for p in track) < 6
            midlake = tmp_path / "midlake.pbstream"
            call(writer, WriteState.Request(filename=str(midlake), include_unfinished_submaps=True))
            assert read_reference(midlake, 0)[0] is not None
            assert grids[-1].info.origin.position.x + grids[-1].info.width*grids[-1].info.resolution < 10.
            gga = False
            step(1.5)
            assert stats[-1].source == "unavailable" and not stats[-1].position_fresh
            gga = True
            until(lambda: stats[-1].source == "gnss_imu")
            step(6., vx=2.)
            step(1.)
            assert abs(x-36.) < .01 and abs(pose().x-first.x-x) < .8
            # Same trajectory, new shoreline 36 m from the start.
            step(2., 36.)
            until(lambda: grids and grids[-1].info.width*grids[-1].info.resolution > 40., 36.)
            track = call(query, TrajectoryQuery.Request(trajectory_id=0), 36.).trajectory
            assert abs(track[-1].pose.position.x - x) < .9
            grid = grids[-1]
            assert cell(grid, 18., 0.) == -1
            assert any(cell(grid, 40.4+i*.05, 0.) > 50 for i in range(24)
                       if 40.4+i*.05 < grid.info.origin.position.x + grid.info.width*grid.info.resolution)
            call(writer, WriteState.Request(filename=str(path), include_unfinished_submaps=True), 36.)
        origin, reference, _ = read_reference(path, 0)
        subprocess.run(["ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
                        "-pbstream_filename", str(path), "-map_filestem", str(path.with_suffix("")),
                        "-resolution", ".05"], check=True, timeout=20)
        stats.clear()
        buf.clear()
        x, y, yaw = 1., 0., .4
        with launch(tmp_path, "f1_localization", "localization.launch.py", f"map:={path}", *args):
            until(lambda: gate and stats and seed_pub.get_subscription_count() and writer.service_is_ready())
            expected = reference.compose(PlanarPose(*origin.project(23.25, 114.25+x/102523.), yaw))
            seed = PoseWithCovarianceStamped(header=Header(stamp=stamp(ms), frame_id="map"))
            seed.pose.pose = expected.message()
            seed_pub.publish(seed)
            until(lambda: stats[-1].source == "gnss_imu")
            step(5., vx=1., turn=.1)
            step(1.)
            current = pose()
            assert abs(current.x-expected.x-5.) < .5 and abs(current.yaw-expected.yaw-.5) < .08
            assert not gate[-1]["ready"]
            step(5., vx=-1., turn=-.1)
            step(1.)
            until(lambda: gate[-1]["ready"], 0., 25.)
            assert abs(pose().x-expected.x) < .45
            step(1.)
            stream = 2
            step(.5)
            assert stats[-1].source == "unavailable" and not stats[-1].heading_valid
            until(lambda: gate[-1]["ready"] and stats[-1].heading_valid, 0., 15.)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
