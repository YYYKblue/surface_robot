"""Exercise the installed mapping entry point with identical timed inputs."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from cartographer_ros_msgs.srv import TrajectoryQuery, WriteState
from f1_interfaces.msg import Wt102Sample
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


def stamp(ms):
    return Time(sec=ms // 1000, nanosec=(ms % 1000) * 1_000_000)


def test_mapping_nodes_follow_rate_and_survive_gap(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '147')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    config = Path(get_package_share_directory('f1_mapping_bringup')) / 'config'
    for name in ('rtk_mapping.lua', 'pavo2_2d.lua'):
        (tmp_path / name).write_bytes((config / name).read_bytes())
    # A circular scan has no preferred heading. Isolate the prediction from
    # arbitrary grid discretization choices made by the correlative matcher.
    (tmp_path / 'fixture.lua').write_text(
        'include "rtk_mapping.lua"\n'
        'TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false\n'
        'TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1000.\n'
        'TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2\n'
        'options.rangefinder_sampling_ratio = 1.\nreturn options\n')
    for enabled in (False, True):
        saved = tmp_path / f'survey-{enabled}.pbstream'
        with (tmp_path / f'launch-{enabled}.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'f1_mapping_bringup', 'lidar_mapping.launch.py',
                'use_sim_time:=true', f'use_wt102_heading:={str(enabled).lower()}',
                f'configuration_directory:={tmp_path}', 'configuration_basename:=fixture.lua'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('wt102_slam_test')
            try:
                scans = node.create_publisher(LaserScan, '/pavo2_scan', qos_profile_sensor_data)
                rates = node.create_publisher(Wt102Sample, '/f1/wt102/sample', qos_profile_sensor_data)
                clock = node.create_publisher(Clock, '/clock', 10)
                query = node.create_client(TrajectoryQuery, '/trajectory_query')
                write = node.create_client(WriteState, '/write_state')
                deadline = time.monotonic() + 12
                while not (query.service_is_ready() and scans.get_subscription_count()
                           and (not enabled or rates.get_subscription_count())):
                    assert process.poll() is None and time.monotonic() < deadline
                    rclpy.spin_once(node, timeout_sec=.025)
                scan = LaserScan(angle_min=-math.pi, angle_max=math.pi-math.tau/360,
                                 angle_increment=math.tau/360, range_min=.1, range_max=12.,
                                 scan_time=.05, ranges=[3.] * 360)
                for ms in range(10000, 15500, 10):
                    clock.publish(Clock(clock=stamp(ms)))
                    if ms < 12500 or ms >= 13500:
                        stream = 1 if ms < 12500 else 2
                        rate = .4 if ms < 12000 else (-.4 if ms >= 13500 else 0.)
                        header = Header(stamp=stamp(ms), frame_id='wt102_link')
                        rates.publish(Wt102Sample(header=header, stream_id=stream,
                                                  kind=Wt102Sample.YAW_RATE, value=rate))
                        rates.publish(Wt102Sample(header=header, stream_id=stream,
                                                  kind=Wt102Sample.RELATIVE_YAW, value=-2.9))
                    if ms % 50 == 0:
                        scan.header = Header(stamp=stamp(ms-10), frame_id='pavo2s_frame')
                        scans.publish(scan)
                    time.sleep(.008)

                def call(client, request):
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=8.)
                    assert future.done() and future.result().status.code == 0
                    return future.result()

                poses = call(query, TrajectoryQuery.Request(trajectory_id=0)).trajectory
                track = [(p.header.stamp.sec + p.header.stamp.nanosec * 1e-9,
                          2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in poses]
                # Query actual scan-matched trajectory nodes, including the outage.
                in_gap = [y for t, y in track if 12.7 < t < 13.4]
                assert len(in_gap) >= 2 and track[-1][0] > 15.1
                if enabled:
                    assert .50 < min(in_gap) <= max(in_gap) < 1.0, track
                    assert abs(track[-1][1]) < .25, track
                else:
                    assert max(abs(y) for _, y in track) < .1, track
                call(write, WriteState.Request(filename=str(saved), include_unfinished_submaps=True))
                assert saved.stat().st_size > 1000
            finally:
                node.destroy_node()
                rclpy.shutdown()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
