import math

from geometry_msgs.msg import PoseWithCovarianceStamped

from f1_pool_cleaning.localization import (
    LocalizationConfig,
    LocalizationGate,
    PoseSample,
    ScanQuality,
    json_status,
    validate_initial_pose,
)


def pose(frame="map", stamp=1_000_000_000, x=-.2, y=.1, yaw=0.):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.)
    message.pose.pose.orientation.w = math.cos(yaw / 2.)
    return message


def test_initial_pose_validation_requires_map_frame_and_planar_finite_quaternion():
    message = pose(frame="odom")
    assert not validate_initial_pose(message, "map")[0]
    message = pose()
    message.pose.pose.orientation.w = 0.
    assert not validate_initial_pose(message, "map")[0]
    message = pose()
    message.pose.pose.position.z = .3
    assert not validate_initial_pose(message, "map")[0]
    assert validate_initial_pose(pose(yaw=.7), "map")[0]


def test_gate_requires_continuous_trajectory_pose_scan_and_overlap_evidence():
    config = LocalizationConfig(stable_sec=.20)
    gate = LocalizationGate(config)
    assert not gate.update(1_000_000_000, False, None, None)
    gate.accept_initial_pose(1_000_000_000)
    current_pose = PoseSample(1_000_000_000, -.2, .1, 0.)
    current_scan = ScanQuality(1_000_000_000, .4, .4)
    assert not gate.update(1_000_000_000, True, current_pose, current_scan)
    assert not gate.update(1_150_000_000, True, current_pose, current_scan)
    assert gate.update(1_250_000_000, True, current_pose, current_scan)
    assert gate.state == "LOCALIZATION_READY"


def test_ready_gate_latches_jump_pause_until_relocalization():
    gate = LocalizationGate(LocalizationConfig(stable_sec=.01))
    gate.accept_initial_pose(1_000_000_000)
    p = PoseSample(1_000_000_000, -.2, .1, 0.)
    s = ScanQuality(1_000_000_000, .4, .4)
    gate.update(1_000_000_000, True, p, s)
    gate.update(1_020_000_000, True, p, s)
    assert gate.ready
    jumped = PoseSample(1_030_000_000, .3, .1, 0.)
    assert not gate.update(1_030_000_000, True, jumped, s)
    assert gate.state == "PAUSED"
    assert not gate.update(2_000_000_000, True, jumped, s)
    gate.reset()
    assert gate.state == "LOCALIZATION_CHECK" and not gate.ready


def test_invalid_replacement_pose_is_not_allowed_to_reuse_old_ready_pose():
    gate = LocalizationGate(LocalizationConfig(stable_sec=.01))
    gate.accept_initial_pose(1_000_000_000)
    p = PoseSample(1_000_000_000, -.2, .1, 0.)
    s = ScanQuality(1_000_000_000, .4, .4)
    gate.update(1_000_000_000, True, p, s)
    gate.update(1_020_000_000, True, p, s)
    assert gate.ready
    gate.mark_paused("invalid replacement initial pose")
    assert not gate.update(2_000_000_000, True, p, s)
    assert gate.state == "PAUSED"


def test_gate_reports_scan_unknown_and_stale_evidence_without_nan_json():
    gate = LocalizationGate()
    gate.accept_initial_pose(1_000_000_000)
    p = PoseSample(1_000_000_000, -.2, .1, 0.)
    gate.update(1_000_000_000, True, p, ScanQuality(1_000_000_000, .4, None))
    assert "unknown" in gate.reason
    status = json_status(gate.as_dict(1_000_000_000))
    assert "NaN" not in status
