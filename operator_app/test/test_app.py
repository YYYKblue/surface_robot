"""Qt interaction/transport regressions; run with QT_QPA_PLATFORM=offscreen."""
import math
import time

import pytest

try:
    from PySide6.QtCore import QPointF, QPoint, Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtCore import QPointF, QPoint, Qt
    from PyQt5.QtTest import QTest
    from PyQt5.QtWidgets import QApplication

from operator_app.map_canvas import MapCanvas
from operator_app.transport import SocketWorker
from operator_app.window import OperatorWindow


@pytest.fixture(scope="module")
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


def mapping(yaw=0.):
    return dict(type="map", map=dict(resolution=.1, width=40, height=20, revision=1,
        origin=dict(x=-2., y=3., yaw=yaw), frame_id="map", data=[0]*800))


@pytest.mark.parametrize("yaw", [0., math.pi/2, -.7])
def test_map_roundtrip_includes_rotated_origin_and_resize(app, yaw):
    canvas = MapCanvas()
    canvas.set_telemetry(mapping(yaw))
    for width, height in [(700, 500), (1200, 800)]:
        canvas.resize(width, height)
        x, y = -2+math.cos(yaw)-.5*math.sin(yaw), 3+math.sin(yaw)+.5*math.cos(yaw)
        assert canvas.screen_to_map(canvas.map_to_screen(x, y)) == pytest.approx((x, y))
    assert canvas.screen_to_map(QPointF(0, 0)) is None


def test_pose_drag_publishes_real_map_heading_without_pi_offset(app):
    canvas = MapCanvas()
    canvas.set_telemetry(mapping(math.pi/2))
    canvas.resize(800, 600)
    canvas.show()
    canvas.set_picking(True)
    selected = []
    canvas.initial_pose_selected.connect(selected.append)
    start = canvas.map_to_screen(-2.5, 4.)
    end = canvas.map_to_screen(-2.5, 4.5)
    QTest.mousePress(canvas, Qt.LeftButton, pos=start.toPoint())
    QTest.mouseMove(canvas, end.toPoint())
    QTest.mouseRelease(canvas, Qt.LeftButton, pos=end.toPoint())
    assert selected[-1]["yaw"] == pytest.approx(math.pi/2, abs=.02)
    assert selected[-1]["x"] == pytest.approx(-2.5, abs=.02)
    canvas.set_telemetry(dict(pose=None, scan=None))
    assert canvas.telemetry["pose"] is None
    canvas.close()


def test_manual_coalescing_preserves_all_discrete_commands(app):
    worker = SocketWorker("unused")
    pose = dict(op="initial_pose", request_id="pose")
    start = dict(op="task", request_id="start", action="start")
    worker.send_packet(pose)
    for seq in range(100):
        worker.send_packet(dict(op="manual_input", seq=seq))
    worker.send_packet(start)
    worker.send_packet(dict(op="manual_input", seq=100))
    packets = worker._take_outgoing()
    assert packets == [pose, start, dict(op="manual_input", seq=100)]
    for _ in range(32):
        assert worker.send_packet(start)
    assert worker.send_packet(pose) is False  # Explicit failure, never silent loss.


@pytest.mark.parametrize("kind, edge_count", [("l", 2), ("rectangle", 4)])
def test_mock_workflow_localization_start_completion_and_stale_gate(app, kind, edge_count):
    window = OperatorWindow("unused", mock=True)
    window.show()
    try:
        QTest.qWait(250)
        window.task_combo.setCurrentIndex(window.task_combo.findData(kind))
        assert not window.task_buttons["start"].isEnabled()
        window.pose_button.setChecked(True)
        window._pose_selected(dict(x=1., y=1., yaw=0., map_revision=1))
        assert window.publish_pose_button.isEnabled()
        window._publish_pose()
        assert not window.task_buttons["start"].isEnabled()
        QTest.qWait(1400)
        assert window.task_buttons["start"].isEnabled()
        window._task("start")
        assert not window.task_buttons["start"].isEnabled()
        assert not window.drive_buttons["FORWARD"].isEnabled()
        QTest.qWait(8500)
        assert "任务完成" in window.task_status.text()
        assert "完成时间" in window.result_label.text()
        assert len(window.seen_completions) == 1
        assert window.canvas.task_overlay["actual_track"]["point_count"] > 30
        assert len(window.canvas.task_overlay["expected_track"]["cleaning_segments"]) == edge_count
        assert not window.canvas.task_overlay["expected_track"]["navigation_path"]
        assert "35.0 cm" in window.wall_distance.text()
        window.last_telemetry_at = time.monotonic()-2
        window._refresh_controls()
        assert not window.task_buttons["start"].isEnabled()
        assert "未知" in window.task_status.text()
        assert window.canvas.telemetry["pose"] is None
        assert window.canvas.task_overlay["actual_track"]["point_count"] > 30
        assert all(not dashed for _, _, dashed in window.canvas._track_layers())
        assert "过期" in window.track_status.text()
        assert "无有效观测" in window.wall_distance.text()
    finally:
        window.close()


def test_track_layers_keep_l_open_and_reject_mismatched_frames_and_runs(app):
    canvas = MapCanvas()
    canvas.set_telemetry(mapping(math.pi/2))
    task = dict(map_id="map-a", run_id="run",
        expected_track=dict(map_id="map-a", frame_id="map", cleaning_segments=[
            dict(points=[[-2., 3.], [-2., 4.]]), dict(points=[[-2., 4.], [-3., 4.]])],
            navigation_path=[[-2.5, 3.], [-2., 3.]]),
        actual_track=dict(map_id="map-a", frame_id="map", run_id="run",
                          segments=[[[-2., 3.], [-2., 3.5]], [[-2., 4.], [-2.5, 4.]]]))
    canvas.set_task_overlay(task, live=True)
    assert len(canvas._track_layers()) == 5
    assert all(len(points) == 2 for points, _, _ in canvas._track_layers())
    canvas.grab()  # Exercise painting with rotated origin and independent segments.
    canvas.set_task_overlay(task, live=False)
    assert len(canvas._track_layers()) == 2  # Historic path survives telemetry expiry.
    task["actual_track"]["run_id"] = "old-run"
    assert not canvas._track_layers()
    task["expected_track"]["frame_id"] = "odom"
    canvas.set_task_overlay(task, live=True)
    assert not canvas._track_layers()


def test_stop_cancel_and_wall_distance_follow_robot_state(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        window._message(dict(type="hello", client_id="test", capabilities=[
            "task_selection", "initial_pose", "start_conditions", "task_stop", "task_tracks"]))
        window._message(mapping())
        state = dict(type="telemetry", active_client="test", task_busy=True, active_tasks=["l"],
                     localization=dict(map_id="map-a", fresh=True), tasks=dict(l=dict(
                         map_id="map-a", fresh=True, phase="FOLLOW", run_id="run", wall_valid=True,
                         wall_fresh=True, center_wall_distance_m=.342)))
        window._message(state)
        assert window.task_buttons["stop"].isEnabled()
        assert "34.2 cm" in window.wall_distance.text()
        window._task("stop")
        sent = window.worker._take_outgoing()[-1]
        assert sent["action"] == "stop" and sent["task_type"] == "l"
        assert not window.task_buttons["cancel"].isEnabled()
        window._message(dict(type="reply", request_id=sent["request_id"], success=True))
        state.update(task_busy=False, active_tasks=[])
        state["tasks"]["l"].update(phase="STOPPED", wall_fresh=False)
        window._message(state)
        assert not window.task_buttons["stop"].isEnabled()
        assert window.task_buttons["cancel"].isEnabled()
        assert "无有效观测" in window.wall_distance.text()
        window._task("cancel")
        assert window.worker._take_outgoing()[-1]["action"] == "cancel"
    finally:
        window.close()
