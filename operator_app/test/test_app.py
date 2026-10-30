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


def test_mock_workflow_localization_start_completion_and_stale_gate(app):
    window = OperatorWindow("unused", mock=True)
    window.show()
    try:
        QTest.qWait(250)
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
        window.last_telemetry_at = time.monotonic()-2
        window._refresh_controls()
        assert not window.task_buttons["start"].isEnabled()
        assert "未知" in window.task_status.text()
        assert window.canvas.telemetry["pose"] is None
    finally:
        window.close()
