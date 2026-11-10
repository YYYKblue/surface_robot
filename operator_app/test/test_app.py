"""Qt interaction/transport regressions; run with QT_QPA_PLATFORM=offscreen."""
import math
import time

import pytest

try:
    from PySide6.QtCore import QPointF, QPoint, Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QTabWidget
except ImportError:
    from PyQt5.QtCore import QPointF, QPoint, Qt
    from PyQt5.QtTest import QTest
    from PyQt5.QtWidgets import QApplication, QTabWidget

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
        assert window.forward_boost_button.isEnabled()
        window.forward_boost_button.click()
        QTest.qWait(250)
        assert window.forward_boost_button.isChecked()
        assert not window.task_buttons["start"].isEnabled()
        window.pose_button.setChecked(True)
        window._pose_selected(dict(x=1., y=1., yaw=0., map_revision=1))
        assert window.publish_pose_button.isEnabled()
        window._publish_pose()
        assert not window.task_buttons["start"].isEnabled()
        QTest.qWait(1400)
        assert window.task_buttons["start"].isEnabled()
        window._task("start")
        assert not window.forward_boost_button.isEnabled()
        assert not window.task_buttons["start"].isEnabled()
        assert not window.drive_buttons["FORWARD"].isEnabled()
        QTest.qWait(8500)
        assert "任务完成" in window.task_status.text()
        assert "完成时间" in window.result_label.text()
        assert len(window.seen_completions) == 1
        assert window.canvas.task_overlay["actual_track"]["point_count"] > 30
        assert "expected_track" not in window.canvas.task_overlay
        assert "停车点" not in window.track_status.text()
        assert "35.0 cm" in window.wall_distance.text()
        window.last_telemetry_at = time.monotonic()-2
        window._refresh_controls()
        assert not window.task_buttons["start"].isEnabled()
        assert "未知" in window.task_status.text()
        assert window.canvas.telemetry["pose"] is None
        assert window.canvas.task_overlay["actual_track"]["point_count"] > 30
        assert all(not dashed for _, _, dashed in window.canvas._track_layers())
        assert "过期" in window.track_status.text()
        assert "停车点" not in window.track_status.text()
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
    assert len(canvas._track_layers()) == 2
    assert all(len(points) == 2 for points, _, _ in canvas._track_layers())
    canvas.grab()  # Exercise painting with rotated origin and independent segments.
    canvas.set_task_overlay(task, live=False)
    assert len(canvas._track_layers()) == 2  # Historic path survives telemetry expiry.
    task["actual_track"]["run_id"] = "old-run"
    assert not canvas._track_layers()
    task["expected_track"]["frame_id"] = "odom"
    canvas.set_task_overlay(task, live=True)
    assert not canvas._track_layers()


def test_canvas_ignores_legacy_scan_expected_and_start_region(app):
    canvas = MapCanvas()
    canvas.set_telemetry(mapping(.7))
    canvas.show()
    app.processEvents()
    baseline = canvas.grab().toImage()
    canvas.set_telemetry(dict(scan=dict(frame_id="map", points=[[-2., 3.5]])))
    canvas.set_task_overlay(dict(map_id="map-a", expected_track=dict(
        map_id="map-a", frame_id="map", basis="boat_center_stop", front_stop_m=.4,
        cleaning_segments=[dict(points=[[-2., 3.], [-2., 4.]])],
        start_region=dict(reference_wall_point=[-2., 3.5],
                          polygon=[[-2., 3.4], [-2., 3.6], [-2.8, 3.6], [-2.8, 3.4]]))), live=True)
    assert not canvas._track_layers()
    assert "scan" not in canvas.telemetry
    assert canvas.grab().toImage() == baseline
    canvas.close()


@pytest.mark.parametrize("scope", ["remaining_open_route", "remaining_l_route"])
def test_remaining_l_completion_never_claims_full_route_coverage(app, monkeypatch, scope):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        window._message(dict(type="hello", client_id="test", capabilities=[
            "task_selection", "initial_pose", "start_conditions", "task_tracks"]))
        window._message(mapping())
        task = dict(map_id="map-a", fresh=True, phase="MOVE_START_SIDE", run_id="run",
                    start_edge_id="edge_1", current_edge_id="edge_1", completion_scope=scope,
                    start_region=dict(reference_wall_point=[-2., 3.5]), coverage_complete=False)
        state = dict(type="telemetry", active_client="test", task_busy=True, active_tasks=["l"],
                     localization=dict(map_id="map-a", fresh=True), tasks=dict(l=task))
        window._message(state)
        assert "侧移" in window.task_status.text() and "edge_1" in window.start_details.text()
        task.update(phase="COMPLETE", completed_at_ns=time.time_ns())
        window._message(state)
        assert "剩余开放墙链" in window.result_label.text()
        assert "覆盖记录仍有缺口" in window.result_label.text()
        assert "要求区间已覆盖" not in window.result_label.text()
    finally:
        window.close()


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


def test_forward_boost_button_tracks_confirmed_state_and_locks(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        window._message(dict(type="hello", client_id="test", capabilities=[
            "task_selection", "initial_pose", "start_conditions", "forward_boost"]))
        task = dict(map_id="map-a", fresh=True, phase="IDLE", run_id="", can_start=True,
                    forward_boost_enabled=False, forward_boost_active=False,
                    forward_boost_reason="disabled", can_set_forward_boost=True)
        state = dict(type="telemetry", active_client="test", task_busy=False, active_tasks=[],
                     localization=dict(map_id="map-a", fresh=True), tasks=dict(l=task))
        window._message(state)
        assert window.forward_boost_button.isEnabled()
        window.forward_boost_button.click()
        sent = window.worker._take_outgoing()[-1]
        assert sent["task_type"] == "l" and sent["action"] == "set_forward_boost" and sent["enabled"] is True
        assert not window.forward_boost_button.isChecked()  # Not optimistic.
        assert not window.forward_boost_button.isEnabled()
        assert not window.task_buttons["start"].isEnabled()
        window._message(dict(type="reply", request_id=sent["request_id"], success=False, message="rejected"))
        assert not window.forward_boost_button.isChecked()
        task.update(forward_boost_enabled=True, forward_boost_reason="inactive_task")
        window._message(state)  # Also represents readback after reconnect.
        assert window.forward_boost_button.isChecked()
        for phase in ("ACQUIRING", "FOLLOW", "PAUSED"):
            task.update(phase=phase, can_set_forward_boost=False)
            state.update(task_busy=phase != "PAUSED")
            window._message(state)
            assert not window.forward_boost_button.isEnabled()
            assert window.forward_boost_button.isChecked()
        task.update(phase="CANCELLED", can_set_forward_boost=True)
        state.update(task_busy=False)
        window._message(state)
        assert window.forward_boost_button.isEnabled()
        window.last_telemetry_at = time.monotonic()-2
        window._refresh_controls()
        assert not window.forward_boost_button.isEnabled()
        assert "未知" in window.forward_boost_button.text()
        window._message(state)
        window.capabilities.discard("forward_boost")
        window._refresh_controls()
        assert not window.forward_boost_button.isEnabled()
        assert "更新" in window.forward_boost_status.text()
    finally:
        window.close()


def test_mapping_catalog_save_load_and_text_input_never_drives(app):
    window = OperatorWindow('unused', mock=True)
    window.show()
    try:
        QTest.qWait(350)
        assert window.saved_maps.count() == 1
        assert window.mapping_buttons['start'].isEnabled()
        window._mapping_action('start')
        QTest.qWait(250)
        assert window.telemetry['runtime']['mode'] == 'mapping'
        assert window.mapping_buttons['save_end'].isEnabled()
        assert not window.pose_button.isEnabled()
        assert not window.task_buttons['start'].isEnabled()
        # Show mapping tab and type all movement keys, including space.
        # Find tab through Qt ownership (works independently of layout index).
        tabs = window.findChild(QTabWidget)
        tabs.setCurrentIndex(2)
        window.map_name.setFocus()
        window.activateWindow()
        QTest.keyClicks(window.map_name, 'wasd test')
        assert window.map_name.text() == 'wasd test'
        assert window.direction == 'STOP' and not window._pressed_keys
        window.map_name.setText('测试池')
        window._mapping_action('save_end')
        QTest.qWait(350)
        assert window.telemetry['runtime']['mode'] == 'idle'
        assert window.saved_maps.currentData()['name'] == '测试池'
        assert '测试池.pbstream' in window.mapping_result.text()
        window._mapping_action('load')
        QTest.qWait(300)
        assert window.telemetry['runtime']['current_map'] == '测试池'
        assert window.pose_button.isEnabled()
        assert not window.task_buttons['start'].isEnabled()
        window._mapping_action('preview')
        QTest.qWait(250)
        preview = window.preview_dialog.findChild(MapCanvas)
        assert preview.telemetry.get('pose') is None
        assert not preview.task_overlay
        window.last_telemetry_at = time.monotonic()-2
        window._refresh_controls()
        assert not window.mapping_buttons['start'].isEnabled()
        assert not window.mapping_buttons['load'].isEnabled()
    finally:
        if window.preview_dialog:
            window.preview_dialog.close()
        window.close()


def test_reconnect_queries_state_without_replaying_old_commands(app, monkeypatch):
    import json
    import sys
    from types import SimpleNamespace
    worker = SocketWorker('unused')
    connections = []
    class Socket:
        def __init__(self):
            self.sent = []
        def settimeout(self, value):
            pass
        def send(self, value):
            self.sent.append(json.loads(value))
        def close(self, **kwargs):
            pass
        def recv(self):
            if len(connections) == 1:
                worker.send_packet(dict(op='mapping', action='load', request_id='old', name='old-map'))
            else:
                worker.stop_requested.set()
            raise ConnectionError('simulated disconnect')
    def connect(*args, **kwargs):
        connection = Socket()
        connections.append(connection)
        return connection
    monkeypatch.setitem(sys.modules, 'websocket', SimpleNamespace(
        create_connection=connect, WebSocketTimeoutException=TimeoutError))
    worker.run()
    assert len(connections) == 2
    assert connections[1].sent[0]['op'] == 'hello'
    assert all(packet.get('request_id') != 'old' for connection in connections for packet in connection.sent)


def test_diagnostics_replays_deduplicates_exports_and_reports_stale_controls(app, monkeypatch, tmp_path):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        window._message(dict(type="hello", client_id="test", capabilities=["diagnostic_history"]))
        packet = dict(type="telemetry", active_client="test", control_fresh=True,
                      state=dict(state="READY", source="NANO", last_stop_reason="command deadline expired"),
                      diagnostics=dict(session="gateway-1", events=[dict(
                          id=1, time_unix=time.time(), source="gateway", reason="input_timeout",
                          details=dict(input_age_ms=310, input_timeout_ms=250))]))
        window._message(packet)
        for _ in range(10):
            window._message(packet)
        assert window.log_view.toPlainText().count("(input_timeout)") == 1
        assert "310" in window.log_view.toPlainText()
        window._status("连接失败：test")
        assert not window.drive_buttons["FORWARD"].isEnabled()
        assert "遥测" in window.mode_label.text()
        window._status("已连接")
        packet["diagnostics"]["events"].append(dict(id=2, time_unix=time.time(), source="bridge",
            reason="ACK progress timeout", details=dict(ack_age_sec=.42)))
        window._message(packet)
        assert window.log_view.toPlainText().count("(input_timeout)") == 1
        assert "下位机 ACK 确认超时" in window.log_view.toPlainText()
        # Gateway restarts reset IDs; history still gets displayed.
        packet["diagnostics"].update(session="gateway-2", events=[dict(
            id=1, time_unix=time.time(), source="gateway", reason="app_disconnected", details={})])
        window._message(packet)
        assert "app_disconnected" in window.log_view.toPlainText()
        window._copy_diagnostics()
        assert QApplication.clipboard().text() == window.log_view.toPlainText()
        destination = tmp_path / "诊断日志.txt"
        window._write_diagnostics(destination)
        assert "ACK progress timeout" in destination.read_text(encoding="utf-8-sig")
        window._expand_diagnostics()
        window._log("App", "expanded live record")
        assert "expanded live record" in window._log_dialog.findChild(type(window.log_view)).toPlainText()
        window._log_dialog.close()
        window.log_view.clear()
        window._message(packet)
        assert "app_disconnected" not in window.log_view.toPlainText()  # Clear does not replay history.
        for i in range(2100):
            window._log("App", f"event {i}")
        assert window.log_view.document().blockCount() == 2000
    finally:
        window.close()


def test_diagnostics_old_gateway_and_delayed_send_timer(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        window._message(dict(type="hello", client_id="test", capabilities=[]))
        packet = dict(type="telemetry", state=dict(state="HANDSHAKE", reason="CTRL timeout"))
        window._message(packet)
        window._message(packet)
        assert "完整日志需更新 Nano 网关" in window.log_view.toPlainText()
        assert window.log_view.toPlainText().count("(CTRL timeout)") == 1
        window._last_send_tick = time.monotonic() - .35
        window._send_input()
        assert "手动意图刷新定时器延迟" in window.log_view.toPlainText()
        window.last_telemetry_at = time.monotonic() - 1
        window._refresh_controls()
        assert "手动按钮已禁用" in window.log_view.toPlainText()
        assert "超过 800 ms" in window.log_view.toPlainText()
        assert "已过期" in window.diagnostic_summary.text()
    finally:
        window.close()


@pytest.mark.parametrize("direction", ["FORWARD", "LEFT", "STOP", "RIGHT", "REVERSE"])
def test_stale_manual_button_moves_focus_without_capturing_numeric_input(app, monkeypatch, direction):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    window.send_timer.stop()
    window.state_timer.stop()
    window.show()
    window.activateWindow()
    app.processEvents()
    packet = dict(type="telemetry", state=dict(allow_fifth_thruster=True))
    try:
        window._message(packet)
        window.drive_buttons[direction].setFocus()
        app.processEvents()
        assert app.focusWidget() is window.drive_buttons[direction]
        window.last_telemetry_at = time.monotonic() - 2
        window._refresh_controls()
        app.processEvents()
        assert app.focusWidget() is window
        window._message(packet)
        assert not window.manual_locked
        # An intentional numeric edit still stops output and holds the lock.
        window.brush_value.setFocus()
        app.processEvents()
        window._refresh_controls()
        window._message(packet)
        assert window.manual_locked
        assert "输入框持有焦点" in window.mode_label.text()
    finally:
        window.close()


def test_received_time_server_queue_and_sequence_guard_telemetry_freshness(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        old = time.monotonic() - 2
        window._message(dict(type="telemetry", _received_monotonic=old,
                             stream=dict(session="gateway", sequence=1), pose=dict(x=0, y=0, yaw=0)))
        assert window.last_telemetry_at == old
        assert window.manual_locked
        assert window.canvas.telemetry["pose"] is None
        received = time.monotonic()
        window._message(dict(type="telemetry", _received_monotonic=received,
                             stream=dict(session="gateway", sequence=2, server_queue_ms=900)))
        assert window.last_telemetry_at == pytest.approx(received - .9)
        assert window.manual_locked
        window._message(dict(type="telemetry", stream=dict(session="gateway", sequence=3)))
        fresh_time = window.last_telemetry_at
        assert not window.manual_locked
        window._message(dict(type="telemetry", _received_monotonic=old,
                             stream=dict(session="gateway", sequence=4)))
        window._message(dict(type="telemetry", stream=dict(session="gateway", sequence=2)))
        window._message(dict(type="telemetry", _received_monotonic=float("nan")))
        assert window.last_telemetry_at == fresh_time
    finally:
        window.close()


def test_independent_tracks_match_map_run_and_gateway_session(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    track = dict(map_id="map-a", run_id="run-a", frame_id="map", point_count=2,
                 segments=[[[0., 0.], [1., 0.]]])
    def telemetry(run="run-a", session="gateway-a", embedded=None):
        task = dict(map_id="map-a", run_id=run, fresh=True)
        if embedded is not None:
            task["actual_track"] = embedded
        return dict(type="telemetry", stream=dict(session=session),
                    localization=dict(map_id="map-a", fresh=True), tasks=dict(rectangle=task))
    try:
        # Display snapshots may arrive before the corresponding fast state.
        window._message(dict(type="tracks", session_id="gateway-a", tasks=dict(rectangle=track)))
        window._message(telemetry())
        assert window.telemetry["tasks"]["rectangle"]["actual_track"] is track
        window._message(telemetry())
        assert window.canvas.task_overlay["actual_track"] is track
        window._message(telemetry(run="run-b"))
        assert "actual_track" not in window.canvas.task_overlay
        window._message(telemetry())
        assert window.canvas.task_overlay["actual_track"] is track
        window._message(telemetry(session="gateway-b"))
        assert "actual_track" not in window.canvas.task_overlay
        window._message(dict(type="tracks", session_id="gateway-a", tasks=dict(rectangle=track)))
        assert not window._tracks
        window._message(telemetry(session="gateway-b", embedded=track))
        assert window.canvas.task_overlay["actual_track"] is track
        window._message(telemetry(session="gateway-b"))
        assert window.canvas.task_overlay["actual_track"] is track
    finally:
        window.close()


def test_incoming_batch_refreshes_controls_once_after_latest_state(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    try:
        old = time.monotonic() - 2
        packets = [dict(type="telemetry", _received_monotonic=old),
                   dict(type="map", map=mapping()["map"]), dict(type="telemetry")]
        monkeypatch.setattr(window.worker, "take_incoming", lambda: packets, raising=False)
        refresh = window._refresh_controls
        refreshes = []
        def counted(*args):
            if not window._processing_incoming:
                refreshes.append(True)
            refresh(*args)
        monkeypatch.setattr(window, "_refresh_controls", counted)
        window._message(dict(type="incoming_ready"))
        assert refreshes == [True]
        assert not window.manual_locked
        assert window.canvas.map_image is not None
    finally:
        window.close()


def test_prepared_map_and_actual_track_screen_caches(app, monkeypatch):
    canvas = MapCanvas()
    packet = mapping()
    prepared = canvas._make_map_image(packet["map"])
    del packet["map"]["data"]
    packet["_map_image"] = prepared
    def no_gui_decode(_):
        raise AssertionError("Prepared maps must not be decoded again on the GUI thread")
    monkeypatch.setattr(canvas, "_make_map_image", no_gui_decode)
    canvas.set_telemetry(packet)
    assert canvas.map_image is prepared
    canvas.set_task_overlay(dict(map_id="a", run_id="r", actual_track=dict(
        map_id="a", run_id="r", frame_id="map", segments=[[[-2., 3.], [-2., 4.]]])), live=True)
    transform = canvas.map_to_screen
    transformed = []
    def counted(x, y):
        transformed.append((x, y))
        return transform(x, y)
    monkeypatch.setattr(canvas, "map_to_screen", counted)
    canvas._screen_tracks()
    canvas._screen_tracks()
    assert len(transformed) == 2
    canvas.resize(1000, 700)
    canvas._screen_tracks()
    assert len(transformed) == 4
    canvas.show()
    app.processEvents()
    canvas.grab()
    cached = canvas._scaled_image
    canvas.grab()
    assert canvas._scaled_image is cached
    canvas.close()


def test_separate_diagnostics_history_does_not_refresh_telemetry(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    packet = dict(type="diagnostics", diagnostics=dict(session="gateway", events=[dict(
        id=1, source="gateway", reason="input_timeout", time_unix=time.time(), details={})]))
    try:
        window._message(packet)
        window._message(packet)
        assert window.log_view.toPlainText().count("input_timeout") == 1
        assert window.last_telemetry_at == 0.
        assert "等待遥测" == window.diagnostic_summary.text()
    finally:
        window.close()


def test_runtime_map_session_rejects_late_decode_and_preserves_matching_early_map(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    base = time.monotonic() - .2
    def state(session, offset):
        return dict(type="telemetry", runtime=dict(session_id=session),
                    _received_monotonic=base + offset / 100., pose=dict(x=1., y=1., yaw=0.))
    def map_packet(session, offset, revision):
        packet = mapping()
        packet.update(runtime_session_id=session, _received_monotonic=base + offset / 100.)
        packet["map"].update(runtime_session_id=session, revision=revision)
        return packet
    try:
        window._message(state("a", 1))
        window._message(map_packet("a", 2, 1))
        assert window.canvas.map_image is not None
        window._message(state("b", 4))
        assert window.canvas.map_image is None
        # Received before the switch, decoded after it: cannot restore old map.
        window._message(map_packet("a", 3, 2))
        assert window.canvas.map_image is None
        window._message(map_packet("b", 5, 3))
        image_b = window.canvas.map_image
        assert image_b is not None
        # Current-session retransmission after reconnect is accepted.
        window._message(map_packet("b", 6, 4))
        assert window.canvas.map_revision == 4
        # A next-session map may finish before its first fast telemetry.
        window._message(map_packet("c", 7, 5))
        image_c = window.canvas.map_image
        assert image_c is not None
        assert window.canvas.telemetry["pose"] is None
        window._message(state("c", 8))
        assert window.canvas.map_image is image_c
        assert window.canvas.telemetry["pose"] == dict(x=1., y=1., yaw=0.)
        # A delayed old reset packet must not clear the new map either.
        window._message(dict(type="map", map=None, runtime_session_id="b",
                             _received_monotonic=base + .09))
        assert window.canvas.map_image is image_c
        window._message(dict(type="map", map=None, runtime_session_id="c",
                             _received_monotonic=base + .091))
        assert window.canvas.map_image is None
        # Legacy map/reset packets without runtime metadata are still accepted.
        window._message(mapping())
        assert window.canvas.map_image is not None
        window._message(dict(type="map", map=None))
        assert window.canvas.map_image is None
    finally:
        window.close()


def test_first_runtime_state_rejects_unknown_old_map_and_accepts_nested_session(app, monkeypatch):
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    base = time.monotonic() - 10.
    try:
        window._message(dict(type="telemetry", runtime=dict(session_id="new"),
                             _received_monotonic=base + 2))
        old = mapping()
        old["map"]["runtime_session_id"] = "old-unseen"
        old["_received_monotonic"] = base + 1
        window._message(old)
        assert window.canvas.map_image is None
        current = mapping()
        current["map"]["runtime_session_id"] = "new"
        current["_received_monotonic"] = base + 1.5
        window._message(current)
        assert window.canvas.map_image is not None
    finally:
        window.close()
