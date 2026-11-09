"""Real localhost WebSocket hub to Qt transport/display, without ROS or hardware."""
import base64
import json
import time
import zlib

import pytest

pytest.importorskip("websocket")
pytest.importorskip("websockets")
hub_module = pytest.importorskip("operator_gateway.websocket_hub")

try:
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtTest import QTest
    from PyQt5.QtWidgets import QApplication

from operator_app.window import OperatorWindow


def test_real_socket_compact_map_pose_tracks_and_burst_recovery():
    app = QApplication.instance() or QApplication([])
    hub = hub_module.WebSocketHub("127.0.0.1", 0, None)
    hub.start()
    port = hub.server.sockets[0].getsockname()[1]
    window = OperatorWindow(f"ws://127.0.0.1:{port}")
    window.show()
    client = None
    sequence = 0
    track = dict(frame_id="map", map_id="test-map", run_id="run",
                 segments=[[[0., 0.], [1., 1.]]], point_count=2, truncated=False)
    mapping = dict(frame_id="map", revision=1, width=1000, height=1000, resolution=.05,
                   origin=dict(x=-2., y=-3., yaw=.3), encoding="indexed8_zlib",
                   data_zlib_base64=base64.b64encode(zlib.compress(bytes([0, 1, 2, 1]) * 250000)).decode())

    def send_telemetry():
        nonlocal sequence
        sequence += 1
        hub.send(client, dict(type="telemetry", stream=dict(session="stream", sequence=sequence,
            generated_monotonic=time.monotonic()), state=dict(state="READY", allow_fifth_thruster=True),
            control_fresh=True, pose=dict(x=sequence / 100., y=1., yaw=.1),
            tasks=dict(l=dict(task_type="l", map_id="test-map", run_id="run", fresh=True, phase="STOPPED")),
            localization=dict(map_id="test-map", fresh=True, ready=True), active_client=client))

    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            for event in hub.poll():
                if event[0] == "packet" and json.loads(event[2]).get("op") == "hello":
                    client = event[1]
                    hub.send(client, dict(type="hello", session_id="stream", client_id=client,
                        capabilities=["task_selection", "initial_pose", "start_conditions", "task_tracks", "diagnostic_history"]))
                    hub.send(client, dict(type="map", map=mapping))
                    hub.send(client, dict(type="tracks", session_id="stream", tasks=dict(l=track)))
            if client:
                send_telemetry()
            QTest.qWait(20)
            if (window.canvas.map_image is not None and window.last_telemetry_at
                    and window.canvas.task_overlay.get("actual_track", {}).get("point_count") == 2):
                break
        assert window.canvas.map_image is not None, window.log_view.toPlainText()
        assert window.canvas.map_image.width() == 1000
        assert window.canvas.map_image.pixelIndex(0, 999) == 0
        assert window.canvas.telemetry["map"]["origin"] == mapping["origin"]
        assert window.canvas.task_overlay["actual_track"]["point_count"] == 2
        assert "data_zlib_base64" not in window.canvas.telemetry["map"]
        assert not window.manual_locked
        for _ in range(500):
            send_telemetry()
        target = sequence
        deadline = time.monotonic() + 3
        while window._telemetry_sequence != target and time.monotonic() < deadline:
            QTest.qWait(20)
        assert window._telemetry_sequence == target
        assert window.canvas.telemetry["pose"]["x"] == target / 100.
        assert hub.stats(include_clients=False)["pending_latest"] <= 3
        assert len(window.worker.incoming_latest) <= 3
    finally:
        window.close()
        hub.close()
        app.processEvents()
