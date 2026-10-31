"""WebSocket transport worker for the Qt operator app."""
import json
import math
import threading
import time
from collections import deque

try:
    from PySide6.QtCore import QThread, Signal
except ImportError:
    from PyQt5.QtCore import QThread, pyqtSignal as Signal


class SocketWorker(QThread):
    message = Signal(dict)
    status = Signal(str)

    def __init__(self, url, *, mock=False, parent=None):
        super().__init__(parent)
        self.url, self.mock = url, mock
        self.commands = deque()
        self.latest_manual = None
        self.queue_lock = threading.Lock()
        self.stop_requested = threading.Event()

    def send_packet(self, packet):
        with self.queue_lock:
            if packet.get("op") == "manual_input":
                self.latest_manual = packet
            else:
                if len(self.commands) >= 32:
                    self.status.emit("操作队列已满，请等待响应")
                    return False
                self.commands.append(packet)
                # Never replay manual intent queued before a discrete operation.
                self.latest_manual = None
        return True

    def _take_outgoing(self):
        with self.queue_lock:
            packets = list(self.commands)
            self.commands.clear()
            if self.latest_manual is not None:
                packets.append(self.latest_manual)
                self.latest_manual = None
            return packets

    def close(self):
        self.stop_requested.set()

    def run(self):
        if self.mock:
            self.status.emit("模拟模式")
            self._mock_loop()
            return
        try:
            import websocket
        except ImportError:
            self.status.emit("缺少 websocket-client，请安装 requirements.txt")
            return
        while not self.stop_requested.is_set():
            ws = None
            try:
                ws = websocket.create_connection(self.url, timeout=.5, enable_multithread=True)
                ws.settimeout(.05)
                self.status.emit("已连接")
                self._send_now(ws, {"op": "hello", "client": "f1-operator-qt", "version": 1})
                while not self.stop_requested.is_set():
                    self._flush(ws)
                    try:
                        raw = ws.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not raw:
                        raise ConnectionError("网关已关闭连接")
                    try:
                        self.message.emit(json.loads(raw))
                    except (TypeError, ValueError):
                        self.status.emit("网关发送了无效数据")
            except Exception as error:
                self.status.emit("连接失败：" + str(error) + "；正在重连")
            finally:
                with self.queue_lock:
                    self.commands.clear()
                    self.latest_manual = None
                if ws is not None:
                    try:
                        self._send_now(ws, {"op": "manual_stop", "reason": "app_closed"})
                    except Exception:
                        pass
                    ws.close(timeout=.2)
            self.stop_requested.wait(.7)

    def _flush(self, ws):
        for packet in self._take_outgoing():
            self._send_now(ws, packet)

    @staticmethod
    def _send_now(ws, packet):
        ws.send(json.dumps(packet, ensure_ascii=False, separators=(",", ":")))

    def _mock_loop(self):
        """Interactive UI fixture only; never opens a hardware connection."""
        import uuid
        self.message.emit({"type": "hello", "client_id": "mock", "capabilities": ["task_selection", "initial_pose", "start_conditions", "task_stop", "task_tracks", "mapping", "map_catalog"]})
        mock_map = {"resolution": .02, "width": 160, "height": 100,
            "frame_id": "map", "revision": 1, "origin": {"x": 0, "y": 0, "yaw": 0}, "data": [0]*16000}
        self.message.emit({"type": "map", "map": mock_map})
        runtime = dict(mode="localization", phase="LOCALIZATION", session_id="mock-session", current_map="模拟地图",
                       scan_fresh=True, pose_fresh=True, map_available=True, busy=False, message="模拟模式，不连接硬件",
                       maps_directory="模拟工作空间/maps", last_saved=None)
        catalog = [dict(name="模拟地图", valid=True, saved_at="模拟数据", reason="")]
        initialized = None
        active = None
        started = None
        tasks = {kind: dict(task_type=kind, phase="IDLE", run_id="", output_enabled=False,
                           map_id="mock-map", edge_count=2 if kind == "l" else 4) for kind in ("l", "rectangle")}
        pose = dict(x=1.15, y=1., yaw=0.)
        route = [[.45, 1.65], [2.75, 1.65], [2.75, .35], [.45, .35], [.45, 1.65]]
        for kind, task in tasks.items():
            task["expected_track"] = dict(frame_id="map", map_id="mock-map",
                cleaning_segments=[dict(edge_id=str(i), points=route[i:i+2])
                                   for i in range(task["edge_count"])], navigation_path=[])
            task["actual_track"] = dict(frame_id="map", map_id="mock-map", run_id="",
                                        segments=[], point_count=0, truncated=False)
        start_pose = dict(pose)
        while not self.stop_requested.wait(.1):
            now = time.monotonic()
            ready = initialized is not None and now-initialized >= 1. and runtime["mode"] == "localization"
            for packet in self._take_outgoing():
                op = packet["op"]
                success, message = True, "模拟操作已接受"
                fields = {}
                if op == "mapping":
                    action = packet["action"]
                    if active and action not in ("list", "preview"):
                        success, message = False, "请先停止清洗任务"
                    elif action == "list":
                        fields["maps"] = list(catalog)
                    elif action == "preview":
                        fields.update(preview=mock_map, name=packet["name"])
                    elif action == "start" and runtime["mode"] != "mapping":
                        initialized, ready = None, False
                        runtime.update(mode="mapping", phase="MAPPING", session_id=uuid.uuid4().hex, current_map="")
                        mock_map["revision"] += 1
                        self.message.emit(dict(type="map", map=dict(mock_map)))
                    elif action in ("save", "save_end") and runtime["mode"] == "mapping":
                        from datetime import datetime
                        base = packet.get("name", "").strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
                        name, index = base, 0
                        while any(entry["name"] == name for entry in catalog):
                            index += 1
                            name = f"{base}_{index:02d}"
                        catalog.insert(0, dict(name=name, valid=True, saved_at="模拟数据", reason=""))
                        saved = dict(name=name, paths=[f"模拟工作空间/maps/{name}{s}" for s in (".pbstream", ".pgm", ".yaml")])
                        fields["saved"] = saved
                        runtime["last_saved"] = saved
                        if action == "save_end":
                            runtime.update(mode="idle", phase="IDLE")
                            self.message.emit(dict(type="map", map=None))
                    elif action == "discard" and runtime["mode"] == "mapping" and packet.get("confirm"):
                        runtime.update(mode="idle", phase="IDLE")
                        self.message.emit(dict(type="map", map=None))
                    elif action == "load" and runtime["mode"] != "mapping":
                        initialized, ready = None, False
                        runtime.update(mode="localization", phase="LOCALIZATION", current_map=packet["name"], session_id=uuid.uuid4().hex)
                        mock_map["revision"] += 1
                        self.message.emit(dict(type="map", map=dict(mock_map)))
                        for task in tasks.values():
                            task.update(phase="IDLE", run_id="")
                            task["actual_track"].update(run_id="", segments=[], point_count=0)
                    else:
                        success, message = False, "当前模式不支持此操作"
                elif op == "initial_pose":
                    initialized, ready = now, False
                    pose = {key: packet[key] for key in ("x", "y", "yaw")}
                elif op == "task":
                    kind, action = packet["task_type"], packet["action"]
                    if action == "start" and ready and active is None:
                        active, started = kind, now
                        tasks[kind].update(phase="NAVIGATING", run_id=uuid.uuid4().hex)
                        start_pose = dict(pose)
                        tasks[kind]["actual_track"].update(run_id=tasks[kind]["run_id"],
                            segments=[[[pose["x"], pose["y"]]]], point_count=1)
                    elif action in ("pause", "stop", "cancel"):
                        tasks[kind]["phase"] = {"pause": "PAUSED", "stop": "STOPPED", "cancel": "CANCELLED"}[action]
                        active = None
                    else:
                        success, message = False, "模拟启动条件未满足"
                else:
                    continue
                self.message.emit(dict(type="reply", request_id=packet.get("request_id"), op=op,
                                       success=success, message=message, **fields))
            if active:
                elapsed = now-started
                if elapsed < 3:
                    a, b, fraction = [start_pose["x"], start_pose["y"]], route[0], elapsed/3
                else:
                    progress = min((elapsed-3)/5, 1.)*tasks[active]["edge_count"]
                    index = min(int(progress), tasks[active]["edge_count"]-1)
                    a, b, fraction = route[index], route[index+1], progress-index
                pose = dict(x=a[0]+(b[0]-a[0])*fraction, y=a[1]+(b[1]-a[1])*fraction,
                            yaw=math.atan2(b[1]-a[1], b[0]-a[0]))
                history = tasks[active]["actual_track"]
                history["segments"][0].append([pose["x"], pose["y"]])
                history["point_count"] += 1
                if now-started > 8:
                    tasks[active].update(phase="COMPLETE", completed_at_ns=time.time_ns())
                    active = None
                elif now-started > 3:
                    tasks[active]["phase"] = "FOLLOW"
            for task in tasks.values():
                blockers = []
                if not ready:
                    blockers.append(dict(code="localization", message="等待定位稳定"))
                if active or task["phase"] == "PAUSED":
                    blockers.append(dict(code="phase", message="任务运行中或需要先取消暂停任务"))
                task.update(fresh=True, can_start=not blockers, start_blockers=blockers,
                            wall_fresh=ready, wall_valid=ready,
                            center_wall_distance_m=.35 if ready else None)
                task["expected_track"]["navigation_path"] = (
                    [[start_pose["x"], start_pose["y"]], route[0]]
                    if task["phase"] == "NAVIGATING" else [])
            self.message.emit(dict(type="telemetry", state=dict(state="READY", connected=True,
                source="NANO", motion_authorized=False, allow_fifth_thruster=True), control_fresh=True,
                pose=pose if runtime["mode"] != "idle" else None, scan=None,
                runtime=dict(runtime), tasks=json.loads(json.dumps(tasks)) if runtime["mode"] == "localization" else {}, task_busy=bool(active),
                operation_pending=False, active_tasks=[active] if active else [], active_client="mock",
                can_set_initial_pose=active is None and runtime["mode"] == "localization",
                localization=dict(fresh=True, ready=ready, initial_pose_received=initialized is not None,
                    state="LOCALIZATION_READY" if ready else "LOCALIZATION_CHECK", map_id="mock-map",
                    pose_fresh=ready, scan_fresh=True, map_available=True, stable_sec=1.,
                    stable_elapsed_sec=0. if initialized is None else now-initialized)))
