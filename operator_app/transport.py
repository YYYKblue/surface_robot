"""WebSocket transport worker for the Qt operator app."""
import json
import math
import threading
import time
import zlib
from collections import deque

try:
    from .map_decoder import MapDecoder, prepare_map
except ImportError:
    from map_decoder import MapDecoder, prepare_map

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
        self.urgent_stop = None
        self.latest_manual = None
        self.latest_manual_at = 0.
        self.manual_generation = 0
        self.queue_lock = threading.Lock()
        self.stop_requested = threading.Event()
        self.incoming_lock = threading.Lock()
        self.incoming_latest = {}
        self.incoming_reliable = deque()
        self.incoming_notified = False
        self.incoming_sequence = 0
        self.incoming_replaced = 0
        self.map_runtime_session = None
        self.map_decoder = MapDecoder(self._queue_incoming, self.status.emit)

    def send_packet(self, packet):
        with self.queue_lock:
            if packet.get("op") == "manual_stop":
                # A full operation queue must never reject the manual stop or
                # leave an older movement intent eligible for transmission.
                self.urgent_stop = packet
                self.latest_manual = None
                self.manual_generation += 1
            elif packet.get("op") == "manual_input":
                self.latest_manual = packet
                self.latest_manual_at = time.monotonic()
            else:
                if len(self.commands) >= 32:
                    self.status.emit("操作队列已满，请等待响应")
                    return False
                self.commands.append(packet)
                # Never replay manual intent queued before a discrete operation.
                self.latest_manual = None
                self.manual_generation += 1
        return True

    def _take_outgoing(self, *, timed=False):
        with self.queue_lock:
            packets = [(self.urgent_stop, None, None)] if self.urgent_stop is not None else []
            self.urgent_stop = None
            packets.extend((packet, None, None) for packet in self.commands)
            self.commands.clear()
            if self.latest_manual is not None:
                # A stalled receiver must not revive an old movement intent.
                if time.monotonic() - self.latest_manual_at < .25:
                    packets.append((self.latest_manual, self.latest_manual_at, self.manual_generation))
                self.latest_manual = None
            return packets if timed else [packet for packet, _, _ in packets]

    def _receive(self, packet, *, received_at=None, wire_bytes=None, parse_ms=None):
        """Keep large Python objects out of Qt's queued signal delivery."""
        if not isinstance(packet, dict):
            raise ValueError("网关消息必须是对象")
        packet = dict(packet)
        packet["_received_monotonic"] = time.monotonic() if received_at is None else received_at
        packet["_transport"] = dict(wire_bytes=wire_bytes, parse_ms=parse_ms)
        with self.incoming_lock:
            self.incoming_sequence += 1
            order = self.incoming_sequence
        if packet.get("type") == "map":
            mapping = packet.get("map") or {}
            session = packet.get("runtime_session_id", mapping.get("runtime_session_id"))
            if session and session != self.map_runtime_session:
                self.map_runtime_session = session
                self.map_decoder.reset()
                with self.incoming_lock:
                    self.incoming_latest.pop("map", None)
            if packet.get("map") is not None:
                self.map_decoder.submit(packet, order)
                return
            self.map_decoder.reset()
        if packet.get("type") == "reply" and packet.get("preview"):
            # Saved-map previews are bounded to 512 pixels by the gateway.
            try:
                packet = prepare_map(packet, "preview")
            except (ValueError, TypeError, KeyError, zlib.error) as error:
                packet.pop("preview", None)
                packet.update(success=False, message="地图预览解码失败：" + str(error))
        self._queue_incoming(packet, order)

    def _queue_incoming(self, packet, order):
        notify = False
        with self.incoming_lock:
            kind = packet.get("type")
            if kind in ("telemetry", "map", "tracks"):
                previous = self.incoming_latest.get(kind)
                if previous and previous[0] > order:
                    return
                if previous:
                    self.incoming_replaced += 1
                self.incoming_latest[kind] = (order, packet)
            else:
                if len(self.incoming_reliable) >= 64:
                    raise ConnectionError("界面操作消息处理超时，正在重新连接")
                self.incoming_reliable.append((order, packet))
            if not self.incoming_notified:
                self.incoming_notified = True
                notify = True
        if notify:
            self.message.emit({"type": "incoming_ready"})

    def take_incoming(self):
        with self.incoming_lock:
            values = list(self.incoming_reliable) + list(self.incoming_latest.values())
            self.incoming_reliable.clear()
            self.incoming_latest.clear()
            self.incoming_notified = False
            replaced = self.incoming_replaced
        now = time.monotonic()
        packets = []
        for _, packet in sorted(values, key=lambda item: item[0]):
            packet["_transport"] = dict(packet.get("_transport") or {},
                gui_queue_ms=round(max(0., now - packet["_received_monotonic"]) * 1000, 2),
                replaced=replaced)
            packets.append(packet)
        return packets

    def _reset_incoming(self):
        self.map_decoder.reset()
        with self.incoming_lock:
            self.incoming_latest.clear()
            # Replies already received remain deliverable after disconnect.
            # A posted Qt wakeup stays armed until take_incoming consumes it.

    def close(self):
        self.stop_requested.set()
        self.map_decoder.close()

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
                    received_at = time.monotonic()
                    try:
                        packet = json.loads(raw)
                        parse_ms = round((time.monotonic() - received_at) * 1000, 2)
                        self._receive(packet, received_at=received_at,
                                      wire_bytes=len(raw.encode("utf-8") if isinstance(raw, str) else raw),
                                      parse_ms=parse_ms)
                    except (TypeError, ValueError):
                        self.status.emit("网关发送了无效数据")
            except Exception as error:
                self._reset_incoming()
                self.status.emit("连接失败：" + str(error) + "；正在重连")
            finally:
                with self.queue_lock:
                    self.commands.clear()
                    self.urgent_stop = None
                    self.latest_manual = None
                if ws is not None:
                    try:
                        self._send_now(ws, {"op": "manual_stop", "reason": "app_closed"})
                    except Exception:
                        pass
                    ws.close(timeout=.2)
            self.stop_requested.wait(.7)

    def _flush(self, ws):
        for packet, generated, generation in self._take_outgoing(timed=True):
            self._flush_urgent_stop(ws)
            if generated is not None:
                with self.queue_lock:
                    invalidated = generation != self.manual_generation
                if invalidated or time.monotonic() - generated >= .25:
                    continue
            self._send_now(ws, packet)
        self._flush_urgent_stop(ws)

    def _flush_urgent_stop(self, ws):
        # Check between writes: a stop can arrive while an earlier operation is
        # in flight. Never hold the GUI-facing queue lock during socket writes.
        with self.queue_lock:
            packet, self.urgent_stop = self.urgent_stop, None
        if packet is not None:
            self._send_now(ws, packet)

    @staticmethod
    def _send_now(ws, packet):
        ws.send(json.dumps(packet, ensure_ascii=False, separators=(",", ":")))

    def _mock_loop(self):
        """Interactive UI fixture only; never opens a hardware connection."""
        import uuid
        self._receive({"type": "hello", "client_id": "mock", "session_id": "mock-stream", "capabilities": ["task_selection", "initial_pose", "start_conditions", "task_stop", "task_tracks", "mapping", "map_catalog", "forward_boost"]})
        mock_map = {"resolution": .02, "width": 160, "height": 100,
            "frame_id": "map", "revision": 1, "origin": {"x": 0, "y": 0, "yaw": 0}, "data": [0]*16000}
        self._receive({"type": "map", "map": dict(mock_map), "runtime_session_id": "mock-session"})
        runtime = dict(mode="localization", phase="LOCALIZATION", session_id="mock-session", current_map="模拟地图",
                       scan_fresh=True, pose_fresh=True, map_available=True, busy=False, message="模拟模式，不连接硬件",
                       maps_directory="模拟工作空间/maps", last_saved=None)
        catalog = [dict(name="模拟地图", valid=True, saved_at="模拟数据", reason="")]
        initialized = None
        active = None
        started = None
        tasks = {kind: dict(task_type=kind, phase="IDLE", run_id="", output_enabled=False,
                           forward_boost_enabled=False, forward_boost_active=False,
                           map_id="mock-map", edge_count=2 if kind == "l" else 4) for kind in ("l", "rectangle")}
        pose = dict(x=1.15, y=1., yaw=0.)
        # Synthetic wall endpoints; target lines are independent centre segments.
        route = [[0., 2.], [3.2, 2.], [3.2, 0.], [0., 0.], [0., 2.]]
        segments = []
        for i, (a, b) in enumerate(zip(route, route[1:])):
            length = math.dist(a, b)
            tx, ty = (b[0]-a[0])/length, (b[1]-a[1])/length
            points = [[a[0]+tx*s+ty*.33, a[1]+ty*s-tx*.33]
                      for s in (.20, length-.40)]
            segments.append(dict(edge_id=str(i), points=points))
        for kind, task in tasks.items():
            task["actual_track"] = dict(frame_id="map", map_id="mock-map", run_id="",
                                        segments=[], point_count=0, truncated=False)
        start_pose = dict(pose)
        last_tracks = 0.
        sequence = 0
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
                        self._receive(dict(type="map", map=dict(mock_map), runtime_session_id=runtime["session_id"]))
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
                            self._receive(dict(type="map", map=None, runtime_session_id=runtime["session_id"]))
                    elif action == "discard" and runtime["mode"] == "mapping" and packet.get("confirm"):
                        runtime.update(mode="idle", phase="IDLE")
                        self._receive(dict(type="map", map=None, runtime_session_id=runtime["session_id"]))
                    elif action == "load" and runtime["mode"] != "mapping":
                        initialized, ready = None, False
                        runtime.update(mode="localization", phase="LOCALIZATION", current_map=packet["name"], session_id=uuid.uuid4().hex)
                        mock_map["revision"] += 1
                        self._receive(dict(type="map", map=dict(mock_map), runtime_session_id=runtime["session_id"]))
                        for task in tasks.values():
                            task.update(phase="IDLE", run_id="", forward_boost_enabled=False)
                            task["actual_track"].update(run_id="", segments=[], point_count=0)
                    else:
                        success, message = False, "当前模式不支持此操作"
                elif op == "initial_pose":
                    initialized, ready = now, False
                    pose = {key: packet[key] for key in ("x", "y", "yaw")}
                elif op == "task":
                    kind, action = packet["task_type"], packet["action"]
                    if action == "set_forward_boost":
                        if active is None and tasks[kind]["phase"] in ("IDLE", "CANCELLED", "STOPPED", "COMPLETE"):
                            tasks[kind]["forward_boost_enabled"] = packet["enabled"]
                        else:
                            success, message = False, "只能在任务开始前设置提速"
                    elif action == "start" and ready and active is None:
                        active, started = kind, now
                        region = dict(edge_id="edge_0", reference_wall_point=[.2, 2.],
                                      polygon=[[.1, 2.], [.3, 2.], [.3, 1.15], [.1, 1.15]])
                        tasks[kind].update(phase="APPROACH_START", run_id=uuid.uuid4().hex,
                                           start_edge_id="edge_0", current_edge_id="edge_0", start_region=region,
                                           completion_scope="remaining_open_route" if kind == "l" else "rectangle_route",
                                           coverage_complete=False, returning_to_start=False)
                        tasks[kind].update(forward_boost_cutoff_reached=False, boost_edge_index=None)
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
                self._receive(dict(type="reply", request_id=packet.get("request_id"), op=op,
                                       success=success, message=message, **fields))
            if active:
                elapsed = now-started
                if elapsed < 3:
                    a, b, fraction = [start_pose["x"], start_pose["y"]], segments[0]["points"][0], elapsed/3
                else:
                    progress = min((elapsed-3)/5, 1.)*tasks[active]["edge_count"]
                    index = min(int(progress), tasks[active]["edge_count"]-1)
                    a, b = segments[index]["points"]
                    fraction = progress-index
                    task = tasks[active]
                    if task.get("boost_edge_index") != index:
                        task.update(boost_edge_index=index, forward_boost_cutoff_reached=False)
                    task["map_front_clearance_m"] = math.dist(a, b)*(1-fraction)+.40
                    if task["map_front_clearance_m"] <= .8:
                        task["forward_boost_cutoff_reached"] = True
                pose = dict(x=a[0]+(b[0]-a[0])*fraction, y=a[1]+(b[1]-a[1])*fraction,
                            yaw=math.atan2(b[1]-a[1], b[0]-a[0]))
                history = tasks[active]["actual_track"]
                history["segments"][0].append([pose["x"], pose["y"]])
                history["point_count"] += 1
                if now-started > 8:
                    tasks[active].update(phase="COMPLETE", completed_at_ns=time.time_ns(), coverage_complete=True)
                    active = None
                elif now-started > 3:
                    tasks[active]["phase"] = "FOLLOW"
            for task in tasks.values():
                editable = task["phase"] in ("IDLE", "CANCELLED", "STOPPED", "COMPLETE")
                boost_active = (task["forward_boost_enabled"] and task["phase"] == "FOLLOW"
                                and not task.get("forward_boost_cutoff_reached", False))
                task.update(forward_boost_editable=editable,
                    can_set_forward_boost=editable and active is None,
                    forward_boost_active=boost_active,
                    forward_boost_reason=("disabled" if not task["forward_boost_enabled"] else
                        "active" if boost_active else "inactive_task" if editable else
                        "endpoint_cutoff" if task.get("forward_boost_cutoff_reached") else "waiting_follow"))
                blockers = []
                if not ready:
                    blockers.append(dict(code="localization", message="等待定位稳定"))
                if active or task["phase"] == "PAUSED":
                    blockers.append(dict(code="phase", message="任务运行中或需要先取消暂停任务"))
                task.update(fresh=True, can_start=not blockers, start_blockers=blockers,
                            wall_fresh=ready, wall_valid=ready,
                            center_wall_distance_m=.35 if ready else None)
            if now - last_tracks >= .5:
                last_tracks = now
                self._receive(dict(type="tracks", session_id="mock-stream", tasks=
                    json.loads(json.dumps({k: t["actual_track"] for k, t in tasks.items()}))))
            sequence += 1
            self._receive(dict(type="telemetry", stream=dict(session="mock-stream", sequence=sequence,
                generated_monotonic=now, generated_unix=time.time(), server_queue_ms=0.), state=dict(state="READY", connected=True,
                source="NANO", motion_authorized=False, allow_fifth_thruster=True), control_fresh=True,
                pose=pose if runtime["mode"] != "idle" else None,
                runtime=dict(runtime), tasks=json.loads(json.dumps({k: {field: value for field, value in t.items()
                    if field != "actual_track"} for k, t in tasks.items()})) if runtime["mode"] == "localization" else {}, task_busy=bool(active),
                operation_pending=False, active_tasks=[active] if active else [], active_client="mock",
                can_set_initial_pose=active is None and runtime["mode"] == "localization",
                localization=dict(fresh=True, ready=ready, initial_pose_received=initialized is not None,
                    state="LOCALIZATION_READY" if ready else "LOCALIZATION_CHECK", map_id="mock-map",
                    pose_fresh=ready, scan_fresh=True, map_available=True, stable_sec=1.,
                    stable_elapsed_sec=0. if initialized is None else now-initialized)))
