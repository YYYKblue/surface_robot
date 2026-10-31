"""Executor-thread state machine for app-managed SLAM and frozen-map operation.

Disk work / process waits run on one worker. ROS futures are polled by the normal
executor; no nested spin and no blocking waits in the manual-control watchdog.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import fcntl
import math
import time
import uuid

from .map_store import MapStore
from .managed_process import ManagedProcess, export_map
from .tracks import TaskTrack

READ_ACTIONS = {"list", "preview"}


class RuntimeControl:
    def _init_runtime(self):
        defaults = dict(managed_runtime=False, workspace_root="", map_timezone="Asia/Shanghai",
                        scan_topic="/pavo2_scan", output_enabled=False, publish_lidar_tf=True,
                        mapping_resolution=.05, rectangle_config="", l_config="", localization_config="")
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.runtime_enabled = bool(self.get_parameter("managed_runtime").value)
        self.runtime_pending = None
        self.runtime_process = None
        self.runtime_pool = None
        self.runtime_lock = None
        self.runtime_epoch_ns = 0
        self.runtime_accept_map = False
        self.runtime_scan_received = 0.
        self.runtime_scan_valid = False
        self.runtime_last_scan_ns = 0
        self.runtime = dict(mode="idle", phase="IDLE", session_id="", current_map="",
                            message="等待建图或选择地图", last_saved=None, last_operation=None)
        if not self.runtime_enabled:
            return
        from cartographer_ros_msgs.srv import WriteState
        self.write_state_type = WriteState
        self.write_state_client = self.create_client(WriteState, "/write_state")
        self.map_store = MapStore(self.get_parameter("workspace_root").value,
                                  self.get_parameter("map_timezone").value)
        self.runtime_log = self.map_store.root.parent / "log" / "operator_runtime"
        self.runtime_log.mkdir(parents=True, exist_ok=True)
        self.runtime_lock = open(self.map_store.root / ".operator-runtime.lock", "a")
        try:
            fcntl.flock(self.runtime_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.runtime_lock.close()
            raise RuntimeError("该工作空间已有运行管理实例") from exc
        self.runtime_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="operator-runtime")
        resolution = float(self.get_parameter("mapping_resolution").value)
        if not math.isfinite(resolution) or not .01 <= resolution <= .5:
            raise ValueError("mapping_resolution 必须在 0.01 至 0.5 米之间")

    def _runtime_busy(self):
        pending = getattr(self, "runtime_pending", None)
        return bool(pending and pending["packet"]["action"] not in READ_ACTIONS)

    def _runtime_task_allowed(self):
        return (not getattr(self, "runtime_enabled", False) or
                (self.runtime["mode"] == "localization" and self.runtime["phase"] == "LOCALIZATION"
                 and not self._runtime_busy()))

    def _runtime_scan(self, message):
        if not self.runtime_enabled:
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        previous = self.runtime_last_scan_ns
        self.runtime_last_scan_ns = stamp
        self.runtime_scan_received = time.monotonic()
        age = self.get_clock().now().nanoseconds - stamp
        self.runtime_scan_valid = (
            message.header.frame_id == "pavo2s_frame" and len(message.ranges) > 1
            and all(math.isfinite(x) for x in (message.angle_min, message.angle_max,
                                               message.angle_increment, message.range_min, message.range_max))
            and message.angle_increment != 0 and 0 <= message.range_min < message.range_max
            and abs(message.angle_min + (len(message.ranges)-1)*message.angle_increment - message.angle_max)
                <= max(.0001, abs(message.angle_increment)*1.5)
            and stamp > previous and 0 <= age < 500_000_000
            and any(math.isfinite(x) and message.range_min <= x <= message.range_max for x in message.ranges))

    def _runtime_scan_fresh(self):
        return self.runtime_scan_valid and time.monotonic() - self.runtime_scan_received < .5

    def _runtime_reset(self):
        self.runtime_epoch_ns = self.get_clock().now().nanoseconds
        self.runtime_accept_map = False
        self.map_data = self.map_signature = self.pose = None
        self.map_revision += 1
        self.tf_buffer.clear()
        self.tasks, self.task_received = {}, {}
        self.localization, self.localization_received = {}, 0.
        self.initial_pose_stamp = None
        self.task_tracks = {kind: TaskTrack() for kind in self.task_tracks}
        # Callers have confirmed no active/uncertain task before a mode switch.
        self.hub.broadcast(dict(type="map", map=None, session_id=self.runtime["session_id"]))

    def _runtime_conflicts(self):
        names = self.get_node_names()
        prefixes = ("cartographer", "pool_localization", "pool_controller", "l_controller", "pool_perception", "l_perception")
        conflicts = [name for name in names if name.startswith(prefixes)]
        if self.get_parameter("publish_lidar_tf").value and "base_to_lidar_planar" in names:
            conflicts.append("base_to_lidar_planar")
        if self.count_publishers("/map"):
            conflicts.append("/map 已有发布者")
        return conflicts

    def _runtime_request(self, client, packet):
        if not self.runtime_enabled:
            self._rpc_reply(client, packet, False, "请使用 operator_system.launch.py 启用建图与选图")
            return
        previous = self.operation_replies.get((client, packet["request_id"]))
        if previous:
            if previous[0] == packet:
                self.hub.send(client, previous[1])
            else:
                self.hub.send(client, dict(type="reply", op="mapping", request_id=packet["request_id"],
                                          success=False, message="request_id 已用于其他操作"))
            return
        pending = self.runtime_pending
        if pending:
            duplicate = pending["client"] == client and pending["packet"] == packet
            self.hub.send(client, dict(type="reply", op="mapping", request_id=packet["request_id"],
                success=duplicate, pending=duplicate, message="已有地图操作处理中"))
            return
        action = packet["action"]
        try:
            if action not in READ_ACTIONS:
                if self.command_pending or self.task_locks or self.uncertain_tasks:
                    raise ValueError("请先停止清洗任务并等待停止确认")
                if self.runtime["mode"] == "localization":
                    for kind in self.runtime.get("task_kinds", []):
                        if not self._source_fresh(self.tasks.get(kind, {}), self.task_received.get(kind, 0)):
                            raise ValueError("清洗控制器状态未确认，禁止切换模式")
                if action in ("save", "save_end", "discard") and self.runtime["mode"] != "mapping":
                    raise ValueError("当前未处于建图模式")
                if action in ("start", "load") and self.runtime["mode"] == "mapping":
                    raise ValueError("请先保存并结束，或明确放弃当前建图")
                if action == "start" and not self._runtime_scan_fresh():
                    raise ValueError("雷达无新鲜有效扫描，请检查雷达状态")
                if action == "discard" and not packet.get("confirm"):
                    raise ValueError("放弃建图需要明确确认")
                if action in ("save", "save_end"):
                    MapStore.name(packet.get("name", ""))
                    if not self.map_data or not self.pose or not self._runtime_scan_fresh():
                        raise ValueError("当前地图、位姿或雷达未就绪，暂不能保存新快照")
                self._stop_manual("mapping_" + action)
            self.runtime_pending = dict(client=client, packet=packet, stage="handover",
                                        since=time.monotonic())
            if action in READ_ACTIONS:
                self._runtime_begin()
        except Exception as exc:
            self._rpc_reply(client, packet, False, str(exc))

    def _runtime_work(self, function, stage):
        self.runtime_pending.update(stage=stage, future=self.runtime_pool.submit(function))

    def _runtime_begin(self):
        packet = self.runtime_pending["packet"]
        action = packet["action"]
        if action == "list":
            def listing():
                entries = self.map_store.catalog()
                for transaction in self.map_store.recovered():
                    entries.append(dict(name=transaction["name"], valid=False, recoverable=True,
                        saved_at="", reason="快照已保留，可重试栅格导出"))
                return entries
            self._runtime_work(listing, "list")
        elif action == "preview":
            self._runtime_work(lambda: self.map_store.preview(packet["name"]), "preview")
        elif action == "load":
            def prepare():
                from f1_pool_cleaning.map_profile import profile
                self.map_store.validate(packet["name"])
                derived = profile(str(self.map_store.path(packet["name"])))
                kinds, errors = [], {}
                for kind, geometry, key in (("rectangle", derived.rectangle, "rectangle"), ("l", derived.route, "route")):
                    if geometry is not None:
                        kinds.append(kind)
                    else:
                        errors[kind] = derived.errors.get(key, "地图不支持该任务")
                return kinds, errors
            self._runtime_work(prepare, "prepared")
        elif action == "start":
            self._runtime_switch()
        elif action == "discard":
            self._runtime_stop("discarded")
        elif action in ("save", "save_end"):
            self.runtime.update(phase="SAVING", message="准备保存快照")
            self._runtime_work(lambda: self.map_store.reserve(packet.get("name", "")), "reserved")
        elif action == "retry_export":
            def recovered():
                candidates = [t for t in self.map_store.recovered() if t["name"] == packet["name"]]
                if len(candidates) != 1:
                    raise ValueError("没有唯一可重试快照")
                return candidates[0]
            self._runtime_work(recovered, "retry")

    def _runtime_stop(self, stage):
        self.runtime_accept_map = False
        self.runtime.update(phase="STOPPING", message="正在退出当前模式")
        process = self.runtime_process
        self._runtime_work(lambda: process.stop() if process else None, stage)

    def _runtime_switch(self):
        if self.command_pending or self.task_locks or self.uncertain_tasks:
            raise ValueError("准备地图期间任务状态改变，请先停止任务")
        if self.runtime_process:
            self._runtime_stop("stopped")
        else:
            self.runtime_pending.update(stage="stopped", future=None)

    def _runtime_spawn(self):
        packet = self.runtime_pending["packet"]
        mapping = packet["action"] == "start"
        self.runtime.update(mode="mapping" if mapping else "localization", phase="STARTING",
            session_id=uuid.uuid4().hex, current_map="" if mapping else packet["name"],
            message="启动建图" if mapping else "加载定位地图", task_kinds=self.runtime_pending.get("task_kinds", []))
        self._runtime_reset()
        arguments = ["ros2", "launch", "f1_operator_gateway", "operator_mode.launch.py",
                     "mode:=" + ("mapping" if mapping else "localization")]
        for key in ("scan_topic", "publish_lidar_tf", "output_enabled", "rectangle_config", "l_config", "localization_config"):
            value = self.get_parameter(key).value
            if value != "":
                arguments.append(f"{key}:={str(value).lower() if isinstance(value, bool) else value}")
        arguments.append("resolution:=" + str(self.get_parameter("mapping_resolution").value))
        if not mapping:
            arguments.append("map:=" + str(self.map_store.path(packet["name"])))
        self.runtime_accept_map = True
        log = self.runtime_log / (self.runtime["session_id"] + ".log")
        self.runtime["log_path"] = str(log)
        self._runtime_work(lambda: ManagedProcess(arguments, log), "spawned")

    def _runtime_export(self, transaction):
        self.runtime_pending["transaction"] = transaction
        self.runtime.update(phase="SAVING", message="正在导出并校验栅格地图")
        def export():
            export_map(transaction["stem"], self.get_parameter("mapping_resolution").value,
                       Path(transaction["directory"]) / "export.log")
            return self.map_store.publish(transaction)
        self._runtime_work(export, "exported")

    def _runtime_finish(self, success, message, **fields):
        pending = self.runtime_pending
        if not pending:
            return
        self.runtime_pending = None
        action = pending["packet"]["action"]
        if action not in READ_ACTIONS:
            self.runtime["last_operation"] = dict(request_id=pending["packet"]["request_id"],
                action=action, success=success, message=message)
            self.runtime["message"] = message
        if self.runtime["phase"] == "SAVING":
            self.runtime["phase"] = {"mapping": "MAPPING", "localization": "LOCALIZATION"}.get(self.runtime["mode"], "IDLE")
        self._rpc_reply(pending["client"], pending["packet"], success, message, **fields)

    def _runtime_tick(self):
        if not getattr(self, "runtime_enabled", False):
            return
        pending = self.runtime_pending
        if not pending:
            if self.runtime_process and not self.runtime_process.alive() and self.runtime["mode"] != "error":
                self._stop_manual("runtime_process_exited")
                self.runtime_accept_map = False
                self.runtime.update(mode="error", phase="ERROR", message="地图进程退出，请检查日志后重新建图或选图")
                self._runtime_reset()
            return
        stage = pending["stage"]
        try:
            if stage == "handover":
                if self.active_client != pending["client"]:
                    self._runtime_finish(False, "操作端已断开，未执行地图操作")
                    return
                if time.monotonic() - pending["since"] > 3:
                    self._runtime_finish(False, "控制权交接超时，未执行地图操作")
                    return
                if self._manual_acquires or any(not f.done() for f in self.manual_releases):
                    return
                if self.command_pending or self.task_locks or self.uncertain_tasks:
                    self._runtime_finish(False, "清洗任务状态改变，请先停止任务")
                    return
                state = self.control_state
                if self.get_parameter("output_enabled").value and time.monotonic() - self.control_received >= .3:
                    return
                if state.get("motion_authorized"):
                    return
                self._runtime_begin()
                return
            if stage == "stopped":
                future = pending.get("future")
                if future:
                    if not future.done():
                        return
                    future.result()
                self.runtime_process = None
                # Wait for DDS discovery to remove old owners, rather than reusing their services.
                pending.update(stage="clear_graph", since=time.monotonic())
                self._runtime_reset()
                return
            if stage == "clear_graph":
                if self._runtime_conflicts():
                    if time.monotonic() - pending["since"] > 8:
                        raise RuntimeError("仍有建图、定位或清洗节点运行，未启动重复实例")
                    return
                self._runtime_spawn()
                return
            if stage == "ready":
                if not self.runtime_process.alive():
                    raise RuntimeError("地图进程启动失败，日志：" + self.runtime["log_path"])
                mapping = self.runtime["mode"] == "mapping"
                ready = self.write_state_client.service_is_ready() if mapping else (
                    self._source_fresh(self.localization, self.localization_received)
                    and self.localization.get("map_id") == str(self.map_store.path(self.runtime["current_map"]).with_suffix(""))
                    and self.map_data is not None
                    and all(self._source_fresh(self.tasks.get(k, {}), self.task_received.get(k, 0))
                            for k in self.runtime["task_kinds"]))
                if ready:
                    self.runtime["phase"] = "MAPPING" if mapping else "LOCALIZATION"
                    self._runtime_finish(True, "建图已启动，请缓慢操船探索" if mapping else "地图已加载，请设置初始位姿")
                elif time.monotonic() - pending["since"] > 30:
                    raise RuntimeError("地图进程就绪超时，日志：" + self.runtime["log_path"])
                return
            if stage == "writing":
                future = pending["future"]
                if self.runtime_process and not self.runtime_process.alive():
                    self.runtime.update(mode="error", phase="ERROR")
                    self._runtime_reset()
                    raise RuntimeError("建图进程在保存期间退出；不能确认快照完整性")
                if not future.done():
                    if time.monotonic() - pending["since"] > 120:
                        self.runtime["message"] = "快照写入超时，结果待确认；保持会话，禁止重复写入"
                    return
                response = future.result()
                if response.status.code != 0:
                    raise RuntimeError("Cartographer 保存失败：" + response.status.message)
                transaction = pending["transaction"]
                path = Path(transaction["stem"] + ".pbstream")
                if not path.is_file() or path.stat().st_size == 0:
                    raise RuntimeError("Cartographer 未产生有效 pbstream")
                self._runtime_export(transaction)
                return
            future = pending.get("future")
            if future is None or not future.done():
                return
            value = future.result()
            if stage == "list":
                self._runtime_finish(True, "地图列表已更新", maps=value, maps_directory=str(self.map_store.root))
            elif stage == "preview":
                self._runtime_finish(True, "地图预览已加载", preview=value, name=pending["packet"]["name"])
            elif stage == "prepared":
                pending["task_kinds"], self.task_unavailable_reasons = value
                self._runtime_switch()
            elif stage == "spawned":
                self.runtime_process = value
                pending.update(stage="ready", since=time.monotonic())
            elif stage == "reserved":
                pending["transaction"] = value
                if not self.write_state_client.service_is_ready():
                    raise RuntimeError("Cartographer 保存服务不可用")
                request = self.write_state_type.Request(filename=value["stem"] + ".pbstream", include_unfinished_submaps=True)
                pending.update(stage="writing", since=time.monotonic(), future=self.write_state_client.call_async(request))
                self.runtime["message"] = "正在写入 Cartographer 快照"
            elif stage == "retry":
                self._runtime_export(value)
            elif stage == "exported":
                self.runtime["last_saved"] = value
                if pending["packet"]["action"] == "save_end":
                    self._runtime_stop("saved_end")
                else:
                    self._runtime_finish(True, "地图已保存：" + value["name"], saved=value)
            elif stage in ("discarded", "saved_end"):
                self.runtime_process = None
                self.runtime.update(mode="idle", phase="IDLE", current_map="", task_kinds=[])
                self._runtime_reset()
                self._runtime_finish(True, "地图已保存并结束建图" if stage == "saved_end" else "已放弃当前建图；此前保存的地图保留",
                                     saved=self.runtime["last_saved"] if stage == "saved_end" else None)
        except Exception as exc:
            message = str(exc)
            if pending.get("transaction"):
                message += "；快照目录：" + pending["transaction"]["directory"]
            if self.runtime["phase"] in ("STARTING", "STOPPING") or stage in ("clear_graph", "ready"):
                self.runtime.update(mode="error", phase="ERROR")
                self.runtime_accept_map = False
                self._runtime_reset()
            self._runtime_finish(False, message)

    def _runtime_view(self):
        if not getattr(self, "runtime_enabled", False):
            return {}
        value = dict(self.runtime, busy=self._runtime_busy(), scan_fresh=self._runtime_scan_fresh(),
                     map_available=self.map_data is not None, pose_fresh=self.pose is not None,
                     maps_directory=str(self.map_store.root))
        value["target_map"] = value["current_map"]
        if value["mode"] != "localization" or value["phase"] in ("STARTING", "STOPPING", "ERROR"):
            value["current_map"] = ""
        pending = self.runtime_pending
        value["operation"] = dict(action=pending["packet"]["action"], request_id=pending["packet"]["request_id"],
                                  stage=pending["stage"]) if pending else None
        return value

    def _close_runtime(self):
        if getattr(self, "runtime_pool", None):
            # A launched process may still be returning from the worker.
            self.runtime_pool.shutdown(wait=True)
            pending = self.runtime_pending
            if pending and pending["stage"] == "spawned":
                try:
                    self.runtime_process = pending["future"].result()
                except Exception:
                    pass
            if self.runtime_process:
                self.runtime_process.stop()
            self.runtime_lock.close()
