"""Task/localization RPCs and fail-closed state aggregation for the operator gateway."""
import json
import math
import time
from collections import OrderedDict

from f1_interfaces.srv import SetInitialPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .tracks import TaskTrack

TASK_PREFIXES = {"rectangle": "/pool_cleaning", "l": "/l_cleaning"}
TERMINAL = {"IDLE", "CANCELLED", "STOPPED", "COMPLETE", "PAUSED"}
MAX_AGE = .6


class TaskControl:
    def _runtime_busy(self):
        return False

    def _runtime_task_allowed(self):
        return True

    def _init_tasks(self):
        self.declare_parameter("localization_prefix", "/pool_cleaning")
        self.declare_parameter("task_unavailable_reasons", "{}")
        prefix = self.get_parameter("localization_prefix").value.rstrip("/")
        if not prefix.startswith("/") or not prefix.strip("/"):
            raise ValueError("localization_prefix must be an absolute ROS namespace")
        self.task_unavailable_reasons = json.loads(self.get_parameter("task_unavailable_reasons").value)
        self.task_clients = {
            kind: {action: self.create_client(Trigger, f"{namespace}/{action}")
                   for action in ("start", "pause", "stop", "cancel")}
            for kind, namespace in TASK_PREFIXES.items()}
        self.tasks, self.task_received = {}, {}
        self.task_tracks = {kind: TaskTrack() for kind in TASK_PREFIXES}
        self.localization, self.localization_received = {}, 0.
        self.command_pending = None
        self.operation_replies = OrderedDict()
        self.task_locks = set()
        self.uncertain_tasks = set()
        self.initial_pose_stamp = None
        self.initial_pose_client = self.create_client(SetInitialPose, prefix + "/set_initial_pose")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, prefix + "/localization_status", self._localization, qos)
        for kind, namespace in TASK_PREFIXES.items():
            self.create_subscription(String, namespace + "/status",
                                     lambda msg, k=kind: self._task_status(k, msg), 5)

    @staticmethod
    def _json_object(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (ValueError, TypeError):
            return {}

    def _task_status(self, kind, message):
        value = self._json_object(message)
        if getattr(self, "runtime_enabled", False):
            expected = self.runtime.get("current_map")
            if (not self.runtime_accept_map or self.runtime["mode"] != "localization" or not expected
                    or value.get("map_id") != str(self.map_store.path(expected).with_suffix(""))
                    or type(value.get("stamp_ns")) is not int or value["stamp_ns"] < self.runtime_epoch_ns):
                return
        self.tasks[kind], self.task_received[kind] = value, time.monotonic()
        # A stale/malformed terminal packet must never unlock a running task.
        if value.get("phase") not in TERMINAL:
            self.task_locks.add(kind)
        elif (kind not in self.uncertain_tasks
              and self._source_fresh(value, self.task_received[kind])
              and not (self.command_pending and self.command_pending["packet"].get("action") == "start"
                       and self.command_pending["packet"].get("task_type") == kind
                       and value.get("run_id") == self.command_pending["old_run"])):
            self.task_locks.discard(kind)

    def _localization(self, message):
        value = self._json_object(message)
        if getattr(self, "runtime_enabled", False):
            expected = self.runtime.get("current_map")
            if (not self.runtime_accept_map or self.runtime["mode"] != "localization" or not expected
                    or value.get("map_id") != str(self.map_store.path(expected).with_suffix(""))
                    or type(value.get("stamp_ns")) is not int or value["stamp_ns"] < self.runtime_epoch_ns):
                return
        self.localization = value
        self.localization_received = time.monotonic()
        if (self.initial_pose_stamp is not None
                and self.localization.get("initial_pose_stamp_ns", 0) >= self.initial_pose_stamp):
            self.initial_pose_stamp = None

    def _source_fresh(self, value, received):
        stamp = value.get("stamp_ns")
        return (bool(received) and time.monotonic() - received < MAX_AGE
                and type(stamp) is int
                and 0 <= self.get_clock().now().nanoseconds - stamp < int(MAX_AGE * 1e9))

    def _localization_view(self):
        value = dict(self.localization)
        value["fresh"] = self._source_fresh(value, self.localization_received)
        value["ready"] = (value["fresh"] and value.get("ready") is True
                          and value.get("state") == "LOCALIZATION_READY"
                          and self.initial_pose_stamp is None)
        return value

    def _tasks_busy(self):
        return bool(self.command_pending or self.task_locks or self._runtime_busy())

    def _task_view(self, kind, *, for_start=False):
        value = dict(self.tasks.get(kind, {}))
        fresh = self._source_fresh(value, self.task_received.get(kind, 0))
        blockers = []
        def add(code, message):
            blockers.append(dict(code=code, message=message))
        if not self._runtime_task_allowed():
            add("runtime", "请先使用地图并完成定位，地图模式操作期间不能启动清洗")
        if not fresh:
            add("task_stale", self.task_unavailable_reasons.get(kind) or "任务未启动或状态已过期")
        if not self.task_clients[kind]["start"].service_is_ready():
            add("task_service", "清洗任务服务不可用")
        loc = self._localization_view()
        if not loc["ready"]:
            add("localization", "等待定位稳定" if loc["fresh"] else "定位状态未收到或已过期")
        if not value.get("map_id") or value.get("map_id") != loc.get("map_id"):
            add("map", "任务与定位地图不一致或尚未确认")
        if value.get("task_type") != kind:
            add("task_type", "控制器类型未确认，请升级或检查接口重映射")
        for item in value.get("start_blockers", []):
            # The gateway can release its own manual lease as part of start.
            # No other owner or hardware fault may be bypassed.
            if (item.get("code") == "bridge" and self.token
                    and self.control_state.get("owner") == "operator_app"
                    and self._fresh_ready() and self.control_state.get("allow_fifth_thruster") is True):
                continue
            blockers.append(item)
        if "can_start" not in value:
            add("schema", "控制器尚未提供启动检查，请升级清洗节点")
        elif value["can_start"] is not True and not value.get("start_blockers"):
            add("preflight", "控制器未确认可以启动")
        if self.task_locks:
            add("busy", "已有清洗任务运行，或其最终状态尚未确认")
        if self.command_pending and not for_start:
            add("pending", "操作处理中")
        value.update(task_type=kind, fresh=fresh, can_start=not blockers, start_blockers=blockers)
        return value

    def _rpc_reply(self, client_id, packet, success, message, **fields):
        result = dict(type="reply", op=packet["op"], request_id=packet.get("request_id", ""),
                      success=success, message=message, **fields)
        for name in ("action", "task_type"):
            if name in packet:
                result[name] = packet[name]
        key = (client_id, packet.get("request_id"))
        if key[1]:
            self.operation_replies[key] = (dict(packet), result)
            while len(self.operation_replies) > 128:
                self.operation_replies.popitem(last=False)
        self.hub.send(client_id, result)

    def _operation_request(self, client_id, packet):
        if not self._runtime_task_allowed() and (packet["op"] == "initial_pose" or packet.get("action") == "start"):
            self._rpc_reply(client_id, packet, False, "当前模式不能设置定位或启动清洗")
            return
        key = (client_id, packet.get("request_id"))
        previous = self.operation_replies.get(key) if key[1] else None
        if previous:
            if previous[0] == packet:
                self.hub.send(client_id, previous[1])
            else:
                self.hub.send(client_id, dict(type="error", message="request_id 已用于其他操作"))
            return
        pending = self.command_pending
        if pending and pending["client"] == client_id and key[1] and pending["packet"].get("request_id") == key[1]:
            self.hub.send(client_id, dict(type="reply", op=packet["op"], request_id=key[1],
                                         success=True, pending=True, message="操作处理中，请等待状态确认"))
            return
        if pending:
            # A stop arriving during start is applied AFTER its asynchronous
            # reply, so a late start callback cannot undo the user's stop.
            if packet["op"] == "task" and packet["action"] in ("pause", "stop", "cancel"):
                if pending["packet"].get("action") == "start" and pending["packet"].get("task_type") == packet["task_type"]:
                    if pending.get("stop_after"):
                        self._rpc_reply(client_id, packet, False, "停止请求处理中")
                    elif pending["stage"] == "handover":
                        self.command_pending = None
                        self._rpc_reply(pending["client"], pending["packet"], False, "启动已取消")
                        self._rpc_reply(client_id, packet, True, "任务尚未启动，已取消启动")
                    else:
                        pending["stop_after"] = (client_id, packet)
                        if pending["stage"] == "status":
                            self._finish_operation(pending, True, "启动请求已接受，正在停止任务")
                    return
            self._rpc_reply(client_id, packet, False, "已有操作处理中")
            return
        if packet["op"] == "initial_pose":
            error = self._initial_pose_error(packet)
        elif packet["action"] == "start":
            error = "; ".join(item["message"] for item in self._task_view(packet["task_type"])["start_blockers"])
        else:
            # Route cancellation to the requested actual controller even if
            # its telemetry is stale; discovery/service response is authoritative.
            error = "" if self.task_clients[packet["task_type"]][packet["action"]].service_is_ready() else "清洗任务服务不可用"
        if error:
            self._rpc_reply(client_id, packet, False, error)
            return
        self._stop_manual("operator_" + packet["op"])
        self.command_pending = dict(client=client_id, packet=packet, stage="handover", since=time.monotonic(),
                                    old_run=self.tasks.get(packet.get("task_type"), {}).get("run_id"))
        self._task_tick()

    def _initial_pose_error(self, packet):
        if self.task_locks:
            return "请先暂停或取消正在运行的清洗任务"
        if not self.initial_pose_client.service_is_ready():
            return "初始位姿服务不可用"
        if not self.map_data or packet["map_revision"] != self.map_revision:
            return "地图已变化或尚未加载，请重新点选初始位姿"
        if not self._source_fresh(self.localization, self.localization_received):
            return "定位管理器状态已过期"
        if self.map_data.get("frame_id") != "map":
            return "初始位姿仅支持 map 坐标系"
        origin = self.map_data["origin"]
        dx, dy = packet["x"] - origin["x"], packet["y"] - origin["y"]
        c, s = math.cos(origin["yaw"]), math.sin(origin["yaw"])
        x, y = c*dx+s*dy, -s*dx+c*dy
        if not (0 <= x < self.map_data["width"]*self.map_data["resolution"]
                and 0 <= y < self.map_data["height"]*self.map_data["resolution"]):
            return "初始位置超出当前地图"
        return ""

    def _task_tick(self):
        pending = self.command_pending
        if pending is None:
            return
        elapsed = time.monotonic() - pending["since"]
        if elapsed > 3 and not pending.get("warned"):
            pending["warned"] = True
            self.hub.send(pending["client"], dict(type="reply", op=pending["packet"]["op"],
                request_id=pending["packet"].get("request_id", ""), success=False, pending=True,
                message="操作超时，结果尚未确认；不会自动重发"))
        if pending["stage"] == "status":
            kind = pending["packet"]["task_type"]
            task = self.tasks.get(kind, {})
            if (self._source_fresh(task, self.task_received.get(kind, 0))
                    and task.get("run_id") and task["run_id"] != pending["old_run"]):
                self._finish_operation(pending, True, "启动请求已确认，请查看任务阶段")
            return
        if pending["stage"] != "handover":
            return
        if elapsed > 3:
            self._finish_operation(pending, False, "控制权交接超时，未发送操作")
            return
        if self._manual_acquires or any(not future.done() for future in self.manual_releases):
            return
        packet = pending["packet"]
        if packet["op"] == "task":
            kind, action = packet["task_type"], packet["action"]
            if action == "start":
                view = self._task_view(kind, for_start=True)
                blockers = view["start_blockers"]
                # Release ACK may precede the next bridge/controller status.
                if blockers and all(item["code"] == "bridge" for item in blockers):
                    return
                if blockers:
                    self._finish_operation(pending, False, "; ".join(b["message"] for b in blockers))
                    return
            client, request = self.task_clients[kind][action], Trigger.Request()
        else:
            error = self._initial_pose_error(packet)
            if error:
                self._finish_operation(pending, False, error)
                return
            state = self.control_state
            if state.get("motion_authorized") and state.get("owner") == "operator_app":
                return
            if state.get("motion_authorized"):
                self._finish_operation(pending, False, "控制权仍被占用，请先停止运动")
                return
            client, request = self.initial_pose_client, SetInitialPose.Request()
            request.pose.header.frame_id = "map"
            request.pose.header.stamp = self.get_clock().now().to_msg()
            pose = request.pose.pose.pose
            pose.position.x, pose.position.y = packet["x"], packet["y"]
            pose.orientation.z, pose.orientation.w = math.sin(packet["yaw"]/2), math.cos(packet["yaw"]/2)
            request.pose.pose.covariance[0] = request.pose.pose.covariance[7] = .25
            request.pose.pose.covariance[35] = math.radians(15)**2
            self.initial_pose_stamp = request.pose.header.stamp.sec*10**9 + request.pose.header.stamp.nanosec
        pending["stage"] = "service"
        if packet.get("action") == "start":
            self.task_locks.add(packet["task_type"])
        try:
            future = client.call_async(request)
            future.add_done_callback(lambda done: self._operation_done(pending, done))
        except Exception as exc:
            if pending["packet"].get("action") == "start":
                self.uncertain_tasks.add(pending["packet"]["task_type"])
            self._finish_operation(pending, False, str(exc))

    def _operation_done(self, pending, future):
        if self.command_pending is not pending:
            return
        try:
            result = future.result()
        except Exception as exc:
            if pending["packet"].get("action") == "start":
                self.uncertain_tasks.add(pending["packet"]["task_type"])
            self._finish_operation(pending, False, str(exc))
            return
        kind = pending["packet"].get("task_type")
        action = pending["packet"].get("action")
        if kind and ((action == "start" and not result.success)
                     or (action in ("pause", "stop", "cancel") and result.success)):
            self.uncertain_tasks.discard(kind)
            # A response confirms the service ran; an observed active phase
            # remains locked until the next fresh terminal status.
            if self.tasks.get(kind, {}).get("phase") in TERMINAL:
                self.task_locks.discard(kind)
        if result.success and pending["packet"].get("action") == "start" and not pending.get("stop_after"):
            pending["stage"] = "status"
            return
        self._finish_operation(pending, bool(result.success), result.message)

    def _finish_operation(self, pending, success, message):
        if self.command_pending is not pending:
            return
        self.command_pending = None
        self._rpc_reply(pending["client"], pending["packet"], success, message)
        if pending.get("stop_after"):
            self._operation_request(*pending["stop_after"])

    def _task_telemetry(self):
        loc = self._localization_view()
        tasks = {kind: self._task_view(kind) for kind in TASK_PREFIXES}
        for kind, task in tasks.items():
            history = self.task_tracks[kind]
            # Ignore stale identities: an old packet must not erase a new run.
            if task["fresh"]:
                history.update(task.get("map_id"), task.get("run_id"),
                    getattr(self, "pose", None),
                    recording=(task.get("phase") not in TERMINAL and loc["ready"]
                               and loc.get("map_id") == task.get("map_id")),
                    epoch=(loc.get("trajectory_id"), loc.get("initial_pose_stamp_ns")))
            else:
                history.last_seen = history.last_recorded = None
            task["actual_track"] = history.snapshot()
        return dict(tasks=tasks,
                    localization=loc, task_busy=self._tasks_busy(),
                    active_tasks=sorted(self.task_locks), operation_pending=bool(self.command_pending),
                    can_set_initial_pose=(not self._tasks_busy() and self.map_data is not None
                        and self._runtime_task_allowed() and loc["fresh"] and self.initial_pose_client.service_is_ready()),
                    operator_available=self.active_client is None)
