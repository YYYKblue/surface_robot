"""Area patrol task, shared Cartographer pose and leased F103 output."""
from array import array
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
from functools import partial
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformException, TransformListener

from f1_interfaces.msg import ActuatorCommand
from f1_interfaces.srv import SetLaneSpacing
from f1_localization.localization import yaw_from_quaternion
from f1_localization.map_io import load_occupancy
from f1_pool_cleaning.lease import Lease
from f1_pool_cleaning.ros_support import stamp, stamp_ns
from f1_pool_cleaning.runner import run

from .mission import Mission, Output, Settings
from .planner import WaterGrid, plan

TERMINAL = {"IDLE", "COMPLETE", "CANCELLED", "STOPPED"}
PREFIX = "/area_patrol/"


class PatrolNode(Node):
    def __init__(self, **kwargs):
        super().__init__("area_patrol", **kwargs)
        def param(key, default):
            return self.declare_parameter(key, default, ParameterDescriptor(read_only=True)).value
        self.cfg = Settings(**{f.name:param(f.name, f.default) for f in fields(Settings)}).validate()
        path = param("map", "")
        self.map_id = str(Path(path).expanduser().resolve().with_suffix(""))
        self.live = param("output_enabled", False)
        polygon = self.declare_parameter("work_polygon_xy", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY, dynamic_typing=True, read_only=True)).value
        self.grid = WaterGrid(load_occupancy(path), polygon)
        self.spacing = self.cfg.lane_spacing_m
        self.phase, self.reason, self.pause_kind = "IDLE", "", ""
        self.run_id, self.completed, self.resume_sequence = "", 0, 0
        self.loc, self.pose = {}, None
        self.pose_ns = self.last_now = 0
        self.loc_received = self.pose_received = 0.
        self.identity = None
        self.route = self.mission = self.plan_key = self.future = self.preview_start = None
        self.plan_error, self.plan_id, self.plan_cells = "等待定位后规划", "", 0
        self.preview_at = 0.
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, PREFIX+"status", 5)
        self.plan_pub = self.create_publisher(String, PREFIX+"plan", latched)
        self.path_pub = self.create_publisher(RosPath, PREFIX+"path", latched)
        self.region_pub = self.create_publisher(OccupancyGrid, PREFIX+"region", latched)
        self.track_pub = self.create_publisher(Marker, PREFIX+"actual_track", latched)
        self.track = deque(maxlen=2000)
        self.track_last = None
        self.track_dirty = True
        self.preview = self.create_publisher(ActuatorCommand, PREFIX+"command_preview", 1)
        self.create_subscription(String, param("localization_status_topic", "/f1_localization/localization_status"),
                                 self._localization, latched)
        self.lease = Lease(self, "patrol") if self.live else None
        for action in ("start", "pause", "resume", "stop", "cancel"):
            self.create_service(Trigger, PREFIX+action, partial(self._action, action))
        self.create_service(SetLaneSpacing, PREFIX+"set_lane_spacing", self._spacing)
        self.create_timer(.05, self._tick)
        self.create_timer(.15, self._status)
        self.create_timer(.5, self._publish_track)

    def _localization(self, msg):
        try:
            obj = json.loads(msg.data)
            self.loc = obj if isinstance(obj, dict) else {}
        except ValueError:
            self.loc = {}
        self.loc_received = time.monotonic()
        if self.phase not in TERMINAL and self.pause_kind != "fault" and self._loc_error():
            self._halt(self._loc_error())

    def _lookup(self, parent, child):
        tf = self.tf.lookup_transform(parent, child, Time())
        t, q = tf.transform.translation, tf.transform.rotation
        if not all(math.isfinite(v) for v in (t.x, t.y)):
            raise ValueError("TF 含非法坐标")
        return (t.x, t.y, yaw_from_quaternion(q)), stamp_ns(tf.header.stamp)

    def _epoch(self):
        return self.loc.get("trajectory_id"), self.loc.get("initial_pose_stamp_ns")

    def _loc_error(self):
        now = self.get_clock().now().nanoseconds
        ns = self.loc.get("stamp_ns")
        if (self.loc.get("map_id") != self.map_id or self.loc.get("ready") is not True
                or self.loc.get("state") != "LOCALIZATION_READY" or type(ns) is not int
                or not 0 <= now-ns < 500_000_000 or time.monotonic()-self.loc_received >= .5):
            return "等待当前地图定位就绪"
        if self.phase not in TERMINAL and self.identity != self._epoch():
            return "定位轨迹发生变化"
        return ""

    def _pose_error(self):
        now = self.get_clock().now().nanoseconds
        if self.pose is None or not 0 <= now-self.pose_ns < 300_000_000 or time.monotonic()-self.pose_received >= .3:
            return "船体地图位姿过期"
        return ""

    def _prepare(self):
        if self.phase not in TERMINAL or self.pose is None or self._loc_error():
            return
        xy = self.pose[:2]
        key = (self._epoch(), self.spacing, self.grid.label(xy))
        if self.future is not None:
            if not self.future.done():
                return
            future, self.future = self.future, None
            try:
                route = future.result()
            except ValueError as exc:
                self.route, self.plan_id, self.plan_error = None, "", str(exc)
                return
            if self.plan_key == key:
                self._accept_plan(route)
                return
        if key == self.plan_key:
            if self.grid.cell(*xy) == self.grid.cell(*self.preview_start) or time.monotonic() - self.preview_at < 1.:
                return
        if self.plan_key != key:
            self.route, self.plan_id = None, ""
        self.preview_at = time.monotonic()
        self.plan_key, self.preview_start = key, xy
        self.plan_error = "正在规划巡游航线"
        self.future = self.worker.submit(plan, self.grid, xy, self.spacing)

    def _accept_plan(self, route):
        self.route, self.plan_id, self.plan_error = route, uuid.uuid4().hex, ""
        self.plan_cells = int(route.region.sum())
        self._publish_plan()

    def _publish_plan(self):
        points = [self.route.legs[0].start, *[leg.end for leg in self.route.legs]]
        path = RosPath()
        path.header.frame_id, path.header.stamp = "map", self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.path_pub.publish(path)
        m = self.grid.map
        region = OccupancyGrid(header=path.header)
        region.info.resolution, region.info.width, region.info.height = m.resolution_m, m.width, m.height
        region.info.origin.position.x, region.info.origin.position.y = m.origin_x_m, m.origin_y_m
        region.info.origin.orientation.w = 1.
        cells = np.full(self.grid.free.shape, -1, dtype=np.int8)
        cells[self.grid.free>0] = 50
        cells[self.grid.labels == self.grid.label(points[0])] = 25
        cells[self.route.region] = 0
        region.data = array("b", np.flipud(cells).tobytes())
        self.region_pub.publish(region)
        self.plan_pub.publish(String(data=json.dumps(dict(map_id=self.map_id, plan_id=self.plan_id,
            frame_id="map", points=[[round(x, 4), round(y, 4)] for x, y in points]))))

    def _spacing(self, req, res):
        if self.phase not in TERMINAL or not max(.10, self.grid.res) <= req.spacing_m <= 10.:
            res.success, res.message = False, "仅允许在任务开始前设置有效间距（0.10–10 m，至少一个栅格）"
        else:
            if self.spacing != req.spacing_m:
                self.spacing, self.route, self.plan_id, self.plan_key = req.spacing_m, None, "", None
            res.success, res.message = True, "巡游间距已设置"
        return res

    def _blockers(self, resume=False):
        errors = []
        if (resume and (self.phase != "PAUSED" or self.pause_kind != "user")) or (not resume and self.phase not in TERMINAL):
            errors.append(("phase", "任务阶段不允许此操作"))
        loc, stream = self._loc_error(), self._pose_error()
        if loc:
            errors.append(("localization", loc))
        if stream:
            errors.append(("feedback", stream))
        if self.pose is not None and not self.grid.label(self.pose[:2]):
            errors.append(("route", "船位在已知可航水域外"))
        if self.lease:
            error = self.lease.error(acquiring=True)
            if error:
                errors.append(("bridge", error))
            if not self.lease.acquire.service_is_ready():
                errors.append(("bridge_service", "等待控制租约服务"))
        return [dict(code=c, message=m) for c, m in errors]

    def _begin_motion(self, resume):
        if self.lease:
            return self.lease.begin(resuming=resume)
        return self._lease_ready(resume, self.get_clock().now().nanoseconds)

    def _finish_planning(self):
        if not self.future.done():
            return
        future, self.future = self.future, None
        try:
            self._accept_plan(future.result())
            self.mission = Mission(self.grid, self.route, self.cfg)
            ok, reason = self._begin_motion(False)
            if not ok:
                self._halt(reason)
        except ValueError as exc:
            self._halt(str(exc))

    def _action(self, action, req, res):
        if action in ("start", "resume"):
            resume = action == "resume"
            self._read_pose()
            blockers = self._blockers(resume)
            if blockers:
                res.message = blockers[0]["message"]
                return res
            if resume:
                res.success, res.message = self._begin_motion(True)
                return res
            self.track.clear()
            self.track_last = None
            self.track_dirty = True
            self.run_id, self.completed, self.resume_sequence = uuid.uuid4().hex, 0, 0
            self.identity = self._epoch()
            # Freeze the actual start exactly once. A drifting boat must not
            # keep invalidating the job or be sent back to the preview origin.
            start = self.pose[:2]
            key = (self.identity, self.spacing, self.grid.label(start))
            if self.route and self.future is None and self.plan_key == key and self.route.legs[0].start == start:
                self.mission = Mission(self.grid, self.route, self.cfg)
                res.success, res.message = self._begin_motion(False)
                if not res.success:
                    self._halt(res.message)
            else:
                if self.future:
                    self.future.cancel()
                self.preview_start, self.plan_key = start, key
                self.route, self.mission, self.plan_id = None, None, ""
                self.phase, self.reason, self.pause_kind = "PLANNING", "根据启动时船位规划巡游航线", ""
                self.future = self.worker.submit(plan, self.grid, start, self.spacing)
                res.success, res.message = True, self.reason
        elif action == "pause":
            if self.phase != "PATROLLING":
                res.message = "当前没有执行中的巡游"
                return res
            self._stop("PAUSED", "人工暂停", "user")
            res.success, res.message = True, self.reason
        else:
            self._stop("CANCELLED" if action=="cancel" else "STOPPED", "巡游任务已结束")
            res.success, res.message = True, self.reason
        return res

    def _lease_paused(self):
        return self.phase == "PAUSED" and self.pause_kind == "user"

    def _lease_waiting(self, resuming):
        self.phase, self.reason = "ACQUIRING", "等待巡游控制租约"

    def _lease_ready(self, resuming, now):
        self._read_pose()
        error = self._loc_error() or self._pose_error()
        if error:
            return False, error
        try:
            self.mission.rejoin(self.pose[:2])
        except ValueError as exc:
            return False, str(exc)
        self.phase, self.pause_kind, self.reason = "PATROLLING", "", "正在执行巡游航线"
        self.resume_sequence += int(resuming)
        return True, self.reason

    def _command(self, output):
        now = self.get_clock().now().nanoseconds
        deadline = now + 120_000_000
        if output.percent or output.side:
            deadline = min(deadline, self.pose_ns + 300_000_000,
                           self.loc.get("stamp_ns", 0) + 500_000_000)
        msg = ActuatorCommand(computed_at=stamp(now), valid_until=stamp(deadline),
            direction=getattr(ActuatorCommand, output.direction), speed_percent=output.percent,
            mode=ActuatorCommand.BOTH, thrust_enabled=bool(output.side), thrust_percent=float(output.side))
        self.preview.publish(msg)
        if self.lease:
            self.lease.publish(msg)

    def _stop(self, phase, reason, kind=""):
        if self.phase == "PLANNING":
            if self.future is not None:
                self.future.cancel()
                self.future = None
            self.plan_key = None
        self.track_last = None
        self._command(Output())
        if self.lease:
            self.lease.stop()
        self.phase, self.reason, self.pause_kind = phase, reason, kind

    def _halt(self, reason):
        self._stop("PAUSED", reason, "fault")

    def _read_pose(self):
        try:
            self.pose, ns = self._lookup("map", "base_link")
            if ns != self.pose_ns:
                self.pose_received = time.monotonic()
            self.pose_ns = ns
        except (TransformException, ValueError):
            self.pose = None

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        self._read_pose()
        clock_jump = now < self.last_now
        self.last_now = now
        if self.phase in TERMINAL:
            self._prepare()
            return
        if self.phase == "PAUSED":
            return
        error = ("时钟回退" if clock_jump else self._loc_error() or self._pose_error()
                 or (self.lease.tick_error() if self.lease else ""))
        if error:
            self._halt(error)
            return
        if self.phase == "PLANNING":
            self._finish_planning()
            return
        if self.phase == "PATROLLING":
            try:
                output = self.mission.update(self.pose, time.monotonic())
            except ValueError as exc:
                self._halt(str(exc))
                return
            position = self.pose[:2]
            if self.track_last is None:
                self.track_last = position
            elif math.dist(position, self.track_last) >= .03:
                self.track.append((self.track_last, position))
                self.track_last = position
                self.track_dirty = True
            if self.mission.index == len(self.mission.legs):
                self.completed = now
                self._stop("COMPLETE", "巡游航段执行完成")
            else:
                self._command(output)

    def _publish_track(self):
        if not self.track_dirty:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "area_patrol"
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.
        marker.scale.x = .025
        marker.color.g, marker.color.a = .7, 1.
        marker.points = [Point(x=x, y=y) for segment in self.track for x, y in segment]
        self.track_pub.publish(marker)
        self.track_dirty = False

    def _status(self):
        blockers, resume = self._blockers(), self._blockers(True)
        m = self.mission
        preview_lanes = sum(leg.kind == "lane" for leg in self.route.legs) if self.route else 0
        result_route = m.route if m else self.route
        status = dict(task_type="area_patrol", map_id=self.map_id, stamp_ns=self.get_clock().now().nanoseconds,
            phase=self.phase, reason=self.reason, run_id=self.run_id, output_enabled=self.live,
            can_start=not blockers, start_blockers=blockers, can_resume=not resume, resume_blockers=resume,
            resume_sequence=self.resume_sequence, pause_kind=self.pause_kind, completed_at_ns=self.completed,
            plan_id=self.plan_id, lane_spacing_m=self.spacing, completion_scope="area_patrol",
            reachable_m2=(m.region_cells if m else self.plan_cells if self.route else 0)*self.grid.res**2,
            lanes_done=m.finished_lanes if m else 0,
            lane_count=m.total_lanes if m else preview_lanes, preview_lane_count=preview_lanes,
            area_coverage=m.observed_cells/m.region_cells if m else 0.,
            skipped_m2=result_route.skipped_m2 if result_route else 0.,
            disconnected_m2=result_route.disconnected_m2 if result_route else 0.)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def destroy_node(self):
        self._stop("STOPPED", "节点关闭")
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    run(PatrolNode, args)
