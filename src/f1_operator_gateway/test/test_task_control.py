"""Exercise RPC sequencing and stale-state interlocks without moving hardware."""
import json
import math
import time
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import String

from operator_gateway.task_control import TaskControl


class Client:
    def __init__(self):
        self.calls = []
        self.available = True

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future


class Gateway(TaskControl):
    def __init__(self):
        self.params = {}
        self.sent = []
        self.clients = {}
        self.hub = SimpleNamespace(send=lambda client, packet: self.sent.append(packet))
        self._init_tasks()
        self.token = ""
        self.control_state = {}
        self._manual_acquires = 0
        self.manual_releases = []
        self.active_client = "app"
        self.map_revision = 1
        self.map_data = dict(frame_id="map", revision=1, width=100, height=100, resolution=.1,
                             origin=dict(x=0., y=0., yaw=0.))
        self.ready()

    def declare_parameter(self, name, value):
        self.params[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def create_client(self, _, path):
        self.clients[path] = Client()
        return self.clients[path]

    def create_subscription(self, *args):
        pass

    def get_clock(self):
        class Clock:
            def now(self):
                ns = time.time_ns()
                return SimpleNamespace(nanoseconds=ns, to_msg=lambda: Time(sec=ns//10**9, nanosec=ns % 10**9))
        return Clock()

    def _stop_manual(self, reason):
        self.token = ""

    def _fresh_ready(self):
        return True

    def ready(self):
        self._localization(String(data=json.dumps(dict(stamp_ns=time.time_ns(), ready=True,
            state="LOCALIZATION_READY", map_id="map-a", initial_pose_received=True))))
        for kind in ("rectangle", "l"):
            self.status(kind)

    def status(self, kind, **fields):
        value = dict(stamp_ns=time.time_ns(), task_type=kind, map_id="map-a", can_start=True,
                     start_blockers=[], phase="IDLE", run_id="old")
        value.update(fields)
        self._task_status(kind, String(data=json.dumps(value)))


def task(kind="l", action="start", request_id="request-1"):
    return dict(op="task", task_type=kind, action=action, request_id=request_id)


def initial_pose():
    return dict(op="initial_pose", request_id="pose-1", map_revision=1, x=2., y=3., yaw=math.pi/2)


def respond(client, success=True):
    client.calls[-1][1].set_result(SimpleNamespace(success=success, message="response"))


def test_routes_l_task_and_waits_for_new_run_snapshot():
    g = Gateway()
    g._operation_request("app", task())
    assert len(g.task_clients["l"]["start"].calls) == 1
    assert not g.task_clients["rectangle"]["start"].calls
    respond(g.task_clients["l"]["start"])
    assert g.command_pending["stage"] == "status"
    g._task_tick()
    assert g.command_pending  # Old idle snapshot is not proof of a new start.
    g.status("l", phase="PLANNING", run_id="new-run")
    g._task_tick()
    assert g.command_pending is None and g.sent[-1]["success"]
    assert g.task_locks == {"l"}
    g._operation_request("app", task("rectangle", request_id="other"))
    assert not g.sent[-1]["success"]
    assert not g.task_clients["rectangle"]["start"].calls


@pytest.mark.parametrize("failure", ["loc_age", "source_age", "task_age", "map", "type", "service"])
def test_start_rejects_bad_evidence(failure):
    g = Gateway()
    if failure == "loc_age":
        g.localization_received -= 1
    elif failure == "source_age":
        g.localization["stamp_ns"] -= 10**9
    elif failure == "task_age":
        g.task_received["l"] -= 1
    elif failure == "map":
        g.tasks["l"]["map_id"] = "other-map"
    elif failure == "type":
        g.tasks["l"]["task_type"] = "rectangle"
    else:
        g.task_clients["l"]["start"].available = False
    g._operation_request("app", task())
    assert not g.sent[-1]["success"]
    assert not g.task_clients["l"]["start"].calls


def test_start_waits_for_pending_manual_acquisition_and_release():
    g = Gateway()
    g._manual_acquires = 1
    g._operation_request("app", task())
    assert not g.task_clients["l"]["start"].calls
    g._manual_acquires = 0
    release = Future()
    g.manual_releases = [release]
    g._task_tick()
    assert not g.task_clients["l"]["start"].calls
    release.set_result(SimpleNamespace(success=True))
    g._task_tick()
    assert len(g.task_clients["l"]["start"].calls) == 1


def test_late_start_reply_is_followed_by_cancel_never_restarts():
    g = Gateway()
    g._operation_request("app", task())
    g.command_pending["since"] -= 4
    g._task_tick()
    assert g.command_pending and g.sent[-1]["pending"]
    g._operation_request("app", task(action="cancel", request_id="stop"))
    assert not g.task_clients["l"]["cancel"].calls
    respond(g.task_clients["l"]["start"])
    assert len(g.task_clients["l"]["cancel"].calls) == 1
    respond(g.task_clients["l"]["cancel"])
    assert g.command_pending is None
    assert len(g.task_clients["l"]["start"].calls) == 1


def test_cancel_while_waiting_for_start_snapshot_is_dispatched():
    g = Gateway()
    g._operation_request("app", task())
    respond(g.task_clients["l"]["start"])
    g._operation_request("app", task(action="pause", request_id="stop"))
    assert len(g.task_clients["l"]["pause"].calls) == 1


def test_duplicate_request_cannot_execute_twice():
    g = Gateway()
    packet = task(action="cancel")
    g._operation_request("app", packet)
    g._operation_request("app", packet)
    respond(g.task_clients["l"]["cancel"])
    g._operation_request("app", packet)
    assert len(g.task_clients["l"]["cancel"].calls) == 1


def test_stale_terminal_state_cannot_release_task_lock():
    g = Gateway()
    g.status("l", phase="FOLLOW")
    g.status("l", phase="COMPLETE", stamp_ns=time.time_ns()-10**9)
    assert "l" in g.task_locks
    g.status("l", phase="COMPLETE")
    assert not g.task_locks


def test_initial_pose_uses_map_frame_and_does_not_reuse_old_ready():
    g = Gateway()
    g._operation_request("app", initial_pose())
    request = g.initial_pose_client.calls[-1][0]
    assert request.pose.header.frame_id == "map"
    assert request.pose.pose.pose.position.x == 2.
    assert request.pose.pose.pose.orientation.z == pytest.approx(math.sqrt(.5))
    assert not g._localization_view()["ready"]
    stamp = g.initial_pose_stamp
    respond(g.initial_pose_client)
    assert not g._localization_view()["ready"]
    g.localization["initial_pose_stamp_ns"] = stamp
    g._localization(String(data=json.dumps(g.localization)))
    assert g._localization_view()["ready"]


@pytest.mark.parametrize("failure", ["revision", "bounds", "busy"])
def test_initial_pose_rejects_wrong_map_or_running_task(failure):
    g = Gateway()
    packet = initial_pose()
    if failure == "revision":
        packet["map_revision"] = 2
    elif failure == "bounds":
        packet["x"] = -1.
    else:
        g.status("rectangle", phase="NAVIGATING")
    g._operation_request("app", packet)
    assert not g.sent[-1]["success"]
    assert not g.initial_pose_client.calls


def test_unknown_start_result_stays_locked_until_explicit_cancel():
    g = Gateway()
    g._operation_request("app", task())
    g.task_clients["l"]["start"].calls[-1][1].set_exception(RuntimeError("transport lost"))
    g.status("l", phase="IDLE")
    assert "l" in g.task_locks
    g._operation_request("app", task(action="cancel", request_id="cancel"))
    respond(g.task_clients["l"]["cancel"])
    assert not g.task_locks and not g.uncertain_tasks
