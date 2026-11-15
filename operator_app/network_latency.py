"""Observed WebSocket RTT; display only, independent of control readiness."""
import threading
import time

try:
    from PySide6.QtWidgets import QLabel
except ImportError:
    from PyQt5.QtWidgets import QLabel


class NetworkLatency:
    # 20 Hz manual input / 250 ms input watchdog. The 100 ms warning leaves
    # margin for jitter; RTT is not the watchdog's inter-arrival measurement.
    PERIOD = .5
    WARNING_MS = 100.
    TIMEOUT = .25

    def __init__(self, *, mock=False):
        self.mock = mock
        self.lock = threading.Lock()
        self.serial = 0  # Never reused, including across reconnects.
        self.connected = False
        self.started_at = self.last_probe_at = self.pending = self.rtt_ms = None
        self.timed_out = False

    def connect(self):
        with self.lock:
            self.connected = True
            self.started_at = time.monotonic()
            self.last_probe_at = self.pending = self.rtt_ms = None
            self.timed_out = False

    def disconnect(self):
        with self.lock:
            self.connected = False
            self.last_probe_at = self.pending = self.rtt_ms = None
            self.timed_out = False

    def probe(self):
        """Reserve one probe immediately before sending; never queue probes."""
        with self.lock:
            now = time.monotonic()
            if not self.connected or self.mock:
                return None
            if self.last_probe_at is not None and now - self.last_probe_at < self.PERIOD:
                return None
            if self.pending is not None:
                self.timed_out = True
            self.serial += 1
            payload = f"f1-rtt:{self.serial}".encode("ascii")
            self.pending = (payload, now)
            self.last_probe_at = now
            return payload

    def pong(self, payload, received_at):
        with self.lock:
            if not self.connected or self.pending is None or payload != self.pending[0]:
                return False
            # Both timestamps are from this process's monotonic clock. This
            # includes socket queueing and peer scheduling, but no Qt queueing.
            self.rtt_ms = round(max(0., received_at - self.pending[1]) * 1000, 1)
            self.pending = None
            self.timed_out = False
            return True

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            if self.mock:
                return "mock", None
            if not self.connected:
                return "disconnected", None
            if self.pending is not None and now - self.pending[1] >= self.TIMEOUT:
                return "timeout", None
            reference = self.last_probe_at if self.last_probe_at is not None else self.started_at
            if now - reference >= self.PERIOD + self.TIMEOUT:
                return "stale", None  # Sender stalled: never retain an old green value.
            if self.timed_out:
                return "timeout", None
            if self.rtt_ms is None:
                return "waiting", None
            level = ("healthy" if self.rtt_ms < self.WARNING_MS else
                     "acceptable" if self.rtt_ms < self.TIMEOUT * 1000 else "slow")
            return level, self.rtt_ms


class LatencyLabel(QLabel):
    COLORS = dict(healthy="#16803c", acceptable="#a66b00", slow="#c62828",
                  timeout="#c62828", stale="#c62828", disconnected="#c62828",
                  waiting="#a66b00", mock="#687c8c")
    TEXT = dict(healthy="健康", acceptable="可接受", slow="超时",
                timeout="超时（≥250 ms 未回复）", stale="测量已过期",
                disconnected="未连接", waiting="测量中…", mock="模拟模式")

    def __init__(self, monitor, name, url):
        super().__init__()
        self.monitor, self.name = monitor, name
        self.setWordWrap(True)
        self.setToolTip(
            f"Windows ↔ Nano（{url}）\n"
            "往返延迟 RTT：每 500 ms 通过当前控制连接发送 Ping，收到匹配 Pong 后计时。\n"
            "绿：<100 ms；黄：100–<250 ms；红：≥250 ms、未回复、读数过期或未连接。\n"
            "按项目默认 50 ms 指令周期、250 ms 输入看门狗设置提示线；100 ms 留出抖动余量。\n"
            "包含网络、套接字排队和两端调度；不是单程延迟，也不是遥测年龄。\n"
            "颜色仅提示连接延迟，可接受不保证控制可用；机器人状态以原有控制门控为准。")
        self.refresh()

    def refresh(self):
        level, rtt_ms = self.monitor.snapshot()
        value = f"{rtt_ms:.1f} ms · " if rtt_ms is not None else ""
        self.setText(f"{self.name}网络 RTT：{value}{self.TEXT[level]}")
        if self.property("latencyState") != level:
            self.setProperty("latencyState", level)
            self.setStyleSheet(f"color:{self.COLORS[level]}; font-weight:600;")
