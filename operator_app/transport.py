"""WebSocket transport worker for the Qt operator app."""
import json
import math
import threading
import time
from queue import Empty, Queue

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
        self.outgoing = Queue(maxsize=4)
        self.stop_requested = threading.Event()

    def send_packet(self, packet):
        # Latest input wins. Do not let a blocked socket accumulate old motion.
        if packet.get("op") == "manual_input":
            preserved = []
            while True:
                try:
                    old = self.outgoing.get_nowait()
                except Empty:
                    break
                if old.get("op") in ("manual_stop", "task"):
                    preserved.append(old)
            for old in preserved:
                try:
                    self.outgoing.put_nowait(old)
                except Exception:
                    break
            try:
                self.outgoing.put_nowait(packet)
            except Exception:
                pass
            return
        try:
            self.outgoing.put_nowait(packet)
        except Exception:
            # A stop/task packet may evict an obsolete motion input, never a
            # queued stop or task packet.
            items = []
            while True:
                try:
                    items.append(self.outgoing.get_nowait())
                except Empty:
                    break
            for index, old in enumerate(items):
                if old.get("op") == "manual_input":
                    del items[index]
                    break
            for old in items:
                try:
                    self.outgoing.put_nowait(old)
                except Exception:
                    break
            try:
                self.outgoing.put_nowait(packet)
            except Exception:
                pass

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
        try:
            ws = websocket.create_connection(self.url, timeout=.2,
                                             enable_multithread=True)
            ws.settimeout(.05)
            self.status.emit("已连接")
            self._send_now(ws, {"op": "hello", "client": "f1-operator-qt", "version": 1})
            while not self.stop_requested.is_set():
                self._flush(ws)
                try:
                    raw = ws.recv()
                except Exception as error:
                    if "timed out" not in str(error).lower() and "timeout" not in str(error).lower():
                        raise
                    continue
                if raw:
                    try:
                        self.message.emit(json.loads(raw))
                    except (TypeError, ValueError):
                        self.status.emit("网关发送了无效数据")
            try:
                self._send_now(ws, {"op": "manual_stop", "reason": "app_closed"})
                ws.close()
            except Exception:
                pass
        except Exception as error:
            self.status.emit("连接失败：" + str(error))

    def _flush(self, ws):
        while True:
            try:
                packet = self.outgoing.get_nowait()
            except Empty:
                return
            self._send_now(ws, packet)

    @staticmethod
    def _send_now(ws, packet):
        ws.send(json.dumps(packet, ensure_ascii=False, separators=(",", ":")))

    def _mock_loop(self):
        started = time.monotonic()
        while not self.stop_requested.wait(.1):
            t = time.monotonic() - started
            self.message.emit({
                "type": "telemetry", "state": {"state": "READY", "connected": True,
                "source": "NANO", "motion_authorized": False},
                "pose": {"x": 1.15 + .15 * math.sin(t / 5), "y": 1.0 + .08 * math.cos(t / 4),
                         "yaw": .15 * math.sin(t / 3)},
                "scan": {"angle_min": -math.pi, "angle_increment": math.pi / 180,
                         "ranges": [1.0 + .12 * math.sin(t + i / 15) for i in range(360)]},
                "map": {"resolution": .02, "width": 160, "height": 100,
                        "origin": {"x": 0, "y": 0},
                        "data": [0] * (160 * 100)},
            })
