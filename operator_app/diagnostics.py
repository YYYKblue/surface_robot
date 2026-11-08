"""Operator-facing diagnostic log, independent of motion control decisions."""
from datetime import datetime
import json
from pathlib import Path
import time

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import (QApplication, QDialog, QFileDialog, QHBoxLayout, QLabel,
                                   QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)
except ImportError:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (QApplication, QDialog, QFileDialog, QHBoxLayout, QLabel,
                                 QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)


REASONS = {
    "input_timeout": "网关接收手动输入超时",
    "app_disconnected": "App 与网关连接断开",
    "text_input": "输入框获得焦点，停止手动输出",
    "window_deactivated": "窗口失去激活，停止手动输出",
    "window_closed": "App 窗口关闭",
    "button": "点击停止手动输出",
    "keyboard": "按空格停止手动输出",
    "automatic_task_active": "自动任务占用控制",
    "gateway_shutdown": "网关退出",
    "command deadline expired": "串口桥执行指令过期",
    "CTRL timeout": "下位机 CTRL 状态超时",
    "ACK progress timeout": "下位机 ACK 确认超时",
    "transmit scheduling gap": "串口发送调度间隔超限",
    "source/epoch changed": "控制源或控制代次发生变化",
    "owner released control": "控制者释放控制权",
    "RC has control; Nano stop cannot override RC": "遥控器 RC 持有控制权",
}


class DiagnosticPanel:
    def _build_diagnostics(self, tabs):
        self._log_states = {}
        self._gateway_cursors = {}
        self._last_send_tick = None
        self._log_dialog = None
        page = QWidget()
        layout = QVBoxLayout(page)
        hint = QLabel("记录连接、手动锁定及停止原因；保留最近 2000 条。\n"
                      "复现后导出日志。时间带时区；Nano 事件使用 Nano 时间，可能与 Windows 有偏差。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.diagnostic_summary = QLabel("等待遥测")
        self.diagnostic_summary.setWordWrap(True)
        layout.addWidget(self.diagnostic_summary)
        expand = QPushButton("展开查看日志")
        expand.clicked.connect(self._expand_diagnostics)
        layout.addWidget(expand)
        actions = QHBoxLayout()
        for title, callback in (("复制日志", self._copy_diagnostics),
                                ("导出日志…", self._export_diagnostics),
                                ("清空日志", lambda: self.log_view.clear())):
            button = QPushButton(title)
            button.clicked.connect(callback)
            actions.addWidget(button)
        layout.addLayout(actions)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        # Reading/copying logs must not capture WASD as a text input.
        self.log_view.setFocusPolicy(Qt.NoFocus)
        self.log_view.document().setMaximumBlockCount(2000)
        layout.addWidget(self.log_view, 1)
        tabs.addTab(page, "诊断日志")
        self._log("App", "日志已启动")

    def _log(self, source, reason, details=None, timestamp=None):
        stamp = datetime.fromtimestamp(time.time() if timestamp is None else timestamp).astimezone().isoformat(timespec="milliseconds")
        translated = REASONS.get(reason)
        message = f"{translated} ({reason})" if translated else reason
        suffix = " " + json.dumps(details, ensure_ascii=False, separators=(",", ":")) if details else ""
        # One physical line per event keeps memory bounded and exports readable.
        self.log_view.appendPlainText(f"{stamp} [{source}] {message}{suffix}".replace("\n", "\\n").replace("\r", "\\r"))

    def _log_change(self, key, value, source, reason, details=None):
        if key not in self._log_states or self._log_states[key] != value:
            self._log_states[key] = value
            self._log(source, reason, details)

    def _copy_diagnostics(self):
        QApplication.clipboard().setText(self.log_view.toPlainText())

    def _expand_diagnostics(self):
        if self._log_dialog is None:
            self._log_dialog = QDialog(self)
            self._log_dialog.setWindowTitle("F1 诊断日志")
            self._log_dialog.resize(1100, 650)
            layout = QVBoxLayout(self._log_dialog)
            viewer = QPlainTextEdit()
            viewer.setReadOnly(True)
            viewer.setDocument(self.log_view.document())
            layout.addWidget(viewer)
        self._log_dialog.show()
        self._log_dialog.raise_()

    def _export_diagnostics(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, "导出诊断日志", datetime.now().strftime("F1-diagnostics-%Y%m%d-%H%M%S.txt"),
            "文本日志 (*.txt)")
        if not filename:
            return
        try:
            self._write_diagnostics(filename)
        except OSError as error:
            self._log("App", "日志导出失败", {"error": str(error)})
            self.status_bar.setText("日志导出失败：" + str(error))
        else:
            self.status_bar.setText("日志已导出：" + filename)

    def _write_diagnostics(self, filename):
        Path(filename).write_text(self.log_view.toPlainText() + "\n", encoding="utf-8-sig")

    def _diagnostic_telemetry(self, packet):
        state = packet.get("state", {})
        history = packet.get("diagnostics") or {}
        session = history.get("session")
        if session:
            if session not in self._gateway_cursors:
                self._log("网关", "接收网关最近诊断记录", {"session": session})
                if len(self._gateway_cursors) >= 8:
                    self._gateway_cursors.pop(next(iter(self._gateway_cursors)))
            cursor = self._gateway_cursors.get(session, 0)
            events = history.get("events", [])
            if events and events[0]["id"] > cursor + 1:
                self._log("网关", "部分早期诊断记录已被覆盖", {"missing_events": events[0]["id"] - cursor - 1})
            for event in events:
                if event["id"] > cursor:
                    self._log("串口桥/Nano" if event["source"] == "bridge" else "网关/Nano",
                              event["reason"], event.get("details"), event.get("time_unix"))
                    cursor = event["id"]
            self._gateway_cursors[session] = cursor
        else:
            keys = ("state", "source", "owner", "motion_authorized", "reason", "last_stop_reason")
            self._log_change("bridge", tuple(state.get(k) for k in keys), "串口桥/接收时刻",
                             state.get("reason", "桥状态更新"), {k: state.get(k) for k in keys})
        fresh = packet.get("control_fresh")
        self._log_change("control_fresh", fresh, "网关",
                         "桥状态新鲜度变化", {"control_fresh": fresh})
        self.diagnostic_summary.setText(
            f"设备：{state.get('state', '未知')} · 控制源：{state.get('source', '未知')}\n"
            f"CTRL 年龄：{state.get('ctrl_age_sec', '未知')} s · ACK 年龄：{state.get('ack_age_sec', '未知')} s\n"
            f"指令剩余：{state.get('command_remaining_sec', '未知')} s\n"
            f"最近撤销原因：{state.get('last_stop_reason') or '无'}")
