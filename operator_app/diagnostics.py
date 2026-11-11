"""Operator-facing diagnostic log, independent of motion control decisions."""
from datetime import datetime
import json
import math
from pathlib import Path
import time

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import (QApplication, QDialog, QFileDialog, QFrame, QHBoxLayout, QLabel,
                                   QPlainTextEdit, QPushButton, QScrollArea, QVBoxLayout, QWidget)
except ImportError:
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import (QApplication, QDialog, QFileDialog, QFrame, QHBoxLayout, QLabel,
                                 QPlainTextEdit, QPushButton, QScrollArea, QVBoxLayout, QWidget)


REASONS = {
    "input_timeout": "网关接收手动输入超时",
    "app_disconnected": "App 与网关连接断开",
    "text_input": "输入框获得焦点，停止手动输出",
    "window_deactivated": "窗口失去激活，停止手动输出",
    "window_closed": "App 窗口关闭",
    "gui_input_expired": "界面操作意图超过 250 ms 未更新，已停止手动输出",
    "diagnostics_export": "导出日志前停止手动输出",
    "button": "点击停止手动输出",
    "keyboard": "按空格停止手动输出",
    "automatic_task_active": "自动任务占用控制",
    "gateway_shutdown": "网关退出",
    "command deadline expired": "串口桥执行指令过期",
    "CTRL timeout": "下位机 CTRL 状态超时",
    "ACK progress timeout": "下位机 ACK 确认超时",
    "firmware protocol unavailable": "F103 协议未就绪或不兼容，已停止手动输出",
    "control_identity_changed": "串口连接、控制源或控制代次改变，已清除旧手动动作",
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
        page = QScrollArea()
        page.setWidgetResizable(True)
        page.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        page.setWidget(content)
        layout = QVBoxLayout(content)
        hint = QLabel("记录连接、手动锁定及停止原因；保留最近 2000 条。\n"
                      "复现后导出日志。时间带时区；Nano 事件使用 Nano 时间，可能与 Windows 有偏差。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.diagnostic_summary = QLabel("等待遥测")
        self.diagnostic_summary.setWordWrap(True)
        layout.addWidget(self.diagnostic_summary)
        self.output_diagnostics = QLabel("F103 输出目标：等待状态")
        self.output_diagnostics.setWordWrap(True)
        layout.addWidget(self.output_diagnostics)
        self.rc_diagnostics = QLabel("RC 接收诊断：等待状态（只读）")
        self.rc_diagnostics.setWordWrap(True)
        layout.addWidget(self.rc_diagnostics)
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
        self.log_view.setMinimumHeight(180)
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
        # Native save dialogs may block GUI timers. Revoke held output before
        # entering the dialog; the independent sender can deliver this stop.
        self._stop("diagnostics_export")
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

    def _diagnostic_history(self, history):
        """Consume event history independently from fast state telemetry."""
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

    def _diagnostic_telemetry(self, packet):
        state = packet.get("state", {})
        history = packet.get("diagnostics") or {}
        if history.get("session"):
            self._diagnostic_history(history)
        elif "diagnostic_history" not in self.capabilities:
            keys = ("state", "source", "owner", "motion_authorized", "reason", "last_stop_reason")
            self._log_change("bridge", tuple(state.get(k) for k in keys), "串口桥/接收时刻",
                             state.get("reason", "桥状态更新"), {k: state.get(k) for k in keys})
        received = packet.get("_received_monotonic")
        if isinstance(received, (int, float)) and received > 0:
            ui_queue_ms = max(0., (time.monotonic() - received) * 1000)
            self._log_change("ui_delay", ui_queue_ms >= 200, "App",
                             "遥测界面处理延迟" if ui_queue_ms >= 200 else "遥测界面处理及时",
                             {"ui_queue_ms": round(ui_queue_ms, 1),
                              "transport": packet.get("_transport", {}),
                              "stream": packet.get("stream", {})})
        fresh = packet.get("control_fresh")
        self._log_change("control_fresh", fresh, "网关",
                         "桥状态新鲜度变化", {"control_fresh": fresh})
        self.diagnostic_summary.setText(
            f"设备：{state.get('state', '未知')} · 控制源：{state.get('source', '未知')}\n"
            f"CTRL 年龄：{state.get('ctrl_age_sec', '未知')} s · ACK 年龄：{state.get('ack_age_sec', '未知')} s\n"
            f"指令剩余：{state.get('command_remaining_sec', '未知')} s\n"
            f"最近撤销原因：{state.get('last_stop_reason') or '无'}")

    def _refresh_output_diagnostics(self, packet, fresh):
        if not fresh or packet.get("control_fresh") is not True:
            self.output_diagnostics.setText("F103 输出目标：状态未收到或已过期")
            self.rc_diagnostics.setText("RC 接收诊断：状态未收到或已过期（只读）")
            return
        state = packet.get("state", {})
        compatible = {True: "兼容", False: "不兼容"}.get(state.get("firmware_protocol_compatible"), "未知")
        protocol = f"F103 协议：v{state.get('firmware_protocol_version', '未知')}（{compatible}）"
        pwm = state.get("target_pwm_bl_br_fl_fr")
        values = (" · ".join(f"{name}={value}" for name, value in zip(("BL", "BR", "FL", "FR"), pwm))
                  if isinstance(pwm, (list, tuple)) and len(pwm) == 4 else "未提供")
        ack = state.get("ack_sequence", "—") if state.get("have_sequence") is True else "未确认"
        self.output_diagnostics.setText(protocol + "\n" + (
            f"四路目标 PWM / μs：{values}\n"
            f"发送序号：{state.get('sent_sequence', '—')} · ACK：{ack} · 结果：{state.get('result', '—')}\n"
            "目标 PWM / ACK 表示指令目标与接收状态，不是实际转速、推力或船速。"
            if self._firmware_ctrl_fresh(state) else
            f"F103 输出目标：CTRL 未收到或已过期\n原因：{state.get('reason') or '等待新鲜 CTRL'}"))
        channels = state.get("rc_channels")
        channel_text = (" · ".join(f"CH{i}={value}" for i, value in enumerate(channels, 1))
                        if isinstance(channels, (list, tuple)) and len(channels) == 14 else "未提供")
        flags = state.get("rc_flags")
        flags_text = f"0x{flags:02X}" if type(flags) is int else "—"
        rc_age = self._reported_age_sec(state, "rc_age_sec")
        rc_fresh = (state.get("connected") is True and state.get("rc_valid") is True
                    and rc_age is not None and rc_age < .4)
        valid = "新鲜帧" if rc_fresh else "无有效帧或已过期"
        if not rc_fresh:
            channel_text = "不显示过期原始通道"
        age_text = f"{rc_age:.3f} s" if rc_age is not None else "未知"
        self.rc_diagnostics.setText(
            f"RC 接收诊断（只读）：{valid} · 帧序号 {state.get('rc_sequence', '—')}\n"
            f"年龄：{age_text} · flags：{flags_text}\n{channel_text}")

    def _reported_age_sec(self, state, field):
        age = state.get(field)
        if type(age) not in (int, float) or not math.isfinite(age) or age < 0 or not self.last_telemetry_at:
            return None
        control_age_ms = (self.telemetry.get("stream") or {}).get("control_age_ms", 0.)
        if (type(control_age_ms) not in (int, float) or not math.isfinite(control_age_ms)
                or control_age_ms < 0):
            return None
        # The gateway can keep publishing ROS state while serial CTRL/RC stalls.
        # Include network-receive-to-display age and known gateway queue delay.
        return age + control_age_ms / 1000. + max(0., time.monotonic()-self.last_telemetry_at)

    def _firmware_ctrl_fresh(self, state):
        age = self._reported_age_sec(state, "ctrl_age_sec")
        return state.get("connected") is True and age is not None and age < .4
