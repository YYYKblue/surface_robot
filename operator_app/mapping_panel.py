"""Mapping/catalogue controls; previews have a separate, pose-free canvas."""
try:
    from PySide6.QtWidgets import (QComboBox, QDialog, QGridLayout, QLabel, QLineEdit,
                                  QMessageBox, QPushButton, QVBoxLayout)
except ImportError:
    from PyQt5.QtWidgets import (QComboBox, QDialog, QGridLayout, QLabel, QLineEdit,
                                QMessageBox, QPushButton, QVBoxLayout)
try:
    from .map_canvas import MapCanvas
except ImportError:
    from map_canvas import MapCanvas

PHASE_LABELS = {"IDLE": "待机", "STARTING": "启动中", "MAPPING": "建图中", "SAVING": "保存中",
                "STOPPING": "退出当前模式", "LOCALIZATION": "地图定位", "ERROR": "模式异常"}


class MappingPanel:
    def _build_mapping(self, parent):
        self.catalog = []
        self.runtime_session = None
        self.preview_dialog = None
        self.displayed_saved_name = None
        box = QVBoxLayout(parent)
        self.mapping_status = QLabel("建图与选图需要受管网关支持")
        self.mapping_status.setWordWrap(True); box.addWidget(self.mapping_status)
        self.current_map_label = QLabel("当前使用地图：无")
        self.current_map_label.setWordWrap(True); box.addWidget(self.current_map_label)
        self.map_name = QLineEdit()
        self.map_name.setMaxLength(80)
        self.map_name.setPlaceholderText("地图名称（留空使用时间戳）")
        self.map_name.setToolTip("中文、字母、数字、下划线或短横线；重名自动加序号。输入时停止键盘驾驶。")
        box.addWidget(self.map_name)
        self.mapping_buttons = {}
        actions = QGridLayout()
        for index, (label, action) in enumerate((("开始建图", "start"), ("保存快照", "save"),
                                                ("保存并结束", "save_end"), ("放弃本次建图", "discard"))):
            button = QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(lambda checked=False, a=action: self._mapping_action(a))
            self.mapping_buttons[action] = button
            actions.addWidget(button, index // 2, index % 2)
        box.addLayout(actions)
        self.mapping_result = QLabel("保存位置：Nano 工作空间 maps/；保存快照后可继续探索。")
        self.mapping_result.setWordWrap(True); box.addWidget(self.mapping_result)
        box.addWidget(QLabel("已保存地图"))
        self.saved_maps = QComboBox()
        self.saved_maps.currentIndexChanged.connect(self._selected_map_changed)
        box.addWidget(self.saved_maps)
        self.selected_map_info = QLabel("等待地图列表")
        self.selected_map_info.setWordWrap(True); box.addWidget(self.selected_map_info)
        catalogue_actions = QGridLayout()
        for index, (label, action) in enumerate((("刷新地图列表", "list"), ("预览所选地图", "preview"),
                                                ("使用此地图", "load"), ("重试快照导出", "retry_export"))):
            button = QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(lambda checked=False, a=action: self._mapping_action(a))
            self.mapping_buttons[action] = button
            catalogue_actions.addWidget(button, index // 2, index % 2)
        box.addLayout(catalogue_actions)
        note = QLabel("建图时使用手动控制探索。切换地图前先结束建图或停止清洗；加载后重新设置初始位姿。")
        note.setWordWrap(True); box.addWidget(note)
        box.addStretch()

    def _selected_map_changed(self, *args):
        entry = self.saved_maps.currentData() or {}
        self.selected_map_info.setText("\n".join(str(x) for x in (
            entry.get("saved_at"), "文件完整" if entry.get("valid") else entry.get("reason", "尚未选择地图")) if x))
        self._refresh_controls()

    def _mapping_action(self, action):
        if not self.mapping_buttons[action].isEnabled():
            return
        fields = dict(action=action)
        if action in ("save", "save_end"):
            fields["name"] = self.map_name.text().strip()
        if action in ("preview", "load", "retry_export"):
            fields["name"] = (self.saved_maps.currentData() or {})["name"]
        if action == "discard":
            self._stop("confirm_discard_mapping")
            answer = QMessageBox.question(self, "放弃本次建图", "退出当前建图并放弃尚未保存的内容？此前保存的地图会保留。",
                                          QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
            fields["confirm"] = True
        if action not in ("list", "preview"):
            self._stop("mapping_" + action)
        self._send_request("mapping", **fields)

    def _refresh_mapping(self, fresh, operator, local_pending):
        if not hasattr(self, "mapping_buttons"):
            return
        runtime = self.telemetry.get("runtime") or {}
        supported = {"mapping", "map_catalog"} <= self.capabilities
        busy = bool(runtime.get("busy") or self.telemetry.get("task_busy") or local_pending)
        available = fresh and supported and operator and not busy
        mode = runtime.get("mode", "idle")
        entry = self.saved_maps.currentData() or {}
        for action, button in self.mapping_buttons.items():
            enabled = available
            if action in ("list", "preview"):
                enabled = fresh and supported and not runtime.get("operation") and not local_pending
            if action == "start":
                enabled = enabled and mode != "mapping" and runtime.get("scan_fresh", False)
            elif action in ("save", "save_end"):
                enabled = (enabled and mode == "mapping" and runtime.get("scan_fresh")
                           and runtime.get("map_available") and runtime.get("pose_fresh"))
            elif action == "discard":
                enabled = enabled and mode == "mapping"
            elif action in ("load", "preview"):
                enabled = enabled and entry.get("valid", False) and (action != "load" or mode != "mapping")
            elif action == "retry_export":
                enabled = enabled and entry.get("recoverable", False)
            button.setEnabled(bool(enabled))
        self.saved_maps.setEnabled(not local_pending)
        if not supported:
            text = "建图与选图需要 operator_system.launch.py 受管网关"
        elif not fresh:
            text = "连接中断或状态过期；重连后同步机器人端建图状态"
        else:
            text = PHASE_LABELS.get(runtime.get("phase"), "等待状态")
            text += " · 雷达：" + ("正常" if runtime.get("scan_fresh") else "无有效扫描")
            text += "\n" + runtime.get("message", "")
        self.mapping_status.setText(text)
        self.current_map_label.setText("当前使用地图：" + (runtime.get("current_map") or "无"))
        if runtime.get("phase") == "STARTING" and runtime.get("target_map"):
            self.current_map_label.setText("正在加载地图：" + runtime["target_map"])
        self.current_map_label.setToolTip(runtime.get("maps_directory", ""))

    def _mapping_reply(self, packet):
        if packet.get("op") != "mapping" or packet.get("pending"):
            return
        if not packet.get("success"):
            self.mapping_result.setText(packet.get("message", "地图操作失败"))
            return
        if "maps" in packet:
            selected = (self.saved_maps.currentData() or {}).get("name")
            self.catalog = packet["maps"]
            self.saved_maps.blockSignals(True)
            self.saved_maps.clear()
            for entry in self.catalog:
                suffix = "" if entry.get("valid") else "（待导出）" if entry.get("recoverable") else "（不可用）"
                self.saved_maps.addItem(entry["name"] + suffix, entry)
            for index, entry in enumerate(self.catalog):
                preferred = (getattr(self, "select_saved_name", None) or selected or
                             ((self.telemetry.get("runtime") or {}).get("last_saved") or {}).get("name"))
                if entry["name"] == preferred:
                    self.saved_maps.setCurrentIndex(index)
                    break
            self.select_saved_name = None
            self.saved_maps.blockSignals(False)
            self._selected_map_changed()
        if packet.get("preview"):
            self._stop("map_preview")
            if self.preview_dialog:
                self.preview_dialog.close()
            self.preview_dialog = QDialog(self)
            self.preview_dialog.setWindowTitle("地图预览：" + packet.get("name", ""))
            self.preview_dialog.resize(850, 650)
            layout = QVBoxLayout(self.preview_dialog)
            layout.addWidget(QLabel("仅预览保存的地图；不显示当前机器人位姿或轨迹。"))
            canvas = MapCanvas()
            canvas.setToolTip("保存的栅格地图预览")
            canvas.set_telemetry(dict(type="map", map=packet["preview"]))
            layout.addWidget(canvas)
            self.preview_dialog.show()
        if packet.get("saved"):
            saved = packet["saved"]
            self.displayed_saved_name = saved["name"]
            self.select_saved_name = saved["name"]
            self.mapping_result.setText("已保存：" + saved["name"] + "\n" + "\n".join(saved["paths"]))
            self._send_request("mapping", action="list")

    def _mapping_telemetry(self, packet):
        runtime = packet.get("runtime") or {}
        session = runtime.get("session_id")
        if session is not None and session != self.runtime_session:
            self.runtime_session = session
            self.completed_results.clear()
            self.initial_task_selected = False
            self.initial_pose = None
            self.pose_button.setChecked(False)
            self.canvas.set_task_overlay({}, live=False)
        operation = runtime.get("last_operation") or {}
        request_id = operation.get("request_id")
        if request_id in self.pending_requests:
            # Completion can precede reconnect; never replay an uncertain command.
            self.pending_requests.pop(request_id, None)
        saved = runtime.get("last_saved")
        if saved and self.displayed_saved_name != saved["name"]:
            self.displayed_saved_name = saved["name"]
            self.mapping_result.setText("已保存：" + saved["name"] + "\n" + "\n".join(saved["paths"]))
