"""Map-frame drawing and initial-pose picking, with one invertible transform."""
import math

try:
    from PySide6.QtCore import QPointF, Qt, Signal, QTimer
    from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
    from PySide6.QtWidgets import QWidget
except ImportError:
    from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal as Signal
    from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
    from PyQt5.QtWidgets import QWidget


class MapCanvas(QWidget):
    initial_pose_selected = Signal(dict)
    map_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(680, 500)
        self.telemetry = {}
        self.task_overlay = {}
        self.overlay_live = False
        self.map_image = None
        self.map_revision = None
        self._scaled_image = None
        self._scaled_key = None
        self._track_cache_key = None
        self._track_cache_owner = None
        self._screen_track_cache = []
        self._paint_timer = QTimer(self)
        self._paint_timer.setSingleShot(True)
        self._paint_timer.timeout.connect(self.update)
        self.picking = False
        self.drag_start = None
        self.candidate = None
        self.setToolTip("点击设置初始位置后，在地图中按住并拖动，箭头指向船头")

    def set_telemetry(self, packet):
        mapping = packet.get("map")
        if mapping is None and packet.get("type") == "map" and "width" in packet:
            mapping = {key: packet[key] for key in
                       ("resolution", "width", "height", "origin", "data", "revision", "frame_id")
                       if key in packet}
        if mapping is not None:
            revision = mapping.get("revision")
            prepared = packet.get("_map_image")
            if revision != self.map_revision or self.map_image is None or prepared is not None:
                self.map_image = prepared if prepared is not None else self._make_map_image(mapping)
                self.map_revision = revision
                self._scaled_key = self._track_cache_key = None
                self.candidate = self.drag_start = None
                self.telemetry["map"] = mapping
                self.map_changed.emit()
            self.telemetry["map"] = mapping
        elif "map" in packet:
            self.map_image = None
            self.map_revision = None
            self._scaled_image = None
            self._scaled_key = self._track_cache_key = None
            self.telemetry.pop("map", None)
            self.candidate = self.drag_start = None
            self.map_changed.emit()
        # Do not retain legacy scan/expected data in the display.
        if "pose" in packet:
            self.telemetry["pose"] = packet["pose"]
        self._request_update()

    def _request_update(self):
        # Display updates are coalesced; manual control keeps its own timer.
        if not self._paint_timer.isActive():
            self._paint_timer.start(33)

    def set_task_overlay(self, task, *, live):
        previous = self.task_overlay
        changed = (previous.get("actual_track") is not task.get("actual_track")
                   or previous.get("map_id") != task.get("map_id")
                   or previous.get("run_id") != task.get("run_id"))
        self.task_overlay, self.overlay_live = task, live
        if changed:
            self._track_cache_key = None
            self._request_update()

    def _track_layers(self):
        """Actual history only; gaps and distinct runs are never joined."""
        task = self.task_overlay
        frame = (self.telemetry.get("map") or {}).get("frame_id")
        actual = task.get("actual_track") or {}
        if (actual.get("frame_id") == frame and actual.get("map_id") == task.get("map_id")
                and actual.get("run_id") == task.get("run_id")):
            return [(segment, "#00877e", False) for segment in actual.get("segments", [])]
        return []

    def _screen_tracks(self):
        actual = self.task_overlay.get("actual_track")
        key = (id(actual), self.map_revision, self.width(), self.height(),
               self.task_overlay.get("map_id"), self.task_overlay.get("run_id"))
        if key != self._track_cache_key:
            polylines = []
            for points, _, _ in self._track_layers():
                polyline = QPolygonF()
                for point in points:
                    if len(point) == 2 and all(math.isfinite(v) for v in point):
                        polyline.append(self.map_to_screen(*point))
                    else:
                        if len(polyline):
                            polylines.append(polyline)
                        polyline = QPolygonF()
                if len(polyline):
                    polylines.append(polyline)
            self._track_cache_key = key
            self._track_cache_owner = actual
            self._screen_track_cache = polylines
        return self._screen_track_cache

    @staticmethod
    def _make_map_image(mapping):
        width, height = int(mapping.get("width", 0)), int(mapping.get("height", 0))
        if width <= 0 or height <= 0:
            return None
        data = mapping.get("data", [])
        # One byte per cell; avoid millions of Qt calls on the UI thread during SLAM.
        if len(data) != width * height:
            return None
        pixels = bytes(2 if value > 50 else 1 if value >= 0 else 0 for value in data)
        image = QImage(pixels, width, height, width, QImage.Format_Indexed8).copy()
        image.setColorTable([QColor(color).rgb() for color in ("#eef2f3", "#ffffff", "#536875")])
        return image.mirrored(False, True)

    def _geometry(self):
        mapping = self.telemetry.get("map") or {}
        resolution = float(mapping.get("resolution", 0))
        width, height = mapping.get("width", 0)*resolution, mapping.get("height", 0)*resolution
        if width <= 0 or height <= 0:
            return None
        area = self.rect().adjusted(28, 28, -28, -28)
        scale = min(area.width()/width, area.height()/height)
        return scale, area.center().x()-width*scale/2, area.center().y()+height*scale/2, width, height

    def map_to_screen(self, x, y):
        geometry = self._geometry()
        if geometry is None:
            return None
        scale, ox, oy, _, _ = geometry
        origin = self.telemetry["map"].get("origin", {})
        yaw = origin.get("yaw", 0.)
        dx, dy = x-origin.get("x", 0.), y-origin.get("y", 0.)
        gx, gy = math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy
        return QPointF(ox+gx*scale, oy-gy*scale)

    def screen_to_map(self, point, *, bounded=True):
        geometry = self._geometry()
        if geometry is None:
            return None
        scale, ox, oy, width, height = geometry
        gx, gy = (point.x()-ox)/scale, (oy-point.y())/scale
        if bounded and not (0 <= gx < width and 0 <= gy < height):
            return None
        origin = self.telemetry["map"].get("origin", {})
        yaw = origin.get("yaw", 0.)
        return (origin.get("x", 0.)+math.cos(yaw)*gx-math.sin(yaw)*gy,
                origin.get("y", 0.)+math.sin(yaw)*gx+math.cos(yaw)*gy)

    def set_picking(self, enabled):
        self.picking = enabled
        self.drag_start = None
        if not enabled:
            self.candidate = None
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        self.update()

    def mousePressEvent(self, event):
        if self.picking and event.button() == Qt.LeftButton:
            self.drag_start = self.screen_to_map(QPointF(event.pos()))
            self.candidate = None
            self.update()
        else:
            super().mousePressEvent(event)

    def _drag(self, point):
        end = self.screen_to_map(point, bounded=False)
        if self.drag_start is None or end is None:
            return
        x, y = self.drag_start
        # Direction needs a visible drag, not a nearly zero-length click.
        if math.hypot(end[0]-x, end[1]-y)*self._geometry()[0] < 8:
            self.candidate = None
        else:
            self.candidate = dict(x=x, y=y, yaw=math.atan2(end[1]-y, end[0]-x), map_revision=self.map_revision)
        self.update()

    def mouseMoveEvent(self, event):
        if self.picking and self.drag_start is not None:
            self._drag(QPointF(event.pos()))
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.picking and event.button() == Qt.LeftButton and self.drag_start is not None:
            self._drag(QPointF(event.pos()))
            self.drag_start = None
            if self.candidate:
                self.initial_pose_selected.emit(dict(self.candidate))
        else:
            super().mouseReleaseEvent(event)

    def _arrow(self, painter, pose, color):
        point = self.map_to_screen(float(pose["x"]), float(pose["y"]))
        if point is None:
            return
        origin = self.telemetry["map"].get("origin", {})
        yaw = float(pose["yaw"])-origin.get("yaw", 0.)
        points = QPolygonF([QPointF(point.x()+25*math.cos(yaw), point.y()-25*math.sin(yaw)),
                            QPointF(point.x()-16*math.cos(yaw+.8), point.y()+16*math.sin(yaw+.8)),
                            QPointF(point.x()-16*math.cos(yaw-.8), point.y()+16*math.sin(yaw-.8))])
        painter.setBrush(QColor(color))
        painter.setPen(QPen(QColor(color), 2))
        painter.drawPolygon(points)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#e9eef1"))
        geometry = self._geometry()
        if geometry and self.map_image is not None:
            scale, ox, oy, width, height = geometry
            scaled_key = (self.map_image.cacheKey(), round(width*scale), round(height*scale))
            if scaled_key != self._scaled_key:
                self._scaled_image = self.map_image.scaled(scaled_key[1], scaled_key[2],
                                                          Qt.IgnoreAspectRatio, Qt.FastTransformation)
                self._scaled_key = scaled_key
            painter.drawImage(round(ox), round(oy-height*scale), self._scaled_image)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor("#00877e"), 3))
            for polyline in self._screen_tracks():
                if len(polyline) > 1:
                    painter.drawPolyline(polyline)
                elif len(polyline) == 1:
                    painter.drawPoint(polyline[0])
            pose = self.telemetry.get("pose")
            if pose:
                self._arrow(painter, pose, "#00877e")
            if self.candidate:
                self._arrow(painter, self.candidate, "#9553c6")
        painter.setPen(QPen(QColor("#637783"), 1))
        painter.drawText(28, 22, "绿色箭头：机器人位置 · 绿线：实际轨迹")
