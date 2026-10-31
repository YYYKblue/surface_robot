"""Map-frame drawing and initial-pose picking, with one invertible transform."""
import math

try:
    from PySide6.QtCore import QPointF, Qt, Signal
    from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
    from PySide6.QtWidgets import QWidget
except ImportError:
    from PyQt5.QtCore import QPointF, Qt, pyqtSignal as Signal
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
            if revision != self.map_revision or self.map_image is None:
                self.map_image = self._make_map_image(mapping)
                self.map_revision = revision
                self.candidate = self.drag_start = None
                self.map_changed.emit()
            self.telemetry["map"] = mapping
        elif "map" in packet:
            self.map_image = None
            self.map_revision = None
            self.telemetry.pop("map", None)
            self.candidate = self.drag_start = None
            self.map_changed.emit()
        # None explicitly invalidates old TF/scans instead of displaying them forever.
        self.telemetry.update({key: value for key, value in packet.items() if key != "map"})
        self.update()

    def set_task_overlay(self, task, *, live):
        self.task_overlay, self.overlay_live = task, live
        self.update()

    def _track_layers(self):
        """Return independent polylines, never close L routes or bridge gaps."""
        task = self.task_overlay
        frame = (self.telemetry.get("map") or {}).get("frame_id")
        expected = task.get("expected_track") or {}
        actual = task.get("actual_track") or {}
        layers = []
        if (self.overlay_live and expected.get("frame_id") == frame
                and expected.get("map_id") == task.get("map_id")):
            layers += [(segment.get("points", []), "#2866cc", True)
                       for segment in expected.get("cleaning_segments", [])]
            layers.append((expected.get("navigation_path", []), "#9c49b5", True))
        if (actual.get("frame_id") == frame and actual.get("map_id") == task.get("map_id")
                and actual.get("run_id") == task.get("run_id")):
            layers += [(segment, "#00877e", False) for segment in actual.get("segments", [])]
        return layers

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
            target = self.map_image.scaled(round(width*scale), round(height*scale),
                                           Qt.IgnoreAspectRatio, Qt.FastTransformation)
            painter.drawImage(round(ox), round(oy-height*scale), target)
            painter.setBrush(Qt.NoBrush)
            for points, color, dashed in self._track_layers():
                pen = QPen(QColor(color), 2 if dashed else 3)
                if dashed:
                    pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                # Invalid coordinates break a line rather than connecting over them.
                polyline = QPolygonF()
                for point in points:
                    if len(point) == 2 and all(math.isfinite(v) for v in point):
                        polyline.append(self.map_to_screen(*point))
                    else:
                        if len(polyline) > 1:
                            painter.drawPolyline(polyline)
                        polyline = QPolygonF()
                if len(polyline) > 1:
                    painter.drawPolyline(polyline)
                elif len(polyline) == 1:
                    painter.drawPoint(polyline[0])
            scan = self.telemetry.get("scan") or {}
            if scan.get("frame_id") == "map":
                painter.setPen(QPen(QColor("#df8b35"), 2))
                for x, y in scan.get("points", []):
                    if math.isfinite(x) and math.isfinite(y):
                        painter.drawPoint(self.map_to_screen(x, y))
            pose = self.telemetry.get("pose")
            if pose:
                self._arrow(painter, pose, "#00877e")
            if self.candidate:
                self._arrow(painter, self.candidate, "#9553c6")
        painter.setPen(QPen(QColor("#637783"), 1))
        painter.drawText(28, 22, "蓝虚线：清洗目标 · 紫虚线：当前导航 · 绿实线：历史轨迹 · 橙点：雷达")
