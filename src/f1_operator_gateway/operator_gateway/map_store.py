"""Local map catalogue and recoverable snapshot publication (no ROS dependency)."""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import json
import math
import os
import uuid

import yaml


class MapStore:
    def __init__(self, workspace, timezone="Asia/Shanghai"):
        workspace = Path(workspace).expanduser()
        if not workspace.is_absolute():
            raise ValueError("workspace_root 必须是工作空间绝对路径")
        self.root = workspace.resolve() / "maps"
        if self.root.is_symlink():
            raise ValueError("maps 不能是工作空间外的符号链接")
        self.root.mkdir(parents=True, exist_ok=True)
        self.timezone = ZoneInfo(timezone)

    @staticmethod
    def name(value):
        if not isinstance(value, str):
            raise ValueError("地图名称必须是文本")
        value = value.strip()
        if len(value) > 80 or (value and not all(c.isalnum() or c in "_-" for c in value)):
            raise ValueError("地图名称限 80 字符，使用中文、字母、数字、下划线或短横线")
        if value.upper() in {"CON", "PRN", "AUX", "NUL", *[f"COM{i}" for i in range(1, 10)], *[f"LPT{i}" for i in range(1, 10)]}:
            raise ValueError("地图名称是 Windows 保留名称")
        return value

    def path(self, name, suffix=".pbstream"):
        if not name or self.name(name) != name:
            raise ValueError("无效地图名称")
        path = self.root / (name + suffix)
        if path.is_symlink() or path.resolve().parent != self.root.resolve():
            raise ValueError("地图文件不能是符号链接或位于 maps 外")
        return path

    def validate(self, name):
        import cv2
        for suffix in (".pbstream", ".yaml", ".pgm"):
            path = self.path(name, suffix)
            if not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f"缺失或空文件：{path.name}")
        meta = yaml.safe_load(self.path(name, ".yaml").read_text(encoding="utf-8"))
        if not isinstance(meta, dict):
            raise ValueError("地图 YAML 格式无效")
        resolution = float(meta["resolution"])
        origin = [float(v) for v in meta["origin"]]
        free, occupied = float(meta["free_thresh"]), float(meta["occupied_thresh"])
        if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
                or not all(math.isfinite(v) for v in origin) or not 0 <= free < occupied <= 1
                or meta.get("negate", 0) not in (0, 1)):
            raise ValueError("地图分辨率、原点或占据阈值无效")
        # Legacy exports may contain an old absolute path. Only read the sibling.
        if Path(str(meta.get("image", ""))).name != name + ".pgm":
            raise ValueError("YAML image 必须对应同名 PGM")
        pixels = cv2.imread(str(self.path(name, ".pgm")), cv2.IMREAD_GRAYSCALE)
        if pixels is None or not pixels.size:
            raise ValueError("地图 PGM 无法解析")
        return meta, pixels

    def catalog(self):
        names = sorted({p.stem for p in self.root.iterdir()
                        if not p.name.startswith(".") and p.suffix in (".pbstream", ".yaml", ".pgm")})
        result = []
        for name in names:
            entry = dict(name=name, valid=False, reason="", saved_at="", modified_ns=0)
            try:
                files = [self.path(name, suffix) for suffix in (".pbstream", ".yaml", ".pgm")]
                modified = max(p.stat().st_mtime_ns for p in files if p.exists())
                entry.update(modified_ns=modified, saved_at=datetime.fromtimestamp(
                    modified / 1e9, self.timezone).isoformat(timespec="seconds"))
                self.validate(name)
                entry["valid"] = True
            except Exception as exc:
                entry["reason"] = str(exc)
            result.append(entry)
        return sorted(result, key=lambda x: (-x["modified_ns"], x["name"]))

    def preview(self, name):
        import cv2
        import numpy as np
        meta, pixels = self.validate(name)
        height, width = pixels.shape
        # Limit wire payload and preserve square pixels / extent by integer subsampling.
        stride = max(1, math.ceil(max(width, height) / 512))
        pixels = pixels[::stride, ::stride]
        probability = pixels.astype(np.float32) / 255.
        if not meta.get("negate", 0):
            probability = 1. - probability
        cells = np.full(pixels.shape, -1, dtype=np.int8)
        cells[probability < float(meta["free_thresh"])] = 0
        cells[probability > float(meta["occupied_thresh"])] = 100
        return dict(frame_id="map", revision=name + ":" + str(self.path(name, ".pgm").stat().st_mtime_ns),
                    resolution=float(meta["resolution"]) * stride,
                    width=cells.shape[1], height=cells.shape[0],
                    origin=dict(zip(("x", "y", "yaw"), meta["origin"])),
                    data=np.flipud(cells).reshape(-1).tolist())

    def reserve(self, requested):
        base = self.name(requested) or datetime.now(self.timezone).strftime("%Y%m%d_%H%M%S")
        for index in range(10000):
            name = base if index == 0 else f"{base[:74]}_{index:02d}"
            # Reserve across processes; unfinished exports stay hidden/recoverable.
            marker = self.root / ("." + name + ".reserved")
            try:
                fd = os.open(marker, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            except FileExistsError:
                continue
            os.close(fd)
            if any((self.root / (name + s)).exists() for s in (".pbstream", ".yaml", ".pgm")):
                marker.unlink()
                continue
            directory = self.root / (".snapshot-" + uuid.uuid4().hex)
            directory.mkdir()
            transaction = dict(name=name, directory=str(directory), marker=str(marker),
                               stem=str(directory / name))
            (directory / "snapshot.json").write_text(json.dumps(transaction, ensure_ascii=False), encoding="utf-8")
            return transaction
        raise ValueError("地图重名过多，请换一个名称")

    def publish(self, transaction):
        name = transaction["name"]
        directory = Path(transaction["directory"])
        # Validate in place before publishing. Rewrite temporary/absolute image path.
        yaml_path = directory / (name + ".yaml")
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        meta["image"] = name + ".pgm"
        yaml_path.write_text(yaml.safe_dump(meta, allow_unicode=True), encoding="utf-8")
        staged = object.__new__(MapStore)
        staged.root, staged.timezone = directory, self.timezone
        staged.validate(name)
        destinations = [self.path(name, s) for s in (".pbstream", ".pgm", ".yaml")]
        if any(p.exists() for p in destinations):
            raise ValueError("目标地图已存在，未覆盖；快照保留在 " + str(directory))
        linked = []
        try:
            # Hard links do not overwrite. YAML last; incomplete triplets are never loadable.
            for target in destinations:
                os.link(directory / target.name, target)
                linked.append(target)
        except Exception:
            for target in linked:
                target.unlink()
            raise
        for child in directory.iterdir():
            child.unlink()
        directory.rmdir()
        Path(transaction["marker"]).unlink()
        return dict(name=name, paths=[str(p) for p in destinations])

    def recovered(self):
        """Expose retained snapshots after an export failure or gateway restart."""
        result = []
        for path in self.root.glob(".snapshot-*/snapshot.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                directory = path.parent
                name = self.name(data["name"])
                if (directory.is_symlink() or directory.resolve().parent != self.root.resolve()
                        or not name):
                    continue
                pbstream = directory / (name + ".pbstream")
                if pbstream.is_file() and not pbstream.is_symlink() and pbstream.stat().st_size:
                    result.append(dict(name=name, directory=str(directory),
                        stem=str(directory / name), marker=str(self.root / ("." + name + ".reserved"))))
            except (OSError, ValueError, KeyError):
                continue
        return result
