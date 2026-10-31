"""Bounded shutdown of only our own process group; subprocesses never use a shell."""
from pathlib import Path
import os
import signal
import subprocess
import time


class ManagedProcess:
    def __init__(self, command, log_path):
        self.log_path = str(log_path)
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, "ab") as output:
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self):
        return self.process.poll() is None

    def _group_alive(self):
        try:
            os.killpg(self.process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def stop(self):
        # First let launch forward SIGINT and reap children itself.
        if self.alive():
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
        for sig, seconds in ((signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            if not self._group_alive():
                break
            os.killpg(self.process.pid, sig)
            deadline = time.monotonic() + seconds
            while self._group_alive() and time.monotonic() < deadline:
                self.process.poll()
                time.sleep(.05)
        self.process.wait(timeout=2)


def export_map(stem, resolution, log_path):
    process = ManagedProcess(["ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
        "-pbstream_filename", stem + ".pbstream", "-map_filestem", stem,
        "-resolution", str(resolution)], log_path)
    try:
        code = process.process.wait(timeout=120)
        if code:
            raise RuntimeError(f"栅格导出失败（退出码 {code}），日志：{log_path}")
    finally:
        process.stop()
