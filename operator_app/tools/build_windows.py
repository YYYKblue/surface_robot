"""Build the dual-control Windows distribution with a native --mock smoke test.

Run after the Windows pytest suite. Outputs stay under operator_app/dist/.
"""
import argparse
import ctypes
from ctypes import wintypes
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def smoke(exe):
    user32 = ctypes.windll.user32
    callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowTextLengthW.argtypes = [wintypes.HWND]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
    process = subprocess.Popen([str(exe), "--mock"], cwd=exe.parent)
    windows = []

    @callback_type
    def visit(hwnd, _):
        pid = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if pid.value == process.pid:
            title = ctypes.create_unicode_buffer(user32.GetWindowTextLengthW(hwnd) + 1)
            user32.GetWindowTextW(hwnd, title, len(title))
            if "水面 / 水下" in title.value:
                windows.append(hwnd)
        return True

    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and process.poll() is None and not windows:
            user32.EnumWindows(visit, 0)
            time.sleep(.1)
        if not windows:
            raise RuntimeError("packaged dual-control window did not appear")
        time.sleep(1)
        user32.PostMessageW(windows[0], 0x0010, 0, 0)  # WM_CLOSE, normal App shutdown.
        if process.wait(timeout=8) != 0:
            raise RuntimeError("packaged mock exited unsuccessfully")
        return "native dual-control window appeared; WM_CLOSE exit=0; mock only"
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tests-passed", type=int, required=True,
                        help="count from the just-completed Windows App pytest run")
    args = parser.parse_args()
    if sys.platform != "win32":
        parser.error("build with native Windows Python")
    app = Path(__file__).resolve().parents[1]
    output = app / "dist" / "dual-control"
    output.mkdir(parents=True, exist_ok=True)
    sources = sorted(app.glob("*.py")) + [app / "requirements.txt"]
    before = {path.name: sha256(path) for path in sources}
    command = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--noconsole",
               "--name", "F1Operator", "--distpath", str(output), "--workpath", str(output / "work"),
               "--specpath", str(output), "--paths", str(app), str(app / "main.py")]
    with (output / "build.log").open("w", encoding="utf-8") as log:
        subprocess.run(command, cwd=app, stdout=log, stderr=subprocess.STDOUT, check=True)
    if before != {path.name: sha256(path) for path in sources}:
        raise RuntimeError("App sources changed during packaging")
    directory = output / "F1Operator"
    result = smoke(directory / "F1Operator.exe")
    info = dict(built_at=datetime.now(timezone.utc).isoformat(), python=sys.version,
                packages={p: importlib.metadata.version(p) for p in
                          ("PySide6", "PyInstaller", "websocket-client")},
                windows_app_tests_passed=args.tests_passed, smoke=result,
                hardware_tested=False, nano_deployed=False, source_sha256=before)
    (directory / "BUILD_INFO.json").write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding="utf-8")
    shutil.copyfile(app / "README.md", directory / "APP_README.md")
    archive = Path(shutil.make_archive(str(output / "F1Operator-dual-control-windows-x64"),
                                      "zip", root_dir=output, base_dir="F1Operator"))
    (output / "SHA256SUMS.txt").write_text(
        f"{sha256(archive)}  {archive.name}\n{sha256(directory / 'F1Operator.exe')}  F1Operator/F1Operator.exe\n",
        encoding="ascii")
    print(json.dumps(dict(archive=str(archive), size=archive.stat().st_size, smoke=result,
                         sha256=sha256(archive)), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
