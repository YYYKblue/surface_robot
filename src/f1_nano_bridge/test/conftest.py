import ctypes as c
import os
from pathlib import Path
import subprocess

import pytest


@pytest.fixture(scope="session")
def firmware_library(tmp_path_factory):
    default = Path(__file__).resolve().parents[4] / "firmware"
    root = Path(os.environ.get("F1_FIRMWARE_DIR", default))
    if not (root / "tests/control_mux_harness.c").is_file():
        pytest.fail("Read-only firmware harness missing; set F1_FIRMWARE_DIR")
    output = tmp_path_factory.mktemp("firmware") / "control.so"
    subprocess.run([
        "cc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
        *[f"-I{root / p}" for p in ("USER", "SYSTEM/sbus", "HARDWARE/Motor")],
        *[str(root / p) for p in ("tests/control_mux_harness.c", "USER/host_protocol.c",
                                  "USER/control_mux.c", "USER/rc_control.c", "SYSTEM/sbus/sbus_protocol.c")],
        "-o", str(output)], check=True, capture_output=True)
    return output


@pytest.fixture
def firmware(firmware_library):
    lib = c.CDLL(str(firmware_library))
    lib.test_host.argtypes = [c.c_void_p, c.c_uint32, c.c_uint32]
    lib.test_remote.argtypes = [c.c_void_p, c.c_uint32]
    lib.test_tick.argtypes = [c.c_uint32]
    lib.test_get.argtypes = [c.c_uint32]
    lib.test_get.restype = c.c_uint32
    lib.test_report.argtypes = [c.c_uint32, c.c_void_p, c.c_uint16]
    lib.test_report.restype = c.c_uint16
    lib.test_init()
    return lib
