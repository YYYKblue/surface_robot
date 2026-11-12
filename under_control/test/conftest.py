import ctypes
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture(scope="session")
def h7_library(tmp_path_factory):
    root = Path(__file__).resolve().parents[2]
    firmware = root / "under/H7firmware"
    if not (firmware / "Communication/Src/nano_comm.c").is_file():
        pytest.skip("H7 firmware sources are not installed in this checkout")
    output = tmp_path_factory.mktemp("h7-native") / "h7.so"
    subprocess.run(["gcc", "-shared", "-fPIC", "-std=c99", "-Wall", "-Wextra", "-Werror",
        "-I" + str(firmware / "Communication/Inc"), "-I" + str(firmware / "Motor/Inc"),
        str(Path(__file__).with_name("h7_harness.c")),
        str(firmware / "Communication/Src/nano_comm.c"), str(firmware / "Motor/Src/motor.c"),
        "-o", str(output)], check=True)
    return output


@pytest.fixture
def h7(h7_library, tmp_path):
    # Every test loads new static firmware state, not a cached dlopen handle.
    path = tmp_path / "h7.so"
    shutil.copyfile(h7_library, path)
    lib = ctypes.CDLL(str(path))
    lib.test_init()
    return lib
