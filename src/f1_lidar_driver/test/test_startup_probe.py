"""Exercise service startup policy without network access or real sleeps."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


SPEC = importlib.util.spec_from_file_location(
    "wait_lidar", Path(__file__).resolve().parents[1] / "deploy/wait-lidar.py")
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


@pytest.mark.parametrize("custom_config", [False, True])
def test_probe_uses_driver_config_and_stops_after_success(tmp_path, monkeypatch, custom_config):
    monkeypatch.setenv("F1_LIDAR_WORKSPACE", str(tmp_path))
    config = tmp_path / "install/f1_lidar_driver/share/f1_lidar_driver/config/pavo2.yaml"
    if custom_config:
        config = tmp_path / "custom.yaml"
        monkeypatch.setenv("F1_LIDAR_CONFIG", str(config))
    else:
        monkeypatch.delenv("F1_LIDAR_CONFIG", raising=False)
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(yaml.safe_dump({"pavo2_scan_node": {"ros__parameters": {
        "lidar_ip": "192.0.2.123"}}}))
    calls, sleeps = [], []

    def ping(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0 if len(calls) == 2 else 1)

    monkeypatch.setattr(probe.subprocess, "run", ping)
    monkeypatch.setattr(probe.time, "sleep", sleeps.append)
    probe.main()
    assert calls == [["ping", "-c1", "-W1", "192.0.2.123"]] * 2
    assert sleeps == [1]


def test_unreachable_device_exhausts_probe_but_allows_driver_start(tmp_path, monkeypatch):
    config = tmp_path / "custom.yaml"
    config.write_text("pavo2_scan_node:\n  ros__parameters:\n    lidar_ip: 192.0.2.123\n")
    monkeypatch.setenv("F1_LIDAR_WORKSPACE", str(tmp_path))
    monkeypatch.setenv("F1_LIDAR_CONFIG", str(config))
    calls, sleeps = [], []

    def ping(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(probe.subprocess, "run", ping)
    monkeypatch.setattr(probe.time, "sleep", sleeps.append)
    assert probe.main() is None
    assert len(calls) == 20
    assert sleeps == [1] * 20
