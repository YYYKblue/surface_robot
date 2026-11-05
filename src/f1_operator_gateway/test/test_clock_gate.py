"""Verify startup gating with clock steps and unavailable synchronization."""
import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "deploy/wait-operator-clock.py"
SPEC = importlib.util.spec_from_file_location("operator_clock_gate", SCRIPT)
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)
BASE = 1790000000 * 10**9


def sample(window, second, offset=0, synced=True, span=0):
    mono = int(second * 10**9)
    return window.observe(synced, BASE + mono + offset, mono, span)[0]


def test_initial_sync_must_be_followed_by_a_full_stable_window():
    window = gate.ClockWindow()
    for second in range(20):
        assert not sample(window, second, synced=False)
    for second in range(20, 25):
        assert not sample(window, second)
    assert sample(window, 25)


@pytest.mark.parametrize("jump", [-100_000_000, 100_000_000])
def test_time_step_restarts_stability_window(jump):
    window = gate.ClockWindow()
    for second in range(5):
        assert not sample(window, second)
    for second in range(5, 10):
        assert not sample(window, second, offset=jump)
    assert sample(window, 10, offset=jump)


@pytest.mark.parametrize("condition", ["sync_lost", "slow_sample", "gap", "unset_date"])
def test_unreliable_observation_cannot_complete_window(condition):
    window = gate.ClockWindow()
    for second in range(5):
        sample(window, second)
    if condition == "sync_lost":
        assert not sample(window, 5, synced=False)
    elif condition == "slow_sample":
        assert not sample(window, 5, span=50_000_000)
    elif condition == "gap":
        assert not sample(window, 8)
    else:
        assert not window.observe(True, 5 * 10**9, 5 * 10**9)[0]
    for second in range(9, 13):
        assert not sample(window, second)
    assert sample(window, 14)


def test_small_slew_does_not_block_startup():
    window = gate.ClockWindow()
    for second in range(5):
        assert not sample(window, second, offset=second * 500_000)
    assert sample(window, 5, offset=2_500_000)


@pytest.mark.parametrize("returncode,output", [(0, "no\n"), (1, "yes\n"), (0, "")])
def test_ntp_query_is_fail_closed(monkeypatch, returncode, output):
    monkeypatch.setattr(gate.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=returncode, stdout=output))
    assert not gate.synchronized()


def test_missing_timedatectl_is_not_ready(monkeypatch):
    def missing(*args, **kwargs):
        raise FileNotFoundError()
    monkeypatch.setattr(gate.subprocess, "run", missing)
    assert not gate.synchronized()


def test_cli_times_out_without_launching_on_missing_sync(tmp_path):
    # Run the actual helper with only a simulated timedatectl on PATH.
    command = tmp_path / "timedatectl"
    command.write_text("#!/bin/sh\nprintf 'no\\n'\n")
    command.chmod(0o755)
    result = subprocess.run([sys.executable, str(SCRIPT), "--timeout", "1"],
                            env={"PATH": str(tmp_path)}, capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert "bridge was not started" in result.stdout
    assert "Operator clock ready" not in result.stdout


class FakeTime:
    def __init__(self, base=BASE, offset=lambda second: 0):
        self.second = 0.
        self.base, self.offset = base, offset

    def monotonic(self):
        return self.second

    def monotonic_ns(self):
        return round(self.second * 1e9)

    def time_ns(self):
        return self.base + self.monotonic_ns() + self.offset(self.second)

    def sleep(self, seconds):
        self.second += seconds


def test_offline_fallback_waits_grace_and_full_stability(monkeypatch, capsys):
    clock = FakeTime()
    monkeypatch.setattr(gate, "time", clock)
    monkeypatch.setattr(gate, "synchronized", lambda: False)
    assert gate.wait_for_clock(offline_after=20) == 0
    assert clock.second == 25
    assert "unverified date" in capsys.readouterr().out


def test_online_sync_does_not_wait_offline_grace(monkeypatch, capsys):
    clock = FakeTime()
    monkeypatch.setattr(gate, "time", clock)
    monkeypatch.setattr(gate, "synchronized", lambda: clock.second >= 2)
    assert gate.wait_for_clock(offline_after=20) == 0
    assert clock.second == 7
    assert "NTP synchronized" in capsys.readouterr().out


@pytest.mark.parametrize("jump", [-100_000_000, 100_000_000])
def test_late_correction_restarts_offline_window(monkeypatch, jump):
    clock = FakeTime(offset=lambda second: jump if second >= 23 else 0)
    monkeypatch.setattr(gate, "time", clock)
    monkeypatch.setattr(gate, "synchronized", lambda: False)
    assert gate.wait_for_clock(offline_after=20) == 0
    assert clock.second == 28


@pytest.mark.parametrize("base", [0, (1 << 31) * 10**9])
def test_offline_rejects_invalid_date(monkeypatch, base):
    clock = FakeTime(base=base)
    monkeypatch.setattr(gate, "time", clock)
    monkeypatch.setattr(gate, "synchronized", lambda: False)
    assert gate.wait_for_clock(timeout=8, offline_after=0) == 1


def test_offline_continuous_jumps_never_release_gate(monkeypatch):
    clock = FakeTime(offset=lambda second: int(second) * 100_000_000)
    monkeypatch.setattr(gate, "time", clock)
    monkeypatch.setattr(gate, "synchronized", lambda: False)
    assert gate.wait_for_clock(timeout=10, offline_after=0) == 1


def test_offline_dbus_timeout_does_not_block_stable_sampling(monkeypatch):
    clock = FakeTime()
    queries = []
    def unavailable():
        queries.append(clock.second)
        clock.sleep(3)
        return False
    monkeypatch.setattr(gate, "time", clock)
    monkeypatch.setattr(gate, "synchronized", unavailable)
    assert gate.wait_for_clock(offline_after=0) == 0
    assert clock.second == 8
    assert len(queries) == 1


def test_cli_offline_launches_without_ntp(tmp_path):
    command = tmp_path / "timedatectl"
    command.write_text("#!/bin/sh\nprintf 'no\\n'\n")
    command.chmod(0o755)
    result = subprocess.run([sys.executable, str(SCRIPT), "--offline-after", "0",
                             "--stable-seconds", "1", "--timeout", "3"],
                            env={"PATH": str(tmp_path)}, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "offline local time" in result.stdout
