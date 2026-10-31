"""Wait for synchronized, stable system time before opening the hardware bridge.

Does not set the clock or change bridge fault protections. A missing time source
keeps the service in start-pre; an administrative stop can interrupt the wait.
"""
import argparse
import math
import subprocess
import time


class ClockWindow:
    MIN_UNIX_NS = 1704067200 * 10**9  # 2024-01-01 UTC; reject unset RTC dates.
    MAX_OFFSET_CHANGE_NS = 20_000_000  # Stricter than the bridge's 50 ms guard.
    MAX_SAMPLE_SPAN_NS = 5_000_000
    MAX_SAMPLE_GAP_NS = 2_000_000_000

    def __init__(self, stable_seconds=5.0):
        self.stable_ns = int(stable_seconds * 10**9)
        self.since = self.offset = self.last = None

    def observe(self, synced, unix_ns, monotonic_ns, sample_span_ns=0):
        reason = None
        if not synced:
            reason = "waiting for NTPSynchronized=yes"
        elif unix_ns < self.MIN_UNIX_NS:
            reason = "waiting for a valid system date"
        elif sample_span_ns > self.MAX_SAMPLE_SPAN_NS:
            reason = "clock sampling delayed; restarting stability window"
        if reason:
            self.since = self.offset = self.last = None
            return False, reason
        offset = unix_ns - monotonic_ns
        if self.since is None:
            self.since, self.offset = monotonic_ns, offset
        elif (monotonic_ns <= self.last or
              monotonic_ns - self.last > self.MAX_SAMPLE_GAP_NS or
              abs(offset - self.offset) > self.MAX_OFFSET_CHANGE_NS):
            self.since, self.offset = monotonic_ns, offset
            self.last = monotonic_ns
            return False, "clock changed or observation interrupted; restarting stability window"
        self.last = monotonic_ns
        ready = monotonic_ns - self.since >= self.stable_ns
        return ready, "clock ready" if ready else "waiting for a stable clock"


def synchronized():
    try:
        result = subprocess.run(
            ["timedatectl", "show", "--property=NTPSynchronized", "--value"],
            capture_output=True, text=True, timeout=3, check=False)
        return result.returncode == 0 and result.stdout.strip() == "yes"
    except (OSError, subprocess.TimeoutExpired):
        return False


def wait_for_clock(stable_seconds=5.0, timeout=0.0):
    window = ClockWindow(stable_seconds)
    started = time.monotonic()
    last_log, last_reason = -math.inf, None
    while True:
        synced = synchronized()
        before = time.monotonic_ns()
        unix_ns = time.time_ns()
        after = time.monotonic_ns()
        ready, reason = window.observe(synced, unix_ns, (before + after) // 2, after - before)
        now = time.monotonic()
        if ready:
            print(f"Operator clock ready: NTP synchronized and stable for {stable_seconds:g}s", flush=True)
            return 0
        if reason != last_reason or now - last_log >= 30:
            print(f"Operator clock gate: {reason}; elapsed={now-started:.1f}s", flush=True)
            last_log, last_reason = now, reason
        if timeout and now - started >= timeout:
            print("Operator clock gate timed out; bridge was not started", flush=True)
            return 1
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stable-seconds", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="Diagnostic timeout in seconds; 0 waits until ready")
    args = parser.parse_args()
    if (not math.isfinite(args.stable_seconds) or args.stable_seconds < 1 or
            not math.isfinite(args.timeout) or args.timeout < 0):
        parser.error("stable-seconds must be >= 1 and timeout must be >= 0")
    return wait_for_clock(args.stable_seconds, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
