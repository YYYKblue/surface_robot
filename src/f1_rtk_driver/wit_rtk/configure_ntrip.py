"""Interactively create private credentials without shell-history or ROS exposure."""

import argparse
from dataclasses import asdict
import getpass
import json
import os
from pathlib import Path
import tempfile

from wit_rtk.ntrip import Config


def save_private(path, config):
    path = Path(path).expanduser()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".ntrip-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(asdict(config), stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="~/.config/f1_rtk_driver/ntrip.json")
    parser.add_argument("--host", default="rtk.ntrip.qxwz.com")
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--mountpoint", default="RTCM32_GGB")
    options = parser.parse_args(args)
    try:
        username = input("NTRIP account: ").strip()
        password = getpass.getpass("NTRIP password (hidden): ")
        config = Config(options.host, options.port, options.mountpoint, username, password)
        path = save_private(options.output, config)
    except (OSError, ValueError, EOFError, KeyboardInterrupt):
        parser.exit(1, "Could not save NTRIP config; check input and destination permissions.\n")
    print(f"Saved private NTRIP configuration (mode 600): {path}")
    print("Restart the RTK service after configuring F1_RTK_NTRIP_CONFIG to this path.")
