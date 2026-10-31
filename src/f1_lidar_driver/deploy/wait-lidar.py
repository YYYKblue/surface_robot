"""Preserve the deployed 20-attempt startup probe; a ping failure is not fatal."""
import ipaddress
import os
from pathlib import Path
import subprocess
import time

import yaml


def main():
    workspace = Path(os.environ["F1_LIDAR_WORKSPACE"])
    config = Path(os.environ.get("F1_LIDAR_CONFIG") or
                  workspace / "install/f1_lidar_driver/share/f1_lidar_driver/config/pavo2.yaml")
    parameters = yaml.safe_load(config.read_text())["pavo2_scan_node"]["ros__parameters"]
    address = str(ipaddress.IPv4Address(parameters.get("lidar_ip", "10.10.10.121")))
    for _ in range(20):
        result = subprocess.run(["ping", "-c1", "-W1", address], check=False,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            return
        time.sleep(1)
    # Like the old ExecStartPre, leave actual connection validation to the SDK.


if __name__ == "__main__":
    main()
