#!/usr/bin/env bash
# Read-only observations: never acquire control, start a task or write the tty.
set -uo pipefail
health_workspace="${F1_WORKSPACE:-/home/nvidia/surface_ws}"
health_user="${F1_USER:-nvidia}"
health_log=/var/log/usv_boot_health.log
health_tmp="$(mktemp /var/log/.usv_boot_health.XXXXXX)" || exit 1
trap 'rm -f "$health_tmp"' EXIT

observe() {
  local health_title="$1" health_timeout="$2" health_rc
  shift 2
  printf '\n[%s]\n' "$health_title"
  timeout -k 1s "$health_timeout" "$@"
  health_rc=$?
  if [ "$health_rc" -ne 0 ]; then
    printf 'check_exit_code=%s (unavailable, timed out or unhealthy; see output)\n' "$health_rc"
  fi
  return 0
}

{
  printf 'USV boot health: %s\nworkspace=%s\n' "$(date -Is)" "$health_workspace"
  observe clock 3s timedatectl show -p NTPSynchronized -p NTP -p TimeUSec
  observe network 3s ip -br addr
  observe routes 3s ip route
  observe services 4s systemctl show pavo2-scan.service f1-operator.service \
    nano-bridge.service pavo2s-scan.service -p Id -p UnitFileState -p ActiveState \
    -p SubState -p NRestarts -p Result -p ExecStart --no-pager
  observe failed_units 3s systemctl --failed --no-pager
  observe websocket_listener 3s ss -ltnp 'sport = :8765'
  observe storage 3s df -h / "$health_workspace"
  printf '\n[map_files]\n'
  find "$health_workspace/maps" -maxdepth 1 -type f \
    \( -name '*.pbstream' -o -name '*.yaml' -o -name '*.pgm' \) -printf '%f %s bytes\n'
  printf '\n[configured_devices]\n'
  # Derive addresses from installed configuration rather than keeping old IPs here.
  python3 - "$health_workspace" <<'PY'
from pathlib import Path
import json
import os
import subprocess
import sys
import yaml

root = Path(sys.argv[1]) / 'install'
for package, filename, node in [('f1_lidar_driver', 'pavo2.yaml', 'pavo2_scan_node'),
                                 ('f1_nano_bridge', 'bridge.yaml', 'nano_bridge')]:
    config = root / package / 'share' / package / 'config' / filename
    try:
        values = yaml.safe_load(config.read_text())[node]['ros__parameters']
        if package == 'f1_lidar_driver':
            print(json.dumps({k: values[k] for k in ('lidar_ip', 'lidar_port', 'local_ip')}), flush=True)
            subprocess.run(['ping', '-c', '2', '-W', '1', values['lidar_ip']], timeout=4, check=False)
        else:
            port = Path(values['port'])
            print(json.dumps(dict(serial_port=str(port), resolved_port=str(port.resolve()),
                                  present=port.exists())), flush=True)
            # Check access as the service user; do not open the device.
            subprocess.run(['runuser', '-u', os.environ.get('F1_USER', 'nvidia'), '--',
                            'test', '-r', str(port)], check=True, timeout=2)
            subprocess.run(['runuser', '-u', os.environ.get('F1_USER', 'nvidia'), '--',
                            'test', '-w', str(port)], check=True, timeout=2)
    except Exception as error:
        print(f'{package}: {error}', flush=True)
PY
  observe ros_observation 20s runuser -u "$health_user" -- env \
    "F1_WORKSPACE=$health_workspace" "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-33}" \
    "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    PYTHONUNBUFFERED=1 /bin/bash --noprofile --norc -c \
    'source /opt/ros/humble/setup.bash && source "$F1_WORKSPACE/install/local_setup.bash" && exec python3 -' <<'PY'
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from f1_interfaces.msg import ControlState
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('usv_boot_health_observer')
report = dict(scans=0, scan_frames=[], control_messages=0, control=None)

def scan(message):
    report['scans'] += 1
    if message.header.frame_id not in report['scan_frames']:
        report['scan_frames'].append(message.header.frame_id)

def control(message):
    report['control_messages'] += 1
    report['control'] = {key: getattr(message, key) for key in (
        'state', 'reason', 'connected', 'motion_authorized', 'source', 'owner',
        'armed', 'ctrl_age_sec', 'ack_age_sec', 'imu_age_sec', 'last_stop_reason')}

node.create_subscription(LaserScan, '/pavo2_scan', scan, qos_profile_sensor_data)
node.create_subscription(ControlState, '/f1/control_state', control, 10)
deadline = time.monotonic() + 8
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
report['scan_publishers'] = node.count_publishers('/pavo2_scan')
report['control_publishers'] = node.count_publishers('/f1/control_state')
print(json.dumps(report, ensure_ascii=False, indent=2))
node.destroy_node()
rclpy.shutdown()
PY
  printf '\nSnapshot complete. This is an observation, not a motion test.\n'
} > "$health_tmp" 2>&1

chmod 0644 "$health_tmp" && mv -f "$health_tmp" "$health_log"
