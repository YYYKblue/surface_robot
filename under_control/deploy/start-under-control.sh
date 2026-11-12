#!/usr/bin/env bash
set -euo pipefail
: "${UNDER_SERIAL_PORT:?Set the H7 USB-to-RS485 device in /etc/under-control.env}"
under_args=(--bind "${UNDER_BIND:-127.0.0.1}" --port "${UNDER_PORT:-8766}" --serial-port "$UNDER_SERIAL_PORT")
case "${UNDER_MOTION_ENABLED:-false}" in
  true) under_args+=(--motion-enabled) ;;
  false) ;;
  *) echo 'UNDER_MOTION_ENABLED must be true or false' >&2; exit 2 ;;
esac
exec /usr/bin/python3 -m under_control "${under_args[@]}"
