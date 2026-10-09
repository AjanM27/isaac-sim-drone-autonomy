#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

exec "${root}/scripts/isaac_python.sh" \
  "${root}/simulation/room_quadcopter/scripts/vendor_hospital.py" "$@"
