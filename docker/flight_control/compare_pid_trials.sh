#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RESULTS="${ROOT}/simulation/room_quadcopter/results/pid_tuning"

if [[ $# -eq 0 ]]; then
  mapfile -t trial_files < <(find "${RESULTS}" -maxdepth 1 -type f -name '*.json' ! -name 'active_profile.json' | sort)
  set -- "${trial_files[@]}"
fi
python3 "${ROOT}/docker/flight_control/compare_pid_trials.py" "$@"
