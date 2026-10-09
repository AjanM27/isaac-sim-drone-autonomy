#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
exec python3 docker/flight_control/parallel_pid_optimizer.py "$@"
