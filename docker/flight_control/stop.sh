#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec docker compose -f "${ROOT}/docker/flight_control/compose.yml" down
