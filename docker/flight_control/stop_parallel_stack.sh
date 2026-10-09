#!/usr/bin/env bash
set -euo pipefail

docker rm -f isaac-pid-sitl-batch isaac-pid-mavros-batch >/dev/null 2>&1 || true
