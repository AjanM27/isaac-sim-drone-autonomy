#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
print_runtime=0
if [[ "${1:-}" == "--print-runtime" ]]; then
  print_runtime=1
  shift
fi
if [[ -f "${ROOT}/.isaac-sim-eula-accepted" ]]; then
  export OMNI_KIT_ACCEPT_EULA=YES
fi

if [[ -n "${ISAAC_SIM_PYTHON:-}" ]]; then
  [[ -x "${ISAAC_SIM_PYTHON}" ]] || {
    echo "ISAAC_SIM_PYTHON is not executable: ${ISAAC_SIM_PYTHON}" >&2
    exit 1
  }
  ((print_runtime)) && { printf '%s\n' "${ISAAC_SIM_PYTHON}"; exit 0; }
  exec "${ISAAC_SIM_PYTHON}" "$@"
fi

if [[ -n "${ISAAC_SIM_ROOT:-}" && -x "${ISAAC_SIM_ROOT}/python.sh" ]]; then
  ((print_runtime)) && { printf '%s\n' "${ISAAC_SIM_ROOT}/python.sh"; exit 0; }
  exec "${ISAAC_SIM_ROOT}/python.sh" "$@"
fi

for candidate in \
  "${ROOT}/.venv/isaacsim/bin/python" \
  "${HOME}/robotics/simulators/isaac_sim/python.sh" \
  "${HOME}/isaacsim/python.sh" \
  "${HOME}/isaac_sim/python.sh" \
  "${HOME}/.local/share/ov/pkg/isaac-sim/python.sh"; do
  if [[ -x "${candidate}" ]]; then
    if [[ "${candidate}" == */python.sh ]]; then
      ISAAC_SIM_ROOT="$(cd "$(dirname "${candidate}")" && pwd)"
      export ISAAC_SIM_ROOT
    fi
    ((print_runtime)) && { printf '%s\n' "${candidate}"; exit 0; }
    exec "${candidate}" "$@"
  fi
done

cat >&2 <<EOF
Isaac Sim's Python runtime was not found.

Run the project bootstrap:
  ./scripts/setup_ubuntu.sh --accept-eula

Or point to an existing installation:
  export ISAAC_SIM_ROOT=/absolute/path/to/isaac_sim
EOF
exit 1
