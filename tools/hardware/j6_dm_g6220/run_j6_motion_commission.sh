#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../../.." && pwd)"
runtime_dir="${repo_root}/.venv/j6-dm313"

if [[ ! -x "${runtime_dir}/bin/python" ]]; then
  echo "BLOCKED: missing isolated J6 runtime: ${runtime_dir}" >&2
  exit 4
fi
if [[ ! -f "${runtime_dir}/lib/python3.13/site-packages/dmcan/dlls/libdm_device.so" ]]; then
  echo "BLOCKED: missing verified libdm_device.so runtime" >&2
  exit 4
fi

# The upstream native backend requires the conda C++ runtime (GLIBCXX_3.4.32).
# Scope the loader path to this one process; do not replace system libraries.
export LD_LIBRARY_PATH="${runtime_dir}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "${runtime_dir}/bin/python" "${script_dir}/j6_motion_commission.py" "$@"
