#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../../.." && pwd)"
repo_runtime_dir="${repo_root}/.venv/j6-dm313"
deployed_runtime_dir="/home/car/go-m8010-robot-arm-v15-20a/.venv/j6-dm313"

if [[ -n "${J6_RUNTIME_DIR:-}" ]]; then
    runtime_dir="${J6_RUNTIME_DIR}"
elif [[ -x "${repo_runtime_dir}/bin/python" ]]; then
    runtime_dir="${repo_runtime_dir}"
elif [[ -x "${deployed_runtime_dir}/bin/python" ]]; then
    # This is the isolated dmcan runtime used by the production V15.30A GUI
    # launcher on the current Ubuntu host.  It is outside this repository, so
    # keep the fallback explicit and fail closed if it disappears.
    runtime_dir="${deployed_runtime_dir}"
else
    runtime_dir="${repo_runtime_dir}"
fi

if [[ ! -x "${runtime_dir}/bin/python" ]]; then
    echo "BLOCKED: missing isolated J6 runtime at ${runtime_dir}" >&2
    exit 4
fi

export LD_LIBRARY_PATH="${runtime_dir}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "${runtime_dir}/bin/python" "${script_dir}/$1" "${@:2}"
