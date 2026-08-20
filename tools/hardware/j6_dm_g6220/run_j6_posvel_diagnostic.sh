#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../../.." && pwd)"
runtime_dir="${repo_root}/.venv/j6-dm313"

if [[ ! -x "${runtime_dir}/bin/python" ]]; then
    echo "BLOCKED: missing isolated J6 runtime at ${runtime_dir}" >&2
    exit 4
fi

export LD_LIBRARY_PATH="${runtime_dir}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "${runtime_dir}/bin/python" "${script_dir}/$1" "${@:2}"
