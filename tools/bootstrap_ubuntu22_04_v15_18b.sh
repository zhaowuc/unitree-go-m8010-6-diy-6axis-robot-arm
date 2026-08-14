#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash tools/bootstrap_ubuntu22_04_v15_18b.sh [--vm-gui]

Default: verify the V15.18B final pure-simulation handoff on Ubuntu/VMware.
--vm-gui: after headless verification, probe the VMware console X session
          and run the V15.18B read-only visualization. Close it to return.

Optional environment:
  V15_18B_VENV_DIR  Python virtual environment (default: <repo>/.venv)
  V15_18A_PYTHON    Optional frozen V15.18A Python; when omitted the historical
                    runtime replay remains PENDING_ON_TARGET_HOST
EOF
}

print_frozen_status() {
  printf '%s\n' 'V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION'
  printf '%s\n' 'PURE_SIMULATION_PHASE = COMPLETE'
  printf '%s\n' 'NEXT_PHASE = V15.19 REAL_HARDWARE_READONLY_BRINGUP'
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  printf '%s\n' 'BOOTSTRAP=FAIL' >&2
  printf '%s\n' 'UBUNTU_RUNTIME_VERIFICATION=FAIL_ON_TARGET_HOST' >&2
  exit 1
}

unhandled_error() {
  rc=$?
  trap - ERR
  printf 'ERROR: bootstrap command failed with rc=%d\n' "${rc}" >&2
  printf '%s\n' 'BOOTSTRAP=FAIL' >&2
  printf '%s\n' 'UBUNTU_RUNTIME_VERIFICATION=FAIL_ON_TARGET_HOST' >&2
  exit "${rc}"
}

trap unhandled_error ERR

note() {
  printf '\n==> %s\n' "$*"
}

need_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

vm_gui=0
while (($#)); do
  case "$1" in
    --vm-gui)
      vm_gui=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unknown argument: $1"
      ;;
  esac
  shift
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
ROS_WS="${REPO_ROOT}/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
REQUIREMENTS="${REPO_ROOT}/requirements-sim-v15_18b.txt"
VENV_DIR="${V15_18B_VENV_DIR:-${REPO_ROOT}/.venv}"
ROS_SETUP=/opt/ros/humble/setup.bash

cd -- "${REPO_ROOT}"

note "Checking Ubuntu 22.04 and required commands"
[[ -r /etc/os-release ]] || die "/etc/os-release is missing"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || \
  die "Ubuntu 22.04 is required; found ID=${ID:-unknown} VERSION_ID=${VERSION_ID:-unknown}"

need_command git
need_command python3
need_command awk
need_command sha256sum
git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "not inside a Git worktree"
[[ "$(git rev-parse --show-toplevel)" == "${REPO_ROOT}" ]] || \
  die "script must run from its own repository worktree"
[[ -f "${REQUIREMENTS}" ]] || die "missing requirements-sim-v15_18b.txt"

python3 - <<'PY' || die "Python 3.10 or newer is required"
import sys
raise SystemExit(0 if sys.version_info >= (3, 10) else 1)
PY

[[ -r "${ROS_SETUP}" ]] || die \
  "ROS 2 Humble is missing. Install Humble for Ubuntu 22.04 from the official ROS documentation; this script does not auto-install ROS."

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
[[ "${ROS_DISTRO:-}" == "humble" ]] || die "ROS_DISTRO is not humble"

need_command colcon
need_command rosdep

note "Creating or reusing Python virtual environment: ${VENV_DIR}"
if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  python3 -m venv "${VENV_DIR}" || die \
    "venv creation failed; install the Ubuntu python3-venv package"
fi
set +u
# shellcheck disable=SC1090
source "${VENV_DIR}/bin/activate"
set -u

python -m pip install --disable-pip-version-check -r "${REQUIREMENTS}"
python - <<'PY'
import mujoco
import numpy

assert mujoco.__version__ == "3.11.0", mujoco.__version__
assert numpy.__version__ == "2.2.6", numpy.__version__
print(f"PYTHON={__import__('sys').version.split()[0]}")
print(f"MUJOCO={mujoco.__version__}")
print(f"NUMPY={numpy.__version__}")
PY

note "Resolving ROS dependencies"
rosdep install \
  --from-paths "${ROS_WS}/src" \
  --ignore-src -r -y \
  --rosdistro humble || die \
  "rosdep failed. Run 'sudo rosdep init' once (if needed), then 'rosdep update', and retry."

note "Building the self-contained ROS 2 workspace"
pushd "${ROS_WS}" >/dev/null
colcon build --symlink-install
popd >/dev/null

set +u
# shellcheck disable=SC1090
source "${ROS_WS}/install/setup.bash"
set -u

for package_name in \
  go_m8010_arm_description \
  go_m8010_arm_mujoco_bridge \
  go_m8010_arm_v15_14_description \
  go_m8010_arm_v15_14_moveit_config \
  go_m8010_arm_v15_14_qa; do
  ros2 pkg prefix "${package_name}" >/dev/null || \
    die "ROS package was not discoverable after build: ${package_name}"
done

note "Running fail-closed repository and MuJoCo compile/readback verification"
python "${REPO_ROOT}/tools/verify_repository_handoff_v15_18b.py" || \
  die "repository handoff verification failed"

note "Checking the frozen V15.18A authority hash"
v15_18a_report="${REPO_ROOT}/V15_18A_静态重力与重力矩验收.json"
[[ -f "${v15_18a_report}" ]] || die "V15.18A authority report is missing"
v15_18a_sha="$({ sha256sum "${v15_18a_report}" || true; } | awk '{print $1}')"
[[ "${v15_18a_sha}" == "640e9104cabd0e2548e07bdd54d5cdb2c66dbdf86a7981eff4d8c12e55b9dfe2" ]] || \
  die "V15.18A frozen authority SHA256 mismatch"
printf 'V15_18A_FROZEN_AUTHORITY_SHA256=%s\n' "${v15_18a_sha}"

ubuntu_runtime_status=PENDING_ON_TARGET_HOST
if [[ -n "${V15_18A_PYTHON:-}" ]]; then
  [[ -x "${V15_18A_PYTHON}" ]] || die \
    "V15_18A_PYTHON is not executable: ${V15_18A_PYTHON}"
  note "Running the frozen V15.18A validator with its dedicated historical environment"
  "${V15_18A_PYTHON}" \
    "${REPO_ROOT}/tools/validate_static_gravity_v15_18.py" \
    --check --mujoco-python "${V15_18A_PYTHON}" || die \
    "V15.18A historical validator check failed"
  ubuntu_runtime_status=PASS
else
  printf '%s\n' 'V15_18A_RUNTIME_CHECK=PENDING_ON_TARGET_HOST'
fi

note "Checking V15.18B numerical-integrator attribution evidence (headless, no ROS nodes)"
python "${REPO_ROOT}/tools/audit_passive_gravity_v15_18b.py" --check || die \
  "V15.18B audit evidence did not reproduce byte-for-byte"
python "${REPO_ROOT}/tools/validate_passive_gravity_v15_18b.py" \
  --check --mujoco-python "${VENV_DIR}/bin/python" || die \
  "V15.18B independent validator evidence check failed"

if ((vm_gui)); then
  note "Probing VMware Ubuntu console GUI"
  if grep -Eqi '(microsoft|wsl)' /proc/sys/kernel/osrelease /proc/version 2>/dev/null; then
    die "WSL is not an accepted VMware GUI target"
  fi
  need_command systemd-detect-virt
  [[ "$(systemd-detect-virt --vm 2>/dev/null || true)" == "vmware" ]] || \
    die "--vm-gui requires a VMware guest"
  console_display="$(
    who | awk -v user="$(id -un)" \
      '$1 == user {
        for (i = 2; i <= NF; ++i) {
          value = $i
          gsub(/[()]/, "", value)
          if (value ~ /^:[0-9]+$/) {print value; exit}
        }
      }'
  )"
  [[ -n "${console_display}" ]] || die \
    "no VMware guest desktop DISPLAY was found for this user; open/login to the Ubuntu console first"
  if [[ -n "${DISPLAY:-}" && "${DISPLAY}" != "${console_display}" ]]; then
    printf 'IGNORED_NON_CONSOLE_DISPLAY=%s\n' "${DISPLAY}"
  fi
  export DISPLAY="${console_display}"

  unset XAUTHORITY
  runtime_dir="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  for candidate in \
    "${runtime_dir}/gdm/Xauthority" \
    "${runtime_dir}/Xauthority" \
    "${HOME}/.Xauthority"; do
    if [[ -r "${candidate}" ]]; then
      export XAUTHORITY="${candidate}"
      break
    fi
  done

  printf 'DISPLAY=%s\n' "${DISPLAY}"
  printf 'XAUTHORITY=%s\n' "${XAUTHORITY:-<unset>}"
  [[ -n "${XAUTHORITY:-}" ]] || die \
    "XAUTHORITY was not found for the VMware guest desktop session"
  if [[ ! -r "${XAUTHORITY}" ]]; then
    die "XAUTHORITY is not readable: ${XAUTHORITY}"
  fi
  need_command xdpyinfo
  xdpyinfo >/dev/null 2>&1 || die \
    "cannot access the VMware console X display; verify DISPLAY/XAUTHORITY for the logged-in guest desktop user"

  note "Launching V15.18B read-only visualization; close the viewer to finish"
  python "${REPO_ROOT}/tools/audit_passive_gravity_v15_18b.py" --visualize
fi

note "V15.18B final pure-simulation handoff verification"
printf 'HEAD=%s\n' "$(git rev-parse HEAD)"
printf 'BRANCH=%s\n' "$(git branch --show-current)"
printf 'VMWARE_GUI=%s\n' "$([[ ${vm_gui} -eq 1 ]] && printf PASS || printf NOT_REQUESTED)"
print_frozen_status
printf 'UBUNTU_RUNTIME_VERIFICATION=%s\n' "${ubuntu_runtime_status}"
printf '%s\n' 'BOOTSTRAP=PASS'
trap - ERR
exit 0
