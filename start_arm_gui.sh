#!/usr/bin/env bash
set -Eeuo pipefail
IFS=$'\n\t'

PREBUILD_ONLY=0
if [[ "${1:-}" == "--prebuild-only" && "$#" -eq 1 ]]; then
  PREBUILD_ONLY=1
elif [[ "$#" -ne 0 ]]; then
  printf '用法：%s [--prebuild-only]\n' "${0##*/}" >&2
  exit 2
fi

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROS_WS="$REPO_ROOT/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
GUI_SOURCE="$ROS_WS/src/go_m8010_arm_gui"
MODEL="$REPO_ROOT/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
MODEL_SHA256="5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
GUI_VENV="$REPO_ROOT/.venv/arm-gui"
GUI_PY="$GUI_VENV/bin/python"
J6_PY="${J6_PYTHON:-/home/car/go-m8010-robot-arm-v15-20a/.venv/j6-dm313/bin/python}"
SDK_ROOT="${UNITREE_MOTOR_SDK_ROOT:-/home/car/vendor/unitree_actuator_sdk}"
TOOLS_BUILD="$ROS_WS/build/v15_30a_gui_tools"
GO_BINARY="$TOOLS_BUILD/v15_30a_gui_go_controller"
GO_BUILD_MANIFEST="$GO_BINARY.build.json"
GO_CONTROLLER_SOURCE="$REPO_ROOT/tools/hardware/v15_30a_gui_go_controller.cpp"
THERMAL_CONFIG="$ROS_WS/src/go_m8010_arm_hardware/config/thermal_limits.yaml"
THERMAL_CONFIG_SHA256="1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
GRAVITY_CONFIG="$ROS_WS/src/go_m8010_arm_hardware/config/gravity_control.yaml"
GO_SDK_INCLUDE="$SDK_ROOT/include"
GO_SDK_LIBRARY="$SDK_ROOT/lib/libUnitreeMotorSDK_Linux64.so"
GO_SDK_HEADERS=(
  "$GO_SDK_INCLUDE/unitreeMotor/unitreeMotor.h"
  "$GO_SDK_INCLUDE/unitreeMotor/include/motor_msg_GO-M8010-6.h"
  "$GO_SDK_INCLUDE/unitreeMotor/include/motor_msg_A1B1.h"
  "$GO_SDK_INCLUDE/serialPort/SerialPort.h"
  "$GO_SDK_INCLUDE/serialPort/include/errorClass.h"
  "$GO_SDK_INCLUDE/IOPort/IOPort.h"
)
RUN_TOKEN="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${ARM_GUI_RUN_DIRECTORY:-$REPO_ROOT/logs/arm_gui/$RUN_TOKEN}"
STATE_DIR="$REPO_ROOT/.runtime/v15_30a_gui"
INITIAL_POSE="$STATE_DIR/initial_pose.json"
PERSISTENT_ZERO="$STATE_DIR/persistent_software_zero.json"
RECOVERY_BRANCH_HINTS="$STATE_DIR/recovery_branch_hints.json"
J2_SESSION_REFERENCE="${J2_SESSION_REFERENCE_FILE:-}"
J2_SESSION_LAUNCH_PERMIT="${J2_SESSION_LAUNCH_PERMIT_FILE:-}"
J2_POWER_SESSION_ID="${J2_POWER_SESSION_ID:-}"
GO_AUX_SESSION_REFERENCE="${GO_AUX_SESSION_REFERENCE_FILE:-}"
GO_AUX_J1_LAUNCH_PERMIT="${GO_AUX_J1_LAUNCH_PERMIT_FILE:-}"
GO_AUX_J345_LAUNCH_PERMIT="${GO_AUX_J345_LAUNCH_PERMIT_FILE:-}"
GO_AUX_POWER_SESSION_ID="${GO_AUX_POWER_SESSION_ID:-}"
GRAVITY_ANCHOR="${GRAVITY_ANCHOR_PATH:-}"
EMPIRICAL_ENVELOPE="${EMPIRICAL_VALIDATION_ENVELOPE_PATH:-}"
EMPIRICAL_ENVELOPE_SHA256="${EMPIRICAL_VALIDATION_ENVELOPE_SHA256:-}"
GRAVITY_ENABLED_FOR_HARDWARE="${GRAVITY_ENABLED_FOR_HARDWARE:-false}"
GRAVITY_SCALE_TARGET="${GRAVITY_SCALE_TARGET:-0.0}"
EXPECTED_GRAVITY_AUTHORITY_CLASS=""
EXPECTED_EMPIRICAL_ENVELOPE_ID=""
EXPECTED_EMPIRICAL_ENVELOPE_SHA256=""
EXPECTED_GRAVITY_ANCHOR_SHA256=""
EXPECTED_GRAVITY_SESSION_ID=""
EXPECTED_GRAVITY_STATE_INSTANCE_ID=""
J6_FEEDBACK_SESSION_ID=""
J6_FEEDBACK_STATE_INSTANCE_ID=""
J6_FEEDBACK_HANDOFF="${J6_FEEDBACK_HANDOFF_FILE:-}"
REUSE_RUNNING_ARM_GUI_CORE="${REUSE_RUNNING_ARM_GUI_CORE:-false}"
EXTERNAL_ARM_GUI_CORE_PID="${EXTERNAL_ARM_GUI_CORE_PID:-}"
GO_AUX_SESSION_REFERENCE_SHA256=""
GO_AUX_EXPECTED_WORKER_SHA256=""
GO_AUX_VALIDATOR="$REPO_ROOT/tools/hardware/v15_30a_validate_go_aux_session_bundle.py"
J2_SESSION_REFERENCE_SHA256=""
J2_EXPECTED_WORKER_SHA256=""
J2_SIGNED_LAUNCH_ATTEMPT=0
if [[ -n "$J2_SESSION_LAUNCH_PERMIT" ||
      -n "$GO_AUX_J1_LAUNCH_PERMIT" ||
      -n "$GO_AUX_J345_LAUNCH_PERMIT" ]]; then
  J2_SIGNED_LAUNCH_ATTEMPT=1
fi
# The user's persistent software zero is the photographed vertical pose.  The
# frozen CAD model has non-zero assembly offsets at J3/J4/J5, so its absolute
# qpos for that same physical pose is intentionally not all zero.  This value
# is display-only and must never be written into the motor/software-zero files.
MUJOCO_SOFTWARE_ZERO_DEG="0,90,-14.40,13.49,47.94,0"
STOP_TOOL="$REPO_ROOT/tools/hardware/v15_30a_gui_stop.py"
WORKER_SUPERVISOR_TOOL="$REPO_ROOT/tools/hardware/v15_30a_worker_supervisor_status.py"
WORKER_SUPERVISOR_STATUS="$RUN_DIR/worker_supervisor_status.json"
WORKER_SUPERVISOR_LOG="$RUN_DIR/worker_supervisor_status.log"

export PYTHONNOUSERSITE=1

ROS_PID=""
WORKER_PIDS=()
WORKER_NAMES=()
WORKER_LOGS=()
WORKER_PORTS=()
WORKER_BOUND=()
declare -A WORKER_PID_BY_DOMAIN=()
WORKER_SUPERVISOR_PID=""
CLEANED=0
HARDWARE_SESSION_STARTED=0

fail() {
  printf '启动已阻止：%s\n' "$*" >&2
  exit 2
}

managed_pid_is_live() {
  local pid=$1
  local state
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | awk 'NR==1 {print $1}')"
  [[ -n "$state" && "$state" != Z* ]]
}

stop_worker_supervisor_status() {
  [[ -n "$WORKER_SUPERVISOR_PID" ]] || return 0
  if managed_pid_is_live "$WORKER_SUPERVISOR_PID"; then
    kill -TERM "$WORKER_SUPERVISOR_PID" 2>/dev/null || true
    local deadline=$((SECONDS + 2))
    while managed_pid_is_live "$WORKER_SUPERVISOR_PID" && \
          (( SECONDS < deadline )); do
      sleep 0.05
    done
  fi
  if managed_pid_is_live "$WORKER_SUPERVISOR_PID"; then
    printf '警告：worker supervisor 心跳未响应 SIGTERM，状态将依靠超时失效。\n' >&2
    kill -KILL "$WORKER_SUPERVISOR_PID" 2>/dev/null || true
  fi
  wait "$WORKER_SUPERVISOR_PID" 2>/dev/null || true
}

cleanup() {
  local original_status=$?
  # --prebuild-only never installs this trap, but keep the guard fail-closed if
  # a future refactor accidentally does.  In particular, do not emit the UDP
  # stop/BRAKE datagrams from a build-only invocation.
  if [[ "$PREBUILD_ONLY" -eq 1 ]]; then
    trap - EXIT INT TERM HUP
    return "$original_status"
  fi
  if [[ "$CLEANED" -ne 0 ]]; then
    return
  fi
  CLEANED=1
  trap - EXIT INT TERM HUP
  set +e
  if [[ "$HARDWARE_SESSION_STARTED" -eq 1 ]]; then
    printf '\n正在停止轨迹并执行制动／失能清理…\n'
    python3 "$STOP_TOOL" --repeat 5 --interval 0.01 >/dev/null 2>&1
  else
    printf '\n启动前检查未通过；尚未启动任何硬件 worker，不发送 UDP 制动报文。\n'
  fi
  # Publish one explicit all-domain supervisor_stopping interlock before any
  # managed process is asked to exit.  A failed final write still goes stale
  # within the state node's one-second freshness window.
  stop_worker_supervisor_status
  if [[ -n "$ROS_PID" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    kill -INT "$ROS_PID" 2>/dev/null
  fi
  if [[ "$HARDWARE_SESSION_STARTED" -eq 1 ]]; then
    python3 "$STOP_TOOL" --repeat 50 --interval 0.02 >/dev/null 2>&1
  fi
  sleep 0.6
  local pid
  for pid in "${WORKER_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null
    fi
  done
  local deadline=$((SECONDS + 5))
  while (( SECONDS < deadline )); do
    local alive=0
    for pid in "${WORKER_PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then alive=1; fi
    done
    if [[ "$alive" -eq 0 ]]; then break; fi
    sleep 0.1
  done
  local forced=0
  for pid in "${WORKER_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      forced=1
      kill -KILL "$pid" 2>/dev/null
    fi
    wait "$pid" 2>/dev/null
  done
  if [[ -n "$ROS_PID" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    local ros_int_deadline=$((SECONDS + 2))
    while kill -0 "$ROS_PID" 2>/dev/null && (( SECONDS < ros_int_deadline )); do
      sleep 0.1
    done
  fi
  if [[ -n "$ROS_PID" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    printf 'ROS2 进程未响应 SIGINT，升级为 SIGTERM…\n' >&2
    kill -TERM "$ROS_PID" 2>/dev/null
    local ros_term_deadline=$((SECONDS + 3))
    while kill -0 "$ROS_PID" 2>/dev/null && (( SECONDS < ros_term_deadline )); do
      sleep 0.1
    done
  fi
  if [[ -n "$ROS_PID" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    forced=1
    printf '严重警告：ROS2 进程未响应 SIGTERM，即将强制终止。\n' >&2
    kill -KILL "$ROS_PID" 2>/dev/null
  fi
  if [[ -n "$ROS_PID" ]]; then wait "$ROS_PID" 2>/dev/null; fi
  local index
  for index in "${!WORKER_NAMES[@]}"; do
    if [[ "${WORKER_NAMES[$index]}" == "J6" ]]; then
      grep -q 'J6_FINAL_DISABLED=PASS' "${WORKER_LOGS[$index]}" 2>/dev/null || \
        printf '警告：J6 日志未确认 DISABLED 终态：%s\n' "${WORKER_LOGS[$index]}" >&2
    else
      grep -q 'FINAL_BRAKE=PASS' "${WORKER_LOGS[$index]}" 2>/dev/null || \
        printf '警告：%s 日志未确认 BRAKE 终态：%s\n' "${WORKER_NAMES[$index]}" "${WORKER_LOGS[$index]}" >&2
    fi
  done
  if [[ "$forced" -ne 0 ]]; then
    printf '严重警告：有受管进程超时后被强制终止，不能声称终态已确认，请人工断电。\n' >&2
  else
    printf '受管进程已退出；终态详情见 %s\n' "$RUN_DIR"
  fi
  return "$original_status"
}

if [[ "$PREBUILD_ONLY" -eq 1 && "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 ]]; then
  fail "--prebuild-only 禁止携带任何 J2/GO-AUX launch permit"
fi
if [[ "$PREBUILD_ONLY" -eq 0 ]]; then
  [[ "$RUN_DIR" == /* ]] || fail "ARM_GUI_RUN_DIRECTORY 必须是绝对路径"
  [[ -n "$J2_SESSION_REFERENCE" && -n "$J2_SESSION_LAUNCH_PERMIT" &&
     -n "$J2_POWER_SESSION_ID" && -n "$GO_AUX_SESSION_REFERENCE" &&
     -n "$GO_AUX_J1_LAUNCH_PERMIT" && -n "$GO_AUX_J345_LAUNCH_PERMIT" &&
     -n "$GO_AUX_POWER_SESSION_ID" && -n "$J6_FEEDBACK_HANDOFF" ]] || \
    fail "正常启动必须显式提供完整 J2/GO-AUX anchor、三个一次性 permit、会话 ID 与 J6 单次反馈接管文件；仅构建请使用 --prebuild-only"
  [[ "$REUSE_RUNNING_ARM_GUI_CORE" == "true" ||
     "$REUSE_RUNNING_ARM_GUI_CORE" == "false" ]] || \
    fail "REUSE_RUNNING_ARM_GUI_CORE 必须为 true/false"
  if [[ "$REUSE_RUNNING_ARM_GUI_CORE" == "true" ]]; then
    [[ "$EXTERNAL_ARM_GUI_CORE_PID" =~ ^[1-9][0-9]*$ ]] || \
      fail "复用核心时必须提供 EXTERNAL_ARM_GUI_CORE_PID"
    managed_pid_is_live "$EXTERNAL_ARM_GUI_CORE_PID" || \
      fail "要复用的 arm GUI 核心进程当前不存活"
    [[ -d "$RUN_DIR" && ! -L "$RUN_DIR" ]] || \
      fail "复用核心时必须提供该核心正在使用的既有 ARM_GUI_RUN_DIRECTORY"
  elif [[ -n "$EXTERNAL_ARM_GUI_CORE_PID" ]]; then
    fail "未启用核心复用时不得提供 EXTERNAL_ARM_GUI_CORE_PID"
  fi
  [[ -f "$J6_FEEDBACK_HANDOFF" && ! -L "$J6_FEEDBACK_HANDOFF" ]] || \
    fail "J6 单次反馈接管文件不是常规文件"
  [[ "$(stat -c '%a' "$J6_FEEDBACK_HANDOFF")" == "600" ]] || \
    fail "J6 单次反馈接管文件不是 owner-only 0600"
  [[ ! -e "$J6_FEEDBACK_HANDOFF.claimed" &&
     ! -L "$J6_FEEDBACK_HANDOFF.claimed" ]] || \
    fail "J6 单次反馈接管文件已被使用"
fi

mkdir -p "$STATE_DIR" "$TOOLS_BUILD"
PERSISTENT_ZERO_SHA256=""
command -v flock >/dev/null || fail "缺少 flock（util-linux）"
if [[ "$PREBUILD_ONLY" -eq 1 ]]; then
  # Build publication mutates GO_BINARY/manifest and the ROS install tree. Hold
  # the supervisor, a dedicated build lock, and every primary bus lock so a
  # live GUI worker or BRAKE-only capture can never race the publication.
  exec {PREBUILD_LOCK_FD}>/tmp/v15_30a_gui_prebuild.lock
  flock -n "$PREBUILD_LOCK_FD" || fail "已有另一个 GUI 预构建正在运行"
  exec {INSTANCE_LOCK_FD}>/tmp/v15_30a_gui_supervisor.lock
  flock -n "$INSTANCE_LOCK_FD" || fail "GUI 会话运行中，禁止替换构建产物"
  exec {LEGACY_FEEDBACK_LOCK_FD}>/tmp/v15_30a_whole_arm_go_feedback.lock
  flock -n "$LEGACY_FEEDBACK_LOCK_FD" || fail "旧反馈进程运行中，禁止替换构建产物"
  exec {PREBUILD_J1_LOCK_FD}>/tmp/v15_30a_gui_j1.lock
  flock -n "$PREBUILD_J1_LOCK_FD" || fail "J1 worker/capture 运行中，禁止预构建"
  exec {PREBUILD_J2_LOCK_FD}>/tmp/v15_30a_gui_j2.lock
  flock -n "$PREBUILD_J2_LOCK_FD" || fail "J2 worker/capture 运行中，禁止预构建"
  exec {PREBUILD_J345_LOCK_FD}>/tmp/v15_30a_gui_j345.lock
  flock -n "$PREBUILD_J345_LOCK_FD" || fail "J345 worker/capture 运行中，禁止预构建"
  exec {PREBUILD_J6_LOCK_FD}>/tmp/v15_30a_gui_j6.lock
  flock -n "$PREBUILD_J6_LOCK_FD" || fail "J6 worker/diagnostic 运行中，禁止预构建"
else
  command -v fuser >/dev/null || fail "缺少 fuser（psmisc）"
  command -v ss >/dev/null || fail "缺少 ss（iproute2），无法可靠检查 UDP 端口"
  mkdir -p "$RUN_DIR"
  [[ -f "$PERSISTENT_ZERO" ]] || fail "持久软件零位文件不存在：$PERSISTENT_ZERO"
  (cd "$STATE_DIR" && sha256sum -c "$(basename "$PERSISTENT_ZERO.sha256")" --status) || \
    fail "持久软件零位文件校验失败"
  PERSISTENT_ZERO_SHA256="$(sha256sum "$PERSISTENT_ZERO" | awk '{print $1}')"
  exec {INSTANCE_LOCK_FD}>/tmp/v15_30a_gui_supervisor.lock
  flock -n "$INSTANCE_LOCK_FD" || fail "已有一个机械臂 GUI 会话在运行"
  exec {LEGACY_FEEDBACK_LOCK_FD}>/tmp/v15_30a_whole_arm_go_feedback.lock
  flock -n "$LEGACY_FEEDBACK_LOCK_FD" || fail "旧的全臂反馈进程仍在占用 GO 物理总线"
  # Only install cleanup after ownership locks succeed. A competing invocation
  # must never send stop/BRAKE traffic into the already-owned live session.
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM HUP

  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  if [[ -z "${DISPLAY:-}" ]] && [[ -S /tmp/.X11-unix/X0 ]]; then export DISPLAY=:0; fi
  if [[ -z "${XAUTHORITY:-}" ]]; then
    for candidate in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
      if [[ -f "$candidate" ]]; then export XAUTHORITY="$candidate"; break; fi
    done
  fi
  if [[ -S "$XDG_RUNTIME_DIR/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"
  fi
  [[ -n "${DISPLAY:-}" ]] || fail "未找到 Ubuntu 图形会话 DISPLAY"
  command -v xdpyinfo >/dev/null || fail "缺少 xdpyinfo"
  xdpyinfo >/dev/null 2>&1 || fail "无法访问当前 Ubuntu 图形会话"
  if [[ -n "${QT_QPA_PLATFORM:-}" ]]; then
    : # Respect an explicitly selected, already validated Qt display backend.
  elif [[ -S "$XDG_RUNTIME_DIR/wayland-0" ]]; then
    export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
    export QT_QPA_PLATFORM=wayland
  elif dpkg-query -W -f='${Status}' libxcb-cursor0 2>/dev/null | grep -q 'install ok installed'; then
    export QT_QPA_PLATFORM=xcb
  else
    fail "既无可用 Wayland 会话，也缺少 libxcb-cursor0"
  fi
fi

[[ -f "$MODEL" ]] || fail "冻结 MuJoCo 生产模型不存在"
actual_model_sha="$(sha256sum "$MODEL" | awk '{print $1}')"
[[ "$actual_model_sha" == "$MODEL_SHA256" ]] || fail "冻结 MuJoCo 生产模型哈希不匹配"
[[ -f "$GRAVITY_CONFIG" ]] || fail "冻结重力配置不存在"
GRAVITY_CONTINUOUS_AUTHORITY="$({
  awk -F: '
    /^[[:space:]]*continuous_rotor_limits_authoritative[[:space:]]*:/ {
      value=$2
      gsub(/[[:space:]]/, "", value)
      print value
    }
  ' "$GRAVITY_CONFIG"
} | sed -n '1p')"
[[ "$GRAVITY_CONTINUOUS_AUTHORITY" == "true" ||
   "$GRAVITY_CONTINUOUS_AUTHORITY" == "false" ]] || \
  fail "冻结重力配置的 continuous authority 布尔值无效"
[[ "$GRAVITY_ENABLED_FOR_HARDWARE" == "true" ||
   "$GRAVITY_ENABLED_FOR_HARDWARE" == "false" ]] || \
  fail "GRAVITY_ENABLED_FOR_HARDWARE 必须为 true/false"
case "$GRAVITY_SCALE_TARGET" in
  0|0.0|0.00|0.25|0.5|0.50|0.75|1|1.0|1.00) ;;
  *) fail "GRAVITY_SCALE_TARGET 必须为 0/0.25/0.50/0.75/1.00" ;;
esac
if [[ "$GRAVITY_ENABLED_FOR_HARDWARE" == "true" ||
      "$GRAVITY_SCALE_TARGET" != "0" &&
      "$GRAVITY_SCALE_TARGET" != "0.0" &&
      "$GRAVITY_SCALE_TARGET" != "0.00" ]]; then
  [[ -n "$GRAVITY_ANCHOR" ]] || \
    fail "主动重力/HOLD 必须提供 anchor"
  if [[ "$GRAVITY_CONTINUOUS_AUTHORITY" != "true" ]]; then
    [[ -n "$EMPIRICAL_ENVELOPE" &&
       -n "$EMPIRICAL_ENVELOPE_SHA256" ]] || \
      fail "无官方连续额定authority时，主动重力/HOLD必须提供 empirical envelope 及其 SHA256"
  fi
fi
if [[ -n "$GRAVITY_ANCHOR" ]]; then
  [[ -f "$GRAVITY_ANCHOR" && ! -L "$GRAVITY_ANCHOR" ]] || \
    fail "重力运行的 anchor 不是常规文件"
fi
if [[ -n "$EMPIRICAL_ENVELOPE" ||
      -n "$EMPIRICAL_ENVELOPE_SHA256" ]]; then
  [[ -f "$EMPIRICAL_ENVELOPE" && ! -L "$EMPIRICAL_ENVELOPE" ]] || \
    fail "empirical envelope 不是常规文件"
  [[ "$EMPIRICAL_ENVELOPE_SHA256" =~ ^[0-9a-f]{64}$ ]] || \
    fail "empirical envelope SHA256 格式无效"
  actual_empirical_envelope_sha256="$(
    sha256sum "$EMPIRICAL_ENVELOPE" | awk '{print $1}'
  )"
  [[ "$actual_empirical_envelope_sha256" == "$EMPIRICAL_ENVELOPE_SHA256" ]] || \
    fail "empirical envelope SHA256 不匹配"
fi

# Freeze the exact authority identity once, before active workers are started.
# Workers stay alive/disabled through the whole acceptance session and reject
# every UDP authority that does not match these CLI pins.
if [[ -z "$GRAVITY_ANCHOR" ]]; then
  EXPECTED_GRAVITY_AUTHORITY_CLASS="NONE"
else
  GRAVITY_BINDING_OUTPUT="$(
  python3 - \
    "$GRAVITY_CONTINUOUS_AUTHORITY" \
    "$GRAVITY_ANCHOR" \
    "$EMPIRICAL_ENVELOPE" \
    "$EMPIRICAL_ENVELOPE_SHA256" <<'PY'
import hashlib
import json
import re
import sys
from pathlib import Path

continuous, anchor_text, envelope_text, expected_envelope_sha = sys.argv[1:]
if not anchor_text:
    # No active gravity authority is configured for this launch.
    print("NONE\n\n\n\n\n")
    raise SystemExit(0)

def load(path_text):
    data = Path(path_text).read_bytes()
    value = json.loads(data.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("document root is not an object")
    return value, hashlib.sha256(data).hexdigest()

anchor, anchor_sha = load(anchor_text)
session = anchor.get("session_id")
state = anchor.get("state_instance_id")
if not isinstance(session, str) or re.fullmatch(
    r"persistent:[0-9a-f]{16}:j2session:[0-9a-f]{16}:"
    r"goauxsession:[0-9a-f]{16}", session
) is None:
    raise ValueError("anchor session_id is invalid")
if not isinstance(state, str) or re.fullmatch(r"[0-9a-f]{32}", state) is None:
    raise ValueError("anchor state_instance_id is invalid")

if continuous == "true":
    authority_class = "OFFICIAL_CONTINUOUS_RATING"
    envelope_id = ""
    envelope_sha = ""
else:
    authority_class = "EMPIRICAL_VALIDATION_ENVELOPE"
    envelope, envelope_sha = load(envelope_text)
    if envelope_sha != expected_envelope_sha:
        raise ValueError("envelope SHA-256 changed during binding")
    binding = envelope.get("binding")
    if not isinstance(binding, dict):
        raise ValueError("envelope binding is missing")
    if (
        binding.get("session_id") != session
        or binding.get("state_instance_id") != state
        or binding.get("anchor_sha256") != anchor_sha
    ):
        raise ValueError("envelope/anchor/session binding mismatch")
    envelope_id = envelope.get("envelope_id")
    if not isinstance(envelope_id, str) or re.fullmatch(
        r"v15-31b-empirical-[0-9a-f]{20}", envelope_id
    ) is None:
        raise ValueError("envelope_id is invalid")

print(authority_class)
print(envelope_id)
print(envelope_sha)
print(anchor_sha)
print(session)
print(state)
PY
)" || fail "无法冻结重力authority worker启动绑定"
  mapfile -t GRAVITY_BINDING_FIELDS <<<"$GRAVITY_BINDING_OUTPUT"
  [[ "${#GRAVITY_BINDING_FIELDS[@]}" -eq 6 ]] || \
    fail "重力authority worker启动绑定字段数无效"
  EXPECTED_GRAVITY_AUTHORITY_CLASS="${GRAVITY_BINDING_FIELDS[0]}"
  EXPECTED_EMPIRICAL_ENVELOPE_ID="${GRAVITY_BINDING_FIELDS[1]}"
  EXPECTED_EMPIRICAL_ENVELOPE_SHA256="${GRAVITY_BINDING_FIELDS[2]}"
  EXPECTED_GRAVITY_ANCHOR_SHA256="${GRAVITY_BINDING_FIELDS[3]}"
  EXPECTED_GRAVITY_SESSION_ID="${GRAVITY_BINDING_FIELDS[4]}"
  EXPECTED_GRAVITY_STATE_INSTANCE_ID="${GRAVITY_BINDING_FIELDS[5]}"
fi

declare -A DOMAIN_READY=([J1]=1 [J2]=1 [J345]=1 [J6]=1)
declare -A DOMAIN_REASON=([J1]="" [J2]="" [J345]="" [J6]="")

mark_domain_unavailable() {
  local domain=$1
  shift
  DOMAIN_READY[$domain]=0
  DOMAIN_REASON[$domain]="$*"
  printf '警告：%s 故障域不可用：%s；GUI、MuJoCo 与其它故障域将继续。\n' \
    "$domain" "$*" >&2
}

clear_j2_session_inputs() {
  J2_SESSION_REFERENCE=""
  J2_SESSION_LAUNCH_PERMIT=""
  J2_POWER_SESSION_ID=""
  J2_SESSION_REFERENCE_SHA256=""
  J2_EXPECTED_WORKER_SHA256=""
}

clear_go_aux_session_inputs() {
  GO_AUX_SESSION_REFERENCE=""
  GO_AUX_J1_LAUNCH_PERMIT=""
  GO_AUX_J345_LAUNCH_PERMIT=""
  GO_AUX_POWER_SESSION_ID=""
  GO_AUX_SESSION_REFERENCE_SHA256=""
  GO_AUX_EXPECTED_WORKER_SHA256=""
}

capture_go_build_inputs() {
  local -a capture_args
  if [[ "$#" -eq 0 ]]; then
    capture_args=(
      "$GO_CONTROLLER_SOURCE" "$GO_SDK_INCLUDE" "$GO_SDK_LIBRARY"
      "${GO_SDK_HEADERS[@]}"
    )
  else
    capture_args=("$@")
  fi
  [[ "${#capture_args[@]}" -eq 9 ]] || return 1
  python3 - "${capture_args[@]}" <<'PY'
import hashlib
import json
import os
import pathlib
import stat
import sys

HEADER_RELATIVE_PATHS = (
    "unitreeMotor/unitreeMotor.h",
    "unitreeMotor/include/motor_msg_GO-M8010-6.h",
    "unitreeMotor/include/motor_msg_A1B1.h",
    "serialPort/SerialPort.h",
    "serialPort/include/errorClass.h",
    "IOPort/IOPort.h",
)

source_path = pathlib.Path(sys.argv[1])
sdk_include_path = pathlib.Path(sys.argv[2])
sdk_library_path = pathlib.Path(sys.argv[3])
header_paths = [pathlib.Path(value) for value in sys.argv[4:]]


def stable_binding(path, label):
    canonical = path.resolve(strict=True)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(canonical, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ValueError(f"{label} is not a non-empty regular file")
        digest = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        after = os.fstat(descriptor)
        identity_before = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            before.st_ctime_ns,
        )
        identity_after = (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        )
        if identity_before != identity_after:
            raise ValueError(f"{label} changed while its fingerprint was read")
    finally:
        os.close(descriptor)
    return {"path": str(canonical), "sha256": digest.hexdigest()}


try:
    if len(header_paths) != len(HEADER_RELATIVE_PATHS):
        raise ValueError("fixed SDK header list length mismatch")
    canonical_include = sdk_include_path.resolve(strict=True)
    if not canonical_include.is_dir():
        raise ValueError("SDK include path is not a directory")
    sdk_headers = []
    for relative_path, supplied_path in zip(HEADER_RELATIVE_PATHS, header_paths):
        canonical_header = supplied_path.resolve(strict=True)
        expected_header = (canonical_include / relative_path).resolve(strict=True)
        if canonical_header != expected_header:
            raise ValueError(f"SDK header path mismatch: {relative_path}")
        binding = stable_binding(canonical_header, f"SDK header {relative_path}")
        binding["include"] = relative_path
        sdk_headers.append(binding)
    fingerprint = {
        "schema": "go-m8010-go-build-input-fingerprint/1.0",
        "controller_source": stable_binding(source_path, "controller source"),
        "sdk_include_path": str(canonical_include),
        "sdk_shared_library": stable_binding(sdk_library_path, "SDK shared library"),
        "sdk_headers": sdk_headers,
    }
    print(json.dumps(fingerprint, sort_keys=True, separators=(",", ":")))
except Exception as exc:
    print(f"GO build input fingerprint failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

verify_go_build_inputs_unchanged() {
  local expected_fingerprint=$1
  local current_fingerprint
  if ! current_fingerprint="$(capture_go_build_inputs)"; then
    return 1
  fi
  [[ "$current_fingerprint" == "$expected_fingerprint" ]]
}

go_sdk_build_inputs_present() {
  local sdk_header
  [[ -f "$GO_SDK_LIBRARY" ]] || return 1
  for sdk_header in "${GO_SDK_HEADERS[@]}"; do
    [[ -f "$sdk_header" ]] || return 1
  done
}

basic_j2_permit_worker_matches() {
  python3 - "$1" "$GO_BINARY" "$GO_BUILD_MANIFEST" \
    "$GO_CONTROLLER_SOURCE" "$GO_SDK_INCLUDE" "$GO_SDK_LIBRARY" \
    "${GO_SDK_HEADERS[@]}" <<'PY'
import hashlib
import json
import os
import pathlib
import stat
import sys

HEADER_RELATIVE_PATHS = (
    "unitreeMotor/unitreeMotor.h",
    "unitreeMotor/include/motor_msg_GO-M8010-6.h",
    "unitreeMotor/include/motor_msg_A1B1.h",
    "serialPort/SerialPort.h",
    "serialPort/include/errorClass.h",
    "IOPort/IOPort.h",
)

permit_path_text = sys.argv[1]
go_binary = pathlib.Path(sys.argv[2])
manifest_path = pathlib.Path(sys.argv[3])
source_path = pathlib.Path(sys.argv[4])
sdk_include_path = pathlib.Path(sys.argv[5])
sdk_library_path = pathlib.Path(sys.argv[6])
header_paths = [pathlib.Path(value) for value in sys.argv[7:]]


def digest(path):
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ValueError(f"build input is not a non-empty regular file: {path}")
        result = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            result.update(chunk)
        after = os.fstat(descriptor)
        if (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            before.st_ctime_ns,
        ) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        ):
            raise ValueError(f"build input changed while hashing: {path}")
        return result.hexdigest()
    finally:
        os.close(descriptor)


def read_bounded_json(path, maximum_bytes, *, secure_manifest=False):
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        status = os.fstat(descriptor)
        if (
            not stat.S_ISREG(status.st_mode)
            or status.st_size <= 0
            or status.st_size > maximum_bytes
        ):
            raise ValueError(f"unsafe or unbounded JSON file: {path}")
        if secure_manifest and (
            status.st_uid != os.geteuid()
            or stat.S_IMODE(status.st_mode) != 0o600
            or status.st_nlink != 1
        ):
            raise ValueError("build manifest is not owner-only 0600/link-count-one")
        chunks = []
        remaining = status.st_size
        while remaining:
            chunk = os.read(descriptor, remaining)
            if not chunk:
                raise ValueError(f"short JSON read: {path}")
            chunks.append(chunk)
            remaining -= len(chunk)
    finally:
        os.close(descriptor)
    document = json.loads(b"".join(chunks).decode("utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return document


def compile_profile(source, binary, sdk_include, sdk_library):
    return {
        "profile_id": "go-m8010-production-g++-c++17-o2-werror-pthread/1.0",
        "argv": [
            "g++",
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pthread",
            f"-I{sdk_include}",
            str(source),
            str(sdk_library),
            f"-Wl,-rpath,{sdk_library.parent}",
            "-o",
            str(binary),
        ],
    }

try:
    canonical_go = go_binary.resolve(strict=True)
    canonical_source = source_path.resolve(strict=True)
    canonical_sdk_include = sdk_include_path.resolve(strict=True)
    canonical_sdk_library = sdk_library_path.resolve(strict=True)
    if len(header_paths) != len(HEADER_RELATIVE_PATHS):
        raise ValueError("fixed SDK header list length mismatch")
    canonical_headers = []
    for relative_path, supplied_path in zip(HEADER_RELATIVE_PATHS, header_paths):
        canonical_header = supplied_path.resolve(strict=True)
        expected_header = (canonical_sdk_include / relative_path).resolve(strict=True)
        if canonical_header != expected_header:
            raise ValueError(f"SDK header path mismatch: {relative_path}")
        canonical_headers.append((relative_path, canonical_header))
    canonical_manifest = manifest_path.resolve(strict=True)
    if canonical_manifest != pathlib.Path(f"{canonical_go}.build.json"):
        raise ValueError("build manifest path does not belong to GO_BINARY")

    manifest = read_bounded_json(
        manifest_path, 131072, secure_manifest=True
    )
    binary_sha256 = digest(canonical_go)
    source_sha256 = digest(canonical_source)
    sdk_sha256 = digest(canonical_sdk_library)
    sdk_header_bindings = [
        {
            "path": str(header_path),
            "sha256": digest(header_path),
            "include": relative_path,
        }
        for relative_path, header_path in canonical_headers
    ]
    if permit_path_text:
        permit = read_bounded_json(pathlib.Path(permit_path_text), 131072)
        worker = permit.get("worker")
        if not isinstance(worker, dict):
            raise ValueError("basic permit worker is missing")
        worker_path_text = worker.get("path")
        worker_sha256 = worker.get("sha256")
        if not isinstance(worker_path_text, str) or not isinstance(worker_sha256, str):
            raise ValueError("basic permit worker binding is malformed")
        if len(worker_sha256) != 64 or any(
            c not in "0123456789abcdef" for c in worker_sha256
        ):
            raise ValueError("basic permit worker SHA-256 is malformed")
        canonical_worker = pathlib.Path(worker_path_text).resolve(strict=True)
        if canonical_worker != canonical_go or worker_path_text != str(canonical_worker):
            raise ValueError("basic permit worker path does not name GO_BINARY canonically")
        if binary_sha256 != worker_sha256:
            raise ValueError("basic permit worker SHA-256 differs from GO_BINARY")

    if manifest.get("schema") != "go-m8010-go-controller-build/1.0":
        raise ValueError("build manifest schema mismatch")
    manifest_source = manifest.get("controller_source")
    manifest_binary = manifest.get("binary")
    manifest_sdk = manifest.get("sdk_shared_library")
    if not all(
        isinstance(value, dict)
        for value in (manifest_source, manifest_binary, manifest_sdk)
    ):
        raise ValueError("build manifest path/SHA bindings are missing")
    if (
        manifest_source.get("path") != str(canonical_source)
        or manifest_source.get("sha256") != source_sha256
    ):
        raise ValueError("build manifest controller source mismatch")
    if (
        manifest_binary.get("path") != str(canonical_go)
        or manifest_binary.get("sha256") != binary_sha256
    ):
        raise ValueError("build manifest binary mismatch")
    if (
        manifest_sdk.get("path") != str(canonical_sdk_library)
        or manifest_sdk.get("sha256") != sdk_sha256
    ):
        raise ValueError("build manifest SDK shared library mismatch")
    if manifest.get("sdk_headers") != sdk_header_bindings:
        raise ValueError("build manifest SDK header path/SHA mismatch")
    expected_profile = compile_profile(
        canonical_source,
        canonical_go,
        canonical_sdk_include,
        canonical_sdk_library,
    )
    if manifest.get("compile_profile") != expected_profile:
        raise ValueError("build manifest compile profile mismatch")
    if manifest.get("offline_self_test") != {
        "passed": True,
        "dry_run": True,
        "serial_opened": False,
        "required_output_lines": ["DRY_RUN=YES", "SERIAL_OPENED=NO"],
    }:
        raise ValueError("build manifest offline self-test declaration mismatch")
except Exception as exc:
    print(f"J2 basic permit worker cache miss: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

run_go_offline_self_test() {
  local output
  if ! output="$("$GO_BINARY")"; then
    return 1
  fi
  grep -Fxq 'DRY_RUN=YES' <<<"$output" && \
    grep -Fxq 'SERIAL_OPENED=NO' <<<"$output"
}

publish_go_build_manifest() {
  local expected_fingerprint=$1
  python3 - "$GO_BUILD_MANIFEST" "$GO_BINARY" "$GO_CONTROLLER_SOURCE" \
    "$GO_SDK_INCLUDE" "$GO_SDK_LIBRARY" "$expected_fingerprint" \
    "${GO_SDK_HEADERS[@]}" <<'PY'
import hashlib
import json
import os
import pathlib
import stat
import subprocess
import sys
import tempfile

HEADER_RELATIVE_PATHS = (
    "unitreeMotor/unitreeMotor.h",
    "unitreeMotor/include/motor_msg_GO-M8010-6.h",
    "unitreeMotor/include/motor_msg_A1B1.h",
    "serialPort/SerialPort.h",
    "serialPort/include/errorClass.h",
    "IOPort/IOPort.h",
)

manifest_path = pathlib.Path(sys.argv[1])
binary_path = pathlib.Path(sys.argv[2]).resolve(strict=True)
source_path = pathlib.Path(sys.argv[3]).resolve(strict=True)
sdk_include_path = pathlib.Path(sys.argv[4]).resolve(strict=True)
sdk_library_path = pathlib.Path(sys.argv[5]).resolve(strict=True)
expected_fingerprint_text = sys.argv[6]
header_paths = [pathlib.Path(value) for value in sys.argv[7:]]


def digest(path):
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ValueError(f"build input is not a non-empty regular file: {path}")
        result = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            result.update(chunk)
        after = os.fstat(descriptor)
        if (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            before.st_ctime_ns,
        ) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        ):
            raise ValueError(f"build input changed while hashing: {path}")
        return result.hexdigest()
    finally:
        os.close(descriptor)


def build_input_fingerprint(canonical_headers):
    return {
        "schema": "go-m8010-go-build-input-fingerprint/1.0",
        "controller_source": {
            "path": str(source_path),
            "sha256": digest(source_path),
        },
        "sdk_include_path": str(sdk_include_path),
        "sdk_shared_library": {
            "path": str(sdk_library_path),
            "sha256": digest(sdk_library_path),
        },
        "sdk_headers": [
            {
                "path": str(header_path),
                "sha256": digest(header_path),
                "include": relative_path,
            }
            for relative_path, header_path in canonical_headers
        ],
    }


def compile_profile():
    return {
        "profile_id": "go-m8010-production-g++-c++17-o2-werror-pthread/1.0",
        "argv": [
            "g++",
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pthread",
            f"-I{sdk_include_path}",
            str(source_path),
            str(sdk_library_path),
            f"-Wl,-rpath,{sdk_library_path.parent}",
            "-o",
            str(binary_path),
        ],
    }


temporary_path = None
try:
    canonical_manifest = manifest_path.parent.resolve(strict=True) / manifest_path.name
    if canonical_manifest != pathlib.Path(f"{binary_path}.build.json"):
        raise ValueError("build manifest path does not belong to GO_BINARY")
    if len(header_paths) != len(HEADER_RELATIVE_PATHS):
        raise ValueError("fixed SDK header list length mismatch")
    canonical_headers = []
    for relative_path, supplied_path in zip(HEADER_RELATIVE_PATHS, header_paths):
        canonical_header = supplied_path.resolve(strict=True)
        expected_header = (sdk_include_path / relative_path).resolve(strict=True)
        if canonical_header != expected_header:
            raise ValueError(f"SDK header path mismatch: {relative_path}")
        canonical_headers.append((relative_path, canonical_header))
    expected_fingerprint = json.loads(expected_fingerprint_text)
    if not isinstance(expected_fingerprint, dict):
        raise ValueError("pre-compile build input fingerprint is not an object")
    if build_input_fingerprint(canonical_headers) != expected_fingerprint:
        raise ValueError("build inputs changed between pre-compile capture and publication")
    binary_sha256 = digest(binary_path)
    self_test = subprocess.run(
        [str(binary_path)],
        check=False,
        capture_output=True,
        text=True,
        timeout=30.0,
    )
    self_test_lines = set(self_test.stdout.splitlines())
    if (
        self_test.returncode != 0
        or "DRY_RUN=YES" not in self_test_lines
        or "SERIAL_OPENED=NO" not in self_test_lines
        or digest(binary_path) != binary_sha256
    ):
        raise ValueError("exact manifest binary failed its no-serial offline self-test")
    if build_input_fingerprint(canonical_headers) != expected_fingerprint:
        raise ValueError("build inputs changed during the manifest offline self-test")
    document = {
        "schema": "go-m8010-go-controller-build/1.0",
        "controller_source": expected_fingerprint["controller_source"],
        "binary": {
            "path": str(binary_path),
            "sha256": binary_sha256,
        },
        "sdk_shared_library": expected_fingerprint["sdk_shared_library"],
        "sdk_headers": expected_fingerprint["sdk_headers"],
        "compile_profile": compile_profile(),
        "offline_self_test": {
            "passed": True,
            "dry_run": True,
            "serial_opened": False,
            "required_output_lines": ["DRY_RUN=YES", "SERIAL_OPENED=NO"],
        },
    }
    data = (
        json.dumps(document, ensure_ascii=False, indent=2) + "\n"
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{canonical_manifest.name}.",
        suffix=".tmp",
        dir=canonical_manifest.parent,
    )
    temporary_path = pathlib.Path(temporary_name)
    with os.fdopen(descriptor, "wb") as stream:
        os.fchmod(stream.fileno(), 0o600)
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    if (
        build_input_fingerprint(canonical_headers) != expected_fingerprint
        or digest(binary_path) != binary_sha256
    ):
        raise ValueError("build inputs or binary changed before manifest publication")
    os.replace(temporary_path, canonical_manifest)
    temporary_path = None
    directory_descriptor = os.open(
        canonical_manifest.parent,
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
    )
    try:
        os.fsync(directory_descriptor)
    finally:
        os.close(directory_descriptor)
    status = canonical_manifest.lstat()
    if (
        not stat.S_ISREG(status.st_mode)
        or stat.S_ISLNK(status.st_mode)
        or status.st_uid != os.geteuid()
        or stat.S_IMODE(status.st_mode) != 0o600
        or status.st_nlink != 1
        or canonical_manifest.read_bytes() != data
    ):
        raise ValueError("published build manifest failed post-write verification")
except Exception as exc:
    print(f"GO build manifest publication failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
finally:
    if temporary_path is not None:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
PY
}

validate_j2_session_launch_gate() {
  PYTHONPATH="$ROS_WS/src/go_m8010_arm_hardware${PYTHONPATH:+:$PYTHONPATH}" \
  python3 - "$1" "$2" "$3" "$PERSISTENT_ZERO" \
    "$PERSISTENT_ZERO.sha256" "$RECOVERY_BRANCH_HINTS" "$INITIAL_POSE" \
    "$GO_BINARY" <<'PY'
import hashlib
import json
import math
import os
import pathlib
import re
import stat
import sys
import time
import uuid
from datetime import datetime, timezone
from go_m8010_arm_hardware.state_model import validate_preserved_session_reference

(
    anchor_text,
    permit_text,
    expected_power_session_id,
    zero_text,
    sidecar_text,
    hints_text,
    initial_pose_text,
    worker_text,
) = sys.argv[1:]

GEAR_RATIO = 6.329999923706055
# GO-M8010-6 position is a signed int32 Q15 value, not the DM-G6220
# +/-12.5-rad PMAX protocol.  Preserve the full GO multi-turn envelope.
MAX_RAW_POSITION_RAD = float(1 << 16)
MAX_RAW_SPAN_RAD = GEAR_RATIO * math.radians(0.20)
MAX_STARTUP_DELTA_RAD = GEAR_RATIO * math.radians(0.25)
MAX_TTL_NS = 30_000_000_000
MAX_CAPTURE_TO_PERMIT_NS = 300_000_000_000
EXPECTED_SERIAL = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0"
EXPECTED_SIGNS = {"J2A": -1, "J2B": 1}
EXPECTED_OPERATOR_GATE = (
    "J2_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;"
    "NOT_AT_MECHANICAL_LIMIT=YES"
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def normalized_sha256(value, label):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{label} SHA-256 is malformed")
    return value


def positive_integer(value, label):
    if type(value) is not int or value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value


def exact_integer(value, label):
    if type(value) is not int:
        raise ValueError(f"{label} must be an integer")
    return value


def canonical_count(mapping, field, *, positive=True):
    value = mapping.get(field)
    if not isinstance(value, str) or re.fullmatch(r"0|[1-9][0-9]*", value) is None:
        raise ValueError(f"worker terminal {field} must be canonical decimal")
    parsed = int(value, 10)
    if (positive and parsed <= 0) or (not positive and parsed < 0):
        raise ValueError(f"worker terminal {field} is outside its count domain")
    return parsed


def finite_number(value, label):
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be finite")
    return float(value)


def nonempty_text(value, label):
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 256
        or any(ord(character) < 0x20 or ord(character) == 0x7F for character in value)
    ):
        raise ValueError(f"{label} is invalid")
    return value


def normalized_boot_id(value, label):
    if not isinstance(value, str):
        raise ValueError(f"{label} is not a lowercase canonical UUID")
    try:
        parsed = uuid.UUID(value)
    except (AttributeError, ValueError) as exc:
        raise ValueError(f"{label} is not a lowercase canonical UUID") from exc
    if str(parsed) != value or parsed.int == 0:
        raise ValueError(f"{label} is not a lowercase canonical UUID")
    return value


def parse_utc(value, label):
    text = nonempty_text(value, label)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError(f"{label} is not ISO-8601") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError(f"{label} lacks a UTC offset")
    if parsed.utcoffset().total_seconds() != 0:
        raise ValueError(f"{label} is not UTC")
    return parsed.astimezone(timezone.utc)


def path_lexists(path):
    return os.path.lexists(os.fspath(path))


def require_secure_regular(path, label):
    status = path.lstat()
    if stat.S_ISLNK(status.st_mode) or not stat.S_ISREG(status.st_mode):
        raise ValueError(f"{label} is not a non-symlink regular file")
    if status.st_uid != os.geteuid():
        raise ValueError(f"{label} owner mismatch")
    if stat.S_IMODE(status.st_mode) != 0o600:
        raise ValueError(f"{label} mode is not 0600")
    if status.st_nlink != 1:
        raise ValueError(f"{label} link count is not one")


def require_secure_directory(path, label):
    status = path.lstat()
    if stat.S_ISLNK(status.st_mode) or not stat.S_ISDIR(status.st_mode):
        raise ValueError(f"{label} is not a non-symlink directory")
    if status.st_uid != os.geteuid():
        raise ValueError(f"{label} owner mismatch")
    if stat.S_IMODE(status.st_mode) != 0o700:
        raise ValueError(f"{label} mode is not 0700")


def load_json(path, label, maximum_bytes=4 * 1024 * 1024):
    data = path.read_bytes()
    if not data or len(data) > maximum_bytes:
        raise ValueError(f"{label} size is invalid")
    document = json.loads(data.decode("utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{label} is not a JSON object")
    return document


try:
    expected_power_session_id = nonempty_text(
        expected_power_session_id, "expected power-session id"
    )
    anchor_arg = pathlib.Path(anchor_text)
    permit_arg = pathlib.Path(permit_text)
    require_secure_regular(anchor_arg, "J2 session reference")
    require_secure_regular(permit_arg, "J2 launch permit")
    anchor_path = anchor_arg.resolve(strict=True)
    permit_path = permit_arg.resolve(strict=True)
    zero_path = pathlib.Path(zero_text).resolve(strict=True)
    sidecar_path = pathlib.Path(sidecar_text).resolve(strict=True)
    hints_path = pathlib.Path(hints_text).resolve(strict=True)
    initial_pose_path = pathlib.Path(initial_pose_text).resolve(strict=True)
    worker_path = pathlib.Path(worker_text).resolve(strict=True)
    nonempty_text(str(anchor_path), "canonical session-reference path")
    nonempty_text(str(permit_path), "canonical launch-permit path")
    nonempty_text(str(worker_path), "canonical worker path")

    lifecycle_root = pathlib.Path(
        f"/run/user/{os.geteuid()}/go-m8010/anchors"
    )
    pending_directory = lifecycle_root / "pending"
    inflight_directory = lifecycle_root / "inflight"
    spent_directory = lifecycle_root / "spent"
    for state_name, directory in (
        ("pending", pending_directory),
        ("inflight", inflight_directory),
        ("spent", spent_directory),
    ):
        require_secure_directory(directory, f"J2 permit {state_name} directory")
    if permit_path.parent != pending_directory:
        raise ValueError("J2 launch permit is outside the canonical pending directory")

    anchor = load_json(anchor_path, "J2 session reference")
    preserved_startup = validate_preserved_session_reference(anchor)
    permit = load_json(permit_path, "J2 launch permit", 131072)
    anchor_sha256 = digest(anchor_path)
    worker_sha256 = digest(worker_path)
    zero_sha256 = digest(zero_path)
    sidecar_sha256 = digest(sidecar_path)
    hints_sha256 = digest(hints_path)
    initial_pose_sha256 = digest(initial_pose_path)

    current_boot_id = normalized_boot_id(
        pathlib.Path("/proc/sys/kernel/random/boot_id").read_text(
            encoding="ascii"
        ).strip(),
        "current host boot id",
    )

    if anchor.get("schema") != "go-m8010-j2-power-session-reference/1.0":
        raise ValueError("session-reference schema mismatch")
    if type(anchor.get("anchor_version")) is not int or anchor.get("anchor_version") != 1:
        raise ValueError("session-reference version mismatch")
    if anchor.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1":
        raise ValueError("session-reference name mismatch")
    anchor_id = nonempty_text(anchor.get("anchor_id"), "session-reference id")
    if anchor.get("parent_persistent_zero_sha256") != zero_sha256:
        raise ValueError("session-reference persistent parent mismatch")
    parent = anchor.get("parent_persistent_software_zero")
    if not isinstance(parent, dict) or (
        parent.get("schema") != "go-m8010-persistent-software-zero/1.0"
        or parent.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1"
        or parent.get("json_sha256") != zero_sha256
        or parent.get("checksum_sidecar_sha256") != sidecar_sha256
    ):
        raise ValueError("session-reference parent evidence mismatch")
    preserved = anchor.get("preserved_inputs")
    if not isinstance(preserved, dict) or (
        preserved.get("recovery_branch_hints_sha256") != hints_sha256
        or preserved.get("initial_pose_sha256") != initial_pose_sha256
        or preserved.get("overwritten_or_deleted") is not False
    ):
        raise ValueError("session-reference preserved-input binding mismatch")
    confirmation = anchor.get("operator_confirmation")
    if not isinstance(confirmation, dict):
        raise ValueError("operator confirmation is missing")
    for field in (
        "vertical_initialization_pose",
        "support_reliable",
        "arm_not_moved",
        "not_at_mechanical_limit",
    ):
        if confirmation.get(field) is not True:
            raise ValueError(f"operator confirmation missing: {field}")
    if (
        confirmation.get("confirmation_gate") != EXPECTED_OPERATOR_GATE
        or confirmation.get("power_session_id") != expected_power_session_id
    ):
        raise ValueError("operator confirmation is bound to another physical session")
    writes = anchor.get("writes")
    if not isinstance(writes, dict) or any(
        writes.get(field) is not False
        for field in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise ValueError("session-reference records a forbidden write")
    authority = anchor.get("control_authority")
    if not isinstance(authority, dict) or any(
        authority.get(field) is not False
        for field in (
            "is_software_zero",
            "authorizes_active_control",
            "authorizes_motor_internal_write",
        )
    ):
        raise ValueError("session-reference authority is invalid")

    evidence = anchor.get("source_evidence")
    if not isinstance(evidence, dict):
        raise ValueError("raw source evidence binding is missing")
    evidence_path_text = nonempty_text(
        evidence.get("path"), "raw source evidence path"
    )
    evidence_sha256 = normalized_sha256(
        evidence.get("sha256"), "raw source evidence"
    )
    evidence_arg = pathlib.Path(evidence_path_text)
    require_secure_regular(evidence_arg, "raw source evidence")
    evidence_path = evidence_arg.resolve(strict=True)
    if evidence_path_text != str(evidence_path) or digest(evidence_path) != evidence_sha256:
        raise ValueError("raw source evidence path or digest mismatch")
    capture_document = load_json(evidence_path, "raw source evidence")
    raw_capture = anchor.get("raw_capture")
    if not isinstance(raw_capture, dict):
        raise ValueError("normalized raw capture is missing")
    if (
        raw_capture.get("source_schema")
        != "go-m8010-j2-brake-raw-capture-statistics/1.0"
        or raw_capture.get("source_file_sha256") != evidence_sha256
        or capture_document.get("schema")
        != "go-m8010-j2-brake-raw-capture-statistics/1.0"
        or capture_document.get("status") != "PASS"
        or capture_document.get("hardware_accessed") is not True
        or capture_document.get("physical_power_state_during_capture") != "24V_ON"
    ):
        raise ValueError("raw capture status or schema mismatch")

    capture_power_session_id = nonempty_text(
        capture_document.get("power_session_id"), "raw power-session id"
    )
    raw_power_session_id = nonempty_text(
        raw_capture.get("power_session_id"), "normalized raw power-session id"
    )
    capture_boot_id = normalized_boot_id(
        capture_document.get("host_boot_id"), "raw capture host boot id"
    )
    normalized_capture_boot_id = normalized_boot_id(
        raw_capture.get("host_boot_id"), "normalized raw capture host boot id"
    )
    capture_boottime_ns = positive_integer(
        capture_document.get("recorded_boottime_ns"),
        "raw capture CLOCK_BOOTTIME",
    )
    normalized_capture_boottime_ns = positive_integer(
        raw_capture.get("recorded_boottime_ns"),
        "normalized raw capture CLOCK_BOOTTIME",
    )
    if (
        capture_power_session_id != expected_power_session_id
        or raw_power_session_id != expected_power_session_id
        or capture_boot_id != current_boot_id
        or normalized_capture_boot_id != current_boot_id
        or capture_boottime_ns != normalized_capture_boottime_ns
    ):
        raise ValueError("raw capture power-session or boot binding mismatch")
    packet_count = positive_integer(capture_document.get("packet_count"), "packet count")
    normalized_packet_count = positive_integer(
        raw_capture.get("packet_count"), "normalized packet count"
    )
    coverage = finite_number(capture_document.get("source_coverage_s"), "source coverage")
    normalized_coverage = finite_number(
        raw_capture.get("source_coverage_s"), "normalized source coverage"
    )
    if (
        packet_count < 500
        or normalized_packet_count != packet_count
        or coverage < 4.0
        or not math.isclose(normalized_coverage, coverage, rel_tol=0.0, abs_tol=1e-12)
    ):
        raise ValueError("raw capture sample count or coverage is insufficient")

    safety = capture_document.get("safety")
    exact_safety = {
        "execution_policy": "BRAKE_ONLY",
        "command_rx_enabled": False,
        "j2_session_reference_configured": False,
        "foc_tx_attempt_count": 0,
        "foc_serial_send_call_count": 0,
        "other_mode_tx_attempt_count": 0,
        "active_or_hold_commands_sent": 0,
        "active_commands_sent": 0,
        "hold_commands_sent": 0,
        "communication_failure_packets": 0,
        "merror_nonzero_packets": 0,
        "motor_internal_zero_modified": False,
        "motor_internal_zero_write_count": 0,
        "rid_written": False,
        "rid_write_count": 0,
        "flash_or_eeprom_written": False,
        "flash_or_eeprom_write_count": 0,
        "startup_brake_prime_passed": True,
    }
    if not isinstance(safety, dict) or any(
        type(safety.get(field)) is not type(expected)
        or safety.get(field) != expected
        for field, expected in exact_safety.items()
    ) or safety.get("all_controller_modes") != ["brake"]:
        raise ValueError("raw capture BRAKE-only safety proof mismatch")
    physical_confirmation = capture_document.get("physical_confirmation")
    if not isinstance(physical_confirmation, dict) or (
        physical_confirmation.get("confirmation_gate")
        != "24V_ON=YES;SUPPORT_RELIABLE=YES;J2_VERTICAL_INITIALIZATION_POSE=YES;"
        "ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES"
    ) or any(
        physical_confirmation.get(field) is not True
        for field in (
            "support_reliable",
            "j2_vertical_initialization_pose",
            "arm_not_moved",
            "arm_stationary",
            "not_at_mechanical_limit",
        )
    ):
        raise ValueError("raw capture physical confirmation is incomplete")
    raw_summary = capture_document.get("raw_safety_summary")
    if not isinstance(raw_summary, dict) or any(
        raw_summary.get(field) is not False
        for field in (
            "branch_inputs_loaded",
            "persistent_zero_loaded",
            "recovery_hint_loaded",
            "integer_turn_branch_inferred",
        )
    ) or (
        type(raw_summary.get("udp_send_calls")) is not int
        or raw_summary.get("udp_send_calls") != 0
    ):
        raise ValueError("raw capture loaded forbidden reference inputs or sent UDP")
    raw_worker = raw_summary.get("worker") if isinstance(raw_summary, dict) else None
    normalized_worker = raw_capture.get("worker")
    if not isinstance(raw_worker, dict) or not isinstance(normalized_worker, dict):
        raise ValueError("raw capture worker evidence is missing")
    raw_worker_path = raw_worker.get("path")
    normalized_worker_path = normalized_worker.get("path")
    raw_worker_sha = normalized_sha256(raw_worker.get("sha256"), "raw worker")
    normalized_worker_sha = normalized_sha256(
        normalized_worker.get("sha256"), "normalized raw worker"
    )
    if (
        type(raw_worker.get("returncode")) is not int
        or raw_worker.get("returncode") != 0
        or raw_worker.get("stop_escalation") != "none"
        or raw_worker.get("forced_kill") is not False
    ):
        raise ValueError("raw worker did not terminate normally")
    terminal = raw_worker.get("terminal_proof")
    normalized_terminal = normalized_worker.get("terminal_proof")
    startup_prime = raw_summary.get("startup_brake_prime")
    normalized_startup_prime = raw_capture.get("startup_brake_prime")
    startup_fields = {
        "state",
        "maximum_invalid_prefix_pairs",
        "invalid_prefix_pairs",
        "required_healthy_pairs",
        "healthy_pairs",
        "attempted_pairs",
        "tx_attempt_count",
        "elapsed_ns",
        "feedback_published_during_prime",
        "window_reopens_after_first_healthy_pair",
    }
    if (
        not isinstance(startup_prime, dict)
        or not isinstance(normalized_startup_prime, dict)
        or set(startup_prime) != startup_fields
        or set(normalized_startup_prime) != startup_fields
        or any(
            type(normalized_startup_prime.get(field))
            is not type(startup_prime.get(field))
            or normalized_startup_prime.get(field) != startup_prime.get(field)
            for field in startup_fields
        )
        or startup_prime.get("state") != "PASS"
        or startup_prime.get("feedback_published_during_prime") is not False
        or startup_prime.get("window_reopens_after_first_healthy_pair") is not False
    ):
        raise ValueError("startup BRAKE prime structured evidence mismatch")
    maximum_invalid_prefix_pairs = exact_integer(
        startup_prime.get("maximum_invalid_prefix_pairs"),
        "startup prime maximum invalid-prefix pairs",
    )
    invalid_prefix_pairs = exact_integer(
        startup_prime.get("invalid_prefix_pairs"),
        "startup prime invalid-prefix pairs",
    )
    required_healthy_pairs = exact_integer(
        startup_prime.get("required_healthy_pairs"),
        "startup prime required healthy pairs",
    )
    healthy_pairs = exact_integer(
        startup_prime.get("healthy_pairs"), "startup prime healthy pairs"
    )
    attempted_pairs = exact_integer(
        startup_prime.get("attempted_pairs"), "startup prime attempted pairs"
    )
    prime_tx = exact_integer(
        startup_prime.get("tx_attempt_count"), "startup prime transmit count"
    )
    prime_elapsed_ns = exact_integer(
        startup_prime.get("elapsed_ns"), "startup prime elapsed time"
    )
    safety_invalid_prefix_pairs = exact_integer(
        safety.get("startup_invalid_prefix_pairs"),
        "safety startup invalid-prefix pairs",
    )
    if (
        maximum_invalid_prefix_pairs != 3
        or not 0 <= invalid_prefix_pairs <= 3
        or required_healthy_pairs != 5
        or healthy_pairs != 5
        or attempted_pairs != invalid_prefix_pairs + healthy_pairs
        or prime_tx != 2 * attempted_pairs
        or not 1 <= prime_elapsed_ns <= 500_000_000
        or safety_invalid_prefix_pairs != invalid_prefix_pairs
    ):
        raise ValueError("startup BRAKE prime counters or bounds mismatch")

    phase_tx = raw_summary.get("phase_tx_accounting")
    normalized_phase_tx = raw_capture.get("phase_tx_accounting")
    phase_fields = {
        "completed_cycles",
        "startup_prime_tx_attempt_count",
        "control_loop_tx_attempt_count",
        "final_brake_tx_attempt_count",
        "aggregate_tx_attempt_count",
    }
    if (
        not isinstance(phase_tx, dict)
        or not isinstance(normalized_phase_tx, dict)
        or set(phase_tx) != phase_fields
        or set(normalized_phase_tx) != phase_fields
        or any(
            type(normalized_phase_tx.get(field)) is not type(phase_tx.get(field))
            or normalized_phase_tx.get(field) != phase_tx.get(field)
            for field in phase_fields
        )
    ):
        raise ValueError("phase transmit accounting structured evidence mismatch")
    phase_counts = {
        field: exact_integer(phase_tx.get(field), f"phase transmit {field}")
        for field in phase_fields
    }
    expected_aggregate_tx = prime_tx + 2 * packet_count + 40
    if (
        phase_counts["completed_cycles"] != packet_count
        or phase_counts["startup_prime_tx_attempt_count"] != prime_tx
        or phase_counts["control_loop_tx_attempt_count"] != 2 * packet_count
        or phase_counts["final_brake_tx_attempt_count"] != 40
        or phase_counts["aggregate_tx_attempt_count"] != expected_aggregate_tx
    ):
        raise ValueError("phase transmit accounting is not the strict phase sum")

    exact_terminal_text = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": "j2",
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "J2_SESSION_REFERENCE_CONFIGURED": "NO",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "FOC_SERIAL_SEND_CALL_COUNT": "0",
        "BRAKE_ONLY_AUDIT": "PASS",
        "FINAL_MODE": "BRAKE",
        "FINAL_BRAKE": "PASS",
        "MOTOR_INTERNAL_ZERO_WRITE": "NO",
        "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
        "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": "3",
        "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": "5",
    }
    terminal_count_fields = {
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS",
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS",
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS",
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT",
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS",
        "COMPLETED_CYCLES",
        "FINAL_BRAKE_TX_ATTEMPT_COUNT",
        "TX_ATTEMPT_TOTAL",
        "BRAKE_TX_ATTEMPT_COUNT",
        "SERIAL_SEND_CALL_COUNT",
    }
    if (
        not isinstance(terminal, dict)
        or not isinstance(normalized_terminal, dict)
        or set(terminal) != set(exact_terminal_text) | terminal_count_fields
        or normalized_terminal != terminal
        or any(terminal.get(field) != value for field, value in exact_terminal_text.items())
    ):
        raise ValueError("raw and normalized worker terminal proof mismatch")
    terminal_counts = {
        field: canonical_count(
            terminal,
            field,
            positive=(
                field != "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"
            ),
        )
        for field in terminal_count_fields
    }
    expected_terminal_counts = {
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": invalid_prefix_pairs,
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": healthy_pairs,
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": attempted_pairs,
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": prime_tx,
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": prime_elapsed_ns,
        "COMPLETED_CYCLES": packet_count,
        "FINAL_BRAKE_TX_ATTEMPT_COUNT": 40,
        "TX_ATTEMPT_TOTAL": expected_aggregate_tx,
        "BRAKE_TX_ATTEMPT_COUNT": expected_aggregate_tx,
        "SERIAL_SEND_CALL_COUNT": expected_aggregate_tx,
    }
    if any(
        terminal_counts[field] != expected
        for field, expected in expected_terminal_counts.items()
    ):
        raise ValueError("worker terminal proof is not bound to structured evidence")
    if (
        raw_worker_path != str(worker_path)
        or normalized_worker_path != str(worker_path)
        or raw_worker_sha != worker_sha256
        or normalized_worker_sha != worker_sha256
        or raw_capture.get("expected_worker_sha256") != worker_sha256
    ):
        raise ValueError("raw capture worker does not match GO_BINARY")

    anchor_motors = anchor.get("motors")
    normalized_motors = raw_capture.get("motors")
    captured_motors = capture_document.get("motors")
    if not all(
        isinstance(value, dict) and set(value) == set(EXPECTED_SIGNS)
        for value in (anchor_motors, normalized_motors, captured_motors)
    ):
        raise ValueError("J2 motor set mismatch")
    for name, sign in EXPECTED_SIGNS.items():
        anchor_motor = anchor_motors[name]
        normalized_motor = normalized_motors[name]
        captured_motor = captured_motors[name]
        statistics = captured_motor.get("unwrapped_raw_position_rad")
        normalized_statistics = normalized_motor.get("unwrapped_raw_position_rad")
        if not isinstance(statistics, dict) or not isinstance(normalized_statistics, dict):
            raise ValueError(f"{name} raw statistics are missing")
        mean = finite_number(statistics.get("mean"), f"{name} raw mean")
        minimum = finite_number(statistics.get("minimum"), f"{name} raw minimum")
        maximum = finite_number(statistics.get("maximum"), f"{name} raw maximum")
        span = finite_number(statistics.get("span"), f"{name} raw span")
        if (
            any(abs(value) > MAX_RAW_POSITION_RAD for value in (mean, minimum, maximum))
            or not minimum <= mean <= maximum
            or span < 0.0
            or span > MAX_RAW_SPAN_RAD
            or not math.isclose(span, maximum - minimum, rel_tol=1e-9, abs_tol=1e-12)
            or captured_motor.get("sample_count") != packet_count
            or normalized_motor.get("sample_count") != packet_count
        ):
            raise ValueError(f"{name} raw statistics violate the stationary contract")
        for field in ("mean", "minimum", "maximum", "span", "standard_deviation"):
            if not math.isclose(
                finite_number(normalized_statistics.get(field), f"{name} normalized {field}"),
                finite_number(statistics.get(field), f"{name} captured {field}"),
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ValueError(f"{name} normalized raw statistics mismatch")
        reference = finite_number(
            anchor_motor.get("session_reference_raw_rad"), f"{name} session reference"
        )
        logical = finite_number(anchor_motor.get("logical_position_rad"), f"{name} logical")
        anchor_span = finite_number(
            anchor_motor.get("sample_span_raw_rad"), f"{name} anchor span"
        )
        gear = finite_number(anchor_motor.get("gear_ratio"), f"{name} gear ratio")
        if (
            (not preserved_startup and not math.isclose(reference, mean, rel_tol=0.0, abs_tol=1e-12))
            or abs(logical) > 1e-9
            or not math.isclose(anchor_span, span, rel_tol=0.0, abs_tol=1e-12)
            or anchor_motor.get("sample_count") != packet_count
            or type(anchor_motor.get("sign")) is not int
            or anchor_motor.get("sign") != sign
            or not math.isclose(gear, GEAR_RATIO, rel_tol=0.0, abs_tol=1e-6)
        ):
            raise ValueError(f"{name} anchor motor binding mismatch")

    if permit.get("schema") != "go-m8010-j2-power-session-launch-permit/1.0":
        raise ValueError("launch-permit schema mismatch")
    if (
        type(permit.get("permit_version")) is not int
        or permit.get("permit_version") != 1
        or permit.get("scope") != "j2"
        or permit.get("single_use") is not True
    ):
        raise ValueError("launch-permit scope or version mismatch")
    permit_id = normalized_sha256(permit.get("permit_id"), "launch permit id")
    expected_permit_id = hashlib.sha256(
        json.dumps(
            {
                "schema": "go-m8010-j2-power-session-launch-permit/1.0",
                "anchor_id": anchor_id,
                "anchor_sha256": anchor_sha256,
                "parent_persistent_zero_sha256": zero_sha256,
                "power_session_id": expected_power_session_id,
                "host_boot_id": current_boot_id,
                "worker_sha256": worker_sha256,
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    if permit_id != expected_permit_id:
        raise ValueError("launch-permit id is not derived from its immutable bindings")
    expected_pending = pending_directory / f"{permit_id}.json"
    expected_inflight = inflight_directory / f"{permit_id}.json"
    expected_spent = spent_directory / f"{permit_id}.json"
    if (
        permit_path != expected_pending
        or permit.get("pending_path") != str(expected_pending)
        or permit.get("inflight_path") != str(expected_inflight)
        or permit.get("spent_path") != str(expected_spent)
    ):
        raise ValueError("launch-permit lifecycle path mismatch")
    if path_lexists(expected_inflight) or path_lexists(expected_spent):
        raise ValueError("launch permit was already consumed or spent")
    if (
        permit.get("power_session_id") != expected_power_session_id
        or normalized_boot_id(permit.get("host_boot_id"), "permit host boot id")
        != current_boot_id
        or permit.get("parent_persistent_zero_sha256") != zero_sha256
    ):
        raise ValueError("launch-permit parent, boot, or power-session mismatch")
    permit_reference = permit.get("session_reference")
    if not isinstance(permit_reference, dict) or (
        permit_reference.get("path") != str(anchor_path)
        or permit_reference.get("sha256") != anchor_sha256
    ):
        raise ValueError("launch permit does not bind the canonical anchor and SHA-256")
    permit_worker = permit.get("worker")
    if not isinstance(permit_worker, dict) or (
        permit_worker.get("path") != str(worker_path)
        or permit_worker.get("sha256") != worker_sha256
    ):
        raise ValueError("launch permit does not bind the compiled GO_BINARY")
    source_capture = permit.get("source_capture")
    if not isinstance(source_capture, dict) or (
        source_capture.get("host_boot_id") != current_boot_id
        or source_capture.get("recorded_boottime_ns") != capture_boottime_ns
    ):
        raise ValueError("launch-permit source-capture binding mismatch")

    issued_boottime_ns = positive_integer(
        permit.get("issued_boottime_ns"), "permit issued CLOCK_BOOTTIME"
    )
    expires_boottime_ns = positive_integer(
        permit.get("expires_boottime_ns"), "permit expires CLOCK_BOOTTIME"
    )
    ttl_seconds = positive_integer(permit.get("ttl_seconds"), "permit TTL")
    ttl_ns = expires_boottime_ns - issued_boottime_ns
    if ttl_seconds > 30 or ttl_ns <= 0 or ttl_ns > MAX_TTL_NS:
        raise ValueError("launch-permit TTL is outside (0, 30] seconds")
    if ttl_ns != ttl_seconds * 1_000_000_000:
        raise ValueError("launch-permit TTL fields disagree")
    if capture_boottime_ns > issued_boottime_ns or (
        issued_boottime_ns - capture_boottime_ns > MAX_CAPTURE_TO_PERMIT_NS
    ):
        raise ValueError("raw capture is outside the 300-second issuance window")
    issued_unix_s = positive_integer(permit.get("issued_unix_s"), "permit issued unix time")
    expires_unix_s = positive_integer(permit.get("expires_unix_s"), "permit expires unix time")
    issued_at = parse_utc(permit.get("issued_at_utc"), "permit issued timestamp")
    expires_at = parse_utc(permit.get("expires_at_utc"), "permit expires timestamp")
    if (
        int(issued_at.timestamp()) != issued_unix_s
        or int(expires_at.timestamp()) != expires_unix_s
        or expires_unix_s - issued_unix_s != ttl_seconds
        or abs((expires_at - issued_at).total_seconds() - ttl_seconds) > 1e-9
    ):
        raise ValueError("launch-permit wall-clock fields disagree")

    serial_scope = permit.get("serial")
    motor_ids = serial_scope.get("motor_ids") if isinstance(serial_scope, dict) else None
    serial_signs = serial_scope.get("signs") if isinstance(serial_scope, dict) else None
    if not isinstance(serial_scope, dict) or (
        serial_scope.get("stable_by_id") != EXPECTED_SERIAL
        or serial_scope.get("bus") != "j2"
        or motor_ids != [0, 1]
        or not isinstance(motor_ids, list)
        or any(type(value) is not int for value in motor_ids)
        or not math.isclose(
            finite_number(serial_scope.get("gear_ratio"), "permit gear ratio"),
            GEAR_RATIO,
            rel_tol=0.0,
            abs_tol=1e-6,
        )
        or serial_signs != EXPECTED_SIGNS
        or not isinstance(serial_signs, dict)
        or any(type(value) is not int for value in serial_signs.values())
    ):
        raise ValueError("launch-permit serial or motor scope mismatch")
    recheck = permit.get("startup_recheck")
    if not isinstance(recheck, dict):
        raise ValueError("launch-permit startup recheck is missing")
    minimum_frames = positive_integer(
        recheck.get("minimum_brake_frames"), "minimum startup BRAKE frames"
    )
    maximum_delta = finite_number(
        recheck.get("max_raw_phase_delta_rad"), "startup raw phase delta"
    )
    maximum_span = finite_number(
        recheck.get("max_raw_span_rad"), "startup raw span"
    )
    if (
        minimum_frames < 50
        or minimum_frames > 150
        or maximum_delta <= 0.0
        or maximum_delta > MAX_STARTUP_DELTA_RAD + 1e-12
        or maximum_span <= 0.0
        or maximum_span > MAX_RAW_SPAN_RAD + 1e-12
    ):
        raise ValueError("launch-permit startup recheck exceeds the consumer contract")

    clock_reader = getattr(time, "clock_gettime_ns", None)
    clock_id = getattr(time, "CLOCK_BOOTTIME", None)
    if not callable(clock_reader) or type(clock_id) is not int:
        raise ValueError("CLOCK_BOOTTIME is unavailable")
    now_boottime_ns = positive_integer(
        clock_reader(clock_id), "current CLOCK_BOOTTIME"
    )
    if now_boottime_ns < issued_boottime_ns or now_boottime_ns >= expires_boottime_ns:
        raise ValueError("launch permit is future-dated or expired")

    print(anchor_path)
    print(anchor_sha256)
    print(permit_path)
    print(expected_power_session_id)
    print(worker_sha256)
except Exception as exc:
    print(f"J2 session launch-gate validation failed: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 0 ]] && ! command -v g++ >/dev/null; then
  for domain in J1 J2 J345; do
    mark_domain_unavailable "$domain" "缺少 g++ 编译器"
  done
elif ! go_sdk_build_inputs_present; then
  for domain in J1 J2 J345; do
    mark_domain_unavailable "$domain" "Unitree 电机 SDK 不完整"
  done
fi

J6_ENV_LIB=""
J6_LD_LIBRARY_PATH=""
if [[ "$PREBUILD_ONLY" -eq 0 ]]; then
  if [[ ! -x "$J6_PY" ]]; then
    mark_domain_unavailable J6 "J6 Python 3.13 / dmcan 环境不存在：$J6_PY"
  elif ! J6_ENV_LIB="$("$J6_PY" -c \
    'import pathlib, sys, sysconfig
candidates = (sysconfig.get_config_var("LIBDIR"), pathlib.Path(sys.prefix) / "lib")
for candidate in candidates:
    if candidate and (pathlib.Path(candidate) / "libstdc++.so.6").exists():
        print(pathlib.Path(candidate).resolve())
        break
else:
    raise SystemExit(1)')"; then
    mark_domain_unavailable J6 "无法从 J6 Python 推导共享库目录"
  elif [[ ! -d "$J6_ENV_LIB" ]]; then
    mark_domain_unavailable J6 "J6 Python 共享库目录不存在：$J6_ENV_LIB"
  else
    J6_LD_LIBRARY_PATH="$J6_ENV_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    if ! env LD_LIBRARY_PATH="$J6_LD_LIBRARY_PATH" "$J6_PY" -c \
      'import dmcan, serial; from dmcan import DmCanContext; assert DmCanContext' \
      >/dev/null 2>&1; then
      mark_domain_unavailable J6 "J6 Python 3.13 无法加载 dmcan / pyserial 及其共享库"
    fi
  fi
fi

udp_port_in_use() {
  local port=$1
  [[ -n "$(ss -H -lun "sport = :$port")" ]]
}

udp_port_owned_by_pid() {
  local port=$1
  local pid=$2
  ss -H -lunp "sport = :$port" 2>/dev/null | \
    awk -v owner="pid=$pid," 'index($0, owner) { found=1 } END { exit !found }'
}

declare -A DOMAIN_PORT=([J1]=15310 [J2]=15312 [J345]=15313 [J6]=15311)
declare -A GO_DEVICE=(
  [J1]='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0'
  [J2]='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0'
  [J345]='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0'
)
if [[ "$PREBUILD_ONLY" -eq 0 ]]; then
  if [[ "$REUSE_RUNNING_ARM_GUI_CORE" != "true" ]] && udp_port_in_use 15300; then
    fail "ROS2 硬件状态核心 UDP 端口 15300 已被占用"
  fi
  for domain in J1 J2 J345 J6; do
    if udp_port_in_use "${DOMAIN_PORT[$domain]}"; then
      mark_domain_unavailable "$domain" "UDP 命令端口 ${DOMAIN_PORT[$domain]} 已被占用"
    fi
  done
  for domain in J1 J2 J345; do
    [[ "${DOMAIN_READY[$domain]}" -eq 1 ]] || continue
    stable_device=${GO_DEVICE[$domain]}
    if [[ ! -e "$stable_device" ]]; then
      mark_domain_unavailable "$domain" "GO 稳定串口链接不存在：$stable_device"
      continue
    fi
    resolved_device="$(readlink -f "$stable_device")"
    if fuser "$resolved_device" >/dev/null 2>&1; then
      mark_domain_unavailable "$domain" "物理串口已被其他进程占用：$resolved_device"
    fi
  done
fi

# Any signed 30-second GO permit is a launch-only phase: no dependency install,
# colcon build, or controller compilation is allowed after it exists.  The
# explicit --prebuild-only run refreshes the binary and provenance manifest
# before a new raw capture/anchor/permit bundle is issued.
if [[ "${DOMAIN_READY[J1]}" -eq 1 || "${DOMAIN_READY[J2]}" -eq 1 || \
      "${DOMAIN_READY[J345]}" -eq 1 ]]; then
  GO_BUILD_FAILURE=""
  GO_REUSED_FROM_J2_PERMIT=0
  GO_BUILD_INPUT_FINGERPRINT=""
  if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 ]]; then
    if [[ ! -x "$GO_BINARY" ]] || \
        ! basic_j2_permit_worker_matches "" >/dev/null 2>&1; then
      GO_BUILD_FAILURE="签发 permit 后禁止编译，且当前 GO 构建 manifest 无效"
    else
      GO_REUSED_FROM_J2_PERMIT=1
      if ! basic_j2_permit_worker_matches "$J2_SESSION_LAUNCH_PERMIT" \
          >/dev/null 2>&1; then
        mark_domain_unavailable J2 \
          "J2 permit/worker 与当前受保护构建 manifest 不匹配；禁止现场重编译"
        clear_j2_session_inputs
      fi
      if ! basic_j2_permit_worker_matches "$GO_AUX_J1_LAUNCH_PERMIT" \
          >/dev/null 2>&1 || \
         ! basic_j2_permit_worker_matches "$GO_AUX_J345_LAUNCH_PERMIT" \
          >/dev/null 2>&1; then
        for domain in J1 J345; do
          mark_domain_unavailable "$domain" \
            "GO-AUX permit/worker 与当前受保护构建 manifest 不匹配；禁止现场重编译"
        done
        clear_go_aux_session_inputs
      fi
      printf '三个 GO permit、当前构建 manifest 与 GO_BINARY 已完成 worker 绑定校验；仅执行离线自检…\n'
    fi
  else
    printf '预构建阶段：正在严格编译 GO 四轴物理总线控制器…\n'
    if ! GO_COMPILE_SOURCE="$(readlink -f -- "$GO_CONTROLLER_SOURCE")" || \
       ! GO_COMPILE_BINARY="$(readlink -f -- "$GO_BINARY")" || \
       ! GO_COMPILE_SDK_INCLUDE="$(readlink -f -- "$GO_SDK_INCLUDE")" || \
       ! GO_COMPILE_SDK_LIBRARY="$(readlink -f -- "$GO_SDK_LIBRARY")"; then
      GO_BUILD_FAILURE="GO 严格编译规范路径解析失败"
    elif ! GO_BUILD_INPUT_FINGERPRINT="$(capture_go_build_inputs \
        "$GO_COMPILE_SOURCE" "$GO_COMPILE_SDK_INCLUDE" \
        "$GO_COMPILE_SDK_LIBRARY" "${GO_SDK_HEADERS[@]}")"; then
      GO_BUILD_FAILURE="GO 编译前输入指纹记录失败"
    elif ! g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
      -I"$GO_COMPILE_SDK_INCLUDE" \
      "$GO_COMPILE_SOURCE" \
      "$GO_COMPILE_SDK_LIBRARY" \
      "-Wl,-rpath,$(dirname -- "$GO_COMPILE_SDK_LIBRARY")" \
      -o "$GO_COMPILE_BINARY"; then
      GO_BUILD_FAILURE="GO 控制器严格编译失败"
    elif ! verify_go_build_inputs_unchanged "$GO_BUILD_INPUT_FINGERPRINT"; then
      GO_BUILD_FAILURE="GO 控制器编译期间源码、SDK 库或 SDK headers 已变化"
    fi
  fi

  if [[ -z "$GO_BUILD_FAILURE" ]] && ! run_go_offline_self_test; then
    GO_BUILD_FAILURE="GO 控制器离线自检失败或未证明 SERIAL_OPENED=NO"
  fi
  if [[ -z "$GO_BUILD_FAILURE" && "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 0 ]] && \
      ! verify_go_build_inputs_unchanged "$GO_BUILD_INPUT_FINGERPRINT"; then
    GO_BUILD_FAILURE="GO 离线自检期间源码、SDK 库或 SDK headers 已变化"
  fi
  if [[ -z "$GO_BUILD_FAILURE" && "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 0 ]] && \
      ! publish_go_build_manifest "$GO_BUILD_INPUT_FINGERPRINT"; then
    GO_BUILD_FAILURE="GO 严格构建 manifest 原子发布失败"
  fi
  if [[ -n "$GO_BUILD_FAILURE" ]]; then
    for domain in J1 J2 J345; do
      if [[ "${DOMAIN_READY[$domain]}" -eq 1 ]]; then
        mark_domain_unavailable "$domain" "$GO_BUILD_FAILURE"
      fi
    done
  fi
else
  printf '提示：没有可启动的 GO 故障域，跳过 GO 构建缓存检查。\n' >&2
fi

# These are the last GO authorization gates.  They follow only the bounded
# manifest check and no-serial self-test, then run before dependency work, ROS,
# or any physical worker.  A build-only invocation has no authorization input
# and deliberately skips both gates.
if [[ "$PREBUILD_ONLY" -eq 0 && "${DOMAIN_READY[J2]}" -eq 1 ]]; then
  J2_GATE_FAILURE=""
  J2_GATE_OUTPUT=""
  if [[ -z "$J2_SESSION_REFERENCE" ]]; then
    J2_GATE_FAILURE="缺少显式 J2_SESSION_REFERENCE_FILE"
  elif [[ -z "$J2_SESSION_LAUNCH_PERMIT" ]]; then
    J2_GATE_FAILURE="缺少显式 J2_SESSION_LAUNCH_PERMIT_FILE"
  elif [[ -z "$J2_POWER_SESSION_ID" ]]; then
    J2_GATE_FAILURE="缺少显式 J2_POWER_SESSION_ID"
  elif [[ ! -f "$J2_SESSION_REFERENCE" || ! -f "$J2_SESSION_LAUNCH_PERMIT" ]]; then
    J2_GATE_FAILURE="J2 anchor 或一次性 launch permit 不存在"
  elif [[ ! -f "$RECOVERY_BRANCH_HINTS" || ! -f "$INITIAL_POSE" ]]; then
    J2_GATE_FAILURE="J2 anchor 所绑定的 hints 或竖直 initial_pose 不存在"
  elif ! J2_GATE_OUTPUT="$(validate_j2_session_launch_gate \
      "$J2_SESSION_REFERENCE" "$J2_SESSION_LAUNCH_PERMIT" \
      "$J2_POWER_SESSION_ID")"; then
    J2_GATE_FAILURE="J2 anchor/permit 完整证据、生命周期或启动契约校验失败"
  else
    mapfile -t J2_GATE_FIELDS <<<"$J2_GATE_OUTPUT"
    if [[ "${#J2_GATE_FIELDS[@]}" -ne 5 ]]; then
      J2_GATE_FAILURE="J2 launch gate 返回了非预期结果"
    else
      J2_SESSION_REFERENCE="${J2_GATE_FIELDS[0]}"
      J2_SESSION_REFERENCE_SHA256="${J2_GATE_FIELDS[1]}"
      J2_SESSION_LAUNCH_PERMIT="${J2_GATE_FIELDS[2]}"
      J2_POWER_SESSION_ID="${J2_GATE_FIELDS[3]}"
      J2_EXPECTED_WORKER_SHA256="${J2_GATE_FIELDS[4]}"
    fi
  fi

  if [[ -z "$J2_GATE_FAILURE" ]]; then
    J2_BUNDLE_AUDIT_OUTPUT=""
    if ! J2_BUNDLE_AUDIT_OUTPUT="$("$GO_BINARY" \
        --audit-j2-session-bundle --bus j2 \
        --zero-file "$PERSISTENT_ZERO" \
        --recovery-hint-file "$RECOVERY_BRANCH_HINTS" \
        --j2-session-reference-file "$J2_SESSION_REFERENCE" \
        --j2-session-launch-permit-file "$J2_SESSION_LAUNCH_PERMIT" \
        --expected-zero-sha256 "$PERSISTENT_ZERO_SHA256" \
        --expected-j2-session-reference-sha256 "$J2_SESSION_REFERENCE_SHA256" \
        --expected-j2-power-session-id "$J2_POWER_SESSION_ID" \
        --expected-worker-sha256 "$J2_EXPECTED_WORKER_SHA256")"; then
      J2_GATE_FAILURE="GO consumer 拒绝 J2 anchor/permit 离线审计"
    elif ! grep -Fxq 'SERIAL_OPENED=NO' <<<"$J2_BUNDLE_AUDIT_OUTPUT" || \
         ! grep -Fxq 'J2_SESSION_BUNDLE_AUDIT=PASS' <<<"$J2_BUNDLE_AUDIT_OUTPUT"; then
      J2_GATE_FAILURE="GO consumer 离线审计未证明 SERIAL_OPENED=NO 与 bundle PASS"
    fi
  fi

  if [[ -n "$J2_GATE_FAILURE" ]]; then
    mark_domain_unavailable J2 "$J2_GATE_FAILURE"
    clear_j2_session_inputs
  fi
elif [[ "$PREBUILD_ONLY" -eq 0 ]]; then
  clear_j2_session_inputs
fi

if [[ "$PREBUILD_ONLY" -eq 0 && \
      ( "${DOMAIN_READY[J1]}" -eq 1 || "${DOMAIN_READY[J345]}" -eq 1 ) ]]; then
  GO_AUX_GATE_FAILURE=""
  GO_AUX_GATE_OUTPUT=""
  if [[ -z "$GO_AUX_SESSION_REFERENCE" ]]; then
    GO_AUX_GATE_FAILURE="缺少显式 GO_AUX_SESSION_REFERENCE_FILE"
  elif [[ -z "$GO_AUX_J1_LAUNCH_PERMIT" || \
          -z "$GO_AUX_J345_LAUNCH_PERMIT" ]]; then
    GO_AUX_GATE_FAILURE="缺少 J1/J345 两个显式一次性 GO-AUX launch permit"
  elif [[ -z "$GO_AUX_POWER_SESSION_ID" ]]; then
    GO_AUX_GATE_FAILURE="缺少显式 GO_AUX_POWER_SESSION_ID"
  elif [[ ! -f "$GO_AUX_SESSION_REFERENCE" || \
          ! -f "$GO_AUX_J1_LAUNCH_PERMIT" || \
          ! -f "$GO_AUX_J345_LAUNCH_PERMIT" ]]; then
    GO_AUX_GATE_FAILURE="GO-AUX anchor 或两个一次性 permit 不存在"
  elif [[ ! -f "$GO_AUX_VALIDATOR" || ! -f "$RECOVERY_BRANCH_HINTS" || \
          ! -f "$INITIAL_POSE" ]]; then
    GO_AUX_GATE_FAILURE="GO-AUX validator 或 anchor 所绑定的受保护输入不存在"
  elif ! GO_AUX_GATE_OUTPUT="$(python3 "$GO_AUX_VALIDATOR" \
      --anchor "$GO_AUX_SESSION_REFERENCE" \
      --j1-permit "$GO_AUX_J1_LAUNCH_PERMIT" \
      --j345-permit "$GO_AUX_J345_LAUNCH_PERMIT" \
      --power-session-id "$GO_AUX_POWER_SESSION_ID" \
      --zero "$PERSISTENT_ZERO" \
      --hints "$RECOVERY_BRANCH_HINTS" \
      --initial-pose "$INITIAL_POSE" \
      --worker "$GO_BINARY")"; then
    GO_AUX_GATE_FAILURE="GO-AUX anchor/permits 的哈希、boot、生命周期或 initial_pose 绑定校验失败"
  else
    mapfile -t GO_AUX_GATE_FIELDS <<<"$GO_AUX_GATE_OUTPUT"
    if [[ "${#GO_AUX_GATE_FIELDS[@]}" -ne 6 ]]; then
      GO_AUX_GATE_FAILURE="GO-AUX launch gate 返回了非预期结果"
    else
      GO_AUX_SESSION_REFERENCE="${GO_AUX_GATE_FIELDS[0]}"
      GO_AUX_SESSION_REFERENCE_SHA256="${GO_AUX_GATE_FIELDS[1]}"
      GO_AUX_J1_LAUNCH_PERMIT="${GO_AUX_GATE_FIELDS[2]}"
      GO_AUX_J345_LAUNCH_PERMIT="${GO_AUX_GATE_FIELDS[3]}"
      GO_AUX_POWER_SESSION_ID="${GO_AUX_GATE_FIELDS[4]}"
      GO_AUX_EXPECTED_WORKER_SHA256="${GO_AUX_GATE_FIELDS[5]}"
    fi
  fi

  if [[ -z "$GO_AUX_GATE_FAILURE" ]]; then
    for go_aux_bus in j1 j345; do
      if [[ "$go_aux_bus" == "j1" ]]; then
        go_aux_permit="$GO_AUX_J1_LAUNCH_PERMIT"
      else
        go_aux_permit="$GO_AUX_J345_LAUNCH_PERMIT"
      fi
      GO_AUX_BUNDLE_AUDIT_OUTPUT=""
      if ! GO_AUX_BUNDLE_AUDIT_OUTPUT="$("$GO_BINARY" \
          --audit-go-aux-session-bundle --bus "$go_aux_bus" \
          --zero-file "$PERSISTENT_ZERO" \
          --recovery-hint-file "$RECOVERY_BRANCH_HINTS" \
          --go-aux-session-reference-file "$GO_AUX_SESSION_REFERENCE" \
          --go-aux-session-launch-permit-file "$go_aux_permit" \
          --expected-zero-sha256 "$PERSISTENT_ZERO_SHA256" \
          --expected-go-aux-session-reference-sha256 \
            "$GO_AUX_SESSION_REFERENCE_SHA256" \
          --expected-go-aux-power-session-id "$GO_AUX_POWER_SESSION_ID" \
          --expected-worker-sha256 "$GO_AUX_EXPECTED_WORKER_SHA256")"; then
        GO_AUX_GATE_FAILURE="GO consumer 拒绝 $go_aux_bus anchor/permit 离线审计"
        break
      elif ! grep -Fxq 'DRY_RUN=YES' <<<"$GO_AUX_BUNDLE_AUDIT_OUTPUT" || \
           ! grep -Fxq 'SERIAL_OPENED=NO' <<<"$GO_AUX_BUNDLE_AUDIT_OUTPUT" || \
           ! grep -Fxq 'GO_AUX_SESSION_BUNDLE_AUDIT=PASS' \
             <<<"$GO_AUX_BUNDLE_AUDIT_OUTPUT"; then
        GO_AUX_GATE_FAILURE="$go_aux_bus consumer 审计未证明 DRY_RUN/SERIAL_OPENED=NO/bundle PASS"
        break
      fi
    done
  fi

  if [[ -n "$GO_AUX_GATE_FAILURE" ]]; then
    for domain in J1 J345; do
      if [[ "${DOMAIN_READY[$domain]}" -eq 1 ]]; then
        mark_domain_unavailable "$domain" "$GO_AUX_GATE_FAILURE"
      fi
    done
    clear_go_aux_session_inputs
  fi
elif [[ "$PREBUILD_ONLY" -eq 0 ]]; then
  clear_go_aux_session_inputs
fi

if [[ "$PREBUILD_ONLY" -eq 0 ]]; then
  for required_domain in J1 J2 J345 J6; do
    [[ "${DOMAIN_READY[$required_domain]}" -eq 1 ]] || \
      fail "完整签名启动拒绝故障域降级：$required_domain：${DOMAIN_REASON[$required_domain]}"
  done

  for reference_sha256 in \
    "$PERSISTENT_ZERO_SHA256" \
    "$J2_SESSION_REFERENCE_SHA256" \
    "$GO_AUX_SESSION_REFERENCE_SHA256"; do
    [[ "$reference_sha256" =~ ^[0-9a-f]{64}$ ]] || \
      fail "J6 feedback identity 需要完整有效的 zero/J2/GO-AUX SHA256"
  done
  reference_session_id="persistent:${PERSISTENT_ZERO_SHA256:0:16}"
  reference_session_id+=":j2session:${J2_SESSION_REFERENCE_SHA256:0:16}"
  reference_session_id+=":goauxsession:${GO_AUX_SESSION_REFERENCE_SHA256:0:16}"
  if [[ -n "$GRAVITY_ANCHOR" ]]; then
    [[ "$EXPECTED_GRAVITY_SESSION_ID" == "$reference_session_id" ]] || \
      fail "active gravity anchor session 与当前 zero/J2/GO-AUX 引用不一致"
    J6_FEEDBACK_SESSION_ID="$EXPECTED_GRAVITY_SESSION_ID"
    J6_FEEDBACK_STATE_INSTANCE_ID="$EXPECTED_GRAVITY_STATE_INSTANCE_ID"
  else
    # NONE is still an empty gravity-authority binding.  This independent
    # feedback identity only binds raw J6 samples to the state-node instance.
    [[ -z "$EXPECTED_GRAVITY_SESSION_ID" && \
       -z "$EXPECTED_GRAVITY_STATE_INSTANCE_ID" ]] || \
      fail "NONE gravity authority 不得携带 session/state pins"
    J6_FEEDBACK_SESSION_ID="$reference_session_id"
    J6_FEEDBACK_STATE_INSTANCE_ID="$(
      python3 -c 'import secrets; print(secrets.token_hex(16))'
    )"
  fi
  [[ "$J6_FEEDBACK_SESSION_ID" =~ \
     ^persistent:[0-9a-f]{16}:j2session:[0-9a-f]{16}:goauxsession:[0-9a-f]{16}$ ]] || \
    fail "冻结的 J6 feedback session_id 无效"
  [[ "$J6_FEEDBACK_STATE_INSTANCE_ID" =~ ^[0-9a-f]{32}$ ]] || \
    fail "冻结的 J6 feedback state_instance_id 无效"
fi

if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 && ! -x "$GUI_PY" ]]; then
  fail "已签发 GO permit，但 GUI 预构建环境不存在；本次禁止安装依赖"
elif [[ ! -x "$GUI_PY" ]]; then
  printf '正在创建 GUI 专用 Python 3.10 环境…\n'
  python3 -m venv --system-site-packages "$GUI_VENV"
fi
check_gui_dependencies() {
  "$GUI_PY" -c '
import mujoco
import numpy
import PySide6
import serial
import yaml

actual = {
    "mujoco": mujoco.__version__,
    "numpy": numpy.__version__,
    "PySide6": PySide6.__version__,
    "PyYAML": yaml.__version__,
    "pyserial": serial.__version__,
}
expected = {
    "mujoco": "3.11.0",
    "numpy": "2.2.6",
    "PySide6": "6.8.3",
    "PyYAML": "6.0.2",
    "pyserial": "3.5",
}
assert actual == expected, (actual, expected)
' >/dev/null 2>&1 || return 1

  local pip_report
  if pip_report="$("$GUI_PY" -m pip check 2>&1)"; then
    return 0
  fi
  PIP_CHECK_REPORT="$pip_report" "$GUI_PY" - "$GUI_VENV" <<'PY'
import importlib.metadata
import os
import pathlib
import re
import sys

venv_root = pathlib.Path(sys.argv[1]).resolve()
report = os.environ.get("PIP_CHECK_REPORT", "")
pattern = re.compile(
    r"^(?P<owner>[A-Za-z0-9_.-]+)\s+\S+\s+(?:requires|has requirement)\s+"
)
external = []
blocking = []

for line in (item.strip() for item in report.splitlines()):
    if not line:
        continue
    match = pattern.match(line)
    if match is None:
        print(f"无法解析 pip check 输出，按失败处理: {line}", file=sys.stderr)
        raise SystemExit(2)
    owner = match.group("owner")
    try:
        distribution = importlib.metadata.distribution(owner)
        owner_root = pathlib.Path(distribution.locate_file("")).resolve()
    except Exception as exc:
        print(f"无法定位 pip check 问题包 {owner}: {exc}", file=sys.stderr)
        raise SystemExit(2)
    try:
        owner_root.relative_to(venv_root)
        inside_venv = True
    except ValueError:
        inside_venv = False
    (blocking if inside_venv else external).append((line, owner_root))

for line, owner_root in external:
    print(
        f"提示：忽略 GUI venv 之外的系统 Python 元数据问题：{line} [{owner_root}]",
        file=sys.stderr,
    )
for line, owner_root in blocking:
    print(f"GUI venv 依赖损坏：{line} [{owner_root}]", file=sys.stderr)

if not external and not blocking:
    print("pip check 失败但没有可分析的报告，按失败处理", file=sys.stderr)
    raise SystemExit(2)
raise SystemExit(1 if blocking else 0)
PY
}

if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 ]]; then
  check_gui_dependencies || \
    fail "已签发 GO permit，但 GUI 预构建依赖校验失败；本次禁止安装依赖"
else
  if ! check_gui_dependencies; then
    printf '正在安装锁定的 GUI 依赖（首次启动需联网）…\n'
    "$GUI_PY" -m pip install --disable-pip-version-check \
      -r "$REPO_ROOT/requirements-arm-gui.txt"
  fi
  check_gui_dependencies || fail "GUI Python 锁定版本或 pip 依赖完整性校验失败"
fi

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 ]]; then
  [[ -f "$ROS_WS/install/setup.bash" ]] || \
    fail "已签发 GO permit，但 ROS2 预构建产物不存在；本次禁止运行 colcon"
  printf 'GO permit 已签发：跳过依赖安装、colcon 与 GO 编译，仅使用预构建产物。\n'
else
  printf '预构建阶段：正在构建两个最小 ROS2 包…\n'
  (cd "$ROS_WS" && "$GUI_PY" -m colcon build --symlink-install \
    --packages-select go_m8010_arm_hardware go_m8010_arm_gui \
    --event-handlers console_direct+)
fi
set +u
source "$ROS_WS/install/setup.bash"
set -u
for required_package in go_m8010_arm_hardware go_m8010_arm_gui; do
  ros2 pkg prefix "$required_package" >/dev/null 2>&1 || \
    fail "ROS2 构建后无法发现必需包：$required_package"
done
expected_shebang="#!$GUI_PY"
for installed_entry in \
  "$ROS_WS/install/go_m8010_arm_gui/lib/go_m8010_arm_gui/arm_gui" \
  "$ROS_WS/install/go_m8010_arm_hardware/lib/go_m8010_arm_hardware/whole_arm_mujoco_mirror"; do
  [[ "$(head -n 1 "$installed_entry")" == "$expected_shebang" ]] || \
    fail "ROS2 Python 入口未绑定 GUI 专用 Python：$installed_entry"
done

if [[ "$PREBUILD_ONLY" -eq 1 ]]; then
  basic_j2_permit_worker_matches "" >/dev/null 2>&1 || \
    fail "预构建结束时 GO_BINARY 与受保护 build manifest 不再匹配"
  printf 'GO_BUILD_MANIFEST=PASS\n'
  printf 'SERIAL_OPENED=NO\n'
  printf 'UDP_OPENED=NO\n'
  printf 'ROS_LAUNCHED=NO\n'
  printf 'PREBUILD_ONLY=PASS\n'
  exit 0
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
# Humble adds an SHM transport when ROS_LOCALHOST_ONLY=1 even after loading
# XML. Restrict the custom UDP interface instead, and reject other profiles.
LOCAL_DDS_PROFILE="$REPO_ROOT/tools/hardware/v15_31d_demo_templates/fastdds_udp_only.xml"
[[ -r "$LOCAL_DDS_PROFILE" ]] || fail "缺少本机回环 DDS 配置：$LOCAL_DDS_PROFILE"
for profile_variable in FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; do
  if [[ -n "${!profile_variable:-}" ]] && ! cmp -s "${!profile_variable}" "$LOCAL_DDS_PROFILE"; then
    fail "$profile_variable 必须与仓库的仅回环 UDP 配置内容一致"
  fi
done
[[ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" == rmw_fastrtps_cpp ]] || \
  fail "仅回环 UDP 启动需要 rmw_fastrtps_cpp"
export FASTRTPS_DEFAULT_PROFILES_FILE="$LOCAL_DDS_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$LOCAL_DDS_PROFILE"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export MUJOCO_GL=glfw
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1

if [[ "$REUSE_RUNNING_ARM_GUI_CORE" == "true" ]]; then
  printf '正在复用贯穿只读捕获与主动验收的同一 arm GUI/state 核心…\n'
  ROS_PID=""
else
  printf '正在启动全中文界面与 MuJoCo 可视镜像…\n'
  (
    exec {INSTANCE_LOCK_FD}>&-
    exec {LEGACY_FEEDBACK_LOCK_FD}>&-
    exec ros2 launch go_m8010_arm_gui arm_gui.launch.py \
      model_path:="$MODEL" \
      session_pose_deg:="$MUJOCO_SOFTWARE_ZERO_DEG" \
      pose_matched:=true \
      config_path:="$GUI_SOURCE/config/arm_gui.yaml" \
      joint_limits_path:="$GUI_SOURCE/config/gui_joint_limits.yaml" \
      initial_pose_path:="$INITIAL_POSE" \
      initial_pose_read_only:=true \
      persistent_zero_path:="$PERSISTENT_ZERO" \
      recovery_hint_path:="$RECOVERY_BRANCH_HINTS" \
      j2_session_reference_path:="$J2_SESSION_REFERENCE" \
      go_aux_session_reference_path:="$GO_AUX_SESSION_REFERENCE" \
      state_instance_id:="$J6_FEEDBACK_STATE_INSTANCE_ID" \
      gravity_anchor_path:="$GRAVITY_ANCHOR" \
      empirical_envelope_path:="$EMPIRICAL_ENVELOPE" \
      expected_empirical_envelope_sha256:="$EMPIRICAL_ENVELOPE_SHA256" \
      gravity_enabled_for_hardware:="$GRAVITY_ENABLED_FOR_HARDWARE" \
      gravity_scale_target:="$GRAVITY_SCALE_TARGET" \
      runtime_log_directory:="$RUN_DIR"
  ) &
  ROS_PID=$!
fi

state_ready=0
for _ in $(seq 1 100); do
  if [[ "$REUSE_RUNNING_ARM_GUI_CORE" == "true" ]]; then
    managed_pid_is_live "$EXTERNAL_ARM_GUI_CORE_PID" || \
      fail "复用的 arm GUI 核心进程提前退出"
  elif ! kill -0 "$ROS_PID" 2>/dev/null; then
    fail "ROS2 启动进程提前退出"
  fi
  if udp_port_in_use 15300; then state_ready=1; break; fi
  sleep 0.1
done
[[ "$state_ready" -eq 1 ]] || fail "硬件状态节点未在 10 秒内就绪"

start_worker() {
  local name=$1
  local log_path=$2
  local command_port=$3
  shift 3
  (
    exec {INSTANCE_LOCK_FD}>&-
    exec {LEGACY_FEEDBACK_LOCK_FD}>&-
    exec "$@"
  ) >"$log_path" 2>&1 &
  local worker_pid=$!
  WORKER_PIDS+=("$worker_pid")
  WORKER_NAMES+=("$name")
  WORKER_LOGS+=("$log_path")
  WORKER_PORTS+=("$command_port")
  WORKER_BOUND+=(0)
  WORKER_PID_BY_DOMAIN["$name"]="$worker_pid"
}

print_worker_diagnostic() {
  local index=$1
  printf '%s 最近日志（%s）:\n' "${WORKER_NAMES[$index]}" "${WORKER_LOGS[$index]}" >&2
  tail -n 20 "${WORKER_LOGS[$index]}" >&2 2>/dev/null || true
}

worker_is_live() {
  managed_pid_is_live "$1"
}

stop_unbound_worker() {
  local index=$1
  local pid=${WORKER_PIDS[$index]}
  if worker_is_live "$pid"; then
    kill -TERM "$pid" 2>/dev/null || true
    local deadline=$((SECONDS + 3))
    while worker_is_live "$pid" && (( SECONDS < deadline )); do sleep 0.1; done
  fi
  if worker_is_live "$pid"; then
    kill -KILL "$pid" 2>/dev/null || true
    printf '严重警告：%s 启动超时且未响应 SIGTERM，已仅强制终止该故障域。\n' \
      "${WORKER_NAMES[$index]}" >&2
  fi
  wait "$pid" 2>/dev/null || true
}

wait_workers_bounded() {
  local deadline=$((SECONDS + 10))
  local index
  while (( SECONDS < deadline )); do
    local unresolved=0
    for index in "${!WORKER_PIDS[@]}"; do
      [[ "${WORKER_BOUND[$index]}" -eq 0 ]] || continue
      if udp_port_owned_by_pid "${WORKER_PORTS[$index]}" "${WORKER_PIDS[$index]}"; then
        WORKER_BOUND[$index]=1
        printf '%s 故障域控制器已绑定 UDP %s。\n' \
          "${WORKER_NAMES[$index]}" "${WORKER_PORTS[$index]}"
      elif ! worker_is_live "${WORKER_PIDS[$index]}"; then
        WORKER_BOUND[$index]=-1
        wait "${WORKER_PIDS[$index]}" 2>/dev/null || true
        printf '警告：%s 故障域在启动握手期间退出；GUI、MuJoCo 与其它故障域继续。\n' \
          "${WORKER_NAMES[$index]}" >&2
        print_worker_diagnostic "$index"
      else
        unresolved=1
      fi
    done
    [[ "$unresolved" -eq 0 ]] && break
    sleep 0.1
  done
  for index in "${!WORKER_PIDS[@]}"; do
    if [[ "${WORKER_BOUND[$index]}" -eq 0 ]]; then
      printf '警告：%s 故障域未在 10 秒内绑定 UDP %s；仅终止该故障域。\n' \
        "${WORKER_NAMES[$index]}" "${WORKER_PORTS[$index]}" >&2
      stop_unbound_worker "$index"
      WORKER_BOUND[$index]=-1
      print_worker_diagnostic "$index"
    fi
  done
}

start_worker_supervisor_status() {
  local domain index bound
  for domain in J1 J2 J345 J6; do
    [[ -n "${WORKER_PID_BY_DOMAIN[$domain]:-}" ]] || \
      fail "worker supervisor 缺少 $domain PID"
    bound=0
    for index in "${!WORKER_NAMES[@]}"; do
      if [[ "${WORKER_NAMES[$index]}" == "$domain" && \
            "${WORKER_BOUND[$index]}" -eq 1 ]]; then
        bound=1
        break
      fi
    done
    [[ "$bound" -eq 1 ]] || \
      fail "worker supervisor 拒绝发布未绑定 UDP 的 $domain"
  done
  [[ ! -e "$WORKER_SUPERVISOR_STATUS" && \
     ! -L "$WORKER_SUPERVISOR_STATUS" ]] || \
    fail "worker supervisor 状态路径在启动前已存在"
  (
    exec {INSTANCE_LOCK_FD}>&-
    exec {LEGACY_FEEDBACK_LOCK_FD}>&-
    exec python3 "$WORKER_SUPERVISOR_TOOL" \
      --output "$WORKER_SUPERVISOR_STATUS" \
      --supervisor-pid "$$" \
      --interval-ms 200 \
      --worker "J1=${WORKER_PID_BY_DOMAIN[J1]}" \
      --worker "J2=${WORKER_PID_BY_DOMAIN[J2]}" \
      --worker "J345=${WORKER_PID_BY_DOMAIN[J345]}" \
      --worker "J6=${WORKER_PID_BY_DOMAIN[J6]}"
  ) >"$WORKER_SUPERVISOR_LOG" 2>&1 &
  WORKER_SUPERVISOR_PID=$!

  local deadline=$((SECONDS + 3))
  while [[ ! -s "$WORKER_SUPERVISOR_STATUS" ]] && \
        (( SECONDS < deadline )); do
    if ! kill -0 "$WORKER_SUPERVISOR_PID" 2>/dev/null; then
      wait "$WORKER_SUPERVISOR_PID" 2>/dev/null || true
      tail -n 20 "$WORKER_SUPERVISOR_LOG" >&2 2>/dev/null || true
      fail "worker supervisor 心跳生产器启动失败"
    fi
    sleep 0.05
  done
  [[ -f "$WORKER_SUPERVISOR_STATUS" && \
     ! -L "$WORKER_SUPERVISOR_STATUS" ]] || \
    fail "worker supervisor 未在 3 秒内发布首帧"
  [[ "$(stat -c '%a' "$WORKER_SUPERVISOR_STATUS")" == "600" ]] || \
    fail "worker supervisor 状态文件不是 owner-only 0600"
}

# From this point onward cleanup owns the physical-worker shutdown path. Before
# this point any failure is preflight-only and must not transmit stop/BRAKE UDP.
HARDWARE_SESSION_STARTED=1
if [[ "${DOMAIN_READY[J1]}" -eq 1 ]]; then
  recovery_hint_args=()
  if [[ -f "$RECOVERY_BRANCH_HINTS" ]]; then
    recovery_hint_args=(--recovery-hint-file "$RECOVERY_BRANCH_HINTS")
  fi
  start_worker J1 "$RUN_DIR/j1_controller.log" 15310 "$GO_BINARY" --execute \
    --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus j1 --feedback-port 15300 \
    --zero-file "$PERSISTENT_ZERO" "${recovery_hint_args[@]}" \
    --go-aux-session-reference-file "$GO_AUX_SESSION_REFERENCE" \
    --go-aux-session-launch-permit-file "$GO_AUX_J1_LAUNCH_PERMIT" \
    --expected-zero-sha256 "$PERSISTENT_ZERO_SHA256" \
    --expected-go-aux-session-reference-sha256 \
      "$GO_AUX_SESSION_REFERENCE_SHA256" \
    --expected-go-aux-power-session-id "$GO_AUX_POWER_SESSION_ID" \
    --expected-worker-sha256 "$GO_AUX_EXPECTED_WORKER_SHA256" \
    --expected-gravity-authority-class "$EXPECTED_GRAVITY_AUTHORITY_CLASS" \
    --expected-empirical-envelope-id "$EXPECTED_EMPIRICAL_ENVELOPE_ID" \
    --expected-empirical-envelope-sha256 "$EXPECTED_EMPIRICAL_ENVELOPE_SHA256" \
    --expected-gravity-anchor-sha256 "$EXPECTED_GRAVITY_ANCHOR_SHA256" \
    --expected-gravity-session-id "$EXPECTED_GRAVITY_SESSION_ID" \
    --expected-gravity-state-instance-id "$EXPECTED_GRAVITY_STATE_INSTANCE_ID" \
    --thermal-config "$THERMAL_CONFIG" \
    --expected-thermal-config-sha256 "$THERMAL_CONFIG_SHA256"
fi
if [[ "${DOMAIN_READY[J2]}" -eq 1 ]]; then
  recovery_hint_args=()
  if [[ -f "$RECOVERY_BRANCH_HINTS" ]]; then
    recovery_hint_args=(--recovery-hint-file "$RECOVERY_BRANCH_HINTS")
  fi
  start_worker J2 "$RUN_DIR/j2_controller.log" 15312 "$GO_BINARY" --execute \
    --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus j2 --feedback-port 15300 \
    --zero-file "$PERSISTENT_ZERO" "${recovery_hint_args[@]}" \
    --j2-session-reference-file "$J2_SESSION_REFERENCE" \
    --j2-session-launch-permit-file "$J2_SESSION_LAUNCH_PERMIT" \
    --expected-zero-sha256 "$PERSISTENT_ZERO_SHA256" \
    --expected-j2-session-reference-sha256 "$J2_SESSION_REFERENCE_SHA256" \
    --expected-j2-power-session-id "$J2_POWER_SESSION_ID" \
    --expected-worker-sha256 "$J2_EXPECTED_WORKER_SHA256" \
    --expected-gravity-authority-class "$EXPECTED_GRAVITY_AUTHORITY_CLASS" \
    --expected-empirical-envelope-id "$EXPECTED_EMPIRICAL_ENVELOPE_ID" \
    --expected-empirical-envelope-sha256 "$EXPECTED_EMPIRICAL_ENVELOPE_SHA256" \
    --expected-gravity-anchor-sha256 "$EXPECTED_GRAVITY_ANCHOR_SHA256" \
    --expected-gravity-session-id "$EXPECTED_GRAVITY_SESSION_ID" \
    --expected-gravity-state-instance-id "$EXPECTED_GRAVITY_STATE_INSTANCE_ID" \
    --thermal-config "$THERMAL_CONFIG" \
    --expected-thermal-config-sha256 "$THERMAL_CONFIG_SHA256"
fi
if [[ "${DOMAIN_READY[J345]}" -eq 1 ]]; then
  recovery_hint_args=()
  if [[ -f "$RECOVERY_BRANCH_HINTS" ]]; then
    recovery_hint_args=(--recovery-hint-file "$RECOVERY_BRANCH_HINTS")
  fi
  start_worker J345 "$RUN_DIR/j345_controller.log" 15313 "$GO_BINARY" --execute \
    --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus j345 --feedback-port 15300 \
    --zero-file "$PERSISTENT_ZERO" "${recovery_hint_args[@]}" \
    --go-aux-session-reference-file "$GO_AUX_SESSION_REFERENCE" \
    --go-aux-session-launch-permit-file "$GO_AUX_J345_LAUNCH_PERMIT" \
    --expected-zero-sha256 "$PERSISTENT_ZERO_SHA256" \
    --expected-go-aux-session-reference-sha256 \
      "$GO_AUX_SESSION_REFERENCE_SHA256" \
    --expected-go-aux-power-session-id "$GO_AUX_POWER_SESSION_ID" \
    --expected-worker-sha256 "$GO_AUX_EXPECTED_WORKER_SHA256" \
    --expected-gravity-authority-class "$EXPECTED_GRAVITY_AUTHORITY_CLASS" \
    --expected-empirical-envelope-id "$EXPECTED_EMPIRICAL_ENVELOPE_ID" \
    --expected-empirical-envelope-sha256 "$EXPECTED_EMPIRICAL_ENVELOPE_SHA256" \
    --expected-gravity-anchor-sha256 "$EXPECTED_GRAVITY_ANCHOR_SHA256" \
    --expected-gravity-session-id "$EXPECTED_GRAVITY_SESSION_ID" \
    --expected-gravity-state-instance-id "$EXPECTED_GRAVITY_STATE_INSTANCE_ID" \
    --thermal-config "$THERMAL_CONFIG" \
    --expected-thermal-config-sha256 "$THERMAL_CONFIG_SHA256"
fi
if [[ "${DOMAIN_READY[J6]}" -eq 1 ]]; then
  J6_TORQUE_OBSERVATION_ARGS=()
  if [[ "${J6_OBSERVE_PROTOCOL_TORQUE:-0}" == 1 ]]; then
    [[ -f "${J6_PROTOCOL_TORQUE_READBACK_FILE:-}" && "${J6_PROTOCOL_TORQUE_READBACK_SHA256:-}" =~ ^[0-9a-f]{64}$ ]] || fail "J6 力矩观察需要明确的只读参数记录及 SHA"
    J6_TORQUE_OBSERVATION_ARGS=(--observe-protocol-torque --protocol-torque-readback "$J6_PROTOCOL_TORQUE_READBACK_FILE" --expected-protocol-torque-readback-sha256 "$J6_PROTOCOL_TORQUE_READBACK_SHA256")
  fi
  start_worker J6 "$RUN_DIR/j6_controller.log" 15311 env \
    LD_LIBRARY_PATH="$J6_LD_LIBRARY_PATH" \
    "$J6_PY" "$REPO_ROOT/tools/hardware/j6_dm_g6220/v15_30a_gui_j6_controller.py" \
    --execute --confirm V15_30A_GUI_J6_CONTROL_AUTHORIZED=YES \
    --command-port 15311 --feedback-port 15300 --zero-file "$PERSISTENT_ZERO" \
    --expected-gravity-authority-class "$EXPECTED_GRAVITY_AUTHORITY_CLASS" \
    --expected-empirical-envelope-id "$EXPECTED_EMPIRICAL_ENVELOPE_ID" \
    --expected-empirical-envelope-sha256 "$EXPECTED_EMPIRICAL_ENVELOPE_SHA256" \
    --expected-gravity-anchor-sha256 "$EXPECTED_GRAVITY_ANCHOR_SHA256" \
    --expected-gravity-session-id "$EXPECTED_GRAVITY_SESSION_ID" \
    --expected-gravity-state-instance-id "$EXPECTED_GRAVITY_STATE_INSTANCE_ID" \
    --feedback-session-id "$J6_FEEDBACK_SESSION_ID" \
    --feedback-state-instance-id "$J6_FEEDBACK_STATE_INSTANCE_ID" \
    --feedback-handoff-file "$J6_FEEDBACK_HANDOFF" \
    --thermal-config "$THERMAL_CONFIG" "${J6_TORQUE_OBSERVATION_ARGS[@]}"
fi

wait_workers_bounded
start_worker_supervisor_status
for index in "${!WORKER_PIDS[@]}"; do
  [[ "${WORKER_BOUND[$index]}" -eq 1 ]] || \
    fail "完整签名启动要求全部 worker 成功绑定；${WORKER_NAMES[$index]} 未就绪"
done

printf '正在检查七路反馈与签名参考哈希；必须连续 5 帧全部健康且保持 BRAKE…\n'
EXPECTED_PERSISTENT_ZERO_SHA256="$PERSISTENT_ZERO_SHA256" \
EXPECTED_J2_SESSION_REFERENCE_SHA256="$J2_SESSION_REFERENCE_SHA256" \
EXPECTED_GO_AUX_SESSION_REFERENCE_SHA256="$GO_AUX_SESSION_REFERENCE_SHA256" \
EXPECTED_WORKER_SUPERVISOR_PID="$$" \
"$GUI_PY" - <<'PY'
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

EXPECTED = {"J1", "J2A", "J2B", "J3", "J4", "J5", "J6"}
EXPECTED_CONTROL_DOMAINS = {"J1", "J2", "J345", "J6"}
EXPECTED_WORKER_SUPERVISOR_PID = int(
    os.environ["EXPECTED_WORKER_SUPERVISOR_PID"]
)
EXPECTED_REFERENCE_HASHES = {
    "persistent_zero_sha256": os.environ["EXPECTED_PERSISTENT_ZERO_SHA256"],
    "j2_session_reference_sha256": os.environ[
        "EXPECTED_J2_SESSION_REFERENCE_SHA256"
    ],
    "go_aux_session_reference_sha256": os.environ[
        "EXPECTED_GO_AUX_SESSION_REFERENCE_SHA256"
    ],
}
deadline = time.monotonic() + 20.0
consecutive = 0
last_reason = "尚未收到 /whole_arm/hardware_state"
last_value = None

rclpy.init()
node = Node("arm_gui_startup_readiness_probe")


def on_state(message: String) -> None:
    global consecutive, last_reason, last_value
    try:
        value = json.loads(message.data)
        if not isinstance(value, dict):
            raise ValueError("状态报文不是对象")
        last_value = value
        motors = value.get("per_motor", {})
        modes = value.get("controller_mode_by_motor", {})
        faults = value.get("controller_fault_by_motor", {})
        control_available = value.get("control_available_by_domain", {})
        control_reasons = value.get("control_reason_by_domain", {})
        supervisor_instance = value.get("worker_supervisor_instance_id")
        checks = [
            (value.get("reference") in {
                "SESSION_REFERENCE_V1", "PERSISTENT_SOFTWARE_ZERO_V1"
             }, "软件参考版本不匹配"),
            (all(value.get(name) == expected for name, expected in
                 EXPECTED_REFERENCE_HASHES.items()),
             "状态节点加载的 zero/J2/GO-AUX 参考哈希与签名启动输入不一致"),
            (set(value.get("available_motors", [])) == EXPECTED, "尚未捕获全部七路参考"),
            (set(motors) == EXPECTED, "反馈电机集合不完整"),
            (bool(value.get("healthy")), "全臂健康状态未通过"),
            (not bool(value.get("j2_sync_fault")), "J2 同步故障已锁定"),
            (int(value.get("invalid_payload_count", 0)) == 0, "存在非法反馈报文"),
            (all(bool(motors[name].get("reference_captured")) for name in EXPECTED),
             "七路软件参考未全部建立"),
            (all(bool(motors[name].get("communication_ok")) and
                 bool(motors[name].get("fresh")) and
                 int(motors[name].get("merror", -1)) == 0 for name in EXPECTED),
             "反馈不新鲜、通信异常或电机报错"),
            (all(modes.get(name) == "brake" for name in EXPECTED),
             "启动握手期间未全部保持 BRAKE"),
            (all(faults.get(name) is False for name in EXPECTED),
             "控制器故障域已锁定"),
            (isinstance(control_available, dict) and
             set(control_available) == EXPECTED_CONTROL_DOMAINS and
             all(control_available.get(domain) is True
                 for domain in EXPECTED_CONTROL_DOMAINS),
             "worker supervisor 未确认全部四域控制接收器"),
            (isinstance(control_reasons, dict) and
             set(control_reasons) == EXPECTED_CONTROL_DOMAINS and
             all(control_reasons.get(domain) == "ready"
                 for domain in EXPECTED_CONTROL_DOMAINS),
             "worker supervisor 四域原因不是 ready"),
            (isinstance(supervisor_instance, str) and
             bool(supervisor_instance) and
             value.get("worker_supervisor_pid") ==
             EXPECTED_WORKER_SUPERVISOR_PID,
             "worker supervisor identity/PID 与启动器不一致"),
        ]
        failed = next((reason for passed, reason in checks if not passed), None)
        if failed is None:
            consecutive += 1
            last_reason = "健康报文连续样本不足"
        else:
            consecutive = 0
            last_reason = failed
    except Exception as exc:
        consecutive = 0
        last_reason = f"状态报文解析失败: {exc}"


node.create_subscription(String, "/whole_arm/hardware_state", on_state, 10)
try:
    while rclpy.ok() and time.monotonic() < deadline and consecutive < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if consecutive >= 5:
    print("启动状态：全部七路反馈、软件参考及 BRAKE 连续健康确认通过。")
elif last_value is None:
    print(
        "启动失败：启动窗口内没有可用关节状态。",
        file=sys.stderr,
    )
    raise SystemExit(2)
else:
    motors = last_value.get("per_motor", {})
    available = set(last_value.get("available_motors", []))
    modes = last_value.get("controller_mode_by_motor", {})
    faults = last_value.get("controller_fault_by_motor", {})
    missing = sorted(EXPECTED - available)
    unhealthy = sorted(
        name for name in EXPECTED
        if name in available and (
            not bool(motors.get(name, {}).get("communication_ok"))
            or not bool(motors.get(name, {}).get("fresh"))
            or int(motors.get(name, {}).get("merror", -1)) != 0
            or bool(faults.get(name, False))
            or modes.get(name) != "brake"
        )
    )
    print(
        "启动失败：不允许部分连接；"
        f"缺失电机={','.join(missing) if missing else '无'}；"
        f"故障/非BRAKE电机={','.join(unhealthy) if unhealthy else '无'}；"
        f"原因={last_reason}。",
        file=sys.stderr,
    )
    raise SystemExit(2)
PY

for index in "${!WORKER_PIDS[@]}"; do
  if [[ "${WORKER_BOUND[$index]}" -eq 1 ]] && \
      { ! worker_is_live "${WORKER_PIDS[$index]}" || \
        ! udp_port_owned_by_pid "${WORKER_PORTS[$index]}" "${WORKER_PIDS[$index]}"; }; then
    WORKER_BOUND[$index]=-1
    wait "${WORKER_PIDS[$index]}" 2>/dev/null || true
    print_worker_diagnostic "$index"
    fail "完整签名启动复查期间 ${WORKER_NAMES[$index]} worker 已退出或丢失 UDP 所有权"
  fi
done
if ! worker_is_live "$WORKER_SUPERVISOR_PID"; then
  wait "$WORKER_SUPERVISOR_PID" 2>/dev/null || true
  tail -n 20 "$WORKER_SUPERVISOR_LOG" >&2 2>/dev/null || true
  fail "完整签名启动复查期间 worker supervisor 心跳生产器已退出"
fi

printf '启动完成：默认为停止／制动，无任何自动运动。\n'
printf '本次运行日志：%s\n' "$RUN_DIR"

reported=()
for index in "${!WORKER_PIDS[@]}"; do
  if [[ "${WORKER_BOUND[$index]}" -eq 1 ]]; then reported+=(0); else reported+=(1); fi
done
MONITORED_CORE_PID="$ROS_PID"
if [[ "$REUSE_RUNNING_ARM_GUI_CORE" == "true" ]]; then
  MONITORED_CORE_PID="$EXTERNAL_ARM_GUI_CORE_PID"
fi
while kill -0 "$MONITORED_CORE_PID" 2>/dev/null; do
  if ! worker_is_live "$WORKER_SUPERVISOR_PID"; then
    wait "$WORKER_SUPERVISOR_PID" 2>/dev/null || true
    tail -n 20 "$WORKER_SUPERVISOR_LOG" >&2 2>/dev/null || true
    fail "worker supervisor 心跳生产器已退出，立即进入制动清理"
  fi
  for index in "${!WORKER_PIDS[@]}"; do
    if [[ "${reported[$index]}" -eq 0 ]] && \
        { ! worker_is_live "${WORKER_PIDS[$index]}" || \
          ! udp_port_owned_by_pid "${WORKER_PORTS[$index]}" "${WORKER_PIDS[$index]}"; }; then
      reported[$index]=1
      printf '警告：%s 故障域已退出；其他故障域与 GUI 继续运行，详见 %s\n' \
        "${WORKER_NAMES[$index]}" "${WORKER_LOGS[$index]}" >&2
    fi
  done
  sleep 0.5
done
if [[ "$REUSE_RUNNING_ARM_GUI_CORE" == "true" ]]; then
  fail "复用的 arm GUI 核心进程已退出"
fi
wait "$ROS_PID"
