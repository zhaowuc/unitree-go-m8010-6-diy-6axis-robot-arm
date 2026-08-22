#!/usr/bin/env bash
set -Eeuo pipefail
IFS=$'\n\t'

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
RUN_TOKEN="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$REPO_ROOT/logs/arm_gui/$RUN_TOKEN"
STATE_DIR="$REPO_ROOT/.runtime/v15_30a_gui"
INITIAL_POSE="$STATE_DIR/initial_pose.json"
STOP_TOOL="$REPO_ROOT/tools/hardware/v15_30a_gui_stop.py"

export PYTHONNOUSERSITE=1

ROS_PID=""
WORKER_PIDS=()
WORKER_NAMES=()
WORKER_LOGS=()
WORKER_PORTS=()
WORKER_BOUND=()
CLEANED=0

fail() {
  printf '启动已阻止：%s\n' "$*" >&2
  exit 2
}

cleanup() {
  local original_status=$?
  if [[ "$CLEANED" -ne 0 ]]; then
    return
  fi
  CLEANED=1
  trap - EXIT INT TERM HUP
  set +e
  printf '\n正在停止轨迹并执行制动／失能清理…\n'
  python3 "$STOP_TOOL" --repeat 5 --interval 0.01 >/dev/null 2>&1
  if [[ -n "$ROS_PID" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    kill -INT "$ROS_PID" 2>/dev/null
  fi
  python3 "$STOP_TOOL" --repeat 50 --interval 0.02 >/dev/null 2>&1
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

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

command -v flock >/dev/null || fail "缺少 flock（util-linux）"
command -v fuser >/dev/null || fail "缺少 fuser（psmisc）"

mkdir -p "$RUN_DIR" "$STATE_DIR" "$TOOLS_BUILD"
exec {INSTANCE_LOCK_FD}>/tmp/v15_30a_gui_supervisor.lock
flock -n "$INSTANCE_LOCK_FD" || fail "已有一个机械臂 GUI 会话在运行"
exec {LEGACY_FEEDBACK_LOCK_FD}>/tmp/v15_30a_whole_arm_go_feedback.lock
flock -n "$LEGACY_FEEDBACK_LOCK_FD" || fail "旧的全臂反馈进程仍在占用 GO 物理总线"

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
command -v ss >/dev/null || fail "缺少 ss（iproute2），无法可靠检查 UDP 端口"
xdpyinfo >/dev/null 2>&1 || fail "无法访问当前 Ubuntu 图形会话"
if dpkg-query -W -f='${Status}' libxcb-cursor0 2>/dev/null | grep -q 'install ok installed'; then
  export QT_QPA_PLATFORM=xcb
elif [[ -S "$XDG_RUNTIME_DIR/wayland-0" ]]; then
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export QT_QPA_PLATFORM=wayland
else
  fail "既无可用 Wayland 会话，也缺少 libxcb-cursor0"
fi

[[ -f "$MODEL" ]] || fail "冻结 MuJoCo 生产模型不存在"
actual_model_sha="$(sha256sum "$MODEL" | awk '{print $1}')"
[[ "$actual_model_sha" == "$MODEL_SHA256" ]] || fail "冻结 MuJoCo 生产模型哈希不匹配"

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

if ! command -v g++ >/dev/null; then
  for domain in J1 J2 J345; do
    mark_domain_unavailable "$domain" "缺少 g++ 编译器"
  done
elif [[ ! -f "$SDK_ROOT/lib/libUnitreeMotorSDK_Linux64.so" ]]; then
  for domain in J1 J2 J345; do
    mark_domain_unavailable "$domain" "Unitree 电机 SDK 不完整"
  done
fi

J6_ENV_LIB=""
J6_LD_LIBRARY_PATH=""
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

if udp_port_in_use 15300; then
  fail "ROS2 硬件状态核心 UDP 端口 15300 已被占用"
fi
declare -A DOMAIN_PORT=([J1]=15310 [J2]=15312 [J345]=15313 [J6]=15311)
for domain in J1 J2 J345 J6; do
  if udp_port_in_use "${DOMAIN_PORT[$domain]}"; then
    mark_domain_unavailable "$domain" "UDP 命令端口 ${DOMAIN_PORT[$domain]} 已被占用"
  fi
done
declare -A GO_DEVICE=(
  [J1]='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0'
  [J2]='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0'
  [J345]='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0'
)
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

if [[ ! -x "$GUI_PY" ]]; then
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

if ! check_gui_dependencies; then
  printf '正在安装锁定的 GUI 依赖（首次启动需联网）…\n'
  "$GUI_PY" -m pip install --disable-pip-version-check -r "$REPO_ROOT/requirements-arm-gui.txt"
fi
check_gui_dependencies || fail "GUI Python 锁定版本或 pip 依赖完整性校验失败"

set +u
source /opt/ros/humble/setup.bash
set -u
printf '正在构建两个最小 ROS2 包…\n'
(cd "$ROS_WS" && "$GUI_PY" -m colcon build --symlink-install \
  --packages-select go_m8010_arm_hardware go_m8010_arm_gui \
  --event-handlers console_direct+)
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

if [[ "${DOMAIN_READY[J1]}" -eq 1 || "${DOMAIN_READY[J2]}" -eq 1 || \
      "${DOMAIN_READY[J345]}" -eq 1 ]]; then
  printf '正在严格编译 GO 四轴物理总线控制器…\n'
  if ! g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
    -I"$SDK_ROOT/include" \
    "$REPO_ROOT/tools/hardware/v15_30a_gui_go_controller.cpp" \
    "$SDK_ROOT/lib/libUnitreeMotorSDK_Linux64.so" \
    "-Wl,-rpath,$SDK_ROOT/lib" \
    -o "$GO_BINARY"; then
    for domain in J1 J2 J345; do
      if [[ "${DOMAIN_READY[$domain]}" -eq 1 ]]; then
        mark_domain_unavailable "$domain" "GO 控制器严格编译失败"
      fi
    done
  fi
else
  printf '提示：没有可启动的 GO 故障域，跳过 GO 控制器编译。\n' >&2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export MUJOCO_GL=glfw
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1

printf '正在启动全中文界面与 MuJoCo 可视镜像…\n'
ros2 launch go_m8010_arm_gui arm_gui.launch.py \
  model_path:="$MODEL" \
  config_path:="$GUI_SOURCE/config/arm_gui.yaml" \
  joint_limits_path:="$GUI_SOURCE/config/gui_joint_limits.yaml" \
  initial_pose_path:="$INITIAL_POSE" \
  runtime_log_directory:="$RUN_DIR" &
ROS_PID=$!

state_ready=0
for _ in $(seq 1 100); do
  if ! kill -0 "$ROS_PID" 2>/dev/null; then fail "ROS2 启动进程提前退出"; fi
  if udp_port_in_use 15300; then state_ready=1; break; fi
  sleep 0.1
done
[[ "$state_ready" -eq 1 ]] || fail "硬件状态节点未在 10 秒内就绪"

start_worker() {
  local name=$1
  local log_path=$2
  local command_port=$3
  shift 3
  "$@" >"$log_path" 2>&1 &
  WORKER_PIDS+=("$!")
  WORKER_NAMES+=("$name")
  WORKER_LOGS+=("$log_path")
  WORKER_PORTS+=("$command_port")
  WORKER_BOUND+=(0)
}

print_worker_diagnostic() {
  local index=$1
  printf '%s 最近日志（%s）:\n' "${WORKER_NAMES[$index]}" "${WORKER_LOGS[$index]}" >&2
  tail -n 20 "${WORKER_LOGS[$index]}" >&2 2>/dev/null || true
}

worker_is_live() {
  local pid=$1
  local state
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | awk 'NR==1 {print $1}')"
  [[ -n "$state" && "$state" != Z* ]]
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

if [[ "${DOMAIN_READY[J1]}" -eq 1 ]]; then
  start_worker J1 "$RUN_DIR/j1_controller.log" 15310 "$GO_BINARY" --execute \
    --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus j1 --feedback-port 15300
fi
if [[ "${DOMAIN_READY[J2]}" -eq 1 ]]; then
  start_worker J2 "$RUN_DIR/j2_controller.log" 15312 "$GO_BINARY" --execute \
    --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus j2 --feedback-port 15300
fi
if [[ "${DOMAIN_READY[J345]}" -eq 1 ]]; then
  start_worker J345 "$RUN_DIR/j345_controller.log" 15313 "$GO_BINARY" --execute \
    --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus j345 --feedback-port 15300
fi
if [[ "${DOMAIN_READY[J6]}" -eq 1 ]]; then
  start_worker J6 "$RUN_DIR/j6_controller.log" 15311 env \
    LD_LIBRARY_PATH="$J6_LD_LIBRARY_PATH" \
    "$J6_PY" "$REPO_ROOT/tools/hardware/j6_dm_g6220/v15_30a_gui_j6_controller.py" \
    --execute --confirm V15_30A_GUI_J6_CONTROL_AUTHORIZED=YES \
    --command-port 15311 --feedback-port 15300
fi

wait_workers_bounded

printf '正在检查七路反馈；全正常时确认连续健康，部分连接时保持 GUI／MuJoCo 运行…\n'
"$GUI_PY" - <<'PY'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

EXPECTED = {"J1", "J2A", "J2B", "J3", "J4", "J5", "J6"}
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
        checks = [
            (value.get("reference") == "SESSION_REFERENCE_V1", "会话参考版本不匹配"),
            (set(value.get("available_motors", [])) == EXPECTED, "尚未捕获全部七路参考"),
            (set(motors) == EXPECTED, "反馈电机集合不完整"),
            (bool(value.get("healthy")), "全臂健康状态未通过"),
            (not bool(value.get("j2_sync_fault")), "J2 同步故障已锁定"),
            (int(value.get("invalid_payload_count", 0)) == 0, "存在非法反馈报文"),
            (all(bool(motors[name].get("reference_captured")) for name in EXPECTED),
             "七路会话参考未全部捕获"),
            (all(bool(motors[name].get("communication_ok")) and
                 bool(motors[name].get("fresh")) and
                 int(motors[name].get("merror", -1)) == 0 for name in EXPECTED),
             "反馈不新鲜、通信异常或电机报错"),
            (all(modes.get(name) == "brake" for name in EXPECTED),
             "启动握手期间未全部保持 BRAKE"),
            (all(faults.get(name) is False for name in EXPECTED),
             "控制器故障域已锁定"),
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
    print("启动状态：全部七路反馈、会话参考及 BRAKE 连续健康确认通过。")
elif last_value is None:
    print(
        "警告：启动窗口内没有可用关节状态；GUI 与 MuJoCo 已保留运行，等待故障域恢复。",
        file=sys.stderr,
    )
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
        "警告：启动状态为部分连接；"
        f"缺失电机={','.join(missing) if missing else '无'}；"
        f"故障/非BRAKE电机={','.join(unhealthy) if unhealthy else '无'}；"
        f"原因={last_reason}。GUI、MuJoCo 与健康故障域继续运行。",
        file=sys.stderr,
    )
PY

for index in "${!WORKER_PIDS[@]}"; do
  if [[ "${WORKER_BOUND[$index]}" -eq 1 ]] && \
      { ! worker_is_live "${WORKER_PIDS[$index]}" || \
        ! udp_port_owned_by_pid "${WORKER_PORTS[$index]}" "${WORKER_PIDS[$index]}"; }; then
    WORKER_BOUND[$index]=-1
    wait "${WORKER_PIDS[$index]}" 2>/dev/null || true
    printf '警告：%s 故障域在状态检查期间退出；其它故障域与 GUI 继续。\n' \
      "${WORKER_NAMES[$index]}" >&2
    print_worker_diagnostic "$index"
  fi
done

printf '启动完成：默认为停止／制动，无任何自动运动。\n'
printf '本次运行日志：%s\n' "$RUN_DIR"

reported=()
for index in "${!WORKER_PIDS[@]}"; do
  if [[ "${WORKER_BOUND[$index]}" -eq 1 ]]; then reported+=(0); else reported+=(1); fi
done
while kill -0 "$ROS_PID" 2>/dev/null; do
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
wait "$ROS_PID"
