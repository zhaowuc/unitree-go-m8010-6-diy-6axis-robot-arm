#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@

repo=@@REPO@@
session=@@SESSION@@
state="$repo/.runtime/v15_30a_gui"
run="$session/run"
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
model="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"

shopt -s nullglob
j2_refs=("$session/j2_anchors"/*.json)
go_refs=("$session/go_anchors"/*.json)
[[ ${#j2_refs[@]} -eq 1 && ${#go_refs[@]} -eq 1 ]]
mkdir -p -m 700 "$run"
if [[ ! -e "$session/state_instance_id" ]]; then
  umask 077
  python3 -c 'import secrets; print(secrets.token_hex(16))' \
    >"$session/state_instance_id"
fi
state_id="$(cat "$session/state_instance_id")"
[[ "$state_id" =~ ^[0-9a-f]{32}$ ]]

zero_sha="$(sha256sum "$state/persistent_software_zero.json" | awk '{print $1}')"
j2_sha="$(sha256sum "${j2_refs[0]}" | awk '{print $1}')"
go_sha="$(sha256sum "${go_refs[0]}" | awk '{print $1}')"
session_id="persistent:${zero_sha:0:16}:j2session:${j2_sha:0:16}:goauxsession:${go_sha:0:16}"
printf '%s\n' "$session_id" >"$session/session_id"
chmod 600 "$session/session_id" "$session/state_instance_id"

export PYTHONNOUSERSITE=1
set +u
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export MUJOCO_GL=glfw
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then export DISPLAY=:0; fi
if [[ -z "${XAUTHORITY:-}" ]]; then
  for candidate in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
    if [[ -f "$candidate" ]]; then export XAUTHORITY="$candidate"; break; fi
  done
fi
if [[ -S "$XDG_RUNTIME_DIR/bus" ]]; then
  export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"
fi
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export QT_QPA_PLATFORM=wayland

exec ros2 launch go_m8010_arm_gui arm_gui.launch.py \
  start_gui:=false \
  model_path:="$model" \
  session_pose_deg:="0,90,-14.40,13.49,47.94,0" \
  pose_matched:=true \
  config_path:="$ws/src/go_m8010_arm_gui/config/arm_gui.yaml" \
  joint_limits_path:="$ws/src/go_m8010_arm_gui/config/gui_joint_limits.yaml" \
  initial_pose_path:="$state/initial_pose.json" \
  initial_pose_read_only:=true \
  persistent_zero_path:="$state/persistent_software_zero.json" \
  recovery_hint_path:="$state/recovery_branch_hints.json" \
  j2_session_reference_path:="${j2_refs[0]}" \
  go_aux_session_reference_path:="${go_refs[0]}" \
  state_instance_id:="$state_id" \
  gravity_enabled_for_hardware:=false \
  gravity_scale_target:=0.0 \
  start_gravity_node:=false \
  runtime_log_directory:="$run"
