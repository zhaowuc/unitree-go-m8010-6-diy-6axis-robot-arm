#!/usr/bin/env bash
set -eo pipefail
recovery_args=()
if [[ "${2:-false}" == true ]]; then
  recovery_args+=(--recover-initial-first)
fi
motion_args=(--excursion-deg "${3:-1}" --speed-deg-s "${4:-1}")
if [[ "${5:-false}" == true ]]; then motion_args+=(--symmetric); fi
if [[ "${6:-false}" == true ]]; then motion_args+=(--return-center); fi
if [[ "${GO_ASSISTED_TEACH:-0}" == 1 ]]; then motion_args+=(--interactive-teach); fi
if [[ "${GO_TEACH_OBSERVE:-0}" == 1 ]]; then motion_args+=(--teach-observe); fi
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export XDG_RUNTIME_DIR=/run/user/$(id -u) DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
export WAYLAND_DISPLAY=wayland-0 QT_QPA_PLATFORM=wayland MUJOCO_GL=glfw
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
python3 - "$repo" "$session" <<'PY'
import pathlib,sys,yaml
r,s=map(pathlib.Path,sys.argv[1:])
data=yaml.safe_load((r/'.runtime/v15_31b_ft/current/attended_gui_params.yaml').read_text())
params=data['/**']['ros__parameters']
previous_root=str(pathlib.Path(params['initial_pose_path']).parents[2])
for key,value in list(params.items()):
    if isinstance(value,str) and value.startswith(previous_root+'/'):
        params[key]=str(r)+value[len(previous_root):]
params['log_directory']=str(s/'run')
with (s/'j1_demo_gui_params.yaml').open('x',encoding='utf-8') as f:
    yaml.safe_dump(data,f,allow_unicode=True)
PY
sha=$(sha256sum "$session/empirical_validation_envelope.json" | cut -d ' ' -f1)
exec "$repo/.venv/arm-gui/bin/python" "$repo/tools/hardware/v15_31d_gui_j1_demo.py" \
 "${recovery_args[@]}" \
 "${motion_args[@]}" \
 --execute --cycles "${1:-1}" --output "$session/evidence/j1_action_group_demo.json" \
 --envelope "$session/empirical_validation_envelope.json" \
 --anchor-validation "$session/evidence/model_session_anchor_validation.json" \
 --expected-envelope-sha256 "$sha" \
 --ros-args -r __node:=arm_control_gui_j1_demo --params-file "$session/j1_demo_gui_params.yaml"
