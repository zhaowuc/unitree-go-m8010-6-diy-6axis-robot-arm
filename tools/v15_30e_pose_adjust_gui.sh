#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT=/home/car/go-m8010-robot-arm-v15-30a-gui
ROS_WS="$REPO_ROOT/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
MODEL="$REPO_ROOT/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
GUI_SOURCE="$ROS_WS/src/go_m8010_arm_gui"
POSE_LIMITS="$GUI_SOURCE/config/gui_joint_limits_pose_adjust.yaml"
RUNTIME=/tmp/v15_30e_pose_adjust
MODEL_SHA=5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9

mkdir -p "$RUNTIME"
if [[ "$(sha256sum "$MODEL" | awk '{print $1}')" != "$MODEL_SHA" ]]; then
  echo 'MODEL_HASH_MISMATCH' >&2
  exit 2
fi

export XDG_RUNTIME_DIR=/run/user/$(id -u)
export DISPLAY=${DISPLAY:-:0}
if [[ -z "${XAUTHORITY:-}" ]]; then
  for candidate in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
    if [[ -f "$candidate" ]]; then
      export XAUTHORITY="$candidate"
      break
    fi
  done
fi
export DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export QT_QPA_PLATFORM=wayland
export MUJOCO_GL=egl
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

PIDS=()
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM HUP

ros2 run go_m8010_arm_hardware whole_arm_mujoco_mirror --ros-args \
  -p model_path:="$MODEL" \
  -p session_pose_deg:="0,170,-170,-12.41,46.7,0.55" \
  -p pose_matched:=false \
  -p session_relative_baseline:=true \
  -p numeric_test_only:=false \
  -p use_viewer:=false \
  -p evidence_directory:="$RUNTIME" \
  >"$RUNTIME/mujoco.log" 2>&1 &
PIDS+=("$!")

ros2 topic pub -r 20 /joint_states sensor_msgs/msg/JointState \
  "{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
  >"$RUNTIME/dummy_joint_states.log" 2>&1 &
PIDS+=("$!")

ros2 run go_m8010_arm_gui arm_gui --ros-args \
  -p config_path:="$GUI_SOURCE/config/arm_gui.yaml" \
  -p joint_limits_path:="$POSE_LIMITS" \
  -p initial_pose_path:="$RUNTIME/initial_pose.json" \
  -p log_directory:="$RUNTIME" \
  -p embedded_model_path:="$MODEL" \
  -p embedded_session_pose_deg:="0,170,-170,-12.41,46.7,0.55" \
  >"$RUNTIME/gui.log" 2>&1 &
GUI_PID=$!
PIDS+=("$GUI_PID")

echo "POSE_ADJUST_GUI_PID=$GUI_PID"
echo "HARDWARE_SERIAL_OPENED=NO"
echo "SESSION_ANCHOR_DEG=0,170,-170,-12.41,46.7,0.55"
echo "ABSOLUTE_LIMITS_DEG=J1:-180..180,J2:-170..170,J3:-170..170,J4:-116..159,J5:-70.6..151.2,J6:-180..180"
wait "$GUI_PID"
