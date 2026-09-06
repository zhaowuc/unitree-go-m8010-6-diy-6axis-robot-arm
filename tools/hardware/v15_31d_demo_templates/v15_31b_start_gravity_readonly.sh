#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@

repo=@@REPO@@
session=@@SESSION@@
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
model="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
gravity_config="$ws/src/go_m8010_arm_hardware/config/gravity_control.yaml"
thermal_config="$ws/src/go_m8010_arm_hardware/config/thermal_limits.yaml"

set +u
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export PYTHONNOUSERSITE=1

exec ros2 run go_m8010_arm_hardware whole_arm_gravity_node --ros-args \
  -p model_path:="$model" \
  -p gravity_config_path:="$gravity_config" \
  -p thermal_config_path:="$thermal_config" \
  -p anchor_path:="$session/model_session_anchor.json" \
  -p calculation_rate_hz:=100.0 \
  -p joint_state_maximum_age_ms:=250.0 \
  -p enabled_for_hardware:=false \
  -p gravity_scale_target:=0.0
