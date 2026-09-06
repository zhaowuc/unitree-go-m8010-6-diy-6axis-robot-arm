#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
model="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
gravity_config="$ws/src/go_m8010_arm_hardware/config/gravity_control.yaml"
thermal_config="$ws/src/go_m8010_arm_hardware/config/thermal_limits.yaml"
envelope_sha=$(sha256sum "$session/empirical_validation_envelope.json" | awk '{print $1}')
set +u
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONNOUSERSITE=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
exec ros2 run go_m8010_arm_hardware whole_arm_gravity_node --ros-args \
  -p model_path:="$model" \
  -p gravity_config_path:="$gravity_config" \
  -p thermal_config_path:="$thermal_config" \
  -p anchor_path:="$session/model_session_anchor.json" \
  -p empirical_envelope_path:="$session/empirical_validation_envelope.json" \
  -p expected_empirical_envelope_sha256:="$envelope_sha" \
  -p empirical_claim_directory:="$session/empirical_claims" \
  -p calculation_rate_hz:=100.0 \
  -p joint_state_maximum_age_ms:=250.0 \
  -p enabled_for_hardware:=true \
  -p gravity_scale_target:=0.0 \
  -p gravity_scale_ramp_seconds:=2.0 \
  -p maximum_rotor_torque_slew_nm_per_s:=1.0
