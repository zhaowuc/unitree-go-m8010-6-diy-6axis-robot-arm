#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
bus=${1:?bus required}
repo=@@REPO@@
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
exec "$ws/build/v15_30a_gui_tools/v15_30a_gui_go_controller" --execute --brake-only \
 --confirm V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES --bus "$bus" --feedback-port 15300 \
 --thermal-config "$ws/src/go_m8010_arm_hardware/config/thermal_limits.yaml" \
 --expected-thermal-config-sha256 1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467
