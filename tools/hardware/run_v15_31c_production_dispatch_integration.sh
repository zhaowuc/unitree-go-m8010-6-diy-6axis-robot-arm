#!/usr/bin/env bash
# Linux/ROS test only: no production node, UDP socket, or motor transport starts.
set -eo pipefail
repo=$(cd -- "$(dirname -- "$0")/../.." && pwd)
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
sdk=${UNITREE_ACTUATOR_SDK_ROOT:-/home/car/vendor/unitree_actuator_sdk}
py=${M8010_TEST_PYTHON:-"$repo/.venv/arm-gui/bin/python"}
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
export PYTHONPATH="$ws/src/go_m8010_arm_gui:$ws/src/go_m8010_arm_hardware:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
export QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl
build_dir=$(mktemp -d -t v15-31c-dispatch-audit.XXXXXXXX)
export M8010_PACKET_AUDIT_BINARY="$build_dir/dispatch_packet"
g++ -std=c++17 -O2 -ffunction-sections -fdata-sections \
  -I"$sdk/include" "$repo/tools/hardware/test_v15_31c_dispatch_packet.cpp" \
  -L"$sdk/lib" -Wl,-rpath,"$sdk/lib" -Wl,--gc-sections \
  -lUnitreeMotorSDK_Linux64 -lpthread -o "$M8010_PACKET_AUDIT_BINARY"
"$py" -m pytest -q --junitxml="$build_dir/results.xml" \
  "$repo/tools/hardware/test_v15_31c_production_dispatch_integration.py"
printf 'OFFLINE_DISPATCH_AUDIT_BINARY=%s\n' "$M8010_PACKET_AUDIT_BINARY"
