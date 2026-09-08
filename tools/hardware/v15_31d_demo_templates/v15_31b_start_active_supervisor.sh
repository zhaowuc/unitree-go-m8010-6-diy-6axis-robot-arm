#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
py="$repo/.venv/arm-gui/bin/python"
shopt -s nullglob
j2_refs=("$session/j2_anchors"/*.json)
go_refs=("$session/go_anchors"/*.json)
[[ ${#j2_refs[@]} -eq 1 && ${#go_refs[@]} -eq 1 ]]
readarray -t permit_values < <("$py" - "$session/j2_permit_publish.json" "$session/go_permit_publish.json" <<'PY'
import json, pathlib, sys
j2=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
go=json.loads(pathlib.Path(sys.argv[2]).read_text(encoding="utf-8"))
assert j2.get("applied") is True and j2.get("launch_permit_published") is True
assert go.get("applied") is True and go.get("launch_permits_published") is True
print(j2["permit_path"])
print(go["permit_paths"]["j1"])
print(go["permit_paths"]["j345"])
PY
)
j2_permit=${permit_values[0]}
j1_permit=${permit_values[1]}
j345_permit=${permit_values[2]}
j2_power=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_session_id"])' "$session/j2_brake_raw.json")
go_power=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_session_id"])' "$session/go_aux_brake_raw.json")
envelope_sha=$(sha256sum "$session/empirical_validation_envelope.json" | awk '{print $1}')
core_pid=$(cat "$session/core_pid")
export ARM_GUI_RUN_DIRECTORY="$session/run"
export J2_SESSION_REFERENCE_FILE="${j2_refs[0]}"
export J2_SESSION_LAUNCH_PERMIT_FILE="$j2_permit"
export J2_POWER_SESSION_ID="$j2_power"
export GO_AUX_SESSION_REFERENCE_FILE="${go_refs[0]}"
export GO_AUX_J1_LAUNCH_PERMIT_FILE="$j1_permit"
export GO_AUX_J345_LAUNCH_PERMIT_FILE="$j345_permit"
export GO_AUX_POWER_SESSION_ID="$go_power"
export J6_FEEDBACK_HANDOFF_FILE="$session/j6_feedback_handoff_active.json"
export REUSE_RUNNING_ARM_GUI_CORE=true
export EXTERNAL_ARM_GUI_CORE_PID="$core_pid"
export GRAVITY_ANCHOR_PATH="$session/model_session_anchor.json"
export EMPIRICAL_VALIDATION_ENVELOPE_PATH="$session/empirical_validation_envelope.json"
export EMPIRICAL_VALIDATION_ENVELOPE_SHA256="$envelope_sha"
export GRAVITY_ENABLED_FOR_HARDWARE=true
export GRAVITY_SCALE_TARGET=0.0
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export PYTHONNOUSERSITE=1 XDG_RUNTIME_DIR=/run/user/$(id -u)
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 QT_QPA_PLATFORM=wayland
if [[ -f "$scripts/hand_guidance_settings.json" ]]; then
  readarray -t torque_values < <("$py" - "$scripts/hand_guidance_settings.json" <<'PY'
import json,sys
v=json.load(open(sys.argv[1]));print(v['j6_torque_readback']);print(v['j6_torque_readback_sha256'])
PY
  )
  export J6_OBSERVE_PROTOCOL_TORQUE=1
  export J6_PROTOCOL_TORQUE_READBACK_FILE="${torque_values[0]}"
  export J6_PROTOCOL_TORQUE_READBACK_SHA256="${torque_values[1]}"
fi
if [[ -z "${XAUTHORITY:-}" ]]; then
  for candidate in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
    if [[ -f "$candidate" ]]; then export XAUTHORITY="$candidate"; break; fi
  done
fi
exec "$repo/start_arm_gui.sh"
