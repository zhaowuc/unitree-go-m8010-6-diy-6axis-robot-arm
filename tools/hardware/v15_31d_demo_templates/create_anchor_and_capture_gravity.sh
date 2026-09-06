#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
py="$repo/.venv/arm-gui/bin/python"
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
model="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
evidence=@@SESSION@@/evidence
power="$evidence/power_on_readonly.json"
anchor_result="$evidence/model_session_anchor_validation.json"
anchor_apply_result="$session/model_session_anchor_apply_result.json"
gravity_result="$evidence/gravity_readonly_validation.json"
runtime_anchor="$session/model_session_anchor.json"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
cd "$repo"

readarray -t identity < <("$py" - "$power" <<'PY'
import hashlib, json, pathlib, sys
p=pathlib.Path(sys.argv[1]); d=json.loads(p.read_text(encoding="utf-8"))
assert d.get("status") == "PASS" and float(d.get("duration_s", 0)) >= 10
print(d["session_id"])
print(d["state_instance_id"])
print(hashlib.sha256(p.read_bytes()).hexdigest())
PY
)
session_id=${identity[0]}
state_id=${identity[1]}
power_sha=${identity[2]}
[[ ! -e "$anchor_result" && ! -e "$anchor_apply_result" && ! -e "$runtime_anchor" ]]
"$py" tools/hardware/v15_31b_create_model_session_anchor.py \
  --power-on-readonly "$power" \
  --expected-power-on-sha256 "$power_sha" \
  --expected-session-id "$session_id" \
  --expected-state-instance-id "$state_id" \
  --output "$runtime_anchor" \
  --validation-output "$anchor_result" --apply \
  --confirm V15_31B_CREATE_MODEL_SESSION_ANCHOR=YES \
  >"$anchor_apply_result"
anchor_sha=$("$py" - "$anchor_apply_result" <<'PY'
import json, pathlib, sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert d.get("status") == "PASS" and d.get("mode") == "APPLY"
assert d.get("applied") is True and d.get("hardware_accessed") is False
print(d["runtime_anchor_sha256"])
PY
)
[[ "$(sha256sum "$runtime_anchor" | awk '{print $1}')" == "$anchor_sha" ]]

systemd-run --user --quiet --collect --unit=@@UNIT@@-gravity-readonly \
  --property=KillMode=mixed \
  --property="StandardOutput=append:$session/gravity_readonly_node.log" \
  --property="StandardError=append:$session/gravity_readonly_node.log" \
  /bin/bash "$session/v15_31b_start_gravity_readonly.sh"
deadline=$((SECONDS + 15))
while (( SECONDS < deadline )); do
  active=$(systemctl --user is-active @@UNIT@@-gravity-readonly.service 2>/dev/null || true)
  [[ "$active" == active ]] && break
  sleep 0.2
done
[[ "${active:-}" == active ]]
sleep 2

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
[[ ! -e "$gravity_result" ]]
capture_gravity_readonly() {
  "$py" tools/hardware/v15_31b_capture_gravity_readonly.py \
    --execute-readonly \
    --confirm '24V_ON=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;NO_ONE_TOUCHING=YES;MODEL_SESSION_ANCHOR_V2=APPLIED;OPERATOR_STOP_READY=YES' \
    --model-path "$model" \
    --anchor "$runtime_anchor" \
    --expected-anchor-sha256 "$anchor_sha" \
    --expected-session-id "$session_id" \
    --expected-state-instance-id "$state_id" \
    --output "$gravity_result" --timeout-s 35
}
if ! capture_gravity_readonly >"$session/gravity_readonly_capture.log" 2>&1; then
  [[ ! -e "$gravity_result" ]]
  sleep 1
  capture_gravity_readonly >>"$session/gravity_readonly_capture.log" 2>&1
fi
"$py" - "$gravity_result" <<'PY'
import json, pathlib, sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert d.get("result", d.get("status")) == "PASS", d
assert float(d.get("duration_s", 0)) >= 10
assert d.get("hardware_tff_enabled") is False
assert d.get("tff_transmitted") is False
print("MODEL_SESSION_ANCHOR=PASS")
print("GRAVITY_READONLY=PASS")
print(f"DURATION_S={d['duration_s']}")
print(f"SAMPLE_COUNT={d.get('sample_count', d.get('valid_sample_count'))}")
print(f"ANCHOR_SHA256={d['anchor_sha256']}")
PY
