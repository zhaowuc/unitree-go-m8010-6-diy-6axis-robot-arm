#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
py="$repo/.venv/arm-gui/bin/python"
evidence=@@SESSION@@/evidence
power="$evidence/power_on_readonly.json"
anchor="$evidence/model_session_anchor_validation.json"
gravity="$evidence/gravity_readonly_validation.json"
model="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
gravity_config="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_hardware/config/gravity_control.yaml"
thermal_config="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_hardware/config/thermal_limits.yaml"
output="$session/empirical_validation_envelope.json"
result="$session/empirical_validation_envelope_create_result.json"
cd "$repo"
readarray -t values < <("$py" - "$power" "$anchor" "$gravity" <<'PY'
import hashlib, json, pathlib, sys
docs=[]
for name in sys.argv[1:]:
    p=pathlib.Path(name)
    d=json.loads(p.read_text(encoding="utf-8"))
    assert d.get("result", d.get("status")) == "PASS"
    docs.append((d, hashlib.sha256(p.read_bytes()).hexdigest()))
power, anchor, gravity = [item[0] for item in docs]
assert power["session_id"] == anchor["session_id"] == gravity["session_id"]
assert power["state_instance_id"] == anchor["state_instance_id"] == gravity["state_instance_id"]
print(docs[0][1]); print(docs[1][1]); print(docs[2][1])
print(power["session_id"]); print(power["state_instance_id"])
PY
)
power_sha=${values[0]}
anchor_sha=${values[1]}
gravity_sha=${values[2]}
session_id=${values[3]}
state_id=${values[4]}
[[ ! -e "$output" && ! -e "$result" ]]
teach_args=()
if [[ "${GO_ASSISTED_TEACH:-0}" == 1 ]]; then teach_args+=(--assisted-teach); fi
"$py" tools/hardware/v15_31b_create_empirical_validation_envelope.py \
  --power-on-readonly "$power" \
  --expected-power-on-sha256 "$power_sha" \
  --model-session-anchor-validation "$anchor" \
  --expected-anchor-validation-sha256 "$anchor_sha" \
  --gravity-readonly-validation "$gravity" \
  --expected-gravity-readonly-sha256 "$gravity_sha" \
  --model "$model" \
  --gravity-config "$gravity_config" \
  --thermal-config "$thermal_config" \
  --expected-session-id "$session_id" \
  --expected-state-instance-id "$state_id" \
  "${teach_args[@]}" \
  --hold-seconds 10 \
  --lifetime-seconds 4200 \
  --output "$output" --apply \
  --confirm V15_31B_CREATE_EMPIRICAL_VALIDATION_ENVELOPE=YES \
  >"$result"
"$py" - "$output" "$result" <<'PY'
import hashlib, json, pathlib, sys
out=pathlib.Path(sys.argv[1]); result=json.loads(pathlib.Path(sys.argv[2]).read_text(encoding="utf-8"))
assert result.get("status") == "PASS" and result.get("applied") is True
sha=hashlib.sha256(out.read_bytes()).hexdigest()
assert sha == result["envelope_sha256"]
print("EMPIRICAL_ENVELOPE=PASS")
print(f"ENVELOPE_SHA256={sha}")
print(f"ENVELOPE_ID={result['envelope']['envelope_id']}")
print(f"EXPIRES_AT_UTC={result['envelope']['expires_at_utc']}")
PY
