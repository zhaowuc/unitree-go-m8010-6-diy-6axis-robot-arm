#!/usr/bin/env bash
set -Eeuo pipefail
recovery_args=()
if [[ "${2:-false}" == true ]]; then
  recovery_args+=(--supported-near-vertical-recovery)
fi
scripts=@@SCRIPTS@@
repo=@@REPO@@
current=@@SESSION@@
evidence=@@SESSION@@/evidence
state="$repo/.runtime/v15_30a_gui"
py="$repo/.venv/arm-gui/bin/python"
j6_py="${J6_PYTHON:-/home/car/go-m8010-robot-arm-v15-20a/.venv/j6-dm313/bin/python}"
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
worker="$ws/build/v15_30a_gui_tools/v15_30a_gui_go_controller"
thermal="$ws/src/go_m8010_arm_hardware/config/thermal_limits.yaml"
model="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
expected_commit=$(git -C "$repo" rev-parse HEAD)
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
cd "$repo"
[[ "$(sha256sum "$state/persistent_software_zero.json" | cut -d ' ' -f1)" == 4a74e16f9e3ad6e13054811892425d17c8ca5963b6ed82b628841610f8e81ddb ]]
[[ "$(sha256sum "$state/initial_pose.json" | cut -d ' ' -f1)" == 8d646ab48022bd0dc97fb7ae79013fd6e4b89598fe0494604ca6164fb4c3f7f8 ]]
[[ ! -e "$current" ]]
mkdir -m 700 "$current" "$evidence" "$current/j2_anchors" "$current/go_anchors" "$current/run"
install -m 700 "$scripts/v15_31b_start_core.sh" "$current/v15_31b_start_core.sh"
install -m 700 "$scripts/v15_31b_start_brake_worker.sh" "$current/v15_31b_start_brake_worker.sh"
install -m 700 "$scripts/v15_31b_start_j6_raw_500.sh" "$current/v15_31b_start_j6_raw_500.sh"
install -m 700 "$scripts/v15_31b_start_gravity_readonly.sh" "$current/v15_31b_start_gravity_readonly.sh"
worker_sha=$(sha256sum "$worker" | awk '{print $1}')
thermal_sha=$(sha256sum "$thermal" | awk '{print $1}')
operator_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
pose_binding_id=$(printf '%s' \
  "V15.31B|$operator_utc|$expected_commit|WHOLE_ARM_VERTICAL_INITIALIZATION_POSE" \
  | sha256sum | awk '{print $1}')
umask 077
printf '%s\n' "$operator_utc" >"$current/operator_confirmed_at_utc"
printf '%s\n' "$pose_binding_id" >"$current/pose_binding_id"
printf '%s\n' "$worker_sha" >"$current/worker_sha256"
printf '%s\n' "$thermal_sha" >"$current/thermal_sha256"

"$py" tools/hardware/v15_30a_go_aux_brake_raw_capture.py \
  --worker "$worker" --expected-worker-sha256 "$worker_sha" \
  --thermal-config "$thermal" --expected-thermal-config-sha256 "$thermal_sha" \
  --target-packets 500 --maximum-runtime-s 15 \
  --output "$current/go_aux_brake_raw.json" \
  --confirm V15_30A_GO_AUX_BRAKE_RAW_CAPTURE=YES \
  --physical-confirmation '24V_ON=YES;SUPPORT_RELIABLE=YES;WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --pose-binding-id "$pose_binding_id" \
  >"$current/go_aux_capture.log" 2>&1
"$py" tools/hardware/v15_30a_j2_brake_raw_capture.py \
  --worker "$worker" --expected-worker-sha256 "$worker_sha" \
  --thermal-config "$thermal" --expected-thermal-config-sha256 "$thermal_sha" \
  --target-packets 500 --maximum-runtime-s 15 \
  --output "$current/j2_brake_raw.json" \
  --confirm V15_30A_J2_BRAKE_RAW_CAPTURE=YES \
  --physical-confirmation '24V_ON=YES;SUPPORT_RELIABLE=YES;J2_VERTICAL_INITIALIZATION_POSE=YES;ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --pose-binding-id "$pose_binding_id" \
  >"$current/j2_capture.log" 2>&1
"$py" - "$current/go_aux_brake_raw.json" "$current/j2_brake_raw.json" <<'PY'
import json, pathlib, sys
for name in sys.argv[1:]:
    d=json.loads(pathlib.Path(name).read_text(encoding="utf-8"))
    assert d.get("status") == "PASS" and d.get("physical_power_off_required") is False
PY

lifecycle_root=/run/user/$(id -u)/go-m8010/anchors
pending="$lifecycle_root/pending"
install -d -m 700 "$pending" "$lifecycle_root/inflight" "$lifecycle_root/spent"
printf '%s\n' "$pending" >"$current/permit_pending_directory"
zero="$state/persistent_software_zero.json"
hints="$state/recovery_branch_hints.json"
initial="$state/initial_pose.json"
zero_sha=$(sha256sum "$zero" | awk '{print $1}')
hints_sha=$(sha256sum "$hints" | awk '{print $1}')
initial_sha=$(sha256sum "$initial" | awk '{print $1}')
j2_sha=$(sha256sum "$current/j2_brake_raw.json" | awk '{print $1}')
go_sha=$(sha256sum "$current/go_aux_brake_raw.json" | awk '{print $1}')
j2_power=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_session_id"])' "$current/j2_brake_raw.json")
go_power=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_session_id"])' "$current/go_aux_brake_raw.json")
j2_recorded=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["recorded_at_utc"])' "$current/j2_brake_raw.json")
go_recorded=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["recorded_at_utc"])' "$current/go_aux_brake_raw.json")

"$py" tools/hardware/v15_30a_create_j2_vertical_session_phase_anchor.py \
  "${recovery_args[@]}" \
  --preserve-reference-file "$repo/.runtime/v15_31b_ft/current/j2_anchors/j2_power_session_reference_v1_20260905T100720Z_781a6fe88c63848a.json" \
  --expected-preserve-reference-sha256 "ed9948b2de6c470496a92209d8f5b6650df0e06b97118251bd0cc6b3a6f1fd0b" \
  --zero-file "$zero" --zero-sha256-file "$zero.sha256" \
  --expected-parent-zero-sha256 "$zero_sha" \
  --recovery-hint-file "$hints" --expected-recovery-hint-sha256 "$hints_sha" \
  --initial-pose-file "$initial" --expected-initial-pose-sha256 "$initial_sha" \
  --capture-statistics-file "$current/j2_brake_raw.json" --expected-capture-sha256 "$j2_sha" \
  --operator-evidence-id "@@UNIT@@-$pose_binding_id-j2" \
  --operator-confirmed-at-utc "$j2_recorded" --operator-power-session-id "$j2_power" \
  --physical-confirmation 'J2_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --anchor-directory "$current/j2_anchors" --launch-permit-directory "$pending" \
  --apply --defer-launch-permit \
  --confirm V15_30A_CREATE_J2_VERTICAL_SESSION_PHASE_ANCHOR=YES \
  >"$current/j2_anchor_apply.json"
"$py" tools/hardware/v15_30a_create_go_aux_vertical_session_phase_anchor.py \
  "${recovery_args[@]}" \
  --preserve-reference-file "$repo/.runtime/v15_31b_ft/current/go_anchors/go_aux_power_session_reference_v1_20260905T100715Z_4279866a88d1aaf9.json" \
  --expected-preserve-reference-sha256 "11f4336a835c96625d2b687c3d1c7e41db54e67ff370eccf269a1a3c513023fb" \
  --zero-file "$zero" --zero-sha256-file "$zero.sha256" \
  --expected-parent-zero-sha256 "$zero_sha" \
  --recovery-hint-file "$hints" --expected-recovery-hint-sha256 "$hints_sha" \
  --initial-pose-file "$initial" --expected-initial-pose-sha256 "$initial_sha" \
  --capture-statistics-file "$current/go_aux_brake_raw.json" --expected-capture-sha256 "$go_sha" \
  --operator-evidence-id "@@UNIT@@-$pose_binding_id-goaux" \
  --operator-confirmed-at-utc "$go_recorded" --operator-power-session-id "$go_power" \
  --physical-confirmation 'WHOLE_ARM_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --anchor-directory "$current/go_anchors" --launch-permit-directory "$pending" \
  --apply --defer-launch-permit \
  --confirm V15_30A_CREATE_GO_AUX_VERTICAL_SESSION_PHASE_ANCHOR=YES \
  >"$current/go_anchor_apply.json"

start_unit() {
  local unit=$1 log=$2; shift 2
  systemd-run --user --quiet --collect --unit="$unit" \
    --property=KillMode=mixed \
    --property="StandardOutput=append:$log" \
    --property="StandardError=append:$log" "$@"
}
start_unit @@UNIT@@-core "$current/core.log" /bin/bash "$current/v15_31b_start_core.sh"
deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  [[ -s "$current/session_id" && -s "$current/state_instance_id" ]] || { sleep 0.2; continue; }
  ss -H -lun 'sport = :15300' | grep -q . && break
  sleep 0.2
done
[[ -s "$current/session_id" && -s "$current/state_instance_id" ]]
ss -H -lun 'sport = :15300' | grep -q .
start_unit @@UNIT@@-brake-j1 "$current/boot_j1.log" /bin/bash "$current/v15_31b_start_brake_worker.sh" j1
start_unit @@UNIT@@-brake-j2 "$current/boot_j2.log" /bin/bash "$current/v15_31b_start_brake_worker.sh" j2
start_unit @@UNIT@@-brake-j345 "$current/boot_j345.log" /bin/bash "$current/v15_31b_start_brake_worker.sh" j345
deadline=$((SECONDS + 12))
ready=0
while (( SECONDS < deadline )); do
  ready=1
  for unit in @@UNIT@@-brake-j1 @@UNIT@@-brake-j2 @@UNIT@@-brake-j345; do
    [[ "$(systemctl --user is-active "$unit.service" 2>/dev/null || true)" == active ]] || ready=0
  done
  if [[ "$ready" == 1 ]]; then
    sleep 1
    ready=1
    for unit in @@UNIT@@-brake-j1 @@UNIT@@-brake-j2 @@UNIT@@-brake-j345; do
      [[ "$(systemctl --user is-active "$unit.service" 2>/dev/null || true)" == active ]] || ready=0
    done
    [[ "$ready" == 1 ]] && break
  fi
  sleep 0.2
done
[[ "$ready" == 1 ]]

/bin/bash "$current/v15_31b_start_j6_raw_500.sh" >"$current/j6_raw.log" 2>&1
"$py" - "$current/j6_disabled_raw.json" <<'PY'
import json, pathlib, sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert d.get("status") == "PASS" and d.get("physical_power_off_required") is False
assert d.get("terminal", {}).get("confirmed") is True
PY
j6_lib=$("$j6_py" -c 'import pathlib,sys,sysconfig
for value in (sysconfig.get_config_var("LIBDIR"), pathlib.Path(sys.prefix)/"lib"):
 p=pathlib.Path(value)
 if (p/"libstdc++.so.6").exists(): print(p.resolve()); break
else: raise SystemExit(1)')
session_id=$(cat "$current/session_id")
state_id=$(cat "$current/state_instance_id")
start_unit @@UNIT@@-j6-controller "$current/j6_controller.log" \
  /usr/bin/env "LD_LIBRARY_PATH=$j6_lib" "$j6_py" \
  "$repo/tools/hardware/j6_dm_g6220/v15_30a_gui_j6_controller.py" \
  --execute --confirm V15_30A_GUI_J6_CONTROL_AUTHORIZED=YES \
  --command-port 15311 --feedback-port 15300 --zero-file "$zero" \
  --thermal-config "$thermal" --expected-gravity-authority-class NONE \
  --feedback-session-id "$session_id" --feedback-state-instance-id "$state_id" \
  --feedback-handoff-file "$current/j6_feedback_handoff.json"
deadline=$((SECONDS + 12))
while (( SECONDS < deadline )); do
  ss -H -lun 'sport = :15311' | grep -q . && break
  sleep 0.2
done
ss -H -lun 'sport = :15311' | grep -q .
sleep 2

/bin/bash "$scripts/restart_core_readonly.sh"

set +u
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONNOUSERSITE=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$scripts/fastdds_udp_only.xml"
# Preserve the model-minus-logical offset; never map the measured drift to zero.
timeout 8 ros2 topic echo --once --full-length /whole_arm/hardware_state std_msgs/msg/String >"$current/model_pose_snapshot.yaml"
readarray -t model_q < <("$py" - "$current/model_pose_snapshot.yaml" <<'PY'
import json,sys,yaml,math
v=json.loads(yaml.safe_load(open(sys.argv[1]).read().replace('\n---',''))['data'])
assert v['telemetry_healthy'] is True
for q, offset in zip(v['position_rad'], (0,90,-14.40,13.49,47.94,0)):
    assert math.isfinite(q)
    print(q + math.radians(offset))
PY
)
[[ ${#model_q[@]} -eq 6 ]]
capture_power_readonly() {
  "$py" tools/hardware/v15_31b_capture_power_on_readonly.py \
    --execute-readonly \
    --confirm '24V_ON=YES;SUPPORT_RELIABLE=YES;MODEL_POSE_ALIGNED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES;OPERATOR_STOP_READY=YES' \
    --model-path "$model" \
    --model-absolute-q-rad "${model_q[@]}" \
    --operator-confirmed-at-utc "$operator_utc" \
    --output "$evidence/power_on_readonly.json" --timeout-s 30
}
if ! capture_power_readonly >"$current/power_on_readonly.log" 2>&1; then
  [[ ! -e "$evidence/power_on_readonly.json" ]]
  sleep 1
  capture_power_readonly >>"$current/power_on_readonly.log" 2>&1
fi
"$py" - "$evidence/power_on_readonly.json" <<'PY'
import json, pathlib, sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert d.get("status") == "PASS" and float(d.get("duration_s", 0)) >= 10
print("PHASE_A=PASS")
print(f"SESSION_ID={d['session_id']}")
print(f"STATE_INSTANCE_ID={d['state_instance_id']}")
print(f"VALID_FRAMES={d['valid_frame_count']}")
PY
