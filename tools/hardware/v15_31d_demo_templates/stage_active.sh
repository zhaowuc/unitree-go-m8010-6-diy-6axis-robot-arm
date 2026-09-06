#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
state="$repo/.runtime/v15_30a_gui"
evidence=@@SESSION@@/evidence
py="$repo/.venv/arm-gui/bin/python"
j6_py="${J6_PYTHON:-/home/car/go-m8010-robot-arm-v15-20a/.venv/j6-dm313/bin/python}"
ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
thermal="$ws/src/go_m8010_arm_hardware/config/thermal_limits.yaml"
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
cd "$repo"

if [[ ! -e "$evidence/model_session_anchor_validation.json" \
      || ! -e "$evidence/gravity_readonly_validation.json" \
      || ! -e "$session/model_session_anchor.json" ]]; then
  /bin/bash "$scripts/create_anchor_and_capture_gravity.sh"
fi
if [[ ! -e "$session/empirical_validation_envelope.json" ]]; then
  /bin/bash "$scripts/create_empirical_envelope.sh"
fi

# The read-only J6 controller's immutable authority is NONE.  End it in
# DISABLED and collect one final fresh handoff for the active bound worker.
systemctl --user stop @@UNIT@@-j6-controller.service 2>/dev/null || true
grep -q 'J6_FINAL_DISABLED=PASS' "$session/j6_controller.log"
j6_lib=$("$j6_py" -c 'import pathlib,sys,sysconfig
for value in (sysconfig.get_config_var("LIBDIR"), pathlib.Path(sys.prefix)/"lib"):
 p=pathlib.Path(value)
 if (p/"libstdc++.so.6").exists(): print(p.resolve()); break
else: raise SystemExit(1)')
export LD_LIBRARY_PATH="$j6_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$j6_py" "$repo/tools/hardware/j6_dm_g6220/v15_30a_j6_disabled_raw_capture.py" \
  --output "$session/j6_disabled_raw_active.json" \
  --pose-binding-id "$(cat "$session/pose_binding_id")" \
  --confirm V15_30A_J6_DISABLED_RAW_CAPTURE=YES \
  --physical-confirmation 'J6_24V_ON=YES;SUPPORT_RELIABLE=YES;WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --sample-count 500 --minimum-source-coverage-s 49.5 \
  --feedback-port 15300 \
  --feedback-session-id "$(cat "$session/session_id")" \
  --feedback-state-instance-id "$(cat "$session/state_instance_id")" \
  --feedback-handoff-output "$session/j6_feedback_handoff_active.json" \
  >"$session/j6_active_handoff_capture.log" 2>&1
"$py" - "$session/j6_disabled_raw_active.json" <<'PY'
import json, pathlib, sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert d.get("status") == "PASS" and d.get("physical_power_off_required") is False
assert d.get("terminal", {}).get("confirmed") is True
PY

# Close the state writer before changing the single-use J6 source.  The active
# gravity subscriber is started first below, then a fresh state writer starts
# with no prior source/sequence identity.  Motor workers remain BRAKE/DISABLED.
systemctl --user stop @@UNIT@@-core.service 2>/dev/null || true
[[ "$(systemctl --user is-active @@UNIT@@-core.service 2>/dev/null || true)" != active ]]

for unit in @@UNIT@@-gravity-readonly @@UNIT@@-brake-j1 @@UNIT@@-brake-j2 @@UNIT@@-brake-j345; do
  systemctl --user stop "$unit.service" 2>/dev/null || true
done
for log in "$session/boot_j1.log" "$session/boot_j2.log" "$session/boot_j345.log"; do
  grep -q 'FINAL_BRAKE=PASS' "$log"
done

# Start the single-use empirical gravity authority at zero scale.  It has no
# effect until the later operator-confirmed ladder and whole-arm HOLD.
install -m 700 "$scripts/v15_31b_start_gravity_active.sh" "$session/v15_31b_start_gravity_active.sh"
install -m 700 "$scripts/v15_31b_start_active_supervisor.sh" "$session/v15_31b_start_active_supervisor.sh"
mkdir -m 700 "$session/empirical_claims"
systemd-run --user --quiet --collect --unit=@@UNIT@@-gravity-active \
  --property=KillMode=mixed \
  --property="StandardOutput=append:$session/gravity_active.log" \
  --property="StandardError=append:$session/gravity_active.log" \
  /bin/bash "$session/v15_31b_start_gravity_active.sh"
sleep 2
[[ "$(systemctl --user is-active @@UNIT@@-gravity-active.service 2>/dev/null || true)" == active ]]

systemd-run --user --quiet --collect --unit=@@UNIT@@-core \
  --property=KillMode=mixed \
  --property="StandardOutput=append:$session/core.log" \
  --property="StandardError=append:$session/core.log" \
  /bin/bash "$session/v15_31b_start_core.sh"
deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  [[ "$(systemctl --user is-active @@UNIT@@-core.service 2>/dev/null || true)" == active ]] || { sleep 0.2; continue; }
  ss -H -lun 'sport = :15300' | grep -q . && break
  sleep 0.2
done
[[ "$(systemctl --user is-active @@UNIT@@-core.service 2>/dev/null || true)" == active ]]
ss -H -lun 'sport = :15300' | grep -q .

# Publish the deferred 30-second permits only after every long-running capture
# is complete, then immediately launch the bound worker supervisor.
pending=$(cat "$session/permit_pending_directory")
zero="$state/persistent_software_zero.json"
hints="$state/recovery_branch_hints.json"
initial="$state/initial_pose.json"
zero_sha=$(sha256sum "$zero" | awk '{print $1}')
hints_sha=$(sha256sum "$hints" | awk '{print $1}')
initial_sha=$(sha256sum "$initial" | awk '{print $1}')
j2_sha=$(sha256sum "$session/j2_brake_raw.json" | awk '{print $1}')
go_sha=$(sha256sum "$session/go_aux_brake_raw.json" | awk '{print $1}')
pose_binding_id=$(cat "$session/pose_binding_id")
j2_power=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_session_id"])' "$session/j2_brake_raw.json")
go_power=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_session_id"])' "$session/go_aux_brake_raw.json")
j2_recorded=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["recorded_at_utc"])' "$session/j2_brake_raw.json")
go_recorded=$("$py" -c 'import json,sys; print(json.load(open(sys.argv[1]))["recorded_at_utc"])' "$session/go_aux_brake_raw.json")
"$py" tools/hardware/v15_30a_create_j2_vertical_session_phase_anchor.py \
  --preserve-reference-file "$repo/.runtime/v15_31b_ft/current/j2_anchors/j2_power_session_reference_v1_20260905T100720Z_781a6fe88c63848a.json" \
  --expected-preserve-reference-sha256 "ed9948b2de6c470496a92209d8f5b6650df0e06b97118251bd0cc6b3a6f1fd0b" \
  --zero-file "$zero" --zero-sha256-file "$zero.sha256" --expected-parent-zero-sha256 "$zero_sha" \
  --recovery-hint-file "$hints" --expected-recovery-hint-sha256 "$hints_sha" \
  --initial-pose-file "$initial" --expected-initial-pose-sha256 "$initial_sha" \
  --capture-statistics-file "$session/j2_brake_raw.json" --expected-capture-sha256 "$j2_sha" \
  --operator-evidence-id "@@UNIT@@-$pose_binding_id-j2" --operator-confirmed-at-utc "$j2_recorded" \
  --operator-power-session-id "$j2_power" \
  --physical-confirmation 'J2_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --anchor-directory "$session/j2_anchors" --launch-permit-directory "$pending" \
  --apply --publish-deferred-launch-permit \
  --confirm V15_30A_CREATE_J2_VERTICAL_SESSION_PHASE_ANCHOR=YES \
  >"$session/j2_permit_publish.json"
"$py" tools/hardware/v15_30a_create_go_aux_vertical_session_phase_anchor.py \
  --preserve-reference-file "$repo/.runtime/v15_31b_ft/current/go_anchors/go_aux_power_session_reference_v1_20260905T100715Z_4279866a88d1aaf9.json" \
  --expected-preserve-reference-sha256 "11f4336a835c96625d2b687c3d1c7e41db54e67ff370eccf269a1a3c513023fb" \
  --zero-file "$zero" --zero-sha256-file "$zero.sha256" --expected-parent-zero-sha256 "$zero_sha" \
  --recovery-hint-file "$hints" --expected-recovery-hint-sha256 "$hints_sha" \
  --initial-pose-file "$initial" --expected-initial-pose-sha256 "$initial_sha" \
  --capture-statistics-file "$session/go_aux_brake_raw.json" --expected-capture-sha256 "$go_sha" \
  --operator-evidence-id "@@UNIT@@-$pose_binding_id-goaux" --operator-confirmed-at-utc "$go_recorded" \
  --operator-power-session-id "$go_power" \
  --physical-confirmation 'WHOLE_ARM_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES' \
  --anchor-directory "$session/go_anchors" --launch-permit-directory "$pending" \
  --apply --publish-deferred-launch-permit \
  --confirm V15_30A_CREATE_GO_AUX_VERTICAL_SESSION_PHASE_ANCHOR=YES \
  >"$session/go_permit_publish.json"

core_pid=$(systemctl --user show @@UNIT@@-core.service -p MainPID --value)
[[ "$core_pid" =~ ^[1-9][0-9]*$ ]]
printf '%s\n' "$core_pid" >"$session/core_pid"
systemd-run --user --quiet --collect --unit=@@UNIT@@-active-supervisor \
  --property=KillMode=mixed \
  --property="StandardOutput=append:$session/active_supervisor.log" \
  --property="StandardError=append:$session/active_supervisor.log" \
  /bin/bash "$session/v15_31b_start_active_supervisor.sh"
deadline=$((SECONDS + 25))
ready=0
while (( SECONDS < deadline )); do
  ready=1
  [[ "$(systemctl --user is-active @@UNIT@@-active-supervisor.service 2>/dev/null || true)" == active ]] || ready=0
  for port in 15310 15311 15312 15313; do
    ss -H -lun "sport = :$port" | grep -q . || ready=0
  done
  [[ "$ready" == 1 ]] && break
  sleep 0.2
done
[[ "$ready" == 1 ]]

deadline=$((SECONDS + 25))
supervisor_ready=0
while (( SECONDS < deadline )); do
  if "$py" - "$session/run/worker_supervisor_status.json" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
if not path.is_file():
    raise SystemExit(1)
value = json.loads(path.read_text(encoding="utf-8"))
domains = value.get("domains")
if not isinstance(domains, dict) or set(domains) != {"J1", "J2", "J345", "J6"}:
    raise SystemExit(1)
if not all(
    item.get("control_available") is True
    and item.get("process_alive") is True
    and item.get("udp_owner_confirmed") is True
    and item.get("reason") == "ready"
    for item in domains.values()
):
    raise SystemExit(1)
PY
  then
    supervisor_ready=1
    break
  fi
  sleep 0.2
done
[[ "$supervisor_ready" == 1 ]]
sleep 2

echo ACTIVE_RUNTIME=READY_FOR_BOUNDED_HOLD_PROBE
