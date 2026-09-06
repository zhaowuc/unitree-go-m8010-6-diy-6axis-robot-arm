#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
session=@@SESSION@@
j6_py="${J6_PYTHON:-/home/car/go-m8010-robot-arm-v15-20a/.venv/j6-dm313/bin/python}"
j6_lib="$("$j6_py" -c 'import pathlib,sys,sysconfig
for value in (sysconfig.get_config_var("LIBDIR"), pathlib.Path(sys.prefix)/"lib"):
 p=pathlib.Path(value)
 if (p/"libstdc++.so.6").exists(): print(p.resolve()); break
else: raise SystemExit(1)')"
export LD_LIBRARY_PATH="$j6_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$j6_py" \
  "$repo/tools/hardware/j6_dm_g6220/v15_30a_j6_disabled_raw_capture.py" \
  --output "$session/j6_disabled_raw.json" \
  --pose-binding-id "$(cat "$session/pose_binding_id")" \
  --confirm V15_30A_J6_DISABLED_RAW_CAPTURE=YES \
  --physical-confirmation "J6_24V_ON=YES;SUPPORT_RELIABLE=YES;WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES" \
  --sample-count 500 \
  --minimum-source-coverage-s 49.5 \
  --feedback-port 15300 \
  --feedback-session-id "$(cat "$session/session_id")" \
  --feedback-state-instance-id "$(cat "$session/state_instance_id")" \
  --feedback-handoff-output "$session/j6_feedback_handoff.json"
