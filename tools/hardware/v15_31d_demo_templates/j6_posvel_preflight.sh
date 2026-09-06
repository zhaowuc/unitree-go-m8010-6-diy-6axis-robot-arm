#!/usr/bin/env bash
set -Eeuo pipefail
scripts=@@SCRIPTS@@
repo=@@REPO@@
j6_py="${J6_PYTHON:-/home/car/go-m8010-robot-arm-v15-20a/.venv/j6-dm313/bin/python}"
tool="$repo/tools/hardware/j6_dm_g6220/j6_posvel_mode_commissioning.py"
inventory="$repo/.runtime/v15_30a_gui/j6_posvel_commissioning_state.json"
output_dir="$scripts/j6_posvel_commissioning"
mkdir -m 700 "$output_dir"
cd "$repo"
# A consumed mode-write session must not be recommissioned without a new power
# cycle. Normal repeats reuse its configuration; the J6 worker still reads RID10
# and requires live POS_VEL before it can enable the drive.
if [[ "${1:-false}" != true ]]; then
  "$j6_py" - "$inventory" <<'PY'
import json,pathlib,sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
mode=d.get("ctrl_mode_after_write",d.get("ctrl_mode_readonly_verification",{}).get("after_disabled_baseline"))
assert d.get("status")=="COMPLETED" and mode==2, "No completed POS_VEL configuration; after a real power cycle use --power-cycled"
assert d.get("final_disable",{}).get("confirmed") is True
print("J6_CONFIGURATION_REUSED=YES; LIVE_RID10_CHECK_REQUIRED_BY_WORKER=YES; NO_MODE_WRITE=YES")
PY
  exit 0
fi
j6_lib=$("$j6_py" -c 'import pathlib,sys,sysconfig
for value in (sysconfig.get_config_var("LIBDIR"), pathlib.Path(sys.prefix)/"lib"):
 p=pathlib.Path(value)
 if (p/"libstdc++.so.6").exists(): print(p.resolve()); break
else: raise SystemExit(1)')
export LD_LIBRARY_PATH="$j6_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
power_cycle=()
if [[ "${1:-false}" == true ]]; then
  power_cycle=(--power-cycle-attestation J6_24V_POWER_CYCLED_SINCE_PRIOR_SESSION=YES)
fi
"$j6_py" "$tool" preflight "${power_cycle[@]}" --output-dir "$output_dir"
token=$("$j6_py" - "$inventory" <<'PY'
import json,pathlib,sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
assert d.get("status") in {"PREFLIGHT_READY", "COMPLETED"}, d
assert d.get("final_disable", {}).get("confirmed") is True, d
if d["status"] == "PREFLIGHT_READY":
    token=d["commissioning_session"]["token"]
    assert isinstance(token,str) and len(token)==64
    print(token)
PY
)
if [[ -n "$token" ]]; then
  "$j6_py" "$tool" switch-and-baseline \
    --operator-gate J6_ALLOW_RUNTIME_CTRL_MODE_SWITCH_TO_POS_VEL=YES \
    --session-token "$token" --output-dir "$output_dir"
fi
"$j6_py" - "$inventory" "$output_dir/result.json" <<'PY'
import json,pathlib,sys
d=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
write=d.get("runtime_mode_write", {})
disabled=d.get("final_disable", {})
mode=d.get("ctrl_mode_after_write", d.get("ctrl_mode_readonly_verification", {}).get("after_disabled_baseline"))
assert d.get("status")=="COMPLETED" and mode==2, d
assert d.get("physical_action_required")=="NONE", d
assert d.get("posvel_disabled_baseline", {}).get("status")=="PASS", d
assert disabled.get("confirmed") is True and disabled.get("feedback_frames",0)>=5, d
assert disabled.get("states")==[0]*5 and disabled.get("errors")==[], d
assert write.get("rid10_write_count") in (0,1), d
assert all(d["safety"].get(key) is False for key in (
    "flash_eeprom_save_used","set_zero_used","id_write_used","other_rid_write_used","automatic_mit_rollback_used")), d
assert write.get("flash_eeprom_save_used") is False, d
result={"status":"PASS","ctrl_mode_rid10":mode,"rid10_write_count":write["rid10_write_count"],
        "final_disabled_frames":disabled["feedback_frames"],"canonical_ledger":sys.argv[1]}
with pathlib.Path(sys.argv[2]).open("x",encoding="utf-8") as stream:
    json.dump(result,stream,ensure_ascii=False,indent=2)
print("J6_POSVEL_PREFLIGHT=PASS "+json.dumps(result))
PY
