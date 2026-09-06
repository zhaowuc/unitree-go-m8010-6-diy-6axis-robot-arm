#!/usr/bin/env python3
"""Repeat the validated site bootstrap and J1 out/back demo, preserving evidence.

Default: inspect required files and validate rendered Bash; no devices or units
are touched. Run on Linux with --execute --cycles 1..3 --supported --vertical
--hands-off --clearance to use conditions already confirmed by the operator.
J6_PYTHON has the same optional override as start_arm_gui.sh.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
TEMPLATES = Path(__file__).with_name("v15_31d_demo_templates")
PINNED = {
    ".runtime/v15_30a_gui/persistent_software_zero.json": "4a74e16f9e3ad6e13054811892425d17c8ca5963b6ed82b628841610f8e81ddb",
    ".runtime/v15_30a_gui/initial_pose.json": "8d646ab48022bd0dc97fb7ae79013fd6e4b89598fe0494604ca6164fb4c3f7f8",
    ".runtime/v15_31b_ft/current/j2_anchors/j2_power_session_reference_v1_20260905T100720Z_781a6fe88c63848a.json": "ed9948b2de6c470496a92209d8f5b6650df0e06b97118251bd0cc6b3a6f1fd0b",
    ".runtime/v15_31b_ft/current/go_anchors/go_aux_power_session_reference_v1_20260905T100715Z_4279866a88d1aaf9.json": "11f4336a835c96625d2b687c3d1c7e41db54e67ff370eccf269a1a3c513023fb",
}
REQUIRED = (*PINNED, ".runtime/v15_30a_gui/persistent_software_zero.json.sha256",
            ".runtime/v15_30a_gui/recovery_branch_hints.json",
            ".runtime/v15_31b_ft/current/attended_gui_params.yaml")
UNIT_SUFFIXES = ("active-supervisor", "j6-controller", "brake-j1", "brake-j2", "brake-j345",
                 "gravity-active", "gravity-readonly", "core")
BUS_LOCKS = ("v15_30a_gui_j1.lock", "go_m8010_ftasqa6f_channel3.lock",
             "v15_30a_gui_j2.lock", "v15_23d_ft_j2_channel1.lock",
             "v15_30a_gui_j345.lock", "v15_23c_j3_bus.lock", "v15_22b_j4_bus.lock",
             "v15_22a_j5_bus.lock", "v15_30a_gui_j6.lock")


def rendered(repo, session, scripts, unit):
    replacements = {"@@REPO@@": shlex.quote(str(repo)), "@@SESSION@@": shlex.quote(str(session)),
                    "@@SCRIPTS@@": shlex.quote(str(scripts)), "@@UNIT@@": unit}
    result = {}
    for source in sorted(TEMPLATES.glob("*.sh")):
        body = source.read_text(encoding="utf-8")
        for marker, value in replacements.items():
            body = body.replace(marker, value)
        if "@@" in body:
            raise ValueError(f"unresolved template marker: {source}")
        result[source.name] = body
    if len(result) != 13:
        raise ValueError("the 13 validated bootstrap/preflight templates are required")
    return result


def checked_bash(bash, bodies):
    for name, body in bodies.items():
        check = subprocess.run([bash, "-n"], input=body, text=True, encoding="utf-8", capture_output=True)
        if check.returncode:
            raise RuntimeError(f"Bash syntax invalid: {name}: {check.stderr}")


def idle_demo_cores():
    """Only read process/port ownership; never seize an existing hardware bus."""
    import fcntl
    for name in BUS_LOCKS:
        path = Path("/tmp") / name
        if path.exists():
            with path.open("rb") as stream:
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as error:
                    raise RuntimeError(f"hardware worker owns {path}; existing session was not stopped") from error
    sockets = subprocess.check_output(["ss", "-H", "-lunp"], text=True)
    occupied = [line for line in sockets.splitlines() if re.search(r":153(?:10|11|12|13)\b", line)]
    if occupied:
        raise RuntimeError("existing command receivers were not stopped: " + " | ".join(occupied))
    units = subprocess.check_output(["systemctl", "--user", "list-units", "--type=service",
                                    "--state=active,activating", "--no-legend", "--no-pager"], text=True)
    cores = []
    for line in units.splitlines():
        unit = line.split()[0]
        if not unit.startswith("v15-31"):
            continue
        if not re.fullmatch(r"v15-31d-(hold|demo-[A-Za-z0-9-]+)-core\.service", unit):
            raise RuntimeError(f"existing project service {unit}; refusing to take it over")
        command = subprocess.check_output(["systemctl", "--user", "show", unit, "-p", "ExecStart", "--value"], text=True)
        if "v15_31b_start_core.sh" not in command:
            raise RuntimeError(f"{unit} is not a recognized read-only demo core")
        cores.append(unit)
    if re.search(r":15300\b", sockets) and not cores:
        raise RuntimeError("UDP 15300 is owned outside a recognized idle demo core")
    return cores


def run_stage(command, log, timeout):
    print(f"RUNNING {log.name}", flush=True)
    with log.open("x", encoding="utf-8") as stream:
        process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = process.wait(timeout=timeout)
            if code:
                lines = log.read_text(encoding="utf-8", errors="replace").splitlines()
                reason = next((line for line in reversed(lines) if line.startswith(("BLOCKED:", "ERROR=", "PHYSICAL_ACTION_REQUIRED="))), "")
                raise RuntimeError(f"exit {code}; {reason}; see {log}")
        finally:
            def group_alive():
                process.poll()
                try:
                    os.killpg(process.pid, 0)
                    return True
                except ProcessLookupError:
                    return False
            if group_alive():
                stream.write(f"\nOWN_PROCESS_GROUP_SIGTERM={process.pid}\n")
                stream.flush()
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 25
                while group_alive() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if group_alive():
                    stream.write(f"OWN_PROCESS_GROUP_SIGKILL_AFTER_25S={process.pid}\n")
                    stream.flush()
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5)
                    raise RuntimeError(f"owned process group {process.pid} exceeded controlled-stop deadline and was terminated; see {log}")


def last_terminal_pass(text, marker):
    values = re.findall(rf"(?m)^{re.escape(marker)}=([^\r\n]*)", text)
    return bool(values and values[-1].strip() == "PASS")


def cleanup(unit, session):
    errors = []
    for suffix in UNIT_SUFFIXES:
        try:
            name = f"{unit}-{suffix}.service"
            load = ["systemctl", "--user", "show", name, "-p", "LoadState", "--value"]
            if subprocess.run(load, text=True, capture_output=True, timeout=5).stdout.strip() == "not-found":
                continue
            stopped = subprocess.run(["systemctl", "--user", "stop", name],
                                     text=True, capture_output=True, timeout=30)
            if stopped.returncode and subprocess.run(load, text=True, capture_output=True, timeout=5).stdout.strip() != "not-found":
                errors.append(f"{name}: {stopped.stderr.strip()}")
        except (OSError, subprocess.TimeoutExpired) as error:
            errors.append(f"{unit}-{suffix}: {error}")
            continue
    logs = {"J1": "j1", "J2": "j2", "J345": "j345", "J6": "j6"}
    terminal = {}
    for domain, stem in logs.items():
        path = session / "run" / f"{stem}_controller.log"
        terminal[domain] = path.is_file() and last_terminal_pass(path.read_text(encoding="utf-8", errors="replace"),
                                                               "J6_FINAL_DISABLED" if domain == "J6" else "FINAL_BRAKE")
    return {"controller_terminal_confirmed": terminal, "stop_errors": errors}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--cycles", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--bash", default=shutil.which("bash") or "/bin/bash")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--power-cycled", action="store_true",
                        help="attest an actual J6 24V power cycle since the prior commissioning session")
    parser.add_argument("--supported-near-vertical-recovery", action="store_true",
                        help="use the bounded near-original-pose recovery, then return to the original pose through the GUI before cycles")
    for flag in ("supported", "vertical", "hands-off", "clearance"):
        parser.add_argument("--" + flag, action="store_true")
    args = parser.parse_args(argv)
    token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + secrets.token_hex(3)
    session = ROOT / ".runtime" / f"v15_31d_demo_{token}"
    scripts, unit = session.with_name(session.name + "_scripts"), f"v15-31d-demo-{token}"
    bodies = rendered(ROOT, session, scripts, unit)
    checked_bash(args.bash, bodies)
    if args.self_test:
        weird = Path("/tmp/demo space 'quote' $(not-executed)")
        checked_bash(args.bash, rendered(weird, weird / "session", weird / "scripts", "v15-31d-demo-test"))
        assert all("--preserve-reference-file" in bodies[name] for name in ("stage_readonly.sh", "stage_active.sh"))
        for name, flag in (("stage_readonly.sh", "--supported-near-vertical-recovery"),
                           ("stage_active.sh", "--supported-near-vertical-recovery"),
                           ("start_bounded_j1_demo.sh", "--recover-initial-first")):
            for enabled in (False, True):
                script = f"set -- 3 {str(enabled).lower()}\n" + bodies[name].split("scripts=", 1)[0]
                check = subprocess.run([args.bash], input=script + 'printf "%s" "${recovery_args[@]}"\n',
                                       text=True, capture_output=True, encoding="utf-8", check=True)
                assert check.stdout == (flag if enabled else "")
        assert last_terminal_pass("FINAL_BRAKE=FAIL\nFINAL_BRAKE=PASS\n", "FINAL_BRAKE")
        assert not last_terminal_pass("FINAL_BRAKE=PASS\nFINAL_BRAKE=FAIL\n", "FINAL_BRAKE")
        assert not last_terminal_pass("FINAL_BRAKE=PASS\nFINAL_BRAKE=UNKNOWN\n", "FINAL_BRAKE")
        assert not last_terminal_pass("prefix J6_FINAL_DISABLED=PASS\n", "J6_FINAL_DISABLED")
        print("DEMO_SESSION_DRY_RUN_SELF_TEST=PASS; no devices or services accessed")
        return 0
    problems = [str(ROOT / name) for name in REQUIRED if not (ROOT / name).is_file()]
    problems += [f"SHA256 mismatch: {ROOT / name}" for name, sha in PINNED.items()
                 if (ROOT / name).is_file() and hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != sha]
    plan = {"mode": "EXECUTE" if args.execute else "DRY_RUN", "cycles": args.cycles,
            "j6_power_cycle_attested": args.power_cycled,
            "supported_near_vertical_recovery": args.supported_near_vertical_recovery,
            "repo": str(ROOT), "session": str(session), "scripts": str(scripts), "unit_prefix": unit,
            "missing_or_mismatched_inputs": problems, "bash_syntax": "PASS",
            "order": ["prebuild-only", "j6_posvel_preflight", "stage_readonly", "stage_active", "bounded_j1_demo", "stop_owned_units"]}
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        return 0
    if sys.platform != "linux" or not all((args.supported, args.vertical, args.hands_off, args.clearance)):
        parser.error("Linux execution requires --supported --vertical --hands-off --clearance")
    if problems:
        parser.error("required preserved inputs unavailable: " + "; ".join(problems))
    runtime = Path(os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    os.environ.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path={runtime}/bus")
    if not runtime.is_dir() or not (runtime / "bus").is_socket():
        parser.error(f"existing user runtime/D-Bus socket required: {runtime / 'bus'}")
    import fcntl
    lock_path = ROOT / ".runtime/v15_31d_demo_session.lock"
    with lock_path.open("a", encoding="utf-8") as launcher_lock:
        try:
            fcntl.flock(launcher_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error(f"another demo launcher owns {lock_path}; no existing core was stopped")
        return execute(args, bodies, plan, session, scripts, unit)


def execute(args, bodies, plan, session, scripts, unit):
    idle_demo_cores()
    scripts.mkdir(parents=True, mode=0o700)
    for name, body in bodies.items():
        (scripts / name).write_text(body, encoding="utf-8", newline="\n")
    shutil.copyfile(TEMPLATES / "fastdds_udp_only.xml", scripts / "fastdds_udp_only.xml")
    result = {"status": "FAIL", "plan": plan}
    def stop_requested(*_):
        raise KeyboardInterrupt("SIGTERM")
    signal.signal(signal.SIGTERM, stop_requested)
    try:
        run_stage([args.bash, str(ROOT / "start_arm_gui.sh"), "--prebuild-only"], scripts / "prebuild.log", 600)
        for core in idle_demo_cores():
            subprocess.run(["systemctl", "--user", "stop", core], check=True, timeout=30)
        if re.search(r":15300\b", subprocess.check_output(["ss", "-H", "-lunp"], text=True)):
            raise RuntimeError("UDP 15300 is still owned; no raw capture or new worker was started")
        run_stage([args.bash, str(scripts / "j6_posvel_preflight.sh"), str(args.power_cycled).lower()],
                  scripts / "j6_posvel_preflight.log", 120)
        for name in ("stage_readonly.sh", "stage_active.sh", "start_bounded_j1_demo.sh"):
            run_stage([args.bash, str(scripts / name), str(args.cycles),
                       str(args.supported_near_vertical_recovery).lower()], scripts / (name + ".log"), 300)
        demo_path = session / "evidence/j1_action_group_demo.json"
        demo = json.loads(demo_path.read_text(encoding="utf-8"))
        result["demo"] = {key: demo.get(key) for key in ("status", "requested_cycles", "completed_cycles", "failure")}
        result["demo"]["path"] = str(demo_path)
        result["status"] = demo["status"]
    except BaseException as error:
        result["failure"] = f"{type(error).__name__}: {error}"
    finally:
        demo_path = session / "evidence/j1_action_group_demo.json"
        if demo_path.is_file() and "demo" not in result:
            try:
                demo = json.loads(demo_path.read_text(encoding="utf-8"))
                result["demo"] = {key: demo.get(key) for key in ("status", "requested_cycles", "completed_cycles", "failure")}
                result["demo"]["path"] = str(demo_path)
            except (OSError, ValueError) as error:
                result["report_error"] = str(error)
        result.update(cleanup(unit, session))
        if result["stop_errors"] or not all(result["controller_terminal_confirmed"].values()):
            result["status"] = "FAIL"
        summary = scripts / "session_result.json"
        summary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"status": result["status"], "summary": str(summary),
                          "evidence": str(session / "evidence"), "terminal": result["controller_terminal_confirmed"]}, ensure_ascii=False))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
