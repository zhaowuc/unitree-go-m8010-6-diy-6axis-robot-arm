#!/usr/bin/env python3
"""Build and validate the frozen V15.15 link2-to-gripper mass ledger V1.

Only mass bookkeeping is produced.  No COM, inertia, gravity, damping,
friction, armature, actuator, mesh, kinematic, TF, MoveIt, ros2_control or
trajectory-bridge data is generated or modified.
"""

from __future__ import annotations

import argparse
from collections import Counter
from decimal import Decimal
import json
from pathlib import Path
import subprocess
import sys

from build_mass_mapping_v15_15 import (
    PROTECTED_INPUTS,
    find_repo_root,
    load_authorities,
    snapshot_protected_inputs,
)


D = Decimal
SOURCE_BRANCH = "agent/v15-14-trajectory-loop"
TARGET_BRANCH = "agent/v15-15-mass-ledger-v1"
SOURCE_COMMIT = "d284bd68388bc61aa1a7badce6fcaa6437a820c5"
SOURCE_TREE = "49574af97884b8ec9ab05c7cc1fd36e2a586f62c"
SCHEMA = "go-m8010-arm-v15.15-mass-ledger-v1/1.0"
REVISION = "V15.15-MASS_LEDGER_V1"
OUTPUT_JSON = "V15_15_实测质量账本_v1.json"
OUTPUT_MD = "V15_15_实测质量账本_v1.md"
SCOPE_LINKS = ("link2", "link3", "link4", "link5", "link6", "gripper")
TOLERANCE = D("1e-9")

ALLOWED_CHANGED_PATHS = {
    "V15_15_实测质量映射契约.json",
    "V15_15_实测质量映射说明.md",
    "tools/build_mass_mapping_v15_15.py",
    OUTPUT_JSON,
    OUTPUT_MD,
    "tools/validate_mass_ledger_v15_15.py",
}

M_GO = "M_GO_M8010_COMPLETE_WITH_WIRING"
M_DM = "M_DM_G6220_COMPLETE"
M_UPPER = "M_UPPER_ARM_PRINT"
M_FOREARM = "M_FOREARM_PRINT"
M_WRIST = "M_WRIST_PRELINK_PRINT"
M_GRIPPER = "M_GRIPPER_CONNECTOR_CAMERA_COMPLETE"
M_A = "M_COMPOUND_A_DM_TO_END"
M_B = "M_COMPOUND_B_J5_TO_END"
M_C = "M_COMPOUND_C_FOREARM_TO_END"

R_A = "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING"
R_B = "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING"
R_C = "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING"

GO_TOTAL = D("0.540")
GO_OUTPUT = D("0.070")
GO_STATOR = D("0.470")
DM_TOTAL = D("0.500")
DM_STATOR = D("0.375")
DM_OUTPUT = D("0.125")
UPPER = D("0.5515")
FOREARM = D("0.135")
WRIST = D("0.089")
GRIPPER = D("0.296")
COMPOUND_A = D("0.855")
COMPOUND_B = D("1.441")
COMPOUND_C = D("2.220")
RESIDUAL_A = D("0.059")
RESIDUAL_B = D("0.046")
RESIDUAL_C = D("0.015")

LINK2_POLICY_MEMBERS = (
    "UpperArm_M35_Bottom_01",
    "UpperArm_M35_Bottom_02",
    "UpperArm_M35_Bottom_03",
    "UpperArm_M35_Top_01",
    "UpperArm_M35_Top_02",
    "UpperArm_M35_Top_03",
    "J3_Output_M4_Screw_01",
    "J3_Output_M4_Screw_02",
    "J3_Output_M4_Screw_03",
    "J3_Output_M4_Screw_04",
    "J3_Output_M4_Screw_05",
    "J3_Output_M4_Screw_06",
)
RESIDUAL_A_MEMBERS = (
    "J5_to_Damiao_J6_Adapter_STL_Display",
    "Adapter_DM_G6220_M4x12_Screw_01",
    "Adapter_DM_G6220_M4x12_Screw_02",
    "Adapter_DM_G6220_M4x12_Screw_03",
)
RESIDUAL_B_MEMBERS = tuple(
    [f"J4_Output_M4_Screw_{i:02d}" for i in range(1, 7)]
    + [f"Wrist_Prelink_J5_M35_Screw_{i:02d}" for i in range(1, 7)]
)
RESIDUAL_C_MEMBERS = tuple(
    [f"Forearm_J3_M35_Screw_{i:02d}" for i in range(1, 7)]
    + [f"Forearm_J4_M35_Screw_{i:02d}" for i in range(1, 7)]
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def num(value: Decimal) -> float:
    return float(value)


def git_bytes(repo: Path, *args: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(repo), *args])


def git_text(repo: Path, *args: str) -> str:
    return git_bytes(repo, *args).decode("utf-8").strip()


def audit_git_scope(repo: Path) -> dict:
    branch = git_text(repo, "branch", "--show-current")
    require(branch == TARGET_BRANCH, f"wrong branch: expected {TARGET_BRANCH}, got {branch}")
    require(git_text(repo, "rev-parse", f"{SOURCE_COMMIT}^{{tree}}") == SOURCE_TREE, "source tree drift")
    require(
        subprocess.run(
            ["git", "-C", str(repo), "merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD"],
            check=False,
        ).returncode
        == 0,
        "V15.14 source commit is not an ancestor of HEAD",
    )
    if subprocess.run(
        ["git", "-C", str(repo), "show-ref", "--verify", "--quiet", f"refs/remotes/origin/{SOURCE_BRANCH}"],
        check=False,
    ).returncode == 0:
        require(
            git_text(repo, "rev-parse", f"origin/{SOURCE_BRANCH}") == SOURCE_COMMIT,
            "origin V15.14 baseline no longer matches accepted source commit",
        )

    tracked = {
        item.decode("utf-8")
        for item in git_bytes(
            repo,
            "-c",
            "core.quotepath=false",
            "diff",
            "--name-only",
            "-z",
            SOURCE_COMMIT,
            "--",
        ).split(b"\0")
        if item
    }
    untracked = {
        item.decode("utf-8")
        for item in git_bytes(
            repo,
            "-c",
            "core.quotepath=false",
            "ls-files",
            "--others",
            "--exclude-standard",
            "-z",
        ).split(b"\0")
        if item
    }
    changed = tracked | untracked
    unexpected = sorted(changed - ALLOWED_CHANGED_PATHS)
    require(not unexpected, f"files outside V15.15 mass-ledger allowlist changed: {unexpected}")
    # The two generated documents may not exist on the first --write pass.
    # Normalize them into the reported work-product set so --write followed by
    # --check is byte-for-byte deterministic without weakening the actual
    # unexpected-path gate above.
    reported_work_products = changed | {OUTPUT_JSON, OUTPUT_MD}
    return {
        "current_branch": branch,
        "source_branch": SOURCE_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "changed_paths": sorted(reported_work_products),
        "allowed_changed_paths": sorted(ALLOWED_CHANGED_PATHS),
        "unexpected_changed_paths": unexpected,
        "pass": True,
    }


def component(
    component_id: str,
    name: str,
    mass: Decimal,
    status: str,
    link: str,
    cad_members: list[str],
    source_id: str,
    notes: str,
    *,
    cad_owners: list[str] | None = None,
    mapping_policy: str = "DIRECT_CAD_OWNER",
    non_cad_scope: list[str] | None = None,
) -> dict:
    return {
        "component_id": component_id,
        "name": name,
        "nominal_mass_kg": num(mass),
        "mass_status": status,
        "ledger_link": link,
        "cad_members": cad_members,
        "cad_owners": cad_owners or [link],
        "geometry_role": "visual",
        "source_measurement_or_model_id": source_id,
        "accounting_role": "ADDITIVE_LEAF",
        "owner_mapping_policy": mapping_policy,
        "non_cad_scope": non_cad_scope or [],
        "notes": notes,
    }


def exact_owner(authority: dict, token: str, owner: str) -> None:
    row = authority["visual_by_token"].get(token)
    require(row is not None, f"missing visual CAD member: {token}")
    require(
        row["owner"] == owner,
        f"CAD_OWNER_CONFLICT EXPECTED_OWNER={owner} CAD_OWNER={row['owner']} OBJECT_NAME={token} "
        "CONFLICT_REASON=rigid_link_membership owner mismatch",
    )
    require(row["geometry_role"] == "visual", f"non-visual mass member: {token}")
    require(row["urdf_link_count"] == "1", f"non-unique CAD ownership count: {token}")
    require(row["source_object"] in authority["object_names"], f"FCStd source object missing: {token}")


def build_ledger(authority: dict) -> tuple[dict, list[dict], list[dict], dict]:
    v = authority["visual_by_token"]

    link_components: dict[str, list[dict]] = {link: [] for link in SCOPE_LINKS}

    motor_components = [
        component("J2A_OUTPUT_EQ", "J2A GO-M8010 output + neutral equivalent", GO_OUTPUT, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link2", ["derived:J2A_GO_M8010_output", "derived:J2A_GO_M8010_neutral"], "MODEL_GO_M8010_V1", "B6808 neutral is included; no separate mass."),
        component("J2B_OUTPUT_EQ", "J2B GO-M8010 output + neutral equivalent", GO_OUTPUT, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link2", ["derived:J2B_GO_M8010_output", "derived:J2B_GO_M8010_neutral"], "MODEL_GO_M8010_V1", "B6808 neutral is included; no separate mass."),
        component("J3_OUTPUT_EQ", "J3 GO-M8010 output + neutral equivalent", GO_OUTPUT, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link2", ["J3_Output_Rotor_STEP_Display", "J3_B6808_Bearing_Neutral_STEP_Display"], "MODEL_GO_M8010_V1", "B6808 neutral is included; no separate mass."),
        component("J3_STATOR_EQ", "J3 GO-M8010 stator/core equivalent", GO_STATOR, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link3", ["J3_Motor_Stator_STEP_Display"], "MODEL_GO_M8010_V1", "Includes the frozen complete-motor wiring boundary; no extra motor wire mass."),
        component("J4_STATOR_EQ", "J4 GO-M8010 stator/core equivalent", GO_STATOR, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link3", ["J4_Motor_Stator_STEP_Display"], "MODEL_GO_M8010_V1", "Includes the frozen complete-motor wiring boundary; no extra motor wire mass."),
        component("J4_OUTPUT_EQ", "J4 GO-M8010 output + neutral equivalent", GO_OUTPUT, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link4", ["J4_Output_Rotor_STEP_Display", "J4_B6808_Bearing_Neutral_STEP_Display"], "MODEL_GO_M8010_V1", "B6808 neutral is included; no separate mass."),
        component("J5_STATOR_EQ", "J5 GO-M8010 stator/core equivalent", GO_STATOR, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link4", ["J5_Motor_Stator_STEP_Display"], "MODEL_GO_M8010_V1", "Includes the frozen complete-motor wiring boundary; no extra motor wire mass."),
        component("J5_OUTPUT_EQ", "J5 GO-M8010 output + neutral equivalent", GO_OUTPUT, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link5", ["J5_Output_Rotor_STEP_Display", "J5_B6808_Bearing_Neutral_STEP_Display"], "MODEL_GO_M8010_V1", "B6808 neutral is included; no separate mass."),
        component("J6_DM_STATOR_EQ", "DM-G6220 stator-side equivalent", DM_STATOR, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link5", ["J6_DM_G6220_Stator_STEP_Display"], "MODEL_DM_G6220_V1", "Engineering allocation from the measured 0.500 kg complete motor."),
        component("J6_DM_OUTPUT_EQ", "DM-G6220 output/rotor-side equivalent", DM_OUTPUT, "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL", "link6", ["J6_DM_G6220_Output_Rotor_STEP_Display"], "MODEL_DM_G6220_V1", "Engineering allocation from the measured 0.500 kg complete motor."),
    ]
    for entry in motor_components:
        link_components[entry["ledger_link"]].append(entry)

    direct_components = [
        component("UPPER_ARM_PRINT_MEASURED", "J2-J3 upper-arm printed structure", UPPER, "MEASURED", "link2", ["UpperArm_A_SleeveSide_PrintPart", "UpperArm_B_Distal_PrintPart"], M_UPPER, "Direct scale measurement; the two print parts remain one unsplit measured group."),
        component("FOREARM_PRINT_MEASURED", "J3-J4 forearm print", FOREARM, "MEASURED", "link3", ["Forearm_v3_HighDetail_Display"], M_FOREARM, "Direct scale measurement without J3/J4 motors."),
        component("WRIST_PRELINK_PRINT_MEASURED", "J4-J5 printed connector", WRIST, "MEASURED", "link4", ["Wrist_Prelink_v1_HighDetail_Display"], M_WRIST, "Direct scale measurement without J4/J5 motors."),
    ]
    for entry in direct_components:
        link_components[entry["ledger_link"]].append(entry)

    residual_components = [
        component(R_A, "Compound-A remainder: J5-to-DM adapter, fasteners and local unweighed assembly remainder", RESIDUAL_A, "DERIVED_FROM_MEASURED_TOTAL", "link5", list(RESIDUAL_A_MEMBERS), R_A, "Nominal closure remainder 0.855-0.500-0.296. CAD named members are all link5; unmodeled local wiring/small hardware remains a lumped boundary.", non_cad_scope=["local assembly wiring not already included with a motor", "other unmodeled small hardware within confirmed Compound A boundary"]),
        component(R_B, "Compound-B remainder: J5 interface fasteners and local unweighed assembly remainder", RESIDUAL_B, "DERIVED_FROM_MEASURED_TOTAL", "link4", list(RESIDUAL_B_MEMBERS), R_B, "Nominal closure remainder 1.441-0.855-0.540. All named CAD members are link4.", non_cad_scope=["local assembly wiring not already included with a motor", "other unmodeled small hardware introduced at the J5 boundary"]),
        component(R_C, "Compound-C remainder: forearm fasteners and local unweighed assembly remainder", RESIDUAL_C, "DERIVED_FROM_MEASURED_TOTAL", "link3", list(RESIDUAL_C_MEMBERS), R_C, "Nominal closure remainder 2.220-0.135-0.540-0.089-1.441. All named CAD members are link3.", non_cad_scope=["local forearm wiring not already included with a motor", "other unmodeled small hardware within the confirmed Compound C boundary"]),
    ]
    for entry in residual_components:
        link_components[entry["ledger_link"]].append(entry)

    gripper_rows = [
        row
        for row in authority["visual_rows"]
        if row["owner"] == "gripper" or row["owner"].startswith("gripper_internal:")
    ]
    gripper_members = sorted(row["token"] for row in gripper_rows)
    gripper_owners = sorted({row["owner"] for row in gripper_rows})
    require(len(gripper_members) == 37, f"unexpected complete gripper visual member count: {len(gripper_members)}")
    require("Gemini_Pro_Camera_STEP_Display" in gripper_members, "Gemini missing from measured complete-end boundary")
    require("J6_Gripper_Connector_STEP_Display" in gripper_members, "gripper connector missing from measured boundary")
    require(
        all(f"DM_G6220_Connector_M4x14_Screw_{i:02d}" in gripper_members for i in range(1, 7)),
        "DM-to-gripper connector screws missing from 0.296 kg boundary",
    )
    gripper_component = component(
        "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP",
        "Complete gripper + connector + Gemini Pro accounting roll-up",
        GRIPPER,
        "MEASURED",
        "gripper",
        gripper_members,
        M_GRIPPER,
        "Direct complete-end measurement. Static gripper and six internal moving rigid roles retain their true CAD owners; this is main-chain bookkeeping only and is not a future internal-link inertial allocation.",
        cad_owners=gripper_owners,
        mapping_policy="ACCOUNTING_ROLLUP_ONLY_NOT_CAD_REPARENTING",
    )
    link_components["gripper"].append(gripper_component)

    # User-authorized V1 boundary policy: link2 has no added residual.  These
    # physical CAD tokens are not zero-mass; they are explicitly included with
    # the frozen link2 closure and therefore must not receive a second additive
    # mass entry.
    non_additive_boundary_members = [
        {
            "group_id": "LINK2_USER_CONFIRMED_NO_RESIDUAL_BOUNDARY",
            "ledger_link": "link2",
            "cad_members": list(LINK2_POLICY_MEMBERS),
            "cad_owners": ["link2"],
            "mass_status": "INCLUDED_NO_SEPARATE_MASS",
            "nominal_mass_kg": None,
            "accounting_role": "NON_ADDITIVE_BOUNDARY_MEMBER",
            "authority": "USER_CONFIRMED_V15_15_V1_NO_ADDITIONAL_J2_REGION_RESIDUAL",
            "notes": "These fasteners have physical mass, but V1 assigns no independent additive mass or residual to them. They are covered by the user-frozen link2 closure; this is not a claim of zero physical mass.",
        }
    ]

    for link, entries in link_components.items():
        for entry in entries:
            for token in entry["cad_members"]:
                if link == "gripper" and v[token]["owner"].startswith("gripper_internal:"):
                    continue
                exact_owner(authority, token, link)
    for token in LINK2_POLICY_MEMBERS:
        exact_owner(authority, token, "link2")

    # The three residual CAD sets must be unique, disjoint and on one side of
    # their respective joint boundaries.
    for token in RESIDUAL_A_MEMBERS:
        exact_owner(authority, token, "link5")
    for token in RESIDUAL_B_MEMBERS:
        exact_owner(authority, token, "link4")
    for token in RESIDUAL_C_MEMBERS:
        exact_owner(authority, token, "link3")
    require(
        not (set(RESIDUAL_A_MEMBERS) & set(RESIDUAL_B_MEMBERS)
             or set(RESIDUAL_A_MEMBERS) & set(RESIDUAL_C_MEMBERS)
             or set(RESIDUAL_B_MEMBERS) & set(RESIDUAL_C_MEMBERS)),
        "residual CAD member sets overlap",
    )

    expected_scope_tokens = {
        row["token"]
        for row in authority["visual_rows"]
        if row["owner"] in SCOPE_LINKS or row["owner"].startswith("gripper_internal:")
    }
    coverage = Counter(
        token
        for entries in link_components.values()
        for entry in entries
        for token in entry["cad_members"]
    )
    coverage.update(
        token
        for group in non_additive_boundary_members
        for token in group["cad_members"]
    )
    require(len(expected_scope_tokens) == 96, f"unexpected scoped visual token count: {len(expected_scope_tokens)}")
    require(set(coverage) == expected_scope_tokens, "scoped visual CAD coverage is not exact")
    require(all(count == 1 for count in coverage.values()), "a scoped visual CAD token is covered more than once")

    ledger: dict[str, dict] = {}
    for link in SCOPE_LINKS:
        entries = link_components[link]
        by_status = {
            status: sum(
                (D(str(entry["nominal_mass_kg"])) for entry in entries if entry["mass_status"] == status),
                D("0"),
            )
            for status in (
                "MEASURED",
                "DERIVED_FROM_MEASURED_TOTAL",
                "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            )
        }
        nominal = sum(by_status.values(), D("0"))
        ledger[link] = {
            "nominal_mass_kg": num(nominal),
            "aggregate_status": "MIXED_STATUS_LEDGER_SUM",
            "subtotal_by_status_kg": {key: num(value) for key, value in by_status.items()},
            "uncertainty_kg": None,
            "uncertainty_reason": "Motor split uncertainty is unquantified and residual uncertainties share correlated compound measurements; no unsupported aggregate uncertainty is synthesized.",
            "is_final_v1_nominal_mass": True,
            "components": entries,
        }

    expected_masses = {
        "link2": D("0.7615"),
        "link3": D("1.090"),
        "link4": D("0.675"),
        "link5": D("0.504"),
        "link6": D("0.125"),
        "gripper": D("0.296"),
    }
    for link, expected in expected_masses.items():
        require(D(str(ledger[link]["nominal_mass_kg"])) == expected, f"{link} mass mismatch")

    return ledger, [entry for entries in link_components.values() for entry in entries], non_additive_boundary_members, {
        "expected_scope_visual_token_count": len(expected_scope_tokens),
        "covered_scope_visual_token_count": len(coverage),
        "missing_tokens": sorted(expected_scope_tokens - set(coverage)),
        "extra_tokens": sorted(set(coverage) - expected_scope_tokens),
        "duplicate_tokens": sorted(token for token, count in coverage.items() if count != 1),
        "gripper_rollup_cad_owner_set": gripper_owners,
        "gripper_rollup_member_count": len(gripper_members),
        "gripper_internal_roles_reparented": False,
        "pass": True,
    }


def build_contract(repo: Path, authority: dict, protected: dict[str, str], git_scope: dict) -> dict:
    require(GO_OUTPUT + GO_STATOR == GO_TOTAL, "GO split closure failed")
    require(DM_STATOR + DM_OUTPUT == DM_TOTAL, "DM split closure failed")
    require(COMPOUND_A - DM_TOTAL - GRIPPER == RESIDUAL_A, "residual A equation failed")
    require(COMPOUND_B - COMPOUND_A - GO_TOTAL == RESIDUAL_B, "residual B equation failed")
    require(
        COMPOUND_C - FOREARM - GO_TOTAL - WRIST - COMPOUND_B == RESIDUAL_C,
        "residual C equation failed",
    )

    ledger, components, non_additive_boundary_members, cad_validation = build_ledger(authority)
    link_total = sum((D(str(ledger[link]["nominal_mass_kg"])) for link in SCOPE_LINKS), D("0"))
    alternative_total = GO_OUTPUT + GO_OUTPUT + UPPER + GO_TOTAL + COMPOUND_C
    require(abs(link_total - D("3.4515")) < TOLERANCE, "link2-to-gripper closure failed")
    require(abs(alternative_total - D("3.4515")) < TOLERANCE, "alternative closure failed")
    require(link_total == alternative_total, "two V1 total paths differ")

    direct_measurements = {
        "upper_arm_print": {"measurement_id": M_UPPER, "mass_kg": num(UPPER), "mass_status": "MEASURED", "scale_uncertainty_abs_kg": 0.005, "boundary": "UpperArm_A + UpperArm_B; no GO motor"},
        "forearm_print": {"measurement_id": M_FOREARM, "mass_kg": num(FOREARM), "mass_status": "MEASURED", "scale_uncertainty_abs_kg": 0.005, "boundary": "Forearm_v3_HighDetail_Display; no J3/J4 motor"},
        "wrist_prelink_print": {"measurement_id": M_WRIST, "mass_kg": num(WRIST), "mass_status": "MEASURED", "scale_uncertainty_abs_kg": 0.005, "boundary": "Wrist_Prelink_v1_HighDetail_Display; no J4/J5 motor"},
        "complete_gripper_connector_camera": {"measurement_id": M_GRIPPER, "mass_kg": num(GRIPPER), "mass_status": "MEASURED", "scale_uncertainty_abs_kg": 0.005, "boundary": "connector + complete gripper + Gemini Pro + associated connector hardware"},
    }
    motor_models = {
        "GO_M8010_6_V1": {
            "measurement_id": M_GO,
            "measured_total_kg": num(GO_TOTAL),
            "measured_total_status": "MEASURED_TOTAL",
            "scale_uncertainty_abs_kg": 0.005,
            "output_equivalent_kg": num(GO_OUTPUT),
            "stator_core_equivalent_kg": num(GO_STATOR),
            "split_status": "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            "split_uncertainty_kg": None,
            "split_uncertainty_status": "UNQUANTIFIED_ENGINEERING_MODEL",
            "neutral_b6808_policy": "included inside output equivalent; no additional mass",
            "attached_wire_policy": "included inside the measured complete-motor/stator-core model; must not be independently added again",
        },
        "DM_G6220_V1": {
            "measurement_id": M_DM,
            "measured_total_kg": num(DM_TOTAL),
            "measured_total_status": "MEASURED_TOTAL",
            "scale_uncertainty_abs_kg": 0.005,
            "stator_side_kg": num(DM_STATOR),
            "output_rotor_side_kg": num(DM_OUTPUT),
            "split_status": "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            "split_uncertainty_kg": None,
            "split_uncertainty_status": "UNQUANTIFIED_ENGINEERING_MODEL_NOT_VENDOR_OFFICIAL_SPLIT",
        },
    }
    compound_measurements = {
        "compound_A": {"measurement_id": M_A, "mass_kg": num(COMPOUND_A), "mass_status": "MEASURED_TOTAL", "scale_uncertainty_abs_kg": 0.005, "accounting_role": "NON_ADDITIVE_CONSTRAINT", "confirmed_contains": [M_DM, M_GRIPPER, R_A]},
        "compound_B": {"measurement_id": M_B, "mass_kg": num(COMPOUND_B), "mass_status": "MEASURED_TOTAL", "scale_uncertainty_abs_kg": 0.005, "accounting_role": "NON_ADDITIVE_CONSTRAINT", "confirmed_contains": [M_A, M_GO, R_B]},
        "compound_C": {"measurement_id": M_C, "mass_kg": num(COMPOUND_C), "mass_status": "MEASURED_TOTAL", "scale_uncertainty_abs_kg": 0.005, "accounting_role": "NON_ADDITIVE_CONSTRAINT", "confirmed_contains": [M_FOREARM, M_GO, M_WRIST, M_B, R_C], "J3_motor_included": False},
    }
    residuals = {
        "residual_A": {"residual_id": R_A, "nominal_mass_kg": num(RESIDUAL_A), "mass_status": "DERIVED_FROM_MEASURED_TOTAL", "owner_link": "link5", "equation": "0.855 - 0.500 - 0.296", "terms": [{"id": M_A, "sign": 1}, {"id": M_DM, "sign": -1}, {"id": M_GRIPPER, "sign": -1}], "conservative_closure_uncertainty_abs_kg": 0.015, "uncertainty_type": "CLOSURE_UNCERTAINTY_NOT_DYNAMICS_IDENTIFICATION_ERROR", "candidate_cad_members": list(RESIDUAL_A_MEMBERS), "cad_owner_validation": "PASS"},
        "residual_B": {"residual_id": R_B, "nominal_mass_kg": num(RESIDUAL_B), "mass_status": "DERIVED_FROM_MEASURED_TOTAL", "owner_link": "link4", "equation": "1.441 - 0.855 - 0.540", "terms": [{"id": M_B, "sign": 1}, {"id": M_A, "sign": -1}, {"id": M_GO, "sign": -1}], "conservative_closure_uncertainty_abs_kg": 0.015, "uncertainty_type": "CLOSURE_UNCERTAINTY_NOT_DYNAMICS_IDENTIFICATION_ERROR", "candidate_cad_members": list(RESIDUAL_B_MEMBERS), "cad_owner_validation": "PASS"},
        "residual_C": {"residual_id": R_C, "nominal_mass_kg": num(RESIDUAL_C), "mass_status": "DERIVED_FROM_MEASURED_TOTAL", "owner_link": "link3", "equation": "2.220 - 0.135 - 0.540 - 0.089 - 1.441", "terms": [{"id": M_C, "sign": 1}, {"id": M_FOREARM, "sign": -1}, {"id": M_GO, "sign": -1}, {"id": M_WRIST, "sign": -1}, {"id": M_B, "sign": -1}], "conservative_closure_uncertainty_abs_kg": 0.025, "uncertainty_type": "CLOSURE_UNCERTAINTY_NOT_DYNAMICS_IDENTIFICATION_ERROR", "uncertainty_interval_contains_zero": True, "candidate_cad_members": list(RESIDUAL_C_MEMBERS), "cad_owner_validation": "PASS"},
    }

    closure_specs = [
        ("GO_0_070_plus_0_470", GO_OUTPUT + GO_STATOR, GO_TOTAL),
        ("DM_0_375_plus_0_125", DM_STATOR + DM_OUTPUT, DM_TOTAL),
        ("compound_A_0_500_plus_0_296_plus_0_059", DM_TOTAL + GRIPPER + RESIDUAL_A, COMPOUND_A),
        ("compound_B_0_855_plus_0_540_plus_0_046", COMPOUND_A + GO_TOTAL + RESIDUAL_B, COMPOUND_B),
        ("compound_C_0_135_plus_0_540_plus_0_089_plus_1_441_plus_0_015", FOREARM + GO_TOTAL + WRIST + COMPOUND_B + RESIDUAL_C, COMPOUND_C),
        ("link2_to_gripper_sum", link_total, D("3.4515")),
        ("alternative_0_140_plus_0_5515_plus_0_540_plus_2_220", alternative_total, D("3.4515")),
        ("two_total_paths_equal", link_total, alternative_total),
    ]
    closure_checks = {
        name: {
            "actual_kg": num(actual),
            "expected_kg": num(expected),
            "absolute_error_kg": num(abs(actual - expected)),
            "tolerance_kg": num(TOLERANCE),
            "pass": abs(actual - expected) < TOLERANCE,
        }
        for name, actual, expected in closure_specs
    }
    require(all(item["pass"] for item in closure_checks.values()), "one or more closure checks failed")
    closure_checks["independence_note"] = {
        "statistically_independent": False,
        "closure_type": "ALGEBRAIC_RECONCILIATION_WITH_SHARED_INPUTS",
        "notes": "The two 3.4515 kg paths are required arithmetic cross-representations, not statistically independent experiments.",
    }

    additive_ids = [entry["component_id"] for entry in components]
    additive_id_counts = Counter(additive_ids)
    additive_ids_unique = all(count == 1 for count in additive_id_counts.values())
    require(additive_ids_unique, "duplicate additive component ID")
    component_by_id = {entry["component_id"]: entry for entry in components}

    parent_ids = {M_GO, M_DM, M_A, M_B, M_C}
    additive_source_ids = {
        entry["source_measurement_or_model_id"] for entry in components
    }
    parent_totals_added = bool(parent_ids.intersection(additive_source_ids))
    require(not parent_totals_added, "non-additive parent total entered link ledger")
    require(
        sum((D(str(entry["nominal_mass_kg"])) for entry in components), D("0")) == link_total,
        "additive leaf sum differs from link ledger total",
    )

    cad_token_counts = Counter(
        token for entry in components for token in entry["cad_members"]
    )
    residual_component_sets = {
        R_A: set(component_by_id[R_A]["cad_members"]),
        R_B: set(component_by_id[R_B]["cad_members"]),
        R_C: set(component_by_id[R_C]["cad_members"]),
    }
    residual_cad_sets_disjoint = not (
        residual_component_sets[R_A] & residual_component_sets[R_B]
        or residual_component_sets[R_A] & residual_component_sets[R_C]
        or residual_component_sets[R_B] & residual_component_sets[R_C]
    )
    require(residual_cad_sets_disjoint, "residual CAD sets overlap in additive ledger")

    def component_mass(component_id: str) -> Decimal:
        require(component_id in component_by_id, f"missing additive component: {component_id}")
        return D(str(component_by_id[component_id]["nominal_mass_kg"]))

    go_instance_components = {
        "J3": ("J3_OUTPUT_EQ", "J3_STATOR_EQ"),
        "J4": ("J4_OUTPUT_EQ", "J4_STATOR_EQ"),
        "J5": ("J5_OUTPUT_EQ", "J5_STATOR_EQ"),
    }
    go_instance_closure = {
        motor: component_mass(output_id) + component_mass(stator_id) == GO_TOTAL
        for motor, (output_id, stator_id) in go_instance_components.items()
    }
    require(all(go_instance_closure.values()), "a complete GO motor allocation does not close to 0.540 kg")
    dm_allocation_closes = (
        component_mass("J6_DM_STATOR_EQ") + component_mass("J6_DM_OUTPUT_EQ") == DM_TOTAL
    )
    require(dm_allocation_closes, "DM allocation does not close to 0.500 kg")

    neutral_tokens = {
        "derived:J2A_GO_M8010_neutral",
        "derived:J2B_GO_M8010_neutral",
        "J3_B6808_Bearing_Neutral_STEP_Display",
        "J4_B6808_Bearing_Neutral_STEP_Display",
        "J5_B6808_Bearing_Neutral_STEP_Display",
    }
    neutral_components = [
        entry for entry in components if neutral_tokens.intersection(entry["cad_members"])
    ]
    neutral_separate_components = [
        entry
        for entry in neutral_components
        if set(entry["cad_members"]).issubset(neutral_tokens)
    ]
    neutral_tokens_exactly_once = all(cad_token_counts[token] == 1 for token in neutral_tokens)
    require(neutral_tokens_exactly_once, "a B6808/neutral token is missing or repeated")
    require(not neutral_separate_components, "B6808/neutral received a separate additive mass")

    motor_attached_wire_components = [
        entry
        for entry in components
        if "MOTOR_WIRE" in entry["component_id"] or "MOTOR_ATTACHED_WIRING" in entry["component_id"]
    ]
    require(not motor_attached_wire_components, "motor-attached wiring received a separate additive mass")

    complete_motor_total_component_count = sum(
        1 for entry in components if entry["source_measurement_or_model_id"] in {M_GO, M_DM}
    )
    require(complete_motor_total_component_count == 0, "complete motor total was added on top of split allocations")

    gripper_measurement_count = sum(
        1
        for entry in components
        if entry["source_measurement_or_model_id"] == M_GRIPPER
        and entry["mass_status"] == "MEASURED"
    )
    component_count_checks = {
        "gemini_pro_count": cad_token_counts["Gemini_Pro_Camera_STEP_Display"],
        "gripper_connector_count": cad_token_counts["J6_Gripper_Connector_STEP_Display"],
        "complete_gripper_measurement_count": gripper_measurement_count,
        "dm_g6220_count": int(dm_allocation_closes),
        "J5_motor_count": int(go_instance_closure["J5"]),
        "J4_motor_count": int(go_instance_closure["J4"]),
        "J2A_output_count": additive_id_counts["J2A_OUTPUT_EQ"],
        "J2B_output_count": additive_id_counts["J2B_OUTPUT_EQ"],
        "adapter_count": cad_token_counts["J5_to_Damiao_J6_Adapter_STL_Display"],
    }
    require(
        all(count == 1 for count in component_count_checks.values()),
        f"double-count occurrence check failed: {component_count_checks}",
    )

    double_count_gate_values = {
        "additive_component_ids_unique": additive_ids_unique,
        "parent_compound_totals_added_to_links": parent_totals_added,
        "complete_motor_totals_added_on_top_of_splits": complete_motor_total_component_count > 0,
        "residual_cad_sets_disjoint": residual_cad_sets_disjoint,
        "neutral_b6808_separate_mass_count": len(neutral_separate_components),
        "neutral_b6808_tokens_covered_once": neutral_tokens_exactly_once,
        "motor_attached_wiring_separate_mass_count": len(motor_attached_wire_components),
        "scoped_visual_tokens_covered_exactly_once": (
            cad_validation["expected_scope_visual_token_count"]
            == cad_validation["covered_scope_visual_token_count"]
            and not cad_validation["missing_tokens"]
            and not cad_validation["extra_tokens"]
            and not cad_validation["duplicate_tokens"]
        ),
    }
    double_count_pass = (
        double_count_gate_values["additive_component_ids_unique"]
        and not double_count_gate_values["parent_compound_totals_added_to_links"]
        and not double_count_gate_values["complete_motor_totals_added_on_top_of_splits"]
        and double_count_gate_values["residual_cad_sets_disjoint"]
        and double_count_gate_values["neutral_b6808_separate_mass_count"] == 0
        and double_count_gate_values["neutral_b6808_tokens_covered_once"]
        and double_count_gate_values["motor_attached_wiring_separate_mass_count"] == 0
        and double_count_gate_values["scoped_visual_tokens_covered_exactly_once"]
        and all(count == 1 for count in component_count_checks.values())
    )
    require(double_count_pass, "computed double-count gate failed")

    protected_entries = [
        {
            "path": path,
            "sha256_before": digest,
            "sha256_after": digest,
            "hash_mode": PROTECTED_INPUTS[path]["hash_mode"],
            "unchanged": True,
            "matches_review_baseline": digest == PROTECTED_INPUTS[path]["sha256"],
        }
        for path, digest in protected.items()
    ]

    contract = {
        "schema": SCHEMA,
        "revision": REVISION,
        "source_branch": SOURCE_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "target_branch": TARGET_BRANCH,
        "measurement_authority": "USER_PHYSICAL_SCALE",
        "scope": "LINK2_TO_GRIPPER_MASS_ONLY_NO_COM_NO_INERTIA",
        "units": {"mass": "kg"},
        "scale_uncertainty": {"direct_single_measurement_abs_kg": 0.005, "authority": "USER_SCALE_MAX_SINGLE_READING_ERROR"},
        "status_definitions": {
            "MEASURED": "Direct leaf assembly measurement with a confirmed bookkeeping boundary.",
            "MEASURED_TOTAL": "Direct complete-motor or nested compound total; non-additive when its split/children are used.",
            "DERIVED_FROM_MEASURED_TOTAL": "Nominal residual from confirmed compound totals; not an independent scale reading.",
            "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL": "Frozen V1 allocation of a measured complete motor; not a disassembled or vendor-official measurement.",
            "INCLUDED_NO_SEPARATE_MASS": "Physical CAD members covered by an explicit V1 boundary policy; not zero physical mass and not independently added.",
        },
        "git_scope_guard": git_scope,
        "frozen_baseline_guard": {
            "protected_inputs": protected_entries,
            "all_protected_inputs_unchanged": True,
            "all_protected_inputs_match_review_baseline": True,
            "frozen_joint_canonical_sha256": authority["canonical_by_version"],
            "all_frozen_joint_elements_unchanged": True,
            "kinematics_tf_collision_control_files_modified": False,
            "com_inertia_gravity_written": False,
        },
        "topology": {
            "tree": "world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper",
            "topology_modified": False,
            "scope_links": list(SCOPE_LINKS),
        },
        "motor_mass_models": motor_models,
        "direct_measurements": direct_measurements,
        "compound_measurements": compound_measurements,
        "measurement_dependency_graph": {
            "confirmed_contains_edges": [
                {"parent": M_A, "children": [M_DM, M_GRIPPER, R_A]},
                {"parent": M_B, "children": [M_A, M_GO, R_B]},
                {"parent": M_C, "children": [M_FOREARM, M_GO, M_WRIST, M_B, R_C]},
            ],
            "is_dag": True,
            "parent_totals_are_non_additive": True,
        },
        "residuals": residuals,
        "component_to_link_mapping": {
            "additive_components": components,
            "non_additive_boundary_members": non_additive_boundary_members,
        },
        "cad_membership_validation": cad_validation,
        "link_mass_ledger": ledger,
        "total_link2_to_gripper_nominal_mass_kg": num(link_total),
        "total_aggregate_status": "MIXED_STATUS_LEDGER_SUM",
        "total_uncertainty_kg": None,
        "total_uncertainty_reason": "Engineering split uncertainties are unquantified and residual closure uncertainties are correlated; no unsupported total uncertainty is synthesized.",
        "closure_checks": closure_checks,
        "uncertainty_model": {
            "residual_A_conservative_abs_kg": 0.015,
            "residual_B_conservative_abs_kg": 0.015,
            "residual_C_conservative_abs_kg": 0.025,
            "residual_uncertainty_type": "CLOSURE_UNCERTAINTY_NOT_DYNAMICS_IDENTIFICATION_ERROR",
            "residuals_are_correlated": True,
            "rss_or_direct_sum_allowed": False,
            "motor_internal_split_uncertainty": "UNQUANTIFIED_ENGINEERING_MODEL",
        },
        "wire_boundary_policy": {
            "motor_attached_wiring_already_in_complete_motor_measurement": True,
            "independent_motor_wire_mass_added": False,
            "residual_wire_scope": "Only local assembly wiring not already included in a measured complete motor; no wire mass is guessed or separately resolved.",
        },
        "double_count_checks": {
            **double_count_gate_values,
            **component_count_checks,
            "complete_motor_total_component_count": complete_motor_total_component_count,
            "double_count_detected": not double_count_pass,
            "pass": double_count_pass,
        },
        "unresolved_items": [],
        "nonblocking_model_limitations": [
            "The 12 link2 fastener tokens are closed only by the user's V1 no-additional-J2-residual policy; they are not claimed to have zero physical mass.",
            "The 0.296 kg gripper value is a main-chain accounting roll-up across static gripper and six internal moving CAD owner roles; it is not an internal inertial allocation.",
            "Motor internal split uncertainty is unquantified.",
            "Residual closure uncertainties are correlated; residual_C nominal 0.015 kg has a conservative +/-0.025 kg interval that crosses zero.",
            "The two 3.4515 kg paths are algebraically linked and are not statistically independent measurements.",
        ],
        "prohibited_outputs": {
            "COM": False,
            "inertia_tensor": False,
            "URDF_inertial": False,
            "MuJoCo_inertial": False,
            "gravity": False,
            "joint_damping": False,
            "joint_friction": False,
            "motor_armature": False,
            "motor_torque_model": False,
        },
        "validation": {
            "cad_membership": "PASS",
            "residual_A_owner": "link5",
            "residual_B_owner": "link4",
            "residual_C_owner": "link3",
            "all_closures": "PASS",
            "double_count": "PASS",
            "unresolved_items_empty": True,
            "forbidden_files_modified": False,
            "all_assertions_pass": True,
        },
        "final_status": "V15.15 MASS_LEDGER_V1 = PASS",
    }
    return contract


def render_markdown(data: dict) -> str:
    lines = [
        "# V15.15 实测质量账本 V1",
        "",
        f"- 基线：`{data['source_branch']}` @ `{data['source_commit']}`",
        f"- 工作分支：`{data['target_branch']}`",
        f"- 最终状态：**{data['final_status']}**",
        "- 范围：仅 `link2` → `gripper` 质量账本；不进入 COM/inertia/gravity/dynamics。",
        "",
        "## 质量状态",
        "",
        "- 完整 GO-M8010 0.540 kg 和 DM-G6220 0.500 kg：`MEASURED_TOTAL`。",
        "- GO 0.070/0.470 kg 与 DM 0.375/0.125 kg：`ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL`，不是拆机实测或厂家官方分拆。",
        "- 上臂、小臂、J4-J5 打印件和完整夹爪末端：`MEASURED`。",
        "- residual A/B/C：`DERIVED_FROM_MEASURED_TOTAL`。",
        "",
        "## 最终 Link 名义质量",
        "",
        "| Link | measured (kg) | derived residual (kg) | engineering split (kg) | V1 total (kg) |",
        "|---|---:|---:|---:|---:|",
    ]
    for link, entry in data["link_mass_ledger"].items():
        s = entry["subtotal_by_status_kg"]
        lines.append(
            f"| `{link}` | {s['MEASURED']:.4f} | {s['DERIVED_FROM_MEASURED_TOTAL']:.4f} | "
            f"{s['ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL']:.4f} | **{entry['nominal_mass_kg']:.4f}** |"
        )
    lines.extend([
        "",
        f"`link2` → `gripper` 总质量：**{data['total_link2_to_gripper_nominal_mass_kg']:.4f} kg**。",
        "",
        "## 残差与 CAD 归属",
        "",
        "| Residual | 名义质量 | 保守闭合不确定度 | 最终归属 | CAD |",
        "|---|---:|---:|---|---|",
    ])
    for key in ("residual_A", "residual_B", "residual_C"):
        r = data["residuals"][key]
        lines.append(
            f"| {key} | {r['nominal_mass_kg']:.3f} kg | ±{r['conservative_closure_uncertainty_abs_kg']:.3f} kg | "
            f"`{r['owner_link']}` | {r['cad_owner_validation']} |"
        )
    lines.extend([
        "",
        "不确定度是组合测量闭合不确定度，不是动力学辨识误差。`residual_C = 0.015 ± 0.025 kg` 的保守区间跨越零；它仅是冻结 V1 的名义闭合项。",
        "",
        "## 闭合检查",
        "",
        "| Check | Actual (kg) | Expected (kg) | Result |",
        "|---|---:|---:|---|",
    ])
    for key, check in data["closure_checks"].items():
        if key == "independence_note":
            continue
        lines.append(
            f"| `{key}` | {check['actual_kg']:.4f} | {check['expected_kg']:.4f} | "
            f"{'PASS' if check['pass'] else 'FAIL'} |"
        )
    lines.extend([
        "",
        "两条 3.4515 kg 计算路径共享同一组实测输入，属于代数协调验算，**不是统计独立实验**。",
        "",
        "## CAD 边界政策",
        "",
        "- residual A 的 adapter + 3 颗螺丝全部唯一属于 `link5`。",
        "- residual B 的 12 颗 J4/J5 接口紧固件全部唯一属于 `link4`。",
        "- residual C 的 12 颗小臂紧固件全部唯一属于 `link3`。",
        "- link2 的 12 个紧固件依据用户“V1 不再增加 J2 附近 residual”政策被覆盖；它们不是零物理质量，但不得再独立加重。",
        "- 0.296 kg 是主链账本 roll-up；17 个夹爪内部活动件保留其 `gripper_internal:*` CAD owner，未被重归属。",
        "",
        "## Double-count 与禁止项",
        "",
        "- Double-count：**PASS**。组合总量和完整电机总量均是 non-additive constraints/bases，Link 仅汇总 leaf/split/residual。",
        "- B6808 neutral、电机自带线材、Gemini Pro、夹爪连接件、DM/J4/J5 电机均未重复加重。",
        "- `unresolved_items = []`。模型局限在 JSON `nonblocking_model_limitations` 中保留，不伪装成已识别的物理参数。",
        "- J1~J6 几何、TF、tcp/camera、mesh/collision、MoveIt、ros2_control、controller、trajectory bridge：未修改。",
        "- COM、inertia、gravity、damping、friction、armature、torque model：未写入。",
        "",
        f"**{data['final_status']}**",
        "",
    ])
    return "\n".join(lines)


def json_payload(data: dict) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def write_text(path: Path, payload: str) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(payload)


def print_summary(data: dict, mode: str) -> None:
    summary = {
        "mode": mode,
        "branch": data["target_branch"],
        "source_commit": data["source_commit"],
        "cad_membership": data["validation"]["cad_membership"],
        "residual_A_owner": data["validation"]["residual_A_owner"],
        "residual_B_owner": data["validation"]["residual_B_owner"],
        "residual_C_owner": data["validation"]["residual_C_owner"],
        "link_masses_kg": {
            link: entry["nominal_mass_kg"] for link, entry in data["link_mass_ledger"].items()
        },
        "total_kg": data["total_link2_to_gripper_nominal_mass_kg"],
        "compound_A": data["closure_checks"]["compound_A_0_500_plus_0_296_plus_0_059"]["pass"],
        "compound_B": data["closure_checks"]["compound_B_0_855_plus_0_540_plus_0_046"]["pass"],
        "compound_C": data["closure_checks"]["compound_C_0_135_plus_0_540_plus_0_089_plus_1_441_plus_0_015"]["pass"],
        "double_count": data["double_count_checks"]["pass"],
        "unresolved_items": data["unresolved_items"],
        "final_status": data["final_status"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--repo-root")
    args = parser.parse_args()
    try:
        repo = find_repo_root(args.repo_root)
        before = snapshot_protected_inputs(repo)
        git_scope = audit_git_scope(repo)
        authority = load_authorities(repo)
        data = build_contract(repo, authority, before, git_scope)
        expected_json = json_payload(data)
        expected_md = render_markdown(data)
        require(snapshot_protected_inputs(repo) == before, "protected inputs changed during validation")
        json_path = repo / OUTPUT_JSON
        md_path = repo / OUTPUT_MD
        if args.write:
            write_text(json_path, expected_json)
            write_text(md_path, expected_md)
            run_mode = "write"
        else:
            require(json_path.is_file(), f"missing {OUTPUT_JSON}")
            require(md_path.is_file(), f"missing {OUTPUT_MD}")
            require(json_path.read_text(encoding="utf-8") == expected_json, f"stale {OUTPUT_JSON}")
            require(md_path.read_text(encoding="utf-8") == expected_md, f"stale {OUTPUT_MD}")
            run_mode = "check"
        require(snapshot_protected_inputs(repo) == before, "protected inputs changed while handling outputs")
        audit_git_scope(repo)
        print_summary(data, run_mode)
        return 0
    except Exception as exc:
        print(f"V15.15 MASS_LEDGER_V1 FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
