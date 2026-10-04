#!/usr/bin/env python3
"""Build and validate the V15.15 measured-mass-to-rigid-link contract.

This tool is deliberately limited to a mass ledger.  It reads the frozen
V15.13 membership/manifest and FreeCAD object inventory, verifies the frozen
V15.13/V15.14 geometry hashes, then writes only:

* V15_15_实测质量映射契约.json
* V15_15_实测质量映射说明.md

It never writes URDF/MJCF, mesh, TF, MoveIt, ros2_control, controller, COM or
inertia data.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
import zipfile
from collections import Counter, defaultdict
from decimal import Decimal
from pathlib import Path
import xml.etree.ElementTree as ET


SCHEMA = "go-m8010-arm-v15.15-measured-mass-mapping/1.0"
REVISION = "V15.15-measured-mass-mapping-v1"
OUTPUT_JSON = "V15_15_实测质量映射契约.json"
OUTPUT_MD = "V15_15_实测质量映射说明.md"
LINKS = (
    "base_link",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "gripper",
)
FROZEN_JOINTS = (
    "J1",
    "J2",
    "J3",
    "J4",
    "J5",
    "J6",
    "J6_to_gripper",
    "gripper_to_tcp_nominal",
    "gripper_to_camera_link",
    "camera_link_to_sim_camera_optical_frame",
)

MEAS_GO = "M_GO_M8010_SINGLE_WITH_WIRING_001"
MEAS_UPPER = "M_UPPER_ARM_PRINT_001"
MEAS_DISTAL = "M_DISTAL_ASSEMBLY_001"
MEAS_FOREARM = "M_FOREARM_ASSEMBLY_001"
MEAS_GRIPPER = "M_GRIPPER_CONNECTOR_001"

D = Decimal
GO_TOTAL = D("0.540")
GO_OUTPUT = D("0.070")
GO_STATOR = D("0.470")
UPPER_ARM = D("0.5515")

PROTECTED_INPUTS = {
    "rigid_links_v15_13/rigid_link_membership.csv":
        {"sha256": "9a93b5a4494a67584bfda6eb0067ef0e75928bfbae77fd6e411ba1e15e9d70f3", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "rigid_links_v15_13/rigid_link_manifest.json":
        {"sha256": "cacf017aaa73e18658869896aaa79cba40938bb18d2cb56d8759ab32c5bb3ef9", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd":
        {"sha256": "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0", "hash_mode": "RAW_BYTES"},
    "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd":
        {"sha256": "2ae05fab2a15c5c75bb05db8b456f149da20f79cedba2fc8d47f1624bcec34c7", "hash_mode": "RAW_BYTES"},
    "完整工程_V15_13_交付/ros2_ws/src/go_m8010_arm_description/urdf/"
    "go_m8010_arm_v15_13.urdf.xacro":
        {"sha256": "fec9863b45addbea15f66ff617e42d6c9a2c06170c240e8ce51361e2b3fe0fdf", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/"
    "go_m8010_arm_v15_14_description/urdf/go_m8010_arm_v15_14.urdf.xacro":
        {"sha256": "f9c15d366e27bef76eeafd44efcca0de96d2f4e944584fcc74c9764a198a914c", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "mujoco_kinematic_v1/go_m8010_arm_v15_13_kinematic.xml":
        {"sha256": "26f9e0efe10b7d208378d816540098625a34826069fdf7bbafc1db94cf073941", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/"
    "go_m8010_arm_v15_14_kinematic.xml":
        {"sha256": "f086d27c61aa3be02234356af6fd8fcf269a3e3b65bdeb08cd04435ba679f0b0", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "V15_13_自碰撞对矩阵契约.json":
        {"sha256": "b95d95f89d8792080c2ff50c55a4054be44fffe4d2916b49e382bde6600ef1d0", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/config/"
    "accepted_frozen_contract_v15_13.json":
        {"sha256": "a6434ddb448f749eeaefeab3313fb2692fb5c7535270da0946f5ba317b48b496", "hash_mode": "UTF8_LF_CANONICAL_BYTES"},
}

MOTOR_SPECS = (
    {
        "motor_id": "J1",
        "joint": "J1",
        "stator_owner": "base_link",
        "stator_token": "derived:J1_GO_M8010_stator",
        "output_owner": "link1",
        "output_token": "derived:J1_GO_M8010_output",
        "neutral_token": "derived:J1_GO_M8010_neutral",
    },
    {
        "motor_id": "J2A",
        "joint": "J2",
        "stator_owner": "link1",
        "stator_token": "derived:J2A_GO_M8010_stator",
        "output_owner": "link2",
        "output_token": "derived:J2A_GO_M8010_output",
        "neutral_token": "derived:J2A_GO_M8010_neutral",
    },
    {
        "motor_id": "J2B",
        "joint": "J2",
        "stator_owner": "link1",
        "stator_token": "derived:J2B_GO_M8010_stator",
        "output_owner": "link2",
        "output_token": "derived:J2B_GO_M8010_output",
        "neutral_token": "derived:J2B_GO_M8010_neutral",
    },
    {
        "motor_id": "J3",
        "joint": "J3",
        "stator_owner": "link3",
        "stator_token": "J3_Motor_Stator_STEP_Display",
        "output_owner": "link2",
        "output_token": "J3_Output_Rotor_STEP_Display",
        "neutral_token": "J3_B6808_Bearing_Neutral_STEP_Display",
    },
    {
        "motor_id": "J4",
        "joint": "J4",
        "stator_owner": "link3",
        "stator_token": "J4_Motor_Stator_STEP_Display",
        "output_owner": "link4",
        "output_token": "J4_Output_Rotor_STEP_Display",
        "neutral_token": "J4_B6808_Bearing_Neutral_STEP_Display",
    },
    {
        "motor_id": "J5",
        "joint": "J5",
        "stator_owner": "link4",
        "stator_token": "J5_Motor_Stator_STEP_Display",
        "output_owner": "link5",
        "output_token": "J5_Output_Rotor_STEP_Display",
        "neutral_token": "J5_B6808_Bearing_Neutral_STEP_Display",
    },
)

UPPER_ARM_MEMBERS = (
    "UpperArm_A_SleeveSide_PrintPart",
    "UpperArm_B_Distal_PrintPart",
)
UPPER_ARM_FASTENERS = tuple(
    f"UpperArm_M35_{side}_{index:02d}"
    for side in ("Bottom", "Top")
    for index in range(1, 4)
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def kg(value: Decimal) -> float:
    return float(value)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def protected_sha256(path: Path, hash_mode: str) -> str:
    if hash_mode == "RAW_BYTES":
        return sha256_file(path)
    require(hash_mode == "UTF8_LF_CANONICAL_BYTES", f"unknown protected hash mode: {hash_mode}")
    payload = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    payload.decode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def canonical_element(element: ET.Element) -> dict:
    return {
        "tag": element.tag,
        "attributes": dict(sorted(element.attrib.items())),
        "text": (element.text or "").strip(),
        "children": [canonical_element(child) for child in list(element)],
    }


def canonical_sha256(element: ET.Element) -> str:
    payload = json.dumps(
        canonical_element(element),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def find_repo_root(explicit: str | None) -> Path:
    candidates: list[Path] = []

    def is_repo_root(candidate: Path) -> bool:
        return (
            (candidate / ".git").exists()
            and (candidate / "rigid_links_v15_13/rigid_link_membership.csv").is_file()
            and (candidate / "rigid_links_v15_13/rigid_link_manifest.json").is_file()
        )

    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        require(is_repo_root(candidate), f"explicit --repo-root is not the V15.13 authority repository: {candidate}")
        return candidate

    candidates.append(Path(__file__).resolve().parents[1])
    for candidate in candidates:
        if is_repo_root(candidate):
            return candidate

    try:
        git_root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            cwd=Path(__file__).resolve().parent,
            text=True,
            encoding="utf-8",
            errors="strict",
            stderr=subprocess.DEVNULL,
        ).strip()
        candidates.append(Path(git_root).resolve())
    except (OSError, subprocess.CalledProcessError, UnicodeError):
        pass

    for candidate in candidates:
        if is_repo_root(candidate):
            return candidate
    raise RuntimeError("cannot locate repository root with V15.13 authority files")


def snapshot_protected_inputs(repo: Path) -> dict[str, str]:
    snapshot: dict[str, str] = {}
    for relative, baseline in PROTECTED_INPUTS.items():
        path = repo / relative
        require(path.is_file(), f"missing protected input: {relative}")
        actual = protected_sha256(path, baseline["hash_mode"])
        require(
            actual == baseline["sha256"],
            f"protected input drift: {relative}: expected {baseline['sha256']}, got {actual}",
        )
        snapshot[relative] = actual
    return snapshot


def load_fcstd_object_names(path: Path) -> set[str]:
    with zipfile.ZipFile(path, "r") as archive:
        document = ET.fromstring(archive.read("Document.xml"))
    return {
        element.attrib["name"]
        for element in document.iter()
        if element.tag.rsplit("}", 1)[-1] == "Object" and "name" in element.attrib
    }


def load_authorities(repo: Path) -> dict:
    membership_path = repo / "rigid_links_v15_13/rigid_link_membership.csv"
    manifest_path = repo / "rigid_links_v15_13/rigid_link_manifest.json"
    derived_fcstd = repo / "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"
    source_fcstd = repo / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
    accepted_path = repo / (
        "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/config/"
        "accepted_frozen_contract_v15_13.json"
    )

    with membership_path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    require(len(rows) == 165, f"unexpected membership row count: {len(rows)}")
    require(
        set(rows[0])
        == {"owner", "geometry_role", "token", "source_object", "derivation", "urdf_link_count"},
        "membership columns changed",
    )
    visual_rows = [row for row in rows if row["geometry_role"] == "visual"]
    require(len(visual_rows) == 135, f"unexpected visual membership count: {len(visual_rows)}")
    visual_tokens = [row["token"] for row in visual_rows]
    require(len(visual_tokens) == len(set(visual_tokens)), "visual membership tokens are not unique")
    require(
        all(row["urdf_link_count"] == "1" for row in visual_rows),
        "a visual token does not have exactly one URDF ownership count",
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    require(tuple(manifest["links"]) == LINKS, "manifest link order/topology changed")
    require(
        manifest["source"]["fcstd_sha256"].lower() == sha256_file(source_fcstd),
        "manifest source FCStd hash mismatch",
    )
    require(
        manifest["derived_fcstd"]["sha256"].lower() == sha256_file(derived_fcstd),
        "manifest derived FCStd hash mismatch",
    )

    expected_owner_tokens: set[tuple[str, str]] = set()
    for owner in LINKS:
        expected_owner_tokens.update(
            (owner, token) for token in manifest["links"][owner]["visual_members"]
        )
    for role, entry in manifest["gripper_internal_rigid_roles"].items():
        expected_owner_tokens.update(
            (f"gripper_internal:{role}", token) for token in entry["members"]
        )
    observed_owner_tokens = {(row["owner"], row["token"]) for row in visual_rows}
    require(
        observed_owner_tokens == expected_owner_tokens,
        "CSV visual ownership differs from rigid_link_manifest",
    )

    object_names = load_fcstd_object_names(derived_fcstd)
    missing_objects = sorted(
        {row["source_object"] for row in visual_rows} - object_names
    )
    require(not missing_objects, f"visual membership objects missing from FCStd: {missing_objects}")

    accepted = json.loads(accepted_path.read_text(encoding="utf-8"))
    v13_xacro = repo / (
        "完整工程_V15_13_交付/ros2_ws/src/go_m8010_arm_description/urdf/"
        "go_m8010_arm_v15_13.urdf.xacro"
    )
    v14_xacro = repo / (
        "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/"
        "go_m8010_arm_v15_14_description/urdf/go_m8010_arm_v15_14.urdf.xacro"
    )
    # The accepted contract records the historical Windows raw-byte Xacro
    # digest.  Preserve that review-controlled metadata, while the portable
    # full-file guard above uses UTF-8/LF canonical bytes and the checks below
    # use canonical XML joint elements.  This avoids weakening the guard or
    # making a clean Linux checkout fail solely because Git converted EOLs.
    require(
        accepted["accepted_source_xacro_sha256"]
        == "66e14959c9bf65abdcc9028d4f8008a803050865ae965fbf168b1cd08229e84b",
        "accepted V15.13 historical Xacro digest metadata drift",
    )
    canonical_by_version: dict[str, dict[str, str]] = {}
    for version, path in (("V15.13", v13_xacro), ("V15.14", v14_xacro)):
        root = ET.parse(path).getroot()
        joints = {joint.get("name"): joint for joint in root.findall("joint")}
        hashes = {}
        for name in FROZEN_JOINTS:
            require(name in joints, f"{version} missing frozen joint {name}")
            actual = canonical_sha256(joints[name])
            expected = accepted["frozen_joint_canonical_sha256"][name]
            require(
                actual == expected,
                f"{version} frozen joint drift: {name}: expected {expected}, got {actual}",
            )
            hashes[name] = actual
        canonical_by_version[version] = hashes

    expected_joint_topology = {
        "J1": ("base_link", "link1"),
        "J2": ("link1", "link2"),
        "J3": ("link2", "link3"),
        "J4": ("link3", "link4"),
        "J5": ("link4", "link5"),
        "J6": ("link5", "link6"),
    }
    topology_joints = {entry["name"]: entry for entry in manifest["main_topology"]["joints"]}
    for joint, (parent, child) in expected_joint_topology.items():
        entry = topology_joints[joint]
        require(
            (entry["parent_link"], entry["child_link"]) == (parent, child),
            f"manifest topology mismatch for {joint}",
        )
    require(
        topology_joints["J2"]["physical_actuators"] == ["J2A", "J2B"],
        "J2 physical actuator pair is not exactly J2A/J2B",
    )

    return {
        "rows": rows,
        "visual_rows": visual_rows,
        "visual_by_token": {row["token"]: row for row in visual_rows},
        "manifest": manifest,
        "object_names": object_names,
        "accepted": accepted,
        "canonical_by_version": canonical_by_version,
        "topology_joints": topology_joints,
    }


def mass_component(
    component_id: str,
    name: str,
    source: str,
    mass: Decimal | None,
    mass_status: str,
    owner_link: str,
    cad_members: list[str],
    measurement_id: str | None,
    notes: str,
    *,
    cad_owners: list[str] | None = None,
    included_in_other_measurement: list[str] | None = None,
    allocation_group_id: str | None = None,
    accounting_role: str = "ADDITIVE_LEDGER_COMPONENT",
    non_cad_inclusions: list[str] | None = None,
    owner_mapping_policy: str = "DIRECT_CAD_OWNER",
) -> dict:
    return {
        "component_id": component_id,
        "name": name,
        "source": source,
        "mass_kg": None if mass is None else kg(mass),
        "mass_status": mass_status,
        "owner_link": owner_link,
        "cad_owners": cad_owners or [owner_link],
        "cad_members": cad_members,
        "geometry_role": "visual",
        "measurement_id": measurement_id,
        "included_in_other_measurement": included_in_other_measurement or [],
        "allocation_group_id": allocation_group_id,
        "accounting_role": accounting_role,
        "non_cad_inclusions": non_cad_inclusions or [],
        "owner_mapping_policy": owner_mapping_policy,
        "notes": notes,
    }


def build_contract(repo: Path, authority: dict, protected: dict[str, str]) -> dict:
    visual_by_token = authority["visual_by_token"]
    topology_joints = authority["topology_joints"]

    require(GO_OUTPUT + GO_STATOR == GO_TOTAL, "GO V1 split does not sum to 0.540 kg")
    require(len(MOTOR_SPECS) == 6, "GO motor inventory is not exactly six motors")
    require(
        [item["motor_id"] for item in MOTOR_SPECS if item["joint"] == "J2"]
        == ["J2A", "J2B"],
        "J2 motor inventory is not exactly J2A/J2B",
    )

    ledger_components: dict[str, list[dict]] = {link: [] for link in LINKS}
    motor_mapping: dict[str, dict] = {}
    used_visual_tokens: set[str] = set()
    for spec in MOTOR_SPECS:
        for token, owner in (
            (spec["stator_token"], spec["stator_owner"]),
            (spec["output_token"], spec["output_owner"]),
            (spec["neutral_token"], spec["output_owner"]),
        ):
            require(token in visual_by_token, f"motor visual token missing: {token}")
            row = visual_by_token[token]
            require(row["owner"] == owner, f"motor owner mismatch: {token}: {row['owner']} != {owner}")
            require(row["geometry_role"] == "visual", f"non-visual motor token: {token}")
            require(row["source_object"] in authority["object_names"], f"motor source object missing: {token}")
        group_id = f"ALLOC_GO_M8010_{spec['motor_id']}"
        stator_component = mass_component(
            f"{group_id}_STATOR_CORE",
            f"{spec['motor_id']} GO-M8010 stator/core equivalent",
            "User physical scale measurement M_GO_M8010_SINGLE_WITH_WIRING_001 and frozen V1 allocation",
            GO_STATOR,
            "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            spec["stator_owner"],
            [spec["stator_token"]],
            MEAS_GO,
            "Includes the cylindrical stator/core side, remaining internal equivalent mass and current attached motor wiring; no separate wiring mass is allowed.",
            included_in_other_measurement=[MEAS_GO],
            allocation_group_id=group_id,
            non_cad_inclusions=["current attached motor wiring"],
        )
        output_component = mass_component(
            f"{group_id}_OUTPUT_EQUIVALENT",
            f"{spec['motor_id']} GO-M8010 output equivalent",
            "User physical scale measurement M_GO_M8010_SINGLE_WITH_WIRING_001 and frozen V1 allocation",
            GO_OUTPUT,
            "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            spec["output_owner"],
            [spec["output_token"], spec["neutral_token"]],
            MEAS_GO,
            "The neutral B6808 visual member is accounted inside this 0.070 kg output equivalent and has no separate V1 additive mass.",
            included_in_other_measurement=[MEAS_GO],
            allocation_group_id=group_id,
        )
        ledger_components[spec["stator_owner"]].append(stator_component)
        ledger_components[spec["output_owner"]].append(output_component)
        used_visual_tokens.update(stator_component["cad_members"])
        used_visual_tokens.update(output_component["cad_members"])
        motor_mapping[spec["motor_id"]] = {
            "joint": spec["joint"],
            "physical_actuator": spec["motor_id"],
            "allocation_group_id": group_id,
            "stator_core": {
                "mass_kg": kg(GO_STATOR),
                "owner_link": spec["stator_owner"],
                "cad_token": spec["stator_token"],
                "cad_source_object": visual_by_token[spec["stator_token"]]["source_object"],
                "mass_status": "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            },
            "output_equivalent": {
                "mass_kg": kg(GO_OUTPUT),
                "owner_link": spec["output_owner"],
                "cad_tokens": [spec["output_token"], spec["neutral_token"]],
                "neutral_bearing_additional_mass_kg": 0.0,
                "mass_status": "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
            },
            "allocation_total_kg": kg(GO_TOTAL),
            "cad_membership_validation": "PASS",
        }

    require(
        not any(
            component["name"].startswith(("J", "GO"))
            for link in ("link6", "gripper")
            for component in ledger_components[link]
        ),
        "GO mass leaked into link6 or gripper",
    )

    upper_print_tokens = sorted(
        row["token"]
        for row in authority["visual_rows"]
        if "UpperArm" in row["token"] and "PrintPart" in row["token"]
    )
    require(upper_print_tokens == sorted(UPPER_ARM_MEMBERS), "upper-arm PrintPart set is not uniquely A+B")
    require(
        all(visual_by_token[token]["owner"] == "link2" for token in UPPER_ARM_MEMBERS),
        "upper-arm PrintPart ownership is not uniquely link2",
    )
    require(
        all(token in visual_by_token and visual_by_token[token]["owner"] == "link2" for token in UPPER_ARM_FASTENERS),
        "upper-arm fastener inventory changed",
    )
    upper_component = mass_component(
        "UPPER_ARM_PRINT_A_PLUS_B",
        "Upper-arm 3D printed structure (A+B combined measurement)",
        "User physical scale measurement M_UPPER_ARM_PRINT_001, cross-checked against CAD PrintPart membership",
        UPPER_ARM,
        "MEASURED",
        "link2",
        list(UPPER_ARM_MEMBERS),
        MEAS_UPPER,
        "One measured group containing exactly the two complementary upper-arm print parts. The 0.5515 kg is not split between A and B. Six UpperArm_M35 metal fasteners and all proxies/axes are excluded.",
    )
    ledger_components["link2"].append(upper_component)
    used_visual_tokens.update(UPPER_ARM_MEMBERS)

    # Every authoritative visual token not covered above remains an explicit
    # unresolved mass member.  Internal gripper roles are accounting-rollups
    # only; their CAD ownership is preserved in cad_owners.
    remaining_by_link: dict[str, list[dict]] = defaultdict(list)
    for row in authority["visual_rows"]:
        if row["token"] in used_visual_tokens:
            continue
        accounting_link = "gripper" if row["owner"].startswith("gripper_internal:") else row["owner"]
        require(accounting_link in LINKS, f"cannot roll up owner {row['owner']}")
        remaining_by_link[accounting_link].append(row)
    for link in LINKS:
        rows = sorted(remaining_by_link[link], key=lambda row: (row["owner"], row["token"]))
        require(rows, f"expected unresolved physical members for {link}")
        cad_owners = sorted({row["owner"] for row in rows})
        rolled_internal = any(owner.startswith("gripper_internal:") for owner in cad_owners)
        component = mass_component(
            f"UNRESOLVED_{link.upper()}_REMAINING_VISUAL_MEMBERS",
            f"Remaining unweighed authoritative visual members for {link}",
            "rigid_links_v15_13/rigid_link_membership.csv; no user-confirmed leaf mass yet",
            None,
            "UNRESOLVED",
            link,
            [row["token"] for row in rows],
            None,
            "No material-density, bounding-box, collision-proxy or guessed fastener/motor/camera mass is applied. Additional leaf-level weighing is required before final link mass.",
            cad_owners=cad_owners,
            owner_mapping_policy=(
                "ACCOUNTING_ROLLUP_ONLY_NOT_CAD_REPARENTING"
                if rolled_internal
                else "DIRECT_CAD_OWNER"
            ),
        )
        ledger_components[link].append(component)
        used_visual_tokens.update(component["cad_members"])

    coverage = Counter(
        token
        for link in LINKS
        for component in ledger_components[link]
        for token in component["cad_members"]
    )
    authoritative_tokens = set(visual_by_token)
    require(set(coverage) == authoritative_tokens, "mass ledger visual-token coverage is not exact")
    require(all(count == 1 for count in coverage.values()), "a visual token is counted more than once")
    require(
        all(
            component["geometry_role"] == "visual"
            and all(token in visual_by_token for token in component["cad_members"])
            for link in LINKS
            for component in ledger_components[link]
        ),
        "collision/reference geometry entered the mass ledger",
    )

    component_ids = [
        component["component_id"]
        for link in LINKS
        for component in ledger_components[link]
    ]
    require(len(component_ids) == len(set(component_ids)), "component IDs are not unique")
    neutral_tokens = {spec["neutral_token"] for spec in MOTOR_SPECS}
    for token in neutral_tokens:
        holders = [
            component
            for link in LINKS
            for component in ledger_components[link]
            if token in component["cad_members"]
        ]
        require(len(holders) == 1 and "OUTPUT_EQUIVALENT" in holders[0]["component_id"], f"neutral mass double-count risk: {token}")
    require(
        not any(
            "wiring" in component["name"].lower()
            for link in LINKS
            for component in ledger_components[link]
        ),
        "separate wiring mass component found",
    )

    link_mass_ledger: dict[str, dict] = {}
    for link in LINKS:
        components = ledger_components[link]
        measured = sum(
            (D(str(component["mass_kg"])) for component in components if component["mass_status"] == "MEASURED"),
            D("0"),
        )
        derived = sum(
            (D(str(component["mass_kg"])) for component in components if component["mass_status"] == "DERIVED_FROM_MEASURED_TOTAL"),
            D("0"),
        )
        estimated = sum(
            (
                D(str(component["mass_kg"]))
                for component in components
                if component["mass_status"] in {
                    "ENGINEERING_ESTIMATE",
                    "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
                }
            ),
            D("0"),
        )
        unresolved_ids = [
            component["component_id"]
            for component in components
            if component["mass_status"] == "UNRESOLVED"
        ]
        require(unresolved_ids, f"{link} unexpectedly has a final mass")
        link_mass_ledger[link] = {
            "confirmed_measured_mass_kg": kg(measured),
            "derived_mass_kg": kg(derived),
            "estimated_mass_kg": kg(estimated),
            "known_accounted_subtotal_kg": kg(measured + derived + estimated),
            "unresolved_mass_kg": None,
            "unresolved_component_count": len(unresolved_ids),
            "unresolved_component_ids": unresolved_ids,
            "is_final_link_mass": False,
            "components": components,
        }

    expected_link_subtotals = {
        "base_link": D("0.470"),
        "link1": D("1.010"),
        "link2": D("0.7615"),
        "link3": D("0.940"),
        "link4": D("0.540"),
        "link5": D("0.070"),
        "link6": D("0"),
        "gripper": D("0"),
    }
    for link, expected in expected_link_subtotals.items():
        actual = D(str(link_mass_ledger[link]["known_accounted_subtotal_kg"]))
        require(actual == expected, f"ledger subtotal mismatch for {link}: {actual} != {expected}")

    modeled_go_total = sum((GO_TOTAL for _ in MOTOR_SPECS), D("0"))
    require(abs(modeled_go_total - D("3.240")) < D("1e-9"), "six GO modeled masses do not total 3.240 kg")
    known_total = sum(expected_link_subtotals.values(), D("0"))
    require(known_total == D("3.7915"), "known system subtotal drift")

    distal_boundary_anchor_members = [
        "Wrist_Prelink_v1_HighDetail_Display",
        "J5_Motor_Stator_STEP_Display",
        "J5_Output_Rotor_STEP_Display",
        "J5_B6808_Bearing_Neutral_STEP_Display",
        "J5_to_Damiao_J6_Adapter_STL_Display",
        "Adapter_DM_G6220_M4x12_Screw_01",
        "Adapter_DM_G6220_M4x12_Screw_02",
        "Adapter_DM_G6220_M4x12_Screw_03",
        "J6_DM_G6220_Stator_STEP_Display",
        "J6_DM_G6220_Output_Rotor_STEP_Display",
        "J6_Gripper_Connector_STEP_Display",
        "DM_G6220_Connector_M4x14_Screw_01",
        "DM_G6220_Connector_M4x14_Screw_02",
        "DM_G6220_Connector_M4x14_Screw_03",
        "DM_G6220_Connector_M4x14_Screw_04",
        "DM_G6220_Connector_M4x14_Screw_05",
        "DM_G6220_Connector_M4x14_Screw_06",
        "Gemini_Pro_Camera_STEP_Display",
    ]
    require(all(token in visual_by_token for token in distal_boundary_anchor_members), "distal candidate token missing")
    gripper_candidate_rows = [
        row
        for row in authority["visual_rows"]
        if row["owner"] == "gripper" or row["owner"].startswith("gripper_internal:")
    ]

    measurements = {
        "go_m8010_single_with_wiring": {
            "measurement_id": MEAS_GO,
            "name": "One GO-M8010-6 with current attached wiring",
            "mass_kg": kg(GO_TOTAL),
            "mass_status": "MEASURED",
            "measurement_boundary_status": "CONFIRMED",
            "accounting_role": "NON_ADDITIVE_MEASUREMENT_BASIS",
            "measured_sample_count": 1,
            "robot_application_count": 6,
            "notes": "The single physical measurement is the basis for six repeated V1 engineering allocations; it is not added again on top of those allocations.",
        },
        "upper_arm_print": {
            "measurement_id": MEAS_UPPER,
            "name": "Upper-arm 3D printed structure",
            "mass_kg": kg(UPPER_ARM),
            "mass_status": "MEASURED",
            "measurement_boundary_status": "CONFIRMED",
            "accounting_role": "ADDITIVE_LEDGER_COMPONENT",
            "owner_link": "link2",
            "cad_members": list(UPPER_ARM_MEMBERS),
            "excluded_cad_members": list(UPPER_ARM_FASTENERS),
            "unique_cad_mapping": True,
            "notes": "CAD metadata defines one upper arm split into exactly two complementary print parts. No motor, metal fastener, collision proxy or reference object is included.",
        },
        "distal_assembly": {
            "measurement_id": MEAS_DISTAL,
            "name": "Damiao motor + GO motor + arm distal end combination",
            "mass_kg": 1.0875,
            "mass_status": "MEASURED",
            "measurement_boundary_status": "NEED_USER_CONFIRMATION",
            "accounting_role": "NON_ADDITIVE_UNRESOLVED_OBSERVATION",
            "ledger_allocation_status": "NOT_ALLOCATED",
            "candidate_owner_links": ["link4", "link5", "link6", "gripper"],
            "candidate_boundary_anchor_members": distal_boundary_anchor_members,
            "candidate_list_is_exhaustive": False,
            "cad_attachment_relation": "CONFIRMED",
            "measurement_inclusion": "UNCONFIRMED",
            "mass_equation": "M_distal_assembly = 1.0875 kg; leaf boundaries unresolved",
            "residual_arithmetic_enabled": False,
            "notes": "CAD proves attachment topology but not which detachable items were on the scale. No DM-G6220, Gemini, adapter, fastener or gripper mass is guessed.",
        },
        "forearm_assembly": {
            "measurement_id": MEAS_FOREARM,
            "name": "Complete forearm-related assembly",
            "mass_kg": 2.723,
            "mass_status": "MEASURED",
            "measurement_boundary_status": "NEED_USER_CONFIRMATION",
            "accounting_role": "NON_ADDITIVE_UNRESOLVED_OBSERVATION",
            "ledger_allocation_status": "NOT_ALLOCATED",
            "candidate_owner_links": ["link3", "link4", "link5", "link6", "gripper"],
            "cad_attachment_relation": "CONFIRMED",
            "measurement_inclusion": "UNCONFIRMED",
            "mass_equation": "M_forearm_assembly = 2.723 kg; nested assembly boundaries unresolved",
            "residual_arithmetic_enabled": False,
            "notes": "The repository proves link3 contains J3 stator, forearm, J4 stator and fasteners, but does not prove the physical proximal/distal cuts used for weighing.",
        },
        "gripper_connector": {
            "measurement_id": MEAS_GRIPPER,
            "name": "Gripper plus directly connected connector",
            "mass_kg": 0.17,
            "mass_status": "MEASURED",
            "measurement_boundary_status": "NEED_USER_CONFIRMATION",
            "accounting_role": "NON_ADDITIVE_UNRESOLVED_OBSERVATION",
            "ledger_allocation_status": "NOT_ALLOCATED",
            "candidate_owner_links": ["link6", "gripper"],
            "candidate_cad_members_requiring_yes_no": [
                "J6_DM_G6220_Output_Rotor_STEP_Display",
                *sorted(row["token"] for row in gripper_candidate_rows),
            ],
            "camera_inclusion": "UNKNOWN",
            "gemini_cad_attachment": "CONFIRMED_TO_GRIPPER",
            "mass_equation": "M_gripper_connector = 0.170 kg; Gemini and moving-gripper membership unresolved",
            "residual_arithmetic_enabled": False,
            "notes": "Gemini belongs to gripper in CAD, but CAD attachment does not prove it was present on the physical scale.",
        },
    }

    hierarchy_edges = [
        {
            "parent_measurement_id": MEAS_FOREARM,
            "child_measurement_id": MEAS_DISTAL,
            "relation": "POSSIBLE_CONTAINS",
            "status": "UNCONFIRMED",
            "arithmetic_enabled": False,
            "residual_mass_kg": None,
        },
        {
            "parent_measurement_id": MEAS_DISTAL,
            "child_measurement_id": MEAS_GRIPPER,
            "relation": "POSSIBLE_CONTAINS",
            "status": "UNCONFIRMED",
            "arithmetic_enabled": False,
            "residual_mass_kg": None,
        },
        {
            "parent_measurement_id": MEAS_FOREARM,
            "child_measurement_id": MEAS_GRIPPER,
            "relation": "POSSIBLE_CONTAINS",
            "status": "UNCONFIRMED",
            "arithmetic_enabled": False,
            "residual_mass_kg": None,
        },
    ]
    require(all(not edge["arithmetic_enabled"] for edge in hierarchy_edges), "unconfirmed hierarchy arithmetic enabled")
    require(
        len({edge["parent_measurement_id"] for edge in hierarchy_edges} | {edge["child_measurement_id"] for edge in hierarchy_edges}) == 3,
        "unexpected hierarchy node set",
    )

    unresolved = [
        {
            "measurement_id": MEAS_DISTAL,
            "measurement_boundary_status": "NEED_USER_CONFIRMATION",
            "required_information": [
                "1087.5 g：请逐项确认 Wrist_Prelink、完整 J5 GO-M8010、J5-to-DM adapter 与 3 颗螺栓、完整 J6 DM-G6220、J6 夹爪连接件与 6 颗螺栓、完整夹爪、Gemini Pro 是否在秤盘上（YES/NO）；或提供带标注的秤盘照片。",
            ],
        },
        {
            "measurement_id": MEAS_FOREARM,
            "measurement_boundary_status": "NEED_USER_CONFIRMATION",
            "required_information": [
                "2723 g：请确认上述 1087.5 g 子装配是否原样完整包含在本次称重中。",
                "2723 g：请确认近端切口是否从 J3 stator 开始，末端是否包含 gripper 与 Gemini Pro。",
            ],
        },
        {
            "measurement_id": MEAS_GRIPPER,
            "measurement_boundary_status": "NEED_USER_CONFIRMATION",
            "required_information": [
                "170 g：请确认是否包含 Gemini Pro。",
                "170 g：请确认是否包含舍机、固定框、六个夹爪内部活动刚体角色、全部夹爪紧固件，以及 link6 的 DM 输出法兰/六颗连接件螺栓。",
                "170 g：如果包含 Gemini Pro，请另行确认 CAD 未建模的相机安装螺丝及相机随件线材是否在秤盘上。",
            ],
        },
    ]

    protected_entries = [
        {
            "path": relative,
            "sha256_before": digest,
            "sha256_after": digest,
            "unchanged": True,
            "matches_review_baseline": digest == PROTECTED_INPUTS[relative]["sha256"],
            "hash_mode": PROTECTED_INPUTS[relative]["hash_mode"],
        }
        for relative, digest in protected.items()
    ]

    contract = {
        "schema": SCHEMA,
        "revision": REVISION,
        "scope": "MASS_LEDGER_ONLY_NO_COM_NO_INERTIA",
        "measurement_authority": "USER_PHYSICAL_SCALE",
        "units": {"mass": "kg"},
        "source_authorities": {
            "rigid_link_membership": {
                "path": "rigid_links_v15_13/rigid_link_membership.csv",
                "sha256": protected["rigid_links_v15_13/rigid_link_membership.csv"],
                "hash_mode": PROTECTED_INPUTS["rigid_links_v15_13/rigid_link_membership.csv"]["hash_mode"],
                "geometry_role_used_for_mass": "visual",
                "row_count": len(authority["rows"]),
                "visual_row_count": len(authority["visual_rows"]),
            },
            "rigid_link_manifest": {
                "path": "rigid_links_v15_13/rigid_link_manifest.json",
                "sha256": protected["rigid_links_v15_13/rigid_link_manifest.json"],
                "hash_mode": PROTECTED_INPUTS["rigid_links_v15_13/rigid_link_manifest.json"]["hash_mode"],
            },
            "source_freecad": {
                "path": "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd",
                "sha256": protected["机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"],
                "hash_mode": "RAW_BYTES",
            },
            "rigid_link_freecad": {
                "path": "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd",
                "sha256": protected["机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"],
                "hash_mode": "RAW_BYTES",
                "object_count_in_document_xml": len(authority["object_names"]),
            },
        },
        "frozen_geometry_guard": {
            "protected_inputs": protected_entries,
            "all_protected_inputs_unchanged": True,
            "all_protected_inputs_match_review_baseline": True,
            "frozen_joint_canonical_sha256": authority["canonical_by_version"],
            "all_frozen_joint_elements_unchanged": True,
            "j1_to_j6_geometry_modified": False,
            "urdf_or_mujoco_inertial_written": False,
        },
        "topology": {
            "tree": "world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper",
            "mass_ledger_links": list(LINKS),
            "j2_dof_count": 1,
            "j2_physical_actuators": topology_joints["J2"]["physical_actuators"],
            "link_partition_modified": False,
        },
        "status_definitions": {
            "mass_status": [
                "MEASURED",
                "DERIVED_FROM_MEASURED_TOTAL",
                "ENGINEERING_ESTIMATE",
                "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
                "UNRESOLVED",
            ],
            "measurement_boundary_status": ["CONFIRMED", "NEED_USER_CONFIRMATION"],
            "inclusion_status": ["CONFIRMED_INCLUDED", "CONFIRMED_NOT_INCLUDED", "UNKNOWN", "UNCONFIRMED"],
            "accounting_role": [
                "ADDITIVE_LEDGER_COMPONENT",
                "NON_ADDITIVE_MEASUREMENT_BASIS",
                "NON_ADDITIVE_UNRESOLVED_OBSERVATION",
                "INCLUDED_NO_SEPARATE_MASS",
            ],
        },
        "motor_mass_model": {
            "GO_M8010_6": {
                "measured_total_kg": kg(GO_TOTAL),
                "measured_total_mass_status": "MEASURED",
                "measured_sample_count": 1,
                "output_equivalent_kg": kg(GO_OUTPUT),
                "stator_core_equivalent_kg": kg(GO_STATOR),
                "allocation_mass_status": "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
                "allocation_identity_pass": GO_OUTPUT + GO_STATOR == GO_TOTAL,
                "neutral_bearing_policy": "accounted inside output equivalent; no additional V1 mass",
                "wiring_policy": "accounted inside stator/core equivalent; no additional wire mass",
                "robot_motor_count": len(MOTOR_SPECS),
                "robot_modeled_total_kg": kg(modeled_go_total),
                "robot_modeled_total_mass_status": "ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL",
                "robot_modeled_total_assertion_tolerance_kg": 1e-9,
            }
        },
        "motor_link_mapping": motor_mapping,
        "measurements": measurements,
        "measurement_hierarchy": {
            "nodes": [MEAS_GO, MEAS_UPPER, MEAS_DISTAL, MEAS_FOREARM, MEAS_GRIPPER],
            "edges": hierarchy_edges,
            "is_dag": True,
            "confirmed_containment_edge_count": 0,
            "unconfirmed_containment_edge_count": len(hierarchy_edges),
            "residual_arithmetic_globally_enabled": False,
            "prohibited_unconfirmed_residuals": [
                "2.723 - 1.0875",
                "1.0875 - 0.170",
            ],
        },
        "link_mass_ledger": link_mass_ledger,
        "known_system_mass_subtotal_kg": kg(known_total),
        "known_system_mass_subtotal_is_final": False,
        "unresolved_measurement_boundaries": unresolved,
        "double_count_checks": {
            "component_ids_unique": True,
            "authoritative_visual_token_count": len(authoritative_tokens),
            "authoritative_visual_tokens_covered_exactly_once": True,
            "collision_or_reference_geometry_used_as_mass": False,
            "neutral_b6808_separate_additive_component_found": False,
            "neutral_b6808_accounted_inside_output_equivalent": True,
            "separate_go_wiring_component_found": False,
            "go_measured_basis_directly_added_on_top_of_allocations": False,
            "unconfirmed_composite_measurements_added_to_link_ledger": False,
            "unconfirmed_hierarchy_arithmetic_enabled": False,
            "parent_and_child_composite_measurements_both_additive": False,
            "double_count_detected": False,
        },
        "validation": {
            "contract_build_status": "PASS",
            "mass_model_completion_status": "INCOMPLETE_BOUNDARIES_REQUIRE_USER_CONFIRMATION",
            "checks": {
                "rigid_link_membership_read": "PASS",
                "rigid_link_manifest_cross_check": "PASS",
                "freecad_visual_source_object_cross_check": "PASS",
                "J1_GO_parent_child_mass_mapping": "PASS",
                "J2A_J2B_dual_motor_mapping": "PASS",
                "J3_mass_mapping": "PASS",
                "J4_mass_mapping": "PASS",
                "J5_mass_mapping": "PASS",
                "six_GO_total_equals_3_240_kg": "PASS",
                "link2_known_minimum_subtotal_kg": 0.7615,
                "upper_arm_551_5g_unique_CAD_mapping": "PASS",
                "distal_1087_5g_boundary_proved": "NO",
                "forearm_2723g_boundary_proved": "NO",
                "gripper_170g_camera_inclusion": "UNKNOWN",
                "double_count_found": "NO",
                "J1_to_J6_geometry_modified": "NO",
                "final_URDF_or_MuJoCo_inertial_written": "NO",
            },
            "all_automated_assertions_pass": True,
        },
        "prohibited_outputs": {
            "center_of_mass_computed": False,
            "inertia_tensor_computed": False,
            "urdf_inertial_written": False,
            "mujoco_inertial_written": False,
            "mujoco_actuator_modified": False,
            "collision_or_visual_mesh_modified": False,
            "joint_or_tf_geometry_modified": False,
            "dm_g6220_mass_guessed": False,
            "gemini_pro_mass_guessed": False,
            "fastener_mass_guessed": False,
            "cad_density_or_bounding_box_mass_inference_used": False,
        },
    }
    return contract


def render_markdown(contract: dict) -> str:
    lines = [
        "# V15.15 实测质量 → CAD 刚性 Link 映射说明",
        "",
        "## 范围与结论",
        "",
        "本阶段仅建立质量账本与映射关系；不计算 COM/惯量张量，不写入 URDF/MuJoCo inertial，不修改关节几何、网格、TF、MoveIt、ros2_control 或控制器。",
        "",
        "- 质量映射契约构建：**PASS**",
        "- 质量模型完整度：**INCOMPLETE_BOUNDARIES_REQUIRE_USER_CONFIRMATION**",
        "- 当前有效账本未发现 double count。",
        "- 1087.5 g、2723 g 和 170 g 仅作为非加和、边界未决的实测观测值，未分配到任何 Link。",
        "",
        "## 权威输入",
        "",
        "| 输入 | SHA-256 | 用途 |",
        "|---|---|---|",
    ]
    for key in ("rigid_link_membership", "rigid_link_manifest", "source_freecad", "rigid_link_freecad"):
        entry = contract["source_authorities"][key]
        lines.append(f"| `{entry['path']}` | `{entry['sha256']}` | {key} |")

    lines.extend([
        "",
        "质量归属只使用 membership 中 `geometry_role=visual` 的 135 个唯一 token；碰撞代理与参考几何不进入质量账本。",
        "",
        "## GO-M8010-6 V1 质量模型",
        "",
        "- 单台带当前线材实测：**0.540 kg (MEASURED)**",
        "- output equivalent：**0.070 kg (ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL)**",
        "- stator/core equivalent：**0.470 kg (ENGINEERING_ESTIMATE_FROM_MEASURED_TOTAL)**",
        "- neutral B6808 已包含在 0.070 kg 内；线材已包含在 0.470 kg 内，两者均不再加重。",
        "",
        "| 电机 | stator/core → Link | output + neutral → Link | CAD 核验 |",
        "|---|---|---|---|",
    ])
    for motor_id, entry in contract["motor_link_mapping"].items():
        lines.append(
            f"| {motor_id} | 0.470 kg → `{entry['stator_core']['owner_link']}` | "
            f"0.070 kg → `{entry['output_equivalent']['owner_link']}` | {entry['cad_membership_validation']} |"
        )

    lines.extend([
        "",
        "六台 GO-M8010 V1 模型合计：**3.240 kg**（由一台实测总质量复制的工程等效模型，不是六台分别称重）。",
        "",
        "## Link 质量账本",
        "",
        "| Link | confirmed measured (kg) | derived (kg) | estimated (kg) | known subtotal (kg) | unresolved | final? |",
        "|---|---:|---:|---:|---:|---|---|",
    ])
    for link, entry in contract["link_mass_ledger"].items():
        unresolved_text = (
            "UNKNOWN"
            if entry["unresolved_mass_kg"] is None
            else f"{entry['unresolved_mass_kg']:.4f} kg"
        )
        lines.append(
            f"| `{link}` | {entry['confirmed_measured_mass_kg']:.4f} | {entry['derived_mass_kg']:.4f} | "
            f"{entry['estimated_mass_kg']:.4f} | {entry['known_accounted_subtotal_kg']:.4f} | "
            f"{unresolved_text} ({entry['unresolved_component_count']} group) | "
            f"{'YES' if entry['is_final_link_mass'] else 'NO'} |"
        )
    lines.extend([
        "",
        "`link2` 已知最低 subtotal = 0.5515 + 0.070 + 0.070 + 0.070 = **0.7615 kg**。这不是 link2 最终总质量。",
        "",
        "## 551.5 g 上臂打印件映射",
        "",
        "CAD 元数据将上臂定义为一个结构、两个互补打印件，因此 0.5515 kg 可唯一映射到 `link2` 上的组合项：",
        "",
        "- `UpperArm_A_SleeveSide_PrintPart`",
        "- `UpperArm_B_Distal_PrintPart`",
        "",
        "不将 0.5515 kg 再拆给 A/B 单件。六颗 `UpperArm_M35_*` 金属紧固件、碰撞代理、轴线和参考体均不包含。",
        "",
        "## 组合称重边界与 inclusion graph",
        "",
        "| 实测项 | 质量 (kg) | 边界 | 帐本处理 |",
        "|---|---:|---|---|",
    ])
    for key in ("distal_assembly", "forearm_assembly", "gripper_connector"):
        entry = contract["measurements"][key]
        lines.append(
            f"| {entry['name']} | {entry['mass_kg']:.4f} | {entry['measurement_boundary_status']} | "
            f"{entry['accounting_role']} / {entry['ledger_allocation_status']} |"
        )
    lines.extend([
        "",
        "```text",
        "M_forearm_assembly (2.723 kg)",
        "  -- POSSIBLE_CONTAINS / UNCONFIRMED --> M_distal_assembly (1.0875 kg)",
        "       -- POSSIBLE_CONTAINS / UNCONFIRMED --> M_gripper_connector (0.170 kg)",
        "  -- POSSIBLE_CONTAINS / UNCONFIRMED --> M_gripper_connector (0.170 kg) [independent direct relation]",
        "```",
        "",
        "CAD 机械连接关系已确认，但它不能证明秤盘上的实物 inclusion。因此不允许计算 `2.723-1.0875` 或 `1.0875-0.170`。",
        "",
        "## 待用户确认",
        "",
    ])
    question_number = 1
    for boundary in contract["unresolved_measurement_boundaries"]:
        for question in boundary["required_information"]:
            lines.append(f"{question_number}. {question}")
            question_number += 1
    lines.extend([
        "",
        "## 自动验收",
        "",
        "| 检查 | 结果 |",
        "|---|---|",
    ])
    for key, value in contract["validation"]["checks"].items():
        lines.append(f"| `{key}` | {value} |")
    lines.extend([
        "",
        "## 明确禁止的本阶段输出",
        "",
        "- COM：未计算",
        "- 惯量张量：未计算",
        "- URDF/MuJoCo inertial：未写入",
        "- J1~J6 origin/axis/limit、TF、碰撞/视觉网格、MoveIt、ros2_control、controller、MuJoCo actuator：未修改",
        "- DM-G6220、Gemini Pro、螺丝、材料密度：未猜测",
        "",
        "本阶段到此停止；不进入 COM、惯量张量或动力学参数化。",
        "",
    ])
    return "\n".join(lines)


def json_text(contract: dict) -> str:
    return json.dumps(contract, ensure_ascii=False, indent=2) + "\n"


def write_text(path: Path, content: str) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(content)


def print_summary(contract: dict, mode: str) -> None:
    checks = contract["validation"]["checks"]
    summary = {
        "mode": mode,
        "status": contract["validation"]["contract_build_status"],
        "rigid_link_membership": checks["rigid_link_membership_read"],
        "J1": checks["J1_GO_parent_child_mass_mapping"],
        "J2A_J2B": checks["J2A_J2B_dual_motor_mapping"],
        "J3": checks["J3_mass_mapping"],
        "J4": checks["J4_mass_mapping"],
        "J5": checks["J5_mass_mapping"],
        "six_GO_total_kg": contract["motor_mass_model"]["GO_M8010_6"]["robot_modeled_total_kg"],
        "link2_known_subtotal_kg": checks["link2_known_minimum_subtotal_kg"],
        "upper_arm_unique_mapping": checks["upper_arm_551_5g_unique_CAD_mapping"],
        "distal_boundary_proved": checks["distal_1087_5g_boundary_proved"],
        "forearm_boundary_proved": checks["forearm_2723g_boundary_proved"],
        "gripper_camera_inclusion": checks["gripper_170g_camera_inclusion"],
        "double_count_found": checks["double_count_found"],
        "J1_to_J6_geometry_modified": checks["J1_to_J6_geometry_modified"],
        "inertial_written": checks["final_URDF_or_MuJoCo_inertial_written"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--write", action="store_true", help="write the JSON and Markdown outputs")
    action.add_argument("--check", action="store_true", help="rebuild in memory and compare committed outputs")
    parser.add_argument("--repo-root", help="optional repository root; no absolute path is embedded in outputs")
    args = parser.parse_args()

    try:
        repo = find_repo_root(args.repo_root)
        before = snapshot_protected_inputs(repo)
        authority = load_authorities(repo)
        contract = build_contract(repo, authority, before)
        json_payload = json_text(contract)
        markdown_payload = render_markdown(contract)
        require(snapshot_protected_inputs(repo) == before, "protected inputs changed during build")

        json_path = repo / OUTPUT_JSON
        md_path = repo / OUTPUT_MD
        if args.write:
            write_text(json_path, json_payload)
            write_text(md_path, markdown_payload)
            require(snapshot_protected_inputs(repo) == before, "protected inputs changed while writing outputs")
            mode = "write"
        else:
            require(json_path.is_file(), f"missing generated output: {OUTPUT_JSON}")
            require(md_path.is_file(), f"missing generated output: {OUTPUT_MD}")
            require(json_path.read_text(encoding="utf-8") == json_payload, f"stale generated output: {OUTPUT_JSON}")
            require(md_path.read_text(encoding="utf-8") == markdown_payload, f"stale generated output: {OUTPUT_MD}")
            mode = "check"
        print_summary(contract, mode)
        return 0
    except Exception as exc:  # fail closed and provide one actionable message
        print(f"V15.15 mass mapping FAIL: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
