from __future__ import annotations

"""Independent fail-closed validation for the V15.14 collision overlay."""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from v15_14_collision_layer import (  # noqa: E402
    EXPECTED_ALL_PAIR_COUNT,
    EXPECTED_EXCLUDED_PAIR_COUNT,
    EXPECTED_PROXY_COUNT,
    EXPECTED_RUNTIME_PAIR_COUNT,
    J1_FIXED_TOKEN,
    J1_MOVING_TOKEN,
    MOTION_ENABLED_PAIRS,
    MOTION_TOKEN,
    build_v15_14_contract,
    canonical_sha256,
    collect_proxy_meshes,
    file_sha256,
    mjcf_frozen_hashes,
    normalized_pair,
    validate_export_manifest,
    validate_frozen_proxy_meshes,
    validate_runtime_token_coverage,
)


EXPECTED_POSE_SOURCE_SHA256 = (
    "9BE9FAB3E5C7850F001C2E46B83AEBF033A02FEFF2A9E27DBC15347DB4098EA4"
)
EXPECTED_POSE_SET_SHA256 = (
    "4D9CFAF15CC20FE16393D1AA3BCCDDC5273E77E2DE08CF9D0045976C301799AA"
)


def load_module(path: Path, name: str):
    specification = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(specification)
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


def static_validation(root: Path) -> dict:
    project_root = root.parent
    source_contract_path = project_root / "V15_13_自碰撞对矩阵契约.json"
    source_xacro = (
        project_root
        / "完整工程_V15_13_交付"
        / "ros2_ws"
        / "src"
        / "go_m8010_arm_description"
        / "urdf"
        / "go_m8010_arm_v15_13.urdf.xacro"
    )
    source_mjcf = (
        project_root / "mujoco_kinematic_v1" / "go_m8010_arm_v15_13_kinematic.xml"
    )
    source_mesh_manifest = project_root / "mujoco_kinematic_v1" / "mesh_export_manifest.json"
    contract_path = root / "config" / "collision_pair_contract_v15_14.json"
    export_manifest_path = (
        root / "config" / "upperarm_motion_proxy_export_v15_14.json"
    )
    urdf_path = (
        root
        / "ros2_ws"
        / "src"
        / "go_m8010_arm_v15_14_description"
        / "urdf"
        / "go_m8010_arm_v15_14.urdf.xacro"
    )
    proxy_mesh_root = urdf_path.parent.parent / "meshes" / "collision_proxies"
    srdf_path = (
        root
        / "ros2_ws"
        / "src"
        / "go_m8010_arm_v15_14_moveit_config"
        / "config"
        / "go_m8010_arm_v15_14.srdf"
    )
    mjcf_path = root / "mujoco_v15_14" / "go_m8010_arm_v15_14_kinematic.xml"
    accepted_path = root / "config" / "accepted_frozen_contract_v15_13.json"

    expected_contract = build_v15_14_contract(source_contract_path)
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    if contract != expected_contract:
        raise RuntimeError("derived collision contract is not the deterministic source overlay")
    counts = (
        contract["proxy_count"],
        contract["all_unordered_pair_count"],
        contract["runtime_full_pair_count"],
        contract["runtime_excluded_pair_count"],
    )
    expected_counts = (
        EXPECTED_PROXY_COUNT,
        EXPECTED_ALL_PAIR_COUNT,
        EXPECTED_RUNTIME_PAIR_COUNT,
        EXPECTED_EXCLUDED_PAIR_COUNT,
    )
    if counts != expected_counts:
        raise RuntimeError(f"pair partition mismatch: {counts} != {expected_counts}")

    proxy_set = set(contract["proxies"])
    full = {normalized_pair(*pair) for pair in contract["runtime_full_pairs"]}
    excluded = {
        normalized_pair(*pair) for pair in contract["runtime_excluded_pairs"]
    }
    all_pairs = full | excluded
    if full & excluded or len(all_pairs) != EXPECTED_ALL_PAIR_COUNT:
        raise RuntimeError("runtime/excluded pair partition is not disjoint and complete")
    if {token for pair in all_pairs for token in pair} != proxy_set:
        raise RuntimeError("pair partition token universe mismatch")
    if {pair for pair in full if MOTION_TOKEN in pair} != MOTION_ENABLED_PAIRS:
        raise RuntimeError("Motion enabled token pairs mismatch")
    if len({pair for pair in excluded if MOTION_TOKEN in pair}) != 22:
        raise RuntimeError("Motion must have exactly 22 excluded token pairs")

    export_manifest = validate_export_manifest(export_manifest_path, root)
    proxy_meshes = collect_proxy_meshes(proxy_mesh_root, contract)
    inherited_mesh_validation = validate_frozen_proxy_meshes(
        source_mesh_manifest, proxy_mesh_root, proxy_meshes
    )
    urdf_root = ET.parse(urdf_path).getroot()
    srdf_root = ET.parse(srdf_path).getroot()
    mjcf_root = ET.parse(mjcf_path).getroot()
    coverage = validate_runtime_token_coverage(
        contract, urdf_root, mjcf_root, proxy_meshes
    )

    motion_link = f"collision_proxy__{MOTION_TOKEN}"
    motion_joint = next(
        (
            joint
            for joint in urdf_root.findall("joint")
            if joint.find("child") is not None
            and joint.find("child").get("link") == motion_link
        ),
        None,
    )
    if motion_joint is None or motion_joint.find("parent").get("link") != "link2":
        raise RuntimeError("URDF Motion proxy is not fixed under link2")
    if motion_joint.get("type") != "fixed":
        raise RuntimeError("URDF Motion proxy joint is not fixed")

    actual_acm = {
        normalized_pair(
            row.get("link1")[len("collision_proxy__") :],
            row.get("link2")[len("collision_proxy__") :],
        )
        for row in srdf_root.findall("disable_collisions")
    }
    if actual_acm != excluded:
        raise RuntimeError(
            f"SRDF ACM mismatch: missing={sorted(excluded-actual_acm)}, "
            f"extra={sorted(actual_acm-excluded)}"
        )

    accepted = json.loads(accepted_path.read_text(encoding="utf-8"))
    source_urdf_root = ET.parse(source_xacro).getroot()
    generated_joints = {
        row.get("name"): row for row in urdf_root.findall("joint")
    }
    source_joints = {
        row.get("name"): row for row in source_urdf_root.findall("joint")
    }
    frozen_hashes = {}
    for name, accepted_hash in accepted["frozen_joint_canonical_sha256"].items():
        source_hash = canonical_sha256(source_joints[name]).lower()
        generated_hash = canonical_sha256(generated_joints[name]).lower()
        if source_hash != accepted_hash or generated_hash != accepted_hash:
            raise RuntimeError(f"frozen URDF joint/transform changed: {name}")
        frozen_hashes[name] = generated_hash

    source_mjcf_hashes = mjcf_frozen_hashes(ET.parse(source_mjcf).getroot())
    derived_mjcf_hashes = mjcf_frozen_hashes(mjcf_root)
    if source_mjcf_hashes != derived_mjcf_hashes:
        raise RuntimeError("derived MJCF changed J1..J6/TCP/camera frozen elements")

    explicit_pairs = list(mjcf_root.findall("./contact/pair"))
    return {
        "pass": True,
        "pair_partition": {
            "proxy_count": counts[0],
            "all_unordered_pair_count": counts[1],
            "runtime_full_pair_count": counts[2],
            "runtime_excluded_pair_count": counts[3],
            "motion_enabled_pair_count": 2,
            "motion_excluded_pair_count": 22,
        },
        "motion_proxy": {
            "urdf_parent": "link2",
            "urdf_mesh_sha256": export_manifest["urdf_mesh"]["sha256"],
            "mjcf_component_count": len(export_manifest["mjcf_components"]),
            "mjcf_explicit_geom_pair_count": len(explicit_pairs),
        },
        "runtime_token_coverage": coverage,
        "inherited_proxy_mesh_validation": inherited_mesh_validation,
        "srdf_acm_exclusion_count": len(actual_acm),
        "frozen_urdf_hashes": frozen_hashes,
        "frozen_mjcf_hashes": derived_mjcf_hashes,
        "source_hashes": {
            "v15_13_contract": file_sha256(source_contract_path),
            "v15_13_xacro": file_sha256(source_xacro),
            "v15_13_mjcf": file_sha256(source_mjcf),
            "v15_13_mesh_export_manifest": file_sha256(source_mesh_manifest),
        },
        "generated_hashes": {
            "contract": file_sha256(contract_path),
            "urdf_xacro": file_sha256(urdf_path),
            "srdf": file_sha256(srdf_path),
            "mjcf": file_sha256(mjcf_path),
            "guard": file_sha256(root / "mujoco_v15_14" / "kinematic_guard.py"),
        },
    }


def runtime_validation(root: Path) -> dict:
    try:
        import mujoco
    except ImportError as error:
        raise RuntimeError("MuJoCo Python module is unavailable") from error

    bundle = root / "mujoco_v15_14"
    model_path = bundle / "go_m8010_arm_v15_14_kinematic.xml"
    pose_source = root.parent / "mujoco_kinematic_v1" / "validate_model.py"
    pose_source_hash = file_sha256(pose_source)
    if pose_source_hash != EXPECTED_POSE_SOURCE_SHA256:
        raise RuntimeError(
            f"frozen 503-pose source hash mismatch: {pose_source_hash} != "
            f"{EXPECTED_POSE_SOURCE_SHA256}"
        )
    pose_module = load_module(pose_source, "_v15_13_frozen_pose_source")
    pose_sets = pose_module.build_pose_sets()
    pose_count = sum(len(rows) for rows in pose_sets.values())
    pose_payload = json.dumps(
        pose_sets, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    pose_set_hash = hashlib.sha256(pose_payload).hexdigest().upper()
    if pose_count != 503 or pose_set_hash != EXPECTED_POSE_SET_SHA256:
        raise RuntimeError(
            "frozen pose-set gate failed: "
            f"count={pose_count}, sha256={pose_set_hash}"
        )
    guard_module = load_module(bundle / "kinematic_guard.py", "_v15_14_guard")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    guard = guard_module.KinematicGuard(model_path=model_path)

    cases = [
        ("zero", [0.0] * 6, None),
        (
            "motion_vs_j1_moving",
            [0.0, -120.0, 0.0, 0.0, 0.0, 0.0],
            normalized_pair(J1_MOVING_TOKEN, MOTION_TOKEN),
        ),
        (
            "motion_vs_j1_fixed",
            [0.0, -140.0, 0.0, 0.0, 0.0, 0.0],
            normalized_pair(J1_FIXED_TOKEN, MOTION_TOKEN),
        ),
    ]
    rows = []
    for name, pose, required_pair in cases:
        result = guard.check_pose_deg(pose)
        motion_contacts = [
            row
            for row in result["contacts"]
            if MOTION_TOKEN in row["proxy_pair"]
        ]
        seen = {tuple(row["proxy_pair"]) for row in motion_contacts}
        if name == "zero":
            passed = result["safe"] and not motion_contacts
        else:
            passed = (not result["safe"]) and required_pair in seen
        if not passed:
            raise RuntimeError(
                f"MuJoCo Motion proxy negative control failed: {name}: {result}"
            )
        if not seen <= MOTION_ENABLED_PAIRS:
            raise RuntimeError(f"MuJoCo emitted a non-contract Motion pair: {seen}")
        rows.append(
            {
                "name": name,
                "pose_deg": pose,
                "safe": bool(result["safe"]),
                "reason": result["reason"],
                "motion_contact_count": len(motion_contacts),
                "motion_token_pairs": [list(pair) for pair in sorted(seen)],
                "minimum_motion_contact_distance_m": (
                    min(float(row["distance_m"]) for row in motion_contacts)
                    if motion_contacts
                    else None
                ),
                "pass": True,
            }
        )
    return {
        "pass": True,
        "mujoco_version": mujoco.__version__,
        "model_nq": int(model.nq),
        "model_ngeom": int(model.ngeom),
        "model_npair": int(model.npair),
        "expected_explicit_geom_pairs": 82,
        "frozen_pose_set_gate": {
            "pass": True,
            "source_sha256": pose_source_hash,
            "pose_count": pose_count,
            "pose_set_sha256": pose_set_hash,
        },
        "cases": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", type=Path, default=TOOLS_DIR.parent, help="V15.14 root"
    )
    parser.add_argument("--runtime-only", action="store_true")
    parser.add_argument("--require-mujoco", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()

    report = {
        "schema": "go-m8010-arm-v15.14-collision-layer-validation/1.0",
        "root": str(root),
    }
    if not args.runtime_only:
        report["static_validation"] = static_validation(root)
    try:
        report["runtime_validation"] = runtime_validation(root)
    except RuntimeError as error:
        if args.require_mujoco or args.runtime_only:
            raise
        report["runtime_validation"] = {
            "pass": None,
            "status": "SKIPPED_MUJOCO_UNAVAILABLE",
            "reason": str(error),
        }
    report["pass"] = bool(report.get("static_validation", {"pass": True})["pass"]) and (
        report["runtime_validation"]["pass"] is not False
    )

    output = args.output
    if output is None:
        output = root / "evidence" / "collision_layer_validation_v15_14.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
