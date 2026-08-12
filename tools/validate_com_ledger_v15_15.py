from __future__ import annotations

"""Independently validate and reproduce the V15.15 COM audit artifacts."""

import argparse
import json
import math
import sys
from decimal import Decimal
from pathlib import Path
from typing import Any

import build_com_ledger_v15_15 as builder


FORBIDDEN_NUMERIC_KEYS = {
    "inertia",
    "inertia_tensor",
    "ixx",
    "iyy",
    "izz",
    "ixy",
    "ixz",
    "iyz",
    "gravity",
    "damping",
    "friction",
    "armature",
}


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def reject_forbidden_numeric_keys(value: Any, path: str = "$") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            lowered = key.lower()
            if lowered in FORBIDDEN_NUMERIC_KEYS and isinstance(child, (int, float, Decimal)):
                raise ValidationError(f"FORBIDDEN_NUMERIC_FIELD:{path}.{key}")
            reject_forbidden_numeric_keys(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            reject_forbidden_numeric_keys(child, f"{path}[{index}]")


def validate_artifact(document: dict[str, Any]) -> None:
    require(document["schema"] == builder.SCHEMA, "SCHEMA_MISMATCH")
    require(document["source_commit"] == builder.SOURCE_COMMIT, "SOURCE_COMMIT_MISMATCH")
    require(document["source_tree"] == builder.SOURCE_TREE, "SOURCE_TREE_MISMATCH")
    require(document["mass_ledger_sha256"] == builder.PROTECTED_HASHES[builder.MASS_LEDGER], "MASS_LEDGER_HASH_MISMATCH")
    require(document["cad_source_sha256"] == builder.PROTECTED_HASHES[builder.SOURCE_CAD], "CAD_HASH_MISMATCH")
    require(list(document["links"]) == builder.LINK_ORDER, "LINK_ORDER_OR_SET_MISMATCH")

    component_ids = []
    total = Decimal("0")
    roundtrips = []
    for link_name in builder.LINK_ORDER:
        link = document["links"][link_name]
        expected_mass = builder.EXPECTED_LINK_MASSES[link_name]
        actual_mass = Decimal(str(link["mass_kg"]))
        component_sum = sum(Decimal(str(item["mass_kg"])) for item in link["components"])
        require(abs(actual_mass - expected_mass) <= builder.MASS_TOL_KG, f"LINK_MASS_CHANGED:{link_name}")
        require(abs(component_sum - expected_mass) <= builder.MASS_TOL_KG, f"COMPONENT_MASS_SUM_FAIL:{link_name}")
        require(link["validation"]["component_mass_sum_pass"] is True, f"MASS_PASS_FLAG_FALSE:{link_name}")
        total += actual_mass
        for component in link["components"]:
            component_ids.append(component["component_id"])
            require(component["owner_link"] == link_name, f"OWNER_LINK_MISMATCH:{component['component_id']}")
            require(component["collision_proxy_used"] is False, f"COLLISION_PROXY_USED:{component['component_id']}")
            require(all("Collision_Proxy" not in member for member in component["geometry_members"]), f"COLLISION_MEMBER:{component['component_id']}")
            if component["blocking"]:
                require(component["cad_com_world_mm"] is None, "UNRESOLVED_COMPONENT_HAS_WORLD_COM")
                require(component["cad_com_link_mm"] is None, "UNRESOLVED_COMPONENT_HAS_LOCAL_COM")
                require(component["com_status"] == "UNRESOLVED", "UNRESOLVED_STATUS_MISMATCH")
            else:
                require(builder.finite_vector(component["cad_com_world_mm"]), f"NONFINITE_COMPONENT_WORLD:{component['component_id']}")
                require(builder.finite_vector(component["cad_com_link_mm"]), f"NONFINITE_COMPONENT_LOCAL:{component['component_id']}")
                require(component["component_com_inside_bbox"] is True, f"COMPONENT_BBOX_FAIL:{component['component_id']}")
                error = float(component["round_trip_error_m"])
                require(math.isfinite(error) and error < builder.ROUNDTRIP_TOL_M, f"ROUNDTRIP_FAIL:{component['component_id']}")
                require(component["independent_audit_reference_pass"] is True, f"CAD_WORLD_REFERENCE_FAIL:{component['component_id']}")
                require(
                    float(component["independent_audit_reference_error_mm"]) <= builder.AUDITED_WORLD_COM_TOL_MM,
                    f"CAD_WORLD_REFERENCE_ERROR:{component['component_id']}",
                )
                roundtrips.append(error)
    require(component_ids == builder.COMPONENT_ORDER, "COMPONENT_ORDER_OR_SET_MISMATCH")
    require(len(set(component_ids)) == 17, "COMPONENT_IDS_NOT_UNIQUE")
    require(abs(total - builder.EXPECTED_TOTAL_MASS) <= builder.MASS_TOL_KG, "TOTAL_MASS_FAIL")
    require(document["double_count_checks"]["neutral_b6808_separate_mass_count"] == 0, "NEUTRAL_DOUBLE_COUNT")
    require(document["double_count_checks"]["gripper_geometry_member_count"] == 37, "GRIPPER_MEMBER_COUNT_MISMATCH")
    require(document["mechanical_zero_condition"]["closure_angle_deg"] == 0.0, "GRIPPER_CLOSURE_NOT_ZERO")

    residuals = [
        component
        for link in document["links"].values()
        for component in link["components"]
        if component["component_id"].startswith("RESIDUAL_")
    ]
    require(len(residuals) == 3, "RESIDUAL_COUNT_MISMATCH")
    for component in residuals:
        require(component["com_status"] == "ENGINEERING_ESTIMATE_GEOMETRIC_CENTROID", "RESIDUAL_METHOD_MISMATCH")
        require(component["cable_geometry_available"] is False, "RESIDUAL_CABLE_FLAG_MISMATCH")
    gripper = document["links"]["gripper"]["components"][0]
    require(gripper["com_status"] == "MEASURED_TOTAL_MASS_WITH_GEOMETRIC_COM_ESTIMATE", "GRIPPER_METHOD_MISMATCH")
    require(gripper["closure_angle_deg"] == 0.0, "GRIPPER_COMPONENT_CLOSURE_NOT_ZERO")

    forearm = next(
        component
        for component in document["links"]["link3"]["components"]
        if component["component_id"] == "FOREARM_PRINT_MEASURED"
    )
    require(
        forearm["com_status"] == "CAD_SOURCE_MESH_ZERO_VOLUME_ARTIFACT_EXCLUDED_VOLUME_CENTER",
        "FOREARM_COM_STATUS_MISMATCH",
    )
    require(forearm["excluded_zero_volume_component_count"] == 1, "FOREARM_ZERO_VOLUME_COMPONENT_COUNT_MISMATCH")
    require(forearm["excluded_zero_volume_facets"] == 2, "FOREARM_ZERO_VOLUME_FACET_COUNT_MISMATCH")
    require(forearm["positive_volume_component_count"] == 1, "FOREARM_POSITIVE_COMPONENT_COUNT_MISMATCH")
    require(
        forearm["all_positive_volume_components_closed_manifold"] is True,
        "FOREARM_POSITIVE_COMPONENT_TOPOLOGY_INVALID",
    )
    require(
        all(
            float(atom["abs_volume_mm3"]) > builder.EPS_VOLUME_MM3
            and atom["is_solid"] is True
            and atom["has_non_manifolds"] is False
            for atom in forearm["geometry_atoms"]
        ),
        "FOREARM_RETAINED_COMPONENT_INVALID",
    )

    unresolved = document["unresolved_items"]
    blocking_codes = {
        (link_name, component["component_id"], code)
        for link_name, link in document["links"].items()
        for component in link["components"]
        if component["blocking"]
        for code in component.get("error_codes", ["COMPONENT_COM_UNRESOLVED"])
    }
    unresolved_codes = {
        (item["owner_link"], item["component_id"], item["code"])
        for item in unresolved
    }
    require(unresolved_codes == blocking_codes, "UNRESOLVED_ITEMS_NOT_DERIVED_FROM_BLOCKING_COMPONENTS")
    require(
        any(code == "UPPER_ARM_PRINT_COM_GEOMETRY_UNRESOLVED" for _, _, code in unresolved_codes),
        "UPPER_ARM_BLOCKING_CODE_MISSING",
    )
    require(document["links"]["link2"]["com_xyz_m_in_link_frame"] is None, "LINK2_COM_MUST_BE_UNRESOLVED")
    require(document["validation"]["resolved_link_com_count"] == 5, "RESOLVED_LINK_COUNT_MISMATCH")
    require(document["validation"]["all_links_resolved"] is False, "ALL_LINKS_RESOLVED_MUST_BE_FALSE")
    require(document["validation"]["all_assertions_pass"] is False, "ALL_ASSERTIONS_PASS_MUST_BE_FALSE")
    require(document["final_status"] == "V15.15 COM_LEDGER_V1 = FAIL", "FAIL_CLOSED_STATUS_MISMATCH")
    require(document["geometry_policy"]["collision_proxy_used"] is False, "COLLISION_PROXY_POLICY_FAIL")
    require(document["prohibited_outputs"]["mass_modified"] is False, "MASS_MODIFIED")
    require(document["prohibited_outputs"]["kinematics_tf_control_modified"] is False, "KINEMATICS_MODIFIED")
    require(document["prohibited_outputs"]["inertia_tensor_written"] is False, "INERTIA_WRITTEN")
    require(document["prohibited_outputs"]["gravity_written"] is False, "GRAVITY_WRITTEN")
    require(
        abs(max(roundtrips) - float(document["validation"]["coordinate_round_trip_max_error_m"])) <= 1.0e-30,
        "MAX_ROUNDTRIP_MISMATCH",
    )
    reject_forbidden_numeric_keys(document["links"])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifact-integrity",
        action="store_true",
        help="return success when deterministic artifact/numeric validation passes even if COM acceptance is FAIL",
    )
    arguments = parser.parse_args()
    try:
        if builder.App is None or builder.Mesh is None:
            raise ValidationError(f"FREECAD_RUNTIME_REQUIRED:{builder.FREECAD_IMPORT_ERROR}")
        root = builder.repo_root(Path(__file__).resolve())
        builder.git_guard(root)
        builder.verify_protected(root)
        json_path = root / builder.OUTPUT_JSON
        markdown_path = root / builder.OUTPUT_MD
        require(json_path.is_file(), "COM_JSON_MISSING")
        require(markdown_path.is_file(), "COM_MARKDOWN_MISSING")
        actual = json.loads(json_path.read_text(encoding="utf-8"), parse_float=Decimal)
        validate_artifact(actual)

        expected = builder.build_document(root)
        validate_artifact(expected)
        expected_json = builder.json_text(expected)
        expected_markdown = builder.markdown_text(expected)
        require(json_path.read_text(encoding="utf-8") == expected_json, "COM_JSON_NOT_DETERMINISTIC")
        require(markdown_path.read_text(encoding="utf-8") == expected_markdown, "COM_MARKDOWN_NOT_DETERMINISTIC")
        require(builder.changed_paths(root) == builder.ALLOWED_CHANGED_PATHS, "CHANGED_PATH_ALLOWLIST_FAIL")

        print("V15.15 COM artifact integrity = PASS")
        print("Mass and resolved-geometry numeric gates = PASS")
        print(expected["final_status"])
        print("Blocking unresolved: UPPER_ARM_PRINT_COM_GEOMETRY_UNRESOLVED")
        if expected["final_status"] != "V15.15 COM_LEDGER_V1 = PASS" and not arguments.artifact_integrity:
            print("COM acceptance exit code = 1 (use --artifact-integrity for deterministic artifact-only validation)")
            return 1
        return 0
    except (ValidationError, builder.AuditError, OSError, KeyError, ValueError) as exc:
        print(f"COM_LEDGER_VALIDATION_ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
