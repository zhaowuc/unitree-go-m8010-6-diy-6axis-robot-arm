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
            if (
                lowered in FORBIDDEN_NUMERIC_KEYS
                and isinstance(child, (int, float, Decimal))
                and not isinstance(child, bool)
            ):
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
        frame = link["frame_world_at_zero"]
        origin = [float(value) for value in frame["origin_mm"]]
        rotation = [[float(value) for value in row] for row in frame["rotation_matrix_row_major"]]
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
                recomputed_local = builder.world_to_local_mm(
                    [float(value) for value in component["cad_com_world_mm"]], origin, rotation
                )
                require(
                    max(
                        abs(recomputed_local[axis] - float(component["cad_com_link_mm"][axis]))
                        for axis in range(3)
                    )
                    <= 1.0e-9,
                    f"COMPONENT_WORLD_TO_LINK_RECOMPUTE_FAIL:{component['component_id']}",
                )
                recomputed_world = builder.local_to_world_mm(recomputed_local, origin, rotation)
                recomputed_roundtrip = (
                    math.sqrt(
                        sum(
                            (recomputed_world[axis] - float(component["cad_com_world_mm"][axis])) ** 2
                            for axis in range(3)
                        )
                    )
                    * 0.001
                )
                require(
                    max(
                        abs(
                            recomputed_world[axis] * 0.001
                            - float(component["reconstructed_world_com_m"][axis])
                        )
                        for axis in range(3)
                    )
                    <= 1.0e-12,
                    f"COMPONENT_LINK_TO_WORLD_RECOMPUTE_FAIL:{component['component_id']}",
                )
                error = float(component["round_trip_error_m"])
                require(math.isfinite(error) and error < builder.ROUNDTRIP_TOL_M, f"ROUNDTRIP_FAIL:{component['component_id']}")
                require(
                    abs(error - recomputed_roundtrip) <= 1.0e-15,
                    f"COMPONENT_ROUNDTRIP_RECOMPUTE_FAIL:{component['component_id']}",
                )
                require(component["independent_audit_reference_pass"] is True, f"CAD_WORLD_REFERENCE_FAIL:{component['component_id']}")
                require(
                    float(component["independent_audit_reference_error_mm"]) <= builder.AUDITED_WORLD_COM_TOL_MM,
                    f"CAD_WORLD_REFERENCE_ERROR:{component['component_id']}",
                )
                roundtrips.append(recomputed_roundtrip)

        recomputed_link_world_mm = [
            float(
                sum(
                    Decimal(str(component["mass_kg"]))
                    * Decimal(str(component["cad_com_world_mm"][axis]))
                    for component in link["components"]
                )
                / actual_mass
            )
            for axis in range(3)
        ]
        require(
            max(
                abs(recomputed_link_world_mm[axis] * 0.001 - float(link["com_xyz_m_world_at_zero"][axis]))
                for axis in range(3)
            )
            <= 1.0e-12,
            f"LINK_WORLD_COM_RECOMPUTE_FAIL:{link_name}",
        )
        recomputed_link_local_mm = builder.world_to_local_mm(recomputed_link_world_mm, origin, rotation)
        require(
            max(
                abs(recomputed_link_local_mm[axis] - float(link["com_xyz_mm_in_link_frame"][axis]))
                for axis in range(3)
            )
            <= 1.0e-9,
            f"LINK_LOCAL_COM_RECOMPUTE_FAIL:{link_name}",
        )
        require(
            max(
                abs(recomputed_link_local_mm[axis] * 0.001 - float(link["com_xyz_m_in_link_frame"][axis]))
                for axis in range(3)
            )
            <= 1.0e-12,
            f"LINK_LOCAL_COM_UNIT_CONVERSION_FAIL:{link_name}",
        )
        link_reconstructed_world = builder.local_to_world_mm(recomputed_link_local_mm, origin, rotation)
        link_roundtrip = (
            math.sqrt(
                sum(
                    (link_reconstructed_world[axis] - recomputed_link_world_mm[axis]) ** 2
                    for axis in range(3)
                )
            )
            * 0.001
        )
        require(link_roundtrip < builder.ROUNDTRIP_TOL_M, f"LINK_ROUNDTRIP_FAIL:{link_name}")
        require(link["validation"]["all_components_resolved"] is True, f"LINK_COMPONENT_UNRESOLVED:{link_name}")
        require(link["validation"]["com_finite"] is True, f"LINK_COM_NONFINITE:{link_name}")
        require(link["validation"]["com_inside_combined_component_bbox"] is True, f"LINK_COM_BBOX_FAIL:{link_name}")
        require(link["validation"]["pass"] is True, f"LINK_VALIDATION_FLAG_FALSE:{link_name}")
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
    require(unresolved_codes == set(), "UNRESOLVED_ITEMS_REMAIN")
    require(builder.finite_vector(document["links"]["link2"]["com_xyz_m_in_link_frame"]), "LINK2_COM_UNRESOLVED")
    require(document["validation"]["resolved_link_com_count"] == 6, "RESOLVED_LINK_COUNT_MISMATCH")
    require(document["validation"]["all_links_resolved"] is True, "ALL_LINKS_RESOLVED_FALSE")
    require(document["validation"]["all_link_numeric_pass"] is True, "LINK_NUMERIC_PASS_FALSE")
    require(document["validation"]["all_assertions_pass"] is True, "ALL_ASSERTIONS_PASS_FALSE")
    require(document["final_status"] == "V15.15 COM_LEDGER_V1 = PASS", "FINAL_PASS_STATUS_MISMATCH")
    require(document["geometry_policy"]["collision_proxy_used"] is False, "COLLISION_PROXY_POLICY_FAIL")
    require(
        all(value is False for value in document["prohibited_outputs"].values()),
        "PROHIBITED_OUTPUT_WAS_WRITTEN",
    )
    require(
        abs(max(roundtrips) - float(document["validation"]["coordinate_round_trip_max_error_m"])) <= 1.0e-30,
        "MAX_ROUNDTRIP_MISMATCH",
    )
    upper = next(
        component
        for component in document["links"]["link2"]["components"]
        if component["component_id"] == "UPPER_ARM_PRINT_MEASURED"
    )
    require(
        upper["com_status"] == "MEASURED_TOTAL_MASS_WITH_EQUAL_DENSITY_VOLUME_WEIGHTED_GEOMETRIC_COM",
        "UPPER_ARM_COM_STATUS_MISMATCH",
    )
    allocation = upper["equal_density_volume_allocation"]
    require(allocation["pass"] is True and allocation["fifty_fifty_used"] is False, "UPPER_ARM_ALLOCATION_POLICY_FAIL")
    parts = allocation["parts"]
    require(len(parts) == 2, "UPPER_ARM_PART_COUNT_FAIL")
    volumes = [Decimal(str(item["volume_mm3"])) for item in parts]
    total_volume = sum(volumes)
    require(total_volume > Decimal("0"), "UPPER_ARM_NONPOSITIVE_TOTAL_VOLUME")
    expected_fractions = [volume / total_volume for volume in volumes]
    expected_allocated_masses = [Decimal("0.5515") * fraction for fraction in expected_fractions]
    require(
        abs(Decimal(str(allocation["total_volume_mm3"])) - total_volume) <= Decimal("1e-6"),
        "UPPER_ARM_TOTAL_VOLUME_RECOMPUTE_FAIL",
    )
    for index, item in enumerate(parts):
        require(
            abs(Decimal(str(item["volume_fraction"])) - expected_fractions[index]) <= Decimal("1e-12"),
            f"UPPER_ARM_VOLUME_FRACTION_RECOMPUTE_FAIL:{index}",
        )
        require(
            abs(Decimal(str(item["allocated_mass_kg"])) - expected_allocated_masses[index])
            <= builder.MASS_TOL_KG,
            f"UPPER_ARM_ALLOCATED_MASS_RECOMPUTE_FAIL:{index}",
        )
    require(abs(Decimal(str(allocation["allocated_mass_sum_kg"])) - Decimal("0.5515")) <= builder.MASS_TOL_KG, "UPPER_ARM_ALLOCATED_MASS_SUM_FAIL")
    require(abs(sum(Decimal(str(item["volume_fraction"])) for item in parts) - Decimal("1")) <= Decimal("1e-12"), "UPPER_ARM_VOLUME_FRACTION_SUM_FAIL")
    recomputed_upper_world_mm = [
        float(
            sum(
                volumes[index] * Decimal(str(parts[index]["center_world_mm"][axis]))
                for index in range(2)
            )
            / total_volume
        )
        for axis in range(3)
    ]
    require(
        max(
            abs(recomputed_upper_world_mm[axis] - float(upper["cad_com_world_mm"][axis]))
            for axis in range(3)
        )
        <= 1.0e-9,
        "UPPER_ARM_WORLD_COM_RECOMPUTE_FAIL",
    )
    require(upper["mass_properties_artifact"]["sha256"] == builder.UPPER_A_MASS_GEOMETRY_SHA256, "UPPER_A_ARTIFACT_HASH_FAIL")
    link2 = document["links"]["link2"]
    require([Decimal(str(item["mass_kg"])) for item in link2["components"]] == [Decimal("0.07"), Decimal("0.07"), Decimal("0.5515"), Decimal("0.07")], "LINK2_COMPONENT_MASSES_CHANGED")
    require(Decimal(str(link2["mass_kg"])) == Decimal("0.7615"), "LINK2_MASS_CHANGED")
    require(all(not component["blocking"] for link in document["links"].values() for component in link["components"]), "BLOCKING_COMPONENT_REMAINS")
    reject_forbidden_numeric_keys(document)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--artifact-integrity",
        action="store_true",
        help="return success when deterministic artifact/numeric validation passes even if COM acceptance is FAIL",
    )
    arguments = parser.parse_args()
    try:
        if builder.App is None or builder.Mesh is None or builder.Part is None:
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
        print("Blocking unresolved: none")
        if expected["final_status"] != "V15.15 COM_LEDGER_V1 = PASS" and not arguments.artifact_integrity:
            print("COM acceptance exit code = 1 (use --artifact-integrity for deterministic artifact-only validation)")
            return 1
        return 0
    except (ValidationError, builder.AuditError, OSError, KeyError, ValueError) as exc:
        print(f"COM_LEDGER_VALIDATION_ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
