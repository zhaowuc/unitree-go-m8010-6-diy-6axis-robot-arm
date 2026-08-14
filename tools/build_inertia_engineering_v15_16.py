#!/usr/bin/env python3
"""Build/check the bounded V15.16 Engineering V1 rigid-inertia decision.

The implementation intentionally reuses the protected V15.16 V2 inertia
mathematics and its already-verified Path-A/canonical-print mass properties.
Only BRep components with known occupied-volume overlap are recomputed.  Each
such component runs in an isolated subprocess with a 60 s hard timeout; a
timeout or ordinary OCCT failure selects a bounded engineering estimate rather
than any mesh refinement or geometry repair.

This tool never writes CAD, URDF, Xacro, MJCF, ros2_control, gravity, armature,
friction, damping, or collision-proxy data.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    import FreeCAD as App
    import numpy as np
except ImportError as exc:  # pragma: no cover - runtime environment gate
    raise SystemExit(
        "Run with a FreeCAD Python interpreter that can import FreeCAD: " + str(exc)
    )


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "0fc439ea8b501802f72c5f28695f51fa4c5ae611"
SOURCE_TREE = "5d4b2bd85e248dcda6309cd2e4b3b4cebd4a22bb"
TARGET_BRANCH = "agent/v15-16-inertia-engineering-v1"

MASS_LEDGER = "V15_15_实测质量账本_v1.json"
COM_LEDGER = "V15_15_COM账本_v2.json"
SOURCE_CAD = "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
V2_AUDIT = "V15_16_刚体惯量_FAIL审计_v2.json"
A1_AUDIT = "V15_16_质量几何去重审计_v1.json"
PRINT_REPORT = "V15_16_打印件惯量几何适用性报告.json"
PRIOR_BUILDER = "tools/build_inertia_ledger_v15_16_v2.py"

ACCEPTANCE_JSON = "V15_16_Engineering惯量验收_v1.json"
ACCEPTANCE_MD = "V15_16_Engineering惯量验收_v1.md"
FREEZE_JSON = "V15_16_刚体惯量_Engineering_V1.json"
FREEZE_MD = "V15_16_刚体惯量_Engineering_V1.md"
BUILDER = "tools/build_inertia_engineering_v15_16.py"
VALIDATOR = "tools/validate_inertia_engineering_v15_16.py"

PASS_ALLOWED_CHANGED_PATHS = {
    ACCEPTANCE_JSON,
    ACCEPTANCE_MD,
    FREEZE_JSON,
    FREEZE_MD,
    BUILDER,
    VALIDATOR,
}
FAIL_ALLOWED_CHANGED_PATHS = {
    ACCEPTANCE_JSON,
    ACCEPTANCE_MD,
    BUILDER,
    VALIDATOR,
}

PROTECTED_HASHES = {
    MASS_LEDGER: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_实测质量账本_v1.md": "fa52587a3309ea5665197283b62ac2e165e67663070408931611fa3b7b4e946d",
    COM_LEDGER: "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    "V15_15_COM账本_v2.md": "64117a409bc37b70e436eef5fee18aa94b21239c3ccdf127adafc5f550c0e59f",
    "V15_15_COM账本_v1.json": "13e3470821541d7ae5c3f10223c51339d7d46df0d118a8e4dbeac2480c462d32",
    "V15_15_COM账本_v1.md": "8875ff34ab6c091b2d5de6fff418553b0834a141adf32f4c88aac7d1d916d11a",
    SOURCE_CAD: "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0",
    "rigid_links_v15_13/rigid_link_membership.csv": "3d2a4d52d679eb97917410c6e60bec22c0c269f91ed31b35fd53ff94167e1b8c",
    "rigid_links_v15_13/rigid_link_manifest.json": "8db76f9228f7a60bd017276239e3669de3cfd443c8cd36d0dd555e184606384f",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl": "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json": "a2b58c9892797b26c4511ff2581e7224d99cd27ca2261cc864252a6e38c21815",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties.stl": "bafb8dc8b06242a844196bc63971f95a0a7dd52834b2a05ab72f40b28460a081",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties_report.json": "dbe4c0751ab7bf4c1e564ce8b078f6de96336aaa203f060b402cc68fa6483e0f",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties.stl": "631217db4ad44f53313aa4691bde3eaaec9e043c9ff83cc6549ea42bd2832760",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties_report.json": "29961b01d574e37ba28c4d7d12cba46b8836911044d29184d1457d7b0160d94f",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl": "585d32a6ac54aca69a082947f31604e5f7b44e5be14eacab2c2db4f45a10f944",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties_report.json": "561d497524b3ed21349caa69f0659f157a2486c50861abe407fad15f8e753614",
    PRIOR_BUILDER: "914d774f9abc4b5aa8638eec9b2533790e5ee446712358fbd33558d69c09e704",
    "tools/validate_inertia_ledger_v15_16_v2.py": "22aadb9d72a3b7e1952bf53c981c889fc251f585174424be8204556595776590",
    "tools/test_inertia_math_v15_16_v2.py": "141fcce0fb2b7af622f4c447bac5f5fa1ce72cf49745fb34aac6adac02c5e7fa",
    V2_AUDIT: "f8e0d62f027c932a49b5b38930ee770fca52c60ca2c3aa64b28308934cca55b8",
    "V15_16_刚体惯量_FAIL审计_v2.md": "94810a472a6e1a651f4a0b614e237d6241bc43bca3d7f66f908065ee26cdcfc7",
    PRINT_REPORT: "7308455fcdab006f68d90401c3206ce7209575feadfd700350122f96dc61cd14",
    A1_AUDIT: "a355735105dc846e982d1476b05f76dba0394ff44572a3d2d31d9567e2b7a73e",
    "V15_16_质量几何去重审计_v1.md": "821146d4a7fe448d471df071857d7758c692dcfbc4ffe929101120a8cb0e05c2",
    "tools/build_mass_geometry_authority_v15_16.py": "5202959aaa9c5b9cee631335fc9ff667ff117c43d3c5ff7d0ae37b34f74318c1",
    "tools/validate_mass_geometry_authority_v15_16.py": "f44fb1f7721d419b3a7314e6e8272c8d7ac7f1ed2fd53d0a5510c8f85da169be",
}

LINK_ORDER = ("link2", "link3", "link4", "link5", "link6", "gripper")
EXPECTED_LINK_MASSES = {
    "link2": 0.7615,
    "link3": 1.0900,
    "link4": 0.6750,
    "link5": 0.5040,
    "link6": 0.1250,
    "gripper": 0.2960,
}
EXPECTED_TOTAL_MASS_KG = 3.4515
SLIVER_COMPONENTS = ("J3_STATOR_EQ", "J4_STATOR_EQ", "J5_STATOR_EQ")
GO_COMPONENTS = (
    "J2A_OUTPUT_EQ",
    "J2B_OUTPUT_EQ",
    "J3_OUTPUT_EQ",
    "J4_OUTPUT_EQ",
    "J5_OUTPUT_EQ",
)
RESIDUAL_COMPONENTS = (
    "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING",
    "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING",
    "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING",
)
PRINT_COMPONENTS = {
    "UPPER_ARM_PRINT_MEASURED",
    "FOREARM_PRINT_MEASURED",
    "WRIST_PRELINK_PRINT_MEASURED",
}

WORKER_TIMEOUT_SECONDS = 60.0
COMPONENT_AUDIT_LIMIT_SECONDS = 300.0
COM_REPRO_LIMIT_M = 1.0e-4
COM_PREFERRED_LIMIT_M = 1.0e-5
OVERLAP_BLOCK_FRACTION = 0.01
SLIVER_FRACTION_LIMIT = 1.0e-6
SLIVER_MASS_LIMIT_MG = 1.0
SLIVER_COM_LIMIT_MM = 0.01
SLIVER_INERTIA_LIMIT_PERCENT = 0.01
SLIVER_SKIP_COMMON_MM3 = 0.02
MM_TO_M = 1.0e-3
MM2_TO_M2 = 1.0e-6
IDENTITY3 = np.eye(3, dtype=float)
WORKER_PREFIX = "V15_16_ENGINEERING_WORKER_JSON="


class AuditFailure(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditFailure(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def canonical_json_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        + "\n"
    ).encode("utf-8")


def load_json(relative: str) -> dict[str, Any]:
    return json.loads((ROOT / relative).read_text(encoding="utf-8"))


def load_prior() -> Any:
    path = ROOT / PRIOR_BUILDER
    spec = importlib.util.spec_from_file_location("v15_16_inertia_v2_protected", path)
    require(spec is not None and spec.loader is not None, "PRIOR_BUILDER_IMPORT_SPEC")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


prior = load_prior()


def git_executable() -> str:
    override = os.environ.get("V15_16_GIT_EXECUTABLE")
    candidates = [
        override,
        shutil.which("git"),
        str(Path.home() / ".codex/tmp/mingit-v15-16a2/runtime/cmd/git.exe"),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(Path(candidate).resolve())
    raise AuditFailure("GIT_EXECUTABLE_NOT_FOUND")


def run_git(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [git_executable(), "-C", str(ROOT), "-c", "core.quotepath=false", *args],
        text=True,
        encoding="utf-8",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=check,
    )


def changed_paths() -> set[str]:
    paths: set[str] = set()
    status = run_git("status", "--porcelain=v1", "--untracked-files=all").stdout
    for line in status.splitlines():
        token = line[3:]
        if " -> " in token:
            token = token.split(" -> ", 1)[1]
        if token:
            paths.add(token.replace("\\", "/"))
    diff = run_git("diff", "--name-only", SOURCE_COMMIT).stdout
    paths.update(line.replace("\\", "/") for line in diff.splitlines() if line)
    return paths


def git_scope_gate() -> dict[str, Any]:
    branch = run_git("branch", "--show-current").stdout.strip()
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    baseline_tree = run_git("rev-parse", f"{SOURCE_COMMIT}^{{tree}}").stdout.strip()
    require(baseline_tree == SOURCE_TREE, f"BASELINE_TREE_MISMATCH:{baseline_tree}")
    ancestor = (
        run_git("merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD", check=False).returncode
        == 0
    )
    require(ancestor, "BASELINE_NOT_ANCESTOR")
    changed = changed_paths()
    unexpected = sorted(changed - PASS_ALLOWED_CHANGED_PATHS)
    require(not unexpected, "UNEXPECTED_CHANGED_PATHS:" + repr(unexpected))
    return {
        "target_branch": TARGET_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "source_commit_is_ancestor": True,
        "allowed_changed_paths_if_pass": sorted(PASS_ALLOWED_CHANGED_PATHS),
        "allowed_changed_paths_if_fail": sorted(FAIL_ALLOWED_CHANGED_PATHS),
        "unexpected_changed_paths": [],
        "pass": True,
    }


def protected_hash_snapshot() -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for relative, expected in sorted(PROTECTED_HASHES.items()):
        path = ROOT / relative
        require(path.is_file(), f"PROTECTED_INPUT_MISSING:{relative}")
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_HASH_MISMATCH:{relative}:{actual}")
        result.append(
            {
                "path": relative,
                "sha256": actual,
                "expected_sha256": expected,
                "locked": True,
            }
        )
    return result


def v3(value: Any) -> np.ndarray:
    if all(hasattr(value, name) for name in ("x", "y", "z")):
        return np.array([float(value.x), float(value.y), float(value.z)], dtype=float)
    return np.asarray(value, dtype=float).reshape(3)


def symmetrize(value: np.ndarray) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    return 0.5 * (matrix + matrix.T)


def frobenius(value: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(value, dtype=float), ord="fro"))


def parallel_axis(inertia_com: np.ndarray, mass_kg: float, displacement_m: np.ndarray) -> np.ndarray:
    d = v3(displacement_m)
    return symmetrize(
        np.asarray(inertia_com, dtype=float)
        + float(mass_kg) * ((float(d @ d) * IDENTITY3) - np.outer(d, d))
    )


def bbox_intersects(left: Any, right: Any, tolerance: float = 1.0e-9) -> bool:
    a = left.BoundBox
    b = right.BoundBox
    return not (
        a.XMax < b.XMin - tolerance
        or b.XMax < a.XMin - tolerance
        or a.YMax < b.YMin - tolerance
        or b.YMax < a.YMin - tolerance
        or a.ZMax < b.ZMin - tolerance
        or b.ZMax < a.ZMin - tolerance
    )


def bbox_corners(box: Any) -> Iterable[np.ndarray]:
    for x in (box.XMin, box.XMax):
        for y in (box.YMin, box.YMax):
            for z in (box.ZMin, box.ZMax):
                yield np.array([x, y, z], dtype=float)


def normalized_result_solids(shape: Any) -> list[Any]:
    if shape.isNull():
        return []
    solids = list(shape.Solids)
    if not solids and shape.ShapeType == "Solid":
        solids = [shape]
    result: list[Any] = []
    for source in solids:
        solid = source.copy()
        if float(solid.Volume) < 0.0:
            solid.reverse()
        if abs(float(solid.Volume)) <= 1.0e-9:
            continue
        require(solid.isValid() and solid.isClosed(), "CUT_RESULT_INVALID_OR_OPEN")
        result.append(solid)
    return result


def raw_inertia_about_reference_mm5(shape: Any, reference_mm: np.ndarray) -> np.ndarray:
    """Unit-density volume inertia of ``shape`` about an arbitrary point."""
    volume = abs(float(shape.Volume))
    displacement = v3(shape.CenterOfMass) - v3(reference_mm)
    return symmetrize(
        prior.matrix_of_inertia(shape)
        + volume
        * (
            float(displacement @ displacement) * IDENTITY3
            - np.outer(displacement, displacement)
        )
    )


def positive_solid_aggregate(shape: Any) -> dict[str, Any]:
    """Mass properties from explicit positive solids, never Compound APIs."""
    solids = list(shape.Solids)
    if not solids and shape.ShapeType == "Solid":
        solids = [shape]
    normalized: list[Any] = []
    for source in solids:
        solid = source.copy()
        if float(solid.Volume) < 0.0:
            solid.reverse()
        require(abs(float(solid.Volume)) > 0.0, "POSITIVE_SOLID_ZERO_VOLUME")
        require(solid.isValid() and solid.isClosed(), "POSITIVE_SOLID_INVALID_OR_OPEN")
        normalized.append(solid)
    require(normalized, "POSITIVE_SOLID_AGGREGATE_EMPTY")
    volume = math.fsum(abs(float(solid.Volume)) for solid in normalized)
    com = np.array(
        [
            math.fsum(
                abs(float(solid.Volume)) * float(v3(solid.CenterOfMass)[axis])
                for solid in normalized
            )
            / volume
            for axis in range(3)
        ],
        dtype=float,
    )
    inertia = np.zeros((3, 3), dtype=float)
    for solid in normalized:
        inertia += raw_inertia_about_reference_mm5(solid, com)
    return {
        "volume_mm3": volume,
        "com_world_mm": com,
        "raw_centroidal_matrix_of_inertia_mm5": symmetrize(inertia),
        "solid_count": len(normalized),
        "compound_volume_mm3_diagnostic": abs(float(shape.Volume)),
    }


def matching_graph_occupied_properties(
    component_id: str,
    atoms: Sequence[dict[str, Any]],
    events: Sequence[dict[str, Any]],
    mass_kg: float,
) -> dict[str, Any]:
    """Exact pairwise inclusion-exclusion for a degree-one overlap graph.

    Every positive-overlap event must be a PARTIAL_INTERPENETRATION and no atom
    may occur in two events.  Therefore there is no possible triple-overlap
    term and pairwise subtraction is exact for the reported occupied set.
    """
    require(events, f"MATCHING_GRAPH_EMPTY:{component_id}")
    degree: dict[str, int] = {}
    for event in events:
        require(
            event["classification"] == "PARTIAL_INTERPENETRATION",
            f"MATCHING_GRAPH_NONPARTIAL:{component_id}",
        )
        for side in ("left", "right"):
            atom_id = str(event[side]["atom_id"])
            degree[atom_id] = degree.get(atom_id, 0) + 1
    require(max(degree.values()) == 1, f"MATCHING_GRAPH_DEGREE_GT_ONE:{component_id}")
    atoms_by_id = {str(atom["atom_id"]): atom for atom in atoms}
    raw_volume = math.fsum(abs(float(atom["_shape"].Volume)) for atom in atoms)
    raw_first = np.zeros(3, dtype=float)
    world_origin = np.zeros(3, dtype=float)
    raw_inertia_origin = np.zeros((3, 3), dtype=float)
    for atom in atoms:
        shape = atom["_shape"]
        raw_first += abs(float(shape.Volume)) * v3(shape.CenterOfMass)
        raw_inertia_origin += raw_inertia_about_reference_mm5(shape, world_origin)

    commons: list[dict[str, Any]] = []
    removed_volume = 0.0
    removed_first = np.zeros(3, dtype=float)
    for event in events:
        left_id = str(event["left"]["atom_id"])
        right_id = str(event["right"]["atom_id"])
        require(left_id in atoms_by_id and right_id in atoms_by_id, "MATCHING_ATOM_MISSING")
        common = atoms_by_id[left_id]["_shape"].common(atoms_by_id[right_id]["_shape"])
        require(not common.isNull(), f"MATCHING_COMMON_NULL:{component_id}")
        aggregate = positive_solid_aggregate(common)
        volume = float(aggregate["volume_mm3"])
        protected_volume = float(event["common_volume_mm3"])
        tolerance = max(1.0e-6, 1.0e-6 * protected_volume)
        require(
            abs(volume - protected_volume) <= tolerance,
            f"MATCHING_COMMON_DRIFT:{component_id}:{volume}:{protected_volume}",
        )
        com = v3(aggregate["com_world_mm"])
        common_first = volume * com
        common_central = np.asarray(
            aggregate["raw_centroidal_matrix_of_inertia_mm5"], dtype=float
        )
        common_inertia_origin = symmetrize(
            common_central
            + volume
            * (float(com @ com) * IDENTITY3 - np.outer(com, com))
        )
        removed_volume += volume
        removed_first += volume * com
        commons.append(
            {
                "shape": common,
                "left_atom_id": left_id,
                "right_atom_id": right_id,
                "volume_mm3": volume,
                "com_world_mm": com.tolist(),
                "first_moment_mm4": common_first.tolist(),
                "inertia_about_world_origin_mm5": common_inertia_origin.tolist(),
                "positive_solid_count": aggregate["solid_count"],
                "compound_volume_mm3_diagnostic": aggregate[
                    "compound_volume_mm3_diagnostic"
                ],
                "raw_centroidal_matrix_of_inertia_mm5": aggregate[
                    "raw_centroidal_matrix_of_inertia_mm5"
                ].tolist(),
            }
        )
    occupied_volume = raw_volume - removed_volume
    require(occupied_volume > 0.0, f"MATCHING_OCCUPIED_NONPOSITIVE:{component_id}")
    occupied_com = (raw_first - removed_first) / occupied_volume
    common_inertia_origin_sum = np.zeros((3, 3), dtype=float)
    for common in commons:
        common_inertia_origin_sum += np.asarray(
            common["inertia_about_world_origin_mm5"], dtype=float
        )
    occupied_first = raw_first - removed_first
    occupied_inertia_origin = symmetrize(
        raw_inertia_origin - common_inertia_origin_sum
    )
    raw_centroidal = symmetrize(
        occupied_inertia_origin
        - occupied_volume
        * (
            float(occupied_com @ occupied_com) * IDENTITY3
            - np.outer(occupied_com, occupied_com)
        )
    )
    raw_centroidal = symmetrize(raw_centroidal)
    density = mass_kg / occupied_volume
    intrinsic = symmetrize(raw_centroidal * density * MM2_TO_M2)
    return {
        "component_id": component_id,
        "method": "NORMAL_OCCT_PAIRWISE_INCLUSION_EXCLUSION_MATCHING_GRAPH_NO_TRIPLE_TERM",
        "engineering_estimate_used": False,
        "raw_volume_mm3": raw_volume,
        "occupied_volume_mm3": occupied_volume,
        "removed_volume_mm3": removed_volume,
        "removed_fraction": removed_volume / raw_volume,
        "geometry_com_world_mm": occupied_com.tolist(),
        "raw_centroidal_matrix_of_inertia_mm5": raw_centroidal.tolist(),
        "geometry_intrinsic_tensor_world_kg_m2": intrinsic.tolist(),
        "effective_density_kg_per_mm3": density,
        "mass_normalization_volume_mm3": occupied_volume,
        "normalized_to_frozen_mass_kg": mass_kg,
        "mass_renormalization_pass": True,
        "containment_actions": [],
        "cut_boolean_count": 0,
        "common_boolean_count": len(commons),
        "matching_graph_max_degree": 1,
        "matching_graph_no_triple_term": True,
        "common_intersections": [
            {key: value for key, value in common.items() if key != "shape"}
            for common in commons
        ],
        "matching_graph_inclusion_exclusion_evidence": {
            "raw_atom_sum_volume_mm3": raw_volume,
            "raw_atom_sum_first_moment_mm4": raw_first.tolist(),
            "raw_atom_sum_inertia_about_world_origin_mm5": symmetrize(
                raw_inertia_origin
            ).tolist(),
            "common_intersections": [
                {key: value for key, value in common.items() if key != "shape"}
                for common in commons
            ],
            "matching_graph_max_degree": 1,
            "matching_graph_no_triple_term": True,
            "occupied_volume_mm3": occupied_volume,
            "occupied_first_moment_mm4": occupied_first.tolist(),
            "occupied_com_world_mm": occupied_com.tolist(),
            "occupied_inertia_about_world_origin_mm5": occupied_inertia_origin.tolist(),
            "occupied_centroidal_volume_inertia_matrix_mm5": raw_centroidal.tolist(),
        },
        "accepted_numerical_sliver_overlaps": [],
        "remaining_unresolved_overlap_volume_upper_bound_mm3": 0.0,
        "remaining_unresolved_overlap_fraction": 0.0,
        "all_nonaccepted_positive_overlap_removed": True,
        "source_geometry_modified": False,
        "mesh_used": False,
    }


def event_map(a1: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {}
    for event in a1["overlap_audit"]["events"]:
        result.setdefault(str(event["component_id"]), []).append(event)
    return result


def component_by_id(components: Sequence[dict[str, Any]], component_id: str) -> dict[str, Any]:
    matches = [item for item in components if item["component_id"] == component_id]
    require(len(matches) == 1, f"COMPONENT_LOOKUP:{component_id}:{len(matches)}")
    return matches[0]


def worker_context(component_id: str) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    _mass, _com, components = prior.load_authorities()
    component = component_by_id(components, component_id)
    document = App.openDocument(str(ROOT / SOURCE_CAD))
    try:
        atoms = prior.brep_component_atoms(document, component)
    finally:
        App.closeDocument(document.Name)
    a1 = load_json(A1_AUDIT)
    events = event_map(a1).get(component_id, [])
    return component, atoms, {"events": events}


def quick_sliver_worker(component_id: str) -> dict[str, Any]:
    require(component_id in SLIVER_COMPONENTS, f"NOT_SLIVER_COMPONENT:{component_id}")
    component, atoms, context = worker_context(component_id)
    events = [event for event in context["events"] if event["classification"] == "NUMERICAL_SLIVER"]
    require(len(events) == 1, f"SLIVER_EVENT_COUNT:{component_id}:{len(events)}")
    event = events[0]
    atoms_by_id = {atom["atom_id"]: atom for atom in atoms}
    left_id = event["left"]["atom_id"]
    right_id = event["right"]["atom_id"]
    require(left_id in atoms_by_id and right_id in atoms_by_id, "SLIVER_ATOM_ID_MISSING")
    left = atoms_by_id[left_id]["_shape"]
    right = atoms_by_id[right_id]["_shape"]
    common = left.common(right)
    require(not common.isNull(), f"SLIVER_COMMON_NULL:{component_id}")
    common_volume = abs(float(common.Volume))
    raw_volume = math.fsum(abs(float(atom["_shape"].Volume)) for atom in atoms)
    frozen_com = v3(component["com_world_mm"])
    r_max_mm = max(
        float(np.linalg.norm(corner - frozen_com))
        for atom in atoms
        for corner in bbox_corners(atom["_shape"].BoundBox)
    )
    return {
        "component_id": component_id,
        "left_atom_id": left_id,
        "right_atom_id": right_id,
        "left_solid_index_one_based": int(event["left"]["source_solid_index_one_based"]),
        "right_solid_index_one_based": int(event["right"]["source_solid_index_one_based"]),
        "common_volume_mm3": common_volume,
        "component_volume_mm3": raw_volume,
        "component_mass_kg": float(component["_authority_mass_kg"]),
        "distance_bound_mm": r_max_mm,
        "method": "FRESH_NORMAL_OCCT_COMMON_VOLUME_NO_MESH_NO_DISTANCE_SCAN",
    }


def occupied_component_worker(component_id: str) -> dict[str, Any]:
    component, atoms, context = worker_context(component_id)
    require(component_id not in PRINT_COMPONENTS, f"PRINT_COMPONENT_WORKER_PROHIBITED:{component_id}")
    events = context["events"]
    raw_volume = math.fsum(abs(float(atom["_shape"].Volume)) for atom in atoms)
    require(raw_volume > 0.0, f"RAW_VOLUME_NONPOSITIVE:{component_id}")

    if component_id in RESIDUAL_COMPONENTS:
        return matching_graph_occupied_properties(
            component_id,
            atoms,
            events,
            float(component["_authority_mass_kg"]),
        )

    drop_ids: set[str] = set()
    containment_actions: list[dict[str, Any]] = []
    accepted_sliver_pairs: set[frozenset[str]] = set()
    atoms_by_id = {str(atom["atom_id"]): atom for atom in atoms}
    for event in events:
        left_id = str(event["left"]["atom_id"])
        right_id = str(event["right"]["atom_id"])
        if event["classification"] == "FULL_CONTAINMENT":
            left_volume = float(event["left"]["metrics"]["volume_mm3"])
            right_volume = float(event["right"]["metrics"]["volume_mm3"])
            drop = left_id if left_volume <= right_volume else right_id
            keep = right_id if drop == left_id else left_id
            common = atoms_by_id[left_id]["_shape"].common(atoms_by_id[right_id]["_shape"])
            require(not common.isNull(), f"CONTAINMENT_COMMON_NULL:{component_id}")
            common_volume = abs(float(common.Volume))
            smaller_volume = min(left_volume, right_volume)
            require(
                abs(common_volume - smaller_volume) <= max(1.0e-6, 1.0e-8 * smaller_volume),
                f"CONTAINMENT_NOT_REPRODUCED:{component_id}:{common_volume}:{smaller_volume}",
            )
            drop_ids.add(drop)
            containment_actions.append(
                {
                    "classification": "FULL_CONTAINMENT",
                    "collapsed_atom_id": drop,
                    "retained_occupied_atom_id": keep,
                    "fresh_common_volume_mm3": common_volume,
                    "smaller_volume_mm3": smaller_volume,
                    "double_count_avoided": True,
                }
            )
        elif event["classification"] == "NUMERICAL_SLIVER":
            accepted_sliver_pairs.add(frozenset((left_id, right_id)))

    survivors = [atom for atom in atoms if atom["atom_id"] not in drop_ids]
    survivors.sort(key=lambda atom: (-abs(float(atom["_shape"].Volume)), atom["atom_id"]))
    accepted: list[dict[str, Any]] = []
    skipped_overlaps: list[dict[str, Any]] = []
    cut_count = 0
    for atom in survivors:
        if component_id in GO_COMPONENTS:
            # The protected all-pairs audit found exactly one positive event in
            # each GO component: the containment confirmed immediately above.
            # After dropping that contained solid, no union Boolean is needed.
            accepted.append(
                {
                    "shape": atom["_shape"].copy(),
                    "source_atom_id": str(atom["atom_id"]),
                }
            )
            continue
        fragments = [
            {
                "shape": atom["_shape"].copy(),
                "source_atom_id": str(atom["atom_id"]),
            }
        ]
        for occupied in accepted:
            next_fragments: list[dict[str, Any]] = []
            for fragment in fragments:
                shape = fragment["shape"]
                if not bbox_intersects(shape, occupied["shape"]):
                    next_fragments.append(fragment)
                    continue
                common = shape.common(occupied["shape"])
                common_volume = 0.0 if common.isNull() else abs(float(common.Volume))
                if common_volume <= 1.0e-9:
                    next_fragments.append(fragment)
                    continue
                source_pair = frozenset(
                    (str(fragment["source_atom_id"]), str(occupied["source_atom_id"]))
                )
                is_accepted_sliver = source_pair in accepted_sliver_pairs
                if is_accepted_sliver and common_volume <= SLIVER_SKIP_COMMON_MM3:
                    skipped_overlaps.append(
                        {
                            "left_atom_id": sorted(source_pair)[0],
                            "right_atom_id": sorted(source_pair)[1],
                            "common_volume_mm3": common_volume,
                            "classification": "ACCEPTED_NUMERICAL_SLIVER_ENGINEERING_V1",
                        }
                    )
                    next_fragments.append(fragment)
                    continue
                before = abs(float(shape.Volume))
                cut = shape.cut(occupied["shape"])
                cut_count += 1
                cut_solids = normalized_result_solids(cut)
                after = math.fsum(abs(float(solid.Volume)) for solid in cut_solids)
                expected = max(0.0, before - common_volume)
                tolerance = max(1.0e-5, 1.0e-8 * before, 1.0e-6 * common_volume)
                require(after <= before + tolerance, f"CUT_VOLUME_INCREASE:{component_id}")
                require(
                    abs(after - expected) <= tolerance,
                    f"CUT_VOLUME_CLOSURE:{component_id}:{after}:{expected}:{tolerance}",
                )
                next_fragments.extend(
                    {
                        "shape": solid,
                        "source_atom_id": str(fragment["source_atom_id"]),
                    }
                    for solid in cut_solids
                )
            fragments = next_fragments
            if not fragments:
                break
        accepted.extend(fragments)

    require(accepted, f"NO_OCCUPIED_SOLIDS:{component_id}")
    occupied_volume = math.fsum(abs(float(item["shape"].Volume)) for item in accepted)
    require(0.0 < occupied_volume <= raw_volume + max(1.0e-5, 1.0e-8 * raw_volume), "OCCUPIED_VOLUME_RANGE")
    geometry_com = np.array(
        [
            math.fsum(
                abs(float(item["shape"].Volume))
                * float(v3(item["shape"].CenterOfMass)[axis])
                for item in accepted
            )
            / occupied_volume
            for axis in range(3)
        ],
        dtype=float,
    )
    mass_kg = float(component["_authority_mass_kg"])
    density = mass_kg / occupied_volume
    raw_centroidal = np.zeros((3, 3), dtype=float)
    for item in accepted:
        shape = item["shape"]
        volume = abs(float(shape.Volume))
        raw_centroidal += raw_inertia_about_reference_mm5(shape, geometry_com)
    raw_centroidal = symmetrize(raw_centroidal)
    intrinsic = symmetrize(raw_centroidal * density * MM2_TO_M2)
    require(np.all(np.isfinite(intrinsic)), f"OCCUPIED_INERTIA_NONFINITE:{component_id}")
    removed_volume = max(0.0, raw_volume - occupied_volume)
    remaining = math.fsum(float(item["common_volume_mm3"]) for item in skipped_overlaps)
    return {
        "component_id": component_id,
        "method": "NORMAL_OCCT_SEQUENTIAL_OCCUPIED_PARTITION_SHALLOW_VALIDITY",
        "engineering_estimate_used": False,
        "raw_volume_mm3": raw_volume,
        "occupied_volume_mm3": occupied_volume,
        "removed_volume_mm3": removed_volume,
        "removed_fraction": removed_volume / raw_volume,
        "geometry_com_world_mm": geometry_com.tolist(),
        "raw_centroidal_matrix_of_inertia_mm5": raw_centroidal.tolist(),
        "geometry_intrinsic_tensor_world_kg_m2": intrinsic.tolist(),
        "effective_density_kg_per_mm3": density,
        "mass_normalization_volume_mm3": occupied_volume,
        "normalized_to_frozen_mass_kg": mass_kg,
        "mass_renormalization_pass": True,
        "containment_actions": containment_actions,
        "cut_boolean_count": cut_count,
        "common_boolean_count": len(containment_actions),
        "accepted_numerical_sliver_overlaps": skipped_overlaps,
        "remaining_unresolved_overlap_volume_upper_bound_mm3": remaining,
        "remaining_unresolved_overlap_fraction": remaining / raw_volume,
        "all_nonaccepted_positive_overlap_removed": True,
        "source_geometry_modified": False,
        "mesh_used": False,
    }


def run_worker(mode: str, component_id: str) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    flag = "--sliver-worker" if mode == "sliver" else "--component-worker"
    command = [sys.executable, str(Path(__file__).resolve()), flag, component_id]
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=str(ROOT),
            text=True,
            encoding="utf-8",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=WORKER_TIMEOUT_SECONDS,
        )
        elapsed = time.monotonic() - started
        lines = [line for line in completed.stdout.splitlines() if line.startswith(WORKER_PREFIX)]
        if completed.returncode != 0 or len(lines) != 1:
            return None, {
                "status": "FAILED",
                "exit_code": completed.returncode,
                "reason": "WORKER_DID_NOT_RETURN_ONE_STRUCTURED_RESULT",
                "elapsed_seconds_runtime_only_not_serialized": elapsed,
                "output_tail": completed.stdout[-2000:],
            }
        return json.loads(lines[0][len(WORKER_PREFIX) :]), {
            "status": "PASS",
            "exit_code": 0,
            "elapsed_seconds_runtime_only_not_serialized": elapsed,
        }
    except subprocess.TimeoutExpired as exc:
        elapsed = time.monotonic() - started
        return None, {
            "status": "TIMEOUT_ENGINEERING_FALLBACK_REQUIRED",
            "exit_code": None,
            "reason": f"HARD_TIMEOUT_{WORKER_TIMEOUT_SECONDS:g}_SECONDS",
            "elapsed_seconds_runtime_only_not_serialized": elapsed,
            "output_tail": str(exc.stdout or "")[-2000:],
        }


def deterministic_worker_record(runtime: dict[str, Any]) -> dict[str, Any]:
    return {
        "status": runtime["status"],
        "exit_code": runtime.get("exit_code"),
        "reason": runtime.get("reason"),
        "hard_timeout_seconds": WORKER_TIMEOUT_SECONDS,
    }


def v2_component_map(v2: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["component_id"]): item for item in v2["components"]}


def a1_component_map(a1: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["component_id"]): item for item in a1["components"]}


def raw_v2_intrinsic(v2_record: dict[str, Any]) -> np.ndarray:
    return symmetrize(
        np.asarray(
            v2_record["path_a_occt"]["inertia_about_component_frozen_com_world_kg_m2"],
            dtype=float,
        )
    )


def fallback_geometry(
    component: dict[str, Any],
    v2_record: dict[str, Any],
    events: Sequence[dict[str, Any]],
    runtime: dict[str, Any],
) -> dict[str, Any]:
    raw_volume = float(v2_record["path_a_occt"]["volume_mm3"])
    removable = [
        event
        for event in events
        if event["classification"] in {"FULL_CONTAINMENT", "PARTIAL_INTERPENETRATION"}
    ]
    removed_upper = min(
        raw_volume * 0.999999,
        math.fsum(float(event["common_volume_mm3"]) for event in removable),
    )
    occupied = raw_volume - removed_upper
    overlap_fraction = removed_upper / raw_volume
    scale = (occupied / raw_volume) ** (2.0 / 3.0)
    intrinsic = raw_v2_intrinsic(v2_record) * scale
    density = float(component["_authority_mass_kg"]) / occupied
    raw_centroidal = intrinsic / (density * MM2_TO_M2)
    accepted_sliver_volume = math.fsum(
        float(event["common_volume_mm3"])
        for event in events
        if event["classification"] == "NUMERICAL_SLIVER"
    )
    return {
        "component_id": component["component_id"],
        "method": "ENGINEERING_OCCUPIED_VOLUME_ESTIMATE_PAIRWISE_IE_UPPER_BOUND_EMPIRICAL_ISOTROPIC_SIMILAR_SHAPE_TENSOR_SCALE",
        "engineering_estimate_used": True,
        "worker": deterministic_worker_record(runtime),
        "raw_volume_mm3": raw_volume,
        "occupied_volume_mm3": occupied,
        "removed_volume_mm3": removed_upper,
        "removed_fraction": overlap_fraction,
        "geometry_com_world_mm": list(component["com_world_mm"]),
        "raw_centroidal_matrix_of_inertia_mm5": raw_centroidal.tolist(),
        "geometry_intrinsic_tensor_world_kg_m2": symmetrize(intrinsic).tolist(),
        "effective_density_kg_per_mm3": density,
        "mass_normalization_volume_mm3": occupied,
        "normalized_to_frozen_mass_kg": float(component["_authority_mass_kg"]),
        "mass_renormalization_pass": True,
        "containment_actions": [
            {
                "classification": "FULL_CONTAINMENT",
                "collapsed_atom_id": (
                    event["left"]["atom_id"]
                    if float(event["left"]["metrics"]["volume_mm3"])
                    <= float(event["right"]["metrics"]["volume_mm3"])
                    else event["right"]["atom_id"]
                ),
                "retained_occupied_atom_id": (
                    event["right"]["atom_id"]
                    if float(event["left"]["metrics"]["volume_mm3"])
                    <= float(event["right"]["metrics"]["volume_mm3"])
                    else event["left"]["atom_id"]
                ),
                "double_count_avoided": True,
            }
            for event in events
            if event["classification"] == "FULL_CONTAINMENT"
        ],
        "cut_boolean_count": 0,
        "accepted_numerical_sliver_overlaps": [],
        "remaining_unresolved_overlap_volume_upper_bound_mm3": removed_upper
        + accepted_sliver_volume,
        "remaining_unresolved_overlap_fraction": (removed_upper + accepted_sliver_volume)
        / raw_volume,
        "all_nonaccepted_positive_overlap_removed": False,
        "source_geometry_modified": False,
        "mesh_used": False,
        "tensor_estimate_is_conservative_bound": False,
        "empirical_tensor_relative_change_from_raw": 1.0 - scale,
        "model_limitation": "BOUNDED_OCCUPIED_VOLUME_BUT_EMPIRICAL_NONCONSERVATIVE_ISOTROPIC_SIMILAR_SHAPE_SECOND_MOMENT_ESTIMATE_AFTER_NORMAL_OCCT_WORKER_FAILURE",
    }


def reused_geometry(
    component: dict[str, Any],
    v2_record: dict[str, Any],
    events: Sequence[dict[str, Any]],
    is_print: bool,
) -> dict[str, Any]:
    raw_volume = float(v2_record["path_a_occt"]["volume_mm3"])
    overlap_upper = math.fsum(float(event["common_volume_mm3"]) for event in events)
    mass_kg = float(component["_authority_mass_kg"])
    density = mass_kg / raw_volume
    intrinsic = raw_v2_intrinsic(v2_record)
    raw_centroidal = intrinsic / (density * MM2_TO_M2)
    return {
        "component_id": component["component_id"],
        "method": (
            "REUSED_PROTECTED_CANONICAL_PRINT_PATH_A_MASS_PROPERTIES_NO_RECOMPUTE"
            if is_print
            else "REUSED_PROTECTED_V2_PATH_A_NO_KNOWN_POSITIVE_OVERLAP"
        ),
        "engineering_estimate_used": bool(is_print and overlap_upper > 0.0),
        "worker": None,
        "raw_volume_mm3": raw_volume,
        "occupied_volume_mm3": raw_volume,
        "removed_volume_mm3": 0.0,
        "removed_fraction": 0.0,
        "geometry_com_world_mm": None,
        "raw_centroidal_matrix_of_inertia_mm5": raw_centroidal.tolist(),
        "geometry_intrinsic_tensor_world_kg_m2": intrinsic.tolist(),
        "effective_density_kg_per_mm3": density,
        "mass_normalization_volume_mm3": raw_volume,
        "normalized_to_frozen_mass_kg": mass_kg,
        "mass_renormalization_pass": True,
        "containment_actions": [],
        "cut_boolean_count": 0,
        "accepted_numerical_sliver_overlaps": [],
        "remaining_unresolved_overlap_volume_upper_bound_mm3": overlap_upper,
        "remaining_unresolved_overlap_fraction": overlap_upper / raw_volume,
        "all_nonaccepted_positive_overlap_removed": overlap_upper == 0.0,
        "source_geometry_modified": False,
        "mesh_used": False,
        "model_limitation": (
            "CANONICAL_PRINT_GEOMETRY_REUSED_AS_REQUIRED; GEOMETRY_CENTROID_NOT_RECOMPUTED; SMALL_INTERPART_INTERPENETRATION_REMAINS_BELOW_1_PERCENT"
            if is_print and overlap_upper > 0.0
            else (
                "PROTECTED_V2_INTRINSIC_REUSED; GEOMETRY_CENTROID_NOT_RECOMPUTED"
                if is_print
                else None
            )
        ),
    }


def build_sliver_records(
    components: Sequence[dict[str, Any]],
    v2_components: dict[str, dict[str, Any]],
    a1_events: dict[str, list[dict[str, Any]]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    blockers: list[dict[str, Any]] = []
    for component_id in SLIVER_COMPONENTS:
        component = component_by_id(components, component_id)
        result, runtime = run_worker("sliver", component_id)
        if result is None:
            blockers.append(
                {
                    "code": "SLIVER_QUICK_RECOMPUTE_FAILED",
                    "component_id": component_id,
                    "blocking": True,
                    "evidence": deterministic_worker_record(runtime),
                }
            )
            events = [
                event
                for event in a1_events[component_id]
                if event["classification"] == "NUMERICAL_SLIVER"
            ]
            require(len(events) == 1, f"PROTECTED_SLIVER_EVENT:{component_id}")
            event = events[0]
            result = {
                "component_id": component_id,
                "left_atom_id": event["left"]["atom_id"],
                "right_atom_id": event["right"]["atom_id"],
                "left_solid_index_one_based": int(event["left"]["source_solid_index_one_based"]),
                "right_solid_index_one_based": int(event["right"]["source_solid_index_one_based"]),
                "common_volume_mm3": float(event["common_volume_mm3"]),
                "component_volume_mm3": float(v2_components[component_id]["path_a_occt"]["volume_mm3"]),
                "component_mass_kg": float(component["_authority_mass_kg"]),
                "distance_bound_mm": None,
                "method": "PROTECTED_A1_VALUE_FALLBACK_NOT_A_FRESH_RECOMPUTE",
            }
        common_volume = float(result["common_volume_mm3"])
        component_volume = float(result["component_volume_mm3"])
        mass_kg = float(result["component_mass_kg"])
        fraction = common_volume / component_volume
        duplicate_mass_kg = mass_kg * fraction
        reference_norm = frobenius(raw_v2_intrinsic(v2_components[component_id]))
        distance_mm = result.get("distance_bound_mm")
        if distance_mm is None:
            com_bound_mm = None
            inertia_abs = None
            inertia_rel_percent = None
            accepted = False
        else:
            distance_mm = float(distance_mm)
            com_bound_mm = fraction / max(1.0e-30, 1.0 - fraction) * distance_mm
            inertia_abs = 2.0 * duplicate_mass_kg * (distance_mm * MM_TO_M) ** 2
            inertia_rel_percent = inertia_abs / reference_norm * 100.0
            accepted = bool(
                fraction < SLIVER_FRACTION_LIMIT
                and duplicate_mass_kg * 1.0e6 < SLIVER_MASS_LIMIT_MG
                and com_bound_mm < SLIVER_COM_LIMIT_MM
                and inertia_rel_percent < SLIVER_INERTIA_LIMIT_PERCENT
            )
        if not accepted:
            blockers.append(
                {
                    "code": "NUMERICAL_SLIVER_ENGINEERING_GATES_FAILED",
                    "component_id": component_id,
                    "blocking": True,
                }
            )
        records.append(
            {
                **result,
                "worker": deterministic_worker_record(runtime),
                "component_fraction": fraction,
                "estimated_duplicate_mass_kg": duplicate_mass_kg,
                "estimated_duplicate_mass_mg": duplicate_mass_kg * 1.0e6,
                "reference_inertia_frobenius_kg_m2": reference_norm,
                "reference_inertia_frobenius_lower_bound_kg_m2": reference_norm,
                "absolute_inertia_bound_kg_m2": inertia_abs,
                "com_influence_bound_mm": com_bound_mm,
                "inertia_relative_influence_bound_percent": inertia_rel_percent,
                "thresholds": {
                    "component_fraction_strict_less_than": SLIVER_FRACTION_LIMIT,
                    "estimated_duplicate_mass_strict_less_than_mg": SLIVER_MASS_LIMIT_MG,
                    "com_influence_strict_less_than_mm": SLIVER_COM_LIMIT_MM,
                    "inertia_relative_influence_strict_less_than_percent": SLIVER_INERTIA_LIMIT_PERCENT,
                },
                "classification": (
                    "ACCEPTED_NUMERICAL_SLIVER_ENGINEERING_V1" if accepted else "REJECTED"
                ),
                "pass": accepted,
            }
        )
    return records, blockers


def build_component_records(
    components: Sequence[dict[str, Any]],
    v2_components: dict[str, dict[str, Any]],
    a1_events: dict[str, list[dict[str, Any]]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    runtime_log: list[dict[str, Any]] = []
    blockers: list[dict[str, Any]] = []
    exact_worker_components = set(GO_COMPONENTS) | set(RESIDUAL_COMPONENTS) | set(SLIVER_COMPONENTS) | {
        "GRIPPER_CONNECTOR_CAMERA_MEASURED_ROLLUP"
    }
    for component in components:
        component_id = str(component["component_id"])
        v2_record = v2_components[component_id]
        events = a1_events.get(component_id, [])
        is_print = component_id in PRINT_COMPONENTS
        if not events or is_print:
            geometry = reused_geometry(component, v2_record, events, is_print)
        elif component_id in exact_worker_components:
            result, runtime = run_worker("component", component_id)
            runtime_log.append(
                {
                    "component_id": component_id,
                    "worker": deterministic_worker_record(runtime),
                    "actual_elapsed_seconds_runtime_only": runtime[
                        "elapsed_seconds_runtime_only_not_serialized"
                    ],
                }
            )
            if result is None:
                geometry = fallback_geometry(component, v2_record, events, runtime)
            else:
                geometry = dict(result)
                geometry["worker"] = deterministic_worker_record(runtime)
                geometry["model_limitation"] = None
        else:
            runtime = {
                "status": "SKIPPED_BY_BOUNDED_FAST_PATH",
                "exit_code": None,
                "reason": "PROTECTED_PAIRWISE_OVERLAP_UPPER_BOUND_BELOW_1_PERCENT",
            }
            geometry = fallback_geometry(component, v2_record, events, runtime)
        remaining_fraction = float(geometry["remaining_unresolved_overlap_fraction"])
        raw_detected_overlap_fraction = (
            math.fsum(float(event["common_volume_mm3"]) for event in events)
            / float(geometry["raw_volume_mm3"])
        )
        overlap_gate = remaining_fraction <= OVERLAP_BLOCK_FRACTION
        if not overlap_gate:
            blockers.append(
                {
                    "code": "COMPONENT_REMAINING_OVERLAP_FRACTION_GT_1_PERCENT",
                    "component_id": component_id,
                    "fraction": remaining_fraction,
                    "blocking": True,
                }
            )
        intrinsic = symmetrize(
            np.asarray(geometry["geometry_intrinsic_tensor_world_kg_m2"], dtype=float)
        )
        require(np.all(np.isfinite(intrinsic)), f"COMPONENT_INTRINSIC_NONFINITE:{component_id}")
        limitations = [str(value) for value in v2_record.get("limitations", {}) if v2_record["limitations"][value]]
        if geometry.get("model_limitation"):
            limitations.append(str(geometry["model_limitation"]))
        records.append(
            {
                "component_id": component_id,
                "owner_link": component["owner_link"],
                "mass_kg": float(component["_authority_mass_kg"]),
                "frozen_com_world_mm": list(component["com_world_mm"]),
                "geometry_centroid_world_mm": geometry["geometry_com_world_mm"],
                "mass_location_authority": "COM_V2_FROZEN_COMPONENT_COM",
                "applied_frozen_com_anchor_world_mm": list(component["com_world_mm"]),
                "intrinsic_tensor_reference": "ABOUT_GEOMETRY_CENTROID_THEN_TRANSLATION_ONLY_PLACED_AT_FROZEN_COMPONENT_COM_V2",
                "geometry_recentered_to_frozen_com_for_inertia_model": True,
                "raw_centroidal_matrix_of_inertia_mm5": geometry[
                    "raw_centroidal_matrix_of_inertia_mm5"
                ],
                "occupied_centroidal_volume_inertia_matrix_mm5": geometry[
                    "raw_centroidal_matrix_of_inertia_mm5"
                ],
                "geometry_intrinsic_tensor_world_kg_m2": intrinsic.tolist(),
                "raw_volume_mm3": float(geometry["raw_volume_mm3"]),
                "occupied_volume_mm3": float(geometry["occupied_volume_mm3"]),
                "mass_normalization_volume_mm3": float(
                    geometry["mass_normalization_volume_mm3"]
                ),
                "normalized_to_frozen_mass_kg": float(
                    geometry["normalized_to_frozen_mass_kg"]
                ),
                "effective_density_kg_per_mm3": float(
                    geometry["effective_density_kg_per_mm3"]
                ),
                "mass_renormalization_pass": bool(geometry["mass_renormalization_pass"]),
                "removed_volume_mm3": float(geometry["removed_volume_mm3"]),
                "removed_fraction": float(geometry["removed_fraction"]),
                "occupied_volume_method": geometry["method"],
                "method": geometry["method"],
                "correction_method": geometry["method"],
                "raw_detected_overlap_fraction": raw_detected_overlap_fraction,
                "engineering_estimate_used": bool(geometry["engineering_estimate_used"]),
                "worker": geometry.get("worker"),
                "containment_actions": geometry["containment_actions"],
                "accepted_numerical_sliver_overlaps": geometry[
                    "accepted_numerical_sliver_overlaps"
                ],
                "matching_graph_inclusion_exclusion_evidence": geometry.get(
                    "matching_graph_inclusion_exclusion_evidence"
                ),
                "remaining_unresolved_overlap_volume_upper_bound_mm3": float(
                    geometry["remaining_unresolved_overlap_volume_upper_bound_mm3"]
                ),
                "raw_detected_overlap_volume_upper_bound_mm3": math.fsum(
                    float(event["common_volume_mm3"]) for event in events
                ),
                "remaining_unresolved_overlap_fraction": remaining_fraction,
                "remaining_or_estimated_uncertainty_fraction": remaining_fraction,
                "overlap_fraction_limit_max_inclusive": OVERLAP_BLOCK_FRACTION,
                "overlap_gate_pass": overlap_gate,
                "source_geometry_modified": False,
                "collision_proxy_used": False,
                "mesh_used": False,
                "confidence_class": v2_record["source_status"],
                "model_limitations": sorted(set(limitations)),
                "pass": overlap_gate,
            }
        )
    return records, runtime_log, blockers


def build_go_records(
    components: Sequence[dict[str, Any]],
    component_records: Sequence[dict[str, Any]],
    a1_events: dict[str, list[dict[str, Any]]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    component_map = {item["component_id"]: item for item in component_records}
    blockers: list[dict[str, Any]] = []
    result: list[dict[str, Any]] = []
    for component_id in GO_COMPONENTS:
        record = component_map[component_id]
        events = [
            event
            for event in a1_events[component_id]
            if event["classification"] == "FULL_CONTAINMENT"
        ]
        actions = list(record["containment_actions"])
        double_count_avoided = len(events) == 1 and len(actions) == 1 and bool(
            actions[0]["double_count_avoided"]
        )
        passed = bool(double_count_avoided and record["overlap_gate_pass"])
        if not passed:
            blockers.append(
                {
                    "code": "GO_OUTPUT_FULL_CONTAINMENT_NOT_CORRECTED",
                    "component_id": component_id,
                    "blocking": True,
                }
            )
        result.append(
            {
                "component_id": component_id,
                "classification": "FULL_CONTAINMENT",
                "raw_volume_mm3": record["raw_volume_mm3"],
                "occupied_volume_mm3": record["occupied_volume_mm3"],
                "removed_volume_mm3": record["removed_volume_mm3"],
                "removed_fraction": record["removed_fraction"],
                "raw_detected_overlap_fraction": record[
                    "raw_detected_overlap_fraction"
                ],
                "method": record["occupied_volume_method"],
                "correction_method": record["correction_method"],
                "remaining_or_estimated_uncertainty_fraction": record[
                    "remaining_or_estimated_uncertainty_fraction"
                ],
                "engineering_estimate_used": record["engineering_estimate_used"],
                "effective_density_kg_per_mm3": record[
                    "effective_density_kg_per_mm3"
                ],
                "mass_normalization_volume_mm3": record[
                    "mass_normalization_volume_mm3"
                ],
                "normalized_to_frozen_mass_kg": record[
                    "normalized_to_frozen_mass_kg"
                ],
                "mass_renormalization_pass": record["mass_renormalization_pass"],
                "double_count_avoided": double_count_avoided,
                "pass": passed,
            }
        )
    return result, blockers


def build_links(
    com: dict[str, Any], component_records: Sequence[dict[str, Any]]
) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]], list[dict[str, Any]]]:
    component_map = {item["component_id"]: item for item in component_records}
    records: list[dict[str, Any]] = []
    private: dict[str, dict[str, Any]] = {}
    blockers: list[dict[str, Any]] = []
    v2 = load_json(V2_AUDIT)
    v2_link_map = {item["link"]: item for item in v2["links"]}
    for link_name in LINK_ORDER:
        authority = com["links"][link_name]
        frame = authority["frame_world_at_mechanical_zero"]
        origin = v3(frame["origin_mm"])
        rotation = np.asarray(frame["rotation_matrix_row_major"], dtype=float)
        require(
            float(np.max(np.abs(rotation.T @ rotation - IDENTITY3))) < 1.0e-12,
            f"LINK_FRAME_NOT_ORTHONORMAL:{link_name}",
        )
        frozen_com_link = v3(authority["com_link_m"])
        frozen_com_world = v3(authority["com_world_mm"])
        ids = list(authority["component_ids"])
        owned = [component_map[component_id] for component_id in ids]
        mass_kg = math.fsum(float(item["mass_kg"]) for item in owned)
        require(abs(mass_kg - EXPECTED_LINK_MASSES[link_name]) < 1.0e-12, f"LINK_MASS:{link_name}")
        recomputed_world_com = np.array(
            [
                math.fsum(
                    float(item["mass_kg"]) * float(item["frozen_com_world_mm"][axis])
                    for item in owned
                )
                / mass_kg
                for axis in range(3)
            ],
            dtype=float,
        )
        recomputed_link_com = rotation.T @ (recomputed_world_com - origin) * MM_TO_M
        world_error_m = float(np.linalg.norm(recomputed_world_com - frozen_com_world) * MM_TO_M)
        link_error_m = float(np.linalg.norm(recomputed_link_com - frozen_com_link))
        com_error_m = max(world_error_m, link_error_m)
        tensor = np.zeros((3, 3), dtype=float)
        contributions: dict[str, np.ndarray] = {}
        for component in owned:
            component_com_link = (
                rotation.T @ (v3(component["frozen_com_world_mm"]) - origin) * MM_TO_M
            )
            intrinsic_link = symmetrize(
                rotation.T
                @ np.asarray(component["geometry_intrinsic_tensor_world_kg_m2"], dtype=float)
                @ rotation
            )
            contribution = parallel_axis(
                intrinsic_link,
                float(component["mass_kg"]),
                component_com_link - frozen_com_link,
            )
            contributions[component["component_id"]] = contribution
            tensor += contribution
        tensor = symmetrize(tensor)
        r_max_m = float(
            v2_link_map[link_name]["candidate_uniform_proxy_path_a_occt"]["candidate_inertia"][
                "r_max_m"
            ]
        )
        metrics = prior.tensor_metrics(tensor, mass_kg, r_max_m)
        unit_trace_bound = 2.0 * mass_kg * r_max_m * r_max_m
        unit_max_abs_entry = float(np.max(np.abs(tensor)))
        unit_trace_bound_pass = bool(
            float(np.trace(tensor)) <= unit_trace_bound * (1.0 + 1.0e-8)
        )
        unit_magnitude_pass = bool(unit_max_abs_entry < 1.0)
        metrics["unit_audit_trace_upper_bound_kg_m2"] = unit_trace_bound
        metrics["unit_audit_max_abs_tensor_entry_kg_m2"] = unit_max_abs_entry
        metrics["unit_audit_max_abs_entry_limit_strict_less_than_kg_m2"] = 1.0
        metrics["unit_audit_trace_bound_pass"] = unit_trace_bound_pass
        metrics["unit_audit_magnitude_pass"] = unit_magnitude_pass
        metrics["unit_audit_pass"] = unit_trace_bound_pass and unit_magnitude_pass
        physics_pass = bool(metrics["pass"] and metrics["unit_audit_pass"])
        com_pass = com_error_m <= COM_REPRO_LIMIT_M
        if not com_pass:
            blockers.append(
                {
                    "code": "LINK_COM_V2_REPRODUCTION_GT_0P1MM",
                    "link": link_name,
                    "error_m": com_error_m,
                    "blocking": True,
                }
            )
        if not physics_pass:
            blockers.append(
                {
                    "code": "LINK_TENSOR_PHYSICS_CHECK_FAILED",
                    "link": link_name,
                    "blocking": True,
                }
            )
        limitations = sorted(
            {
                limitation
                for component in owned
                for limitation in component["model_limitations"]
            }
        )
        record = {
            "link": link_name,
            "mass_kg": mass_kg,
            "component_ids": ids,
            "com_xyz_m_in_link_frame": frozen_com_link.tolist(),
            "com_world_m": (frozen_com_world * MM_TO_M).tolist(),
            "inertia_tensor_kg_m2": tensor.tolist(),
            "inertia_reference_point": "FROZEN_COM_V2",
            "inertia_expressed_in": "OWNER_LINK_FRAME",
            "r_max_m": r_max_m,
            "com_v2_reproduction": {
                "recomputed_world_com_mm": recomputed_world_com.tolist(),
                "recomputed_link_com_m": recomputed_link_com.tolist(),
                "world_error_m": world_error_m,
                "owner_link_error_m": link_error_m,
                "max_error_m": com_error_m,
                "preferred_limit_m": COM_PREFERRED_LIMIT_M,
                "hard_limit_max_inclusive_m": COM_REPRO_LIMIT_M,
                "preferred_pass": com_error_m < COM_PREFERRED_LIMIT_M,
                "pass": com_pass,
            },
            "tensor_checks": metrics,
            "confidence_class": "ENGINEERING_V1_CANDIDATE",
            "model_limitations": limitations,
            "pass": com_pass and physics_pass,
        }
        records.append(record)
        private[link_name] = {
            "tensor": tensor,
            "record": record,
            "rotation": rotation,
            "origin": origin,
            "frozen_com_link_m": frozen_com_link,
            "component_contributions": contributions,
        }
    return records, private, blockers


def residual_status(percent: float) -> str:
    if percent <= 5.0:
        return "PASS_HIGH_CONFIDENCE"
    if percent <= 10.0:
        return "PASS_WITH_MODEL_LIMITATION"
    if percent <= 15.0:
        return "REVIEW_REQUIRED"
    return "FAIL"


def build_residual_sensitivity(
    components: Sequence[dict[str, Any]],
    component_records: Sequence[dict[str, Any]],
    link_private: dict[str, dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    authority_components = {item["component_id"]: item for item in components}
    records = {item["component_id"]: item for item in component_records}
    output: list[dict[str, Any]] = []
    blockers: list[dict[str, Any]] = []
    for component_id in RESIDUAL_COMPONENTS:
        component = authority_components[component_id]
        record = records[component_id]
        link_name = str(component["owner_link"])
        private = link_private[link_name]
        uniform_link = np.asarray(private["tensor"], dtype=float)
        uniform_contribution = np.asarray(
            private["component_contributions"][component_id], dtype=float
        )
        rotation = np.asarray(private["rotation"], dtype=float)
        origin = v3(private["origin"])
        link_com = v3(private["frozen_com_link_m"])
        component_com_link = (
            rotation.T @ (v3(component["com_world_mm"]) - origin) * MM_TO_M
        )
        component_mass = float(component["_authority_mass_kg"])
        point_contribution = parallel_axis(
            np.zeros((3, 3), dtype=float),
            component_mass,
            component_com_link - link_com,
        )
        point_link = symmetrize(uniform_link - uniform_contribution + point_contribution)
        difference = frobenius(uniform_link - point_link)
        denominator = frobenius(uniform_link)
        require(denominator > 0.0, f"RESIDUAL_DENOMINATOR:{component_id}")
        relative = difference / denominator
        percent = relative * 100.0
        status = residual_status(percent)
        freeze_gate = percent < 10.0
        if not freeze_gate:
            blockers.append(
                {
                    "code": "RESIDUAL_SENSITIVITY_NOT_LT_10_PERCENT",
                    "component_id": component_id,
                    "sensitivity_percent": percent,
                    "status": status,
                    "blocking": True,
                }
            )
        output.append(
            {
                "component_id": component_id,
                "owner_link": link_name,
                "mass_kg": component_mass,
                "uniform_proxy_geometry_model": "OCCUPIED_VOLUME_CORRECTED_TRACEABLE_CAD_UNIFORM_EFFECTIVE_DENSITY",
                "point_model": "POINT_MASS_AT_FROZEN_RESIDUAL_COM_INTRINSIC_INERTIA_ZERO",
                "occupied_volume_method": record["occupied_volume_method"],
                "uniform_proxy_component_contribution_about_link_frozen_com_kg_m2": symmetrize(
                    uniform_contribution
                ).tolist(),
                "point_mass_component_contribution_about_link_frozen_com_kg_m2": symmetrize(
                    point_contribution
                ).tolist(),
                "candidate_final_link_uniform_proxy_tensor_kg_m2": symmetrize(uniform_link).tolist(),
                "candidate_final_link_point_model_tensor_kg_m2": point_link.tolist(),
                "difference_frobenius_kg_m2": difference,
                "denominator_candidate_final_uniform_link_frobenius_kg_m2": denominator,
                "relative_sensitivity": relative,
                "sensitivity_percent": percent,
                "thresholds_percent": {
                    "pass_high_confidence_max_inclusive": 5.0,
                    "pass_with_model_limitation_gt_5_max_inclusive": 10.0,
                    "review_required_gt_10_max_inclusive": 15.0,
                    "fail_strictly_greater_than": 15.0,
                    "freeze_gate_strictly_less_than": 10.0,
                },
                "status": status,
                "freeze_gate_pass": freeze_gate,
                "model_limitations": (
                    ["RESIDUAL_SPATIAL_INERTIA_MODEL_UNCERTAINTY"]
                    if status == "PASS_WITH_MODEL_LIMITATION"
                    else []
                ),
            }
        )
    return output, blockers


def fmt_optional_engineering(value: int | float | None) -> str:
    """Preserve ``.12g`` output for numbers and render unavailable bounds safely."""
    if value is None:
        return "N/A"
    return format(value, ".12g")


def markdown_acceptance(ledger: dict[str, Any]) -> str:
    lines = [
        "# V15.16 Engineering 惯量验收 V1",
        "",
        f"> 结论：**{ledger['final_status']}**",
        "",
        "本报告是第一版动力学模型的工程验收，不是 CAD BRep 数学完美性证明。",
        "Mass V1 和 COM V2 保持不变；未使用 collision proxy，未写入 URDF/MJCF，未开启 gravity。",
        "",
        "## Numerical sliver",
        "",
        "| component | common mm³ | fraction | mass mg | COM mm | inertia % | status |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for item in ledger["sliver_engineering_acceptance"]["items"]:
        lines.append(
            f"| {item['component_id']} | "
            f"{fmt_optional_engineering(item['common_volume_mm3'])} | "
            f"{fmt_optional_engineering(item['component_fraction'])} | "
            f"{fmt_optional_engineering(item['estimated_duplicate_mass_mg'])} | "
            f"{fmt_optional_engineering(item['com_influence_bound_mm'])} | "
            f"{fmt_optional_engineering(item['inertia_relative_influence_bound_percent'])} | "
            f"{item['classification']} |"
        )
    lines.extend(
        [
            "",
            "## GO output FULL_CONTAINMENT",
            "",
            "| component | raw mm³ | occupied mm³ | removed % | method | avoided |",
            "|---|---:|---:|---:|---|---|",
        ]
    )
    for item in ledger["go_output_full_containment"]["items"]:
        lines.append(
            f"| {item['component_id']} | {item['raw_volume_mm3']:.12g} | "
            f"{item['occupied_volume_mm3']:.12g} | {item['removed_fraction'] * 100.0:.12g} | "
            f"{item['method']} | {item['double_count_avoided']} |"
        )
    lines.extend(
        [
            "",
            "## Residual sensitivity",
            "",
            "| residual | link | sensitivity % | status | freeze gate |",
            "|---|---|---:|---|---|",
        ]
    )
    for item in ledger["residual_sensitivity"]:
        lines.append(
            f"| {item['component_id']} | {item['owner_link']} | "
            f"{item['sensitivity_percent']:.12g} | {item['status']} | {item['freeze_gate_pass']} |"
        )
    lines.extend(["", "## Candidate tensors", ""])
    for item in ledger["links"]:
        lines.extend(
            [
                f"### {item['link']}",
                "",
                f"- mass: `{item['mass_kg']:.12g} kg`",
                f"- COM V2 (link m): `{item['com_xyz_m_in_link_frame']}`",
                "- inertia (kg·m²):",
                "",
                "```text",
                *["[" + ", ".join(f"{value:.17g}" for value in row) + "]" for row in item["inertia_tensor_kg_m2"]],
                "```",
                "",
            ]
        )
    lines.extend(
        [
            "## 工程限制",
            "",
            "Residual A/B/C 包含线材、螺钉和转接结构，其空间惯量是 V1 工程估计。",
            "5–10% 的 residual sensitivity 会记录 `RESIDUAL_SPATIAL_INERTIA_MODEL_UNCERTAINTY`，但允许 Engineering V1 冻结。",
            "后续应用实机电流/力矩、重力补偿和摆动辨识进一步修正。",
            "",
            "## Hard unresolved items",
            "",
            "```json",
            json.dumps(ledger["unresolved_items"], ensure_ascii=False, indent=2),
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def build_freeze_ledger(acceptance: dict[str, Any]) -> dict[str, Any]:
    links = []
    residual_map = {item["owner_link"]: item for item in acceptance["residual_sensitivity"]}
    for source in acceptance["links"]:
        limitations = list(source["model_limitations"])
        residual = residual_map.get(source["link"])
        if residual:
            limitations.extend(residual["model_limitations"])
        links.append(
            {
                "link": source["link"],
                "mass_kg": source["mass_kg"],
                "com_xyz_m_in_link_frame": source["com_xyz_m_in_link_frame"],
                "inertia_tensor_kg_m2": source["inertia_tensor_kg_m2"],
                "inertia_reference_point": "FROZEN_COM_V2",
                "inertia_expressed_in": "OWNER_LINK_FRAME",
                "confidence_class": "ENGINEERING_V1",
                "model_limitations": sorted(set(limitations)),
                "tensor_checks": source["tensor_checks"],
            }
        )
    return {
        "schema": "go-m8010-arm-v15.16-rigid-inertia-engineering-v1/1.0",
        "revision": "V15.16-RIGID_BODY_INERTIA-ENGINEERING_V1",
        "status": "ENGINEERING_V1",
        "final_status": "V15.16 INERTIA_ENGINEERING_V1 = PASS",
        "not_measured_inertia": True,
        "not_high_fidelity_identified_dynamics": True,
        "authorities": acceptance["authorities"],
        "total_mass_kg": acceptance["validation"]["total_mass_kg"],
        "links": links,
        "residual_sensitivity": acceptance["residual_sensitivity"],
        "collision_proxy_used": False,
        "deployment_performed": False,
        "gravity_enabled": False,
        "unresolved_items": [],
    }


def markdown_freeze(ledger: dict[str, Any]) -> str:
    lines = [
        "# V15.16 刚体惯量 Engineering V1",
        "",
        "> 状态：**ENGINEERING_V1**。这不是实测惯量，也不是高保真辨识动力学。",
        "",
        f"- total mass: `{ledger['total_mass_kg']:.12g} kg`",
        "- Mass authority: `V15_15_实测质量账本_v1.json`",
        "- COM authority: `V15_15_COM账本_v2.json`",
        "- collision proxy / deployment / gravity: `NO / NO / NO`",
        "",
    ]
    for item in ledger["links"]:
        lines.extend(
            [
                f"## {item['link']}",
                "",
                f"- mass: `{item['mass_kg']:.12g} kg`",
                f"- COM (link m): `{item['com_xyz_m_in_link_frame']}`",
                f"- confidence: `{item['confidence_class']}`",
                f"- limitations: `{item['model_limitations']}`",
                "- inertia tensor (kg·m²):",
                "",
                "```text",
                *["[" + ", ".join(f"{value:.17g}" for value in row) + "]" for row in item["inertia_tensor_kg_m2"]],
                "```",
                "",
            ]
        )
    return "\n".join(lines)


def build_outputs() -> tuple[dict[str, bytes], dict[str, Any], list[dict[str, Any]]]:
    build_started = time.monotonic()
    git_guard = git_scope_gate()
    protected_before = protected_hash_snapshot()
    mass, com, components = prior.load_authorities()
    v2 = load_json(V2_AUDIT)
    a1 = load_json(A1_AUDIT)
    print_report = load_json(PRINT_REPORT)
    require(v2["unit_audit"]["pass"], "PROTECTED_V2_UNIT_AUDIT_NOT_PASS")
    require(v2["math_test"]["pass"], "PROTECTED_V2_MATH_TEST_NOT_PASS")
    require(v2["occt_matrix_semantics_gate"]["pass"], "PROTECTED_V2_OCCT_SEMANTICS_NOT_PASS")
    require(print_report["all_four_pass"], "PROTECTED_PRINT_REPORT_NOT_PASS")
    require(len(components) == 17, "COMPONENT_COUNT_NOT_17")
    v2_components = v2_component_map(v2)
    events = event_map(a1)

    slivers, sliver_blockers = build_sliver_records(components, v2_components, events)
    component_records, runtime_log, component_blockers = build_component_records(
        components, v2_components, events
    )
    go_records, go_blockers = build_go_records(components, component_records, events)
    links, link_private, link_blockers = build_links(com, component_records)
    residual, residual_blockers = build_residual_sensitivity(
        components, component_records, link_private
    )
    blockers = (
        sliver_blockers
        + component_blockers
        + go_blockers
        + link_blockers
        + residual_blockers
    )

    protected_after = protected_hash_snapshot()
    require(protected_before == protected_after, "PROTECTED_INPUT_CHANGED_DURING_BUILD")
    total_mass = math.fsum(float(item["mass_kg"]) for item in links)
    total_mass_pass = abs(total_mass - EXPECTED_TOTAL_MASS_KG) < 1.0e-12
    if not total_mass_pass:
        blockers.append(
            {
                "code": "TOTAL_MASS_NOT_3P4515_KG",
                "actual_kg": total_mass,
                "blocking": True,
            }
        )
    max_com = max(float(item["com_v2_reproduction"]["max_error_m"]) for item in links)
    max_remaining = max(
        float(item["remaining_unresolved_overlap_fraction"])
        for item in component_records
    )
    all_slivers = all(item["pass"] for item in slivers)
    all_go = len(go_records) == 5 and all(item["pass"] for item in go_records)
    all_tensors = all(item["tensor_checks"]["pass"] for item in links)
    all_link_unit_audits = all(
        item["tensor_checks"]["unit_audit_pass"] for item in links
    )
    all_residual = all(item["freeze_gate_pass"] for item in residual)
    all_gates = bool(
        total_mass_pass
        and max_com <= COM_REPRO_LIMIT_M
        and print_report["all_four_pass"]
        and all_slivers
        and all_go
        and max_remaining <= OVERLAP_BLOCK_FRACTION
        and all_tensors
        and all_link_unit_audits
        and all_residual
        and not blockers
    )
    final_status = (
        "V15.16 INERTIA_ENGINEERING_V1 = PASS"
        if all_gates
        else "V15.16 INERTIA_ENGINEERING_V1 = FAIL"
    )

    acceptance: dict[str, Any] = {
        "schema": "go-m8010-arm-v15.16-inertia-engineering-acceptance-v1/1.0",
        "revision": "V15.16B-ENGINEERING_INERTIA_FREEZE_DECISION_V1",
        "scope": "FIRST_RIGID_BODY_DYNAMICS_ENGINEERING_ACCEPTANCE_NO_DEPLOYMENT",
        "final_status": final_status,
        "freeze_permitted": all_gates,
        "authoritative_inertia_written": all_gates,
        "provenance": {
            "source_commit": SOURCE_COMMIT,
            "source_tree": SOURCE_TREE,
            "target_branch": TARGET_BRANCH,
            "runtime": {
                "python": sys.version.split()[0],
                "freecad": ".".join(str(value) for value in App.Version()[:3]),
                "numpy": np.__version__,
            },
        },
        "git_scope_guard": git_guard,
        "authorities": {
            "mass": {
                "path": MASS_LEDGER,
                "sha256": PROTECTED_HASHES[MASS_LEDGER],
                "modified": False,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            },
            "com_v2": {
                "path": COM_LEDGER,
                "sha256": PROTECTED_HASHES[COM_LEDGER],
                "modified": False,
            },
            "protected_v2_inertia_math": {
                "builder_path": PRIOR_BUILDER,
                "builder_sha256": PROTECTED_HASHES[PRIOR_BUILDER],
                "math_test_pass_reused": True,
                "occt_semantics_pass_reused": True,
            },
            "protected_print_geometry": {
                "report_path": PRINT_REPORT,
                "report_sha256": PROTECTED_HASHES[PRINT_REPORT],
                "all_four_pass": True,
                "recomputed": False,
            },
            "protected_overlap_audit": {
                "path": A1_AUDIT,
                "sha256": PROTECTED_HASHES[A1_AUDIT],
                "classification_counts": a1["overlap_audit"]["classification_counts"],
            },
            "protected_inputs_before": protected_before,
            "protected_inputs_after": protected_after,
            "all_protected_inputs_unchanged": True,
        },
        "runtime_budget_policy": {
            "target_total_minutes": [10, 30],
            "component_worker_hard_timeout_seconds": WORKER_TIMEOUT_SECONDS,
            "component_geometry_audit_hard_limit_seconds": COMPONENT_AUDIT_LIMIT_SECONDS,
            "ordinary_boolean_timeout_policy": "ISOLATED_COMPONENT_WORKER_TERMINATED_AT_60_SECONDS_THEN_ENGINEERING_ESTIMATE",
            "mesh_used": False,
            "full_vertex_classification_used": False,
            "brep_distance_scan_used": False,
            "mesh_convergence_used": False,
            "forbidden_linear_deflections_mm_used": [],
            "source_geometry_repair_used": False,
            "runtime_worker_summary": [
                {
                    "component_id": item["component_id"],
                    "worker": item["worker"],
                }
                for item in runtime_log
            ],
        },
        "sliver_engineering_acceptance": {
            "policy": "ACCEPT_IMPACT_BOUNDS_BELOW_ENGINEERING_V1_THRESHOLDS_OVERLAP_NOT_ZERO",
            "items": slivers,
            "accepted_count": sum(item["pass"] for item in slivers),
            "expected_count": 3,
            "all_three_pass": all_slivers,
        },
        "go_output_full_containment": {
            "items": go_records,
            "double_count_avoided_count": sum(item["double_count_avoided"] for item in go_records),
            "expected_count": 5,
            "max_raw_detected_overlap_fraction": max(
                item["raw_detected_overlap_fraction"] for item in go_records
            ),
            "max_remaining_or_estimated_uncertainty_fraction": max(
                item["remaining_or_estimated_uncertainty_fraction"]
                for item in go_records
            ),
            "all_five_pass": all_go,
        },
        "components": component_records,
        "links": links,
        "residual_sensitivity": residual,
        "unit_audit": {
            "cad_length_unit": "mm",
            "mass_unit": "kg",
            "raw_volume_second_integral_unit": "mm^5",
            "effective_density_unit": "kg/mm^3",
            "final_inertia_unit": "kg*m^2",
            "conversion": "raw_mm5*(mass_kg/volume_mm3)*1e-6",
            "protected_math_test_pass": True,
            "protected_occt_semantics_pass": True,
            "pass": True,
        },
        "validation": {
            "component_count": len(component_records),
            "link_count": len(links),
            "total_mass_kg": total_mass,
            "expected_total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            "total_mass_pass": total_mass_pass,
            "mass_closure_pass": total_mass_pass
            and all(
                abs(
                    math.fsum(
                        float(component["mass_kg"])
                        for component in component_records
                        if component["owner_link"] == link["link"]
                    )
                    - float(link["mass_kg"])
                )
                < 1.0e-12
                for link in links
            ),
            "max_com_v2_reproduction_error_m": max_com,
            "com_v2_limit_max_inclusive_m": COM_REPRO_LIMIT_M,
            "com_v2_reproduction_pass": max_com <= COM_REPRO_LIMIT_M,
            "print_geometry_pass": print_report["all_four_pass"],
            "sliver_acceptance_pass": all_slivers,
            "go_full_containment_double_count_avoided_pass": all_go,
            "max_raw_detected_component_overlap_fraction": max(
                float(item["raw_detected_overlap_fraction"])
                for item in component_records
            ),
            "max_remaining_component_overlap_fraction": max_remaining,
            "max_remaining_or_estimated_uncertainty_fraction": max_remaining,
            "remaining_component_overlap_fraction_limit_max_inclusive": OVERLAP_BLOCK_FRACTION,
            "remaining_component_overlap_pass": max_remaining <= OVERLAP_BLOCK_FRACTION,
            "all_component_mass_renormalization_pass": all(
                item["mass_renormalization_pass"] for item in component_records
            ),
            "tensor_finite_pass": all(item["tensor_checks"]["finite_pass"] for item in links),
            "tensor_symmetry_pass": all(item["tensor_checks"]["symmetry_pass"] for item in links),
            "tensor_positive_definite_pass": all(
                item["tensor_checks"]["positive_definite_pass"] for item in links
            ),
            "principal_triangle_inequality_pass": all(
                item["tensor_checks"]["principal_triangle_inequality_pass"] for item in links
            ),
            "radius_of_gyration_sanity_pass": all(
                item["tensor_checks"]["radius_of_gyration_sanity_pass"] for item in links
            ),
            "unit_audit_pass": all_link_unit_audits,
            "residual_a_b_c_lt_10_percent_pass": all_residual,
            "collision_proxy_used": False,
            "unresolved_hard_blocker_count": len(blockers),
            "all_freeze_gates_pass": all_gates,
            "pass": all_gates,
        },
        "prohibited_actions": {
            "collision_proxy_used": False,
            "mass_or_com_v2_modified": False,
            "source_cad_modified": False,
            "urdf_xacro_mjcf_or_ros2_control_modified": False,
            "gravity_enabled": False,
            "armature_added": False,
            "friction_or_damping_added": False,
        },
        "unresolved_items": blockers,
    }
    require(
        acceptance["validation"]["mass_closure_pass"],
        "INTERNAL_COMPONENT_LINK_MASS_CLOSURE",
    )
    outputs: dict[str, bytes] = {
        ACCEPTANCE_JSON: canonical_json_bytes(acceptance),
        ACCEPTANCE_MD: markdown_acceptance(acceptance).encode("utf-8"),
    }
    if all_gates:
        freeze = build_freeze_ledger(acceptance)
        outputs[FREEZE_JSON] = canonical_json_bytes(freeze)
        outputs[FREEZE_MD] = markdown_freeze(freeze).encode("utf-8")
    runtime_log.append(
        {
            "component_id": "__TOTAL_BUILD__",
            "actual_elapsed_seconds_runtime_only": time.monotonic() - build_started,
        }
    )
    return outputs, acceptance, runtime_log


def write_or_check(mode: str) -> None:
    outputs, ledger, runtime = build_outputs()
    if mode == "write":
        for relative, data in outputs.items():
            (ROOT / relative).write_bytes(data)
        for relative in (FREEZE_JSON, FREEZE_MD):
            if relative not in outputs and (ROOT / relative).exists():
                (ROOT / relative).unlink()
        git_scope_gate()
        protected_hash_snapshot()
    else:
        for relative, expected in outputs.items():
            path = ROOT / relative
            require(path.is_file(), f"CHECK_OUTPUT_MISSING:{relative}")
            require(path.read_bytes() == expected, f"CHECK_OUTPUT_NONDETERMINISTIC:{relative}")
        for relative in (FREEZE_JSON, FREEZE_MD):
            if relative not in outputs:
                require(not (ROOT / relative).exists(), f"FAIL_MUST_NOT_HAVE_FREEZE_OUTPUT:{relative}")
    changed = changed_paths()
    expected_scope = PASS_ALLOWED_CHANGED_PATHS if ledger["freeze_permitted"] else FAIL_ALLOWED_CHANGED_PATHS
    require(changed <= expected_scope, "FINAL_CHANGED_PATH_SCOPE:" + repr(sorted(changed)))
    total_runtime = next(
        item["actual_elapsed_seconds_runtime_only"]
        for item in runtime
        if item["component_id"] == "__TOTAL_BUILD__"
    )
    print(f"V15.16 Engineering inertia builder: {mode.upper()} PASS")
    print(f"  final status = {ledger['final_status']}")
    print(f"  total runtime seconds (not serialized) = {total_runtime:.3f}")
    print(
        "  residual = "
        + ", ".join(
            f"{item['component_id']}:{item['sensitivity_percent']:.9g}%:{item['status']}"
            for item in ledger["residual_sensitivity"]
        )
    )
    print(
        "  slivers = "
        + ", ".join(
            f"{item['component_id']}:{item['classification']}"
            for item in ledger["sliver_engineering_acceptance"]["items"]
        )
    )
    print(
        "  GO containment corrected = "
        f"{ledger['go_output_full_containment']['double_count_avoided_count']}/5"
    )
    for relative, data in outputs.items():
        digest = sha256_file(ROOT / relative) if mode == "write" else sha256_bytes(data)
        print(f"  {relative} SHA256 = {digest}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build/check bounded V15.16 Engineering V1 rigid inertia."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--write", action="store_true")
    group.add_argument("--check", action="store_true")
    group.add_argument("--component-worker", metavar="COMPONENT_ID")
    group.add_argument("--sliver-worker", metavar="COMPONENT_ID")
    args = parser.parse_args()
    try:
        if args.component_worker:
            result = occupied_component_worker(args.component_worker)
            print(WORKER_PREFIX + json.dumps(result, ensure_ascii=False, sort_keys=True, allow_nan=False))
            return 0
        if args.sliver_worker:
            result = quick_sliver_worker(args.sliver_worker)
            print(WORKER_PREFIX + json.dumps(result, ensure_ascii=False, sort_keys=True, allow_nan=False))
            return 0
        write_or_check("write" if args.write else "check")
        return 0
    except (AuditFailure, KeyError, ValueError, TypeError) as exc:
        print("V15.16 Engineering inertia builder: AUDIT ERROR", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
