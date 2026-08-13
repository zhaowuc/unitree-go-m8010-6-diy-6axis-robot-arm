#!/usr/bin/env python3
"""Build the deterministic V15.16A occupied-volume *failure audit*.

The program reconstructs the 17 frozen mass components from the protected
V15.15 authorities, classifies the original 91 positive-volume atom pairs,
The original 91 pairs are fully classified.  The output remains explicitly
non-authoritative because three protected stator BReps cannot produce a valid,
volume-conservative OCCT union.  It never saves CAD, changes a component mass
or frozen COM, evaluates residual sensitivity, or computes/freezes inertia.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Sequence

try:
    import FreeCAD as App
    import MeshPart
    import Part
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Run with FreeCAD Python (D:/freecad/bin/python.exe): " + str(exc))

# The protected prior builder supplies the already-reviewed authority parser,
# STL-to-OCCT reconstruction, signed-tetra math, and atom identity convention.
# It is hash-locked below and is never allowed to write from this program.
import build_inertia_ledger_v15_16_v2 as prior


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "bbd45c79eaefffa9813887937fd51739db1293d9"
SOURCE_TREE = "a6a806b016b96e00cfc4048b1d3a1de889eae779"
TARGET_BRANCH = "agent/v15-16-mass-geometry-authority-v1"

OUTPUT_JSON = "V15_16_质量几何去重审计_v1.json"
OUTPUT_MD = "V15_16_质量几何去重审计_v1.md"
VALIDATOR = "tools/validate_mass_geometry_authority_v15_16.py"
BUILDER = "tools/build_mass_geometry_authority_v15_16.py"
ALLOWED_CHANGED_PATHS = {OUTPUT_JSON, OUTPUT_MD, BUILDER, VALIDATOR}

PRIOR_FAIL_JSON = "V15_16_刚体惯量_FAIL审计_v2.json"
PRIOR_PROTECTED_HASHES = {
    PRIOR_FAIL_JSON: "f8e0d62f027c932a49b5b38930ee770fca52c60ca2c3aa64b28308934cca55b8",
    "V15_16_刚体惯量_FAIL审计_v2.md": "94810a472a6e1a651f4a0b614e237d6241bc43bca3d7f66f908065ee26cdcfc7",
    "V15_16_打印件惯量几何适用性报告.json": "7308455fcdab006f68d90401c3206ce7209575feadfd700350122f96dc61cd14",
    "tools/build_inertia_ledger_v15_16_v2.py": "914d774f9abc4b5aa8638eec9b2533790e5ee446712358fbd33558d69c09e704",
    "tools/validate_inertia_ledger_v15_16_v2.py": "22aadb9d72a3b7e1952bf53c981c889fc251f585174424be8204556595776590",
    "tools/test_inertia_math_v15_16_v2.py": "141fcce0fb2b7af622f4c447bac5f5fa1ce72cf49745fb34aac6adac02c5e7fa",
}
PROTECTED_HASHES = {**prior.PROTECTED_HASHES, **PRIOR_PROTECTED_HASHES}

EXPECTED_ATOMS = 266
EXPECTED_PAIRS = 3899
EXPECTED_ORIGINAL_OVERLAPS = 91
EXPECTED_COMPONENTS = 17
EXPECTED_TOTAL_MASS_KG = 3.4515
LINK_ORDER = ("link2", "link3", "link4", "link5", "link6", "gripper")
GO_OUTPUT_COMPONENTS = (
    "J2A_OUTPUT_EQ", "J2B_OUTPUT_EQ", "J3_OUTPUT_EQ", "J4_OUTPUT_EQ", "J5_OUTPUT_EQ"
)
BLOCKED_STATOR_COMPONENTS = ("J3_STATOR_EQ", "J4_STATOR_EQ", "J5_STATOR_EQ")

# The 91-pair population is deliberately locked to V2's original threshold.
ORIGINAL_POSITIVE_TOL_MM3 = 1.0e-7
CANONICAL_ABSOLUTE_TOL_MM3 = 1.0e-9
CANONICAL_RELATIVE_TOL = 1.0e-12
CONTAINMENT_ABSOLUTE_TOL_MM3 = 1.0e-7
CONTAINMENT_RELATIVE_TOL = 1.0e-9
EXACT_VOLUME_REL_TOL = 1.0e-9
EXACT_BBOX_TOL_MM = 1.0e-7
EXACT_CENTROID_TOL_MM = 1.0e-7
EXACT_SURFACE_REL_TOL = 1.0e-9
NUMERICAL_SLIVER_SMALLER_FRACTION_MAX = 1.0e-6
PATH_B_VOLUME_REL_LIMIT = 1.0e-6
PATH_B_COM_LIMIT_MM = 1.0e-3  # exactly 1e-6 m
PATH_B_DEFLECTION_LADDER_MM = (1.0e-4, 3.0e-5, 1.0e-5, 3.0e-6)
PATH_B_ANGULAR_DEFLECTION_RAD = 0.1


class AuditFailure(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditFailure(message)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_json_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2,
                       allow_nan=False) + "\n").encode("utf-8")


def run_git(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(ROOT), "-c", "core.quotepath=false", *args],
        text=True, encoding="utf-8", stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=check,
    )


def changed_paths() -> set[str]:
    paths: set[str] = set()
    for line in run_git("status", "--porcelain=v1", "--untracked-files=all").stdout.splitlines():
        token = line[3:]
        if " -> " in token:
            token = token.split(" -> ", 1)[1]
        if token:
            paths.add(token.replace("\\", "/"))
    paths.update(
        line.replace("\\", "/")
        for line in run_git("diff", "--name-only", SOURCE_COMMIT).stdout.splitlines()
        if line
    )
    return paths


def git_scope_gate() -> dict[str, Any]:
    branch = run_git("branch", "--show-current").stdout.strip()
    require(branch == TARGET_BRANCH, f"WRONG_BRANCH:{branch}")
    source_tree = run_git("rev-parse", f"{SOURCE_COMMIT}^{{tree}}").stdout.strip()
    require(source_tree == SOURCE_TREE, f"SOURCE_TREE_MISMATCH:{source_tree}")
    require(run_git("merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD", check=False).returncode == 0,
            "SOURCE_COMMIT_NOT_ANCESTOR")
    unexpected = sorted(changed_paths() - ALLOWED_CHANGED_PATHS)
    require(not unexpected, "UNEXPECTED_CHANGED_PATHS:" + repr(unexpected))
    return {
        "target_branch": TARGET_BRANCH,
        "source_commit": SOURCE_COMMIT,
        "source_tree": SOURCE_TREE,
        "source_commit_is_ancestor": True,
        "allowed_changed_paths": sorted(ALLOWED_CHANGED_PATHS),
        "unexpected_changed_paths": [],
        "pass": True,
    }


def protected_hash_snapshot() -> list[dict[str, Any]]:
    result = []
    for relative, expected in sorted(PROTECTED_HASHES.items()):
        path = ROOT / relative
        require(path.is_file(), f"PROTECTED_INPUT_MISSING:{relative}")
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_HASH_MISMATCH:{relative}:{actual}")
        result.append({"path": relative, "sha256": actual, "locked": True})
    return result


def vec(value: Any) -> np.ndarray:
    return np.asarray([float(value[0]), float(value[1]), float(value[2])], dtype=float)


def bbox_values(shape: Any) -> list[float]:
    b = shape.BoundBox
    return [float(b.XMin), float(b.YMin), float(b.ZMin),
            float(b.XMax), float(b.YMax), float(b.ZMax)]


def bbox_record(shape: Any) -> dict[str, list[float]]:
    q = bbox_values(shape)
    return {"min_mm": q[:3], "max_mm": q[3:],
            "size_mm": [q[3] - q[0], q[4] - q[1], q[5] - q[2]]}


def bbox_intersects(left: Any, right: Any, tolerance: float = 1.0e-9) -> bool:
    a, b = left.BoundBox, right.BoundBox
    return not (a.XMax < b.XMin - tolerance or b.XMax < a.XMin - tolerance
                or a.YMax < b.YMin - tolerance or b.YMax < a.YMin - tolerance
                or a.ZMax < b.ZMin - tolerance or b.ZMax < a.ZMin - tolerance)


def topology_signature(shape: Any) -> dict[str, int | str]:
    return {
        "shape_type": str(shape.ShapeType), "solids": len(shape.Solids),
        "shells": len(shape.Shells), "faces": len(shape.Faces),
        "wires": len(shape.Wires), "edges": len(shape.Edges),
        "vertices": len(shape.Vertexes),
    }


def deep_check_errors(shape: Any) -> list[str]:
    try:
        result = shape.check(True)
        # OCCT/FreeCAD returns ``None`` for a clean shape and a sequence for
        # reported faults (the exact item type is version-dependent).
        return [] if result is None else sorted(str(item) for item in result)
    except Exception as exc:
        return ["CHECK_EXCEPTION:" + str(exc)]


def shape_is_deep_valid_solid(shape: Any) -> tuple[bool, list[str]]:
    errors = deep_check_errors(shape)
    valid = (not shape.isNull() and shape.isValid() and shape.isClosed()
             and float(shape.Volume) > 0.0 and not errors)
    return valid, errors


def atom_metrics(atom: dict[str, Any]) -> dict[str, Any]:
    shape = atom["_shape"]
    return {
        "volume_mm3": float(shape.Volume),
        "centroid_world_mm": vec(shape.CenterOfMass).tolist(),
        "bbox_world_mm": bbox_record(shape),
        "surface_area_mm2": float(shape.Area),
        "topology_signature": topology_signature(shape),
    }


def atom_identity(atom: dict[str, Any]) -> dict[str, Any]:
    return {
        "atom_id": atom["atom_id"], "member": atom["member"],
        "source_object": atom["source_object"],
        "source_solid_index_one_based": int(atom["source_solid_index"]),
        "metrics": atom_metrics(atom),
    }


def common_volume(left: Any, right: Any) -> float:
    if not bbox_intersects(left, right):
        return 0.0
    common = left.common(right)
    return 0.0 if common.isNull() else abs(float(common.Volume))


def relative_difference(left: float, right: float) -> float:
    return abs(left - right) / max(abs(left), abs(right), 1.0e-300)


def classify_pair(left: dict[str, Any], right: dict[str, Any], common: float,
                  pair_order: int) -> dict[str, Any]:
    lm, rm = atom_metrics(left), atom_metrics(right)
    lv, rv = float(lm["volume_mm3"]), float(rm["volume_mm3"])
    smaller = min(lv, rv)
    fraction_left, fraction_right = common / lv, common / rv
    smaller_fraction = common / smaller
    symmetric_difference = max(0.0, lv + rv - 2.0 * common)
    containment_tol = max(CONTAINMENT_ABSOLUTE_TOL_MM3,
                          CONTAINMENT_RELATIVE_TOL * smaller)
    contained = smaller - common <= containment_tol
    volume_equal = relative_difference(lv, rv) <= EXACT_VOLUME_REL_TOL
    bbox_delta = max(abs(a - b) for a, b in zip(bbox_values(left["_shape"]),
                                                bbox_values(right["_shape"])))
    centroid_delta = float(np.linalg.norm(vec(left["_shape"].CenterOfMass)
                                          - vec(right["_shape"].CenterOfMass)))
    surface_equal = relative_difference(float(lm["surface_area_mm2"]),
                                        float(rm["surface_area_mm2"])) <= EXACT_SURFACE_REL_TOL
    topology_equal = lm["topology_signature"] == rm["topology_signature"]
    symmetric_tol = max(CONTAINMENT_ABSOLUTE_TOL_MM3,
                        CONTAINMENT_RELATIVE_TOL * max(lv, rv))
    exact = (volume_equal and lv - common <= symmetric_tol and rv - common <= symmetric_tol
             and symmetric_difference <= symmetric_tol and bbox_delta <= EXACT_BBOX_TOL_MM
             and centroid_delta <= EXACT_CENTROID_TOL_MM and surface_equal and topology_equal)
    if exact:
        classification = "EXACT_DUPLICATE"
    elif contained:
        classification = "FULL_CONTAINMENT"
    elif smaller_fraction <= NUMERICAL_SLIVER_SMALLER_FRACTION_MAX:
        classification = "NUMERICAL_SLIVER"
    else:
        classification = "PARTIAL_INTERPENETRATION"
    return {
        "pair_order": pair_order, "component_id": left["component_id"],
        "left": atom_identity(left), "right": atom_identity(right),
        "common_volume_mm3": common,
        "overlap_fraction_left": fraction_left,
        "overlap_fraction_right": fraction_right,
        "smaller_fraction": smaller_fraction,
        "symmetric_difference_volume_mm3": symmetric_difference,
        "classification": classification,
        "classification_evidence": {
            "volume_relative_difference": relative_difference(lv, rv),
            "volume_equal": volume_equal,
            "smaller_minus_common_mm3": smaller - common,
            "containment_tolerance_mm3": containment_tol,
            "smaller_fully_contained": contained,
            "bbox_max_coordinate_delta_mm": bbox_delta,
            "centroid_distance_mm": centroid_delta,
            "surface_area_relative_difference": relative_difference(
                float(lm["surface_area_mm2"]), float(rm["surface_area_mm2"])),
            "topology_signature_equal": topology_equal,
            "exact_duplicate_all_gates_pass": exact,
        },
        "occupied_volume_action": "REQUIRES_COMPONENT_LOCAL_SPATIAL_UNION_NOT_YET_ESTABLISHED",
    }


def reconstruct_atoms(document: Any, components: Sequence[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    _, artifact_cache = prior.audit_print_artifacts(components)
    result: dict[str, list[dict[str, Any]]] = {}
    for component in components:
        is_print = any(member.get("artifact_path") for member in component.get("members", []))
        atoms = (prior.print_component_atoms(component, artifact_cache) if is_print
                 else prior.brep_component_atoms(document, component))
        result[component["component_id"]] = atoms
    return result


def audit_original_pairs(components: Sequence[dict[str, Any]],
                         atoms_by_component: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    events: list[dict[str, Any]] = []
    union_edges: dict[str, list[tuple[str, str, float]]] = {}
    failures: list[dict[str, Any]] = []
    pair_count = bbox_count = union_trigger_count = 0
    component_summaries = []
    for component in components:
        cid = component["component_id"]
        atoms = atoms_by_component[cid]
        raw_volume = math.fsum(float(atom["_shape"].Volume) for atom in atoms)
        union_trigger_tol = max(CANONICAL_ABSOLUTE_TOL_MM3,
                                CANONICAL_RELATIVE_TOL * raw_volume)
        component_pairs = component_bbox = component_events = component_union_edges = 0
        edges: list[tuple[str, str, float]] = []
        for left, right in itertools.combinations(atoms, 2):
            pair_count += 1
            component_pairs += 1
            if not bbox_intersects(left["_shape"], right["_shape"]):
                continue
            bbox_count += 1
            component_bbox += 1
            try:
                cv = common_volume(left["_shape"], right["_shape"])
            except Exception as exc:
                failures.append({"component_id": cid, "left_atom_id": left["atom_id"],
                                 "right_atom_id": right["atom_id"], "error": str(exc)})
                continue
            if cv > union_trigger_tol:
                edges.append((left["atom_id"], right["atom_id"], cv))
                union_trigger_count += 1
                component_union_edges += 1
            if cv > ORIGINAL_POSITIVE_TOL_MM3:
                component_events += 1
                events.append(classify_pair(left, right, cv, len(events) + 1))
        union_edges[cid] = edges
        component_summaries.append({
            "component_id": cid, "atom_count": len(atoms),
            "internal_pair_count_checked": component_pairs,
            "bbox_candidate_count_boolean_checked": component_bbox,
            "original_positive_overlap_pair_count": component_events,
            "union_trigger_pair_count": component_union_edges,
            "union_trigger_tolerance_mm3": union_trigger_tol,
        })
    counts = Counter(event["classification"] for event in events)
    for name in ("EXACT_DUPLICATE", "FULL_CONTAINMENT", "PARTIAL_INTERPENETRATION",
                 "NUMERICAL_SLIVER", "ZERO_VOLUME_CONTACT"):
        counts.setdefault(name, 0)
    return {
        "original_atom_count": sum(len(v) for v in atoms_by_component.values()),
        "internal_pair_count_checked": pair_count,
        "bbox_candidate_count_boolean_checked": bbox_count,
        "original_positive_overlap_threshold_mm3_strictly_greater_than": ORIGINAL_POSITIVE_TOL_MM3,
        "original_positive_overlap_pair_count": len(events),
        "classification_counts": dict(sorted(counts.items())),
        "events": events,
        "zero_volume_contacts_in_original_positive_population": [],
        "union_trigger_pair_count": union_trigger_count,
        "component_summaries": component_summaries,
        "boolean_failures": failures,
        "_union_edges": union_edges,
    }


def normalize_result_solids(shape: Any) -> list[Any]:
    result = []
    for source in shape.Solids:
        solid = source.copy()
        if float(solid.Volume) < 0.0:
            solid.reverse()
        if float(solid.Volume) <= CANONICAL_ABSOLUTE_TOL_MM3:
            continue
        valid, errors = shape_is_deep_valid_solid(solid)
        require(valid, "CANONICAL_SOLID_DEEP_INVALID:" + repr(errors))
        result.append(solid)
    return result


def collapse_contained(atoms: Sequence[dict[str, Any]], events: Sequence[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    by_id = {atom["atom_id"]: atom for atom in atoms}
    dropped: set[str] = set()
    decisions = []
    for event in events:
        if event["classification"] not in {"EXACT_DUPLICATE", "FULL_CONTAINMENT"}:
            continue
        left_id, right_id = event["left"]["atom_id"], event["right"]["atom_id"]
        if left_id in dropped or right_id in dropped:
            continue
        lv = float(by_id[left_id]["_shape"].Volume)
        rv = float(by_id[right_id]["_shape"].Volume)
        if event["classification"] == "EXACT_DUPLICATE":
            keep, drop = sorted((left_id, right_id))
        else:
            keep, drop = ((left_id, right_id) if lv > rv else (right_id, left_id))
        dropped.add(drop)
        decisions.append({
            "classification": event["classification"], "retained_occupied_set_atom_id": keep,
            "collapsed_atom_id": drop,
            "collapsed_atom_physical_component_membership_retained": True,
            "meaning": "only duplicate spatial occupancy is omitted; member identity and frozen component mass remain",
        })
    survivors = [atom for atom in atoms if atom["atom_id"] not in dropped]
    require(survivors, "CONTAINMENT_COLLAPSE_REMOVED_ALL")
    return survivors, decisions


def connected_clusters(atoms: Sequence[dict[str, Any]], edges: Sequence[tuple[str, str, float]]) -> list[list[dict[str, Any]]]:
    by_id = {atom["atom_id"]: atom for atom in atoms}
    adjacency = {atom_id: set() for atom_id in by_id}
    for left, right, _ in edges:
        if left in by_id and right in by_id:
            adjacency[left].add(right)
            adjacency[right].add(left)
    clusters = []
    pending = set(by_id)
    while pending:
        seed = min(pending)
        stack, ids = [seed], []
        pending.remove(seed)
        while stack:
            current = stack.pop()
            ids.append(current)
            for neighbor in sorted(adjacency[current], reverse=True):
                if neighbor in pending:
                    pending.remove(neighbor)
                    stack.append(neighbor)
        clusters.append([by_id[token] for token in sorted(ids)])
    return clusters


def try_cluster_fuse(cluster: Sequence[dict[str, Any]]) -> tuple[list[Any] | None, str | None]:
    if len(cluster) == 1:
        shape = cluster[0]["_shape"].copy()
        valid, errors = shape_is_deep_valid_solid(shape)
        return ([shape], "SOURCE_SOLID_DERIVED_COPY") if valid else (None, "SINGLE_SOURCE_INVALID:" + repr(errors))
    try:
        shapes = [atom["_shape"].copy() for atom in cluster]
        fused = shapes[0].multiFuse(shapes[1:])
        candidates: list[tuple[str, Any, str | None]] = []
        try:
            candidates.append(("MULTIFUSE_REMOVE_SPLITTER", fused.removeSplitter(), None))
        except Exception as exc:
            candidates.append(("MULTIFUSE_REMOVE_SPLITTER", fused, "REMOVE_SPLITTER_EXCEPTION:" + str(exc)))
        # Some OCCT models are valid immediately after BOP fusion but become
        # invalid when removeSplitter refines coincident/internal faces.  The
        # unrefined exact fuse is acceptable only after the same deep-validity
        # and pairwise-common-volume gates.
        candidates.append(("MULTIFUSE_UNREFINED", fused, None))
        rejected = []
        for method, candidate, preliminary_error in candidates:
            try:
                solids = normalize_result_solids(candidate)
                require(solids, "FUSE_EMPTY")
                for left, right in itertools.combinations(solids, 2):
                    require(common_volume(left, right) <= CANONICAL_ABSOLUTE_TOL_MM3,
                            "FUSE_INTERNAL_OVERLAP")
                status = method
                if preliminary_error:
                    status += ":" + preliminary_error
                return solids, status
            except Exception as exc:
                rejected.append(f"{method}:{preliminary_error or ''}:{exc}")
        return None, ";".join(rejected)
    except Exception as exc:
        return None, str(exc)


def general_fuse_partition(atoms: Sequence[dict[str, Any]]) -> tuple[list[Any] | None, str | None]:
    """Partition the occupied set into exact, mutually disjoint OCCT cells."""
    if len(atoms) == 1:
        return try_cluster_fuse(atoms)
    try:
        shapes = [atom["_shape"].copy() for atom in atoms]
        partition, _history = shapes[0].generalFuse(shapes[1:], 0.0)
        solids = normalize_result_solids(partition)
        require(solids, "GENERAL_FUSE_EMPTY")
        for left, right in itertools.combinations(solids, 2):
            require(common_volume(left, right) <= CANONICAL_ABSOLUTE_TOL_MM3,
                    "GENERAL_FUSE_INTERNAL_OVERLAP")
        return solids, None
    except Exception as exc:
        return None, str(exc)


def exact_disjoint_partition(atoms: Sequence[dict[str, Any]], component_tol: float) -> list[Any]:
    accepted: list[Any] = []
    # Largest-first makes full containment a no-op and minimizes repeated cuts.
    ordered = sorted(atoms, key=lambda a: (-float(a["_shape"].Volume), a["atom_id"]))
    for atom in ordered:
        fragments = [atom["_shape"].copy()]
        for occupied in accepted:
            next_fragments = []
            for fragment in fragments:
                if not bbox_intersects(fragment, occupied):
                    next_fragments.append(fragment)
                    continue
                cv = common_volume(fragment, occupied)
                if cv <= component_tol:
                    next_fragments.append(fragment)
                    continue
                fragment_volume = float(fragment.Volume)
                cut = fragment.cut(occupied)
                if cut.isNull() or abs(float(cut.Volume)) <= component_tol:
                    require(fragment_volume - cv <= component_tol,
                            "CUT_DROPPED_NONCOVERED_VOLUME")
                    continue
                cut_solids = normalize_result_solids(cut)
                cut_volume = math.fsum(float(solid.Volume) for solid in cut_solids)
                expected_cut_volume = max(0.0, fragment_volume - cv)
                conservation_tolerance = max(component_tol, 1.0e-9 * fragment_volume)
                require(cut_volume <= fragment_volume + conservation_tolerance,
                        "CUT_VOLUME_NONMONOTONIC")
                require(abs(cut_volume - expected_cut_volume) <= conservation_tolerance,
                        "CUT_VOLUME_NONCONSERVATIVE")
                next_fragments.extend(cut_solids)
            fragments = next_fragments
            if not fragments:
                break
        accepted.extend(fragments)
    require(accepted, "PARTITION_EMPTY")
    return accepted


def build_canonical_component(atoms: Sequence[dict[str, Any]], component_events: Sequence[dict[str, Any]],
                              union_edges: Sequence[tuple[str, str, float]], raw_volume: float) -> tuple[list[Any], dict[str, Any]]:
    survivors, collapse = collapse_contained(atoms, component_events)
    clusters = connected_clusters(survivors, union_edges)
    fused_solids: list[Any] = []
    cluster_reports = []
    fallback_reason = None
    for index, cluster in enumerate(clusters, 1):
        solids, method_or_error = try_cluster_fuse(cluster)
        cluster_reports.append({"cluster_index": index,
                                "atom_ids": [a["atom_id"] for a in cluster],
                                "method": method_or_error if solids is not None else "MULTIFUSE_REJECTED",
                                "result_solid_count": len(solids or []),
                                "rejection_or_note": None if solids is not None else method_or_error})
        if solids is None:
            fallback_reason = f"cluster_{index}:{method_or_error}"
            break
        fused_solids.extend(solids)
    component_tol = max(CANONICAL_ABSOLUTE_TOL_MM3,
                        CANONICAL_RELATIVE_TOL * raw_volume)
    if fallback_reason is not None:
        general_solids, general_error = general_fuse_partition(survivors)
        if general_solids is not None:
            fused_solids = general_solids
            method = "OCCT_GENERAL_FUSE_EXACT_OCCUPIED_SET_CELL_PARTITION_FALLBACK"
        else:
            fused_solids = exact_disjoint_partition(survivors, component_tol)
            method = "OCCT_SEQUENTIAL_CUT_EXACT_OCCUPIED_SET_PARTITION_LAST_RESORT"
            fallback_reason += ";generalFuse_rejected:" + str(general_error)
    else:
        method = "CLASSIFICATION_COLLAPSE_PLUS_OCCT_MULTIFUSE_REMOVE_SPLITTER"
    return fused_solids, {
        "method": method, "derived_copies_only": True,
        "component_local_only": True, "cross_component_union": False,
        "containment_and_exact_duplicate_collapse": collapse,
        "collapse_preserves_physical_component_membership": True,
        "cluster_reports": cluster_reports, "fallback_reason": fallback_reason,
        "disjoint_components_permitted_as_multiple_solids": True,
    }


def occt_volume_com(solids: Sequence[Any]) -> tuple[float, np.ndarray]:
    volume = math.fsum(float(shape.Volume) for shape in solids)
    require(volume > 0.0, "UNION_VOLUME_NONPOSITIVE")
    com = np.array([
        math.fsum(float(shape.Volume) * float(shape.CenterOfMass[axis]) for shape in solids) / volume
        for axis in range(3)
    ], dtype=float)
    return volume, com


def mesh_solid_volume_com(shape: Any, deflection: float) -> tuple[float, np.ndarray, int, int]:
    mesh = MeshPart.meshFromShape(
        Shape=shape.cleaned(), LinearDeflection=float(deflection),
        AngularDeflection=PATH_B_ANGULAR_DEFLECTION_RAD, Relative=False,
    )
    mesh.harmonizeNormals()
    mesh.fixDegenerations()
    require(mesh.CountFacets > 0 and mesh.isSolid(), "PATH_B_MESH_NOT_SOLID")
    require(not mesh.hasNonManifolds(), "PATH_B_MESH_NONMANIFOLD")
    vertices, faces = prior.mesh_topology(mesh)
    props = prior.anchored_polyhedron_properties(vertices, faces, 1.0, require_topology=True)
    return (float(props["volume_mm3"]), vec(props["com_mm"]),
            int(mesh.CountPoints), int(mesh.CountFacets))


def adaptive_path_b(solids: Sequence[Any], path_a_volume: float,
                    path_a_com: np.ndarray) -> dict[str, Any]:
    attempts = []
    selected = None
    for deflection in PATH_B_DEFLECTION_LADDER_MM:
        parts = [mesh_solid_volume_com(shape, deflection) for shape in solids]
        volume = math.fsum(part[0] for part in parts)
        com = np.array([
            math.fsum(part[0] * float(part[1][axis]) for part in parts) / volume
            for axis in range(3)
        ], dtype=float)
        volume_error = abs(volume - path_a_volume) / path_a_volume
        com_error = float(np.linalg.norm(com - path_a_com))
        attempt = {
            "linear_deflection_mm": deflection,
            "angular_deflection_rad": PATH_B_ANGULAR_DEFLECTION_RAD,
            "point_count": sum(part[2] for part in parts),
            "triangle_count": sum(part[3] for part in parts),
            "volume_mm3": volume, "com_world_mm": com.tolist(),
            "volume_relative_error_vs_path_a": volume_error,
            "com_difference_mm_vs_path_a": com_error,
            "pass": volume_error < PATH_B_VOLUME_REL_LIMIT and com_error < PATH_B_COM_LIMIT_MM,
        }
        attempts.append(attempt)
        if attempt["pass"]:
            selected = attempt
            break
    require(selected is not None, "PATH_B_ADAPTIVE_LADDER_EXHAUSTED:" + repr(attempts[-1]))
    return {
        "method": "FINE_WATERTIGHT_TESSELLATION_BBOX_ANCHORED_SIGNED_TETRAHEDRA",
        "mesh_volume_or_center_of_gravity_api_used": False,
        "attempts": attempts,
        "selected_linear_deflection_mm": selected["linear_deflection_mm"],
        "selected_volume_mm3": selected["volume_mm3"],
        "selected_com_world_mm": selected["com_world_mm"],
        "selected_point_count": selected["point_count"],
        "selected_triangle_count": selected["triangle_count"],
    }


def canonical_pair_audit(solids: Sequence[Any], union_volume: float) -> dict[str, Any]:
    tolerance = max(CANONICAL_ABSOLUTE_TOL_MM3,
                    CANONICAL_RELATIVE_TOL * union_volume)
    positive, failures = [], []
    pair_count = 0
    deep_checks = []
    for index, shape in enumerate(solids, 1):
        valid, errors = shape_is_deep_valid_solid(shape)
        deep_checks.append({"canonical_solid_index_one_based": index,
                            "valid": valid, "deep_bop_check_errors": errors})
    for (li, left), (ri, right) in itertools.combinations(enumerate(solids, 1), 2):
        pair_count += 1
        if not bbox_intersects(left, right):
            continue
        try:
            cv = common_volume(left, right)
        except Exception as exc:
            failures.append({"left": li, "right": ri, "error": str(exc)})
            continue
        if cv > tolerance:
            positive.append({"left": li, "right": ri, "common_volume_mm3": cv})
    return {
        "internal_pair_count_checked": pair_count,
        "positive_overlap_tolerance_mm3": tolerance,
        "positive_overlap_pair_count": len(positive),
        "positive_overlap_pairs": positive,
        "boolean_failures": failures,
        "canonical_solid_deep_validity": deep_checks,
        "all_solids_valid_closed_deep_bop_clean": all(x["valid"] for x in deep_checks),
        "pass": not positive and not failures and all(x["valid"] for x in deep_checks),
    }


def canonical_solid_summaries(solids: Sequence[Any]) -> list[dict[str, Any]]:
    summaries = []
    for index, shape in enumerate(solids, 1):
        valid, errors = shape_is_deep_valid_solid(shape)
        summaries.append({
            "canonical_solid_index_one_based": index,
            "stable_identity": "derived_component_occupied_solid_{:04d}".format(index),
            "volume_mm3": float(shape.Volume),
            "centroid_world_mm": vec(shape.CenterOfMass).tolist(),
            "bbox_world_mm": bbox_record(shape),
            "surface_area_mm2": float(shape.Area),
            "topology_signature": topology_signature(shape),
            "is_valid": bool(shape.isValid()), "is_closed": bool(shape.isClosed()),
            "deep_bop_check_errors": errors, "deep_valid": valid,
        })
    return summaries


def occupied_set_coverage_audit(atoms: Sequence[dict[str, Any]], solids: Sequence[Any],
                                union_volume: float) -> dict[str, Any]:
    """Prove every raw atom is covered by the pairwise-disjoint canonical set.

    Since canonical solids have already passed the no-positive-common-volume
    gate, summing ``Volume(atom common canonical_i)`` cannot double-count a
    positive region.  This is used only for accepted canonical occupied sets.
    """
    tolerance = max(CANONICAL_ABSOLUTE_TOL_MM3,
                    CANONICAL_RELATIVE_TOL * union_volume)
    records, failures = [], []
    for atom in atoms:
        atom_volume = float(atom["_shape"].Volume)
        covered_terms = []
        for index, canonical in enumerate(solids, 1):
            if not bbox_intersects(atom["_shape"], canonical):
                continue
            try:
                cv = common_volume(atom["_shape"], canonical)
            except Exception as exc:
                failures.append({"atom_id": atom["atom_id"],
                                 "canonical_solid_index_one_based": index,
                                 "error": str(exc)})
                continue
            if cv > 0.0:
                covered_terms.append(cv)
        covered = math.fsum(covered_terms)
        uncovered = max(0.0, atom_volume - covered)
        overshoot = max(0.0, covered - atom_volume)
        local_tolerance = max(tolerance, CANONICAL_RELATIVE_TOL * atom_volume)
        records.append({
            "atom_id": atom["atom_id"], "raw_atom_volume_mm3": atom_volume,
            "covered_by_canonical_volume_mm3": covered,
            "uncovered_volume_mm3": uncovered,
            "coverage_overshoot_mm3": overshoot,
            "tolerance_mm3": local_tolerance,
            "pass": uncovered <= local_tolerance and overshoot <= local_tolerance,
        })
    return {
        "method": "SUM_OCCT_COMMON_VOLUMES_AGAINST_PAIRWISE_DISJOINT_CANONICAL_SOLIDS",
        "atom_count_checked": len(records), "records": records,
        "boolean_failures": failures,
        "max_uncovered_volume_mm3": max((r["uncovered_volume_mm3"] for r in records), default=0.0),
        "max_coverage_overshoot_mm3": max((r["coverage_overshoot_mm3"] for r in records), default=0.0),
        "pass": not failures and all(record["pass"] for record in records),
    }


def component_records(components: Sequence[dict[str, Any]],
                      atoms_by_component: dict[str, list[dict[str, Any]]],
                      overlap: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, np.ndarray]]:
    events_by_component = {c["component_id"]: [] for c in components}
    for event in overlap["events"]:
        events_by_component[event["component_id"]].append(event)
    records, union_coms = [], {}
    for component in components:
        cid = component["component_id"]
        atoms = atoms_by_component[cid]
        raw_volume = math.fsum(float(atom["_shape"].Volume) for atom in atoms)
        canonical, recipe = build_canonical_component(
            atoms, events_by_component[cid], overlap["_union_edges"][cid], raw_volume)
        union_volume, union_com = occt_volume_com(canonical)
        pair_audit = canonical_pair_audit(canonical, union_volume)
        require(pair_audit["pass"], f"CANONICAL_PAIR_AUDIT:{cid}")
        coverage = occupied_set_coverage_audit(atoms, canonical, union_volume)
        require(coverage["pass"], f"CANONICAL_COVERAGE_AUDIT:{cid}")
        solid_summaries = canonical_solid_summaries(canonical)
        path_b = adaptive_path_b(canonical, union_volume, union_com)
        selected_volume = float(path_b["selected_volume_mm3"])
        selected_com = vec(path_b["selected_com_world_mm"])
        volume_error = abs(selected_volume - union_volume) / union_volume
        com_error_mm = float(np.linalg.norm(selected_com - union_com))
        frozen_mass = float(component["_authority_mass_kg"])
        frozen_com = vec(component["com_world_mm"])
        delta = union_com - frozen_com
        removed = raw_volume - union_volume
        require(removed >= -1.0e-6, f"UNION_VOLUME_GREATER_THAN_RAW:{cid}:{removed}")
        removed = max(0.0, removed)
        union_coms[cid] = union_com
        record = {
            "component_id": cid, "owner_link": component["owner_link"],
            "frozen_mass_kg": frozen_mass, "mass_unchanged": True,
            "mass_authority": prior.MASS_LEDGER,
            "geometry_membership_and_frozen_com_authority": prior.COM_LEDGER,
            "geometry_class": component["geometry_class"],
            "member_tokens": list(component["geometry_member_order_inherited_exactly_from_v1_mass_ledger"]),
            "atom_count": len(atoms), "raw_abs_volume_mm3": raw_volume,
            "canonical_solid_count": len(canonical), "union_volume_mm3": union_volume,
            "removed_overlap_effective_volume_mm3": removed,
            "duplicate_fraction": removed / raw_volume,
            "rho_raw_kg_per_mm3": frozen_mass / raw_volume,
            "rho_union_kg_per_mm3": frozen_mass / union_volume,
            "rho_raw_kg_per_m3": frozen_mass / raw_volume * 1.0e9,
            "rho_union_kg_per_m3": frozen_mass / union_volume * 1.0e9,
            "raw_com_v2_world_mm": frozen_com.tolist(),
            "union_com_world_mm": union_com.tolist(),
            "union_minus_com_v2_delta_xyz_mm": delta.tolist(),
            "union_minus_com_v2_delta_norm_mm": float(np.linalg.norm(delta)),
            "path_a_occt": {"volume_mm3": union_volume, "com_world_mm": union_com.tolist()},
            "path_b_anchored_tetra": path_b,
            "path_a_b_agreement": {
                "volume_relative_error": volume_error,
                "com_difference_mm": com_error_mm,
                "com_difference_m": com_error_mm * 1.0e-3,
                "volume_limit_strict_less_than": PATH_B_VOLUME_REL_LIMIT,
                "com_limit_strict_less_than_m": PATH_B_COM_LIMIT_MM * 1.0e-3,
                "pass": volume_error < PATH_B_VOLUME_REL_LIMIT and com_error_mm < PATH_B_COM_LIMIT_MM,
            },
            "canonical_recipe": recipe,
            "canonical_solid_summaries": solid_summaries,
            "canonical_internal_pair_audit": pair_audit,
            "raw_atom_occupied_set_coverage_audit": coverage,
            "collision_proxy_used": False,
        }
        if cid == "UPPER_ARM_PRINT_MEASURED":
            record["historical_upper_arm_a_b_allocation"] = {
                "role": "AUDIT_ONLY_NOT_UNION_NUMERIC_AUTHORITY",
                "source": prior.COM_LEDGER,
                "allocation": component["equal_density_volume_allocation"],
                "future_authoritative_geometry_policy": "UNIFORM_DENSITY_OF_A_UNION_B_OCCUPIED_VOLUME",
            }
        records.append(record)
    return records, union_coms


def link_records(com_authority: dict[str, Any], components: Sequence[dict[str, Any]],
                 union_coms: dict[str, np.ndarray]) -> list[dict[str, Any]]:
    result = []
    for link in LINK_ORDER:
        owned = [component for component in components if component["owner_link"] == link]
        mass = math.fsum(float(component["_authority_mass_kg"]) for component in owned)
        candidate_world = np.array([
            math.fsum(float(component["_authority_mass_kg"])
                      * float(union_coms[component["component_id"]][axis]) for component in owned) / mass
            for axis in range(3)
        ], dtype=float)
        authority = com_authority["links"][link]
        frozen_world = vec(authority["com_world_mm"])
        origin = vec(authority["frame_world_at_mechanical_zero"]["origin_mm"])
        rotation = np.asarray(authority["frame_world_at_mechanical_zero"]["rotation_matrix_row_major"], dtype=float)
        candidate_link = rotation.T @ (candidate_world - origin)
        frozen_link = vec(authority["com_link_mm"])
        result.append({
            "link": link, "frozen_mass_kg": mass,
            "component_ids": [component["component_id"] for component in owned],
            "com_v2_world_mm": frozen_world.tolist(),
            "union_candidate_com_world_mm": candidate_world.tolist(),
            "union_candidate_minus_com_v2_delta_xyz_world_mm": (candidate_world - frozen_world).tolist(),
            "union_candidate_com_owner_link_mm": candidate_link.tolist(),
            "com_v2_owner_link_mm": frozen_link.tolist(),
            "union_candidate_minus_com_v2_delta_xyz_owner_link_mm": (candidate_link - frozen_link).tolist(),
            "union_candidate_minus_com_v2_delta_norm_mm": float(np.linalg.norm(candidate_world - frozen_world)),
            "com_v2_modified": False,
        })
    return result


def diagnostic_go_occupied_set(component_id: str, atoms: Sequence[dict[str, Any]],
                               events: Sequence[dict[str, Any]]) -> dict[str, Any]:
    containments = [event for event in events if event["component_id"] == component_id
                    and event["classification"] == "FULL_CONTAINMENT"]
    require(len(containments) == 1, f"GO_CONTAINMENT_COUNT:{component_id}")
    survivors, decisions = collapse_contained(atoms, containments)
    raw_volume = math.fsum(float(atom["_shape"].Volume) for atom in atoms)
    union_volume = math.fsum(float(atom["_shape"].Volume) for atom in survivors)
    union_com = np.array([
        math.fsum(float(atom["_shape"].Volume) * float(atom["_shape"].CenterOfMass[axis])
                  for atom in survivors) / union_volume for axis in range(3)
    ])
    tolerance = max(CANONICAL_ABSOLUTE_TOL_MM3, CANONICAL_RELATIVE_TOL * union_volume)
    positive = []
    for left, right in itertools.combinations(survivors, 2):
        cv = common_volume(left["_shape"], right["_shape"])
        if cv > tolerance:
            positive.append({"left_atom_id": left["atom_id"], "right_atom_id": right["atom_id"],
                             "common_volume_mm3": cv})
    require(not positive, f"GO_DIAGNOSTIC_CANONICAL_OVERLAP:{component_id}")
    event = containments[0]
    removed = raw_volume - union_volume
    return {
        "component_id": component_id, "status": "PATH_A_DIAGNOSTIC_ONLY_NOT_AUTHORITY",
        "authority_eligible": False,
        "reason_not_authority": "GLOBAL_FAIL_GATE_CLOSED_BEFORE_REQUIRED_ALL_COMPONENT_PATH_A_B_ACCEPTANCE",
        "raw_abs_volume_mm3": raw_volume, "diagnostic_union_volume_mm3": union_volume,
        "removed_overlap_effective_volume_mm3": removed,
        "duplicate_fraction": removed / raw_volume,
        "diagnostic_union_com_world_mm": union_com.tolist(),
        "canonical_positive_overlap_tolerance_mm3": tolerance,
        "diagnostic_canonical_positive_overlap_pair_count": 0,
        "containment_decisions": decisions,
        "left_atom_id": event["left"]["atom_id"], "right_atom_id": event["right"]["atom_id"],
        "left_solid_index_one_based": event["left"]["source_solid_index_one_based"],
        "right_solid_index_one_based": event["right"]["source_solid_index_one_based"],
        "left_volume_mm3": event["left"]["metrics"]["volume_mm3"],
        "right_volume_mm3": event["right"]["metrics"]["volume_mm3"],
        "common_volume_mm3": event["common_volume_mm3"],
        "smaller_fraction": event["smaller_fraction"],
        "classification": event["classification"],
        "neutral_physical_membership_retained": True,
        "path_b_crosscheck": None,
    }


def blocked_method_evidence() -> list[dict[str, Any]]:
    return [
        {"method": "OCCT_FUSE_AND_MULTIFUSE_ALL_DETERMINISTIC_ORDERS",
         "parameters": "original tolerances; refined and unrefined; sequential and multiFuse",
         "result": "REJECTED_INVALID_OR_VOLUME_GREATER_THAN_SET_THEORETIC_UNION"},
        {"method": "OCCT_FUZZY_FUSE_MULTIFUSE_GENERALFUSE_SPLITAPI",
         "parameters": {"fuzzy_tolerance_mm_ladder": [1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4]},
         "result": "REJECTED_INVALID_OR_NONCONSERVATIVE_FOR_EVERY_SETTING"},
        {"method": "OCCT_GENERALFUSE_CELL_PARTITION",
         "result": "REJECTED_CONTAINS_INVALID_FRAGMENT_AND_FRAGMENT_VOLUME_SUM_NONCONSERVATIVE",
         "representative_j3": {
             "value_role": "READ_ONLY_PROBE_LOG_VALUES_NOT_BUILDER_RECOMPUTED",
             "fragment_volume_sum_mm3": 174599.410265897,
             "expected_raw_minus_common_mm3": 174593.964187770,
             "volume_excess_mm3": 5.446078127,
         }},
        {"method": "OCCT_SEQUENTIAL_CUT_OCCUPIED_PARTITION",
         "result": "REJECTED_CUT_INVALID_OR_CUT_VOLUME_INCREASES"},
        {"method": "OCCT_SHAPE_FIX_WORKING_MIN_MAX_PRECISION_LADDER",
         "result": "REJECTED_NO_VALID_CONSERVATIVE_UNION_COMBINATION"},
        {"method": "OCCT_FIXTOLERANCE_AND_LIMITTOLERANCE_DERIVED_COPIES",
         "result": "REJECTED_CAPPING_BELOW_SOURCE_MAXIMUM_MAKES_SOURCE_INVALID"},
        {"method": "STEP_ROUNDTRIP_DERIVED_COPY",
         "result": "REJECTED_SOURCE_VOLUME_DRIFT_AND_FUSE_STILL_INVALID",
         "representative_j3": {
             "solid_22_volume_delta_mm3": 0.232148252089,
             "solid_22_com_delta_mm": 7.726e-5,
             "solid_24_volume_delta_mm3": -0.042077925056,
             "solid_24_com_delta_mm": 2.617e-6,
         }},
        {"method": "MESHPart_0p01MM_SEAM_ONLY_DEDUP_WELD_THEN_FACETED_OCCT_BREP",
         "result": "REJECTED_SOURCE_TESSELLATION_NOT_WATERTIGHT_MANIFOLD_OR_SELF_INTERSECTION_FREE",
         "representative_j3_solid_22": {
             "points_before_cleanup": 184229, "facets_before_cleanup": 368536,
             "facets_after_builtin_cleanup": 368534,
             "single_incidence_edges_observed_range": [18, 22],
             "triple_incidence_edges": 12, "self_intersections": 86,
             "coordinate_weld_ladder_mm": [0.0, 1e-7, 1e-6, 1e-5],
             "nearest_bad_vertex_gap_mm_approx": 0.0248263,
         }},
        {"method": "SHAPE_TESSELLATE_0p01MM_ANCHORED_TETRA_TO_FACETED_BREP",
         "result": "REJECTED_TOPOLOGY_AND_SELF_INTERSECTION;_0p003MM_MEMORY_ERROR",
         "representative_j3_solid_22": {
             "vertices": 622665, "facets": 1245406, "self_intersections": 144,
             "anchored_tetra_topology_pass": False,
         }},
    ]


def bounded_boolean_result(shape: Any, expected_union_volume: float) -> dict[str, Any]:
    """Serialize validity/conservation without accepting a failed boolean."""
    if shape is None or shape.isNull():
        return {"is_null": True, "is_valid": False, "is_closed": False,
                "volume_mm3": None, "volume_drift_from_expected_union_mm3": None,
                "deep_bop_check_errors": ["NULL_RESULT"], "accepted": False}
    volume = abs(float(shape.Volume))
    deep_errors = deep_check_errors(shape)
    valid = bool(shape.isValid())
    closed = bool(shape.isClosed())
    drift = volume - expected_union_volume
    tolerance = max(CANONICAL_ABSOLUTE_TOL_MM3,
                    CANONICAL_RELATIVE_TOL * expected_union_volume)
    accepted = valid and closed and not deep_errors and abs(drift) <= tolerance
    return {
        "is_null": False, "is_valid": bool(shape.isValid()),
        "is_closed": bool(shape.isClosed()), "solid_count": len(shape.Solids),
        "volume_mm3": volume,
        "volume_drift_from_expected_union_mm3": drift,
        "acceptance_tolerance_mm3": tolerance,
        "deep_bop_check_errors": deep_errors,
        "accepted": accepted,
    }


def reproduce_blocker_pair_gates(left: Any, right: Any) -> dict[str, Any]:
    """Run bounded, deterministic OCCT evidence gates on one blocked pair."""
    lv, rv = float(left.Volume), float(right.Volume)
    cv = common_volume(left, right)
    expected_union = lv + rv - cv
    results: dict[str, Any] = {
        "left_volume_mm3": lv, "right_volume_mm3": rv,
        "common_volume_mm3": cv,
        "set_theoretic_union_volume_mm3": expected_union,
    }
    for label, operation in (
        ("fuse_left_right", lambda: left.copy().fuse(right.copy())),
        ("fuse_right_left", lambda: right.copy().fuse(left.copy())),
        ("general_fuse", lambda: left.copy().generalFuse([right.copy()], 0.0)[0]),
    ):
        try:
            results[label] = bounded_boolean_result(operation(), expected_union)
        except Exception as exc:
            results[label] = {"exception": str(exc), "accepted": False}
    for label, minuend, subtrahend, expected_volume in (
        ("cut_left_minus_right", left, right, lv - cv),
        ("cut_right_minus_left", right, left, rv - cv),
    ):
        try:
            cut = minuend.copy().cut(subtrahend.copy())
            actual = 0.0 if cut.isNull() else abs(float(cut.Volume))
            results[label] = {
                **bounded_boolean_result(cut, expected_volume),
                "expected_difference_volume_mm3": expected_volume,
                "volume_drift_from_expected_difference_mm3": actual - expected_volume,
                "volume_monotonic": actual <= float(minuend.Volume) + 1.0e-7,
            }
            results[label]["accepted"] = bool(
                results[label]["accepted"] and results[label]["volume_monotonic"])
        except Exception as exc:
            results[label] = {"exception": str(exc), "accepted": False}
    route_labels = ("fuse_left_right", "fuse_right_left", "general_fuse",
                    "cut_left_minus_right", "cut_right_minus_left")
    results["all_candidate_routes_rejected"] = all(
        not bool(results[label].get("accepted", False)) for label in route_labels)
    require(results["all_candidate_routes_rejected"],
            "BLOCKER_ROUTE_UNEXPECTEDLY_ACCEPTED")
    return results


def markdown_report(ledger: dict[str, Any]) -> str:
    counts = ledger["overlap_audit"]["classification_counts"]
    lines = [
        "# V15.16A 质量几何去重失败审计", "",
        "> 结论：**FAIL-CLOSED**。本文件完整分类 91 个原始 overlap，但未建立 occupied-volume authority。", "",
        "## 原始 overlap 分类", "",
        f"- atoms / internal pairs / positive pairs: `{ledger['overlap_audit']['original_atom_count']}` / `{ledger['overlap_audit']['internal_pair_count_checked']}` / `{ledger['overlap_audit']['original_positive_overlap_pair_count']}`",
        f"- EXACT_DUPLICATE / FULL_CONTAINMENT / PARTIAL_INTERPENETRATION / NUMERICAL_SLIVER: `{counts['EXACT_DUPLICATE']}` / `{counts['FULL_CONTAINMENT']}` / `{counts['PARTIAL_INTERPENETRATION']}` / `{counts['NUMERICAL_SLIVER']}`", "",
        "## 未解决的 OCCT BRep 并集阻断", "",
        "| component | pair | common mm³ | smaller fraction |", "|---|---|---:|---:|",
    ]
    for item in ledger["unresolved_overlap_items"]:
        lines.append("| {} | {} ↔ {} | {:.12g} | {:.12g} |".format(
            item["component_id"], item["left_solid_index_one_based"],
            item["right_solid_index_one_based"], item["common_volume_mm3"],
            item["smaller_fraction"]))
    lines.extend([
        "", "上述三组源 BRep 浅层 valid/closed，但深度 BOP 检查和所有有界修复/并集路径均无法产生有效、体积守恒的 canonical occupied set。", "",
        "## GO output 诊断（非 authority）", "",
        "| component | raw mm³ | diagnostic union mm³ | removed fraction | classification |",
        "|---|---:|---:|---:|---|",
    ])
    for item in ledger["go_output_diagnostics"]:
        lines.append("| {} | {:.12g} | {:.12g} | {:.12g} | {} |".format(
            item["component_id"], item["raw_abs_volume_mm3"],
            item["diagnostic_union_volume_mm3"], item["duplicate_fraction"],
            item["classification"]))
    lines.extend([
        "", "## 不变项", "", "- component mass / total mass: unchanged / `3.4515 kg`",
        "- collision proxy used: `NO`", "- COM V2 modified: `NO`",
        "- original CAD modified: `NO`", "- final inertia frozen: `NO`",
        "- residual sensitivity recalculated: `NO`", "", "## 最终状态", "",
        f"`{ledger['final_status']}`", "",
    ])
    return "\n".join(lines)


def build_outputs() -> tuple[bytes, bytes, dict[str, Any]]:
    git_guard = git_scope_gate()
    protected_before = protected_hash_snapshot()
    _mass, _com, components = prior.load_authorities()
    require(len(components) == EXPECTED_COMPONENTS, "COMPONENT_COUNT")
    prior_fail = json.loads((ROOT / PRIOR_FAIL_JSON).read_text(encoding="utf-8"))
    document = App.openDocument(str(ROOT / prior.SOURCE_CAD))
    try:
        atoms_by_component = reconstruct_atoms(document, components)
        overlap = audit_original_pairs(components, atoms_by_component)
    finally:
        App.closeDocument(document.Name)
    protected_after = protected_hash_snapshot()
    require(protected_before == protected_after, "PROTECTED_INPUT_CHANGED_DURING_BUILD")
    require(overlap["original_atom_count"] == EXPECTED_ATOMS, "ATOM_COUNT")
    require(overlap["internal_pair_count_checked"] == EXPECTED_PAIRS, "PAIR_COUNT")
    require(overlap["original_positive_overlap_pair_count"] == EXPECTED_ORIGINAL_OVERLAPS,
            "ORIGINAL_OVERLAP_COUNT")
    require(not overlap["boolean_failures"], "ORIGINAL_BOOLEAN_FAILURES")
    expected_counts = {"EXACT_DUPLICATE": 0, "FULL_CONTAINMENT": 5,
                       "PARTIAL_INTERPENETRATION": 83, "NUMERICAL_SLIVER": 3,
                       "ZERO_VOLUME_CONTACT": 0}
    require(overlap["classification_counts"] == expected_counts,
            "CLASSIFICATION_COUNTS:" + repr(overlap["classification_counts"]))
    prior_events = prior_fail["duplicate_overlap_audit"]["positive_overlap_events"]
    prior_ids = [(event["component_id"], event["left_atom_id"], event["right_atom_id"])
                 for event in prior_events]
    recomputed_ids = [(event["component_id"], event["left"]["atom_id"], event["right"]["atom_id"])
                      for event in overlap["events"]]
    require(recomputed_ids == prior_ids, "PRIOR_91_IDENTITY_CROSSCHECK")
    overlap["prior_v2_identity_crosscheck"] = {
        "prior_path": PRIOR_FAIL_JSON, "prior_sha256": PROTECTED_HASHES[PRIOR_FAIL_JSON],
        "role": "COUNT_AND_ATOM_IDENTITY_CROSSCHECK_ONLY_NOT_NUMERIC_UNION_AUTHORITY",
        "ordered_event_identities_identical": True, "pass": True,
    }
    overlap.pop("_union_edges")

    component_records_out = []
    for component in components:
        cid = component["component_id"]
        events = [event for event in overlap["events"] if event["component_id"] == cid]
        raw_volume = math.fsum(float(atom["_shape"].Volume) for atom in atoms_by_component[cid])
        component_records_out.append({
            "component_id": cid, "owner_link": component["owner_link"],
            "frozen_mass_kg": float(component["_authority_mass_kg"]),
            "mass_unchanged": True, "atom_count": len(atoms_by_component[cid]),
            "raw_abs_volume_mm3": raw_volume,
            "original_positive_overlap_pair_count": len(events),
            "classification_counts": dict(sorted(Counter(event["classification"] for event in events).items())),
            "occupied_volume_policy_status": (
                "BLOCKED_BY_SOURCE_BREP_BOOLEAN_PATHOLOGY" if cid in BLOCKED_STATOR_COMPONENTS
                else "NOT_ESTABLISHED_AFTER_GLOBAL_FAIL_GATE_CLOSED"
            ),
            "canonical_union_volume_mm3": None, "canonical_union_com_world_mm": None,
            "removed_overlap_effective_volume_mm3": None, "duplicate_fraction": None,
            "rho_union_kg_per_mm3": None, "canonical_positive_overlap_pair_count": None,
            "path_a_b_crosscheck": None, "union_minus_com_v2_delta": None,
            "authority_eligible": False, "collision_proxy_used": False,
        })

    go_diagnostics = [diagnostic_go_occupied_set(cid, atoms_by_component[cid], overlap["events"])
                      for cid in GO_OUTPUT_COMPONENTS]
    methods = blocked_method_evidence()
    unresolved = []
    for cid in BLOCKED_STATOR_COMPONENTS:
        matches = [event for event in overlap["events"] if event["component_id"] == cid
                   and {event["left"]["source_solid_index_one_based"],
                        event["right"]["source_solid_index_one_based"]} == {22, 24}]
        require(len(matches) == 1 and matches[0]["classification"] == "NUMERICAL_SLIVER",
                f"BLOCKED_STATOR_EVENT:{cid}")
        event = matches[0]
        left_atom = next(atom for atom in atoms_by_component[cid]
                         if atom["atom_id"] == event["left"]["atom_id"])
        right_atom = next(atom for atom in atoms_by_component[cid]
                          if atom["atom_id"] == event["right"]["atom_id"])
        reproduced = reproduce_blocker_pair_gates(left_atom["_shape"], right_atom["_shape"])
        require(abs(reproduced["common_volume_mm3"] - event["common_volume_mm3"]) <= 1.0e-10,
                f"BLOCKER_COMMON_REPRO:{cid}")
        unresolved.append({
            "code": "OCCT_SOURCE_BREP_BOOLEAN_PATHOLOGY",
            "component_id": cid, "left_atom_id": event["left"]["atom_id"],
            "right_atom_id": event["right"]["atom_id"],
            "left_solid_index_one_based": event["left"]["source_solid_index_one_based"],
            "right_solid_index_one_based": event["right"]["source_solid_index_one_based"],
            "left_volume_mm3": event["left"]["metrics"]["volume_mm3"],
            "right_volume_mm3": event["right"]["metrics"]["volume_mm3"],
            "common_volume_mm3": event["common_volume_mm3"],
            "smaller_fraction": event["smaller_fraction"],
            "classification": event["classification"],
            "source_max_edge_vertex_tolerances_mm": {
                "solid_22": 0.009979726937784, "solid_24": 0.007955604692057},
            "shallow_source_is_valid_and_closed": True,
            "deep_bop_source_check": "FAIL_SELF_INTERSECT_TOO_SMALL_EDGE_C0_AND_INVALID_CURVE_ON_SURFACE",
            "set_theoretic_pair_union_volume_mm3": (
                event["left"]["metrics"]["volume_mm3"]
                + event["right"]["metrics"]["volume_mm3"] - event["common_volume_mm3"]),
            "reproduced_blocker_gates": reproduced,
            "supplemental_read_only_probe_evidence": {
                "role": "BOUNDED_ADVERSARIAL_PROBE_EVIDENCE_NOT_REEXECUTED_BY_BUILDER",
                "scope": "DERIVED_COPIES_ONLY_NO_SOURCE_OR_DEPLOYMENT_WRITE",
                "method_attempts": methods,
            },
            "accepted_candidate": None, "resolved": False,
            "required_resolution": "SUPPLY_REPAIRED_WATERTIGHT_STATOR_MASS_GEOMETRY_AUTHORITY_OR_DIRECT_OCCUPIED_VOLUME_AUTHORITY",
        })
    total_mass = math.fsum(float(component["_authority_mass_kg"]) for component in components)
    ledger = {
        "schema": "go-m8010-arm-v15.16-mass-geometry-authority-v1-fail-audit/1.0",
        "revision": "V15.16A-MASS_GEOMETRY_DEOVERLAP_FAIL_AUDIT_V1",
        "scope": "DETERMINISTIC_FAIL_CLOSED_AUDIT_NOT_OCCUPIED_VOLUME_AUTHORITY",
        "final_status": "V15.16 MASS_GEOMETRY_AUTHORITY_V1 = FAIL",
        "authoritative": False, "occupied_volume_authority_established": False,
        "authoritative_ledger_written": False, "freeze_permitted": False,
        "policy_not_established": True,
        "provenance": {
            "source_commit": SOURCE_COMMIT, "source_tree": SOURCE_TREE,
            "target_branch": TARGET_BRANCH, "source_commit_strictly_locked": True,
            "runtime": {"python": sys.version.split()[0],
                        "freecad": ".".join(str(x) for x in App.Version()[:3]),
                        "occt": getattr(Part, "OCC_VERSION", "UNKNOWN"),
                        "numpy": np.__version__},
        },
        "git_scope_guard": git_guard,
        "authorities": {
            "mass": {"path": prior.MASS_LEDGER, "sha256": PROTECTED_HASHES[prior.MASS_LEDGER],
                     "role": "FROZEN_COMPONENT_MASS_AUTHORITY"},
            "com_v2": {"path": prior.COM_LEDGER, "sha256": PROTECTED_HASHES[prior.COM_LEDGER],
                       "role": "FROZEN_MEMBERSHIP_PLACEMENT_AND_COM_COMPARISON_AUTHORITY"},
            "geometry": {"path": prior.SOURCE_CAD, "sha256": PROTECTED_HASHES[prior.SOURCE_CAD],
                         "opened_without_save": True, "collision_proxy_used": False},
            "protected_inputs_before": protected_before,
            "protected_inputs_after": protected_after,
            "all_protected_inputs_unchanged": True,
        },
        "classification_thresholds": {
            "original_positive_volume_strictly_greater_than_mm3": ORIGINAL_POSITIVE_TOL_MM3,
            "exact_volume_relative_tolerance": EXACT_VOLUME_REL_TOL,
            "exact_common_and_symmetric_difference_absolute_floor_mm3": CONTAINMENT_ABSOLUTE_TOL_MM3,
            "exact_common_and_symmetric_difference_relative_tolerance": CONTAINMENT_RELATIVE_TOL,
            "exact_bbox_max_coordinate_delta_mm": EXACT_BBOX_TOL_MM,
            "exact_centroid_distance_mm": EXACT_CENTROID_TOL_MM,
            "exact_surface_area_relative_tolerance": EXACT_SURFACE_REL_TOL,
            "exact_topology_signature_must_equal": True,
            "full_containment_requires_common_approximately_smaller_and_not_exact_duplicate": True,
            "numerical_sliver_smaller_fraction_max_inclusive": NUMERICAL_SLIVER_SMALLER_FRACTION_MAX,
            "zero_volume_contact_is_not_mass_duplication": True,
            "canonical_pair_target_tolerance_if_authority_can_be_established":
                "max(1e-9 mm^3,1e-12*component_union_volume)",
        },
        "component_order": [component["component_id"] for component in components],
        "overlap_audit": overlap, "components": component_records_out,
        "go_output_diagnostics": go_diagnostics,
        "links": [{"link": link, "union_candidate_com_world_mm": None,
                   "union_candidate_minus_com_v2_delta_norm_mm": None,
                   "status": "N/A_OCCUPIED_VOLUME_AUTHORITY_NOT_ESTABLISHED"}
                  for link in LINK_ORDER],
        "validation": {
            "fail_audit_evidence_complete": True,
            "all_91_original_overlaps_classified": True,
            "unresolved_overlap_item_count": len(unresolved),
            "all_17_component_policies_established": False,
            "canonical_positive_overlap_pair_count": None,
            "canonical_positive_overlap_zero": False,
            "all_path_a_b_crosschecks_pass": False,
            "max_path_a_b_volume_relative_error": None,
            "max_path_a_b_com_error_m": None,
            "all_union_com_deltas_reported": False,
            "all_component_masses_unchanged": True,
            "total_mass_kg": total_mass, "expected_total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            "total_mass_pass": abs(total_mass - EXPECTED_TOTAL_MASS_KG) < 1.0e-12,
            "protected_sources_unchanged": protected_before == protected_after,
            "authority_established": False, "authority_pass": False, "pass": False,
        },
        "prohibited_actions": {
            "collision_proxy_used": False, "modified_com_v2": False,
            "modified_original_cad": False, "modified_v15_15_authorities": False,
            "final_inertia_tensor_computed": False, "final_inertia_frozen": False,
            "residual_sensitivity_recalculated": False, "deployment_files_modified": False,
        },
        "unresolved_overlap_items": unresolved,
        "unresolved_items": unresolved,
    }
    require(len(unresolved) == 3, "UNRESOLVED_COUNT")
    require(ledger["validation"]["fail_audit_evidence_complete"], "FAIL_AUDIT_INCOMPLETE")
    require(not ledger["authoritative"] and not ledger["validation"]["authority_pass"],
            "FAIL_AUDIT_FALSE_AUTHORITY")
    return canonical_json_bytes(ledger), markdown_report(ledger).encode("utf-8"), ledger


def write_or_check(mode: str) -> None:
    json_bytes, md_bytes, ledger = build_outputs()
    outputs = {OUTPUT_JSON: json_bytes, OUTPUT_MD: md_bytes}
    if mode == "write":
        for relative, data in outputs.items():
            (ROOT / relative).write_bytes(data)
        git_scope_gate()
        protected_hash_snapshot()
    else:
        for relative, expected in outputs.items():
            path = ROOT / relative
            require(path.is_file(), f"OUTPUT_MISSING:{relative}")
            require(path.read_bytes() == expected, f"OUTPUT_NONDETERMINISTIC:{relative}")
    counts = ledger["overlap_audit"]["classification_counts"]
    print(f"V15.16A FAIL AUDIT BUILD = PASS ({mode.upper()})")
    print("  authority status =", ledger["final_status"])
    print("  overlap classifications =", ", ".join(f"{k}:{v}" for k, v in counts.items()))
    print("  unresolved stator pairs =", ledger["validation"]["unresolved_overlap_item_count"])
    print("  canonical positive overlaps = N/A (authority not established)")
    print("  Path A/B maxima = N/A (global fail gate closed)")
    for relative, data in outputs.items():
        print(f"  {relative} SHA256 = {hashlib.sha256(data).hexdigest()}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build/check V15.16A occupied-volume failure audit")
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--write", action="store_true")
    modes.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        write_or_check("write" if args.write else "check")
    except (AuditFailure, prior.AuditFailure) as exc:
        print("V15.16A mass geometry authority builder: FAIL-CLOSED", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
