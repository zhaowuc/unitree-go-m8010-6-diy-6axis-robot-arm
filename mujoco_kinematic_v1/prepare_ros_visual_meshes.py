from __future__ import annotations

"""Build practical RViz visual meshes from the audited full-resolution STL.

Only visual triangles are reduced.  Link frames, joint transforms, collision
meshes, mass properties and the full MuJoCo render meshes are untouched.
Disconnected CAD parts are simplified independently so screws, covers and the
complete adaptive gripper cannot disappear through a global face budget.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
SOURCE_DIR = ROOT / "meshes" / "visual_mm"
DEFAULT_OUTPUT_DIR = ROOT / "meshes" / "ros_visual_mm"
REPORT = ROOT / "QA_ROS2_V15_13_RVIZ可视网格.json"

FACE_BUDGET = {
    "base_link": 140_000,
    "link1": 260_000,
    "link2": 280_000,
    "link3": 240_000,
    "link4": 220_000,
    "link5": 180_000,
    "link6": 10_000,
    "gripper": 250_000,
}
KEEP_COMPONENT_FACES = 1_200
# A half-millimetre envelope is only for RViz display LOD; the full MuJoCo
# visual and all collision geometry remain exact and untouched.
BOUNDS_TOLERANCE_MM = 0.50


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def as_mesh(path: Path, *, process: bool) -> trimesh.Trimesh:
    loaded = trimesh.load(path, force="mesh", process=process)
    if not isinstance(loaded, trimesh.Trimesh):
        raise TypeError(f"Expected a mesh: {path}")
    return loaded


def simplify_link(name: str, output_dir: Path) -> dict:
    source = SOURCE_DIR / f"{name}.stl"
    target = output_dir / f"{name}.stl"
    mesh = as_mesh(source, process=True)
    source_bounds = mesh.bounds.copy()
    source_faces = int(len(mesh.faces))
    components = list(mesh.split(only_watertight=False))
    small = [part for part in components if len(part.faces) <= KEEP_COMPONENT_FACES]
    large = [part for part in components if len(part.faces) > KEEP_COMPONENT_FACES]
    preserved_faces = sum(len(part.faces) for part in small)
    large_faces = sum(len(part.faces) for part in large)
    requested_budget = min(FACE_BUDGET[name], source_faces)
    large_budget = max(requested_budget - preserved_faces, 0)
    ratio = min(1.0, large_budget / large_faces) if large_faces else 1.0

    output_parts: list[trimesh.Trimesh] = [part.copy() for part in small]
    failures: list[dict] = []
    for index, part in enumerate(large):
        part_faces = int(len(part.faces))
        target_faces = max(4, min(part_faces, int(round(part_faces * ratio))))
        if target_faces >= part_faces:
            output_parts.append(part.copy())
            continue
        try:
            simplified = part.simplify_quadric_decimation(
                face_count=target_faces, aggression=7
            )
            if not isinstance(simplified, trimesh.Trimesh) or len(simplified.faces) < 4:
                raise RuntimeError("simplifier returned an empty component")
            output_parts.append(simplified)
        except Exception as exc:  # Preserve geometry instead of dropping a CAD part.
            failures.append(
                {
                    "component_index": index,
                    "faces": part_faces,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
            output_parts.append(part.copy())

    result = trimesh.util.concatenate(output_parts)
    result.remove_unreferenced_vertices()
    target.parent.mkdir(parents=True, exist_ok=True)
    result.export(target, file_type="stl")
    verified = as_mesh(target, process=False)
    bounds_delta = np.abs(verified.bounds - source_bounds)
    max_bounds_delta = float(bounds_delta.max())
    if max_bounds_delta > BOUNDS_TOLERANCE_MM:
        raise RuntimeError(
            f"{name}: bounds changed by {max_bounds_delta:.6f} mm "
            f"(limit {BOUNDS_TOLERANCE_MM} mm)"
        )
    if target.stat().st_size >= 100_000_000:
        raise RuntimeError(f"{name}: RViz STL exceeds 100 MB")

    return {
        "source": str(source.relative_to(ROOT)).replace("\\", "/"),
        "output": str(target.relative_to(ROOT)).replace("\\", "/"),
        "source_sha256": sha256(source),
        "output_sha256": sha256(target),
        "source_faces": source_faces,
        "output_faces": int(len(verified.faces)),
        "source_components": len(components),
        "preserved_small_components": len(small),
        "simplification_failures": failures,
        "source_bounds_mm": source_bounds.tolist(),
        "output_bounds_mm": verified.bounds.tolist(),
        "max_bounds_delta_mm": max_bounds_delta,
        "output_bytes": target.stat().st_size,
        "pass": max_bounds_delta <= BOUNDS_TOLERANCE_MM,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    arguments = parser.parse_args()
    output_dir = arguments.output_dir.resolve()
    rows = [simplify_link(name, output_dir) for name in FACE_BUDGET]
    report = {
        "schema": "go-m8010-arm-v15.13-rviz-visual-mesh-qa/1.0",
        "purpose": "RViz-only visual LOD generated from audited FCStd/MuJoCo full meshes",
        "does_not_modify": [
            "J1-J6 origin/axis/limit",
            "collision meshes",
            "mass/inertia",
            "frozen camera extrinsic",
        ],
        "bounds_tolerance_mm": BOUNDS_TOLERANCE_MM,
        "links": {name: row for name, row in zip(FACE_BUDGET, rows)},
    }
    report["source_faces"] = sum(row["source_faces"] for row in rows)
    report["output_faces"] = sum(row["output_faces"] for row in rows)
    report["max_bounds_delta_mm"] = max(row["max_bounds_delta_mm"] for row in rows)
    report["pass"] = all(row["pass"] for row in rows)
    REPORT.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "status": "PASS" if report["pass"] else "FAIL",
                "source_faces": report["source_faces"],
                "output_faces": report["output_faces"],
                "max_bounds_delta_mm": report["max_bounds_delta_mm"],
                "report": str(REPORT),
            }
        )
    )
    if not report["pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
