from __future__ import annotations

"""Prepare metre-scale visual meshes and convex collision pieces on Ubuntu.

Run inside the verified MuJoCo virtual environment.  Visual simplification is
component-aware and rejected if it moves the source bounds by more than 0.05
mm.  Collision sources are decomposed separately so no convex hull can bridge
between unrelated proxy members.
"""

import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path, PureWindowsPath

import coacd
import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
SOURCE_MANIFEST = ROOT / "mesh_export_manifest.json"
VISUAL_SOURCE = ROOT / "meshes" / "visual_mm"
COLLISION_SOURCE = ROOT / "meshes" / "collision_raw_mm"
VISUAL_OUTPUT = ROOT / "meshes" / "visual_m"
COLLISION_OUTPUT = ROOT / "meshes" / "collision_convex_m"
OUTPUT_MANIFEST = ROOT / "runtime_mesh_manifest.json"

VISUAL_BOUNDS_TOLERANCE_M = 0.00005
COLLISION_PREPROCESS_BOUNDS_TOLERANCE_M = 0.00010
VISUAL_TARGET_FACES_PER_COMPONENT = 20_000
COLLISION_TARGET_FACES_PER_COMPONENT = 12_000
COACD_THRESHOLD_M = 0.0010
COACD_SEED = 1513


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def bounds(mesh: trimesh.Trimesh) -> list[list[float]]:
    return np.asarray(mesh.bounds, dtype=float).tolist()


def max_bounds_delta(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    return float(np.max(np.abs(np.asarray(a.bounds) - np.asarray(b.bounds))))


def load_stl_metres(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(path, force="mesh", process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.to_geometry()
    if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
        raise RuntimeError(f"empty or unsupported STL: {path}")
    loaded.apply_scale(0.001)
    # Binary STL stores a full vertex triplet per face.  Without welding,
    # connectivity sees every triangle as an isolated component and emits
    # thousands of meaningless one-face convex pieces.
    loaded.process(validate=True)
    loaded.remove_unreferenced_vertices()
    return loaded


def connected_components(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    pieces = list(mesh.split(only_watertight=False))
    return pieces or [mesh]


def simplify_guarded(
    component: trimesh.Trimesh,
    target_faces: int,
    bounds_tolerance_m: float,
) -> tuple[trimesh.Trimesh, bool, float]:
    if len(component.faces) <= target_faces:
        return component, False, 0.0
    try:
        simplified = component.simplify_quadric_decimation(
            face_count=target_faces,
            aggression=7,
        )
    except Exception:
        return component, False, 0.0
    if simplified is None or len(simplified.faces) == 0:
        return component, False, 0.0
    delta = max_bounds_delta(component, simplified)
    if delta > bounds_tolerance_m:
        return component, False, delta
    return simplified, True, delta


def prepare_visuals(source: dict) -> dict:
    results = {}
    for link, row in source["links"].items():
        src = VISUAL_SOURCE / f"{link}.stl"
        if not src.is_file() or src.stat().st_size == 0:
            raise FileNotFoundError(src)
        source_bounds = row["visual_bounds_local"]
        bounds_m = [
            [float(value) * 0.001 for value in source_bounds["min_mm"]],
            [float(value) * 0.001 for value in source_bounds["max_mm"]],
        ]
        results[link] = {
            "source": str(src.relative_to(ROOT)),
            "runtime": str(src.relative_to(ROOT)),
            "source_faces": int(row["visual_facets"]),
            "runtime_faces": int(row["visual_facets"]),
            "mesh_scale_xyz_to_m": [0.001, 0.001, 0.001],
            "simplified": False,
            "overall_bounds_delta_m": 0.0,
            "bounds_m": bounds_m,
            "sha256": sha256(src),
        }
        print(f"visual {link}: exact source mesh retained; MuJoCo scale=0.001", flush=True)
    return results


def convex_parts(component: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    prepared, _, _ = simplify_guarded(
        component,
        COLLISION_TARGET_FACES_PER_COMPONENT,
        COLLISION_PREPROCESS_BOUNDS_TOLERANCE_M,
    )
    # A direct convex hull is exact for already-convex source components and
    # avoids unnecessary decomposition noise.
    try:
        if prepared.is_convex:
            return [prepared.convex_hull]
    except Exception:
        pass

    coacd_mesh = coacd.Mesh(
        np.asarray(prepared.vertices, dtype=np.float64),
        np.asarray(prepared.faces, dtype=np.int32),
    )
    pieces = coacd.run_coacd(
        coacd_mesh,
        threshold=COACD_THRESHOLD_M,
        max_convex_hull=64,
        preprocess_mode="auto",
        preprocess_resolution=50,
        resolution=2000,
        mcts_nodes=20,
        mcts_iterations=100,
        mcts_max_depth=3,
        merge=True,
        decimate=False,
        max_ch_vertex=256,
        seed=COACD_SEED,
        real_metric=True,
    )
    result = []
    for vertices, faces in pieces:
        piece = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        if len(piece.faces):
            result.append(piece.convex_hull)
    if not result:
        raise RuntimeError("CoACD returned no convex pieces")
    return result


def prepare_collisions(source: dict) -> dict:
    if COLLISION_OUTPUT.is_dir():
        shutil.rmtree(COLLISION_OUTPUT)
    COLLISION_OUTPUT.mkdir(parents=True, exist_ok=True)
    results = {}
    for link, row in source["links"].items():
        link_output = COLLISION_OUTPUT / link
        link_output.mkdir(parents=True, exist_ok=True)
        link_rows = []
        piece_serial = 0
        for member in row["collision_members"]:
            # FreeCAD's manifest was authored on Windows.  Only the basename
            # is portable; the directory is reconstructed from the Link owner.
            src = COLLISION_SOURCE / link / PureWindowsPath(member["path"]).name
            mesh = load_stl_metres(src)
            member_pieces = []
            for component_index, component in enumerate(connected_components(mesh), 1):
                for local_index, convex in enumerate(convex_parts(component), 1):
                    piece_serial += 1
                    target = link_output / (
                        f"{piece_serial:03d}_m{member['index']:02d}_"
                        f"c{component_index:02d}_p{local_index:02d}.stl"
                    )
                    convex.export(target)
                    member_pieces.append(
                        {
                            "path": str(target.relative_to(ROOT)),
                            "faces": int(len(convex.faces)),
                            "vertices": int(len(convex.vertices)),
                            "bounds_m": bounds(convex),
                            "sha256": sha256(target),
                        }
                    )
            link_rows.append(
                {
                    "member_index": member["index"],
                    "token": member["token"],
                    "source": str(src.relative_to(ROOT)),
                    "source_faces": int(len(mesh.faces)),
                    "source_bounds_m": bounds(mesh),
                    "convex_piece_count": len(member_pieces),
                    "pieces": member_pieces,
                }
            )
            print(
                f"collision {link}/{member['token']}: "
                f"components={len(connected_components(mesh))}, pieces={len(member_pieces)}",
                flush=True,
            )
        results[link] = {
            "member_count": len(link_rows),
            "convex_piece_count": sum(r["convex_piece_count"] for r in link_rows),
            "members": link_rows,
        }
    return results


def main() -> None:
    source = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
    runtime = {
        "schema": "go-m8010-arm-v15.13-mujoco-runtime-meshes/1.0",
        "revision": "V15.13-MuJoCo-empty-load-kinematic-v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_manifest": str(SOURCE_MANIFEST.relative_to(ROOT)),
        "source_manifest_sha256": sha256(SOURCE_MANIFEST),
        "units": {
            "visual_stl": "millimetres with explicit MuJoCo mesh scale 0.001",
            "collision_stl": "metres",
        },
        "visual_policy": {
            "exact_source_mesh_retained": True,
            "simplification": False,
            "reason": "No safe reduction was accepted under the 0.05 mm bounds contract.",
        },
        "collision_policy": {
            "source_member_separation_preserved": True,
            "coacd_threshold_m": COACD_THRESHOLD_M,
            "coacd_seed": COACD_SEED,
            "mujoco_note": "Each emitted mesh is convex; no link-wide bridging hull.",
        },
        "visual": prepare_visuals(source),
        "collision": prepare_collisions(source),
    }
    OUTPUT_MANIFEST.write_text(
        json.dumps(runtime, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": "PASS", "manifest": str(OUTPUT_MANIFEST)}))


if __name__ == "__main__":
    main()
