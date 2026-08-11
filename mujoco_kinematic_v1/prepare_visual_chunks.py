from __future__ import annotations

"""Split exact visual STL triangles into MuJoCo-compatible <=200k chunks."""

import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
SOURCE_MANIFEST = ROOT / "mesh_export_manifest.json"
RUNTIME_MANIFEST = ROOT / "runtime_mesh_manifest.json"
VISUAL_SOURCE = ROOT / "meshes" / "visual_mm"
VISUAL_CHUNKS = ROOT / "meshes" / "visual_chunks_mm"
MAX_FACES_PER_CHUNK = 190_000


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def main() -> None:
    source = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
    runtime = json.loads(RUNTIME_MANIFEST.read_text(encoding="utf-8"))
    if VISUAL_CHUNKS.is_dir():
        shutil.rmtree(VISUAL_CHUNKS)
    VISUAL_CHUNKS.mkdir(parents=True, exist_ok=True)

    visual_rows = {}
    for link, row in source["links"].items():
        src = VISUAL_SOURCE / f"{link}.stl"
        mesh = trimesh.load(src, force="mesh", process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.to_geometry()
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            raise RuntimeError(f"invalid visual mesh: {src}")
        link_dir = VISUAL_CHUNKS / link
        link_dir.mkdir(parents=True, exist_ok=True)
        chunks = []
        union_min = np.full(3, np.inf)
        union_max = np.full(3, -np.inf)
        for chunk_index, start in enumerate(range(0, len(mesh.faces), MAX_FACES_PER_CHUNK), 1):
            faces = mesh.faces[start : start + MAX_FACES_PER_CHUNK]
            chunk = trimesh.Trimesh(vertices=mesh.vertices.copy(), faces=faces.copy(), process=False)
            chunk.remove_unreferenced_vertices()
            if len(chunk.faces) > 200_000:
                raise RuntimeError("MuJoCo visual chunk face limit exceeded")
            target = link_dir / f"{chunk_index:03d}.stl"
            chunk.export(target)
            union_min = np.minimum(union_min, chunk.bounds[0])
            union_max = np.maximum(union_max, chunk.bounds[1])
            chunks.append(
                {
                    "path": str(target.relative_to(ROOT)),
                    "faces": int(len(chunk.faces)),
                    "vertices": int(len(chunk.vertices)),
                    "sha256": sha256(target),
                }
            )
        source_bounds = np.asarray(
            [
                row["visual_bounds_local"]["min_mm"],
                row["visual_bounds_local"]["max_mm"],
            ],
            dtype=float,
        )
        union_bounds = np.vstack((union_min, union_max))
        bounds_error_mm = float(np.max(np.abs(source_bounds - union_bounds)))
        if bounds_error_mm > 1.0e-5:
            raise RuntimeError(f"{link}: visual chunk union moved bounds by {bounds_error_mm} mm")
        visual_rows[link] = {
            "source": str(src.relative_to(ROOT)),
            "source_sha256": sha256(src),
            "source_faces": int(len(mesh.faces)),
            "runtime_faces": sum(chunk["faces"] for chunk in chunks),
            "runtime_chunks": chunks,
            "chunk_count": len(chunks),
            "max_faces_per_chunk": max(chunk["faces"] for chunk in chunks),
            "mesh_scale_xyz_to_m": [0.001, 0.001, 0.001],
            "simplified": False,
            "triangle_set_preserved": True,
            "overall_bounds_delta_m": bounds_error_mm * 0.001,
            "bounds_m": (source_bounds * 0.001).tolist(),
        }
        print(
            f"visual {link}: exact {len(mesh.faces)} faces -> {len(chunks)} chunks, "
            f"max={visual_rows[link]['max_faces_per_chunk']}",
            flush=True,
        )
    runtime["visual"] = visual_rows
    runtime["visual_policy"] = {
        "exact_source_mesh_retained": True,
        "simplification": False,
        "triangle_set_preserved": True,
        "mujoco_stl_face_limit": 200_000,
        "chunk_face_limit": MAX_FACES_PER_CHUNK,
    }
    RUNTIME_MANIFEST.write_text(
        json.dumps(runtime, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": "PASS", "manifest": str(RUNTIME_MANIFEST)}))


if __name__ == "__main__":
    main()
