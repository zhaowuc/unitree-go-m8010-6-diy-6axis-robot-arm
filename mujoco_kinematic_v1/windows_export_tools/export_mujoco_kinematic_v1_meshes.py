from __future__ import annotations

"""Export corrected V15.13 rigid meshes for the MuJoCo kinematic model.

The legacy rigid-link exporter assigned a relative Placement to copied nested
TopoShapes.  Imported STEP members can already contain internal placements, so
that path could effectively count an assembly transform twice.  This exporter
uses FreeCAD's own world-space mesh export first and then *bakes* the inverse
measured Link frame into the vertices.

Collision members are also kept as separate raw meshes.  The Ubuntu-side build
decomposes them independently; this prevents a single convex hull from bridging
empty space between disconnected collision proxies.
"""

import hashlib
import json
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import Mesh
import Part

sys.path.insert(0, str(Path(__file__).resolve().parent))
import export_robot_arm_v15_13_complete_project as configured_export


exporter = configured_export.exporter

PROJECT_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
SOURCE_FCSTD = PROJECT_ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
AXIS_JSON = PROJECT_ROOT / "V15_13_真实关节轴与相机上置零位.json"
OUTPUT_ROOT = PROJECT_ROOT / "mujoco_kinematic_v1"
VISUAL_DIR = OUTPUT_ROOT / "meshes" / "visual_mm"
RAW_COLLISION_DIR = OUTPUT_ROOT / "meshes" / "collision_raw_mm"
TEMP_DIR = OUTPUT_ROOT / "_freecad_world_mesh_tmp"
MANIFEST_JSON = OUTPUT_ROOT / "mesh_export_manifest.json"

# This is the exact owner mapping used by the accepted V15.13 deep collision
# audit.  Display geometry, decorative bearings, screws and detailed motor
# STEP solids are deliberately not allowed to re-enter the collision model.
AUDITED_COLLISION_NAMES = {
    "base_link": ["J1_Fixed_Collision_Proxy"],
    "link1": ["J1_Moving_Collision_Proxy"],
    "link2": ["UpperArm_Full_Collision_Proxy", "J3_Output_Flange_Proxy"],
    "link3": [
        "J3_Motor_Stator_Collision_Proxy",
        "Forearm_Collision_Proxy",
        "J4_Motor_Stator_Collision_Proxy",
    ],
    "link4": [
        "J4_Output_Flange_Collision_Proxy",
        "Wrist_Prelink_Collision_Proxy",
        "J5_Motor_Stator_Collision_Proxy",
    ],
    "link5": [
        "J5_Output_Flange_Collision_Proxy",
        "J5_to_Damiao_J6_Adapter_Collision_Proxy",
        "J6_DM_G6220_Stator_Collision_Proxy",
    ],
    "link6": ["J6_DM_G6220_Output_Rotor_Collision_Proxy"],
    "gripper": [
        "J6_Gripper_Connector_Collision_Proxy",
        "Gripper_Fixed_Frame_Collision_Proxy",
        "Gripper_Servo_Collision_Proxy",
        "Gemini_Pro_Camera_Collision_Proxy",
    ],
}

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def safe_name(value: str) -> str:
    value = value.replace("derived:", "derived_")
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_")


def mesh_bounds(mesh: Mesh.Mesh) -> dict:
    box = mesh.BoundBox
    return {
        "min_mm": [box.XMin, box.YMin, box.ZMin],
        "max_mm": [box.XMax, box.YMax, box.ZMax],
        "size_mm": [box.XLength, box.YLength, box.ZLength],
    }


def export_world_mesh(objects: list, path: Path) -> Mesh.Mesh:
    if path.exists():
        path.unlink()
    Mesh.export(objects, str(path))
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"FreeCAD produced no mesh: {path}")
    mesh = Mesh.Mesh(str(path))
    if mesh.CountFacets == 0:
        raise RuntimeError(f"FreeCAD produced an empty mesh: {path}")
    return mesh


def bake_world_to_local(mesh: Mesh.Mesh, frame_world: App.Placement) -> Mesh.Mesh:
    result = Mesh.Mesh(mesh)
    result.transform(frame_world.inverse().toMatrix())
    result.Placement = App.Placement()
    return result


def direct_object_local_mesh(
    obj,
    frame_world: App.Placement,
    temp_path: Path,
) -> Mesh.Mesh:
    return bake_world_to_local(export_world_mesh([obj], temp_path), frame_world)


def motor_component_indices(component: str, solid_count: int) -> set[int]:
    if solid_count != exporter.EXPECTED_GO_M8010_SOLID_COUNT:
        raise RuntimeError(
            f"expected {exporter.EXPECTED_GO_M8010_SOLID_COUNT} GO-M8010 solids, "
            f"found {solid_count}"
        )
    if component == "output":
        return set(exporter.OUTPUT_SOLID_INDICES)
    if component == "output_collision":
        return set(exporter.OUTPUT_COLLISION_SOLID_INDICES)
    if component == "neutral":
        return set(exporter.NEUTRAL_SOLID_INDICES)
    if component == "stator":
        return (
            set(range(1, solid_count + 1))
            - set(exporter.OUTPUT_SOLID_INDICES)
            - set(exporter.NEUTRAL_SOLID_INDICES)
        )
    raise ValueError(component)


def motor_component_local_mesh(
    doc,
    motor_obj,
    component: str,
    frame_world: App.Placement,
    temp_path: Path,
    serial: int,
) -> Mesh.Mesh:
    source = motor_obj.Shape.copy()
    source.Placement = App.Placement()
    solids = list(source.Solids)
    indices = motor_component_indices(component, len(solids))
    selected = [solid.copy() for index, solid in enumerate(solids, 1) if index in indices]
    if len(selected) != len(indices):
        raise RuntimeError(f"{motor_obj.Name}: incomplete {component} split")

    temp_name = f"MuJoCoWorldExportMotor_{serial:03d}"
    temp_obj = doc.addObject("Part::Feature", temp_name)
    temp_obj.Label = temp_name
    temp_obj.Shape = Part.makeCompound(selected)
    # The temporary object is at document root, so the measured global
    # placement is assigned exactly once.
    temp_obj.Placement = motor_obj.getGlobalPlacement()
    doc.recompute()
    try:
        mesh_world = export_world_mesh([temp_obj], temp_path)
    finally:
        doc.removeObject(temp_obj.Name)
        doc.recompute()
    return bake_world_to_local(mesh_world, frame_world)


def spec_local_mesh(
    doc,
    spec: dict,
    frame_world: App.Placement,
    temp_path: Path,
    serial: int,
) -> Mesh.Mesh:
    obj = doc.getObject(spec["object"])
    if obj is None:
        raise RuntimeError(f"missing source object: {spec['object']}")
    if spec["kind"] == "object":
        return direct_object_local_mesh(obj, frame_world, temp_path)
    if spec["kind"] == "motor_component":
        return motor_component_local_mesh(
            doc,
            obj,
            spec["component"],
            frame_world,
            temp_path,
            serial,
        )
    raise ValueError(spec["kind"])


def token(spec: dict) -> str:
    return spec.get("token") or spec["object"]


def build() -> None:
    for required in (SOURCE_FCSTD, AXIS_JSON):
        if not required.is_file():
            raise FileNotFoundError(required)

    axis_data = json.loads(AXIS_JSON.read_text(encoding="utf-8"))
    frames = axis_data["frames_at_mechanical_zero"]
    frame_placements = {
        name: exporter.placement_from_frame(frames[name])
        for name in ("base_link", "link1", "link2", "link3", "link4", "link5", "link6")
    }
    frame_placements["gripper"] = frame_placements["link6"]

    for directory in (OUTPUT_ROOT, VISUAL_DIR, RAW_COLLISION_DIR, TEMP_DIR):
        directory.mkdir(parents=True, exist_ok=True)

    doc = App.openDocument(str(SOURCE_FCSTD))
    if doc is None:
        raise RuntimeError("could not open V15.13 FCStd")

    results: dict[str, dict] = {}
    serial = 0
    try:
        for link_name, link_spec in exporter.LINK_SPECS.items():
            frame_world = frame_placements[link_name]
            visual_specs = list(link_spec["visual"])
            collision_specs = exporter.specs_from_names(AUDITED_COLLISION_NAMES[link_name])

            # The first MuJoCo version has six arm joints only.  The gripper is
            # therefore frozen at its verified zero pose and all of its moving
            # members are welded into the fixed gripper body for this model.
            if link_name == "gripper":
                for names in exporter.GRIPPER_INTERNAL_ROLE_NAMES.values():
                    visual_specs.extend(exporter.specs_from_names(names))
                    collision_specs.append(exporter.object_spec(names[0]))

            visual_combined = Mesh.Mesh()
            visual_rows: list[dict] = []
            for index, spec in enumerate(visual_specs, 1):
                serial += 1
                local = spec_local_mesh(
                    doc,
                    spec,
                    frame_world,
                    TEMP_DIR / f"visual_{serial:04d}.stl",
                    serial,
                )
                visual_combined.addMesh(local)
                visual_rows.append(
                    {
                        "index": index,
                        "token": token(spec),
                        "facets": local.CountFacets,
                        "bounds_local": mesh_bounds(local),
                    }
                )
            if visual_combined.CountFacets == 0:
                raise RuntimeError(f"empty visual mesh: {link_name}")
            visual_path = VISUAL_DIR / f"{link_name}.stl"
            visual_combined.write(str(visual_path))

            link_collision_dir = RAW_COLLISION_DIR / link_name
            link_collision_dir.mkdir(parents=True, exist_ok=True)
            collision_rows: list[dict] = []
            for index, spec in enumerate(collision_specs, 1):
                serial += 1
                local = spec_local_mesh(
                    doc,
                    spec,
                    frame_world,
                    TEMP_DIR / f"collision_{serial:04d}.stl",
                    serial,
                )
                part_path = link_collision_dir / f"{index:02d}_{safe_name(token(spec))}.stl"
                local.write(str(part_path))
                collision_rows.append(
                    {
                        "index": index,
                        "token": token(spec),
                        "path": str(part_path),
                        "sha256": sha256(part_path),
                        "facets": local.CountFacets,
                        "bounds_local": mesh_bounds(local),
                    }
                )

            result = {
                "frame_world_at_zero": {
                    "origin_mm": frames["link6" if link_name == "gripper" else link_name][
                        "origin_world_mm"
                    ],
                    "rotation_matrix_row_major": exporter.rotation_matrix_from_frame(
                        frames["link6" if link_name == "gripper" else link_name]
                    ),
                },
                "visual_mesh": str(visual_path),
                "visual_sha256": sha256(visual_path),
                "visual_facets": visual_combined.CountFacets,
                "visual_bounds_local": mesh_bounds(visual_combined),
                "visual_members": visual_rows,
                "collision_members": collision_rows,
                "gripper_frozen_at_verified_zero": link_name == "gripper",
            }
            max_size = max(result["visual_bounds_local"]["size_mm"])
            if not 1.0 < max_size < 1000.0:
                raise RuntimeError(f"implausible corrected {link_name} visual bounds: {max_size} mm")
            results[link_name] = result
            print(
                f"{link_name}: visual facets={visual_combined.CountFacets}, "
                f"size_mm={result['visual_bounds_local']['size_mm']}, "
                f"collision_members={len(collision_rows)}"
            )
    finally:
        App.closeDocument(doc.Name)
        if TEMP_DIR.is_dir():
            shutil.rmtree(TEMP_DIR)

    manifest = {
        "schema": "go-m8010-arm-v15.13-mujoco-kinematic-meshes/1.0",
        "revision": "V15.13-MuJoCo-empty-load-kinematic-v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_fcstd": str(SOURCE_FCSTD),
        "source_fcstd_sha256": sha256(SOURCE_FCSTD),
        "axis_contract": str(AXIS_JSON),
        "axis_contract_sha256": sha256(AXIS_JSON),
        "method": (
            "FreeCAD world-space mesh export, followed by baked inverse measured-Link-frame transform"
        ),
        "units": {
            "visual_stl_numeric_unit": "mm",
            "raw_collision_stl_numeric_unit": "mm",
            "mujoco_visual_mesh_scale": [0.001, 0.001, 0.001],
        },
        "topology": (
            "world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> "
            "J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper"
        ),
        "gripper_policy": (
            "First six-axis kinematic version: verified gripper zero pose is welded; no seventh arm DOF."
        ),
        "collision_contract": {
            "source": str(PROJECT_ROOT / "QA_V15_13_整机深度碰撞与限位.json"),
            "exact_audited_proxy_count": sum(len(v) for v in AUDITED_COLLISION_NAMES.values())
            + len(exporter.GRIPPER_INTERNAL_ROLE_NAMES),
            "note": "24 audited proxy objects; gripper internal collision uses the six audited moving-role lead objects.",
        },
        "links": results,
    }
    MANIFEST_JSON.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": "PASS", "manifest": str(MANIFEST_JSON)}, ensure_ascii=False))


if __name__ == "__main__":
    build()
