from __future__ import annotations

"""Read-only FreeCAD export of the V15.14 pair-scoped upper-arm proxy.

Run from this script's directory with FreeCADCmd, for example::

    FreeCADCmd -c "exec(open('export_upperarm_motion_proxy_v15_14.py',
    encoding='utf-8').read())"

The accepted V15.13 FCStd is never saved.  FreeCAD first emits the object's
world-space tessellation, then the inverse frozen link2 frame is baked into the
mesh vertices.  This is the same transform order used by the accepted V15.13
MuJoCo exporter and avoids double-counting nested imported placements.
"""

import hashlib
import json
import tempfile
from pathlib import Path

import FreeCAD as App
import Mesh


SCRIPT_PATH = Path(globals().get("__file__", Path.cwd() / "export_upperarm_motion_proxy_v15_14.py"))
V15_14_ROOT = SCRIPT_PATH.resolve().parents[1]
PROJECT_ROOT = V15_14_ROOT.parent
SOURCE_FCSTD = (
    PROJECT_ROOT
    / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
)
AXIS_CONTRACT = PROJECT_ROOT / "V15_13_真实关节轴与相机上置零位.json"
PROXY_NAME = "UpperArm_Motion_Collision_Proxy"
EXPECTED_OWNER = "J2_Driven_Reference_Rigid"

URDF_MESH = (
    V15_14_ROOT
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_v15_14_description"
    / "meshes"
    / "collision_proxies"
    / "link2"
    / "03_UpperArm_Motion_Collision_Proxy.stl"
)
MJCF_COMPONENT_DIR = (
    V15_14_ROOT
    / "mujoco_v15_14"
    / "meshes"
    / "collision_motion_proxy_mm"
    / "link2"
)
OUTPUT_MANIFEST = V15_14_ROOT / "config" / "upperarm_motion_proxy_export_v15_14.json"

EXPECTED_SOURCE_FCSTD_SHA256 = (
    "B4A02A97AFAFF88E246A46340DAD4CACD3C645A7456FDD087A69552E14D3DDC0"
)
EXPECTED_AXIS_CONTRACT_SHA256 = (
    "9B46C1FD816530235E80C453587E58E62A1DCCF1828C72C3867CC8DF720EE5A9"
)
EXPECTED_BREP_SHA256 = (
    "2E7C0D1F1A61B612261EE537498A24FE5410886BE96FD07A9404D6604A274F7D"
)
EXPECTED_LOCAL_STL_SHA256 = (
    "F67DACE5D0D6327BE1D2091E93B0F538823AC526DB9750E604969BFDF2F75C20"
)
EXPECTED_FACETS = 512
EXPECTED_COMPONENTS = 2


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def brep_sha256(shape) -> str:
    raw = shape.exportBrepToString()
    if isinstance(raw, str):
        raw = raw.encode("utf-8")
    return hashlib.sha256(raw).hexdigest().upper()


def bounds(mesh) -> dict:
    box = mesh.BoundBox
    return {
        "min_mm": [float(box.XMin), float(box.YMin), float(box.ZMin)],
        "max_mm": [float(box.XMax), float(box.YMax), float(box.ZMax)],
        "size_mm": [float(box.XLength), float(box.YLength), float(box.ZLength)],
    }


def placement_record(placement: App.Placement) -> dict:
    return {
        "base_mm": [
            float(placement.Base.x),
            float(placement.Base.y),
            float(placement.Base.z),
        ],
        "quaternion_xyzw": [float(value) for value in placement.Rotation.Q],
    }


def link2_frame() -> App.Placement:
    contract = json.loads(AXIS_CONTRACT.read_text(encoding="utf-8-sig"))
    frame = contract["frames_at_mechanical_zero"]["link2"]
    matrix = App.Matrix()
    x_axis = frame["x_axis_world_at_zero"]
    y_axis = frame["y_axis_world_at_zero"]
    z_axis = frame["z_axis_world_at_zero"]
    origin = frame["origin_world_mm"]
    matrix.A11, matrix.A21, matrix.A31 = map(float, x_axis)
    matrix.A12, matrix.A22, matrix.A32 = map(float, y_axis)
    matrix.A13, matrix.A23, matrix.A33 = map(float, z_axis)
    matrix.A14, matrix.A24, matrix.A34 = map(float, origin)
    return App.Placement(matrix)


def component_sort_key(mesh) -> tuple:
    box = mesh.BoundBox
    return (
        round(float(box.XMin), 9),
        round(float(box.YMin), 9),
        round(float(box.ZMin), 9),
        int(mesh.CountFacets),
    )


def main() -> None:
    if sha256(SOURCE_FCSTD) != EXPECTED_SOURCE_FCSTD_SHA256:
        raise RuntimeError("frozen FCStd hash mismatch; refusing to export")
    if sha256(AXIS_CONTRACT) != EXPECTED_AXIS_CONTRACT_SHA256:
        raise RuntimeError("frozen axis/frame contract hash mismatch; refusing to export")

    doc = App.openDocument(str(SOURCE_FCSTD))
    if doc is None:
        raise RuntimeError(f"cannot open {SOURCE_FCSTD}")
    try:
        doc.recompute()
        proxy = doc.getObject(PROXY_NAME)
        if proxy is None or not hasattr(proxy, "Shape") or proxy.Shape.isNull():
            raise RuntimeError(f"missing/non-geometric {PROXY_NAME}")
        owners = {parent.Name for parent in proxy.InList}
        if EXPECTED_OWNER not in owners:
            raise RuntimeError(f"{PROXY_NAME} owner mismatch: {sorted(owners)}")
        if not proxy.Shape.isValid() or not proxy.Shape.isClosed():
            raise RuntimeError(f"{PROXY_NAME} must be valid and closed")
        if len(proxy.Shape.Solids) != EXPECTED_COMPONENTS:
            raise RuntimeError(
                f"expected {EXPECTED_COMPONENTS} proxy solids, got {len(proxy.Shape.Solids)}"
            )
        # BREP text serialization can carry transient internal identifiers on a
        # second call, so capture and reuse the first deterministic result.
        source_brep_hash = brep_sha256(proxy.Shape)
        if source_brep_hash != EXPECTED_BREP_SHA256:
            raise RuntimeError("upper-arm motion proxy B-Rep hash mismatch")

        with tempfile.TemporaryDirectory(prefix="v15_14_upperarm_motion_proxy_") as temp:
            world_stl = Path(temp) / "motion_proxy_world.stl"
            Mesh.export([proxy], str(world_stl))
            local_mesh = Mesh.Mesh(str(world_stl))
            world_bounds = bounds(local_mesh)
            local_mesh.transform(link2_frame().inverse().toMatrix())
            local_mesh.Placement = App.Placement()

            URDF_MESH.parent.mkdir(parents=True, exist_ok=True)
            local_mesh.write(str(URDF_MESH))
            if int(local_mesh.CountFacets) != EXPECTED_FACETS:
                raise RuntimeError(
                    f"expected {EXPECTED_FACETS} facets, got {local_mesh.CountFacets}"
                )
            local_hash = sha256(URDF_MESH)
            if local_hash != EXPECTED_LOCAL_STL_SHA256:
                raise RuntimeError(
                    "deterministic local STL hash mismatch: "
                    f"{local_hash} != {EXPECTED_LOCAL_STL_SHA256}"
                )

            components = sorted(local_mesh.getSeparateComponents(), key=component_sort_key)
            if len(components) != EXPECTED_COMPONENTS:
                raise RuntimeError(
                    f"expected {EXPECTED_COMPONENTS} disconnected meshes, got {len(components)}"
                )
            MJCF_COMPONENT_DIR.mkdir(parents=True, exist_ok=True)
            resolved_component_dir = MJCF_COMPONENT_DIR.resolve()
            if V15_14_ROOT.resolve() not in resolved_component_dir.parents:
                raise RuntimeError("refusing to clean component files outside V15.14")
            for stale in MJCF_COMPONENT_DIR.glob(
                "UpperArm_Motion_Collision_Proxy__component_*.stl"
            ):
                stale.unlink()

            component_rows = []
            for index, component in enumerate(components, 1):
                target = MJCF_COMPONENT_DIR / (
                    f"UpperArm_Motion_Collision_Proxy__component_{index:03d}.stl"
                )
                component.write(str(target))
                component_rows.append(
                    {
                        "index": index,
                        "path": str(target.relative_to(V15_14_ROOT)).replace("\\", "/"),
                        "units": "mm",
                        "facets": int(component.CountFacets),
                        "bounds_local_mm": bounds(component),
                        "sha256": sha256(target),
                    }
                )

        manifest = {
            "schema": "go-m8010-arm-v15.14-upperarm-motion-proxy-export/1.0",
            "source_policy": "read-only frozen V15.13 FCStd; document is never saved",
            "source_fcstd": str(SOURCE_FCSTD),
            "source_fcstd_sha256": sha256(SOURCE_FCSTD),
            "axis_contract": str(AXIS_CONTRACT),
            "axis_contract_sha256": sha256(AXIS_CONTRACT),
            "object": PROXY_NAME,
            "owner": "link2",
            "source_group": EXPECTED_OWNER,
            "brep_sha256": source_brep_hash,
            "source_shape": {
                "valid": bool(proxy.Shape.isValid()),
                "closed": bool(proxy.Shape.isClosed()),
                "solid_count": len(proxy.Shape.Solids),
                "face_count": len(proxy.Shape.Faces),
                "volume_mm3": float(proxy.Shape.Volume),
                "area_mm2": float(proxy.Shape.Area),
                "global_placement": placement_record(proxy.getGlobalPlacement()),
            },
            "transform": "FreeCAD world mesh, then inverse frozen link2 frame baked into vertices",
            "world_bounds_mm": world_bounds,
            "urdf_mesh": {
                "path": str(URDF_MESH.relative_to(V15_14_ROOT)).replace("\\", "/"),
                "units": "mm",
                "scale_to_m": [0.001, 0.001, 0.001],
                "facets": int(local_mesh.CountFacets),
                "bounds_local_mm": bounds(local_mesh),
                "sha256": sha256(URDF_MESH),
            },
            "mjcf_components": component_rows,
        }
        OUTPUT_MANIFEST.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_MANIFEST.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(manifest, ensure_ascii=True, indent=2))
    finally:
        App.closeDocument(doc.Name)


if __name__ == "__main__":
    main()
