from __future__ import annotations

"""V15.13 runtime swept-collision guard including the Gemini camera.

This module extends the validated V14 J1..J6 + adaptive gripper guard with the
saved conservative Gemini collision proxy.  It also routes collision queries
involving the repaired STL-derived connector through VTK triangle contact
testing because OCC ``distToShape`` can stall on that repaired BRep.

The guard never saves the FCStd document.
"""

import importlib.util
import json
import math
from pathlib import Path

import FreeCAD as App
import vtk


ROOT = Path(__file__).resolve().parent
V14_GUARD = ROOT / "j123456_gripper_physical_collision_guard_v14.py"
CAMERA_PROXY_NAME = "Gemini_Pro_Camera_Collision_Proxy"
REVISION = "V15.13-camera-up-zero-camera-collision-runtime-guard"
CONTACT_DISTANCE_TOLERANCE_MM = 1.0e-6

if not V14_GUARD.is_file():
    raise FileNotFoundError(V14_GUARD)

spec = importlib.util.spec_from_file_location("_v15_13_v14_guard", str(V14_GUARD))
_v14 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(_v14)


def _append_unique(values, value):
    return tuple(values) if value in values else tuple(values) + (value,)


_v14.GRIPPER_STATIC_BODY_NAMES = _append_unique(
    _v14.GRIPPER_STATIC_BODY_NAMES, CAMERA_PROXY_NAME
)
_v14.GRIPPER_REQUIRED_SHAPE_NAMES = _append_unique(
    _v14.GRIPPER_REQUIRED_SHAPE_NAMES, CAMERA_PROXY_NAME
)
_v14.GRIPPER_REQUIRED_NAMES = _append_unique(
    _v14.GRIPPER_REQUIRED_NAMES, CAMERA_PROXY_NAME
)
_v14.ROTOR_SAME_RIGID_NAMES = set(_v14.ROTOR_SAME_RIGID_NAMES) | {
    CAMERA_PROXY_NAME
}
for fixed_name in (
    _v14.CONNECTOR_NAME,
    _v14.FIXED_FRAME_NAME,
    _v14.SERVO_PROXY_NAME,
):
    _v14.SAME_RIGID_EXCLUDED_PAIRS.add(
        frozenset((CAMERA_PROXY_NAME, fixed_name))
    )

_ORIGINAL_GUARD_PAIR_RESULT = _v14._guard_pair_result
_LOCAL_POLY_CACHE = {}


def _shape_signature(shape):
    box = shape.BoundBox
    return (
        float(shape.Volume),
        int(len(shape.Vertexes)),
        int(len(shape.Faces)),
        float(box.XMin),
        float(box.YMin),
        float(box.ZMin),
        float(box.XMax),
        float(box.YMax),
        float(box.ZMax),
    )


def _local_poly_data(obj, deflection_mm=0.12):
    local_shape = obj.Shape.copy()
    local_shape.Placement = App.Placement()
    signature = (_shape_signature(local_shape), float(deflection_mm))
    key = (obj.Document.Name, obj.Name)
    cached = _LOCAL_POLY_CACHE.get(key)
    if cached is not None and cached[0] == signature:
        return cached[1]

    vertices, faces = local_shape.tessellate(float(deflection_mm))
    points = vtk.vtkPoints()
    points.SetNumberOfPoints(len(vertices))
    for index, point in enumerate(vertices):
        points.SetPoint(index, float(point.x), float(point.y), float(point.z))
    cells = vtk.vtkCellArray()
    for first, second, third in faces:
        triangle = vtk.vtkTriangle()
        triangle.GetPointIds().SetId(0, int(first))
        triangle.GetPointIds().SetId(1, int(second))
        triangle.GetPointIds().SetId(2, int(third))
        cells.InsertNextCell(triangle)
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    poly.BuildCells()
    poly.BuildLinks()
    _LOCAL_POLY_CACHE[key] = (signature, poly)
    return poly


def _vtk_matrix(placement):
    source = placement.toMatrix()
    result = vtk.vtkMatrix4x4()
    values = (
        (source.A11, source.A12, source.A13, source.A14),
        (source.A21, source.A22, source.A23, source.A24),
        (source.A31, source.A32, source.A33, source.A34),
        (source.A41, source.A42, source.A43, source.A44),
    )
    for row in range(4):
        for column in range(4):
            result.SetElement(row, column, float(values[row][column]))
    return result


def _world_poly_data(obj):
    transform = vtk.vtkTransform()
    transform.SetMatrix(_vtk_matrix(obj.getGlobalPlacement()))
    transformed = vtk.vtkTransformPolyDataFilter()
    transformed.SetInputData(_local_poly_data(obj))
    transformed.SetTransform(transform)
    transformed.Update()
    result = vtk.vtkPolyData()
    result.DeepCopy(transformed.GetOutput())
    return result


def _minimum_surface_distance(first, second):
    distance = vtk.vtkDistancePolyDataFilter()
    distance.SetInputData(0, first)
    distance.SetInputData(1, second)
    distance.SignedDistanceOff()
    distance.ComputeSecondDistanceOn()
    distance.Update()
    minima = []
    for output in (distance.GetOutput(), distance.GetSecondDistanceOutput()):
        scalars = output.GetPointData().GetScalars()
        if scalars is None:
            continue
        for index in range(scalars.GetNumberOfTuples()):
            minima.append(abs(float(scalars.GetTuple1(index))))
    return min(minima) if minima else None


def _vtk_pair_result(doc, first_name, second_name, compute_clearance=False):
    first_obj = doc.getObject(first_name)
    second_obj = doc.getObject(second_name)
    if first_obj is None or second_obj is None:
        raise RuntimeError(f"missing V15.13 collision object: {first_name}, {second_name}")
    first = _world_poly_data(first_obj)
    second = _world_poly_data(second_obj)
    first_bounds = first.GetBounds()
    second_bounds = second.GetBounds()
    if any(
        first_bounds[2 * axis + 1] < second_bounds[2 * axis]
        or second_bounds[2 * axis + 1] < first_bounds[2 * axis]
        for axis in range(3)
    ):
        return {
            "overlap_mm3": 0.0,
            "clearance_mm": None,
            "collision": False,
            "broad_phase": "VTK_AABB_SEPARATED",
        }

    identity0 = vtk.vtkTransform()
    identity1 = vtk.vtkTransform()
    collision = vtk.vtkCollisionDetectionFilter()
    collision.SetInputData(0, first)
    collision.SetInputData(1, second)
    collision.SetTransform(0, identity0)
    collision.SetTransform(1, identity1)
    collision.SetBoxTolerance(1.0e-5)
    collision.SetCellTolerance(1.0e-6)
    collision.SetNumberOfCellsPerNode(2)
    collision.SetCollisionModeToFirstContact()
    collision.GenerateScalarsOff()
    collision.Update()
    contacts = int(collision.GetNumberOfContacts())
    clearance = None
    if contacts == 0 and compute_clearance:
        clearance = _minimum_surface_distance(first, second)
    return {
        "overlap_mm3": None if contacts else 0.0,
        "clearance_mm": clearance,
        "collision": contacts > 0,
        "broad_phase": "VTK_TRIANGLE_FIRST_CONTACT",
        "vtk_contact_count": contacts,
        "triangle_counts": [
            int(first.GetNumberOfCells()),
            int(second.GetNumberOfCells()),
        ],
    }


def _guard_pair_result_v15(
    doc,
    first_name,
    first_shape,
    second_name,
    second_shape,
    compute_clearance=False,
):
    del first_shape, second_shape
    if _v14.CONNECTOR_NAME in (first_name, second_name):
        return _vtk_pair_result(
            doc,
            first_name,
            second_name,
            compute_clearance=compute_clearance,
        )
    return _ORIGINAL_GUARD_PAIR_RESULT(
        doc,
        first_name,
        _v14.world_shape(doc.getObject(first_name)),
        second_name,
        _v14.world_shape(doc.getObject(second_name)),
        compute_clearance,
    )


_v14._guard_pair_result = _guard_pair_result_v15


def active_document():
    doc = _v14.active_v14_document()
    camera = doc.getObject(CAMERA_PROXY_NAME)
    if camera is None or camera.Shape.isNull() or not camera.Shape.isValid():
        raise RuntimeError("V15.13 camera collision proxy is missing or invalid")
    return doc


def collision_result(doc=None, compute_clearance=True):
    result = dict(
        _v14.collision_result(
            doc or active_document(), compute_clearance=compute_clearance
        )
    )
    result["scope"] = (
        "J1..J6 complete chain + adaptive gripper + conservative Gemini camera proxy"
    )
    result["revision"] = REVISION
    result["camera_collision_proxy"] = CAMERA_PROXY_NAME
    result["connector_pair_solver"] = "VTK triangle first-contact fallback"
    return result


def install():
    doc = active_document()
    result = _v14.install()
    status = doc.getObject(_v14._v13["STATUS_NAME"])
    if status is not None:
        if "V15CameraGuardRevision" not in status.PropertiesList:
            status.addProperty(
                "App::PropertyString",
                "V15CameraGuardRevision",
                "Runtime Collision Guard V15.13",
            )
        status.V15CameraGuardRevision = REVISION
    App.collision_result_v15_13 = collision_result
    App.install_collision_guard_v15_13 = install
    App.set_j6_angle_v15_13 = _v14.set_j6_angle_v14
    App.set_gripper_closure_angle_v15_13 = _v14.set_gripper_closure_angle_v14
    print(
        "V15.13 J1..J6 + gripper + Gemini camera swept collision guard installed.\n"
    )
    return result


set_j6_angle_v15_13 = _v14.set_j6_angle_v14
set_gripper_closure_angle_v15_13 = _v14.set_gripper_closure_angle_v14
set_j1_angle_v15_13 = _v14.set_j1_angle_v14
set_j2_j3_j4_j5_angles_v15_13 = _v14.set_j2_j3_j4_j5_angles_v14


if __name__ == "__main__":
    installed = install()
    print(json.dumps(installed, ensure_ascii=False, indent=2, default=str))
