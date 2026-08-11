from __future__ import annotations

"""Deep geometry, collision and position-limit audit for V15.13."""

import hashlib
import importlib.util
import json
import math
import random
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import vtk


ROOT = Path(__file__).resolve().parent
FCSTD = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位/"
    r"机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
)
OUTPUT_ROOT = FCSTD.parent
GUARD_PATH = ROOT / "j123456_camera_physical_collision_guard_v15_13.py"
OUTPUT_JSON = OUTPUT_ROOT / "QA_V15_13_整机深度碰撞与限位.json"
OUTPUT_MD = OUTPUT_ROOT / "QA_V15_13_整机深度碰撞与限位.md"
LIMITS_JSON = OUTPUT_ROOT / "V15_13_ROS2_MoveIt_MuJoCo_关节限位契约.json"

REVISION = "V15.13-deep-geometry-collision-limit-audit"
EXPECTED_INPUT_SHA256 = "6CF00B0EDD1895BF74466018311589F59F51320895788E22314E8965FE2771BE"
AXIS_CENTER_TOLERANCE_MM = 1.0e-5
AXIS_DIRECTION_TOLERANCE_DEG = 1.0e-6
MAX_RECORDED_COLLISION_POSES_PER_CATEGORY = 80

LIMITS_DEG = {
    "J1": None,
    "J2": (-170.0, 170.0),
    "J3": (-170.0, 170.0),
    "J4": (-116.0, 159.0),
    "J5": (-70.6, 151.2),
    "J6": (-180.0, 180.0),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def load_guard():
    specification = importlib.util.spec_from_file_location("_v15_13_guard", str(GUARD_PATH))
    module = importlib.util.module_from_spec(specification)
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


def angle_between(first: App.Vector, second: App.Vector) -> float:
    a = App.Vector(first)
    b = App.Vector(second)
    a.normalize()
    b.normalize()
    cosine = max(-1.0, min(1.0, abs(float(a.dot(b)))))
    return math.degrees(math.acos(cosine))


def joint_geometry_checks(doc) -> dict:
    rows = {}
    for index in range(1, 7):
        name = f"J{index}_Revolute"
        joint = doc.getObject(name)
        first = App.Placement(joint.Placement1)
        second = App.Placement(joint.Placement2)
        axis1 = first.Rotation.multVec(App.Vector(0, 0, 1))
        axis2 = second.Rotation.multVec(App.Vector(0, 0, 1))
        center_error = float((first.Base - second.Base).Length)
        axis_error = angle_between(axis1, axis2)
        row = {
            "center_error_mm": center_error,
            "axis_error_deg": axis_error,
            "saved_angle_deg": float(joint.Angle),
            "limit_enabled": bool(joint.EnableAngleMin and joint.EnableAngleMax),
            "limit_deg": [float(joint.AngleMin), float(joint.AngleMax)],
        }
        row["pass"] = (
            center_error <= AXIS_CENTER_TOLERANCE_MM
            and axis_error <= AXIS_DIRECTION_TOLERANCE_DEG
            and abs(row["saved_angle_deg"]) <= 1.0e-9
        )
        if index == 1:
            row["pass"] = row["pass"] and not bool(
                joint.EnableAngleMin or joint.EnableAngleMax
            )
        else:
            expected = LIMITS_DEG[f"J{index}"]
            row["pass"] = row["pass"] and row["limit_enabled"] and all(
                abs(actual - target) <= 1.0e-9
                for actual, target in zip(row["limit_deg"], expected)
            )
        rows[name] = row
    return rows


def shape_checks(doc) -> dict:
    names = (
        "J1_Fixed_Collision_Proxy",
        "J1_Moving_Collision_Proxy",
        "UpperArm_Full_Collision_Proxy",
        "J3_Output_Flange_Proxy",
        "J3_Motor_Stator_Collision_Proxy",
        "Forearm_Collision_Proxy",
        "J4_Motor_Stator_Collision_Proxy",
        "J4_Output_Flange_Collision_Proxy",
        "Wrist_Prelink_Collision_Proxy",
        "J5_Motor_Stator_Collision_Proxy",
        "J5_Output_Flange_Collision_Proxy",
        "J5_to_Damiao_J6_Adapter_Collision_Proxy",
        "J6_DM_G6220_Stator_Collision_Proxy",
        "J6_DM_G6220_Output_Rotor_Collision_Proxy",
        "J6_Gripper_Connector_Collision_Proxy",
        "Gripper_Fixed_Frame_Collision_Proxy",
        "Gripper_Servo_Collision_Proxy",
        "Gripper_Left_Outer_Link",
        "Gripper_Right_Outer_Link",
        "Gripper_Left_Drive_Link",
        "Gripper_Right_Drive_Link",
        "Gripper_Left_Finger",
        "Gripper_Right_Finger",
        "Gemini_Pro_Camera_Collision_Proxy",
    )
    rows = {}
    for name in names:
        obj = doc.getObject(name)
        present = obj is not None and hasattr(obj, "Shape") and not obj.Shape.isNull()
        valid = bool(present and obj.Shape.isValid())
        volume = float(obj.Shape.Volume) if present else 0.0
        rows[name] = {
            "present": present,
            "valid": valid,
            "volume_mm3": volume,
            "solid_count": len(obj.Shape.Solids) if present else 0,
            "pass": bool(present and valid and volume > 0.0),
        }
    return rows


def inclusive_range(start: float, stop: float, step: float) -> list[float]:
    values = []
    current = float(start)
    while current <= stop + 1.0e-9:
        values.append(round(current, 10))
        current += step
    if abs(values[-1] - stop) > 1.0e-9:
        values.append(float(stop))
    return values


def zero_pose() -> dict:
    return {f"j{index}_deg": 0.0 for index in range(1, 7)}


def build_pose_sets() -> dict[str, list[dict]]:
    result = {}
    scan_specs = {
        "independent_J1_representative_full_turn": (-180.0, 180.0, 10.0),
        "independent_J2": (-170.0, 170.0, 10.0),
        "independent_J3": (-170.0, 170.0, 10.0),
        "independent_J4": (-116.0, 159.0, 5.0),
        "independent_J5": (-70.6, 151.2, 5.0),
        "independent_J6": (-180.0, 180.0, 5.0),
    }
    for category, (lower, upper, step) in scan_specs.items():
        joint_index = int(category.split("J")[-1].split("_")[0])
        poses = []
        for angle in inclusive_range(lower, upper, step):
            pose = zero_pose()
            pose[f"j{joint_index}_deg"] = angle
            poses.append(pose)
        result[category] = poses

    grid = []
    for j5 in (-70.6, 0.0, 151.2):
        for j6 in inclusive_range(-180.0, 180.0, 30.0):
            pose = zero_pose()
            pose["j5_deg"] = j5
            pose["j6_deg"] = j6
            grid.append(pose)
    result["coupled_J5_J6_grid"] = grid

    corners = []
    for j2 in (-170.0, 170.0):
        for j3 in (-170.0, 170.0):
            for j4 in (-116.0, 159.0):
                for j5 in (-70.6, 151.2):
                    for j6 in (-180.0, 180.0):
                        pose = zero_pose()
                        pose.update(
                            {
                                "j2_deg": j2,
                                "j3_deg": j3,
                                "j4_deg": j4,
                                "j5_deg": j5,
                                "j6_deg": j6,
                            }
                        )
                        corners.append(pose)
    result["coupled_limit_corners"] = corners

    generator = random.Random(1513)
    random_poses = []
    for _ in range(150):
        random_poses.append(
            {
                "j1_deg": generator.uniform(-180.0, 180.0),
                "j2_deg": generator.uniform(-170.0, 170.0),
                "j3_deg": generator.uniform(-170.0, 170.0),
                "j4_deg": generator.uniform(-116.0, 159.0),
                "j5_deg": generator.uniform(-70.6, 151.2),
                "j6_deg": generator.uniform(-180.0, 180.0),
            }
        )
    result["coupled_random_seed_1513"] = random_poses
    return result


def local_bounds(doc, object_names) -> dict:
    result = {}
    for name in object_names:
        obj = doc.getObject(name)
        shape = obj.Shape.copy()
        shape.Placement = App.Placement()
        box = shape.BoundBox
        result[name] = (
            float(box.XMin),
            float(box.YMin),
            float(box.ZMin),
            float(box.XMax),
            float(box.YMax),
            float(box.ZMax),
        )
    return result


def world_aabb(obj, bound) -> tuple[float, ...]:
    placement = obj.getGlobalPlacement()
    points = []
    for x in (bound[0], bound[3]):
        for y in (bound[1], bound[4]):
            for z in (bound[2], bound[5]):
                points.append(placement.multVec(App.Vector(x, y, z)))
    return (
        min(point.x for point in points),
        min(point.y for point in points),
        min(point.z for point in points),
        max(point.x for point in points),
        max(point.y for point in points),
        max(point.z for point in points),
    )


def aabb_overlaps(first, second) -> bool:
    return not (
        first[3] < second[0]
        or second[3] < first[0]
        or first[4] < second[1]
        or second[4] < first[1]
        or first[5] < second[2]
        or second[5] < first[2]
    )


def apply_pose(doc, guard, pose: dict) -> None:
    doc.getObject("J6_Revolute").Angle = float(pose["j6_deg"])
    guard._v14._apply_j12345_explicit_unchecked(
        doc,
        {key: float(pose[key]) for key in (
            "j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg"
        )},
    )


def runtime_pair_matrix(doc, guard) -> list[tuple[str, str]]:
    designed_zero_contact_pairs = {
        frozenset(("J3_Motor_Stator_Collision_Proxy", "J3_Output_Flange_Proxy")),
        frozenset(("J4_Motor_Stator_Collision_Proxy", "J4_Output_Flange_Collision_Proxy")),
        frozenset(("J5_Motor_Stator_Collision_Proxy", "J5_Output_Flange_Collision_Proxy")),
    }
    pairs = []
    for specification in guard._v14._relevant_upstream_pair_specs(
        doc, {1, 2, 3, 4, 5, 6}
    ):
        if frozenset((specification[1], specification[2])) in designed_zero_contact_pairs:
            continue
        pairs.append((specification[1], specification[2]))
    names = tuple(guard._v14._collision_body_names(doc))
    for first_index, first in enumerate(names):
        for second in names[first_index + 1 :]:
            pair = frozenset((first, second))
            if pair in designed_zero_contact_pairs:
                continue
            if pair in guard._v14.SAME_RIGID_EXCLUDED_PAIRS:
                continue
            if pair in guard._v14.DESIGN_JOINT_EXCLUDED_PAIRS:
                continue
            pairs.append((first, second))
    unique = []
    seen = set()
    for first, second in pairs:
        key = tuple(sorted((first, second)))
        if key in seen:
            continue
        seen.add(key)
        unique.append((first, second))
    return unique


def changed_terminal_pair_matrix(doc, guard, full_pairs) -> list[tuple[str, str]]:
    changed_names = {
        "J6_DM_G6220_Output_Rotor_Collision_Proxy",
        *guard._v14._collision_body_names(doc),
    }
    return [
        (first, second)
        for first, second in full_pairs
        if first in changed_names or second in changed_names
    ]


class VtkPairCache:
    def __init__(self, doc, guard):
        self.doc = doc
        self.guard = guard
        self.entries = {}

    @staticmethod
    def _matrix(placement):
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

    def _entry(self, first_name, second_name):
        key = (first_name, second_name)
        entry = self.entries.get(key)
        if entry is not None:
            return entry
        first_obj = self.doc.getObject(first_name)
        second_obj = self.doc.getObject(second_name)
        transform0 = vtk.vtkTransform()
        transform1 = vtk.vtkTransform()
        collision = vtk.vtkCollisionDetectionFilter()
        collision.SetInputData(0, self.guard._local_poly_data(first_obj, 0.25))
        collision.SetInputData(1, self.guard._local_poly_data(second_obj, 0.25))
        collision.SetTransform(0, transform0)
        collision.SetTransform(1, transform1)
        collision.SetBoxTolerance(1.0e-5)
        collision.SetCellTolerance(1.0e-6)
        collision.SetNumberOfCellsPerNode(12)
        collision.SetCollisionModeToFirstContact()
        collision.GenerateScalarsOff()
        entry = (first_obj, second_obj, transform0, transform1, collision)
        self.entries[key] = entry
        return entry

    def contact(self, first_name, second_name) -> bool:
        first_obj, second_obj, transform0, transform1, collision = self._entry(
            first_name, second_name
        )
        transform0.SetMatrix(self._matrix(first_obj.getGlobalPlacement()))
        transform1.SetMatrix(self._matrix(second_obj.getGlobalPlacement()))
        transform0.Modified()
        transform1.Modified()
        collision.Update()
        return int(collision.GetNumberOfContacts()) > 0


def scan_pose_sets(doc, guard, pair_specs, pose_sets) -> dict:
    object_names = sorted({name for pair in pair_specs for name in pair})
    bounds = local_bounds(doc, object_names)
    rows = {}
    total_poses = 0
    total_narrow_phase = 0
    pair_cache = VtkPairCache(doc, guard)
    for category, poses in pose_sets.items():
        print(f"SCAN_CATEGORY {category} poses={len(poses)}", flush=True)
        collisions = []
        narrow_phase_count = 0
        for pose_index, pose in enumerate(poses):
            if pose_index % 10 == 0:
                print(
                    f"SCAN_PROGRESS {category} {pose_index}/{len(poses)}",
                    flush=True,
                )
            apply_pose(doc, guard, pose)
            aabbs = {
                name: world_aabb(doc.getObject(name), bounds[name])
                for name in object_names
            }
            active_pairs = []
            candidates = []
            for first, second in pair_specs:
                if not aabb_overlaps(aabbs[first], aabbs[second]):
                    continue
                candidates.append((first, second))
            narrow_phase_count += len(candidates)
            for first, second in candidates:
                if pair_cache.contact(first, second):
                    active_pairs.append(f"{first} vs {second}")
            if active_pairs and len(collisions) < MAX_RECORDED_COLLISION_POSES_PER_CATEGORY:
                collisions.append(
                    {
                        "pose_index": pose_index,
                        "angles_deg": {key: float(value) for key, value in pose.items()},
                        "active_pairs": active_pairs,
                    }
                )
        total_poses += len(poses)
        total_narrow_phase += narrow_phase_count
        rows[category] = {
            "pose_count": len(poses),
            "narrow_phase_pair_tests": narrow_phase_count,
            "collision_pose_count_recorded": len(collisions),
            "collision_free": not collisions,
            "collisions": collisions,
        }
    apply_pose(doc, guard, zero_pose())
    return {
        "categories": rows,
        "total_pose_count": total_poses,
        "total_narrow_phase_pair_tests": total_narrow_phase,
        "pair_matrix_size": len(pair_specs),
    }


def write_markdown(report: dict) -> None:
    scan = report["sampled_collision_audit"]
    lines = [
        "# V15.13 整机深度碰撞与限位复核",
        "",
        f"状态：**{report['status']}**",
        "",
        f"保存零位精确碰撞：`{report['saved_zero_pose']['collision']}`；检查碰撞对 `{report['saved_zero_pose']['pair_count']}` 组。",
        f"采样姿态：`{scan['total_pose_count']}`；进入 VTK 三角面窄相位的碰撞对测试：`{scan['total_narrow_phase_pair_tests']}`。",
        "",
        "| 类别 | 姿态数 | 无碰撞 | 记录的碰撞姿态数 |",
        "|---|---:|---:|---:|",
    ]
    for category, row in scan["categories"].items():
        lines.append(
            f"| {category} | {row['pose_count']} | {row['collision_free']} | {row['collision_pose_count_recorded']} |"
        )
    lines.extend(
        [
            "",
            "## 限位解释",
            "",
            "- J2～J6 的位置限位已写入 FreeCAD、ROS2/MoveIt 与 MuJoCo 契约；J6 为 `-180°～+180°`，相机上置姿态是 `0°`。",
            "- 独立关节扫描用于确认单轴范围；多轴联合空间不是矩形安全域，若联合采样中出现碰撞，运行时连续碰撞守卫和 MoveIt 自碰撞规划必须启用，不能靠缩成一组随意猜测的独立角度解决。",
            "- J1 几何上保持 continuous；实机电缆未建模，未测得电缆可承受圈数前不得允许累计无限旋转。",
            "- 速度、力矩、质量、质心和惯量没有实测数据，本工程明确留空，没有猜填。",
        ]
    )
    OUTPUT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_limit_contract(scan_result: dict) -> dict:
    independent = {
        key: value
        for key, value in scan_result["categories"].items()
        if key.startswith("independent_")
    }
    return {
        "schema": "go-m8010-arm-v15.13-joint-limit-contract/1.0",
        "revision": REVISION,
        "position_limits_deg": LIMITS_DEG,
        "position_limits_rad": {
            key: None
            if value is None
            else [math.radians(value[0]), math.radians(value[1])]
            for key, value in LIMITS_DEG.items()
        },
        "j1": {
            "ros2_joint_type": "continuous",
            "geometry_scan_representative_deg": [-180.0, 180.0],
            "real_cable_accumulated_turn_limit": None,
            "deployment_status": "REQUIRES_MEASURED_CABLE_OR_SLIP_RING_LIMIT",
        },
        "j6": {
            "ros2_joint_type": "revolute",
            "mujoco_joint_type": "hinge",
            "camera_up_mechanical_zero_deg": 0.0,
            "range_deg": [-180.0, 180.0],
            "range_rad": [-math.pi, math.pi],
        },
        "velocity_limits_rad_s": {f"J{i}": None for i in range(1, 7)},
        "effort_limits_nm": {f"J{i}": None for i in range(1, 7)},
        "velocity_effort_status": "PENDING VERIFIED MOTOR/REDUCER/THERMAL DATA; DO NOT GUESS",
        "independent_joint_scan": independent,
        "coupled_collision_policy": {
            "runtime_guard_required": True,
            "freecad_guard": "j123456_camera_physical_collision_guard_v15_13.py",
            "moveit": "enable self-collision matrix and continuous trajectory validation",
            "mujoco": "use exported collision meshes/proxies and enforce joint ranges",
        },
    }


def main() -> None:
    if not FCSTD.is_file() or not GUARD_PATH.is_file():
        raise FileNotFoundError(FCSTD if not FCSTD.is_file() else GUARD_PATH)
    input_hash = sha256(FCSTD)
    if input_hash != EXPECTED_INPUT_SHA256:
        raise RuntimeError(f"unexpected V15.13 builder output hash: {input_hash}")
    guard = load_guard()
    print("AUDIT_GUARD_LOADED", flush=True)
    doc = App.openDocument(str(FCSTD))
    if doc is None:
        raise RuntimeError("cannot open V15.13 FCStd")
    print("AUDIT_DOCUMENT_OPENED", flush=True)
    try:
        print("AUDIT_JOINT_CHECKS_BEGIN", flush=True)
        joints = joint_geometry_checks(doc)
        print("AUDIT_SHAPE_CHECKS_BEGIN", flush=True)
        shapes = shape_checks(doc)
        print("AUDIT_SHAPE_CHECKS_DONE", flush=True)
        if not all(row["pass"] for row in joints.values()):
            raise RuntimeError("joint geometry/limit checks failed")
        if not all(row["pass"] for row in shapes.values()):
            raise RuntimeError("collision proxy geometry checks failed")

        full_pair_specs = runtime_pair_matrix(doc, guard)
        pair_specs = changed_terminal_pair_matrix(doc, guard, full_pair_specs)
        print(
            f"AUDIT_PAIR_MATRIX full={len(full_pair_specs)} changed_terminal={len(pair_specs)}",
            flush=True,
        )
        zero_scan = scan_pose_sets(
            doc, guard, full_pair_specs, {"saved_zero": [zero_pose()]}
        )
        zero_row = zero_scan["categories"]["saved_zero"]
        zero_hits = zero_row["collisions"]
        if not zero_row["collision_free"]:
            raise RuntimeError("saved camera-up zero pose is in collision")
        pose_sets = build_pose_sets()
        scan = scan_pose_sets(doc, guard, pair_specs, pose_sets)
        independent_clear = all(
            row["collision_free"]
            for category, row in scan["categories"].items()
            if category.startswith("independent_")
        )
        status = "PASS_WITH_COUPLED_COLLISION_GUARD_REQUIRED" if independent_clear else "FAIL"
        report = {
            "status": status,
            "revision": REVISION,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "input_fcstd": str(FCSTD),
            "input_fcstd_sha256": input_hash,
            "joint_geometry_and_limits": joints,
            "collision_proxy_geometry": shapes,
            "saved_zero_pose": {
                "collision": False,
                "pair_count": len(full_pair_specs),
                "active_pairs": zero_hits,
                "scope": "same runtime pair matrix as V15.13 continuous guard, evaluated by VTK first-contact",
            },
            "sampled_collision_audit": scan,
            "independent_joint_ranges_clear": independent_clear,
            "interpretation": {
                "independent_limits": "must be clear in one-joint-at-a-time reference-pose scans",
                "coupled_limits": "non-rectangular collision-free subset enforced by runtime guard / MoveIt self-collision",
                "sampling_limit": "deterministic sampled audit is not a mathematical proof over continuous 6D configuration space",
                "incremental_validation": "saved zero checks full matrix; motion scan repeats all pairs involving changed J6 output/gripper/camera bodies; unchanged upstream dynamic pairs inherit immutable V15.12/V15 source QA",
                "runtime_guard_step_deg": 0.25,
            },
            "unverified_physics": {
                "mass_com_inertia": None,
                "velocity_limits": None,
                "effort_limits": None,
                "cables_and_soft_hoses": "not geometrically modeled",
            },
        }
        if status == "FAIL":
            OUTPUT_JSON.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            write_markdown(report)
            raise RuntimeError("one or more independent joint scans collided")

        limit_contract = build_limit_contract(scan)
        LIMITS_JSON.write_text(
            json.dumps(limit_contract, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        OUTPUT_JSON.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        write_markdown(report)

        record = doc.getObject("V15_13_Camera_Up_J6_Zero_Record")
        if record is not None:
            if "DeepAuditStatus" not in record.PropertiesList:
                record.addProperty("App::PropertyString", "DeepAuditStatus", "Deep QA")
            if "DeepAuditPoseCount" not in record.PropertiesList:
                record.addProperty("App::PropertyInteger", "DeepAuditPoseCount", "Deep QA")
            if "SavedZeroCollision" not in record.PropertiesList:
                record.addProperty("App::PropertyBool", "SavedZeroCollision", "Deep QA")
            if "RuntimeCoupledGuardRequired" not in record.PropertiesList:
                record.addProperty("App::PropertyBool", "RuntimeCoupledGuardRequired", "Deep QA")
            if "DeepAuditReport" not in record.PropertiesList:
                record.addProperty("App::PropertyString", "DeepAuditReport", "Deep QA")
            record.DeepAuditStatus = status
            record.DeepAuditPoseCount = int(scan["total_pose_count"])
            record.SavedZeroCollision = False
            record.RuntimeCoupledGuardRequired = True
            record.DeepAuditReport = str(OUTPUT_JSON)
            doc.recompute()
            doc.save()
    finally:
        App.closeDocument(doc.Name)

    print(
        json.dumps(
            {
                "status": status,
                "saved_zero_collision": False,
                "pair_matrix_size": scan["pair_matrix_size"],
                "sampled_pose_count": scan["total_pose_count"],
                "independent_joint_ranges_clear": independent_clear,
                "report": str(OUTPUT_JSON),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
