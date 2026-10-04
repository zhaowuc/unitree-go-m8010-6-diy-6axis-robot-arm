from __future__ import annotations

"""Generate the V15.14 ROS description without changing frozen V15.13 geometry.

The generated model keeps every kinematic joint and fixed camera/TCP transform
verbatim.  It replaces the coarse one-mesh-per-link collision representation
with the 25 V15.14 collision proxies (24 general proxies plus the pair-scoped
upper-arm motion proxy), then appends a topic-based ros2_control system.  It
also derives a V15.14 MJCF bundle without modifying the V15.13 authority.
"""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from v15_14_collision_layer import (
    build_v15_14_contract,
    collect_proxy_meshes as collect_local_proxy_meshes,
    derive_v15_14_guard,
    derive_v15_14_mjcf,
    file_sha256 as collision_file_sha256,
    validate_export_manifest,
    validate_frozen_proxy_meshes,
    validate_runtime_token_coverage,
)


V15_14_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = V15_14_ROOT.parent
SOURCE_XACRO = (
    PROJECT_ROOT
    / "完整工程_V15_13_交付"
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_description"
    / "urdf"
    / "go_m8010_arm_v15_13.urdf.xacro"
)
SOURCE_COLLISION_CONTRACT = PROJECT_ROOT / "V15_13_自碰撞对矩阵契约.json"
SOURCE_MJCF_ROOT = PROJECT_ROOT / "mujoco_kinematic_v1"
SOURCE_MJCF = SOURCE_MJCF_ROOT / "go_m8010_arm_v15_13_kinematic.xml"
SOURCE_GUARD = SOURCE_MJCF_ROOT / "kinematic_guard.py"
SOURCE_MESH_MANIFEST = SOURCE_MJCF_ROOT / "mesh_export_manifest.json"
OUTPUT_COLLISION_CONTRACT = V15_14_ROOT / "config" / "collision_pair_contract_v15_14.json"
OUTPUT_COLLISION_MANIFEST = V15_14_ROOT / "config" / "collision_layer_manifest_v15_14.json"
EXPORT_MANIFEST = V15_14_ROOT / "config" / "upperarm_motion_proxy_export_v15_14.json"
PROXY_MESH_ROOT = (
    V15_14_ROOT
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_v15_14_description"
    / "meshes"
    / "collision_proxies"
)
OUTPUT_MJCF_ROOT = V15_14_ROOT / "mujoco_v15_14"
OUTPUT_MJCF = OUTPUT_MJCF_ROOT / "go_m8010_arm_v15_14_kinematic.xml"
OUTPUT_GUARD = OUTPUT_MJCF_ROOT / "kinematic_guard.py"
OUTPUT_MJCF_CONTRACT = OUTPUT_MJCF_ROOT / "collision_pair_contract_v15_14.json"
OUTPUT_XACRO = (
    V15_14_ROOT
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_v15_14_description"
    / "urdf"
    / "go_m8010_arm_v15_14.urdf.xacro"
)
OUTPUT_SRDF = (
    V15_14_ROOT
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_v15_14_moveit_config"
    / "config"
    / "go_m8010_arm_v15_14.srdf"
)
OUTPUT_BASELINE = V15_14_ROOT / "config" / "frozen_geometry_baseline.json"
ACCEPTED_FROZEN_CONTRACT = (
    V15_14_ROOT / "config" / "accepted_frozen_contract_v15_13.json"
)

XACRO_NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO_NS)

RIGID_LINKS = (
    "base_link",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "gripper",
)
FROZEN_JOINTS = (
    "J1",
    "J2",
    "J3",
    "J4",
    "J5",
    "J6",
    "J6_to_gripper",
    "gripper_to_tcp_nominal",
    "gripper_to_camera_link",
    "camera_link_to_sim_camera_optical_frame",
)
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")


def canonical_element(element: ET.Element) -> dict:
    return {
        "tag": element.tag,
        "attributes": dict(sorted(element.attrib.items())),
        "text": (element.text or "").strip(),
        "children": [canonical_element(child) for child in list(element)],
    }


def canonical_sha256(element: ET.Element) -> str:
    payload = json.dumps(
        canonical_element(element), ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def collect_proxy_meshes(contract: dict) -> dict[str, Path]:
    return collect_local_proxy_meshes(PROXY_MESH_ROOT, contract)


def proxy_link_name(token: str) -> str:
    return f"collision_proxy__{token}"


def insert_xacro_arg(root: ET.Element, name: str, default: str) -> None:
    arg_tag = f"{{{XACRO_NS}}}arg"
    children = list(root)
    insert_at = max(
        (index for index, child in enumerate(children) if child.tag == arg_tag),
        default=-1,
    ) + 1
    root.insert(insert_at, ET.Element(arg_tag, {"name": name, "default": default}))


def add_proxy_collision_links(root: ET.Element, proxy_meshes: dict[str, Path]) -> None:
    root.append(
        ET.Comment(
            " V15.14 MoveIt collision model: 25 accepted component proxies; "
            "no visual or inertial data are added. "
        )
    )
    for token, relative_mesh in sorted(
        proxy_meshes.items(), key=lambda item: item[1].as_posix()
    ):
        parent_link = relative_mesh.parts[0]
        link_name = proxy_link_name(token)
        link = ET.SubElement(root, "link", {"name": link_name})
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(collision, "geometry")
        mesh_path = relative_mesh.as_posix()
        ET.SubElement(
            geometry,
            "mesh",
            {
                "filename": (
                    "package://go_m8010_arm_v15_14_description/"
                    f"meshes/collision_proxies/{mesh_path}"
                ),
                "scale": "0.001 0.001 0.001",
            },
        )
        fixed_joint = ET.SubElement(
            root, "joint", {"name": f"{parent_link}_to_{link_name}", "type": "fixed"}
        )
        ET.SubElement(fixed_joint, "parent", {"link": parent_link})
        ET.SubElement(fixed_joint, "child", {"link": link_name})
        ET.SubElement(fixed_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})


def add_ros2_control(root: ET.Element) -> None:
    root.append(
        ET.Comment(
            " Simulation-only command/state transport. FollowJointTrajectory is "
            "provided by joint_trajectory_controller, not by the MuJoCo bridge. "
        )
    )
    control = ET.SubElement(root, "ros2_control", {"name": "MujocoTopicSystem", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "topic_based_ros2_control/TopicBasedSystem"
    ET.SubElement(hardware, "param", {"name": "joint_commands_topic"}).text = "$(arg mujoco_joint_commands_topic)"
    ET.SubElement(hardware, "param", {"name": "joint_states_topic"}).text = "$(arg mujoco_joint_states_topic)"
    ET.SubElement(hardware, "param", {"name": "trigger_joint_command_threshold"}).text = "-1"
    ET.SubElement(hardware, "param", {"name": "sum_wrapped_joint_states"}).text = "false"
    for joint_name in JOINTS:
        joint = ET.SubElement(control, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(joint, "command_interface", {"name": "velocity"})
        position_state = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(position_state, "param", {"name": "initial_value"}).text = "0.0"
        ET.SubElement(joint, "state_interface", {"name": "velocity"})


def generate_xacro(proxy_meshes: dict[str, Path]) -> tuple[ET.Element, ET.Element]:
    source_root = ET.parse(SOURCE_XACRO).getroot()
    generated_root = ET.fromstring(ET.tostring(source_root, encoding="utf-8"))
    generated_root.set("name", "go_m8010_arm_v15_14")

    source_joints = {joint.get("name"): joint for joint in source_root.findall("joint")}
    for link_name in RIGID_LINKS:
        link = generated_root.find(f"link[@name='{link_name}']")
        if link is None:
            raise RuntimeError(f"missing rigid link {link_name}")
        collisions = list(link.findall("collision"))
        if len(collisions) != 1:
            raise RuntimeError(f"expected one legacy collision on {link_name}")
        link.remove(collisions[0])

    insert_xacro_arg(generated_root, "mujoco_joint_commands_topic", "/mujoco/joint_commands")
    insert_xacro_arg(generated_root, "mujoco_joint_states_topic", "/mujoco/joint_states_raw")
    add_proxy_collision_links(generated_root, proxy_meshes)
    add_ros2_control(generated_root)

    generated_joints = {joint.get("name"): joint for joint in generated_root.findall("joint")}
    for joint_name in FROZEN_JOINTS:
        if canonical_element(source_joints[joint_name]) != canonical_element(generated_joints[joint_name]):
            raise RuntimeError(f"frozen joint changed while generating: {joint_name}")
    return source_root, generated_root


def generate_srdf(contract: dict) -> ET.Element:
    root = ET.Element("robot", {"name": "go_m8010_arm_v15_14"})
    arm = ET.SubElement(root, "group", {"name": "arm"})
    ET.SubElement(arm, "chain", {"base_link": "base_link", "tip_link": "tcp_nominal"})
    tool = ET.SubElement(root, "group", {"name": "tcp_nominal_tool"})
    ET.SubElement(tool, "link", {"name": "tcp_nominal"})
    ET.SubElement(
        root,
        "end_effector",
        {
            "name": "tcp_nominal",
            "parent_link": "gripper",
            "group": "tcp_nominal_tool",
            "parent_group": "arm",
        },
    )
    zero = ET.SubElement(root, "group_state", {"name": "mechanical_zero", "group": "arm"})
    for joint_name in JOINTS:
        ET.SubElement(zero, "joint", {"name": joint_name, "value": "0"})
    for first, second in contract["runtime_excluded_pairs"]:
        ET.SubElement(
            root,
            "disable_collisions",
            {
                "link1": proxy_link_name(first),
                "link2": proxy_link_name(second),
                "reason": "V15.14_pair_scoped_collision_contract",
            },
        )
    return root


def write_xml(root: ET.Element, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    path.write_text('<?xml version="1.0"?>\n' + body + "\n", encoding="utf-8")


def write_json(value: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def main() -> None:
    accepted = json.loads(ACCEPTED_FROZEN_CONTRACT.read_text(encoding="utf-8"))
    source_xacro_sha256 = file_sha256(SOURCE_XACRO)
    source_collision_contract_sha256 = file_sha256(SOURCE_COLLISION_CONTRACT)
    source_root = ET.parse(SOURCE_XACRO).getroot()
    source_joints = {joint.get("name"): joint for joint in source_root.findall("joint")}
    source_joint_hashes = {
        name: canonical_sha256(source_joints[name]) for name in FROZEN_JOINTS
    }
    accepted_failures = []
    if source_xacro_sha256 != accepted["accepted_source_xacro_sha256"]:
        accepted_failures.append("source_xacro_sha256")
    if (
        source_collision_contract_sha256
        != accepted["accepted_collision_contract_sha256"]
    ):
        accepted_failures.append("source_collision_contract_sha256")
    if source_joint_hashes != accepted["frozen_joint_canonical_sha256"]:
        accepted_failures.append("frozen_joint_canonical_sha256")
    if accepted_failures:
        raise RuntimeError(
            "accepted V15.13 frozen contract mismatch; no outputs written: "
            + ", ".join(accepted_failures)
        )

    contract = build_v15_14_contract(SOURCE_COLLISION_CONTRACT)
    export_manifest = validate_export_manifest(EXPORT_MANIFEST, V15_14_ROOT)
    proxy_meshes = collect_proxy_meshes(contract)
    inherited_mesh_validation = validate_frozen_proxy_meshes(
        SOURCE_MESH_MANIFEST, PROXY_MESH_ROOT, proxy_meshes
    )
    source_root, generated_root = generate_xacro(proxy_meshes)
    generated_srdf = generate_srdf(contract)
    generated_mjcf, mjcf_metadata = derive_v15_14_mjcf(
        SOURCE_MJCF, OUTPUT_MJCF, export_manifest
    )
    generated_guard, guard_metadata = derive_v15_14_guard(SOURCE_GUARD)
    coverage = validate_runtime_token_coverage(
        contract, generated_root, generated_mjcf, proxy_meshes
    )

    generated_joints = {
        joint.get("name"): joint for joint in generated_root.findall("joint")
    }
    all_frozen_joints_identical = all(
        canonical_element(source_joints[name])
        == canonical_element(generated_joints[name])
        for name in FROZEN_JOINTS
    )
    if not all_frozen_joints_identical:
        raise RuntimeError("V15.14 generated Xacro changed a frozen joint")

    # No output is written until all source hashes, pair partitioning, CAD mesh
    # provenance, frozen transforms and runtime token coverage have passed.
    write_json(contract, OUTPUT_COLLISION_CONTRACT)
    write_json(contract, OUTPUT_MJCF_CONTRACT)
    write_xml(generated_root, OUTPUT_XACRO)
    write_xml(generated_srdf, OUTPUT_SRDF)
    write_xml(generated_mjcf, OUTPUT_MJCF)
    OUTPUT_GUARD.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_GUARD.write_text(generated_guard, encoding="utf-8")

    generated_hashes = {
        "collision_contract_sha256": collision_file_sha256(
            OUTPUT_COLLISION_CONTRACT
        ),
        "xacro_sha256": collision_file_sha256(OUTPUT_XACRO),
        "srdf_sha256": collision_file_sha256(OUTPUT_SRDF),
        "mjcf_sha256": collision_file_sha256(OUTPUT_MJCF),
        "guard_sha256": collision_file_sha256(OUTPUT_GUARD),
        "mjcf_contract_sha256": collision_file_sha256(OUTPUT_MJCF_CONTRACT),
        "motion_export_manifest_sha256": collision_file_sha256(EXPORT_MANIFEST),
        "motion_urdf_mesh_sha256": collision_file_sha256(
            V15_14_ROOT / export_manifest["urdf_mesh"]["path"]
        ),
    }
    collision_layer_manifest = {
        "schema": "go-m8010-arm-v15.14-collision-layer-manifest/1.0",
        "source_authorities": {
            "v15_13_xacro": str(SOURCE_XACRO),
            "v15_13_xacro_sha256": source_xacro_sha256,
            "v15_13_collision_contract": str(SOURCE_COLLISION_CONTRACT),
            "v15_13_collision_contract_sha256": source_collision_contract_sha256,
            "v15_13_mjcf": str(SOURCE_MJCF),
            "v15_13_mjcf_sha256": mjcf_metadata["source_mjcf_sha256"],
            "v15_13_guard": str(SOURCE_GUARD),
            "v15_13_guard_sha256": guard_metadata["source_guard_sha256"],
            "v15_13_mesh_export_manifest": str(SOURCE_MESH_MANIFEST),
            "v15_13_mesh_export_manifest_sha256": inherited_mesh_validation[
                "source_manifest_sha256"
            ],
        },
        "pair_partition": {
            "proxy_count": contract["proxy_count"],
            "all_unordered_pair_count": contract["all_unordered_pair_count"],
            "runtime_full_pair_count": contract["runtime_full_pair_count"],
            "runtime_excluded_pair_count": contract[
                "runtime_excluded_pair_count"
            ],
            "specialized_pair_geometry": contract[
                "specialized_pair_geometry"
            ],
        },
        "motion_proxy_export": export_manifest,
        "inherited_proxy_mesh_validation": inherited_mesh_validation,
        "runtime_token_coverage": coverage,
        "derived_mjcf": mjcf_metadata,
        "derived_guard": guard_metadata,
        "generated_files": {
            "collision_contract": str(OUTPUT_COLLISION_CONTRACT),
            "xacro": str(OUTPUT_XACRO),
            "srdf": str(OUTPUT_SRDF),
            "mjcf": str(OUTPUT_MJCF),
            "guard": str(OUTPUT_GUARD),
            **generated_hashes,
        },
    }
    write_json(collision_layer_manifest, OUTPUT_COLLISION_MANIFEST)

    baseline = {
        "schema": "go-m8010-arm-v15.14-frozen-geometry-baseline/2.0",
        "source_xacro": str(SOURCE_XACRO),
        "accepted_frozen_contract": str(ACCEPTED_FROZEN_CONTRACT),
        "accepted_frozen_contract_sha256": file_sha256(ACCEPTED_FROZEN_CONTRACT),
        "accepted_contract_pass": True,
        "source_xacro_sha256": source_xacro_sha256,
        "source_v15_13_collision_contract_sha256": source_collision_contract_sha256,
        "derived_v15_14_collision_contract_sha256": generated_hashes[
            "collision_contract_sha256"
        ],
        "proxy_count": len(proxy_meshes),
        "all_unordered_pair_count": contract["all_unordered_pair_count"],
        "runtime_checked_pair_count": len(contract["runtime_full_pairs"]),
        "runtime_excluded_pair_count": len(contract["runtime_excluded_pairs"]),
        "runtime_token_coverage": coverage,
        "inherited_proxy_mesh_validation": inherited_mesh_validation,
        "derived_mjcf": mjcf_metadata,
        "generated_hashes": generated_hashes,
        "frozen_joint_canonical_sha256": source_joint_hashes,
        "generated_joint_canonical_sha256": {
            name: canonical_sha256(generated_joints[name]) for name in FROZEN_JOINTS
        },
        "all_frozen_joints_identical": all_frozen_joints_identical,
        "simulation_only_limits": {
            "status": "PROVISIONAL_SOFTWARE_EXECUTION_ENVELOPE_NOT_HARDWARE_RATING",
            "position_limits": "inherited unchanged from V15.13",
            "velocity_rad_s": 0.5,
            "acceleration_rad_s2": 1.0,
        },
    }
    write_json(baseline, OUTPUT_BASELINE)
    print(json.dumps(baseline, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
