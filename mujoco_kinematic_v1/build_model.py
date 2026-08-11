from __future__ import annotations

"""Generate the six-axis V15.13 MuJoCo empty-load kinematic MJCF."""

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
AXIS_JSON = ROOT / "V15_13_真实关节轴与相机上置零位.json"
MESH_MANIFEST = ROOT / "runtime_mesh_manifest.json"
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
MODEL_CONTRACT = ROOT / "model_contract.json"
GROUND_Z_M = -0.09

LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper"]
JOINT_SOURCE = {"J1": "J1", "J2": "J2A", "J3": "J3", "J4": "J4", "J5": "J5", "J6": "J6"}
PARENT_CHILD = {
    "J1": ("base_link", "link1"),
    "J2": ("link1", "link2"),
    "J3": ("link2", "link3"),
    "J4": ("link3", "link4"),
    "J5": ("link4", "link5"),
    "J6": ("link5", "link6"),
}

COLORS = {
    "base_link": "0.28 0.30 0.34 1",
    "link1": "0.54 0.09 0.13 1",
    "link2": "0.62 0.12 0.16 1",
    "link3": "0.48 0.07 0.11 1",
    "link4": "0.66 0.14 0.18 1",
    "link5": "0.39 0.06 0.10 1",
    "link6": "0.20 0.22 0.25 1",
    "gripper": "0.05 0.50 0.58 1",
}


def fmt(values) -> str:
    return " ".join(f"{float(v):.15g}" for v in values)


def safe(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_")


def perpendicular_witness(axis) -> list[float]:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(axis, seed))) > 0.85:
        seed = np.array([0.0, 1.0, 0.0])
    witness = seed - axis * np.dot(axis, seed)
    witness = witness / np.linalg.norm(witness) * 0.04
    return witness.tolist()


def add_assets(
    asset: ET.Element,
    runtime: dict,
) -> tuple[dict[str, list[str]], dict[str, list[tuple[str, str]]]]:
    visual_assets: dict[str, list[str]] = {}
    collision_assets: dict[str, list[tuple[str, str]]] = {}
    for link in LINKS:
        visual = runtime["visual"][link]
        link_visual_assets = []
        for chunk_index, chunk in enumerate(visual["runtime_chunks"], 1):
            name = f"visual_{link}_{chunk_index:03d}"
            ET.SubElement(
                asset,
                "mesh",
                name=name,
                file=chunk["path"],
                scale="0.001 0.001 0.001",
            )
            link_visual_assets.append(name)
        visual_assets[link] = link_visual_assets
        link_assets = []
        for member in runtime["collision"][link]["members"]:
            for piece_index, piece in enumerate(member["pieces"], 1):
                name = f"collision_{link}_{member['member_index']:02d}_{piece_index:03d}"
                ET.SubElement(asset, "mesh", name=name, file=piece["path"])
                link_assets.append((name, member["token"]))
        collision_assets[link] = link_assets
    return visual_assets, collision_assets


def add_link_geometries(
    body: ET.Element,
    link: str,
    visual_assets: dict[str, list[str]],
    collision_assets: dict[str, list[tuple[str, str]]],
) -> None:
    for chunk_index, asset_name in enumerate(visual_assets[link], 1):
        ET.SubElement(
            body,
            "geom",
            name=f"visual__{link}__{chunk_index:03d}",
            type="mesh",
            mesh=asset_name,
            rgba=COLORS[link],
            contype="0",
            conaffinity="0",
            group="2",
            mass="0",
        )
    token_counts: dict[str, int] = {}
    for asset_name, token in collision_assets[link]:
        token_counts[token] = token_counts.get(token, 0) + 1
        # The V15.13 guard audits one intentional non-adjacent designed seat:
        # the gripper connector against the J6 stator.  Two contact mask bits
        # suppress only that pair; all other non-adjacent pairs remain active.
        if token == "J6_Gripper_Connector_Collision_Proxy":
            contype, conaffinity = "2", "6"
        elif token == "J6_DM_G6220_Stator_Collision_Proxy":
            contype, conaffinity = "1", "5"
        else:
            contype, conaffinity = "1", "7"
        # Bit 4 is reserved for the physical ground.  The fixed base_link is
        # intentionally excluded from the ground mask because its audited CAD
        # support face is exactly coplanar with the floor at Z=-90 mm.
        if link == "base_link":
            conaffinity = str(int(conaffinity) & ~4)
        ET.SubElement(
            body,
            "geom",
            name=(
                f"collision__{link}__{safe(token)}__{token_counts[token]:03d}"
            ),
            type="mesh",
            mesh=asset_name,
            rgba="0.95 0.42 0.08 0.20",
            contype=contype,
            conaffinity=conaffinity,
            group="3",
            mass="0",
            solref="0.002 1",
            solimp="0.95 0.99 0.001",
        )


def main() -> None:
    axes = json.loads(AXIS_JSON.read_text(encoding="utf-8"))
    runtime = json.loads(MESH_MANIFEST.read_text(encoding="utf-8"))
    joints_by_source = {row["name"]: row for row in axes["joints"]}

    root = ET.Element("mujoco", model="go_m8010_arm_v15_13_kinematic")
    ET.SubElement(
        root,
        "compiler",
        angle="radian",
        autolimits="true",
        fusestatic="false",
        discardvisual="false",
        meshdir=".",
    )
    ET.SubElement(root, "option", gravity="0 0 0", timestep="0.002", integrator="implicitfast")
    ET.SubElement(root, "size", njmax="4000", nconmax="2000")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "global",
        azimuth="135",
        elevation="-18",
        offwidth="960",
        offheight="720",
    )
    ET.SubElement(visual, "headlight", ambient="0.45 0.45 0.45", diffuse="0.75 0.75 0.75")
    ET.SubElement(visual, "rgba", haze="0.12 0.14 0.18 1")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="skybox",
        type="skybox",
        builtin="gradient",
        rgb1="0.18 0.21 0.27",
        rgb2="0.035 0.045 0.065",
        width="512",
        height="3072",
    )
    ET.SubElement(
        asset,
        "texture",
        name="ground_grid",
        type="2d",
        builtin="checker",
        rgb1="0.16 0.18 0.22",
        rgb2="0.24 0.27 0.32",
        mark="edge",
        markrgb="0.45 0.50 0.57",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "material",
        name="ground_grid_material",
        texture="ground_grid",
        texrepeat="40 40",
        texuniform="true",
        reflectance="0.08",
        shininess="0.15",
    )
    visual_assets, collision_assets = add_assets(asset, runtime)

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 -1.2 1.8", dir="0 0.4 -1", directional="true")
    ET.SubElement(worldbody, "camera", name="overview", pos="1.1 -1.1 0.8", xyaxes="0.707 0.707 0 -0.35 0.35 0.866")
    ET.SubElement(
        worldbody,
        "geom",
        name="ground",
        type="plane",
        pos=fmt([0.0, 0.0, GROUND_Z_M]),
        size="2 2 0.02",
        material="ground_grid_material",
        contype="4",
        conaffinity="0",
        group="0",
        friction="0.9 0.02 0.002",
        solref="0.002 1",
        solimp="0.95 0.99 0.001",
    )

    body_by_link: dict[str, ET.Element] = {}
    base = ET.SubElement(worldbody, "body", name="base_link", pos="0 0 0", quat="1 0 0 0")
    body_by_link["base_link"] = base
    add_link_geometries(base, "base_link", visual_assets, collision_assets)

    contract_joints = {}
    for joint_name in ("J1", "J2", "J3", "J4", "J5", "J6"):
        source_joint = joints_by_source[JOINT_SOURCE[joint_name]]
        parent_link, child_link = PARENT_CHILD[joint_name]
        mj = source_joint["mujoco_mjcf"]
        child = ET.SubElement(
            body_by_link[parent_link],
            "body",
            name=child_link,
            pos=fmt(mj["body_pos_m_in_parent"]),
            quat=fmt(mj["body_quat_wxyz_in_parent"]),
        )
        body_by_link[child_link] = child
        joint_attrs = {
            "name": joint_name,
            "type": "hinge",
            "pos": fmt(mj["joint_pos_m_in_child_body"]),
            "axis": fmt(mj["joint_axis_in_child_body"]),
            "ref": "0",
            "limited": "true" if mj["joint_limited"] else "false",
            "damping": "0",
            "frictionloss": "0",
            "armature": "0",
        }
        if mj["joint_limited"]:
            joint_attrs["range"] = fmt(mj["joint_range_rad"])
        ET.SubElement(child, "joint", **joint_attrs)
        # Compile-only numerical placeholder.  This model must never be used
        # for dynamics until measured mass/COM/inertia are supplied.
        ET.SubElement(child, "inertial", pos="0 0 0", mass="1", diaginertia="0.001 0.001 0.001")
        add_link_geometries(child, child_link, visual_assets, collision_assets)
        axis = mj["joint_axis_in_child_body"]
        ET.SubElement(
            child,
            "site",
            name=f"{joint_name}_axis_origin",
            pos="0 0 0",
            type="sphere",
            size="0.004",
            rgba="1 0.9 0.05 1",
            group="4",
        )
        witness = perpendicular_witness(axis)
        ET.SubElement(
            child,
            "site",
            name=f"{joint_name}_positive_witness",
            pos=fmt(witness),
            type="sphere",
            size="0.003",
            rgba="0.1 1 0.2 1",
            group="4",
        )
        contract_joints[joint_name] = {
            "source_joint": source_joint["name"],
            "parent_link": parent_link,
            "child_link": child_link,
            "body_pos_m_in_parent": mj["body_pos_m_in_parent"],
            "body_quat_wxyz_in_parent": mj["body_quat_wxyz_in_parent"],
            "axis_in_child_body": axis,
            "limited": mj["joint_limited"],
            "range_rad": mj["joint_range_rad"],
            "positive_witness_local_m": witness,
        }

    gripper = ET.SubElement(body_by_link["link6"], "body", name="gripper", pos="0 0 0", quat="1 0 0 0")
    ET.SubElement(gripper, "inertial", pos="0 0 0", mass="1", diaginertia="0.001 0.001 0.001")
    add_link_geometries(gripper, "gripper", visual_assets, collision_assets)
    ET.SubElement(gripper, "site", name="tool_reference", pos="0 0 0", type="sphere", size="0.003", rgba="0.2 0.6 1 1", group="4")

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(MODEL_XML, encoding="utf-8", xml_declaration=True)
    contract = {
        "schema": "go-m8010-arm-v15.13-mujoco-kinematic-contract/1.0",
        "revision": "V15.13-MuJoCo-empty-load-kinematic-v1",
        "model_xml": MODEL_XML.name,
        "model_purpose": "kinematics, measured axes, position limits and self-collision only",
        "dynamics_valid": False,
        "dummy_inertials": True,
        "dummy_inertial_warning": "Numerical compile placeholders only; never use for dynamics or torque control.",
        "base_frame_policy": "CAD base_link frame is mapped to MuJoCo world identity; all relative transforms are preserved.",
        "topology": "world/base_link/J1/link1/J2/link2/J3/link3/J4/link4/J5/link5/J6/link6/fixed/gripper",
        "joints": contract_joints,
        "gripper": "frozen at verified V15.13 zero pose and fixed to link6",
        "mesh_units": "visual STL mm with scale 0.001; convex collision STL metres",
        "collision_filter": (
            "MuJoCo broad/narrow phase plus V15_13_self-collision-pair contract in kinematic_guard.py; "
            "same-body/direct-parent contacts and audited designed contacts are ignored; "
            "ground uses collision bit 4 and is active for every moving link but excluded for the fixed base."
        ),
        "ground": {
            "z_m": GROUND_Z_M,
            "derivation": "exact base_link visual lower Z bound from runtime_mesh_manifest.json",
            "visual": "40x40 checker grid plus gradient skybox",
            "collision": "physical plane; collision bit 4; fixed base excluded; moving links active",
        },
    }
    MODEL_CONTRACT.write_text(json.dumps(contract, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "PASS", "xml": str(MODEL_XML), "contract": str(MODEL_CONTRACT)}))


if __name__ == "__main__":
    main()
