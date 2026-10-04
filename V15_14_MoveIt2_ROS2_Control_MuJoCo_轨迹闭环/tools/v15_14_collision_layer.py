from __future__ import annotations

"""Pair-scoped V15.14 collision-layer contract and model helpers.

V15.13 remains the immutable geometry/kinematics authority.  V15.14 adds the
already-existing CAD ``UpperArm_Motion_Collision_Proxy`` as a collision-only,
pair-scoped geometry.  It is enabled only against the fixed and moving J1
proxies; its 22 other unordered pairs are deliberately excluded.
"""

import hashlib
import itertools
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


MOTION_TOKEN = "UpperArm_Motion_Collision_Proxy"
J1_FIXED_TOKEN = "J1_Fixed_Collision_Proxy"
J1_MOVING_TOKEN = "J1_Moving_Collision_Proxy"
MOTION_ENABLED_PAIRS = {
    tuple(sorted((J1_FIXED_TOKEN, MOTION_TOKEN))),
    tuple(sorted((J1_MOVING_TOKEN, MOTION_TOKEN))),
}

EXPECTED_SOURCE_CONTRACT_SHA256 = (
    "75156E7FC1683F7309AA686BD7D714DAC0883B6145E4BE93C7ED3A1D4A95CCEA"
)
EXPECTED_SOURCE_MJCF_SHA256 = (
    "26F9E0EFE10B7D208378D816540098625A34826069FDF7BBAFC1DB94CF073941"
)
EXPECTED_SOURCE_GUARD_SHA256 = (
    "1EA118C453FDF10183BF314B722A68C7969E6E68020B8356B0ABBF24CB3E40F6"
)
EXPECTED_SOURCE_MESH_MANIFEST_SHA256 = (
    "22CFB9A194495B442731405A023CEC8F83FB03D17B2E814CF13FEBC924D1CE3A"
)
EXPECTED_MOTION_STL_SHA256 = (
    "F67DACE5D0D6327BE1D2091E93B0F538823AC526DB9750E604969BFDF2F75C20"
)

EXPECTED_PROXY_COUNT = 25
EXPECTED_ALL_PAIR_COUNT = 300
EXPECTED_RUNTIME_PAIR_COUNT = 231
EXPECTED_EXCLUDED_PAIR_COUNT = 69

COLLISION_GEOM_RE = re.compile(r"^collision__[^_].*?__(.+)__\d+$")


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def canonical_element(element: ET.Element) -> dict:
    return {
        "tag": element.tag,
        "attributes": dict(sorted(element.attrib.items())),
        "text": (element.text or "").strip(),
        "children": [canonical_element(child) for child in list(element)],
    }


def canonical_sha256(element: ET.Element) -> str:
    payload = json.dumps(
        canonical_element(element),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest().upper()


def normalized_pair(first: str, second: str) -> tuple[str, str]:
    if first == second:
        raise ValueError(f"self pair is invalid: {first}")
    return tuple(sorted((str(first), str(second))))


def normalized_pairs(rows) -> list[list[str]]:
    return [list(pair) for pair in sorted({normalized_pair(*row) for row in rows})]


def build_v15_14_contract(source_path: Path) -> dict:
    source_hash = file_sha256(source_path)
    if source_hash != EXPECTED_SOURCE_CONTRACT_SHA256:
        raise RuntimeError(
            "frozen V15.13 collision contract hash mismatch: "
            f"{source_hash} != {EXPECTED_SOURCE_CONTRACT_SHA256}"
        )
    source = json.loads(source_path.read_text(encoding="utf-8"))
    source_proxies = {str(token) for token in source["proxies"]}
    if len(source_proxies) != 24 or MOTION_TOKEN in source_proxies:
        raise RuntimeError("unexpected V15.13 proxy universe")

    proxies = sorted(source_proxies | {MOTION_TOKEN})
    all_pairs = {
        normalized_pair(first, second)
        for first, second in itertools.combinations(proxies, 2)
    }
    runtime_pairs = {
        normalized_pair(first, second) for first, second in source["runtime_full_pairs"]
    }
    if {pair for pair in runtime_pairs if MOTION_TOKEN in pair} != MOTION_ENABLED_PAIRS:
        raise RuntimeError("V15.13 runtime contract lost the two pair-scoped Motion pairs")
    if not runtime_pairs <= all_pairs:
        raise RuntimeError("runtime pair contains a token outside the V15.14 proxy universe")
    excluded_pairs = all_pairs - runtime_pairs

    contract = {
        "schema": "go-m8010-arm-v15.14-self-collision-pair-contract/2.0",
        "revision": "V15.14-pair-scoped-upperarm-motion-proxy",
        "source_v15_13_contract": str(source_path),
        "source_v15_13_contract_sha256": source_hash,
        "proxy_count": len(proxies),
        "all_unordered_pair_count": len(all_pairs),
        "runtime_full_pair_count": len(runtime_pairs),
        "runtime_excluded_pair_count": len(excluded_pairs),
        "proxies": proxies,
        "runtime_full_pairs": normalized_pairs(runtime_pairs),
        "runtime_excluded_pairs": normalized_pairs(excluded_pairs),
        "specialized_pair_geometry": {
            "token": MOTION_TOKEN,
            "parent_link": "link2",
            "role": "collision-only alternate excluding the permitted J2 sleeve/motor mounting neighborhood",
            "enabled_pairs": normalized_pairs(MOTION_ENABLED_PAIRS),
            "excluded_pair_count": len(
                {pair for pair in excluded_pairs if MOTION_TOKEN in pair}
            ),
            "general_upperarm_token": "UpperArm_Full_Collision_Proxy",
        },
    }
    expected = (
        EXPECTED_PROXY_COUNT,
        EXPECTED_ALL_PAIR_COUNT,
        EXPECTED_RUNTIME_PAIR_COUNT,
        EXPECTED_EXCLUDED_PAIR_COUNT,
    )
    actual = (
        contract["proxy_count"],
        contract["all_unordered_pair_count"],
        contract["runtime_full_pair_count"],
        contract["runtime_excluded_pair_count"],
    )
    if actual != expected:
        raise RuntimeError(f"V15.14 collision contract count mismatch: {actual} != {expected}")
    if contract["specialized_pair_geometry"]["excluded_pair_count"] != 22:
        raise RuntimeError("Motion proxy must be excluded against exactly 22 other proxies")
    return contract


def validate_export_manifest(manifest_path: Path, v15_14_root: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("object") != MOTION_TOKEN or manifest.get("owner") != "link2":
        raise RuntimeError("upper-arm motion export manifest identity/owner mismatch")
    raw = manifest["urdf_mesh"]
    raw_path = v15_14_root / raw["path"]
    if file_sha256(raw_path) != EXPECTED_MOTION_STL_SHA256:
        raise RuntimeError("upper-arm motion URDF STL hash mismatch")
    components = manifest.get("mjcf_components", [])
    if len(components) != 2:
        raise RuntimeError("upper-arm motion MJCF export must contain two components")
    for row in components:
        path = v15_14_root / row["path"]
        if file_sha256(path) != str(row["sha256"]).upper():
            raise RuntimeError(f"component mesh hash mismatch: {path}")
        if row.get("units") != "mm":
            raise RuntimeError(f"component mesh unit mismatch: {path}")
    return manifest


def collect_proxy_meshes(mesh_root: Path, contract: dict) -> dict[str, Path]:
    found: dict[str, Path] = {}
    for path in sorted(mesh_root.rglob("*.stl")):
        match = re.match(r"^\d+_(.+)\.stl$", path.name, flags=re.IGNORECASE)
        if not match:
            raise RuntimeError(f"unrecognized collision proxy filename: {path}")
        token = match.group(1)
        if token in found:
            raise RuntimeError(f"duplicate collision proxy token: {token}")
        found[token] = path.relative_to(mesh_root)
    expected = set(contract["proxies"])
    actual = set(found)
    if actual != expected:
        raise RuntimeError(
            f"proxy/mesh mismatch: missing={sorted(expected-actual)}, "
            f"extra={sorted(actual-expected)}"
        )
    if len(found) != EXPECTED_PROXY_COUNT:
        raise RuntimeError(f"expected {EXPECTED_PROXY_COUNT} proxies, got {len(found)}")
    return found


def validate_frozen_proxy_meshes(
    source_manifest_path: Path,
    mesh_root: Path,
    proxy_meshes: dict[str, Path],
) -> dict:
    """Prove that the 24 inherited proxy STLs are byte-identical to V15.13."""

    source_hash = file_sha256(source_manifest_path)
    if source_hash != EXPECTED_SOURCE_MESH_MANIFEST_SHA256:
        raise RuntimeError(
            "frozen V15.13 mesh export manifest hash mismatch: "
            f"{source_hash} != {EXPECTED_SOURCE_MESH_MANIFEST_SHA256}"
        )
    source = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    inherited: dict[str, dict] = {}
    for owner, link_row in source.get("links", {}).items():
        for member in link_row.get("collision_members", []):
            token = str(member["token"])
            if token in inherited:
                raise RuntimeError(f"duplicate V15.13 collision token: {token}")
            inherited[token] = {
                "owner": str(owner),
                "sha256": str(member["sha256"]).upper(),
            }
    expected_tokens = set(proxy_meshes) - {MOTION_TOKEN}
    if set(inherited) != expected_tokens or len(inherited) != 24:
        raise RuntimeError(
            "V15.13 proxy manifest/local token mismatch: "
            f"missing={sorted(expected_tokens-set(inherited))}, "
            f"extra={sorted(set(inherited)-expected_tokens)}"
        )
    mesh_hashes = {}
    for token, row in sorted(inherited.items()):
        relative = proxy_meshes[token]
        if relative.parts[0] != row["owner"]:
            raise RuntimeError(
                f"V15.13 proxy owner mismatch for {token}: "
                f"{relative.parts[0]} != {row['owner']}"
            )
        actual_hash = file_sha256(mesh_root / relative)
        if actual_hash != row["sha256"]:
            raise RuntimeError(
                f"V15.13 proxy STL hash mismatch for {token}: "
                f"{actual_hash} != {row['sha256']}"
            )
        mesh_hashes[token] = actual_hash
    return {
        "pass": True,
        "source_manifest": str(source_manifest_path),
        "source_manifest_sha256": source_hash,
        "verified_proxy_count": len(mesh_hashes),
        "proxy_sha256": mesh_hashes,
    }


def geom_token(name: str | None) -> str | None:
    if not name or not name.startswith("collision__"):
        return None
    parts = name.split("__")
    if len(parts) != 4 or parts[0] != "collision" or not parts[3].isdigit():
        return None
    return parts[2]


def mjcf_token_geoms(root: ET.Element) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for geom in root.findall(".//geom"):
        token = geom_token(geom.get("name"))
        if token is not None:
            result.setdefault(token, []).append(str(geom.get("name")))
    return {token: sorted(names) for token, names in result.items()}


def _named_element(root: ET.Element, tag: str, name: str) -> ET.Element:
    element = root.find(f".//{tag}[@name='{name}']")
    if element is None:
        raise RuntimeError(f"missing MJCF {tag} {name}")
    return element


def mjcf_frozen_hashes(root: ET.Element) -> dict[str, str]:
    elements = {
        **{f"joint:{name}": _named_element(root, "joint", name) for name in ("J1", "J2", "J3", "J4", "J5", "J6")},
        "site:tool_reference": _named_element(root, "site", "tool_reference"),
        "site:tcp_nominal": _named_element(root, "site", "tcp_nominal"),
        "body:camera_link": _named_element(root, "body", "camera_link"),
        "body:sim_camera_optical_frame": _named_element(
            root, "body", "sim_camera_optical_frame"
        ),
        "camera:sim_gemini_pro_renderer": _named_element(
            root, "camera", "sim_gemini_pro_renderer"
        ),
    }
    return {name: canonical_sha256(element) for name, element in elements.items()}


def derive_v15_14_mjcf(
    source_mjcf: Path,
    output_mjcf: Path,
    export_manifest: dict,
) -> tuple[ET.Element, dict]:
    source_hash = file_sha256(source_mjcf)
    if source_hash != EXPECTED_SOURCE_MJCF_SHA256:
        raise RuntimeError(
            f"frozen V15.13 MJCF hash mismatch: {source_hash} != {EXPECTED_SOURCE_MJCF_SHA256}"
        )
    root = ET.parse(source_mjcf).getroot()
    frozen_before = mjcf_frozen_hashes(root)
    root.set("model", "go_m8010_arm_v15_14_kinematic")

    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("source MJCF has no asset section")
    for mesh in asset.findall("mesh"):
        source_file = mesh.get("file")
        if source_file:
            mesh.set(
                "file",
                (Path("../..") / "mujoco_kinematic_v1" / Path(source_file)).as_posix(),
            )

    output_parent = output_mjcf.parent
    motion_mesh_names = []
    for index, row in enumerate(export_manifest["mjcf_components"], 1):
        component_path = output_parent.parent / row["path"]
        component_path = component_path.resolve()
        relative = Path(row["path"])
        if not relative.parts or relative.parts[0] != output_parent.name:
            raise RuntimeError(f"unexpected MJCF component path: {relative}")
        file_from_xml = Path(*relative.parts[1:]).as_posix()
        mesh_name = f"collision_link2_motion_{index:03d}"
        ET.SubElement(
            asset,
            "mesh",
            {
                "name": mesh_name,
                "file": file_from_xml,
                "scale": "0.001 0.001 0.001",
            },
        )
        motion_mesh_names.append(mesh_name)

    link2 = _named_element(root, "body", "link2")
    motion_geom_names = []
    for index, mesh_name in enumerate(motion_mesh_names, 1):
        geom_name = f"collision__link2__{MOTION_TOKEN}__{index:03d}"
        ET.SubElement(
            link2,
            "geom",
            {
                "name": geom_name,
                "type": "mesh",
                "mesh": mesh_name,
                "rgba": "0.98 0.10 0.85 0.28",
                # Collision masks stay disabled.  Explicit MJCF geom pairs below
                # activate only the two audited token pairs, including the
                # link1/link2 parent-child pair filtered by MuJoCo by default.
                "contype": "0",
                "conaffinity": "0",
                "group": "3",
                "mass": "0",
                "solref": "0.002 1",
                "solimp": "0.95 0.99 0.001",
            },
        )
        motion_geom_names.append(geom_name)

    token_geoms = mjcf_token_geoms(root)
    obstacle_geom_names = {
        J1_FIXED_TOKEN: token_geoms.get(J1_FIXED_TOKEN, []),
        J1_MOVING_TOKEN: token_geoms.get(J1_MOVING_TOKEN, []),
    }
    if not all(obstacle_geom_names.values()):
        raise RuntimeError("source MJCF is missing a J1 collision token")
    contact = root.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        worldbody = root.find("worldbody")
        insert_at = list(root).index(worldbody) if worldbody is not None else len(root)
        root.insert(insert_at, contact)
    explicit_pairs = []
    for token in (J1_FIXED_TOKEN, J1_MOVING_TOKEN):
        for obstacle_geom in obstacle_geom_names[token]:
            for motion_geom in motion_geom_names:
                ET.SubElement(
                    contact,
                    "pair",
                    {
                        "geom1": obstacle_geom,
                        "geom2": motion_geom,
                    },
                )
                explicit_pairs.append([obstacle_geom, motion_geom])

    frozen_after = mjcf_frozen_hashes(root)
    if frozen_after != frozen_before:
        changed = sorted(
            name for name in frozen_before if frozen_before[name] != frozen_after.get(name)
        )
        raise RuntimeError(f"derived MJCF changed frozen elements: {changed}")

    metadata = {
        "source_mjcf": str(source_mjcf),
        "source_mjcf_sha256": source_hash,
        "motion_component_count": len(motion_mesh_names),
        "motion_geom_names": motion_geom_names,
        "j1_fixed_geom_count": len(obstacle_geom_names[J1_FIXED_TOKEN]),
        "j1_moving_geom_count": len(obstacle_geom_names[J1_MOVING_TOKEN]),
        "explicit_geom_pair_count": len(explicit_pairs),
        "explicit_token_pairs": normalized_pairs(MOTION_ENABLED_PAIRS),
        "frozen_element_hashes_source": frozen_before,
        "frozen_element_hashes_derived": frozen_after,
        "all_frozen_elements_identical": True,
    }
    return root, metadata


def validate_runtime_token_coverage(
    contract: dict,
    urdf_root: ET.Element,
    mjcf_root: ET.Element,
    proxy_meshes: dict[str, Path],
) -> dict:
    expected = set(contract["proxies"])
    runtime_tokens = {
        token for pair in contract["runtime_full_pairs"] for token in pair
    }
    if runtime_tokens != expected:
        raise RuntimeError(
            "fail-closed: runtime pair tokens do not cover the complete proxy universe"
        )

    urdf_links = {
        str(link.get("name"))[len("collision_proxy__") :]
        for link in urdf_root.findall("link")
        if str(link.get("name", "")).startswith("collision_proxy__")
    }
    mjcf_geoms = mjcf_token_geoms(mjcf_root)
    mjcf_tokens = set(mjcf_geoms)
    mesh_tokens = set(proxy_meshes)
    failures = {}
    for label, actual in (
        ("URDF proxy links", urdf_links),
        ("MJCF collision geoms", mjcf_tokens),
        ("proxy meshes", mesh_tokens),
    ):
        if actual != expected:
            failures[label] = {
                "missing": sorted(expected - actual),
                "extra": sorted(actual - expected),
            }
    for first, second in contract["runtime_full_pairs"]:
        if first not in urdf_links or second not in urdf_links:
            failures.setdefault("runtime_pair_urdf", []).append([first, second])
        if first not in mjcf_geoms or second not in mjcf_geoms:
            failures.setdefault("runtime_pair_mjcf", []).append([first, second])
    if failures:
        raise RuntimeError(
            "fail-closed runtime collision token coverage failure: "
            + json.dumps(failures, ensure_ascii=False, sort_keys=True)
        )

    motion_geoms = [
        geom
        for geom in mjcf_root.findall(".//geom")
        if geom_token(geom.get("name")) == MOTION_TOKEN
    ]
    if len(motion_geoms) != 2 or any(
        geom.get("contype") != "0" or geom.get("conaffinity") != "0"
        for geom in motion_geoms
    ):
        raise RuntimeError("Motion MJCF geoms must be two mask-disabled components")
    explicit_token_pairs = set()
    for pair in mjcf_root.findall("./contact/pair"):
        first = geom_token(pair.get("geom1"))
        second = geom_token(pair.get("geom2"))
        if MOTION_TOKEN in (first, second):
            if first is None or second is None:
                raise RuntimeError("invalid explicit Motion geom pair")
            explicit_token_pairs.add(normalized_pair(first, second))
    if explicit_token_pairs != MOTION_ENABLED_PAIRS:
        raise RuntimeError(
            f"Motion explicit token pair mismatch: {sorted(explicit_token_pairs)}"
        )

    return {
        "pass": True,
        "runtime_pair_token_count": len(runtime_tokens),
        "urdf_proxy_token_count": len(urdf_links),
        "mjcf_proxy_token_count": len(mjcf_tokens),
        "mesh_proxy_token_count": len(mesh_tokens),
        "motion_geom_count": len(motion_geoms),
        "motion_enabled_token_pairs": normalized_pairs(explicit_token_pairs),
    }


def derive_v15_14_guard(source_guard: Path) -> tuple[str, dict]:
    source_hash = file_sha256(source_guard)
    if source_hash != EXPECTED_SOURCE_GUARD_SHA256:
        raise RuntimeError(
            f"frozen V15.13 guard hash mismatch: {source_hash} != "
            f"{EXPECTED_SOURCE_GUARD_SHA256}"
        )
    source = source_guard.read_text(encoding="utf-8")
    required = {
        '"""Fail-closed position-limit and swept self-collision guard for V15.13."""':
            '"""Fail-closed position-limit and swept self-collision guard for V15.14."""',
        'ROOT / "go_m8010_arm_v15_13_kinematic.xml"':
            'ROOT / "go_m8010_arm_v15_14_kinematic.xml"',
        'ROOT / "V15_13_自碰撞对矩阵契约.json"':
            'ROOT / "collision_pair_contract_v15_14.json"',
    }
    derived = source
    for old, new in required.items():
        if derived.count(old) != 1:
            raise RuntimeError(f"source guard derivation anchor mismatch: {old}")
        derived = derived.replace(old, new)

    coverage_anchor = (
        "        self.allowed_proxy_pairs = {\n"
        "            tuple(sorted(pair)) for pair in pair_contract[\"runtime_full_pairs\"]\n"
        "        }\n"
    )
    if derived.count(coverage_anchor) != 1:
        raise RuntimeError("source guard runtime-pair anchor mismatch")
    coverage_block = coverage_anchor + (
        "        runtime_tokens = {token for pair in self.allowed_proxy_pairs for token in pair}\n"
        "        model_tokens = set()\n"
        "        for geom_id in range(self.model.ngeom):\n"
        "            geom_name = mujoco.mj_id2name(\n"
        "                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id\n"
        "            )\n"
        "            proxy = self._proxy(geom_name)\n"
        "            if geom_name and geom_name.startswith(\"collision__\"):\n"
        "                model_tokens.add(proxy)\n"
        "        missing = sorted(runtime_tokens - model_tokens)\n"
        "        if missing:\n"
        "            raise RuntimeError(\n"
        "                \"fail-closed: runtime collision tokens have no MJCF geom: \"\n"
        "                + \", \".join(missing)\n"
        "            )\n"
    )
    derived = derived.replace(coverage_anchor, coverage_block)
    metadata = {
        "source_guard": str(source_guard),
        "source_guard_sha256": source_hash,
        "derivation": [
            "versioned model filename",
            "versioned pair-contract filename",
            "runtime pair token -> MJCF geom fail-closed assertion",
        ],
    }
    return derived, metadata
