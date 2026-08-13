#!/usr/bin/env python3
"""Independent validator for the bounded V15.16 Engineering V1 inertia freeze.

This validator deliberately does not import the builder, FreeCAD, Part, or Mesh.
It validates the frozen authorities and recomputes the engineering decisions from
the primitive evidence serialized by the builder.  The only builder execution is
an immutable ``--check`` run, bracketed by protected/output hash snapshots.
"""

from __future__ import annotations

import hashlib
import ast
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any, Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
BASELINE_COMMIT = "0fc439ea8b501802f72c5f28695f51fa4c5ae611"
TARGET_BRANCH = "agent/v15-16-inertia-engineering-v1"

MASS_LEDGER = "V15_15_实测质量账本_v1.json"
COM_LEDGER = "V15_15_COM账本_v2.json"
ACCEPTANCE_JSON = "V15_16_Engineering惯量验收_v1.json"
ACCEPTANCE_MD = "V15_16_Engineering惯量验收_v1.md"
FREEZE_JSON = "V15_16_刚体惯量_Engineering_V1.json"
FREEZE_MD = "V15_16_刚体惯量_Engineering_V1.md"
BUILDER = "tools/build_inertia_engineering_v15_16.py"
VALIDATOR = "tools/validate_inertia_engineering_v15_16.py"
EXPECTED_BUILDER_SHA256 = "c0d707d7879013fbc8035539779eb33997880299c01150242c70bc1f54278efe"

PASS_PATHS = {
    ACCEPTANCE_JSON, ACCEPTANCE_MD, FREEZE_JSON, FREEZE_MD, BUILDER, VALIDATOR,
}
FAIL_PATHS = {ACCEPTANCE_JSON, ACCEPTANCE_MD, BUILDER, VALIDATOR}

PROTECTED_HASHES = {
    MASS_LEDGER: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_实测质量账本_v1.md": "fa52587a3309ea5665197283b62ac2e165e67663070408931611fa3b7b4e946d",
    COM_LEDGER: "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    "V15_15_COM账本_v2.md": "64117a409bc37b70e436eef5fee18aa94b21239c3ccdf127adafc5f550c0e59f",
    "V15_16_刚体惯量_FAIL审计_v2.json": "f8e0d62f027c932a49b5b38930ee770fca52c60ca2c3aa64b28308934cca55b8",
    "V15_16_刚体惯量_FAIL审计_v2.md": "94810a472a6e1a651f4a0b614e237d6241bc43bca3d7f66f908065ee26cdcfc7",
    "V15_16_打印件惯量几何适用性报告.json": "7308455fcdab006f68d90401c3206ce7209575feadfd700350122f96dc61cd14",
    "tools/build_inertia_ledger_v15_16_v2.py": "914d774f9abc4b5aa8638eec9b2533790e5ee446712358fbd33558d69c09e704",
    "tools/validate_inertia_ledger_v15_16_v2.py": "22aadb9d72a3b7e1952bf53c981c889fc251f585174424be8204556595776590",
    "tools/test_inertia_math_v15_16_v2.py": "141fcce0fb2b7af622f4c447bac5f5fa1ce72cf49745fb34aac6adac02c5e7fa",
    "V15_16_质量几何去重审计_v1.json": "a355735105dc846e982d1476b05f76dba0394ff44572a3d2d31d9567e2b7a73e",
    "V15_16_质量几何去重审计_v1.md": "821146d4a7fe448d471df071857d7758c692dcfbc4ffe929101120a8cb0e05c2",
    "tools/build_mass_geometry_authority_v15_16.py": "5202959aaa9c5b9cee631335fc9ff667ff117c43d3c5ff7d0ae37b34f74318c1",
    "tools/validate_mass_geometry_authority_v15_16.py": "f44fb1f7721d419b3a7314e6e8272c8d7ac7f1ed2fd53d0a5510c8f85da169be",
    "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd": "b4a02a97afaff88e246a46340dad4cacd3c645a7456fdd087a69552e14d3ddc0",
    "rigid_links_v15_13/rigid_link_membership.csv": "3d2a4d52d679eb97917410c6e60bec22c0c269f91ed31b35fd53ff94167e1b8c",
    "rigid_links_v15_13/rigid_link_manifest.json": "8db76f9228f7a60bd017276239e3669de3cfd443c8cd36d0dd555e184606384f",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl": "67aed62b304cfef3a338b0e59b035426b8e012619c333d032e8304c389352f45",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json": "a2b58c9892797b26c4511ff2581e7224d99cd27ca2261cc864252a6e38c21815",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties.stl": "bafb8dc8b06242a844196bc63971f95a0a7dd52834b2a05ab72f40b28460a081",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties_report.json": "dbe4c0751ab7bf4c1e564ce8b078f6de96336aaa203f060b402cc68fa6483e0f",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties.stl": "631217db4ad44f53313aa4691bde3eaaec9e043c9ff83cc6549ea42bd2832760",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties_report.json": "29961b01d574e37ba28c4d7d12cba46b8836911044d29184d1457d7b0160d94f",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl": "585d32a6ac54aca69a082947f31604e5f7b44e5be14eacab2c2db4f45a10f944",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties_report.json": "561d497524b3ed21349caa69f0659f157a2486c50861abe407fad15f8e753614",
}

LINK_ORDER = ("link2", "link3", "link4", "link5", "link6", "gripper")
EXPECTED_LINK_MASS = {
    "link2": 0.7615, "link3": 1.0900, "link4": 0.6750,
    "link5": 0.5040, "link6": 0.1250, "gripper": 0.2960,
}
EXPECTED_LINK_COM_M = {
    "link2": (-0.0968260275422941, -0.0162717756433767, +0.000268520965084),
    "link3": (-0.101150064084458, +0.0340911469413685, +0.0259162751456549),
    "link4": (-0.003256288062400, +0.0266919013259697, +0.077122730778035),
    "link5": (-0.00731479345518648, -0.000133826679861508, +0.0638899122254533),
    "link6": (-1.10476363300582e-15, +1.36043032308159e-16, -0.000501195778571886),
    "gripper": (-0.00340498241672133, -0.000799702727956827, +0.0349365123622226),
}
EXPECTED_TOTAL_MASS_KG = 3.4515
SLIVER_COMPONENTS = ("J3_STATOR_EQ", "J4_STATOR_EQ", "J5_STATOR_EQ")
EXPECTED_SLIVER_DISTANCE_MM = {
    "J3_STATOR_EQ": 119.47948016273767,
    "J4_STATOR_EQ": 119.47948016273773,
    "J5_STATOR_EQ": 121.09016788156883,
}
GO_COMPONENTS = ("J2A_OUTPUT_EQ", "J2B_OUTPUT_EQ", "J3_OUTPUT_EQ", "J4_OUTPUT_EQ", "J5_OUTPUT_EQ")
PRINT_COMPONENTS = ("UPPER_ARM_PRINT_MEASURED", "FOREARM_PRINT_MEASURED", "WRIST_PRELINK_PRINT_MEASURED")
RESIDUALS = {
    "RESIDUAL_A_LINK5_ADAPTER_HARDWARE_WIRING": ("link5", 0.059, "residual_A"),
    "RESIDUAL_B_LINK4_J5_INTERFACE_HARDWARE_WIRING": ("link4", 0.046, "residual_B"),
    "RESIDUAL_C_LINK3_FOREARM_HARDWARE_WIRING": ("link3", 0.015, "residual_C"),
}

MASS_TOL_KG = 1.0e-12
COM_FREEZE_LIMIT_M = 1.0e-4
COM_PREFERRED_LIMIT_M = 1.0e-5
MATRIX_COMPARE_TOL = 5.0e-13
SYMMETRY_TOL = 1.0e-12
SLIVER_FRACTION_LIMIT = 1.0e-6
SLIVER_MASS_LIMIT_MG = 1.0
SLIVER_COM_LIMIT_MM = 0.01
SLIVER_INERTIA_LIMIT_PERCENT = 0.01
OVERLAP_LIMIT = 0.01
ACCEPTANCE_SCHEMA = "go-m8010-arm-v15.16-inertia-engineering-acceptance-v1/1.0"
FREEZE_SCHEMA = "go-m8010-arm-v15.16-rigid-inertia-engineering-v1/1.0"


class ValidationError(RuntimeError):
    pass


def require(condition: bool, code: str) -> None:
    if not condition:
        raise ValidationError(code)


def number(value: Any, code: str) -> float:
    require(isinstance(value, (int, float)) and not isinstance(value, bool), code)
    result = float(value)
    require(math.isfinite(result), code + ":NONFINITE")
    return result


def close(actual: float, expected: float, tolerance: float, code: str) -> None:
    require(abs(actual - expected) <= tolerance, f"{code}:{actual!r}!={expected!r}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(relative: str) -> dict[str, Any]:
    path = ROOT / relative
    require(path.is_file(), "MISSING_JSON:" + relative)
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    require(isinstance(value, dict), "JSON_ROOT_NOT_OBJECT:" + relative)
    return value


def field(record: Mapping[str, Any], names: Sequence[str], code: str) -> Any:
    for name in names:
        if name in record:
            return record[name]
    raise ValidationError(code + ":MISSING:" + "|".join(names))


def records(value: Any, id_names: Sequence[str], code: str) -> list[dict[str, Any]]:
    if isinstance(value, list):
        require(all(isinstance(item, dict) for item in value), code + ":LIST_MEMBER")
        return list(value)
    if isinstance(value, dict):
        result: list[dict[str, Any]] = []
        for key, item in value.items():
            require(isinstance(item, dict), code + ":MAP_MEMBER")
            copy = dict(item)
            if not any(name in copy for name in id_names):
                copy[id_names[0]] = key
            result.append(copy)
        return result
    raise ValidationError(code + ":NOT_RECORDS")


def indexed(items: Iterable[Mapping[str, Any]], names: Sequence[str], code: str) -> dict[str, Mapping[str, Any]]:
    result: dict[str, Mapping[str, Any]] = {}
    for item in items:
        key = str(field(item, names, code + ":ID"))
        require(key not in result, code + ":DUPLICATE:" + key)
        result[key] = item
    return result


def vector3(value: Any, code: str) -> tuple[float, float, float]:
    require(isinstance(value, (list, tuple)) and len(value) == 3, code)
    return tuple(number(item, code) for item in value)  # type: ignore[return-value]


def matrix3(value: Any, code: str) -> list[list[float]]:
    require(isinstance(value, list) and len(value) == 3, code)
    result: list[list[float]] = []
    for row in value:
        require(isinstance(row, list) and len(row) == 3, code)
        result.append([number(item, code) for item in row])
    return result


def vector_distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def frobenius(matrix: Sequence[Sequence[float]]) -> float:
    return math.sqrt(sum(float(item) ** 2 for row in matrix for item in row))


def matrix_difference(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(left[i][j]) - float(right[i][j]) for j in range(3)] for i in range(3)]


def matrix_add(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(left[i][j]) + float(right[i][j]) for j in range(3)] for i in range(3)]


def matrix_scale(matrix: Sequence[Sequence[float]], factor: float) -> list[list[float]]:
    return [[float(item) * factor for item in row] for row in matrix]


def matrix_determinant(matrix: Sequence[Sequence[float]]) -> float:
    a = matrix
    return (
        float(a[0][0]) * (float(a[1][1]) * float(a[2][2]) - float(a[1][2]) * float(a[2][1]))
        - float(a[0][1]) * (float(a[1][0]) * float(a[2][2]) - float(a[1][2]) * float(a[2][0]))
        + float(a[0][2]) * (float(a[1][0]) * float(a[2][1]) - float(a[1][1]) * float(a[2][0]))
    )


def require_matrix_close(actual: Any, expected: Any, code: str, relative: float = 1.0e-10, absolute: float = 1.0e-12) -> None:
    actual_matrix = matrix3(actual, code + ":ACTUAL")
    expected_matrix = matrix3(expected, code + ":EXPECTED")
    error = frobenius(matrix_difference(actual_matrix, expected_matrix))
    require(error <= max(absolute, relative * frobenius(expected_matrix)), f"{code}:{error}")


def require_vector_close(actual: Any, expected: Any, code: str, relative: float = 1.0e-10, absolute: float = 1.0e-9) -> None:
    actual_vector = vector3(actual, code + ":ACTUAL")
    expected_vector = vector3(expected, code + ":EXPECTED")
    error = vector_distance(actual_vector, expected_vector)
    expected_norm = math.sqrt(sum(value * value for value in expected_vector))
    require(error <= max(absolute, relative * expected_norm), f"{code}:{error}")


def matrix_multiply(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(float(left[i][k]) * float(right[k][j]) for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(matrix[j][i]) for j in range(3)] for i in range(3)]


def parallel_axis_matrix(mass_kg: float, displacement_m: Sequence[float]) -> list[list[float]]:
    squared = sum(float(value) ** 2 for value in displacement_m)
    return [[
        mass_kg * ((squared if i == j else 0.0) - float(displacement_m[i]) * float(displacement_m[j]))
        for j in range(3)
    ] for i in range(3)]


def symmetric_eigenvalues(matrix: Sequence[Sequence[float]]) -> list[float]:
    """Jacobi eigenvalues for a real symmetric 3x3 matrix, independent of NumPy."""
    a = [[0.5 * (float(matrix[i][j]) + float(matrix[j][i])) for j in range(3)] for i in range(3)]
    for _ in range(64):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(a[pair[0]][pair[1]]))
        if abs(a[p][q]) <= 1.0e-18 * max(1.0, max(abs(a[i][i]) for i in range(3))):
            break
        tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        t = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau)) if tau != 0.0 else 1.0
        c = 1.0 / math.sqrt(1.0 + t * t)
        s = t * c
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = a[q][p] = 0.0
        for r in range(3):
            if r in (p, q):
                continue
            arp, arq = a[r][p], a[r][q]
            a[r][p] = a[p][r] = c * arp - s * arq
            a[r][q] = a[q][r] = s * arp + c * arq
    return sorted(a[i][i] for i in range(3))


def recursively_values(value: Any, key: str) -> list[Any]:
    found: list[Any] = []
    if isinstance(value, dict):
        for name, child in value.items():
            if name == key:
                found.append(child)
            found.extend(recursively_values(child, key))
    elif isinstance(value, list):
        for child in value:
            found.extend(recursively_values(child, key))
    return found


def overlap_event_map(audit: Mapping[str, Any]) -> dict[str, list[dict[str, Any]]]:
    overlap = field(audit, ("overlap_audit",), "PROTECTED_OVERLAP_AUDIT")
    require(isinstance(overlap, dict), "PROTECTED_OVERLAP_AUDIT_TYPE")
    result: dict[str, list[dict[str, Any]]] = {}
    for event in records(field(overlap, ("events",), "PROTECTED_OVERLAP_EVENTS"), ("component_id",), "PROTECTED_OVERLAP_EVENTS"):
        result.setdefault(str(field(event, ("component_id",), "PROTECTED_EVENT_COMPONENT")), []).append(event)
    return result


def protected_v2_component_map(audit: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    return indexed(records(field(audit, ("components",), "PROTECTED_V2_COMPONENTS"), ("component_id",), "PROTECTED_V2_COMPONENTS"), ("component_id",), "PROTECTED_V2_COMPONENTS")


def git_executable() -> str:
    override = os.environ.get("V15_16_GIT_EXECUTABLE")
    candidates = [override, shutil.which("git"), r"C:\Users\91592\.codex\tmp\mingit-v15-16a2\runtime\cmd\git.exe"]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(Path(candidate).resolve())
    raise ValidationError("GIT_EXECUTABLE_NOT_FOUND")


def run_git(*arguments: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        [git_executable(), "-c", "core.quotepath=false", *arguments],
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if check and result.returncode != 0:
        raise ValidationError("GIT_FAILED:" + " ".join(arguments) + ":" + result.stderr.strip())
    return result


def changed_paths() -> set[str]:
    result: set[str] = set()
    commands = (
        ("diff", "--name-only", BASELINE_COMMIT),
        ("diff", "--name-only", "--cached"),
        ("diff", "--name-only"),
        ("ls-files", "--others", "--exclude-standard"),
    )
    for command in commands:
        output = run_git(*command).stdout
        result.update(line.strip().replace("\\", "/") for line in output.splitlines() if line.strip())
    return result


def protected_snapshot() -> dict[str, str]:
    snapshot: dict[str, str] = {}
    for relative, expected in PROTECTED_HASHES.items():
        path = ROOT / relative
        require(path.is_file(), "PROTECTED_MISSING:" + relative)
        actual = sha256_file(path)
        require(actual == expected, f"PROTECTED_HASH:{relative}:{actual}")
        snapshot[relative] = actual
    return snapshot


def output_snapshot(paths: Iterable[str]) -> dict[str, str]:
    return {relative: sha256_file(ROOT / relative) for relative in paths if (ROOT / relative).is_file()}


def serialized_hash_map(value: Any, code: str) -> dict[str, str]:
    if isinstance(value, dict):
        if all(isinstance(item, str) for item in value.values()):
            return {str(path).replace("\\", "/"): str(digest).lower() for path, digest in value.items()}
        if "items" in value:
            return serialized_hash_map(value["items"], code)
    if isinstance(value, list):
        result: dict[str, str] = {}
        for item in value:
            require(isinstance(item, dict), code + ":ITEM")
            path = str(field(item, ("path", "relative_path"), code + ":PATH")).replace("\\", "/")
            digest = str(field(item, ("sha256",), code + ":SHA")).lower()
            require(path not in result, code + ":DUPLICATE:" + path)
            result[path] = digest
        return result
    raise ValidationError(code + ":TYPE")


def validate_serialized_provenance(report: Mapping[str, Any], pass_mode: bool) -> None:
    require(report.get("schema") == ACCEPTANCE_SCHEMA, "ACCEPTANCE_SCHEMA")
    provenance = field(report, ("provenance",), "PROVENANCE")
    require(isinstance(provenance, dict), "PROVENANCE_TYPE")
    require(provenance.get("baseline_commit", provenance.get("source_commit")) == BASELINE_COMMIT, "PROVENANCE_BASELINE")
    require(provenance.get("target_branch", TARGET_BRANCH) == TARGET_BRANCH, "PROVENANCE_BRANCH")
    authorities = field(report, ("authorities",), "AUTHORITIES")
    require(isinstance(authorities, dict), "AUTHORITIES_TYPE")
    before = serialized_hash_map(field(authorities, ("protected_inputs_before",), "SERIALIZED_PROTECTED_BEFORE"), "SERIALIZED_PROTECTED_BEFORE")
    after = serialized_hash_map(field(authorities, ("protected_inputs_after",), "SERIALIZED_PROTECTED_AFTER"), "SERIALIZED_PROTECTED_AFTER")
    require(before == after, "SERIALIZED_PROTECTED_CHANGED")
    for path, digest in PROTECTED_HASHES.items():
        require(before.get(path) == digest, "SERIALIZED_PROTECTED_HASH:" + path)
    guard = field(report, ("git_scope_guard",), "GIT_SCOPE_GUARD")
    require(isinstance(guard, dict), "GIT_SCOPE_GUARD_TYPE")
    serialized_pass_allowed = {
        str(item).replace("\\", "/")
        for item in field(guard, ("allowed_changed_paths_if_pass",), "GIT_PASS_ALLOWED_PATHS")
    }
    serialized_fail_allowed = {
        str(item).replace("\\", "/")
        for item in field(guard, ("allowed_changed_paths_if_fail",), "GIT_FAIL_ALLOWED_PATHS")
    }
    require(serialized_pass_allowed == PASS_PATHS, "SERIALIZED_PASS_ALLOWED_PATHS")
    require(serialized_fail_allowed == FAIL_PATHS, "SERIALIZED_FAIL_ALLOWED_PATHS")
    require(guard.get("target_branch") == TARGET_BRANCH, "SERIALIZED_GIT_BRANCH")
    require(guard.get("source_commit") == BASELINE_COMMIT, "SERIALIZED_GIT_BASELINE")
    require(guard.get("source_commit_is_ancestor") is True, "SERIALIZED_GIT_ANCESTOR")
    require(guard.get("unexpected_changed_paths") == [], "SERIALIZED_GIT_UNEXPECTED")
    require(guard.get("pass") is True, "SERIALIZED_GIT_SCOPE_PASS")


def validate_git_scope(pass_mode: bool) -> None:
    branch = run_git("branch", "--show-current").stdout.strip()
    require(branch == TARGET_BRANCH, "WRONG_BRANCH:" + branch)
    require(run_git("cat-file", "-t", BASELINE_COMMIT).stdout.strip() == "commit", "BASELINE_MISSING")
    expected = PASS_PATHS if pass_mode else FAIL_PATHS
    actual = changed_paths()
    require(actual == expected, f"CHANGED_PATH_SET:{sorted(actual)!r}!={sorted(expected)!r}")
    if pass_mode:
        require((ROOT / FREEZE_JSON).is_file() and (ROOT / FREEZE_MD).is_file(), "PASS_FREEZE_OUTPUTS_MISSING")
    else:
        require(not (ROOT / FREEZE_JSON).exists() and not (ROOT / FREEZE_MD).exists(), "FAIL_MUST_NOT_HAVE_FREEZE_OUTPUTS")


def authority_components(mass: Mapping[str, Any]) -> tuple[dict[str, dict[str, Any]], dict[str, list[str]]]:
    ledger = field(mass, ("link_mass_ledger",), "MASS_LINK_LEDGER")
    require(isinstance(ledger, dict), "MASS_LINK_LEDGER_TYPE")
    components: dict[str, dict[str, Any]] = {}
    link_components: dict[str, list[str]] = {}
    for link in LINK_ORDER:
        entry = field(ledger, (link,), "MASS_LINK:" + link)
        require(isinstance(entry, dict), "MASS_LINK_TYPE:" + link)
        link_mass = number(field(entry, ("nominal_mass_kg",), "MASS_LINK_VALUE:" + link), "MASS_LINK_VALUE:" + link)
        close(link_mass, EXPECTED_LINK_MASS[link], MASS_TOL_KG, "MASS_AUTHORITY_LINK:" + link)
        raw_components = records(field(entry, ("components",), "MASS_COMPONENTS:" + link), ("component_id",), "MASS_COMPONENTS:" + link)
        ids: list[str] = []
        subtotal = 0.0
        for component in raw_components:
            component_id = str(field(component, ("component_id",), "MASS_COMPONENT_ID"))
            require(component_id not in components, "MASS_COMPONENT_DUPLICATE:" + component_id)
            mass_kg = number(field(component, ("nominal_mass_kg",), "MASS_COMPONENT_MASS:" + component_id), "MASS_COMPONENT_MASS:" + component_id)
            owner = str(field(component, ("ledger_link",), "MASS_COMPONENT_OWNER:" + component_id))
            require(owner == link, "MASS_COMPONENT_OWNER:" + component_id)
            components[component_id] = {"mass_kg": mass_kg, "owner_link": link}
            ids.append(component_id)
            subtotal += mass_kg
        close(subtotal, EXPECTED_LINK_MASS[link], MASS_TOL_KG, "MASS_AUTHORITY_COMPONENT_CLOSURE:" + link)
        link_components[link] = ids
    require(len(components) == 17, "MASS_AUTHORITY_COMPONENT_COUNT")
    close(sum(item["mass_kg"] for item in components.values()), EXPECTED_TOTAL_MASS_KG, MASS_TOL_KG, "MASS_AUTHORITY_TOTAL")
    close(number(field(mass, ("total_link2_to_gripper_nominal_mass_kg",), "MASS_AUTHORITY_TOTAL_FIELD"), "MASS_AUTHORITY_TOTAL_FIELD"), EXPECTED_TOTAL_MASS_KG, MASS_TOL_KG, "MASS_AUTHORITY_TOTAL_FIELD")
    return components, link_components


def validate_com_authority(
    com: Mapping[str, Any], link_components: Mapping[str, list[str]],
) -> tuple[
    dict[str, tuple[float, float, float]],
    dict[str, tuple[float, float, float]],
    dict[str, list[list[float]]],
]:
    raw_links = field(com, ("links",), "COM_LINKS")
    link_map = indexed(records(raw_links, ("link",), "COM_LINKS"), ("link",), "COM_LINKS")
    require(set(link_map) == set(LINK_ORDER), "COM_LINK_SET")
    world_coms: dict[str, tuple[float, float, float]] = {}
    link_rotations: dict[str, list[list[float]]] = {}
    for link in LINK_ORDER:
        entry = link_map[link]
        close(number(field(entry, ("mass_kg",), "COM_MASS:" + link), "COM_MASS:" + link), EXPECTED_LINK_MASS[link], MASS_TOL_KG, "COM_MASS:" + link)
        actual = vector3(field(entry, ("com_link_m",), "COM_VALUE:" + link), "COM_VALUE:" + link)
        require(vector_distance(actual, EXPECTED_LINK_COM_M[link]) <= 2.0e-14, "COM_AUTHORITY_VALUE:" + link)
        world_coms[link] = vector3(field(entry, ("com_world_m",), "COM_WORLD_VALUE:" + link), "COM_WORLD_VALUE:" + link)
        frame = field(entry, ("frame_world_at_mechanical_zero",), "COM_FRAME:" + link)
        require(isinstance(frame, dict), "COM_FRAME_TYPE:" + link)
        link_rotations[link] = matrix3(field(frame, ("rotation_matrix_row_major",), "COM_ROTATION:" + link), "COM_ROTATION:" + link)
        ids = [str(item) for item in field(entry, ("component_ids",), "COM_COMPONENT_IDS:" + link)]
        require(len(ids) == len(set(ids)), "COM_COMPONENT_DUPLICATE:" + link)
        require(set(ids) == set(link_components[link]), "COM_COMPONENT_MEMBERSHIP:" + link)
    raw_components = records(field(com, ("components",), "COM_COMPONENTS"), ("component_id",), "COM_COMPONENTS")
    component_map = indexed(raw_components, ("component_id",), "COM_COMPONENTS")
    require(len(component_map) == 17, "COM_COMPONENT_COUNT")
    component_world_coms: dict[str, tuple[float, float, float]] = {}
    for component_id, component in component_map.items():
        value_mm = vector3(field(component, ("com_world_mm",), "COM_COMPONENT_WORLD:" + component_id), "COM_COMPONENT_WORLD:" + component_id)
        component_world_coms[component_id] = tuple(value * 1.0e-3 for value in value_mm)
    return world_coms, component_world_coms, link_rotations


def component_candidate_com_world_m(record: Mapping[str, Any], component_id: str) -> tuple[float, float, float]:
    direct_names = ("com_world_m", "candidate_com_world_m", "recomputed_com_world_m")
    for name in direct_names:
        if name in record:
            return vector3(record[name], "COMPONENT_COM_WORLD:" + component_id)
    if "frozen_com_world_mm" in record:
        value_mm = vector3(record["frozen_com_world_mm"], "COMPONENT_COM_WORLD_MM:" + component_id)
        return tuple(value * 1.0e-3 for value in value_mm)
    for container_name in ("candidate_mass_properties", "canonical_mass_properties", "engineering_mass_properties"):
        container = record.get(container_name)
        if isinstance(container, dict):
            for name in direct_names:
                if name in container:
                    return vector3(container[name], "COMPONENT_COM_WORLD:" + component_id)
    raise ValidationError("COMPONENT_COM_WORLD_MISSING:" + component_id)


def validate_mass_com_reproduction(
    report: Mapping[str, Any], authority_components_map: Mapping[str, Mapping[str, Any]],
    link_components: Mapping[str, list[str]], authority_world_coms: Mapping[str, Sequence[float]],
    authority_component_world_coms: Mapping[str, Sequence[float]],
) -> tuple[dict[str, Mapping[str, Any]], dict[str, Mapping[str, Any]], float]:
    component_items = records(field(report, ("components", "component_candidates"), "REPORT_COMPONENTS"), ("component_id",), "REPORT_COMPONENTS")
    component_map = indexed(component_items, ("component_id",), "REPORT_COMPONENTS")
    require(set(component_map) == set(authority_components_map), "REPORT_COMPONENT_SET")
    link_items = records(field(report, ("links", "candidate_links"), "REPORT_LINKS"), ("link",), "REPORT_LINKS")
    link_map = indexed(link_items, ("link",), "REPORT_LINKS")
    require(set(link_map) == set(LINK_ORDER), "REPORT_LINK_SET")
    maximum_error = 0.0
    for component_id, authority in authority_components_map.items():
        record = component_map[component_id]
        close(number(field(record, ("mass_kg",), "REPORT_COMPONENT_MASS:" + component_id), "REPORT_COMPONENT_MASS:" + component_id), float(authority["mass_kg"]), MASS_TOL_KG, "REPORT_COMPONENT_MASS:" + component_id)
        require(str(field(record, ("owner_link",), "REPORT_COMPONENT_OWNER:" + component_id)) == authority["owner_link"], "REPORT_COMPONENT_OWNER:" + component_id)
        frozen_com = component_candidate_com_world_m(record, component_id)
        require(vector_distance(frozen_com, authority_component_world_coms[component_id]) <= 2.0e-12, "COMPONENT_FROZEN_COM_AUTHORITY:" + component_id)
        require(record.get("mass_location_authority") == "COM_V2_FROZEN_COMPONENT_COM", "COMPONENT_MASS_LOCATION_AUTHORITY:" + component_id)
        anchor_mm = vector3(field(record, ("applied_frozen_com_anchor_world_mm",), "COMPONENT_APPLIED_ANCHOR:" + component_id), "COMPONENT_APPLIED_ANCHOR:" + component_id)
        require(vector_distance(tuple(value * 1.0e-3 for value in anchor_mm), frozen_com) <= 2.0e-12, "COMPONENT_APPLIED_ANCHOR:" + component_id)
        require(record.get("intrinsic_tensor_reference") == "ABOUT_GEOMETRY_CENTROID_THEN_TRANSLATION_ONLY_PLACED_AT_FROZEN_COMPONENT_COM_V2", "COMPONENT_INTRINSIC_REFERENCE:" + component_id)
        require(record.get("geometry_recentered_to_frozen_com_for_inertia_model") is True, "COMPONENT_GEOMETRY_RECENTERING:" + component_id)
        if "geometry_centroid_world_mm" in record and record["geometry_centroid_world_mm"] is not None:
            geometry_centroid_mm = vector3(record["geometry_centroid_world_mm"], "COMPONENT_GEOMETRY_CENTROID:" + component_id)
            frozen_com_mm = tuple(value * 1.0e3 for value in frozen_com)
            if component_id in GO_COMPONENTS:
                require(vector_distance(geometry_centroid_mm, frozen_com_mm) <= 0.1, "GO_GEOMETRY_TO_FROZEN_ANCHOR_GT_0P1MM:" + component_id)
    for link in LINK_ORDER:
        record = link_map[link]
        mass = number(field(record, ("mass_kg",), "REPORT_LINK_MASS:" + link), "REPORT_LINK_MASS:" + link)
        close(mass, EXPECTED_LINK_MASS[link], MASS_TOL_KG, "REPORT_LINK_MASS:" + link)
        subtotal = sum(float(authority_components_map[item]["mass_kg"]) for item in link_components[link])
        close(subtotal, mass, MASS_TOL_KG, "REPORT_LINK_COMPONENT_MASS_CLOSURE:" + link)
        weighted = [0.0, 0.0, 0.0]
        for component_id in link_components[link]:
            component_mass = float(authority_components_map[component_id]["mass_kg"])
            component_com = component_candidate_com_world_m(component_map[component_id], component_id)
            for axis in range(3):
                weighted[axis] += component_mass * component_com[axis]
        reproduced = tuple(value / mass for value in weighted)
        serialized_world = vector3(field(record, ("com_world_m",), "REPORT_LINK_COM_WORLD:" + link), "REPORT_LINK_COM_WORLD:" + link)
        require(vector_distance(reproduced, serialized_world) <= 2.0e-12, "REPORT_LINK_COMPONENT_COM_CLOSURE:" + link)
        serialized_link = vector3(field(record, ("com_xyz_m_in_link_frame", "com_owner_link_m", "com_link_m", "frozen_com_xyz_m_in_link_frame"), "REPORT_LINK_COM_LINK:" + link), "REPORT_LINK_COM_LINK:" + link)
        world_error = vector_distance(reproduced, authority_world_coms[link])
        link_error = vector_distance(serialized_link, EXPECTED_LINK_COM_M[link])
        reported_reproduction = field(record, ("com_v2_reproduction",), "REPORT_COM_REPRODUCTION:" + link)
        require(isinstance(reported_reproduction, dict), "REPORT_COM_REPRODUCTION_TYPE:" + link)
        close(number(field(reported_reproduction, ("world_error_m",), "REPORT_WORLD_ERROR:" + link), "REPORT_WORLD_ERROR:" + link), world_error, max(1.0e-13, world_error * 1.0e-8), "REPORT_WORLD_ERROR:" + link)
        close(number(field(reported_reproduction, ("owner_link_error_m",), "REPORT_LINK_ERROR:" + link), "REPORT_LINK_ERROR:" + link), link_error, max(1.0e-13, link_error * 1.0e-8), "REPORT_LINK_ERROR:" + link)
        close(number(field(reported_reproduction, ("max_error_m",), "REPORT_MAX_ERROR:" + link), "REPORT_MAX_ERROR:" + link), max(world_error, link_error), max(1.0e-13, max(world_error, link_error) * 1.0e-8), "REPORT_MAX_ERROR:" + link)
        require(reported_reproduction.get("pass") is (max(world_error, link_error) <= COM_FREEZE_LIMIT_M), "REPORT_COM_PASS:" + link)
        error = max(link_error, world_error)
        maximum_error = max(maximum_error, error)
    close(sum(number(field(link_map[link], ("mass_kg",), "REPORT_TOTAL:" + link), "REPORT_TOTAL:" + link) for link in LINK_ORDER), EXPECTED_TOTAL_MASS_KG, MASS_TOL_KG, "REPORT_TOTAL_MASS")
    return link_map, component_map, maximum_error


def validate_slivers(
    report: Mapping[str, Any], authority_components_map: Mapping[str, Mapping[str, Any]],
    protected_v2: Mapping[str, Any], protected_a1: Mapping[str, Any],
) -> bool:
    raw = field(report, ("sliver_engineering_acceptance", "numerical_slivers", "sliver_audit", "numerical_sliver_engineering_acceptance"), "SLIVER_SECTION")
    if isinstance(raw, dict) and "items" in raw:
        raw = raw["items"]
    items = records(raw, ("component_id",), "SLIVER_SECTION")
    slivers = indexed(items, ("component_id",), "SLIVER_SECTION")
    require(set(slivers) == set(SLIVER_COMPONENTS), "SLIVER_COMPONENT_SET")
    v2_components = protected_v2_component_map(protected_v2)
    protected_events = overlap_event_map(protected_a1)
    all_accepted = True
    for component_id in SLIVER_COMPONENTS:
        record = slivers[component_id]
        all_events = protected_events.get(component_id, [])
        events = [event for event in all_events if event.get("classification") == "NUMERICAL_SLIVER"]
        require(len(all_events) == 6 and len(events) == 1, "PROTECTED_SLIVER_EVENT:" + component_id)
        event = events[0]
        require(record.get("left_atom_id") == field(event, ("left",), "SLIVER_EVENT_LEFT")["atom_id"], "SLIVER_LEFT_ATOM:" + component_id)
        require(record.get("right_atom_id") == field(event, ("right",), "SLIVER_EVENT_RIGHT")["atom_id"], "SLIVER_RIGHT_ATOM:" + component_id)
        common = number(field(record, ("common_volume_mm3", "overlap_volume_mm3", "conservative_common_volume_mm3"), "SLIVER_COMMON:" + component_id), "SLIVER_COMMON:" + component_id)
        component_volume = number(field(record, ("component_volume_mm3", "raw_component_volume_mm3"), "SLIVER_COMPONENT_VOLUME:" + component_id), "SLIVER_COMPONENT_VOLUME:" + component_id)
        require(common > 0.0 and component_volume > common, "SLIVER_VOLUME_DOMAIN:" + component_id)
        # Bounded magnitude check: the fast audit may vary normally, but not by an order of magnitude.
        require(0.008 <= common <= 0.033, "SLIVER_COMMON_MAGNITUDE:" + component_id)
        protected_common = number(field(event, ("common_volume_mm3",), "PROTECTED_SLIVER_COMMON"), "PROTECTED_SLIVER_COMMON")
        close(common, protected_common, max(1.0e-6, protected_common * 1.0e-6), "SLIVER_COMMON_PROTECTED:" + component_id)
        v2_path = field(v2_components[component_id], ("path_a_occt",), "SLIVER_V2_PATH:" + component_id)
        protected_volume = number(field(v2_path, ("volume_mm3",), "SLIVER_V2_VOLUME:" + component_id), "SLIVER_V2_VOLUME:" + component_id)
        close(component_volume, protected_volume, max(1.0e-7, protected_volume * 1.0e-12), "SLIVER_COMPONENT_VOLUME_PROTECTED:" + component_id)
        fraction = common / component_volume
        mass_kg = float(authority_components_map[component_id]["mass_kg"])
        close(number(field(record, ("component_mass_kg",), "SLIVER_COMPONENT_MASS:" + component_id), "SLIVER_COMPONENT_MASS:" + component_id), mass_kg, MASS_TOL_KG, "SLIVER_COMPONENT_MASS:" + component_id)
        duplicate_mass_kg = mass_kg * fraction
        duplicate_mass_mg = duplicate_mass_kg * 1.0e6
        inertia_reference = number(field(record, ("reference_inertia_frobenius_kg_m2", "component_inertia_frobenius_kg_m2"), "SLIVER_INERTIA_REFERENCE:" + component_id), "SLIVER_INERTIA_REFERENCE:" + component_id)
        protected_tensor = matrix3(field(v2_path, ("inertia_about_component_frozen_com_world_kg_m2",), "SLIVER_V2_TENSOR:" + component_id), "SLIVER_V2_TENSOR:" + component_id)
        close(inertia_reference, frobenius(protected_tensor), 1.0e-15, "SLIVER_INERTIA_REFERENCE_PROTECTED:" + component_id)
        reported_fraction = number(field(record, ("component_fraction", "component_volume_fraction", "volume_fraction"), "SLIVER_FRACTION:" + component_id), "SLIVER_FRACTION:" + component_id)
        reported_mass = number(field(record, ("estimated_duplicate_mass_mg", "duplicate_mass_bound_mg", "mass_bound_mg"), "SLIVER_MASS_BOUND:" + component_id), "SLIVER_MASS_BOUND:" + component_id)
        close(reported_fraction, fraction, max(1.0e-15, fraction * 1.0e-9), "SLIVER_RECOMPUTE_FRACTION:" + component_id)
        close(reported_mass, duplicate_mass_mg, max(1.0e-9, duplicate_mass_mg * 1.0e-8), "SLIVER_RECOMPUTE_MASS:" + component_id)
        distance_value = field(record, ("distance_bound_mm", "r_max_mm", "com_distance_bound_mm"), "SLIVER_DISTANCE:" + component_id)
        if distance_value is None:
            require(record.get("com_influence_bound_mm") is None, "FAILED_SLIVER_COM_BOUND_NOT_NULL:" + component_id)
            require(record.get("absolute_inertia_bound_kg_m2") is None, "FAILED_SLIVER_ABSOLUTE_INERTIA_NOT_NULL:" + component_id)
            require(record.get("inertia_relative_influence_bound_percent") is None, "FAILED_SLIVER_RELATIVE_INERTIA_NOT_NULL:" + component_id)
            require(record.get("classification") == "REJECTED" and record.get("pass") is False, "FAILED_SLIVER_DECISION:" + component_id)
            worker = field(record, ("worker",), "FAILED_SLIVER_WORKER:" + component_id)
            require(isinstance(worker, dict) and worker.get("status") != "PASS", "FAILED_SLIVER_WORKER_EVIDENCE:" + component_id)
            unresolved = field(report, ("unresolved_items",), "FAILED_SLIVER_UNRESOLVED")
            require(isinstance(unresolved, list), "FAILED_SLIVER_UNRESOLVED_TYPE")
            codes = {
                str(item.get("code"))
                for item in unresolved
                if isinstance(item, dict) and item.get("component_id") == component_id
            }
            require(
                {"SLIVER_QUICK_RECOMPUTE_FAILED", "NUMERICAL_SLIVER_ENGINEERING_GATES_FAILED"} <= codes,
                "FAILED_SLIVER_BLOCKERS:" + component_id,
            )
            all_accepted = False
            continue
        distance_mm = number(distance_value, "SLIVER_DISTANCE:" + component_id)
        close(distance_mm, EXPECTED_SLIVER_DISTANCE_MM[component_id], 1.0e-9, "SLIVER_DISTANCE_PROTECTED:" + component_id)
        worker = field(record, ("worker",), "SLIVER_WORKER:" + component_id)
        require(isinstance(worker, dict) and worker.get("status") == "PASS", "SLIVER_WORKER_NOT_PASS:" + component_id)
        require(distance_mm > 0.0 and inertia_reference > 0.0 and duplicate_mass_kg < mass_kg, "SLIVER_BOUND_DOMAIN:" + component_id)
        com_bound_mm = duplicate_mass_kg / (mass_kg - duplicate_mass_kg) * distance_mm
        inertia_bound_percent = 2.0 * duplicate_mass_kg * (distance_mm * 1.0e-3) ** 2 / inertia_reference * 100.0
        reported_com = number(field(record, ("com_influence_bound_mm", "com_bound_mm"), "SLIVER_COM_BOUND:" + component_id), "SLIVER_COM_BOUND:" + component_id)
        reported_inertia = number(field(record, ("inertia_relative_influence_bound_percent", "inertia_bound_percent"), "SLIVER_INERTIA_BOUND:" + component_id), "SLIVER_INERTIA_BOUND:" + component_id)
        reported_absolute_inertia = number(field(record, ("absolute_inertia_bound_kg_m2",), "SLIVER_ABSOLUTE_INERTIA:" + component_id), "SLIVER_ABSOLUTE_INERTIA:" + component_id)
        absolute_inertia_bound = 2.0 * duplicate_mass_kg * (distance_mm * 1.0e-3) ** 2
        close(reported_com, com_bound_mm, max(1.0e-12, com_bound_mm * 1.0e-8), "SLIVER_RECOMPUTE_COM:" + component_id)
        close(reported_inertia, inertia_bound_percent, max(1.0e-11, inertia_bound_percent * 1.0e-8), "SLIVER_RECOMPUTE_INERTIA:" + component_id)
        close(reported_absolute_inertia, absolute_inertia_bound, max(1.0e-16, absolute_inertia_bound * 1.0e-8), "SLIVER_RECOMPUTE_ABSOLUTE_INERTIA:" + component_id)
        accepted = (
            fraction < SLIVER_FRACTION_LIMIT
            and duplicate_mass_mg < SLIVER_MASS_LIMIT_MG
            and com_bound_mm < SLIVER_COM_LIMIT_MM
            and inertia_bound_percent < SLIVER_INERTIA_LIMIT_PERCENT
        )
        classification = str(field(record, ("classification",), "SLIVER_CLASSIFICATION:" + component_id))
        expected_classification = "ACCEPTED_NUMERICAL_SLIVER_ENGINEERING_V1" if accepted else "REJECTED"
        require(classification == expected_classification, "SLIVER_CLASSIFICATION:" + component_id)
        require(record.get("pass") is accepted, "SLIVER_PASS_FLAG:" + component_id)
        all_accepted = all_accepted and accepted
    return all_accepted


def validate_go_containment(
    report: Mapping[str, Any], authority_components_map: Mapping[str, Mapping[str, Any]],
    protected_v2: Mapping[str, Any], protected_a1: Mapping[str, Any],
) -> float:
    raw = field(report, ("go_output_full_containment", "go_output_containment", "go_output_corrections"), "GO_SECTION")
    if isinstance(raw, dict) and "items" in raw:
        raw = raw["items"]
    items = records(raw, ("component_id",), "GO_SECTION")
    go_map = indexed(items, ("component_id",), "GO_SECTION")
    require(set(go_map) == set(GO_COMPONENTS), "GO_COMPONENT_SET")
    component_map = indexed(
        records(field(report, ("components",), "GO_COMPONENT_RECORDS"), ("component_id",), "GO_COMPONENT_RECORDS"),
        ("component_id",), "GO_COMPONENT_RECORDS",
    )
    v2_components = protected_v2_component_map(protected_v2)
    protected_events = overlap_event_map(protected_a1)
    maximum_remaining = 0.0
    for component_id in GO_COMPONENTS:
        record = go_map[component_id]
        events = protected_events.get(component_id, [])
        require(len(events) == 1 and events[0].get("classification") == "FULL_CONTAINMENT", "PROTECTED_GO_EVENT:" + component_id)
        event = events[0]
        raw_volume = number(field(record, ("raw_volume_mm3",), "GO_RAW:" + component_id), "GO_RAW:" + component_id)
        occupied = number(field(record, ("occupied_volume_mm3", "corrected_volume_mm3", "union_volume_mm3"), "GO_OCCUPIED:" + component_id), "GO_OCCUPIED:" + component_id)
        removed = raw_volume - occupied
        require(raw_volume > occupied > 0.0 and removed > 0.0, "GO_VOLUME_DOMAIN:" + component_id)
        v2_path = field(v2_components[component_id], ("path_a_occt",), "GO_V2_PATH:" + component_id)
        protected_raw = number(field(v2_path, ("volume_mm3",), "GO_V2_VOLUME:" + component_id), "GO_V2_VOLUME:" + component_id)
        close(raw_volume, protected_raw, max(1.0e-7, protected_raw * 1.0e-12), "GO_RAW_PROTECTED:" + component_id)
        left = field(event, ("left",), "GO_EVENT_LEFT:" + component_id)
        right = field(event, ("right",), "GO_EVENT_RIGHT:" + component_id)
        require(isinstance(left, dict) and isinstance(right, dict), "GO_EVENT_SIDES:" + component_id)
        left_volume = number(field(field(left, ("metrics",), "GO_LEFT_METRICS"), ("volume_mm3",), "GO_LEFT_VOLUME"), "GO_LEFT_VOLUME")
        right_volume = number(field(field(right, ("metrics",), "GO_RIGHT_METRICS"), ("volume_mm3",), "GO_RIGHT_VOLUME"), "GO_RIGHT_VOLUME")
        smaller_volume = min(left_volume, right_volume)
        protected_common = number(field(event, ("common_volume_mm3",), "GO_PROTECTED_COMMON"), "GO_PROTECTED_COMMON")
        close(protected_common, smaller_volume, max(1.0e-6, smaller_volume * 1.0e-8), "GO_PROTECTED_CONTAINMENT:" + component_id)
        close(occupied, raw_volume - smaller_volume, max(1.0e-6, raw_volume * 1.0e-10), "GO_OCCUPIED_FROM_PROTECTED_EVENT:" + component_id)
        fraction = removed / raw_volume
        reported_removed = number(field(record, ("removed_volume_mm3", "duplicate_volume_removed_mm3"), "GO_REMOVED:" + component_id), "GO_REMOVED:" + component_id)
        reported_fraction = number(field(record, ("removed_fraction", "duplicate_fraction"), "GO_FRACTION:" + component_id), "GO_FRACTION:" + component_id)
        close(reported_removed, removed, max(1.0e-7, removed * 1.0e-9), "GO_RECOMPUTE_REMOVED:" + component_id)
        close(reported_fraction, fraction, max(1.0e-12, fraction * 1.0e-9), "GO_RECOMPUTE_FRACTION:" + component_id)
        if "raw_detected_overlap_fraction" in record:
            close(number(record["raw_detected_overlap_fraction"], "GO_RAW_OVERLAP:" + component_id), fraction, max(1.0e-12, fraction * 1.0e-9), "GO_RAW_OVERLAP:" + component_id)
        require(0.002 <= fraction <= 0.006, "GO_KNOWN_CORRECTION_MAGNITUDE:" + component_id)
        require(field(record, ("double_count_avoided", "double_count_removed"), "GO_DOUBLE_COUNT:" + component_id) is True, "GO_DOUBLE_COUNT:" + component_id)
        method = str(field(record, ("correction_method",), "GO_METHOD:" + component_id)).upper()
        require("COLLISION" not in method and "MESH" not in method, "GO_FORBIDDEN_METHOD:" + component_id)
        require(any(token in method for token in ("OCCT", "OCCUPIED", "CONTAINMENT", "UNION")), "GO_METHOD:" + component_id)
        remaining = number(field(record, ("remaining_or_estimated_uncertainty_fraction",), "GO_REMAINING:" + component_id), "GO_REMAINING:" + component_id)
        maximum_remaining = max(maximum_remaining, remaining)
        component = component_map[component_id]
        actions = records(field(component, ("containment_actions",), "GO_CONTAINMENT_ACTIONS:" + component_id), ("collapsed_atom_id",), "GO_CONTAINMENT_ACTIONS:" + component_id)
        require(len(actions) == 1, "GO_CONTAINMENT_ACTION_COUNT:" + component_id)
        expected_drop = str(left["atom_id"] if left_volume <= right_volume else right["atom_id"])
        expected_keep = str(right["atom_id"] if left_volume <= right_volume else left["atom_id"])
        require(actions[0].get("collapsed_atom_id") == expected_drop, "GO_COLLAPSED_ATOM:" + component_id)
        require(actions[0].get("retained_occupied_atom_id") == expected_keep, "GO_RETAINED_ATOM:" + component_id)
        if record.get("engineering_estimate_used") is False:
            close(number(field(actions[0], ("fresh_common_volume_mm3",), "GO_FRESH_COMMON:" + component_id), "GO_FRESH_COMMON:" + component_id), protected_common, max(1.0e-6, protected_common * 1.0e-8), "GO_FRESH_COMMON:" + component_id)
        component_occupied = number(field(component, ("occupied_volume_mm3",), "GO_COMPONENT_OCCUPIED:" + component_id), "GO_COMPONENT_OCCUPIED:" + component_id)
        close(component_occupied, occupied, max(1.0e-7, occupied * 1.0e-10), "GO_COMPONENT_OCCUPIED:" + component_id)
        frozen_mass = float(authority_components_map[component_id]["mass_kg"])
        density = frozen_mass / occupied
        reported_density = number(field(component, ("effective_density_kg_per_mm3",), "GO_EFFECTIVE_DENSITY:" + component_id), "GO_EFFECTIVE_DENSITY:" + component_id)
        close(reported_density, density, max(1.0e-18, density * 1.0e-10), "GO_EFFECTIVE_DENSITY:" + component_id)
        close(number(field(component, ("mass_normalization_volume_mm3",), "GO_NORMALIZATION_VOLUME:" + component_id), "GO_NORMALIZATION_VOLUME:" + component_id), occupied, max(1.0e-7, occupied * 1.0e-10), "GO_NORMALIZATION_VOLUME:" + component_id)
        close(number(field(component, ("normalized_to_frozen_mass_kg",), "GO_NORMALIZED_MASS:" + component_id), "GO_NORMALIZED_MASS:" + component_id), frozen_mass, MASS_TOL_KG, "GO_NORMALIZED_MASS:" + component_id)
        require(component.get("mass_renormalization_pass") is True, "GO_MASS_RENORMALIZATION:" + component_id)
        raw_matrix = matrix3(field(component, ("occupied_centroidal_volume_inertia_matrix_mm5",), "GO_RAW_INERTIA:" + component_id), "GO_RAW_INERTIA:" + component_id)
        expected_intrinsic = [[raw_matrix[i][j] * density * 1.0e-6 for j in range(3)] for i in range(3)]
        actual_intrinsic = matrix3(field(component, ("geometry_intrinsic_tensor_world_kg_m2",), "GO_INTRINSIC:" + component_id), "GO_INTRINSIC:" + component_id)
        require(frobenius(matrix_difference(expected_intrinsic, actual_intrinsic)) <= MATRIX_COMPARE_TOL, "GO_INTRINSIC_RENORMALIZATION:" + component_id)
    components = records(field(report, ("components",), "OVERLAP_COMPONENTS"), ("component_id",), "OVERLAP_COMPONENTS")
    for component in components:
        # Corrected raw overlap is evidence of work done, not a remaining blocker.
        remaining = number(field(component, ("remaining_or_estimated_uncertainty_fraction",), "COMPONENT_REMAINING_OVERLAP"), "COMPONENT_REMAINING_OVERLAP")
        maximum_remaining = max(maximum_remaining, remaining)
    validation = field(report, ("validation",), "VALIDATION")
    require(isinstance(validation, dict), "VALIDATION_TYPE")
    global_remaining = number(field(validation, ("max_remaining_or_estimated_uncertainty_fraction", "max_remaining_component_overlap_fraction"), "MAX_REMAINING_OVERLAP"), "MAX_REMAINING_OVERLAP")
    require(global_remaining >= maximum_remaining - 1.0e-15, "MAX_REMAINING_OVERLAP_UNDERREPORT")
    return global_remaining


def validate_print_reuse(report: Mapping[str, Any], protected_v2: Mapping[str, Any]) -> None:
    suitability = load_json("V15_16_打印件惯量几何适用性报告.json")
    require(suitability.get("all_four_pass") is True, "PRINT_SUITABILITY_AUTHORITY")
    artifacts = records(field(suitability, ("artifacts",), "PRINT_ARTIFACTS"), ("member",), "PRINT_ARTIFACTS")
    require(len(artifacts) == 4 and all(item.get("pass") is True for item in artifacts), "PRINT_ARTIFACT_PASS")
    component_map = indexed(records(field(report, ("components",), "PRINT_COMPONENTS"), ("component_id",), "PRINT_COMPONENTS"), ("component_id",), "PRINT_COMPONENTS")
    v2_components = protected_v2_component_map(protected_v2)
    for component_id in PRINT_COMPONENTS:
        method = str(field(component_map[component_id], ("method",), "PRINT_METHOD:" + component_id))
        require(method == "REUSED_PROTECTED_CANONICAL_PRINT_PATH_A_MASS_PROPERTIES_NO_RECOMPUTE", "PRINT_METHOD:" + component_id)
        v2_path = field(v2_components[component_id], ("path_a_occt",), "PRINT_V2_PATH:" + component_id)
        reported_volume = number(field(component_map[component_id], ("raw_volume_mm3",), "PRINT_VOLUME:" + component_id), "PRINT_VOLUME:" + component_id)
        protected_volume = number(field(v2_path, ("volume_mm3",), "PRINT_V2_VOLUME:" + component_id), "PRINT_V2_VOLUME:" + component_id)
        close(reported_volume, protected_volume, max(1.0e-9, protected_volume * 1.0e-12), "PRINT_PROTECTED_VOLUME:" + component_id)
        require_matrix_close(
            field(component_map[component_id], ("geometry_intrinsic_tensor_world_kg_m2",), "PRINT_TENSOR:" + component_id),
            field(v2_path, ("inertia_about_component_frozen_com_world_kg_m2",), "PRINT_V2_TENSOR:" + component_id),
            "PRINT_PROTECTED_TENSOR:" + component_id,
            relative=1.0e-12,
            absolute=1.0e-15,
        )


def validate_engineering_fallbacks(
    report_components: Mapping[str, Mapping[str, Any]],
    authority_components_map: Mapping[str, Mapping[str, Any]],
    protected_v2: Mapping[str, Any], protected_a1: Mapping[str, Any],
) -> None:
    v2_components = protected_v2_component_map(protected_v2)
    protected_events = overlap_event_map(protected_a1)
    for component_id, component in report_components.items():
        if component.get("engineering_estimate_used") is not True:
            continue
        method = str(field(component, ("correction_method",), "FALLBACK_METHOD:" + component_id))
        if not method.startswith("ENGINEERING_OCCUPIED_VOLUME_ESTIMATE_"):
            continue
        require("EMPIRICAL" in method and "ISOTROPIC" in method, "FALLBACK_EMPIRICAL_LABEL:" + component_id)
        limitations = field(component, ("model_limitations",), "FALLBACK_LIMITATIONS:" + component_id)
        require(isinstance(limitations, list) and any("EMPIRICAL_NONCONSERVATIVE" in str(item) for item in limitations), "FALLBACK_NONCONSERVATIVE_LIMITATION:" + component_id)
        v2_path = field(v2_components[component_id], ("path_a_occt",), "FALLBACK_V2_PATH:" + component_id)
        raw_volume = number(field(v2_path, ("volume_mm3",), "FALLBACK_V2_VOLUME:" + component_id), "FALLBACK_V2_VOLUME:" + component_id)
        removable = [event for event in protected_events.get(component_id, []) if event.get("classification") in {"FULL_CONTAINMENT", "PARTIAL_INTERPENETRATION"}]
        removed_upper = min(raw_volume * 0.999999, math.fsum(number(field(event, ("common_volume_mm3",), "FALLBACK_EVENT_VOLUME"), "FALLBACK_EVENT_VOLUME") for event in removable))
        occupied = raw_volume - removed_upper
        close(number(field(component, ("raw_volume_mm3",), "FALLBACK_RAW:" + component_id), "FALLBACK_RAW:" + component_id), raw_volume, max(1.0e-8, raw_volume * 1.0e-12), "FALLBACK_RAW:" + component_id)
        close(number(field(component, ("occupied_volume_mm3",), "FALLBACK_OCCUPIED:" + component_id), "FALLBACK_OCCUPIED:" + component_id), occupied, max(1.0e-8, occupied * 1.0e-12), "FALLBACK_OCCUPIED:" + component_id)
        scale = (occupied / raw_volume) ** (2.0 / 3.0)
        protected_tensor = matrix3(field(v2_path, ("inertia_about_component_frozen_com_world_kg_m2",), "FALLBACK_V2_TENSOR:" + component_id), "FALLBACK_V2_TENSOR:" + component_id)
        expected_tensor = matrix_scale(protected_tensor, scale)
        require_matrix_close(field(component, ("geometry_intrinsic_tensor_world_kg_m2",), "FALLBACK_TENSOR:" + component_id), expected_tensor, "FALLBACK_EMPIRICAL_TENSOR:" + component_id, relative=1.0e-11, absolute=1.0e-14)
        frozen_mass = float(authority_components_map[component_id]["mass_kg"])
        close(number(field(component, ("effective_density_kg_per_mm3",), "FALLBACK_DENSITY:" + component_id), "FALLBACK_DENSITY:" + component_id), frozen_mass / occupied, max(1.0e-18, frozen_mass / occupied * 1.0e-10), "FALLBACK_DENSITY:" + component_id)


def validate_matching_graph_primitives(
    report_components: Mapping[str, Mapping[str, Any]],
    authority_components_map: Mapping[str, Mapping[str, Any]],
    protected_v2: Mapping[str, Any], protected_a1: Mapping[str, Any],
) -> None:
    protected_events = overlap_event_map(protected_a1)
    v2_components = protected_v2_component_map(protected_v2)
    exact_method = "NORMAL_OCCT_PAIRWISE_INCLUSION_EXCLUSION_MATCHING_GRAPH_NO_TRIPLE_TERM"
    for component_id in RESIDUALS:
        component = report_components[component_id]
        if component.get("correction_method") != exact_method:
            require(component.get("engineering_estimate_used") is True, "RESIDUAL_NONEXACT_WITHOUT_FALLBACK:" + component_id)
            continue
        evidence = field(component, ("matching_graph_inclusion_exclusion_evidence",), "MATCHING_EVIDENCE:" + component_id)
        require(isinstance(evidence, dict), "MATCHING_EVIDENCE_TYPE:" + component_id)
        require(evidence.get("matching_graph_max_degree") == 1, "MATCHING_GRAPH_DEGREE:" + component_id)
        require(evidence.get("matching_graph_no_triple_term") is True, "MATCHING_GRAPH_TRIPLE:" + component_id)
        events = [event for event in protected_events.get(component_id, []) if event.get("classification") == "PARTIAL_INTERPENETRATION"]
        require(events and len(events) == len(protected_events.get(component_id, [])), "MATCHING_PROTECTED_EVENT_CLASSES:" + component_id)
        protected_by_pair: dict[frozenset[str], Mapping[str, Any]] = {}
        degree: dict[str, int] = {}
        for event in events:
            left_record = field(event, ("left",), "MATCHING_EVENT_LEFT")
            right_record = field(event, ("right",), "MATCHING_EVENT_RIGHT")
            require(isinstance(left_record, dict) and isinstance(right_record, dict), "MATCHING_EVENT_SIDES")
            left = str(field(left_record, ("atom_id",), "MATCHING_EVENT_LEFT_ID"))
            right = str(field(right_record, ("atom_id",), "MATCHING_EVENT_RIGHT_ID"))
            pair = frozenset((left, right))
            require(len(pair) == 2 and pair not in protected_by_pair, "MATCHING_EVENT_PAIR:" + component_id)
            protected_by_pair[pair] = event
            degree[left] = degree.get(left, 0) + 1
            degree[right] = degree.get(right, 0) + 1
        require(max(degree.values()) == 1, "MATCHING_PROTECTED_DEGREE:" + component_id)

        raw_volume = number(field(evidence, ("raw_atom_sum_volume_mm3",), "MATCHING_RAW_VOLUME:" + component_id), "MATCHING_RAW_VOLUME:" + component_id)
        v2_path = field(v2_components[component_id], ("path_a_occt",), "MATCHING_V2_PATH:" + component_id)
        protected_raw_volume = number(field(v2_path, ("volume_mm3",), "MATCHING_V2_VOLUME:" + component_id), "MATCHING_V2_VOLUME:" + component_id)
        close(raw_volume, protected_raw_volume, max(1.0e-7, protected_raw_volume * 1.0e-12), "MATCHING_RAW_PROTECTED:" + component_id)
        raw_first = vector3(field(evidence, ("raw_atom_sum_first_moment_mm4",), "MATCHING_RAW_FIRST:" + component_id), "MATCHING_RAW_FIRST:" + component_id)
        raw_origin = matrix3(field(evidence, ("raw_atom_sum_inertia_about_world_origin_mm5",), "MATCHING_RAW_ORIGIN:" + component_id), "MATCHING_RAW_ORIGIN:" + component_id)
        protected_raw_com = vector3(field(v2_path, ("com_world_mm",), "MATCHING_V2_COM:" + component_id), "MATCHING_V2_COM:" + component_id)
        protected_raw_first = tuple(protected_raw_volume * value for value in protected_raw_com)
        require_vector_close(raw_first, protected_raw_first, "MATCHING_RAW_FIRST_PROTECTED:" + component_id, relative=1.0e-11, absolute=1.0e-5)
        protected_mass = number(field(v2_path, ("mass_kg",), "MATCHING_V2_MASS:" + component_id), "MATCHING_V2_MASS:" + component_id)
        close(protected_mass, float(authority_components_map[component_id]["mass_kg"]), MASS_TOL_KG, "MATCHING_V2_MASS:" + component_id)
        protected_inertia_at_frozen = matrix3(field(v2_path, ("inertia_about_component_frozen_com_world_kg_m2",), "MATCHING_V2_TENSOR:" + component_id), "MATCHING_V2_TENSOR:" + component_id)
        frozen_com_mm = vector3(field(v2_components[component_id], ("frozen_com_world_mm",), "MATCHING_V2_FROZEN_COM:" + component_id), "MATCHING_V2_FROZEN_COM:" + component_id)
        raw_density = protected_mass / protected_raw_volume
        unit_inertia_at_frozen = matrix_scale(protected_inertia_at_frozen, 1.0 / (raw_density * 1.0e-6))
        protected_raw_origin = matrix_add(
            unit_inertia_at_frozen,
            matrix_difference(
                parallel_axis_matrix(protected_raw_volume, protected_raw_com),
                parallel_axis_matrix(protected_raw_volume, tuple(protected_raw_com[i] - frozen_com_mm[i] for i in range(3))),
            ),
        )
        require_matrix_close(raw_origin, protected_raw_origin, "MATCHING_RAW_ORIGIN_PROTECTED:" + component_id, relative=1.0e-10, absolute=1.0e-3)
        common_records = records(field(evidence, ("common_intersections",), "MATCHING_COMMONS:" + component_id), ("left_atom_id",), "MATCHING_COMMONS:" + component_id)
        require(len(common_records) == len(events), "MATCHING_COMMON_COUNT:" + component_id)
        removed_volume = 0.0
        removed_first = [0.0, 0.0, 0.0]
        removed_origin = [[0.0, 0.0, 0.0] for _ in range(3)]
        seen_pairs: set[frozenset[str]] = set()
        for common in common_records:
            left = str(field(common, ("left_atom_id",), "MATCHING_COMMON_LEFT"))
            right = str(field(common, ("right_atom_id",), "MATCHING_COMMON_RIGHT"))
            pair = frozenset((left, right))
            require(pair in protected_by_pair and pair not in seen_pairs, "MATCHING_COMMON_PAIR:" + component_id)
            seen_pairs.add(pair)
            volume = number(field(common, ("volume_mm3",), "MATCHING_COMMON_VOLUME"), "MATCHING_COMMON_VOLUME")
            protected_volume = number(field(protected_by_pair[pair], ("common_volume_mm3",), "MATCHING_PROTECTED_VOLUME"), "MATCHING_PROTECTED_VOLUME")
            close(volume, protected_volume, max(1.0e-6, protected_volume * 1.0e-6), "MATCHING_COMMON_PROTECTED_VOLUME:" + component_id)
            com = vector3(field(common, ("com_world_mm",), "MATCHING_COMMON_COM"), "MATCHING_COMMON_COM")
            first = tuple(volume * value for value in com)
            require_vector_close(field(common, ("first_moment_mm4",), "MATCHING_COMMON_FIRST"), first, "MATCHING_COMMON_FIRST", relative=1.0e-12, absolute=1.0e-7)
            central = matrix3(field(common, ("raw_centroidal_matrix_of_inertia_mm5",), "MATCHING_COMMON_CENTRAL"), "MATCHING_COMMON_CENTRAL")
            origin = matrix_add(central, parallel_axis_matrix(volume, com))
            require_matrix_close(field(common, ("inertia_about_world_origin_mm5",), "MATCHING_COMMON_ORIGIN"), origin, "MATCHING_COMMON_ORIGIN", relative=1.0e-11, absolute=1.0e-5)
            require(number(field(common, ("positive_solid_count",), "MATCHING_POSITIVE_SOLIDS"), "MATCHING_POSITIVE_SOLIDS") >= 1.0, "MATCHING_POSITIVE_SOLIDS")
            removed_volume += volume
            removed_first = [removed_first[i] + first[i] for i in range(3)]
            removed_origin = matrix_add(removed_origin, origin)
        require(seen_pairs == set(protected_by_pair), "MATCHING_COMMON_PAIR_COVERAGE:" + component_id)
        occupied_volume = raw_volume - removed_volume
        occupied_first = tuple(raw_first[i] - removed_first[i] for i in range(3))
        occupied_com = tuple(value / occupied_volume for value in occupied_first)
        occupied_origin = matrix_difference(raw_origin, removed_origin)
        occupied_central = matrix_difference(occupied_origin, parallel_axis_matrix(occupied_volume, occupied_com))
        close(number(field(component, ("raw_volume_mm3",), "MATCHING_COMPONENT_RAW"), "MATCHING_COMPONENT_RAW"), raw_volume, max(1.0e-7, raw_volume * 1.0e-11), "MATCHING_COMPONENT_RAW:" + component_id)
        close(number(field(component, ("occupied_volume_mm3",), "MATCHING_COMPONENT_OCCUPIED"), "MATCHING_COMPONENT_OCCUPIED"), occupied_volume, max(1.0e-7, occupied_volume * 1.0e-11), "MATCHING_COMPONENT_OCCUPIED:" + component_id)
        close(number(field(component, ("removed_volume_mm3",), "MATCHING_COMPONENT_REMOVED"), "MATCHING_COMPONENT_REMOVED"), removed_volume, max(1.0e-7, removed_volume * 1.0e-10), "MATCHING_COMPONENT_REMOVED:" + component_id)
        close(number(field(evidence, ("occupied_volume_mm3",), "MATCHING_OCCUPIED_VOLUME"), "MATCHING_OCCUPIED_VOLUME"), occupied_volume, max(1.0e-7, occupied_volume * 1.0e-11), "MATCHING_OCCUPIED_VOLUME:" + component_id)
        require_vector_close(field(evidence, ("occupied_first_moment_mm4",), "MATCHING_OCCUPIED_FIRST"), occupied_first, "MATCHING_OCCUPIED_FIRST:" + component_id, relative=1.0e-11, absolute=1.0e-6)
        require_vector_close(field(evidence, ("occupied_com_world_mm",), "MATCHING_OCCUPIED_COM"), occupied_com, "MATCHING_OCCUPIED_COM:" + component_id, relative=1.0e-11, absolute=1.0e-8)
        require_matrix_close(field(evidence, ("occupied_inertia_about_world_origin_mm5",), "MATCHING_OCCUPIED_ORIGIN"), occupied_origin, "MATCHING_OCCUPIED_ORIGIN:" + component_id, relative=1.0e-10, absolute=1.0e-4)
        require_matrix_close(field(evidence, ("occupied_centroidal_volume_inertia_matrix_mm5",), "MATCHING_OCCUPIED_CENTRAL"), occupied_central, "MATCHING_OCCUPIED_CENTRAL:" + component_id, relative=1.0e-10, absolute=1.0e-4)
        require_matrix_close(field(component, ("occupied_centroidal_volume_inertia_matrix_mm5",), "MATCHING_COMPONENT_CENTRAL"), occupied_central, "MATCHING_COMPONENT_CENTRAL:" + component_id, relative=1.0e-10, absolute=1.0e-4)
        require_vector_close(field(component, ("geometry_centroid_world_mm",), "MATCHING_COMPONENT_COM"), occupied_com, "MATCHING_COMPONENT_COM:" + component_id, relative=1.0e-11, absolute=1.0e-8)
        mass = float(authority_components_map[component_id]["mass_kg"])
        density = mass / occupied_volume
        close(number(field(component, ("effective_density_kg_per_mm3",), "MATCHING_COMPONENT_DENSITY"), "MATCHING_COMPONENT_DENSITY"), density, max(1.0e-18, density * 1.0e-10), "MATCHING_COMPONENT_DENSITY:" + component_id)
        expected_intrinsic = matrix_scale(occupied_central, density * 1.0e-6)
        require_matrix_close(field(component, ("geometry_intrinsic_tensor_world_kg_m2",), "MATCHING_COMPONENT_INTRINSIC"), expected_intrinsic, "MATCHING_COMPONENT_INTRINSIC:" + component_id, relative=1.0e-10, absolute=1.0e-14)


def link_tensor(record: Mapping[str, Any], link: str) -> list[list[float]]:
    for name in ("inertia_tensor_kg_m2", "tensor_kg_m2", "matrix_kg_m2"):
        if name in record:
            return matrix3(record[name], "LINK_TENSOR:" + link)
    candidate = record.get("candidate_inertia")
    if isinstance(candidate, dict) and "matrix_kg_m2" in candidate:
        return matrix3(candidate["matrix_kg_m2"], "LINK_TENSOR:" + link)
    raise ValidationError("LINK_TENSOR_MISSING:" + link)


def tensor_gate_results(
    matrix: Sequence[Sequence[float]], mass_kg: float, r_max_m: float,
) -> dict[str, bool]:
    symmetry_error = max(abs(float(matrix[i][j]) - float(matrix[j][i])) for i in range(3) for j in range(3))
    eigenvalues = symmetric_eigenvalues(matrix)
    radii = [math.sqrt(value / mass_kg) for value in eigenvalues if value > 0.0]
    trace = sum(float(matrix[i][i]) for i in range(3))
    return {
        "finite_pass": all(math.isfinite(float(item)) for row in matrix for item in row),
        "symmetry_pass": symmetry_error <= SYMMETRY_TOL,
        "positive_definite_pass": eigenvalues[0] > 0.0,
        "principal_triangle_inequality_pass": eigenvalues[0] + eigenvalues[1] >= eigenvalues[2] - 1.0e-12,
        "radius_of_gyration_sanity_pass": len(radii) == 3 and all(0.0 < radius <= r_max_m * (1.0 + 1.0e-9) for radius in radii),
        "unit_audit_pass": (
            trace <= 2.0 * mass_kg * r_max_m * r_max_m * (1.0 + 1.0e-8)
            and max(abs(float(item)) for row in matrix for item in row) < 1.0
        ),
    }


def validate_tensors(
    report_links: Mapping[str, Mapping[str, Any]], freeze: Mapping[str, Any] | None,
    report_components: Mapping[str, Mapping[str, Any]],
    authority_components_map: Mapping[str, Mapping[str, Any]],
    link_components: Mapping[str, list[str]],
    authority_component_world_coms: Mapping[str, Sequence[float]],
    authority_link_world_coms: Mapping[str, Sequence[float]],
    link_rotations: Mapping[str, Sequence[Sequence[float]]],
) -> tuple[dict[str, list[list[float]]], bool]:
    freeze_links: dict[str, Mapping[str, Any]] = {}
    if freeze is not None:
        freeze_links = indexed(
            records(field(freeze, ("links",), "FREEZE_LINKS"), ("link",), "FREEZE_LINKS"),
            ("link",), "FREEZE_LINKS",
        )
        require(set(freeze_links) == set(LINK_ORDER), "FREEZE_LINK_SET")
    result: dict[str, list[list[float]]] = {}
    all_physics_pass = True
    for link in LINK_ORDER:
        record = report_links[link]
        matrix = link_tensor(record, link)
        mass = number(field(record, ("mass_kg",), "TENSOR_MASS:" + link), "TENSOR_MASS:" + link)
        r_max = number(field(record, ("r_max_m", "maximum_radius_m", "geometry_radius_bound_m"), "TENSOR_RMAX:" + link), "TENSOR_RMAX:" + link)
        require(r_max > 0.0, "TENSOR_RMAX:" + link)
        gates = tensor_gate_results(matrix, mass, r_max)
        serialized_checks = field(record, ("tensor_checks",), "TENSOR_CHECKS:" + link)
        require(isinstance(serialized_checks, dict), "TENSOR_CHECKS_TYPE:" + link)
        for key, value in gates.items():
            require(serialized_checks.get(key) is value, "TENSOR_CHECK_FLAG:" + link + ":" + key)
        all_physics_pass = all_physics_pass and all(gates.values())
        world_sum = [[0.0, 0.0, 0.0] for _ in range(3)]
        for component_id in link_components[link]:
            component_record = report_components[component_id]
            intrinsic = matrix3(
                field(component_record, ("geometry_intrinsic_tensor_world_kg_m2",), "COMPONENT_INTRINSIC:" + component_id),
                "COMPONENT_INTRINSIC:" + component_id,
            )
            intrinsic_symmetry = max(abs(intrinsic[i][j] - intrinsic[j][i]) for i in range(3) for j in range(3))
            require(intrinsic_symmetry <= SYMMETRY_TOL, "COMPONENT_INTRINSIC_SYMMETRY:" + component_id)
            require(symmetric_eigenvalues(intrinsic)[0] >= -1.0e-14, "COMPONENT_INTRINSIC_PSD:" + component_id)
            component_com = authority_component_world_coms[component_id]
            link_com = authority_link_world_coms[link]
            displacement = [float(component_com[i]) - float(link_com[i]) for i in range(3)]
            contribution = matrix_add(
                intrinsic,
                parallel_axis_matrix(float(authority_components_map[component_id]["mass_kg"]), displacement),
            )
            world_sum = matrix_add(world_sum, contribution)
        rotation = link_rotations[link]
        recomputed_link = matrix_multiply(matrix_multiply(transpose(rotation), world_sum), rotation)
        require(
            frobenius(matrix_difference(recomputed_link, matrix)) <= MATRIX_COMPARE_TOL,
            "LINK_TENSOR_INDEPENDENT_REASSEMBLY:" + link,
        )
        result[link] = matrix
        if freeze is not None:
            frozen_record = freeze_links[link]
            close(number(field(frozen_record, ("mass_kg",), "FREEZE_MASS:" + link), "FREEZE_MASS:" + link), mass, MASS_TOL_KG, "FREEZE_MASS:" + link)
            frozen_com = vector3(field(frozen_record, ("com_xyz_m_in_link_frame", "com_owner_link_m", "com_link_m", "frozen_com_xyz_m_in_link_frame"), "FREEZE_COM:" + link), "FREEZE_COM:" + link)
            require(vector_distance(frozen_com, EXPECTED_LINK_COM_M[link]) <= COM_FREEZE_LIMIT_M, "FREEZE_COM:" + link)
            frozen_matrix = link_tensor(frozen_record, link)
            require(frobenius(matrix_difference(frozen_matrix, matrix)) <= MATRIX_COMPARE_TOL, "FREEZE_TENSOR_MISMATCH:" + link)
            require(str(frozen_record.get("confidence_class", "")), "FREEZE_CONFIDENCE_MISSING:" + link)
            limitations = frozen_record.get("model_limitations")
            require(isinstance(limitations, list), "FREEZE_LIMITATIONS_MISSING:" + link)
    return result, all_physics_pass


def residual_classification(percent: float) -> str:
    if 0.0 <= percent <= 5.0:
        return "PASS_HIGH_CONFIDENCE"
    if percent <= 10.0:
        return "PASS_WITH_MODEL_LIMITATION"
    if percent <= 15.0:
        return "REVIEW_REQUIRED"
    return "FAIL"


def validate_residual_sensitivity(
    report: Mapping[str, Any],
    report_links: Mapping[str, Mapping[str, Any]],
    report_components: Mapping[str, Mapping[str, Any]],
    authority_components_map: Mapping[str, Mapping[str, Any]],
    authority_component_world_coms: Mapping[str, Sequence[float]],
    authority_link_world_coms: Mapping[str, Sequence[float]],
    link_rotations: Mapping[str, Sequence[Sequence[float]]],
) -> dict[str, float]:
    raw = field(report, ("residual_sensitivity", "residual_sensitivity_audit"), "RESIDUAL_SECTION")
    items = records(raw, ("component_id",), "RESIDUAL_SECTION")
    item_map = indexed(items, ("component_id",), "RESIDUAL_SECTION")
    require(set(item_map) == set(RESIDUALS), "RESIDUAL_COMPONENT_SET")
    result: dict[str, float] = {}
    for component_id, (owner, expected_mass, short_id) in RESIDUALS.items():
        record = item_map[component_id]
        require(str(field(record, ("owner_link",), "RESIDUAL_OWNER:" + component_id)) == owner, "RESIDUAL_OWNER:" + component_id)
        close(number(field(record, ("mass_kg",), "RESIDUAL_MASS:" + component_id), "RESIDUAL_MASS:" + component_id), expected_mass, MASS_TOL_KG, "RESIDUAL_MASS:" + component_id)
        close(float(authority_components_map[component_id]["mass_kg"]), expected_mass, MASS_TOL_KG, "RESIDUAL_AUTHORITY_MASS:" + component_id)
        uniform = matrix3(field(record, ("candidate_final_link_uniform_proxy_tensor_kg_m2", "uniform_final_link_tensor_kg_m2"), "RESIDUAL_UNIFORM:" + component_id), "RESIDUAL_UNIFORM:" + component_id)
        point = matrix3(field(record, ("candidate_final_link_point_model_tensor_kg_m2", "point_final_link_tensor_kg_m2"), "RESIDUAL_POINT:" + component_id), "RESIDUAL_POINT:" + component_id)
        uniform_contribution = matrix3(field(record, ("uniform_proxy_component_contribution_about_link_frozen_com_kg_m2",), "RESIDUAL_UNIFORM_CONTRIBUTION:" + component_id), "RESIDUAL_UNIFORM_CONTRIBUTION:" + component_id)
        point_contribution = matrix3(field(record, ("point_mass_component_contribution_about_link_frozen_com_kg_m2",), "RESIDUAL_POINT_CONTRIBUTION:" + component_id), "RESIDUAL_POINT_CONTRIBUTION:" + component_id)
        final_link_tensor = link_tensor(report_links[owner], owner)
        require_matrix_close(uniform, final_link_tensor, "RESIDUAL_UNIFORM_EQUALS_FINAL_LINK:" + component_id, relative=1.0e-13, absolute=1.0e-15)
        displacement = [
            float(authority_component_world_coms[component_id][axis]) - float(authority_link_world_coms[owner][axis])
            for axis in range(3)
        ]
        expected_point_world = parallel_axis_matrix(expected_mass, displacement)
        rotation = link_rotations[owner]
        intrinsic_world = matrix3(
            field(
                report_components[component_id],
                ("geometry_intrinsic_tensor_world_kg_m2",),
                "RESIDUAL_COMPONENT_INTRINSIC:" + component_id,
            ),
            "RESIDUAL_COMPONENT_INTRINSIC:" + component_id,
        )
        expected_uniform_world = matrix_add(intrinsic_world, expected_point_world)
        expected_uniform_link = matrix_multiply(
            matrix_multiply(transpose(rotation), expected_uniform_world), rotation,
        )
        expected_point_link = matrix_multiply(matrix_multiply(transpose(rotation), expected_point_world), rotation)
        require_matrix_close(
            uniform_contribution, expected_uniform_link,
            "RESIDUAL_UNIFORM_CONTRIBUTION_RECOMPUTE:" + component_id,
            relative=1.0e-12, absolute=1.0e-15,
        )
        require(frobenius(matrix_difference(expected_point_link, point_contribution)) <= MATRIX_COMPARE_TOL, "RESIDUAL_POINT_MODEL_RECOMPUTE:" + component_id)
        expected_point_final = matrix_add(
            matrix_difference(final_link_tensor, expected_uniform_link),
            expected_point_link,
        )
        require_matrix_close(
            point, expected_point_final,
            "RESIDUAL_POINT_FINAL_RECOMPUTE:" + component_id,
            relative=1.0e-12, absolute=1.0e-15,
        )
        denominator = frobenius(uniform)
        require(denominator > 0.0, "RESIDUAL_DENOMINATOR:" + component_id)
        relative = frobenius(matrix_difference(uniform, point)) / denominator
        percent = 100.0 * relative
        reported = number(field(record, ("sensitivity_percent",), "RESIDUAL_PERCENT:" + component_id), "RESIDUAL_PERCENT:" + component_id)
        close(reported, percent, max(1.0e-10, percent * 1.0e-9), "RESIDUAL_RECOMPUTE:" + component_id)
        classification = residual_classification(percent)
        require(str(field(record, ("status", "classification"), "RESIDUAL_STATUS:" + component_id)) == classification, "RESIDUAL_STATUS:" + component_id)
        require(record.get("freeze_gate_pass") is (percent < 10.0), "RESIDUAL_FREEZE_GATE_FLAG:" + component_id)
        if classification == "PASS_WITH_MODEL_LIMITATION":
            limitations = record.get("model_limitations")
            require(isinstance(limitations, list) and limitations == ["RESIDUAL_SPATIAL_INERTIA_MODEL_UNCERTAINTY"], "RESIDUAL_LIMITATION_MISSING:" + component_id)
        result[short_id] = percent
    return result


def validate_runtime_and_prohibitions(report: Mapping[str, Any]) -> None:
    runtime = field(report, ("runtime_budget_policy", "runtime_policy", "bounded_runtime_audit"), "RUNTIME_POLICY")
    require(isinstance(runtime, dict), "RUNTIME_POLICY_TYPE")
    require(runtime.get("mesh_used") is False, "MESH_USED")
    require(runtime.get("full_vertex_classification_used") is False, "ALL_VERTEX_CLASSIFICATION_USED")
    require(runtime.get("brep_distance_scan_used") is False, "BREP_DISTANCE_SCAN_USED")
    require(runtime.get("mesh_convergence_used") is False, "MESH_CONVERGENCE_USED")
    require(runtime.get("forbidden_linear_deflections_mm_used") == [], "FORBIDDEN_DEFLECTION_USED")
    require(runtime.get("source_geometry_repair_used") is False, "SOURCE_REPAIR_USED")
    worker_timeout = number(field(runtime, ("component_worker_hard_timeout_seconds",), "WORKER_TIMEOUT"), "WORKER_TIMEOUT")
    audit_limit = number(field(runtime, ("component_geometry_audit_hard_limit_seconds",), "GEOMETRY_AUDIT_LIMIT"), "GEOMETRY_AUDIT_LIMIT")
    require(worker_timeout <= 60.0 and audit_limit <= 300.0, "GEOMETRY_TIMEOUT_POLICY")
    require(all(value is False for value in recursively_values(report, "collision_proxy_used")), "COLLISION_PROXY_USED")
    prohibited = field(report, ("prohibited_actions", "prohibited_outputs"), "PROHIBITED_OUTPUTS")
    require(isinstance(prohibited, dict), "PROHIBITED_OUTPUTS_TYPE")
    for key in (
        "collision_proxy_used", "mass_or_com_v2_modified", "source_cad_modified",
        "urdf_xacro_mjcf_or_ros2_control_modified", "gravity_enabled",
        "armature_added", "friction_or_damping_added",
    ):
        require(prohibited.get(key) is False, "PROHIBITED_FLAG:" + key)


def validate_unit_audit(report: Mapping[str, Any]) -> None:
    unit = field(report, ("unit_audit",), "UNIT_AUDIT")
    require(isinstance(unit, dict), "UNIT_AUDIT_TYPE")
    require(unit.get("mass_unit") == "kg", "UNIT_MASS")
    require(unit.get("cad_length_unit") == "mm", "UNIT_CAD_LENGTH")
    require(unit.get("raw_volume_second_integral_unit") == "mm^5", "UNIT_RAW_SECOND")
    require(unit.get("effective_density_unit") == "kg/mm^3", "UNIT_DENSITY")
    require(unit.get("final_inertia_unit") == "kg*m^2", "UNIT_INERTIA")
    require(unit.get("conversion") == "raw_mm5*(mass_kg/volume_mm3)*1e-6", "UNIT_CONVERSION")
    require(unit.get("pass") is True, "UNIT_AUDIT_PASS")


def validate_status(report: Mapping[str, Any], freeze: Mapping[str, Any] | None, pass_mode: bool) -> None:
    expected = "V15.16 INERTIA_ENGINEERING_V1 = PASS" if pass_mode else "V15.16 INERTIA_ENGINEERING_V1 = FAIL"
    require(str(field(report, ("final_status",), "FINAL_STATUS")) == expected, "FINAL_STATUS")
    require(report.get("freeze_permitted") is pass_mode, "FREEZE_PERMITTED")
    unresolved = field(report, ("unresolved_items", "hard_unresolved_items"), "UNRESOLVED_ITEMS")
    require(isinstance(unresolved, list), "UNRESOLVED_ITEMS_TYPE")
    if pass_mode:
        require(unresolved == [], "PASS_UNRESOLVED_NOT_EMPTY")
        require(freeze is not None, "PASS_FREEZE_MISSING")
        require(freeze.get("schema") == FREEZE_SCHEMA, "FREEZE_SCHEMA")
        require(freeze.get("final_status") == expected, "FREEZE_FINAL_STATUS")
        require(freeze.get("status") == "ENGINEERING_V1", "FREEZE_ENGINEERING_LABEL")


def static_self_audit() -> None:
    source = (ROOT / VALIDATOR).read_text(encoding="utf-8")
    tree = ast.parse(source, filename=VALIDATOR)
    forbidden_modules = {"FreeCAD", "Part", "Mesh", "build_inertia_engineering_v15_16"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                require(alias.name.split(".")[0] not in forbidden_modules, "VALIDATOR_FORBIDDEN_IMPORT:" + alias.name)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            require(module.split(".")[0] not in forbidden_modules, "VALIDATOR_FORBIDDEN_IMPORT:" + module)
    builder_source = (ROOT / BUILDER).read_text(encoding="utf-8")
    require(sha256_file(ROOT / BUILDER) == EXPECTED_BUILDER_SHA256, "BUILDER_FROZEN_SHA256")
    builder_tree = ast.parse(builder_source, filename=BUILDER)
    for node in ast.walk(builder_tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                require(alias.name.split(".")[0] not in {"Mesh", "MeshPart"}, "BUILDER_FORBIDDEN_MESH_IMPORT:" + alias.name)
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            require(module.split(".")[0] not in {"Mesh", "MeshPart"}, "BUILDER_FORBIDDEN_MESH_IMPORT:" + module)
    for token in ("0.0003", "0.0001", "meshFromShape", "tessellate(", ".isInside("):
        require(token not in builder_source, "BUILDER_FORBIDDEN_EXHAUSTIVE_TOKEN:" + token)
    constants: dict[str, float] = {}
    for node in builder_tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            if isinstance(node.value, ast.Constant) and isinstance(node.value.value, (int, float)):
                constants[node.targets[0].id] = float(node.value.value)
    require(constants.get("WORKER_TIMEOUT_SECONDS") == 60.0, "BUILDER_WORKER_TIMEOUT_CONSTANT")
    require(constants.get("COMPONENT_AUDIT_LIMIT_SECONDS") == 300.0, "BUILDER_COMPONENT_AUDIT_CONSTANT")
    run_worker = next((node for node in builder_tree.body if isinstance(node, ast.FunctionDef) and node.name == "run_worker"), None)
    require(run_worker is not None, "BUILDER_RUN_WORKER_MISSING")
    timeout_calls = []
    for node in ast.walk(run_worker):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "run":
            for keyword in node.keywords:
                if keyword.arg == "timeout":
                    timeout_calls.append(keyword.value)
    require(len(timeout_calls) == 1, "BUILDER_WORKER_TIMEOUT_CALL_COUNT")
    require(isinstance(timeout_calls[0], ast.Name) and timeout_calls[0].id == "WORKER_TIMEOUT_SECONDS", "BUILDER_WORKER_TIMEOUT_NOT_ENFORCED")


def validate_freeze_limitations(freeze: Mapping[str, Any] | None, residuals: Mapping[str, float]) -> None:
    if freeze is None:
        return
    freeze_links = indexed(
        records(field(freeze, ("links",), "FREEZE_LIMITATION_LINKS"), ("link",), "FREEZE_LIMITATION_LINKS"),
        ("link",), "FREEZE_LIMITATION_LINKS",
    )
    residual_to_link = {"residual_A": "link5", "residual_B": "link4", "residual_C": "link3"}
    for residual_id, percent in residuals.items():
        limitations = field(freeze_links[residual_to_link[residual_id]], ("model_limitations",), "FREEZE_MODEL_LIMITATIONS:" + residual_id)
        require(isinstance(limitations, list), "FREEZE_MODEL_LIMITATIONS_TYPE:" + residual_id)
        recorded = "RESIDUAL_SPATIAL_INERTIA_MODEL_UNCERTAINTY" in limitations
        require(recorded is (5.0 < percent <= 10.0), "FREEZE_RESIDUAL_LIMITATION:" + residual_id)


def run_builder_check(pass_mode: bool, protected_before: Mapping[str, str]) -> None:
    outputs = PASS_PATHS if pass_mode else FAIL_PATHS
    before = output_snapshot(outputs)
    python_exe = os.environ.get("V15_16_ENGINEERING_PYTHON", sys.executable)
    result = subprocess.run(
        [python_exe, str(ROOT / BUILDER), "--check"], cwd=ROOT,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        check=False, timeout=1800,
        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
    )
    require(result.returncode == 0, "BUILDER_CHECK_FAILED:" + result.stderr[-2000:])
    after = output_snapshot(outputs)
    require(before == after, "BUILDER_CHECK_MUTATED_OUTPUTS")
    protected_after = protected_snapshot()
    require(dict(protected_before) == protected_after, "BUILDER_CHECK_MUTATED_PROTECTED")


def main() -> int:
    try:
        actual_changed = changed_paths()
        if actual_changed == PASS_PATHS:
            inferred_pass_mode = True
            expected_outputs = PASS_PATHS
        elif actual_changed == FAIL_PATHS:
            inferred_pass_mode = False
            expected_outputs = FAIL_PATHS
        else:
            raise ValidationError("INITIAL_EXACT_CHANGED_PATH_SET:" + repr(sorted(actual_changed)))
        initial_output_hashes = output_snapshot(expected_outputs)
        require(set(initial_output_hashes) == expected_outputs, "INITIAL_OUTPUT_SET_NOT_EXACT")
        static_self_audit()
        protected_before = protected_snapshot()
        report = load_json(ACCEPTANCE_JSON)
        final_status = str(field(report, ("final_status",), "FINAL_STATUS"))
        require(final_status in {
            "V15.16 INERTIA_ENGINEERING_V1 = PASS",
            "V15.16 INERTIA_ENGINEERING_V1 = FAIL",
        }, "FINAL_STATUS_VALUE")
        pass_mode = final_status.endswith("= PASS")
        require(pass_mode is inferred_pass_mode, "FINAL_STATUS_VS_INITIAL_EXACT_SET")
        validate_git_scope(pass_mode)
        freeze = load_json(FREEZE_JSON) if pass_mode else None
        validate_serialized_provenance(report, pass_mode)

        mass = load_json(MASS_LEDGER)
        com = load_json(COM_LEDGER)
        protected_v2 = load_json("V15_16_刚体惯量_FAIL审计_v2.json")
        protected_a1 = load_json("V15_16_质量几何去重审计_v1.json")
        authority_component_map, link_components = authority_components(mass)
        authority_world_coms, authority_component_world_coms, link_rotations = validate_com_authority(com, link_components)
        report_links, report_components, max_com_error = validate_mass_com_reproduction(
            report, authority_component_map, link_components, authority_world_coms,
            authority_component_world_coms,
        )
        slivers_pass = validate_slivers(
            report, authority_component_map, protected_v2, protected_a1,
        )
        max_remaining_overlap = validate_go_containment(
            report, authority_component_map, protected_v2, protected_a1,
        )
        validate_print_reuse(report, protected_v2)
        validate_matching_graph_primitives(
            report_components, authority_component_map, protected_v2, protected_a1,
        )
        validate_engineering_fallbacks(
            report_components, authority_component_map, protected_v2, protected_a1,
        )
        _, tensor_physics_pass = validate_tensors(
            report_links, freeze, report_components, authority_component_map,
            link_components, authority_component_world_coms,
            authority_world_coms, link_rotations,
        )
        residuals = validate_residual_sensitivity(
            report, report_links, report_components, authority_component_map,
            authority_component_world_coms, authority_world_coms, link_rotations,
        )
        validate_freeze_limitations(freeze, residuals)
        validate_runtime_and_prohibitions(report)
        validate_unit_audit(report)
        validate_status(report, freeze, pass_mode)

        computed_freeze_pass = (
            slivers_pass
            and max_remaining_overlap <= OVERLAP_LIMIT
            and max_com_error <= COM_FREEZE_LIMIT_M
            and tensor_physics_pass
            and all(value < 10.0 for value in residuals.values())
            and field(report, ("unresolved_items",), "UNRESOLVED_ITEMS") == []
        )
        require(computed_freeze_pass is pass_mode, "FINAL_STATUS_NOT_DERIVED_FROM_ENGINEERING_GATES")

        validation = field(report, ("validation",), "VALIDATION")
        require(isinstance(validation, dict), "VALIDATION_TYPE")
        close(number(field(validation, ("total_mass_kg",), "VALIDATION_TOTAL_MASS"), "VALIDATION_TOTAL_MASS"), EXPECTED_TOTAL_MASS_KG, MASS_TOL_KG, "VALIDATION_TOTAL_MASS")
        reported_com_error = number(field(validation, ("max_com_v2_reproduction_error_m",), "VALIDATION_COM_ERROR"), "VALIDATION_COM_ERROR")
        close(reported_com_error, max_com_error, max(1.0e-13, max_com_error * 1.0e-8), "VALIDATION_COM_RECOMPUTE")
        require(validation.get("com_v2_reproduction_pass") is (reported_com_error <= COM_FREEZE_LIMIT_M), "VALIDATION_COM_GATE_FLAG")
        require(validation.get("remaining_component_overlap_pass") is (max_remaining_overlap <= OVERLAP_LIMIT), "VALIDATION_OVERLAP_GATE_FLAG")
        require(validation.get("sliver_acceptance_pass") is slivers_pass, "VALIDATION_SLIVER_GATE_FLAG")
        require(validation.get("tensor_finite_pass") is tensor_physics_pass or not pass_mode, "VALIDATION_TENSOR_GATE_FLAG")
        require(validation.get("residual_a_b_c_lt_10_percent_pass") is all(value < 10.0 for value in residuals.values()), "VALIDATION_RESIDUAL_GATE_FLAG")
        require(validation.get("all_freeze_gates_pass") is computed_freeze_pass, "VALIDATION_ALL_GATES_FLAG")
        require(validation.get("pass") is computed_freeze_pass, "VALIDATION_PASS_FLAG")

        run_builder_check(pass_mode, protected_before)
        require(output_snapshot(expected_outputs) == initial_output_hashes, "FINAL_OUTPUT_HASH_TOCTOU")
        require(protected_snapshot() == protected_before, "FINAL_PROTECTED_SNAPSHOT")
        validate_git_scope(pass_mode)
        print(json.dumps({
            "status": "PASS",
            "validated_final_status": final_status,
            "max_com_v2_reproduction_error_m": max_com_error,
            "residual_sensitivity_percent": residuals,
            "builder_check_toctou": "PASS",
            "freecad_geometry_recomputed_by_validator": False,
        }, ensure_ascii=False, sort_keys=True))
        return 0
    except (ValidationError, json.JSONDecodeError, OSError, subprocess.SubprocessError) as exc:
        print("VALIDATION_FAIL:" + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
