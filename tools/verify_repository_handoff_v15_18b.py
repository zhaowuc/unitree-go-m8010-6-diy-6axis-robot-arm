#!/usr/bin/env python3
from __future__ import annotations

"""Fail-closed repository portability verifier for the V15.18B handoff.

The default mode accepts only a clean, ordinary-Git final candidate.
``--allow-dirty`` includes non-ignored untracked paths for pre-commit
diagnostics, but never emits the authoritative ``REPOSITORY_HANDOFF=PASS``
result.
"""

import argparse
from dataclasses import dataclass
import gc
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import subprocess
from typing import Any, Callable, Mapping, Sequence
import unicodedata
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
TARGET_BRANCH = "agent/v15-18b-final-simulation-handoff"
TARGET_TAG = "v15.18b-final-simulation-handoff"
SOURCE_COMMIT = "6dc27f6f8196c691f9f6b1c7684202dec6af2b6a"
EXPECTED_REMOTE_URL = "https://github.com/zhaowuc/go-m8010-robot-arm.git"
MAX_TRACKED_BYTES = 100_000_000

V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
ROS_WS_REL = V14_REL + "/ros2_ws"
ROS_SRC_REL = ROS_WS_REL + "/src"
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
BRIDGE_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"

EXPECTED_AUTHORITIES = {
    "V15_15_实测质量账本_v1.json": "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_COM账本_v2.json": "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    "V15_16_刚体惯量_Engineering_V1.json": "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401",
    "V15_17_Inertial参数部署验收.json": "07b0adc158423671a0de97b85796421424d5bae1603c1ace629f821a0938604e",
    "V15_17_ProductionHash迁移与轨迹闭环验收.json": "e280996b9e894492dac8e2e2fc59ee86e49af34f29adab6b5cd989edc29cd455",
    "V15_18A_静态重力与重力矩验收.json": "640e9104cabd0e2548e07bdd54d5cdb2c66dbdf86a7981eff4d8c12e55b9dfe2",
    MJCF_REL: "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9",
    BRIDGE_REL: "d3273ae0ecfe3ee2bf427a5732f4106206593a2c23fd0bc39aea1f3381f7a5a0",
}

EXPECTED_V15_18B_ARTIFACTS = {
    "tools/audit_passive_gravity_v15_18b.py": "bbba568b8b52694b8f995a63d6b0baa68840f343ccad59d0c016b6fa544a03f9",
    "tools/validate_passive_gravity_v15_18b.py": "95344b1c11bf7ecfe138434de93cab8fd327b969efa8289616fc5bc5ebb8c658",
    "V15_18B_短时被动重力动力学验收.json": "4cde5893981e8388d76997bef3f6cd05abf427b814bb16fe2de6cab42569d730",
    "V15_18B_短时被动重力动力学验收.md": "d9e759068598b252743d37c8d7c10d732b1516d7a098b3cc3455fc95abeff5f7",
}

ROOT_CAUSE_CONDITION_KEYS = [
    "v15_18a_static_gravity_pass",
    "initial_qacc_identity_pass",
    "early_motion_direction_pass",
    "implicitfast_energy_convergence_pass",
    "implicitfast_trajectory_convergence_pass",
    "rk4_energy_reference_pass",
    "rk4_timestep_convergence_pass",
    "no_contacts_all_runs_pass",
    "no_active_joint_limits_all_runs_pass",
    "all_values_finite_all_runs_pass",
    "determinism_pass",
]

PRODUCTION_INTEGRATOR_LIMITATION = (
    "PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_"
    "ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION"
)

EXPECTED_ROS_PACKAGES = {
    "go_m8010_arm_description",
    "go_m8010_arm_mujoco_bridge",
    "go_m8010_arm_v15_14_description",
    "go_m8010_arm_v15_14_moveit_config",
    "go_m8010_arm_v15_14_qa",
}

REQUIRED_TRACKED = {
    ".gitattributes",
    ".gitignore",
    "README.md",
    "docs/UBUNTU22_04_HANDOFF_V15_18B.md",
    "requirements-sim-v15_18b.txt",
    "V15_18B_短时被动重力动力学验收.json",
    "V15_18B_短时被动重力动力学验收.md",
    "V15_18B_GitHub与Ubuntu交付验收.json",
    "V15_18B_GitHub与Ubuntu交付验收.md",
    "tools/build_mass_mapping_v15_15.py",
    "tools/validate_mass_ledger_v15_15.py",
    "tools/build_com_ledger_v15_15_v2.py",
    "tools/validate_com_ledger_v15_15_v2.py",
    "tools/build_mass_geometry_authority_v15_16.py",
    "tools/validate_mass_geometry_authority_v15_16.py",
    "tools/build_inertia_engineering_v15_16.py",
    "tools/validate_inertia_engineering_v15_16.py",
    "tools/apply_inertial_parameters_v15_17.py",
    "tools/validate_inertial_deployment_v15_17.py",
    "tools/validate_v15_17_production_hash_migration.py",
    "tools/audit_static_gravity_v15_18.py",
    "tools/validate_static_gravity_v15_18.py",
    "tools/audit_passive_gravity_v15_18b.py",
    "tools/validate_passive_gravity_v15_18b.py",
    "tools/bootstrap_ubuntu22_04_v15_18b.sh",
    "tools/verify_repository_handoff_v15_18b.py",
}

RUNTIME_SUFFIXES = (".py", ".sh", ".yaml", ".yml", ".xml", ".xacro")
RUNTIME_BASENAMES = {"CMakeLists.txt", "package.xml"}
LFS_POINTER_MARKER = b"version https://git-lfs.github.com/spec/v1"


class HandoffError(RuntimeError):
    """A missing, stale, non-portable, or contradictory handoff artifact."""


@dataclass(frozen=True)
class GitEntry:
    mode: str
    object_id: str
    stage: int
    path: str


def require(condition: bool, message: str) -> None:
    if not condition:
        raise HandoffError(message)


def clean_error(value: object) -> str:
    return " ".join(str(value).strip().split())


def run_process(
    command: Sequence[str],
    *,
    binary: bool = False,
    input_bytes: bytes | None = None,
    timeout: float = 120.0,
) -> subprocess.CompletedProcess[Any]:
    try:
        return subprocess.run(
            list(command),
            cwd=str(ROOT),
            input=input_bytes,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=not binary,
            encoding=None if binary else "utf-8",
            errors=None if binary else "replace",
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise HandoffError(
            f"process infrastructure failure for {command[0]}: {clean_error(error)}"
        ) from error


def git_text(arguments: Sequence[str], *, timeout: float = 120.0) -> str:
    result = run_process(["git", "-c", "core.quotepath=false", *arguments], timeout=timeout)
    require(
        result.returncode == 0,
        "git command failed: " + clean_error(result.stderr or result.stdout),
    )
    return str(result.stdout)


def git_bytes(arguments: Sequence[str], *, timeout: float = 120.0) -> bytes:
    result = run_process(["git", *arguments], binary=True, timeout=timeout)
    require(
        result.returncode == 0,
        "git command failed: "
        + clean_error(bytes(result.stderr or result.stdout).decode("utf-8", "replace")),
    )
    return bytes(result.stdout)


def repo_path(relative: str) -> Path:
    require(relative == relative.replace("\\", "/"), f"non-POSIX repository path: {relative}")
    posix = PurePosixPath(relative)
    require(
        bool(posix.parts)
        and not posix.is_absolute()
        and not PureWindowsPath(relative).is_absolute()
        and ".." not in posix.parts,
        f"unsafe repository path: {relative}",
    )
    return ROOT.joinpath(*posix.parts)


def resolved_repo_relative(path: Path, *, label: str) -> str:
    resolved_root = ROOT.resolve()
    resolved = path.resolve()
    require(
        resolved == resolved_root or resolved_root in resolved.parents,
        f"{label} escaped repository: {path}",
    )
    return resolved.relative_to(resolved_root).as_posix()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise HandoffError(f"invalid JSON {path.name}: {clean_error(error)}") from error
    require(isinstance(value, dict), f"JSON root is not an object: {path.name}")
    return value


def parse_git_index() -> dict[str, GitEntry]:
    raw = git_bytes(["ls-files", "-s", "-z"])
    entries: dict[str, GitEntry] = {}
    for record in raw.split(b"\0"):
        if not record:
            continue
        try:
            metadata, raw_path = record.split(b"\t", 1)
            mode, object_id, stage_text = metadata.decode("ascii").split()
            path = raw_path.decode("utf-8")
            stage = int(stage_text)
        except (ValueError, UnicodeError) as error:
            raise HandoffError("malformed or non-UTF-8 Git index entry") from error
        require(path not in entries, f"duplicate Git index path: {path}")
        entries[path] = GitEntry(mode, object_id, stage, path)
    require(entries, "Git index is empty")
    require(
        all(entry.stage == 0 for entry in entries.values()),
        "Git index contains unmerged stages",
    )
    return entries


def untracked_paths() -> set[str]:
    raw = git_bytes(["ls-files", "--others", "--exclude-standard", "-z"])
    try:
        return {row.decode("utf-8") for row in raw.split(b"\0") if row}
    except UnicodeError as error:
        raise HandoffError("non-UTF-8 untracked repository path") from error


def require_tracked(
    relative: str,
    entries: Mapping[str, GitEntry],
    *,
    allow_dirty: bool,
) -> None:
    path = repo_path(relative)
    require(path.exists() or path.is_symlink(), f"required path is missing: {relative}")
    if relative not in entries:
        require(allow_dirty, f"required path is not tracked: {relative}")


def check_git_identity(
    entries: Mapping[str, GitEntry],
    *,
    expected_commit: str | None,
    allow_dirty: bool,
) -> dict[str, Any]:
    top = Path(git_text(["rev-parse", "--show-toplevel"]).strip()).resolve()
    require(top == ROOT.resolve(), f"repository root mismatch: {top}")
    branch = git_text(["branch", "--show-current"]).strip()
    head = git_text(["rev-parse", "HEAD"]).strip().lower()
    require(branch == TARGET_BRANCH, f"branch mismatch: {branch}")
    require(re.fullmatch(r"[0-9a-f]{40}", head) is not None, "HEAD is not a full commit SHA")

    ancestry = run_process(["git", "merge-base", "--is-ancestor", SOURCE_COMMIT, head])
    require(ancestry.returncode == 0, f"HEAD is not a descendant of {SOURCE_COMMIT}")
    if not allow_dirty:
        require(head != SOURCE_COMMIT, "final handoff branch still points at the V15.18A source commit")

    if expected_commit:
        expected = expected_commit.strip().lower()
        require(re.fullmatch(r"[0-9a-f]{40}", expected) is not None, "--expected-commit must be 40 lowercase/uppercase hex characters")
        require(head == expected, f"HEAD {head} != expected final commit {expected}")

    status_raw = git_bytes(["status", "--porcelain=v1", "-z"])
    dirty_rows = [row for row in status_raw.split(b"\0") if row]
    if not allow_dirty:
        require(not dirty_rows, "working tree/index is not clean")

    fsck = run_process(["git", "fsck", "--full"], timeout=300.0)
    require(
        fsck.returncode == 0,
        "git fsck --full failed: " + clean_error(fsck.stderr or fsck.stdout),
    )

    remote_url = git_text(["remote", "get-url", "origin"]).strip()
    require(
        remote_url.rstrip("/") == EXPECTED_REMOTE_URL.rstrip("/"),
        f"origin URL mismatch: {remote_url}",
    )

    remote_branch_head: str | None = None
    remote_tag_head: str | None = None
    if not allow_dirty:
        local_tag_type = git_text(["cat-file", "-t", f"refs/tags/{TARGET_TAG}"]).strip()
        require(local_tag_type == "tag", f"target tag is not annotated: {TARGET_TAG}")
        local_tag_head = git_text(["rev-parse", f"refs/tags/{TARGET_TAG}^{{}}"]).strip().lower()
        require(local_tag_head == head, f"local annotated tag does not peel to HEAD: {local_tag_head}")

        remote = run_process(
            [
                "git",
                "ls-remote",
                "origin",
                f"refs/heads/{TARGET_BRANCH}",
                f"refs/tags/{TARGET_TAG}",
                f"refs/tags/{TARGET_TAG}^{{}}",
            ],
            timeout=120.0,
        )
        require(
            remote.returncode == 0,
            "git ls-remote failed: " + clean_error(remote.stderr or remote.stdout),
        )
        remote_refs: dict[str, str] = {}
        for line in remote.stdout.splitlines():
            fields = line.split("\t", 1)
            require(len(fields) == 2, f"malformed ls-remote row: {line}")
            remote_refs[fields[1]] = fields[0].lower()
        remote_branch_head = remote_refs.get(f"refs/heads/{TARGET_BRANCH}")
        remote_tag_head = remote_refs.get(f"refs/tags/{TARGET_TAG}^{{}}")
        require(remote_branch_head == head, f"remote branch does not equal HEAD: {remote_branch_head}")
        require(remote_tag_head == head, f"remote annotated tag does not peel to HEAD: {remote_tag_head}")
        require(
            f"refs/tags/{TARGET_TAG}" in remote_refs,
            f"remote annotated tag object is missing: {TARGET_TAG}",
        )
    return {
        "branch": branch,
        "commit": head,
        "source_commit_is_ancestor": True,
        "dirty_entry_count": len(dirty_rows),
        "tracked_file_count": len(entries),
        "origin": remote_url,
        "recommended_origin": EXPECTED_REMOTE_URL,
        "remote_branch_head": remote_branch_head,
        "remote_tag_peeled_head": remote_tag_head,
        "git_fsck_full": "PASS",
    }


def check_authorities(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Any]:
    hashes: dict[str, str] = {}
    for relative, expected in EXPECTED_AUTHORITIES.items():
        require_tracked(relative, entries, allow_dirty=allow_dirty)
        path = repo_path(relative)
        require(path.is_file(), f"authority is not a regular file: {relative}")
        actual = sha256_file(path)
        require(actual == expected, f"authority SHA256 mismatch: {relative}: {actual}")
        hashes[relative] = actual
    return {
        "authority_count": len(hashes),
        "production_mjcf_sha256": hashes[MJCF_REL],
        "production_bridge_sha256": hashes[BRIDGE_REL],
        "all_sha256_match": True,
    }


def combined_paths(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> set[str]:
    paths = set(entries)
    if allow_dirty:
        paths.update(untracked_paths())
    return paths


def check_repository_inventory(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Any]:
    all_paths = combined_paths(entries, allow_dirty=allow_dirty)
    case_groups: dict[str, list[str]] = {}
    nfc_groups: dict[str, list[str]] = {}
    for relative in all_paths:
        case_groups.setdefault(relative.casefold(), []).append(relative)
        nfc_groups.setdefault(unicodedata.normalize("NFC", relative), []).append(relative)
    case_collisions = [sorted(rows) for rows in case_groups.values() if len(rows) > 1]
    nfc_collisions = [sorted(rows) for rows in nfc_groups.values() if len(rows) > 1]
    require(not case_collisions, f"case-sensitive path collisions: {case_collisions}")
    require(not nfc_collisions, f"Unicode NFC path collisions: {nfc_collisions}")

    lfs_pointers: list[str] = []
    lfs_attributes: list[str] = []
    symlink_rows: list[dict[str, Any]] = []
    total_bytes = 0
    size_rows: list[tuple[int, str]] = []
    forbidden_generated: list[str] = []
    forbidden_segments = {"build", "install", "log", ".venv", "__pycache__"}

    for relative, entry in entries.items():
        parts = set(PurePosixPath(relative).parts)
        if parts & forbidden_segments or relative.endswith((".pyc", ".pyo")):
            forbidden_generated.append(relative)

        path = repo_path(relative)
        require(path.exists() or path.is_symlink(), f"tracked path is absent from worktree: {relative}")
        if entry.mode == "120000":
            require(path.is_symlink(), f"tracked symlink was not checked out as a symlink: {relative}")
            target_exists = path.exists()
            target_inside = False
            if target_exists:
                resolved = path.resolve()
                target_inside = resolved == ROOT.resolve() or ROOT.resolve() in resolved.parents
            symlink_rows.append(
                {"path": relative, "target_exists": target_exists, "target_inside_repository": target_inside}
            )
            require(target_exists, f"broken tracked symlink: {relative}")
            require(target_inside, f"tracked symlink escapes repository: {relative}")
            continue

        require(path.is_file(), f"tracked path is not a regular file: {relative}")
        size = path.stat().st_size
        total_bytes += size
        size_rows.append((size, relative))
        require(size <= MAX_TRACKED_BYTES, f"tracked file exceeds 100 MB: {relative} ({size} bytes)")
        with path.open("rb") as stream:
            prefix = stream.read(1024)
        if LFS_POINTER_MARKER in prefix:
            lfs_pointers.append(relative)

    require(not forbidden_generated, f"generated/cache paths are tracked: {forbidden_generated}")
    require(not lfs_pointers, f"Git LFS pointer files found: {lfs_pointers}")

    for relative in sorted(path for path in entries if path.endswith(".gitattributes")):
        text = repo_path(relative).read_text(encoding="utf-8")
        if re.search(r"(?:filter|diff|merge)\s*=\s*lfs(?:\s|$)", text, re.IGNORECASE):
            lfs_attributes.append(relative)
    require(not lfs_attributes, f"Git LFS attributes found: {lfs_attributes}")
    require(".lfsconfig" not in entries, "tracked .lfsconfig makes the repository depend on Git LFS")

    attributes = repo_path(".gitattributes").read_text(encoding="utf-8")
    require("* text=auto eol=lf" in attributes, ".gitattributes lacks the cross-platform LF default")
    require(MJCF_REL + " text eol=crlf" in attributes, ".gitattributes lacks the production MJCF CRLF byte contract")
    for binary_pattern in ("*.stl binary", "*.FCStd binary", "*.step binary"):
        require(binary_pattern in attributes, f"ordinary-Git binary attribute missing: {binary_pattern}")

    submodules = run_process(["git", "submodule", "status", "--recursive"])
    require(
        submodules.returncode == 0,
        "git submodule status failed: " + clean_error(submodules.stderr or submodules.stdout),
    )
    submodule_rows = [row for row in str(submodules.stdout).splitlines() if row.strip()]
    bad_submodules = [row for row in submodule_rows if row[0] in "-+U"]
    require(not bad_submodules, f"uninitialized/divergent submodules: {bad_submodules}")

    largest = [
        {"path": relative, "bytes": size}
        for size, relative in sorted(size_rows, reverse=True)[:10]
    ]
    count_objects = git_text(["count-objects", "-vH"]).strip().splitlines()
    return {
        "tracked_file_count": len(entries),
        "tracked_total_bytes": total_bytes,
        "largest_tracked_files": largest,
        "lfs_pointer_count": 0,
        "lfs_attribute_count": 0,
        "case_collision_count": 0,
        "nfc_collision_count": 0,
        "tracked_symlink_count": len(symlink_rows),
        "broken_symlink_count": 0,
        "submodule_count": len(submodule_rows),
        "uninitialized_submodule_count": 0,
        "tracked_generated_path_count": 0,
        "git_count_objects": count_objects,
    }


def check_shell_contract(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Any]:
    paths = sorted(path for path in combined_paths(entries, allow_dirty=allow_dirty) if path.endswith(".sh"))
    require(paths, "repository contains no shell handoff script")
    executable: list[str] = []
    development_untracked: list[str] = []
    for relative in paths:
        path = repo_path(relative)
        require(path.is_file(), f"shell script missing: {relative}")
        payload = path.read_bytes()
        require(b"\r" not in payload, f"shell script is not LF-only: {relative}")
        require(payload.startswith(b"#!/usr/bin/env bash\n"), f"shell script lacks the portable bash shebang: {relative}")
        require(b"set -euo pipefail\n" in payload[:256], f"shell script lacks strict mode: {relative}")
        entry = entries.get(relative)
        if entry is None:
            require(allow_dirty, f"shell script is untracked: {relative}")
            development_untracked.append(relative)
        else:
            require(entry.mode == "100755", f"shell script Git mode is not executable (100755): {relative} ({entry.mode})")
            executable.append(relative)
    return {
        "shell_script_count": len(paths),
        "lf_only_count": len(paths),
        "git_executable_count": len(executable),
        "development_untracked_mode_pending": development_untracked,
    }


def is_runtime_path(relative: str) -> bool:
    in_scope = relative.startswith("tools/") or relative.startswith(ROS_WS_REL + "/")
    if not in_scope:
        return False
    name = PurePosixPath(relative).name
    return name in RUNTIME_BASENAMES or name.endswith(RUNTIME_SUFFIXES)


def check_runtime_absolute_paths(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Any]:
    runtime_paths = sorted(
        path
        for path in combined_paths(entries, allow_dirty=allow_dirty)
        if is_runtime_path(path)
    )
    drive_pattern = re.compile(r"(?<![A-Za-z0-9_])[A-Za-z]:[\\/]")
    unc_pattern = re.compile(r"\\\\[A-Za-z0-9.$_-]+\\[A-Za-z0-9.$_-]+")
    user_root = "/" + "home" + "/"
    mac_user_root = "/" + "Users" + "/"
    fixed_home_pattern = re.compile(
        "(?:" + re.escape(user_root) + "|" + re.escape(mac_user_root) + r")[A-Za-z0-9._-]+(?:/|$)"
    )
    root_home_pattern = re.compile("/" + "root" + r"(?:/|$)")
    hits: list[dict[str, Any]] = []
    for relative in runtime_paths:
        path = repo_path(relative)
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise HandoffError(f"runtime text is not readable UTF-8: {relative}: {clean_error(error)}") from error
        for line_number, line in enumerate(text.splitlines(), 1):
            labels = []
            if drive_pattern.search(line):
                labels.append("drive_absolute")
            if unc_pattern.search(line):
                labels.append("unc_absolute")
            if fixed_home_pattern.search(line) or root_home_pattern.search(line):
                labels.append("fixed_user_home")
            if labels:
                hits.append(
                    {
                        "path": relative,
                        "line": line_number,
                        "classes": labels,
                        "text": line.strip()[:240],
                    }
                )
    require(not hits, f"runtime absolute path hits: {json.dumps(hits, ensure_ascii=False)}")
    return {"runtime_file_count": len(runtime_paths), "absolute_path_hit_count": 0}


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def discover_ros_packages(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Path]:
    source_root = repo_path(ROS_SRC_REL)
    require(source_root.is_dir(), f"ROS source directory missing: {ROS_SRC_REL}")
    packages: dict[str, Path] = {}
    package_files = sorted(source_root.rglob("package.xml"))
    for package_xml in package_files:
        relative_package = resolved_repo_relative(package_xml, label="package.xml")
        require_tracked(relative_package, entries, allow_dirty=allow_dirty)
        try:
            root = ET.parse(package_xml).getroot()
        except (OSError, ET.ParseError) as error:
            raise HandoffError(f"invalid package.xml {relative_package}: {clean_error(error)}") from error
        require(local_name(root.tag) == "package", f"package.xml root is not <package>: {relative_package}")
        names = [str(node.text or "").strip() for node in root if local_name(node.tag) == "name"]
        require(len(names) == 1 and names[0], f"package.xml must contain exactly one name: {relative_package}")
        name = names[0]
        require(name not in packages, f"duplicate ROS package name: {name}")

        required_tags = {"version", "description", "maintainer", "license"}
        present_tags = {local_name(node.tag) for node in root if str(node.text or "").strip()}
        require(required_tags <= present_tags, f"package.xml metadata incomplete: {relative_package}")
        buildtools = [str(node.text or "").strip() for node in root if local_name(node.tag) == "buildtool_depend"]
        require("ament_cmake" in buildtools, f"package lacks ament_cmake buildtool dependency: {name}")
        build_types = [
            str(node.text or "").strip()
            for node in root.iter()
            if local_name(node.tag) == "build_type"
        ]
        require(build_types == ["ament_cmake"], f"package build_type is not exact ament_cmake: {name}")

        cmake = package_xml.parent / "CMakeLists.txt"
        relative_cmake = resolved_repo_relative(cmake, label="CMakeLists.txt")
        require_tracked(relative_cmake, entries, allow_dirty=allow_dirty)
        cmake_text = cmake.read_text(encoding="utf-8")
        require(re.search(r"\bproject\s*\(\s*" + re.escape(name) + r"\s*\)", cmake_text) is not None, f"CMake project name mismatch: {name}")
        require(re.search(r"\bfind_package\s*\(\s*ament_cmake\s+REQUIRED\s*\)", cmake_text) is not None, f"CMake lacks required ament_cmake: {name}")
        require(re.search(r"\bament_package\s*\(\s*\)", cmake_text) is not None, f"CMake lacks ament_package(): {name}")
        packages[name] = package_xml.parent

    require(set(packages) == EXPECTED_ROS_PACKAGES, f"ROS package set mismatch: {sorted(packages)}")

    for name, package_root in packages.items():
        xml_root = ET.parse(package_root / "package.xml").getroot()
        dependencies = {
            str(node.text or "").strip()
            for node in xml_root
            if local_name(node.tag).endswith("depend") and str(node.text or "").strip()
        }
        missing_internal = sorted(
            dependency
            for dependency in dependencies
            if dependency.startswith("go_m8010_") and dependency not in packages
        )
        require(not missing_internal, f"{name} has missing internal ROS dependencies: {missing_internal}")
    return packages


def substitute_xacro_reference(raw: str, arguments: Mapping[str, str]) -> str:
    result = raw
    arg_pattern = re.compile(r"\$\(\s*arg\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")
    for _ in range(16):
        match = arg_pattern.search(result)
        if not match:
            break
        name = match.group(1)
        require(name in arguments, f"mesh reference uses unknown Xacro arg: {name}")
        result = result[: match.start()] + arguments[name] + result[match.end() :]
    return result


def check_urdf_mesh_references(
    entries: Mapping[str, GitEntry],
    packages: Mapping[str, Path],
    *,
    allow_dirty: bool,
) -> dict[str, Any]:
    source_root = repo_path(ROS_SRC_REL)
    xml_files = sorted(
        path
        for path in source_root.rglob("*")
        if path.is_file() and (path.name.endswith(".xacro") or path.name.endswith(".urdf"))
    )
    require(xml_files, "ROS workspace has no URDF/Xacro files")
    resolved_rows: list[dict[str, str]] = []
    for xml_path in xml_files:
        relative_xml = resolved_repo_relative(xml_path, label="URDF/Xacro")
        require_tracked(relative_xml, entries, allow_dirty=allow_dirty)
        try:
            root = ET.parse(xml_path).getroot()
        except (OSError, ET.ParseError) as error:
            raise HandoffError(f"invalid URDF/Xacro {relative_xml}: {clean_error(error)}") from error
        arguments = {
            str(node.get("name")): str(node.get("default"))
            for node in root.iter()
            if local_name(node.tag) == "arg" and node.get("name") and node.get("default") is not None
        }
        for node in root.iter():
            if local_name(node.tag) != "mesh" or not node.get("filename"):
                continue
            raw = str(node.get("filename"))
            reference = substitute_xacro_reference(raw, arguments)

            find_match = re.fullmatch(r"\$\(\s*find\s+([A-Za-z0-9_]+)\s*\)(/.*)", reference)
            if find_match:
                package_name, suffix = find_match.groups()
                require(package_name in packages, f"unknown package in mesh reference: {raw}")
                candidate = packages[package_name].joinpath(*PurePosixPath(suffix.lstrip("/")).parts)
            elif reference.startswith("package://"):
                remainder = reference[len("package://") :]
                package_name, separator, suffix = remainder.partition("/")
                require(separator and package_name in packages, f"unknown/malformed package mesh URI: {raw}")
                candidate = packages[package_name].joinpath(*PurePosixPath(suffix).parts)
            else:
                require("$(" not in reference and "${" not in reference, f"unresolved mesh substitution: {raw}")
                require(
                    not PurePosixPath(reference).is_absolute()
                    and not PureWindowsPath(reference).is_absolute(),
                    f"absolute URDF/Xacro mesh reference: {raw}",
                )
                candidate = xml_path.parent.joinpath(*PurePosixPath(reference).parts)

            relative_asset = resolved_repo_relative(candidate, label="URDF/Xacro mesh")
            require(candidate.resolve().is_file(), f"URDF/Xacro mesh is missing: {raw} -> {relative_asset}")
            require_tracked(relative_asset, entries, allow_dirty=allow_dirty)
            with candidate.resolve().open("rb") as stream:
                stream.read(1)
            resolved_rows.append({"source": relative_xml, "reference": raw, "asset": relative_asset})

    require(resolved_rows, "URDF/Xacro files contain no mesh references")
    return {
        "ros_package_count": len(packages),
        "ros_packages": sorted(packages),
        "urdf_xacro_file_count": len(xml_files),
        "mesh_reference_count": len(resolved_rows),
        "unique_mesh_asset_count": len({row["asset"] for row in resolved_rows}),
        "all_mesh_assets_tracked": True,
    }


def check_final_artifacts(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Any]:
    for relative in sorted(REQUIRED_TRACKED):
        require_tracked(relative, entries, allow_dirty=allow_dirty)

    requirements = [
        line.strip()
        for line in repo_path("requirements-sim-v15_18b.txt").read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    require(requirements == ["mujoco==3.11.0", "numpy==2.2.6"], f"unexpected direct Python dependency set: {requirements}")

    artifact_hashes: dict[str, str] = {}
    for relative, expected in EXPECTED_V15_18B_ARTIFACTS.items():
        actual = sha256_file(repo_path(relative))
        require(actual == expected, f"V15.18B artifact SHA256 mismatch: {relative}: {actual}")
        artifact_hashes[relative] = actual

    readme = repo_path("README.md").read_text(encoding="utf-8")
    require("当前主线为 V15.13" not in readme, "README still claims V15.13 is current")
    require("git lfs pull" not in readme.lower(), "README still instructs git lfs pull")
    require("不使用 Git LFS" in readme, "README lacks the ordinary-Git/no-LFS statement")
    require("V15.18B" in readme and "V15.19" in readme, "README lacks current/next phase identity")
    require(
        "V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION" in readme,
        "README lacks the V15.18B numerical-integrator limitation status",
    )
    require(
        "PURE_SIMULATION_PHASE = COMPLETE" in readme,
        "README does not mark the pure-simulation phase complete",
    )
    require(
        "NEXT_PHASE = V15.19 REAL_HARDWARE_READONLY_BRINGUP" in readme,
        "README lacks the exact next-phase identity",
    )
    require(
        "UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST" in readme,
        "README does not preserve the nonblocking Ubuntu pending state",
    )

    passive = read_json(repo_path("V15_18B_短时被动重力动力学验收.json"))
    require(
        passive.get("schema") == "go-m8010-arm-v15.18b-passive-gravity-dynamics-audit/2.0",
        "V15.18B authority schema is not 2.0",
    )
    require(
        passive.get("final_status")
        == "V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION",
        "V15.18B authority lacks the exact accepted limitation status",
    )
    require(passive.get("pass") is True, "V15.18B authority pass flag is not true")
    require(
        passive.get("root_cause_classification")
        == "NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED",
        "V15.18B root-cause classification is not confirmed",
    )
    require(
        passive.get("continuous_time_dynamics_model") == "PASS",
        "continuous-time dynamics model is not PASS",
    )
    require(
        passive.get("ubuntu_runtime_verification") == "PENDING_ON_TARGET_HOST",
        "Ubuntu runtime status is not the allowed target-host pending state",
    )
    require(
        passive.get("production_integrator_limitation") == PRODUCTION_INTEGRATOR_LIMITATION,
        "V15.18B production-integrator limitation text is not exact",
    )
    require(
        passive.get("root_cause_condition_keys") == ROOT_CAUSE_CONDITION_KEYS,
        "V15.18B root-cause condition key order/content is not exact-11",
    )
    condition_map = passive.get("root_cause_condition_map_exact_11")
    require(isinstance(condition_map, dict), "V15.18B exact-11 condition map is missing")
    require(list(condition_map) == sorted(ROOT_CAUSE_CONDITION_KEYS), "V15.18B exact-11 map keys are stale")
    require(all(value is True for value in condition_map.values()), "V15.18B exact-11 condition map is not all PASS")
    gates = passive.get("acceptance_gates")
    require(isinstance(gates, dict) and gates, "V15.18B acceptance gates are missing")
    require(all(value is True for value in gates.values()), "one or more V15.18B acceptance gates failed")
    protected = passive.get("protected_hashes")
    require(
        isinstance(protected, dict) and protected.get("all_unchanged") is True,
        "V15.18B protected authority before/after gate failed",
    )
    require(passive.get("unresolved_items") == [], "V15.18B authority has unresolved items")
    require(
        passive.get("hard_unresolved_items") == [],
        "V15.18B authority has hard unresolved items",
    )

    handoff = read_json(repo_path("V15_18B_GitHub与Ubuntu交付验收.json"))
    require(
        handoff.get("schema")
        == "go-m8010-arm-v15.18b-final-simulation-github-handoff/2.0",
        "final handoff report schema is stale",
    )
    require(handoff.get("pass") is True, "final handoff report pass flag is not true")
    require(
        handoff.get("status") == "V15.18B FINAL_SIMULATION_AND_GITHUB_HANDOFF = PASS",
        "final handoff report status is not PASS",
    )
    require(
        handoff.get("physics_status")
        == "V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION",
        "final handoff report physics status is stale",
    )
    require(
        handoff.get("ubuntu_runtime_verification") == "PENDING_ON_TARGET_HOST",
        "final handoff report Ubuntu status is stale",
    )
    require(handoff.get("hard_unresolved_items") == [], "final handoff report has hard unresolved items")
    binding = handoff.get("post_commit_binding")
    require(
        isinstance(binding, dict)
        and binding.get("self_referential_commit_sha_serialized") is False,
        "final handoff report lacks the non-self-referential commit binding",
    )
    handoff_markdown = repo_path("V15_18B_GitHub与Ubuntu交付验收.md").read_text(encoding="utf-8")
    for required_text in (
        "V15.18B FINAL_SIMULATION_AND_GITHUB_HANDOFF = PASS",
        "PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION",
        "UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST",
        "Hard unresolved items",
        "`[]`",
    ):
        require(required_text in handoff_markdown, f"final handoff Markdown lacks: {required_text}")
    return {
        "required_tracked_path_count": len(REQUIRED_TRACKED),
        "direct_python_dependencies": requirements,
        "v15_18b_final_status": passive.get("final_status"),
        "v15_18b_unresolved_item_count": 0,
        "root_cause_classification": passive.get("root_cause_classification"),
        "continuous_time_dynamics_model": passive.get("continuous_time_dynamics_model"),
        "ubuntu_runtime_verification": passive.get("ubuntu_runtime_verification"),
        "artifact_sha256": artifact_hashes,
        "root_cause_condition_count": len(condition_map),
        "acceptance_gate_count": len(gates),
        "final_handoff_status": handoff.get("status"),
    }


def mjcf_file_assets(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> tuple[ET.Element, list[tuple[ET.Element, str, Path, str]]]:
    source_path = repo_path(MJCF_REL)
    try:
        root = ET.fromstring(source_path.read_bytes())
    except (OSError, ET.ParseError) as error:
        raise HandoffError(f"production MJCF is invalid XML: {clean_error(error)}") from error
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    rows: list[tuple[ET.Element, str, Path, str]] = []
    raw_seen: set[str] = set()
    asset_seen: set[str] = set()
    for node in root.iter():
        if not node.get("file"):
            continue
        raw = str(node.get("file"))
        require("\\" not in raw, f"MJCF file reference is not POSIX relative: {raw}")
        require(
            not PurePosixPath(raw).is_absolute() and not PureWindowsPath(raw).is_absolute(),
            f"absolute MJCF file reference: {raw}",
        )
        base = mesh_dir if local_name(node.tag) == "mesh" else texture_dir
        candidate = base.joinpath(*PurePosixPath(raw).parts)
        relative = resolved_repo_relative(candidate, label="MJCF asset")
        require(candidate.resolve().is_file(), f"MJCF asset is missing: {raw} -> {relative}")
        require_tracked(relative, entries, allow_dirty=allow_dirty)
        with candidate.resolve().open("rb") as stream:
            stream.read(1)
        require(raw not in raw_seen, f"duplicate MJCF file reference: {raw}")
        require(relative not in asset_seen, f"duplicate resolved MJCF asset: {relative}")
        raw_seen.add(raw)
        asset_seen.add(relative)
        rows.append((node, raw, candidate.resolve(), relative))
    require(len(rows) == 1008, f"production MJCF file reference count is not 1008: {len(rows)}")
    require(all(local_name(node.tag) == "mesh" for node, _, _, _ in rows), "production MJCF contains a non-mesh file reference")
    return root, rows


def check_mjcf_assets_and_compile(
    entries: Mapping[str, GitEntry], *, allow_dirty: bool
) -> dict[str, Any]:
    before = sha256_file(repo_path(MJCF_REL))
    require(before == EXPECTED_AUTHORITIES[MJCF_REL], "production MJCF hash changed before compile")
    root, rows = mjcf_file_assets(entries, allow_dirty=allow_dirty)

    try:
        import mujoco  # type: ignore
    except Exception as error:
        raise HandoffError(
            "MuJoCo import failed; install requirements-sim-v15_18b.txt: "
            + clean_error(error)
        ) from error
    require(mujoco.__version__ == "3.11.0", f"MuJoCo version is not 3.11.0: {mujoco.__version__}")

    compiler = root.find("./compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")

    assets: dict[str, bytes] = {}
    for index, (node, _, path, _) in enumerate(rows):
        name = f"asset_{index:04d}{path.suffix.lower() or '.bin'}"
        require(name not in assets, f"internal compile asset name collision: {name}")
        assets[name] = path.read_bytes()
        node.set("file", name)

    try:
        xml_text = ET.tostring(root, encoding="unicode")
        model = mujoco.MjModel.from_xml_string(xml_text, assets=assets)
    except Exception as error:
        raise HandoffError(f"production MJCF compile failed: {clean_error(error)}") from error

    readback = {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nbody": int(model.nbody),
        "nmesh": int(model.nmesh),
        "ngeom": int(model.ngeom),
        "gravity_m_s2": [float(value) for value in model.opt.gravity],
    }
    require(readback["nq"] == 6 and readback["nv"] == 6, f"compiled joint DOF count changed: {readback}")
    require(readback["nbody"] == 11, f"compiled body count changed: {readback['nbody']}")
    require(readback["nmesh"] == 1008, f"compiled mesh count changed: {readback['nmesh']}")
    require(readback["ngeom"] == 1009, f"compiled geom count changed: {readback['ngeom']}")
    require(max(abs(value) for value in readback["gravity_m_s2"]) == 0.0, "production MJCF default gravity is not OFF")

    after = sha256_file(repo_path(MJCF_REL))
    require(after == before, "production MJCF bytes changed during compile/readback")
    del model
    del assets
    gc.collect()
    return {
        "mujoco_version": mujoco.__version__,
        "production_mjcf_sha256_before": before,
        "production_mjcf_sha256_after": after,
        "mjcf_file_reference_count": len(rows),
        "unique_readable_tracked_asset_count": len({relative for _, _, _, relative in rows}),
        "compile": "PASS",
        "readback": readback,
    }


def render_value(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--expected-commit",
        default=os.environ.get("V15_18B_EXPECTED_COMMIT"),
        help="require HEAD to equal a 40-character final commit",
    )
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="pre-commit diagnostics only; includes non-ignored untracked paths",
    )
    arguments = parser.parse_args()

    checks: list[tuple[str, Callable[[], dict[str, Any]]]] = []
    try:
        entries = parse_git_index()
    except HandoffError as error:
        print("REPOSITORY_HANDOFF=FAIL")
        print("CHECK_GIT_INDEX=FAIL")
        print("ERROR=" + clean_error(error))
        return 1

    def package_check() -> dict[str, Any]:
        packages = discover_ros_packages(entries, allow_dirty=arguments.allow_dirty)
        return check_urdf_mesh_references(
            entries, packages, allow_dirty=arguments.allow_dirty
        )

    checks.extend(
        [
            (
                "GIT_IDENTITY_FSCK",
                lambda: check_git_identity(
                    entries,
                    expected_commit=arguments.expected_commit,
                    allow_dirty=arguments.allow_dirty,
                ),
            ),
            (
                "FROZEN_AUTHORITIES",
                lambda: check_authorities(entries, allow_dirty=arguments.allow_dirty),
            ),
            (
                "REPOSITORY_INVENTORY",
                lambda: check_repository_inventory(entries, allow_dirty=arguments.allow_dirty),
            ),
            (
                "SHELL_EOL_EXECUTABLE",
                lambda: check_shell_contract(entries, allow_dirty=arguments.allow_dirty),
            ),
            (
                "RUNTIME_ABSOLUTE_PATHS",
                lambda: check_runtime_absolute_paths(entries, allow_dirty=arguments.allow_dirty),
            ),
            ("ROS_PACKAGES_URDF_ASSETS", package_check),
            (
                "FINAL_HANDOFF_ARTIFACTS",
                lambda: check_final_artifacts(entries, allow_dirty=arguments.allow_dirty),
            ),
            (
                "MJCF_1008_COMPILE_READBACK",
                lambda: check_mjcf_assets_and_compile(entries, allow_dirty=arguments.allow_dirty),
            ),
        ]
    )

    failures: list[tuple[str, str]] = []
    results: dict[str, dict[str, Any]] = {}
    for name, check in checks:
        try:
            result = check()
        except (HandoffError, OSError, UnicodeError, ET.ParseError, ValueError) as error:
            message = clean_error(error)
            failures.append((name, message))
            print(f"CHECK_{name}=FAIL")
            print(f"ERROR_{name}={message}")
        else:
            results[name] = result
            print(f"CHECK_{name}=PASS")
            print(f"DETAIL_{name}={render_value(result)}")

    if failures:
        print("REPOSITORY_HANDOFF=FAIL")
        print(f"FAILURE_COUNT={len(failures)}")
        return 1

    identity = results["GIT_IDENTITY_FSCK"]
    mjcf = results["MJCF_1008_COMPILE_READBACK"]
    print(f"BRANCH={identity['branch']}")
    print(f"COMMIT={identity['commit']}")
    print(f"PRODUCTION_MJCF_SHA256={mjcf['production_mjcf_sha256_after']}")
    print("MJCF_ASSET_REFERENCES=1008/1008")
    if arguments.allow_dirty:
        print("REPOSITORY_HANDOFF=DEVELOPMENT_PASS_NOT_FINAL")
    else:
        print("REPOSITORY_HANDOFF=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
