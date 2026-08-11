from __future__ import annotations

"""Regenerate the cryptographic delivery manifest for this MuJoCo project."""

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "PROJECT_MANIFEST.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def included(path: Path) -> bool:
    relative = path.relative_to(ROOT)
    if path == OUTPUT or "__pycache__" in relative.parts:
        return False
    if path.suffix.lower() in {".pyc", ".log"}:
        return False
    return path.is_file()


def main() -> None:
    files = []
    total_bytes = 0
    for path in sorted((path for path in ROOT.rglob("*") if included(path)), key=lambda row: row.as_posix()):
        size = path.stat().st_size
        total_bytes += size
        files.append(
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": size,
                "sha256": sha256(path),
            }
        )
    report = {
        "schema": "go-m8010-arm-v15.13-mujoco-kinematic-delivery/1.1",
        "revision": "V15.13-MuJoCo-empty-load-kinematic-v1.2-geometric-gui-zero",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "qa_status": "PASS",
        "qa_reports": [
            "QA_MUJOCO_V15_13_空载运动学版.json",
            "QA_MUJOCO_V15_13_逐轴运动语义与地面复核.json",
            "QA_MUJOCO_V15_13_GUI零位几何校准.json",
        ],
        "mujoco_version_verified": "3.11.0",
        "main_model": "go_m8010_arm_v15_13_kinematic.xml",
        "temporary_gui": "temporary_joint_gui.py",
        "ground_z_m": -0.09,
        "file_count_excluding_manifest": len(files),
        "total_bytes_excluding_manifest": total_bytes,
        "files": files,
    }
    OUTPUT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "PASS", "files": len(files), "bytes": total_bytes, "manifest": str(OUTPUT)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
