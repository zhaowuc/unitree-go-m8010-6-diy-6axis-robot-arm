from __future__ import annotations

"""Export the exact V15.13 runtime self-collision pair matrix."""

import importlib.util
import itertools
import json
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App


WORKSPACE = Path(__file__).resolve().parent
AUDIT_PATH = WORKSPACE / "audit_robot_arm_v15_13_deep.py"
OUTPUT_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
OUTPUT_JSON = OUTPUT_ROOT / "V15_13_自碰撞对矩阵契约.json"


def load_module(path: Path, name: str):
    specification = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(specification)
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


def normalized(pairs):
    return sorted([sorted((first, second)) for first, second in pairs])


def main() -> None:
    audit = load_module(AUDIT_PATH, "_v15_13_audit_for_pair_export")
    guard = audit.load_guard()
    doc = App.openDocument(str(audit.FCSTD))
    if doc is None:
        raise RuntimeError("could not open V15.13 FCStd")
    try:
        full = audit.runtime_pair_matrix(doc, guard)
        changed_terminal = audit.changed_terminal_pair_matrix(doc, guard, full)
        names = sorted(audit.shape_checks(doc))
    finally:
        App.closeDocument(doc.Name)

    all_pairs = {tuple(sorted(pair)) for pair in itertools.combinations(names, 2)}
    full_set = {tuple(sorted(pair)) for pair in full}
    changed_set = {tuple(sorted(pair)) for pair in changed_terminal}
    report = {
        "schema": "go-m8010-arm-v15.13-self-collision-pair-contract/1.0",
        "revision": "V15.13-deep-audit-runtime-pair-matrix",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_audit": str(AUDIT_PATH),
        "source_fcstd": str(audit.FCSTD),
        "proxy_count": len(names),
        "all_unordered_pair_count": len(all_pairs),
        "runtime_full_pair_count": len(full_set),
        "v15_13_changed_terminal_regression_pair_count": len(changed_set),
        "proxies": names,
        "runtime_full_pairs": normalized(full_set),
        "runtime_excluded_pairs": normalized(all_pairs - full_set),
        "v15_13_changed_terminal_pairs": normalized(changed_set),
        "v15_13_503_pose_regression_scope": (
            "Only pairs involving J6 rotor or a gripper/camera proxy; unchanged upstream pairs inherit prior immutable QA."
        ),
    }
    OUTPUT_JSON.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": "PASS",
                "output": str(OUTPUT_JSON),
                "full_pairs": len(full_set),
                "changed_terminal_pairs": len(changed_set),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
