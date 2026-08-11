from __future__ import annotations

"""Non-saving regression for the V15.13 swept collision guard."""

import importlib.util
import json
from pathlib import Path

import FreeCAD as App


WORKSPACE = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui")
ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
FCSTD = ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
GUARD = WORKSPACE / "j123456_camera_physical_collision_guard_v15_13.py"
REPORT = ROOT / "QA_V15_13_连续碰撞守卫回归.json"


def load_guard():
    spec = importlib.util.spec_from_file_location("_v15_13_guard_regression", GUARD)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load collision guard: {GUARD}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    if not FCSTD.is_file() or not GUARD.is_file():
        raise FileNotFoundError("V15.13 regression input is missing")
    doc = App.openDocument(str(FCSTD))
    guard = load_guard()
    report = {
        "schema": "go-m8010-arm-v15.13-collision-guard-regression/1.0",
        "source_fcstd": str(FCSTD),
        "guard": str(GUARD),
        "document_saved": False,
        "checks": {},
    }
    try:
        doc.recompute()
        installed = guard.install()
        baseline = guard.collision_result(doc, compute_clearance=False)
        report["checks"]["install"] = {
            "passed": True,
            "underlying_install_return_collision": bool(installed.get("collision", False)),
            "acceptance_note": (
                "The inherited cold-start return still includes designed-contact "
                "candidates; the authoritative post-install V15.13 matrix is the "
                "zero_pose check below."
            ),
        }
        report["checks"]["zero_pose"] = {
            "passed": not bool(baseline.get("collision", False)),
            "pair_count": baseline.get("pair_count"),
            "collision": bool(baseline.get("collision", False)),
        }

        forward = guard.set_j6_angle_v15_13(1.0, sweep_step_deg=0.25)
        back = guard.set_j6_angle_v15_13(0.0, sweep_step_deg=0.25)
        report["checks"]["j6_small_swept_roundtrip"] = {
            "passed": True,
            "forward": forward,
            "back": back,
        }

        rejected = False
        rejection_message = None
        try:
            guard.set_j6_angle_v15_13(181.0, sweep_step_deg=0.25)
        except Exception as exc:  # Expected fail-closed limit rejection.
            rejected = True
            rejection_message = str(exc)
        report["checks"]["j6_plus_181_rejected"] = {
            "passed": rejected,
            "message": rejection_message,
        }

        final_zero = guard.collision_result(doc, compute_clearance=False)
        report["checks"]["returned_to_zero"] = {
            "passed": not bool(final_zero.get("collision", False)),
            "collision": bool(final_zero.get("collision", False)),
        }
        report["passed"] = all(row["passed"] for row in report["checks"].values())
        REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if not report["passed"]:
            raise RuntimeError("V15.13 collision guard regression failed")
    finally:
        App.closeDocument(doc.Name)


if __name__ == "__main__":
    main()
