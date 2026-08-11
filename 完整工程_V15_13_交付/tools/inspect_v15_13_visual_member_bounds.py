from __future__ import annotations

import json
from pathlib import Path

import FreeCAD as App
import Mesh


ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
FCSTD = ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
MANIFEST = ROOT / "rigid_links_v15_13" / "rigid_link_manifest.json"


def bounds(obj):
    if hasattr(obj, "Mesh"):
        mesh = Mesh.Mesh(obj.Mesh)
        mesh.Placement = obj.getGlobalPlacement()
        box = mesh.BoundBox
    else:
        shape = obj.Shape.copy()
        shape.Placement = obj.getGlobalPlacement()
        box = shape.BoundBox
    return {
        "min": [box.XMin, box.YMin, box.ZMin],
        "max": [box.XMax, box.YMax, box.ZMax],
        "size": [box.XLength, box.YLength, box.ZLength],
    }


def main():
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    doc = App.openDocument(str(FCSTD))
    report = {}
    try:
        for link, row in data["links"].items():
            members = {}
            for token in row["visual_members"]:
                if token.startswith("derived:"):
                    continue
                obj = doc.getObject(token)
                if obj is None:
                    raise RuntimeError(f"missing visual member: {token}")
                members[token] = bounds(obj)
            report[link] = members
    finally:
        App.closeDocument(doc.Name)
    target = ROOT / "QA_V15_13_visual成员世界包围盒.json"
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for link, members in report.items():
        print(f"[{link}]")
        for name, row in members.items():
            if max(row["size"]) > 800.0:
                print(name, row["size"], row["min"], row["max"])


if __name__ == "__main__":
    main()
