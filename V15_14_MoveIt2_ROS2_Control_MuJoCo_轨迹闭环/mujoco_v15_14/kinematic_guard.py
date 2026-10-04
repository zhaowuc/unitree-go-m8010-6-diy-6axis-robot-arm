from __future__ import annotations

"""Fail-closed position-limit and swept self-collision guard for V15.14."""

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_14_kinematic.xml"
PAIR_CONTRACT = ROOT / "collision_pair_contract_v15_14.json"
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")


class KinematicGuard:
    def __init__(
        self,
        model_path: Path = MODEL_XML,
        *,
        model: mujoco.MjModel | None = None,
        data: mujoco.MjData | None = None,
    ):
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(str(model_path))
        self.data = data if data is not None else mujoco.MjData(self.model)
        if data is not None and model is None:
            raise ValueError("data requires the matching model")
        self.joint_ids = {
            name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINTS
        }
        pair_contract = json.loads(PAIR_CONTRACT.read_text(encoding="utf-8"))
        self.allowed_proxy_pairs = {
            tuple(sorted(pair)) for pair in pair_contract["runtime_full_pairs"]
        }
        runtime_tokens = {token for pair in self.allowed_proxy_pairs for token in pair}
        model_tokens = set()
        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            )
            proxy = self._proxy(geom_name)
            if geom_name and geom_name.startswith("collision__"):
                model_tokens.add(proxy)
        missing = sorted(runtime_tokens - model_tokens)
        if missing:
            raise RuntimeError(
                "fail-closed: runtime collision tokens have no MJCF geom: "
                + ", ".join(missing)
            )

    @staticmethod
    def _proxy(name: str) -> str:
        if name and name.startswith("collision__"):
            parts = name.split("__")
            if len(parts) >= 4:
                return parts[2]
        return name

    def _set_deg(self, values_deg) -> None:
        for name, value_deg in zip(JOINTS, values_deg):
            joint_id = self.joint_ids[name]
            qadr = int(self.model.jnt_qposadr[joint_id])
            self.data.qpos[qadr] = math.radians(float(value_deg))
        mujoco.mj_forward(self.model, self.data)

    def _contacts(self) -> list[dict]:
        rows = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            first = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)
            )
            second = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)
            )
            proxy_pair = tuple(sorted((self._proxy(first), self._proxy(second))))
            is_ground_contact = "ground" in proxy_pair
            if not is_ground_contact and proxy_pair not in self.allowed_proxy_pairs:
                continue
            rows.append(
                {
                    "geom1": first,
                    "geom2": second,
                    "distance_m": float(contact.dist),
                    "proxy_pair": list(proxy_pair),
                    "contact_class": "ground" if is_ground_contact else "self_collision",
                }
            )
        return rows

    def check_pose_deg(self, values_deg) -> dict:
        values = [float(value) for value in values_deg]
        if len(values) != 6:
            raise ValueError("expected J1..J6 six angles")
        violations = []
        for name, value_deg in zip(JOINTS, values):
            joint_id = self.joint_ids[name]
            if not bool(self.model.jnt_limited[joint_id]):
                continue
            lower, upper = np.degrees(self.model.jnt_range[joint_id])
            if value_deg < lower - 1.0e-9 or value_deg > upper + 1.0e-9:
                violations.append(
                    {
                        "joint": name,
                        "value_deg": value_deg,
                        "allowed_deg": [float(lower), float(upper)],
                    }
                )
        if violations:
            return {
                "safe": False,
                "reason": "position_limit",
                "angles_deg": dict(zip(JOINTS, values)),
                "violations": violations,
                "contacts": [],
            }
        self._set_deg(values)
        contacts = self._contacts()
        return {
            "safe": not contacts,
            "reason": (
                "clear"
                if not contacts
                else ("ground_collision" if any(row["contact_class"] == "ground" for row in contacts) else "self_collision")
            ),
            "angles_deg": dict(zip(JOINTS, values)),
            "violations": [],
            "contacts": contacts,
        }

    def check_swept_deg(self, start_deg, target_deg, max_step_deg: float = 0.25) -> dict:
        start = np.asarray(start_deg, dtype=float)
        target = np.asarray(target_deg, dtype=float)
        if start.shape != (6,) or target.shape != (6,):
            raise ValueError("start and target must each contain J1..J6")
        steps = max(1, int(math.ceil(float(np.max(np.abs(target - start))) / max_step_deg)))
        for step in range(steps + 1):
            alpha = step / steps
            values = start + (target - start) * alpha
            result = self.check_pose_deg(values)
            if not result["safe"]:
                result.update(
                    swept_safe=False,
                    first_unsafe_step=step,
                    step_count=steps,
                    interpolation_fraction=alpha,
                )
                return result
        return {
            "safe": True,
            "swept_safe": True,
            "reason": "clear",
            "start_deg": dict(zip(JOINTS, start.tolist())),
            "target_deg": dict(zip(JOINTS, target.tolist())),
            "step_count": steps,
            "max_step_deg": max_step_deg,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("angles", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    args = parser.parse_args()
    result = KinematicGuard().check_pose_deg(args.angles)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["safe"] else 2)


if __name__ == "__main__":
    main()
