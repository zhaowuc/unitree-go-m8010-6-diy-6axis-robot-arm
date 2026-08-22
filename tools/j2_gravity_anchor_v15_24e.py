#!/usr/bin/env python3
"""Interactive SESSION_LOCAL_GRAVITY_ANCHOR_V1 editor for V15.24E."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

# The MuJoCo venv intentionally excludes Ubuntu's packaged Pillow.
sys.path.append("/usr/lib/python3/dist-packages")
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
from PIL import Image, ImageTk
import mujoco


MODEL_REL = Path(
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/"
    "go_m8010_arm_v15_14_kinematic.xml"
)
EXPECTED_MODEL_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
LEDGER_SHA256 = {
    "V15_15_实测质量账本_v1.json":
        "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    "V15_15_COM账本_v2.json":
        "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    "V15_16_刚体惯量_Engineering_V1.json":
        "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401",
}
OUTPUT_EXACT = Path("/tmp/v15_24e_model_anchor.json")
JOINTS = ("J2", "J3", "J4", "J5", "J6")
GATE = "MUJOCO_PHYSICAL_POSE_MATCHED=YES"
BOUNDARY_ROUTE_CONFIRMATION_SOURCE = (
    "USER_CHAT_EXPLICIT_BOUNDARY_B_PLUS_5DEG_STAGE_ROUTE_AUTHORIZATION"
)
BOUNDARY_TO_CENTER_OFFSET_DEG = 5.0
FORMAL_CENTER_RELATIVE_ROUTE_DEG = (0.0, 5.0, 0.0, -5.0, 0.0)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_authorities(repo: Path) -> tuple[Path, dict[str, str]]:
    model_path = repo / MODEL_REL
    if not model_path.is_file():
        raise RuntimeError("FROZEN_MODEL_MISSING")
    if sha256(model_path) != EXPECTED_MODEL_SHA256:
        raise RuntimeError("FROZEN_MODEL_HASH_MISMATCH_BLOCKED")
    hashes: dict[str, str] = {}
    for name, expected in LEDGER_SHA256.items():
        path = repo / name
        if not path.is_file():
            raise RuntimeError(f"LEDGER_MISSING:{name}")
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"LEDGER_HASH_MISMATCH_BLOCKED:{name}")
        hashes[name] = actual
    return model_path, hashes


class GravityAuthority:
    def __init__(self, model_path: Path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.gravity[:] = (0.0, 0.0, -9.81)
        self.qadr: dict[str, int] = {}
        self.dadr: dict[str, int] = {}
        self.jid: dict[str, int] = {}
        for name in ("J1",) + JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"MODEL_JOINT_MISSING:{name}")
            self.jid[name] = jid
            self.qadr[name] = int(self.model.jnt_qposadr[jid])
            self.dadr[name] = int(self.model.jnt_dofadr[jid])
        if self.model.nq != 6 or self.model.nv != 6:
            raise RuntimeError("MODEL_DOF_CONTRACT_MISMATCH")

    def joint_range_deg(self, name: str) -> tuple[float, float]:
        values = self.model.jnt_range[self.jid[name]]
        return float(math.degrees(values[0])), float(math.degrees(values[1]))

    def set_pose(self, q: dict[str, float]) -> None:
        self.data.qpos[:] = 0.0
        for name in JOINTS:
            self.data.qpos[self.qadr[name]] = q[name]
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def j2_gravity(self, q: dict[str, float]) -> float:
        self.set_pose(q)
        return float(self.data.qfrc_bias[self.dadr["J2"]])

    def _audit_j2_offsets(
            self, reference: dict[str, float],
            j2_offsets_deg: tuple[float, ...], method: str,
            reference_pose_label: str) -> dict:
        if len(j2_offsets_deg) < 5:
            raise RuntimeError("UNCERTAINTY_SWEEP_REQUIRES_AT_LEAST_5_J2_POINTS")
        if any(not math.isfinite(value) for value in j2_offsets_deg):
            raise RuntimeError("UNCERTAINTY_SWEEP_NONFINITE_J2_OFFSET_BLOCKED")
        for name in JOINTS:
            if name not in reference or not math.isfinite(reference[name]):
                raise RuntimeError(f"NONFINITE_REFERENCE_POSE_BLOCKED:{name}")
        samples: list[float] = []
        sample_records: list[dict] = []
        range_violations: list[dict] = []
        extrapolation_sample_count = 0
        nonfinite_sample_count = 0
        # Evaluate every requested coordinate exactly as supplied. In
        # particular, do not clamp a session pose or uncertainty corner to a
        # nominal MuJoCo joint range: this audit must expose extrapolation.
        for j2_delta_deg in j2_offsets_deg:
            for downstream in itertools.product((-5.0, 0.0, 5.0), repeat=4):
                deltas = dict(zip(("J3", "J4", "J5", "J6"), downstream))
                q = dict(reference)
                q["J2"] += math.radians(j2_delta_deg)
                for name, delta in deltas.items():
                    q[name] += math.radians(delta)
                outside_joint_range = False
                for name in JOINTS:
                    lo, hi = self.model.jnt_range[self.jid[name]]
                    # Treat values equal to a nominal endpoint within machine
                    # precision as on-range. This tolerance affects only the
                    # range label; q itself is still evaluated unchanged.
                    if q[name] < lo - 1e-12 or q[name] > hi + 1e-12:
                        outside_joint_range = True
                        range_violations.append({
                            "joint": name,
                            "q_rad": q[name],
                            "range_rad": [float(lo), float(hi)],
                            "j2_route_delta_deg": j2_delta_deg,
                            "downstream_delta_deg": deltas,
                        })
                if outside_joint_range:
                    extrapolation_sample_count += 1
                torque = self.j2_gravity(q)
                if not math.isfinite(torque):
                    nonfinite_sample_count += 1
                samples.append(torque)
                sample_records.append({
                    "j2_route_delta_deg": j2_delta_deg,
                    "downstream_delta_deg": deltas,
                    "outside_nominal_joint_range": outside_joint_range,
                    "q_model_rad_evaluated_without_clipping": dict(q),
                    "q_model_deg_evaluated_without_clipping": {
                        name: math.degrees(value) for name, value in q.items()
                    },
                    "tau_g_j2_nm": torque,
                })
        required_sample_count = len(j2_offsets_deg) * 81
        if len(samples) != required_sample_count:
            raise RuntimeError(
                "UNCERTAINTY_SWEEP_COVERAGE_INCOMPLETE_BLOCKED:"
                f"{len(samples)}/{required_sample_count}")
        if required_sample_count < 405:
            raise RuntimeError(
                f"UNCERTAINTY_SWEEP_BELOW_405_BLOCKED:{required_sample_count}")
        if nonfinite_sample_count:
            raise RuntimeError(
                "UNCERTAINTY_SWEEP_NONFINITE_TORQUE_BLOCKED:"
                f"{nonfinite_sample_count}/{required_sample_count}")
        minimum = min(samples)
        maximum = max(samples)
        sign_robust = (minimum > 0.0) or (maximum < 0.0)
        reference_torque = self.j2_gravity(reference)
        if not math.isfinite(reference_torque):
            raise RuntimeError("REFERENCE_GRAVITY_TORQUE_NONFINITE_BLOCKED")
        return {
            "method": method,
            "reference_pose_label": reference_pose_label,
            "j2_offsets_from_reference_deg": list(j2_offsets_deg),
            "sample_count": len(samples),
            "required_sample_count": required_sample_count,
            "coverage_complete": True,
            "all_samples_finite": True,
            "finite_sample_count": len(samples),
            "nonfinite_sample_count": 0,
            "joint_values_clipped": False,
            "nominal_joint_range_comparison_tolerance_rad": 1e-12,
            "nominal_joint_range_policy": (
                "EVALUATE_ALL_REQUESTED_COORDINATES_WITHOUT_CLIPPING_AND_RECORD_"
                "EVERY_EXTRAPOLATED_SAMPLE"
            ),
            "nominal_joint_range_extrapolation_sample_count":
                extrapolation_sample_count,
            "nominal_joint_range_violation_record_count": len(range_violations),
            "nominal_joint_range_extrapolation_reason": (
                "REQUESTED_ROUTE_OR_DOWNSTREAM_UNCERTAINTY_EXCEEDS_MODEL_NOMINAL_"
                "JOINT_RANGE;ALL_POINTS_EVALUATED_WITHOUT_CLIPPING"
                if range_violations else "NONE"),
            "nominal_joint_range_first_violations": range_violations[:20],
            "tau_g_j2_min_nm": minimum,
            "tau_g_j2_max_nm": maximum,
            "anchor_tau_g_j2_nm": reference_torque,
            "reference_tau_g_j2_nm": reference_torque,
            "sign": "POSITIVE" if reference_torque > 0.0 else
                    "NEGATIVE" if reference_torque < 0.0 else "ZERO",
            "sign_robust": sign_robust,
            "minimum_abs_tau_g_j2_nm": min(abs(value) for value in samples),
            "sample_records": sample_records,
        }

    def audit_uncertainty(self, anchor: dict[str, float]) -> dict:
        # Legacy/GUI contract: the anchor is the test center and J2 is audited
        # symmetrically through +/-5 degrees.
        return self._audit_j2_offsets(
            anchor, (-5.0, -2.5, 0.0, 2.5, 5.0),
            "J2_ROUTE_5_POINT_X_J3_J4_J5_J6_SIMULTANEOUS_PLUS_MINUS_5DEG_CORNERS",
            "SESSION_MODEL_ANCHOR")

    def audit_boundary_center_route(
            self, boundary: dict[str, float],
            center_offset_deg: float) -> dict:
        if not math.isfinite(center_offset_deg):
            raise RuntimeError("CENTER_OFFSET_NONFINITE_BLOCKED")
        if not math.isclose(
                center_offset_deg, BOUNDARY_TO_CENTER_OFFSET_DEG,
                rel_tol=0.0, abs_tol=1e-12):
            raise RuntimeError(
                "CENTER_OFFSET_MUST_BE_EXACTLY_PLUS_5_DEG_FOR_THIS_ROUTE")
        # B is the stable physical/model anchor. C=B+5deg is only a commanded
        # relative center. Formal C-relative 0,+5,0,-5,0 therefore occupies
        # the absolute B-relative interval [0,+10] and never crosses below B.
        j2_offsets = (0.0, 2.5, 5.0, 7.5, 10.0)
        full_route = (0.0,) + tuple(
            center_offset_deg + value
            for value in FORMAL_CENTER_RELATIVE_ROUTE_DEG) + (0.0,)
        audit = self._audit_j2_offsets(
            boundary, j2_offsets,
            "BOUNDARY_B_ABSOLUTE_ROUTE_0_TO_PLUS10_5_POINT_X_"
            "J3_J4_J5_J6_SIMULTANEOUS_PLUS_MINUS_5DEG_CORNERS",
            "BOUNDARY_B_MODEL_REFERENCE")
        audit.update({
            # These exact keys are the fail-closed analyzer contract.
            "anchor_role": "STABLE_BOUNDARY_B",
            "j2_route_offsets_deg": list(j2_offsets),
            "test_center_offset_from_boundary_deg": center_offset_deg,
            "formal_center_relative_route_deg":
                list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
            "full_planned_boundary_relative_route_deg": list(full_route),
            "planned_boundary_relative_min_deg": min(full_route),
            "planned_boundary_relative_max_deg": max(full_route),
            "crosses_below_boundary": min(full_route) < 0.0,
            # Descriptive aliases retained for readers of early drafts.
            "test_center_role": "RELATIVE_COMMAND_POINT_NOT_MODEL_ANCHOR",
            "boundary_to_test_center_j2_offset_deg": center_offset_deg,
            "formal_test_route_center_relative_deg":
                list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
            "full_planned_route_boundary_relative_deg": list(full_route),
            "planned_route_boundary_relative_min_deg": min(full_route),
            "planned_route_boundary_relative_max_deg": max(full_route),
            "planned_route_crosses_below_boundary": min(full_route) < 0.0,
            "audited_boundary_relative_offsets_deg": list(j2_offsets),
        })
        return audit


def freeze_anchor(output: Path, authority: GravityAuthority,
                  ledger_hashes: dict[str, str], q: dict[str, float],
                  confirmation_source: str,
                  slider_screenshot_sha256: str | None = None) -> dict:
    for name in JOINTS:
        value = q[name]
        lo, hi = authority.model.jnt_range[authority.jid[name]]
        if not math.isfinite(value) or value < lo - 1e-12 or value > hi + 1e-12:
            raise RuntimeError(f"MODEL_ANCHOR_OUTSIDE_NOMINAL_RANGE:{name}:{value}")
    audit = authority.audit_uncertainty(q)
    if not audit["coverage_complete"] or audit["sample_count"] != 405:
        raise RuntimeError("UNCERTAINTY_SWEEP_NOT_405_BLOCKED")
    if not audit["sign_robust"]:
        raise RuntimeError("GRAVITY_SIGN_NOT_ROBUST_BLOCKED")
    payload = {
        "schema": "SESSION_LOCAL_GRAVITY_ANCHOR_V1_MODEL",
        "scope": "SESSION_ONLY_NOT_PERMANENT_ZERO",
        "operator_gate": GATE,
        "operator_gate_entry_method": confirmation_source,
        "slider_screenshot_sha256": slider_screenshot_sha256,
        "model": {
            "path": MODEL_REL.as_posix(),
            "sha256": EXPECTED_MODEL_SHA256,
            "mujoco_version": mujoco.__version__,
            "runtime_gravity_m_s2": [0.0, 0.0, -9.81],
        },
        "frozen_ledgers_sha256": ledger_hashes,
        "anchor_role": "SESSION_MODEL_ANCHOR_AND_LEGACY_TEST_CENTER",
        "q_anchor_model_rad": q,
        "q_anchor_model_deg": {
            name: math.degrees(value) for name, value in q.items()
        },
        "uncertainty_audit": audit,
        "permanent_zero_modified": False,
        "cad_zero": "PENDING",
        "ros_zero": "PENDING",
    }
    temporary = output.with_name(output.name + ".tmp")
    temporary.unlink(missing_ok=True)
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8", newline="\n")
    temporary.replace(output)
    return audit


def pose_nominal_range_audit(
        authority: GravityAuthority, q: dict[str, float]) -> dict:
    violations: list[dict] = []
    for name in JOINTS:
        value = q[name]
        lo, hi = authority.model.jnt_range[authority.jid[name]]
        if value < lo - 1e-12 or value > hi + 1e-12:
            nearest = lo if value < lo else hi
            violations.append({
                "joint": name,
                "q_rad": value,
                "q_deg": math.degrees(value),
                "nominal_range_rad": [float(lo), float(hi)],
                "nominal_range_deg": [math.degrees(lo), math.degrees(hi)],
                "outside_by_deg": abs(math.degrees(value - nearest)),
            })
    return {
        "all_coordinates_finite": all(
            math.isfinite(q[name]) for name in JOINTS),
        "inside_nominal_joint_ranges": not violations,
        "nominal_joint_range_violation_count": len(violations),
        "nominal_joint_range_violations": violations,
        "joint_values_clipped": False,
    }


def freeze_boundary_route_anchor(
        output: Path, authority: GravityAuthority,
        ledger_hashes: dict[str, str], boundary_q: dict[str, float],
        center_offset_deg: float, confirmation_source: str,
        slider_screenshot_sha256: str | None = None) -> dict:
    """Freeze stable boundary B; C is a relative route point, not an anchor."""
    for name in JOINTS:
        value = boundary_q[name]
        lo, hi = authority.model.jnt_range[authority.jid[name]]
        if not math.isfinite(value) or value < lo - 1e-12 or value > hi + 1e-12:
            raise RuntimeError(
                f"BOUNDARY_MODEL_ANCHOR_OUTSIDE_NOMINAL_RANGE:{name}:{value}")
    audit = authority.audit_boundary_center_route(
        boundary_q, center_offset_deg)
    if (not audit["coverage_complete"] or
            audit["sample_count"] != 405 or
            not audit["all_samples_finite"]):
        raise RuntimeError("BOUNDARY_ROUTE_UNCERTAINTY_SWEEP_NOT_405_FINITE_BLOCKED")
    if audit["joint_values_clipped"]:
        raise RuntimeError("BOUNDARY_ROUTE_JOINT_CLIPPING_FORBIDDEN")
    if audit["planned_route_crosses_below_boundary"]:
        raise RuntimeError("PLANNED_ROUTE_CROSSES_UNSAFE_BOUNDARY_BLOCKED")
    if not audit["sign_robust"]:
        raise RuntimeError("BOUNDARY_ROUTE_GRAVITY_SIGN_NOT_ROBUST_BLOCKED")

    center_q = dict(boundary_q)
    center_q["J2"] += math.radians(center_offset_deg)
    boundary_range = pose_nominal_range_audit(authority, boundary_q)
    center_range = pose_nominal_range_audit(authority, center_q)
    if not boundary_range["inside_nominal_joint_ranges"]:
        raise RuntimeError("BOUNDARY_MODEL_ANCHOR_NOMINAL_RANGE_AUDIT_FAILED")
    payload = {
        "schema": "SESSION_LOCAL_GRAVITY_ANCHOR_V1_MODEL",
        "scope": "SESSION_ONLY_NOT_PERMANENT_ZERO",
        "operator_gate": GATE,
        "operator_gate_entry_method": confirmation_source,
        "slider_screenshot_sha256": slider_screenshot_sha256,
        "model": {
            "path": MODEL_REL.as_posix(),
            "sha256": EXPECTED_MODEL_SHA256,
            "mujoco_version": mujoco.__version__,
            "runtime_gravity_m_s2": [0.0, 0.0, -9.81],
        },
        "frozen_ledgers_sha256": ledger_hashes,
        "anchor_role": "STABLE_BOUNDARY_B",
        "model_reference_equation": (
            "q_model[J2]=q_boundary_B_model[J2]+q_relative_from_boundary_B;"
            "NO_WRAP_OR_NOMINAL_RANGE_CLIPPING"
        ),
        "q_anchor_model_rad": boundary_q,
        "q_anchor_model_deg": {
            name: math.degrees(value) for name, value in boundary_q.items()
        },
        "q_test_center_model_rad": center_q,
        "q_test_center_model_deg": {
            name: math.degrees(value) for name, value in center_q.items()
        },
        "route_contract": {
            "boundary_label": "B",
            "test_center_label": "C",
            "boundary_to_center_logical_j2_offset_deg": center_offset_deg,
            "formal_center_relative_route_deg":
                list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
            "full_boundary_relative_route_deg":
                audit["full_planned_route_boundary_relative_deg"],
            "safe_boundary_relative_interval_deg": [0.0, 10.0],
            "center_is_model_anchor": False,
            "boundary_is_model_anchor": True,
            "hardware_center_realization": (
                "REQUIRES_COMMANDED_PLUS_5DEG_STAGE_AND_ENCODER_FEEDBACK_"
                "VERIFICATION_BEFORE_FORMAL_CENTER_RELATIVE_ROUTE"
            ),
        },
        "pose_provenance": {
            "boundary_B": {
                "role": "STABLE_PHYSICAL_START_AND_SESSION_MODEL_ANCHOR",
                "source": confirmation_source,
                "visual_reference_sha256": slider_screenshot_sha256,
                "q_model_rad": boundary_q,
                "q_model_deg": {
                    name: math.degrees(value)
                    for name, value in boundary_q.items()
                },
                "nominal_range_audit": boundary_range,
            },
            "test_center_C": {
                "role": "DERIVED_RELATIVE_COMMAND_POINT_NOT_SESSION_ANCHOR",
                "derivation": (
                    "BOUNDARY_B_MODEL_J2_PLUS_EXACT_LOGICAL_STAGE_OFFSET"
                ),
                "logical_j2_offset_from_boundary_deg": center_offset_deg,
                "physical_realization_status_at_anchor_freeze": (
                    "PENDING_COMMANDED_STAGE_AND_ENCODER_FEEDBACK_VERIFICATION"
                ),
                "q_model_rad": center_q,
                "q_model_deg": {
                    name: math.degrees(value)
                    for name, value in center_q.items()
                },
                "nominal_range_audit": center_range,
            },
        },
        "nominal_range_extrapolation": {
            "present": (
                audit["nominal_joint_range_extrapolation_sample_count"] > 0
            ),
            "route_uncertainty_sample_count":
                audit["nominal_joint_range_extrapolation_sample_count"],
            "route_uncertainty_violation_record_count":
                audit["nominal_joint_range_violation_record_count"],
            "coordinates_clipped": False,
            "policy": audit["nominal_joint_range_policy"],
        },
        "uncertainty_audit": audit,
        "permanent_zero_modified": False,
        "cad_zero": "PENDING",
        "ros_zero": "PENDING",
    }
    temporary = output.with_name(output.name + ".tmp")
    temporary.unlink(missing_ok=True)
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8", newline="\n")
    temporary.replace(output)
    return audit


class AnchorApp:
    def __init__(self, repo: Path, output: Path, authority: GravityAuthority,
                 ledger_hashes: dict[str, str], model_path: Path):
        self.repo = repo
        self.output = output
        self.authority = authority
        self.ledger_hashes = ledger_hashes
        self.model_path = model_path
        self.root = tk.Tk()
        self.root.title("V15.24E — SESSION LOCAL GRAVITY ANCHOR V1")
        self.root.geometry("1420x940")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.renderer = mujoco.Renderer(authority.model, height=300, width=400)
        self.photo: ImageTk.PhotoImage | None = None
        self.closed = False
        self.saved = False
        self.vars: dict[str, tk.DoubleVar] = {}

        title = ttk.Label(
            self.root,
            text=("仅本次实验的临时姿态锚点 — 不是 CAD/ROS/motor zero。\n"
                  "请对照现实机械臂，调整 J2–J6，使关节方向与大致角度可靠一致。"),
            font=("Sans", 14, "bold"), justify="center")
        title.pack(pady=8)

        self.image_label = ttk.Label(self.root)
        self.image_label.pack(padx=8, pady=4)

        slider_frame = ttk.Frame(self.root)
        slider_frame.pack(fill="x", padx=20, pady=4)
        for row, name in enumerate(JOINTS):
            lo, hi = authority.joint_range_deg(name)
            var = tk.DoubleVar(value=0.0)
            self.vars[name] = var
            ttk.Label(slider_frame, text=name, width=5,
                      font=("Sans", 12, "bold")).grid(row=row, column=0)
            scale = ttk.Scale(slider_frame, from_=lo, to=hi, variable=var,
                              orient="horizontal", length=1050,
                              command=lambda _value: self.update_labels())
            scale.grid(row=row, column=1, sticky="ew", padx=8, pady=3)
            label = ttk.Label(slider_frame, width=22, anchor="e")
            label.grid(row=row, column=2)
            setattr(self, f"label_{name}", label)
        slider_frame.columnconfigure(1, weight=1)

        self.gravity_label = ttk.Label(self.root, font=("Sans", 12, "bold"))
        self.gravity_label.pack(pady=4)
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=8)
        ttk.Button(button_frame, text="重置模型零位",
                   command=self.reset).pack(side="left", padx=8)
        ttk.Button(button_frame, text="确认匹配并冻结锚点",
                   command=self.confirm).pack(side="left", padx=8)
        ttk.Button(button_frame, text="关闭（不保存）",
                   command=self.close).pack(side="left", padx=8)
        self.update_labels()
        self.root.after(50, self.render)

    def current_q(self) -> dict[str, float]:
        return {name: math.radians(var.get()) for name, var in self.vars.items()}

    def update_labels(self) -> None:
        for name, var in self.vars.items():
            lo, hi = self.authority.joint_range_deg(name)
            getattr(self, f"label_{name}").configure(
                text=f"{var.get():+8.2f}°   [{lo:.0f}, {hi:.0f}]")
        try:
            torque = self.authority.j2_gravity(self.current_q())
            self.gravity_label.configure(
                text=f"当前模型 J2 保持重力矩 qfrc_bias：{torque:+.6f} N·m")
        except Exception as error:
            self.gravity_label.configure(text=f"模型计算错误：{error}")

    def render(self) -> None:
        if self.closed:
            return
        q = self.current_q()
        self.authority.set_pose(q)
        views: list[np.ndarray] = []
        center = self.authority.model.stat.center.copy()
        distance = float(self.authority.model.stat.extent * 1.8)
        for azimuth, elevation in ((135.0, -20.0), (90.0, 0.0), (0.0, 0.0)):
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = center
            camera.distance = distance
            camera.azimuth = azimuth
            camera.elevation = elevation
            self.renderer.update_scene(self.authority.data, camera=camera)
            views.append(self.renderer.render().copy())
        montage = np.concatenate(views, axis=1)
        image = Image.fromarray(montage)
        self.photo = ImageTk.PhotoImage(image=image)
        self.image_label.configure(image=self.photo)
        self.update_labels()
        self.root.after(120, self.render)

    def reset(self) -> None:
        for var in self.vars.values():
            var.set(0.0)
        self.update_labels()

    def confirm(self) -> None:
        gate = simpledialog.askstring(
            "操作员确认",
            "确认虚拟姿态与现实机械臂的关节方向和大致角度一致后，输入：\n"
            + GATE,
            parent=self.root)
        if gate != GATE:
            messagebox.showerror("未冻结", "确认文本不匹配，锚点未保存。")
            return
        q = self.current_q()
        try:
            audit = freeze_anchor(
                self.output, self.authority, self.ledger_hashes, q,
                "GUI_EXACT_TEXT_ENTRY")
        except Exception as error:
            messagebox.showerror(
                "BLOCKED",
                "锚点不确定度扫掠未完整通过，未保存锚点，禁止硬件转矩。\n\n"
                + str(error))
            return
        self.saved = True
        messagebox.showinfo(
            "锚点已冻结",
            f"GRAVITY_SIGN_ROBUST=YES\n"
            f"J2 gravity envelope: {audit['tau_g_j2_min_nm']:.6f} .. "
            f"{audit['tau_g_j2_max_nm']:.6f} N·m")
        self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.renderer.close()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=OUTPUT_EXACT)
    parser.add_argument("--self-test", action="store_true")
    record_mode = parser.add_mutually_exclusive_group()
    record_mode.add_argument(
        "--record-deg", nargs=5, type=float,
        metavar=("J2", "J3", "J4", "J5", "J6"))
    record_mode.add_argument(
        "--record-boundary-deg", nargs=5, type=float,
        metavar=("J2_B", "J3_B", "J4_B", "J5_B", "J6_B"),
        help=("record stable boundary B as the model anchor; test center C "
              "is derived using --center-offset-deg"))
    parser.add_argument("--center-offset-deg", type=float)
    parser.add_argument("--operator-gate")
    parser.add_argument("--confirmation-source")
    parser.add_argument("--slider-screenshot-sha256")
    args = parser.parse_args()
    repo = args.repo.resolve()
    output = args.output.resolve()
    if output != OUTPUT_EXACT:
        raise RuntimeError("OUTPUT_PATH_NOT_ALLOWED")
    model_path, ledger_hashes = validate_authorities(repo)
    authority = GravityAuthority(model_path)
    if args.self_test:
        anchor = {name: 0.0 for name in JOINTS}
        audit = authority.audit_uncertainty(anchor)
        if (not audit["sign_robust"] or not audit["coverage_complete"] or
                audit["sample_count"] != 405):
            raise RuntimeError("ZERO_POSE_SIGN_ROBUSTNESS_SELF_TEST")
        boundary = dict(anchor)
        boundary["J2"] = float(
            authority.model.jnt_range[authority.jid["J2"]][1])
        boundary_audit = authority.audit_boundary_center_route(
            boundary, BOUNDARY_TO_CENTER_OFFSET_DEG)
        if (not boundary_audit["coverage_complete"] or
                boundary_audit["sample_count"] != 405 or
                not boundary_audit["all_samples_finite"] or
                boundary_audit["joint_values_clipped"] or
                boundary_audit["crosses_below_boundary"] or
                boundary_audit["j2_route_offsets_deg"] !=
                [0.0, 2.5, 5.0, 7.5, 10.0] or
                boundary_audit["full_planned_boundary_relative_route_deg"] !=
                [0.0, 5.0, 10.0, 5.0, 0.0, 5.0, 0.0] or
                boundary_audit[
                    "nominal_joint_range_extrapolation_sample_count"] == 0):
            raise RuntimeError("BOUNDARY_ROUTE_AUDIT_SELF_TEST")
        print("V15_24E_GRAVITY_ANCHOR_SELF_TEST=PASS")
        print("MODEL_HASH_VERIFIED=YES")
        print("MASS_COM_INERTIA_LEDGER_HASHES_VERIFIED=YES")
        print(f"ZERO_POSE_J2_GRAVITY_NM={audit['anchor_tau_g_j2_nm']:.17g}")
        print(f"ZERO_POSE_UNCERTAINTY_MIN_NM={audit['tau_g_j2_min_nm']:.17g}")
        print(f"ZERO_POSE_UNCERTAINTY_MAX_NM={audit['tau_g_j2_max_nm']:.17g}")
        print("BOUNDARY_ROUTE_AUDIT_405_FINITE=YES")
        print("BOUNDARY_ROUTE_JOINT_VALUES_CLIPPED=NO")
        print("BOUNDARY_ROUTE_FULL_RELATIVE_DEG=0,5,10,5,0,5,0")
        print("BOUNDARY_ROUTE_NOMINAL_RANGE_EXTRAPOLATION_RECORDED=YES")
        print("PERMANENT_ZERO_MODIFIED=NO")
        return 0
    if args.record_boundary_deg is not None:
        if args.operator_gate != GATE:
            raise RuntimeError("OPERATOR_GATE_MISMATCH")
        if args.confirmation_source != BOUNDARY_ROUTE_CONFIRMATION_SOURCE:
            raise RuntimeError("BOUNDARY_ROUTE_CONFIRMATION_SOURCE_MISMATCH")
        if args.center_offset_deg is None:
            raise RuntimeError("BOUNDARY_ROUTE_CENTER_OFFSET_REQUIRED")
        output.unlink(missing_ok=True)
        boundary_q = {
            name: math.radians(value)
            for name, value in zip(JOINTS, args.record_boundary_deg)
        }
        audit = freeze_boundary_route_anchor(
            output, authority, ledger_hashes, boundary_q,
            args.center_offset_deg, args.confirmation_source,
            args.slider_screenshot_sha256)
        print("SESSION_LOCAL_GRAVITY_ANCHOR_V1_MODEL=RECORDED")
        print("ANCHOR_ROLE=STABLE_BOUNDARY_B")
        print("OPERATOR_GATE=" + GATE)
        print("CONFIRMATION_SOURCE=" + BOUNDARY_ROUTE_CONFIRMATION_SOURCE)
        print("TEST_CENTER_OFFSET_FROM_BOUNDARY_DEG=5")
        print("FORMAL_CENTER_RELATIVE_ROUTE_DEG=0,5,0,-5,0")
        print("FULL_PLANNED_BOUNDARY_RELATIVE_ROUTE_DEG=0,5,10,5,0,5,0")
        print("GRAVITY_SIGN_ROBUST=YES")
        print(f"ANCHOR_TAU_G_J2_NM={audit['anchor_tau_g_j2_nm']:.17g}")
        print(f"UNCERTAINTY_MIN_NM={audit['tau_g_j2_min_nm']:.17g}")
        print(f"UNCERTAINTY_MAX_NM={audit['tau_g_j2_max_nm']:.17g}")
        print(f"UNCERTAINTY_SAMPLE_COUNT={audit['sample_count']}")
        print(f"NOMINAL_RANGE_EXTRAPOLATION_SAMPLE_COUNT="
              f"{audit['nominal_joint_range_extrapolation_sample_count']}")
        print("JOINT_VALUES_CLIPPED=NO")
        return 0
    if args.record_deg is not None:
        if args.operator_gate != GATE:
            raise RuntimeError("OPERATOR_GATE_MISMATCH")
        if args.confirmation_source != "USER_CHAT_EXPLICIT_CURRENT_SLIDERS_AUTHORIZATION":
            raise RuntimeError("CONFIRMATION_SOURCE_MISMATCH")
        output.unlink(missing_ok=True)
        q = {
            name: math.radians(value)
            for name, value in zip(JOINTS, args.record_deg)
        }
        audit = freeze_anchor(
            output, authority, ledger_hashes, q,
            args.confirmation_source, args.slider_screenshot_sha256)
        print("SESSION_LOCAL_GRAVITY_ANCHOR_V1_MODEL=RECORDED")
        print("OPERATOR_GATE=" + GATE)
        print("GRAVITY_SIGN_ROBUST=YES")
        print(f"ANCHOR_TAU_G_J2_NM={audit['anchor_tau_g_j2_nm']:.17g}")
        print(f"UNCERTAINTY_MIN_NM={audit['tau_g_j2_min_nm']:.17g}")
        print(f"UNCERTAINTY_MAX_NM={audit['tau_g_j2_max_nm']:.17g}")
        print(f"UNCERTAINTY_SAMPLE_COUNT={audit['sample_count']}")
        return 0
    if args.center_offset_deg is not None:
        raise RuntimeError("CENTER_OFFSET_REQUIRES_RECORD_BOUNDARY_DEG")
    # A model anchor is session-local. Starting a new editor invalidates any
    # stale /tmp result so closing/cancelling cannot authorize hardware.
    output.unlink(missing_ok=True)
    output.with_name(output.name + ".tmp").unlink(missing_ok=True)
    app = AnchorApp(repo, output, authority, ledger_hashes, model_path)
    app.run()
    return 0 if app.saved else 2


if __name__ == "__main__":
    raise SystemExit(main())
