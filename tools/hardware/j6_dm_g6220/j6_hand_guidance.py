"""Pure, opt-in J6 POS_VEL reference admission. No CAN or torque commands."""
from dataclasses import dataclass, replace
import math

COMMAND_SCHEMA = "go-m8010-gui-command/1.5"
GUIDANCE_SCHEMA = "go-m8010-hand-guidance-reference/1.0"
SPEED = math.radians(30.0)
EXCURSION = math.radians(10.0)
REFERENCE_ERROR = math.radians(2.0)
DEADMAN_NS = 600_000_000_000


class GuidanceReferenceRejected(ValueError):
    """Discard an out-of-bounds candidate without committing or renewing it."""


def is_guidance(command):
    return isinstance(command, dict) and command.get("schema") == COMMAND_SCHEMA


def guidance_authority_identity(command):
    authority = command.get("gravity_authority", {})
    return tuple(authority.get(key) for key in ("source_instance_id", "session_id", "state_instance_id",
        "empirical_envelope_id", "empirical_envelope_sha256", "anchor_sha256"))


def return_only_authority(command):
    authority = command.get("gravity_authority", {})
    return (authority.get("empirical_allowed_teach_joints") == ["J1", "J2", "J3", "J4", "J5", "J6"]
            and authority.get("empirical_maximum_teach_excursion_deg") == 10.0
            and authority.get("empirical_maximum_teach_seconds") == 600.0
            and authority.get("empirical_maximum_teach_velocity_deg_s") == 30.0
            and authority.get("empirical_position_validation_authorized") is True
            and authority.get("empirical_assisted_teach_authorized") is False)


def guidance_feedback_velocity_is_safe(value):
    return type(value) in (int, float) and math.isfinite(value) and abs(value) <= SPEED


def six(value, label):
    if not isinstance(value, list) or len(value) != 6 or any(
            type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError("J6_GUIDANCE_" + label + "_INVALID")
    return tuple(float(x) for x in value)


def validate_guidance_shape(command):
    if not is_guidance(command) or command.get("mode") == "brake":
        return
    g = command.get("hand_guidance")
    fields = {"schema", "origin_rad", "velocity_rad_s", "maximum_velocity_deg_s",
              "maximum_excursion_deg", "maximum_reference_error_deg", "freeze_reference"}
    if not isinstance(g, dict) or set(g) != fields or g["schema"] != GUIDANCE_SCHEMA:
        raise ValueError("J6_GUIDANCE_PROFILE_INVALID")
    for name, expected in (("maximum_velocity_deg_s", 30.0),
                           ("maximum_excursion_deg", 10.0), ("maximum_reference_error_deg", 2.0)):
        if type(g[name]) not in (int, float) or g[name] != expected:
            raise ValueError("J6_GUIDANCE_UNITS_OR_LIMIT_INVALID")
    six(g["origin_rad"], "ORIGIN")
    velocity = six(g["velocity_rad_s"], "VELOCITY")
    if type(g["freeze_reference"]) is not bool:
        raise ValueError("J6_GUIDANCE_FREEZE_INVALID")
    if g["freeze_reference"] and any(velocity):
        raise ValueError("J6_GUIDANCE_FREEZE_VELOCITY_NONZERO")
    if any(abs(v) > SPEED + 1e-12 for v in velocity):
        raise GuidanceReferenceRejected("J6_GUIDANCE_REFERENCE_SPEED_LIMIT")
    if command["active_joint_mask"] != [True] * 6:
        raise ValueError("J6_GUIDANCE_ACTIVE_MASK_INVALID")
    if command["mode"] == "teach" and not any(command["moving_joint_mask"]):
        raise ValueError("J6_GUIDANCE_MOVING_MASK_EMPTY")
    if command["mode"] == "hold" and any(velocity):
        raise ValueError("J6_GUIDANCE_HOLD_VELOCITY_NONZERO")
    if six(command.get("feedforward_nm"), "FEEDFORWARD")[5] != 0.0:
        raise ValueError("J6_GUIDANCE_POSVEL_HAS_NO_TORQUE_COMMAND")


@dataclass(frozen=True)
class GuidanceState:
    active: bool = False
    paused: bool = False
    paused_reason: str = ""
    origin: tuple = ()
    moving_mask: tuple = ()
    targets: tuple = ()
    velocity: tuple = ()
    started_ns: int = 0
    source_ns: int = 0
    press_epoch: int = 0
    accepted_epoch: int = 0
    source_instance_id: str = ""
    authority_binding: tuple = ()
    kp: tuple = ()
    kd: tuple = ()


def advance_deadman(state, now_ns):
    if state is not None and state.active and not state.paused and now_ns - state.started_ns >= DEADMAN_NS:
        return replace(state, paused=True, paused_reason="DEADMAN_TIMEOUT", velocity=(0.0,) * 6)
    return state


def stage_guidance_candidate(candidate, state, context, previous, now_ns):
    """Return candidate/state to commit only after all other admission succeeds."""
    state = advance_deadman(state, now_ns)
    if candidate.get("mode") in {"brake", "drag"} or not candidate.get("active_joint_mask", [False] * 6)[5]:
        return candidate, None
    source_id = candidate.get("source_instance_id")
    binding = guidance_authority_identity(candidate)
    kp, kd = tuple(candidate.get("kp", ())), tuple(candidate.get("kd", ()))
    if state is not None and state.targets and (
            source_id != state.source_instance_id or binding != state.authority_binding
            or kp != state.kp or kd != state.kd):
        raise ValueError("J6_GUIDANCE_SESSION_IDENTITY_OR_GAINS_CHANGED")
    return_only = return_only_authority(candidate)
    if not is_guidance(candidate):
        if state is not None and state.active:
            raise ValueError("J6_GUIDANCE_EXIT_REQUIRES_V15_HOLD_ACK")
        if return_only:
            if candidate.get("mode") == "position":
                trajectory = candidate.get("trajectory", {})
                moving = [i for i, flag in enumerate(candidate.get("moving_joint_mask", ())) if flag]
                if state is None or not state.origin or candidate.get("schema") != "go-m8010-gui-command/1.3" or len(moving) != 1:
                    raise GuidanceReferenceRejected("J6_GUIDANCE_RETURN_ORIGIN_OR_TRAJECTORY_MISSING")
                j = moving[0]
                start = six(trajectory.get("start_rad"), "RETURN_START")[j]
                target = six(trajectory.get("target_rad"), "RETURN_TARGET")[j]
                if not min(start, state.origin[j]) - 1e-12 <= target <= max(start, state.origin[j]) + 1e-12:
                    raise GuidanceReferenceRejected("J6_GUIDANCE_RETURN_MOVES_AWAY_OR_CROSSES_ORIGIN")
            elif candidate.get("mode") == "hold":
                held = state.targets if state is not None else tuple((previous or {}).get("targets_rad", ()))
                prior_same = bool(previous and previous.get("mode") == "hold"
                    and source_id == previous.get("source_instance_id")
                    and binding == guidance_authority_identity(previous)
                    and kp == tuple(previous.get("kp", ())) and kd == tuple(previous.get("kd", ())))
                if (len(held) != 6 or six(candidate.get("targets_rad"), "TARGET")[5] != held[5]
                        or (state is None and not prior_same)):
                    raise GuidanceReferenceRejected("J6_GUIDANCE_RETURN_ONLY_REQUIRES_EXISTING_HOLD")
            else:
                raise GuidanceReferenceRejected("J6_GUIDANCE_RETURN_ONLY")
        return candidate, (replace(state, targets=six(candidate["targets_rad"], "TARGET"),
            accepted_epoch=candidate["activation_epoch"], source_ns=candidate["source_monotonic_ns"])
            if state is not None and state.origin else None)
    validate_guidance_shape(candidate)
    if not context or context.get("torque_qualified") is not True:
        raise ValueError("J6_GUIDANCE_TORQUE_UNITS_NOT_QUALIFIED")
    if context.get("healthy_foc") is not True:
        raise ValueError("J6_GUIDANCE_HEALTHY_POSITION_HOLD_REQUIRED")
    actual = context.get("actual_rad")
    if type(actual) not in (int, float) or not math.isfinite(actual):
        raise ValueError("J6_GUIDANCE_ACTUAL_INVALID")
    g = candidate["hand_guidance"]
    origin = six(g["origin_rad"], "ORIGIN")
    targets = six(candidate["targets_rad"], "TARGET")
    velocity = six(g["velocity_rad_s"], "VELOCITY")
    epoch, source = candidate["activation_epoch"], candidate["source_monotonic_ns"]
    if return_only and candidate["mode"] == "teach" and not (
            state is not None and state.active and epoch == state.press_epoch
            and g["freeze_reference"] is True and not any(velocity)):
        raise GuidanceReferenceRejected("J6_GUIDANCE_RETURN_ONLY")
    if return_only and candidate["mode"] == "hold" and (state is None or not state.origin):
        if not guidance_feedback_velocity_is_safe(context.get("actual_velocity_rad_s")):
            raise ValueError("J6_GUIDANCE_FIXED_HOLD_FEEDBACK_VELOCITY_INVALID")
        if (not previous or previous.get("mode") != "hold"
                or epoch < previous["activation_epoch"]
                or (epoch == previous["activation_epoch"] and not is_guidance(previous))
                or targets[5] != six(previous.get("targets_rad"), "PREVIOUS_HOLD_TARGET")[5]
                or any(velocity) or source_id != previous.get("source_instance_id")
                or binding != guidance_authority_identity(previous)
                or kp != tuple(previous.get("kp", ())) or kd != tuple(previous.get("kd", ()))):
            raise GuidanceReferenceRejected("J6_GUIDANCE_RETURN_ONLY_REQUIRES_EXISTING_HOLD")
        return candidate, GuidanceState(targets=targets, velocity=velocity, source_ns=source,
            accepted_epoch=epoch, source_instance_id=source_id, authority_binding=binding, kp=kp, kd=kd)
    if candidate["mode"] == "hold":
        if state is not None and state.active:
            if epoch <= state.press_epoch or targets[5] != state.targets[5] or origin != state.origin:
                raise ValueError("J6_GUIDANCE_HOLD_ACK_MISMATCH")
        elif state is not None and state.origin:
            if targets[5] != state.targets[5] or origin != state.origin:
                raise GuidanceReferenceRejected("J6_GUIDANCE_HOLD_TARGET_OR_SESSION_ORIGIN_CHANGED")
        else:
            if (not previous or previous.get("mode") != "hold"
                    or epoch <= previous["activation_epoch"]
                    or targets[5] != six(previous["targets_rad"], "PREVIOUS_HOLD_TARGET")[5]):
                raise GuidanceReferenceRejected("J6_GUIDANCE_INITIAL_HOLD_REQUIRES_PRIOR_EXACT_HOLD")
            if origin[5] != targets[5]:
                raise GuidanceReferenceRejected("J6_GUIDANCE_FIRST_ORIGIN_REQUIRES_PRIOR_HOLD_TARGET")
            # This abort retains an already owned target; it captures no new
            # pose even if a hand has displaced the actual angle beyond 2 deg.
            if not guidance_feedback_velocity_is_safe(context.get("actual_velocity_rad_s")):
                raise ValueError("J6_GUIDANCE_FIXED_HOLD_FEEDBACK_VELOCITY_INVALID")
        return candidate, GuidanceState(origin=origin, targets=targets, velocity=velocity,
            source_ns=source, accepted_epoch=epoch, source_instance_id=source_id,
            authority_binding=binding, kp=kp, kd=kd)
    if state is not None and state.active:
        if epoch != state.press_epoch or origin != state.origin or tuple(candidate["moving_joint_mask"]) != state.moving_mask:
            raise ValueError("J6_GUIDANCE_PRESS_IDENTITY_CHANGED")
        if g["freeze_reference"] and not state.paused:
            state = replace(state, paused=True, paused_reason="GUI_RELEASE", velocity=(0.0,) * 6)
        if state.paused:
            normalized = dict(candidate, targets_rad=list(state.targets), _guidance_paused=True)
            normalized["hand_guidance"] = dict(g, velocity_rad_s=[0.0] * 6)
            return normalized, replace(state, source_ns=source)
        dt = (source - state.source_ns) * 1e-9
        if dt <= 0.0 or abs(targets[5] - state.targets[5]) > SPEED * dt + 1e-9:
            raise GuidanceReferenceRejected("J6_GUIDANCE_REFERENCE_STEP_LIMIT")
    else:
        if g["freeze_reference"]:
            raise ValueError("J6_GUIDANCE_FIRST_REFERENCE_CANNOT_FREEZE")
        if not previous or previous.get("mode") != "hold" or epoch <= previous["activation_epoch"]:
            raise ValueError("J6_GUIDANCE_PREVIOUS_HOLD_NEW_EPOCH_REQUIRED")
        if state is not None and state.origin and origin != state.origin:
            raise GuidanceReferenceRejected("J6_GUIDANCE_SESSION_ORIGIN_CHANGED")
        if (state is None or not state.origin) and origin[5] != six(previous["targets_rad"], "PREVIOUS_HOLD_TARGET")[5]:
            raise GuidanceReferenceRejected("J6_GUIDANCE_FIRST_ORIGIN_REQUIRES_PRIOR_HOLD_TARGET")
        # origin remains the session range anchor. A new press starts from
        # the last accepted HOLD target, including after earlier movement.
        if targets[5] != six(previous["targets_rad"], "PREVIOUS_HOLD_TARGET")[5] or any(velocity):
            raise GuidanceReferenceRejected("J6_GUIDANCE_ENTRY_REQUIRES_EXACT_HOLD_AND_ZERO_VELOCITY")
    if abs(targets[5] - origin[5]) > EXCURSION + 1e-12:
        raise GuidanceReferenceRejected("J6_GUIDANCE_REFERENCE_EXCURSION_LIMIT")
    if abs(targets[5] - actual) > REFERENCE_ERROR + 1e-12:
        retaining_fixed_target = (state is not None and state.active
                                  and targets[5] == state.targets[5] and velocity[5] == 0.0)
        if not retaining_fixed_target:
            raise GuidanceReferenceRejected("J6_GUIDANCE_REFERENCE_ACTUAL_ERROR_LIMIT")
        if not guidance_feedback_velocity_is_safe(context.get("actual_velocity_rad_s")):
            raise ValueError("J6_GUIDANCE_FIXED_HOLD_FEEDBACK_VELOCITY_INVALID")
    if not candidate["moving_joint_mask"][5]:
        fixed = state.targets[5] if state is not None and state.active else previous["targets_rad"][5]
        if targets[5] != fixed or velocity[5] != 0.0:
            raise GuidanceReferenceRejected("J6_GUIDANCE_NONMOVING_TARGET_CHANGED")
    if state is None or not state.active:
        state = GuidanceState(active=True, origin=origin, moving_mask=tuple(candidate["moving_joint_mask"]),
            started_ns=now_ns, press_epoch=epoch, accepted_epoch=epoch,
            source_instance_id=source_id, authority_binding=binding, kp=kp, kd=kd)
    return candidate, replace(state, targets=targets, velocity=velocity, source_ns=source)


def runtime_guidance_command(command, state, now_ns):
    state = advance_deadman(state, now_ns)
    if is_guidance(command) and state is not None and state.active and state.paused:
        command = dict(command, targets_rad=list(state.targets), _guidance_paused=True)
        command["hand_guidance"] = dict(command["hand_guidance"], velocity_rad_s=[0.0] * 6)
    return command, state
