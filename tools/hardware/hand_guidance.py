"""Six-axis outer admittance reference generator; no transport or hardware access.

q/dq use radians; tau_external uses joint-side Nm, positive toward +q. The caller
subtracts a separately calibrated, frozen bias before calling update. This core
never estimates bias or changes physical torque/gain limits. A fault returns the
previous goal with zero reference velocity until an explicit reset.
"""
from dataclasses import dataclass
import math
from numbers import Real


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"NONFINITE_OR_INVALID_{name}")
    return float(value)


def _six(values, name):
    values = tuple(values)
    if len(values) != 6:
        raise ValueError(f"SIX_VALUES_REQUIRED_{name}")
    return tuple(_finite(value, name) for value in values)


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class GuidanceProfile:
    # Virtual parameters, not claims about actual joint inertia or motor rating.
    virtual_mass: tuple = (1.5, 3.0, 2.5, 0.6, 0.4, 0.2)
    damping: tuple = (8.0, 16.0, 12.0, 4.0, 3.0, 2.0)
    engage_torque_nm: tuple = (0.20, 0.40, 0.30, 0.12, 0.08, 0.05)
    release_torque_nm: tuple = (0.10, 0.20, 0.15, 0.06, 0.04, 0.025)
    speed_deg_s: float = 3.0
    acceleration_deg_s2: float = 10.0
    target_error_deg: float = 2.0
    settle_velocity_deg_s: float = 0.25
    settle_time_s: float = 0.3
    max_feedback_age_s: float = 0.1
    max_dt_s: float = 0.05
    enabled: tuple = (True,) * 6

    def __post_init__(self):
        for name in ("virtual_mass", "damping", "engage_torque_nm", "release_torque_nm"):
            object.__setattr__(self, name, _six(getattr(self, name), name))
        if any(value <= 0 for value in self.virtual_mass + self.damping):
            raise ValueError("POSITIVE_VIRTUAL_MASS_AND_DAMPING_REQUIRED")
        if any(not 0 <= release < engage for release, engage in zip(self.release_torque_nm, self.engage_torque_nm)):
            raise ValueError("TORQUE_HYSTERESIS_REQUIRES_0_LE_RELEASE_LT_ENGAGE")
        for name in ("speed_deg_s", "acceleration_deg_s2", "target_error_deg", "settle_velocity_deg_s",
                     "settle_time_s", "max_feedback_age_s", "max_dt_s"):
            value = _finite(getattr(self, name), name)
            if value <= 0:
                raise ValueError(f"POSITIVE_{name}_REQUIRED")
            object.__setattr__(self, name, value)
        if self.speed_deg_s > 30:
            raise ValueError("REFERENCE_SPEED_HARD_LIMIT_30_DEG_S")
        enabled = tuple(self.enabled)
        if len(enabled) != 6 or any(type(value) is not bool for value in enabled):
            raise ValueError("SIX_BOOLEAN_ENABLE_FLAGS_REQUIRED")
        object.__setattr__(self, "enabled", enabled)


@dataclass(frozen=True)
class GuidanceOutput:
    q_ref: tuple
    dq_ref: tuple
    state: str
    fault: str | None
    limited: tuple


class HandGuidance:
    def __init__(self, profile: GuidanceProfile, initial_q, now_s=0.0):
        self.profile = profile
        self.reset(initial_q, now_s)

    def reset(self, q, now_s):
        self._q = _six(q, "Q")
        self._time = _finite(now_s, "TIME")
        self._v = (0.0,) * 6
        self._active = (False,) * 6
        self._quiet_since = (None,) * 6
        self._fault = None

    def update(self, now_s, q, dq, tau_external, *, source_time_s):
        if self._fault is not None:
            return GuidanceOutput(self._q, (0.0,) * 6, "fault", self._fault, (False,) * 6)
        try:
            now = _finite(now_s, "TIME")
            source = _finite(source_time_s, "SOURCE_TIME")
            if source < 0:
                raise ValueError("NEGATIVE_SOURCE_TIME")
            dt = now - self._time
            if not 0 < dt <= self.profile.max_dt_s:
                raise ValueError(f"INVALID_DT:{dt:.6g}")
            age = now - source
            if not 0 <= age <= self.profile.max_feedback_age_s:
                raise ValueError(f"{'FUTURE' if age < 0 else 'STALE'}_FEEDBACK:{age:.6g}s")
            q, dq, torque = _six(q, "Q"), _six(dq, "DQ"), _six(tau_external, "TAU_EXTERNAL")
            speed = math.radians(self.profile.speed_deg_s)
            accel = math.radians(self.profile.acceleration_deg_s2)
            lead = math.radians(self.profile.target_error_deg)
            quiet_speed = math.radians(self.profile.settle_velocity_deg_s)
            goals, velocities, contacts, quiet_times, limited = [], [], [], [], []
            for index in range(6):
                previous_q, previous_v = self._q[index], self._v[index]
                lower, upper = q[index] - lead, q[index] + lead
                if not lower - 1e-12 <= previous_q <= upper + 1e-12:
                    raise ValueError(f"J{index + 1}_ACTUAL_OUTSIDE_REFERENCE_ENVELOPE")
                contact = self._active[index]
                contact = abs(torque[index]) > (self.profile.release_torque_nm[index] if contact
                                                else self.profile.engage_torque_nm[index])
                contact = contact and self.profile.enabled[index]
                force = math.copysign(max(0.0, abs(torque[index]) - self.profile.release_torque_nm[index]), torque[index]) if contact else 0.0
                mass, damping = self.profile.virtual_mass[index], self.profile.damping[index]
                requested_v = (previous_v + dt * force / mass) / (1.0 + dt * damping / mass)
                if not math.isfinite(requested_v):
                    raise ValueError(f"J{index + 1}_NONFINITE_ADMITTANCE")
                velocity = _clip(requested_v, max(-speed, previous_v - accel * dt), min(speed, previous_v + accel * dt))
                # Reserve stopping distance inside the measured-position lead
                # bound. Velocity state is the limited output, not a hidden
                # integrator that can wind up behind the position clamp.
                def closing_speed(distance):
                    return max(0.0, math.sqrt((accel * dt) ** 2 + 2 * accel * max(0.0, distance)) - accel * dt)
                velocity = _clip(velocity, -closing_speed(previous_q - lower), closing_speed(upper - previous_q))
                if abs(velocity - previous_v) > accel * dt + 1e-10:
                    raise ValueError(f"J{index + 1}_REFERENCE_ENVELOPE_CANNOT_BRAKE")
                quiet = self._quiet_since[index]
                if not contact and abs(dq[index]) <= quiet_speed and abs(velocity) <= quiet_speed:
                    quiet = now if quiet is None else quiet
                    if now - quiet >= self.profile.settle_time_s and abs(velocity) <= accel * dt:
                        velocity = 0.0
                else:
                    quiet = None
                if not self.profile.enabled[index]:
                    velocity, quiet = 0.0, now
                goals.append(previous_q + velocity * dt)
                velocities.append(velocity)
                contacts.append(contact)
                quiet_times.append(quiet)
                limited.append(abs(velocity - requested_v) > 1e-10)
            self._q, self._v, self._active = tuple(goals), tuple(velocities), tuple(contacts)
            self._quiet_since, self._time = tuple(quiet_times), now
            state = "guiding" if any(contacts) else "settling" if any(self._v) else "holding"
            return GuidanceOutput(self._q, self._v, state, None, tuple(limited))
        except (TypeError, ValueError, OverflowError) as error:
            self._fault = str(error)
            self._v = (0.0,) * 6
            return GuidanceOutput(self._q, self._v, "fault", self._fault, (False,) * 6)

    def pause(self, now_s):
        """Hold the previous reference during an input gap; never recapture q."""
        self._time = _finite(now_s, "TIME")
        self._v = (0.0,) * 6
        self._active = (False,) * 6
        self._quiet_since = (None,) * 6
        return GuidanceOutput(self._q, self._v, "fault" if self._fault else "holding",
                              self._fault, (False,) * 6)
