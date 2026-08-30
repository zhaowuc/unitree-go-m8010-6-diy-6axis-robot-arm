"""Pure, latched thermal policy for all seven physical motors.

The manager separates transport continuity from actuation safety.  A thermal
trip never becomes permission to stop publishing feedback, and cooling never
becomes permission to resume motion without a new activation epoch plus an
explicit operator confirmation.
"""

from __future__ import annotations

import math
import statistics
import hashlib
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Deque, Mapping, Optional

import yaml


MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
DOMAIN_MOTORS = {
    "J1": ("J1",),
    "J2": ("J2A", "J2B"),
    "J345": ("J3", "J4", "J5"),
    "J6": ("J6",),
}
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)


class ThermalState(str, Enum):
    OFFLINE = "OFFLINE"
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    DERATING = "DERATING"
    THERMAL_STOP = "THERMAL_STOP"
    COOLDOWN = "COOLDOWN"
    WAIT_OPERATOR_CONFIRM = "WAIT_OPERATOR_CONFIRM"


def thermal_state_from_controller_metadata(
    temperature_c: Optional[float],
    *,
    fault_latched: bool,
    cooldown_ready: bool,
    limits: "ThermalLimits",
) -> ThermalState:
    """Normalize worker latch metadata into the shared seven-motor enum."""

    if type(fault_latched) is not bool or type(cooldown_ready) is not bool:
        raise TypeError("thermal controller flags must be bool")
    if not isinstance(limits, ThermalLimits):
        raise TypeError("limits must be ThermalLimits")
    if temperature_c is None:
        return ThermalState.OFFLINE
    temperature = float(temperature_c)
    if not math.isfinite(temperature) or temperature < 0.0:
        return ThermalState.OFFLINE
    if fault_latched:
        if cooldown_ready and temperature < limits.rearm_below_c:
            return ThermalState.WAIT_OPERATOR_CONFIRM
        if temperature < limits.rearm_below_c:
            return ThermalState.COOLDOWN
        return ThermalState.THERMAL_STOP
    if temperature >= limits.thermal_stop_c:
        # Raw temperature is an immediate safety authority even if a delayed
        # metadata sample has not exposed the latch bit yet.
        return ThermalState.THERMAL_STOP
    if temperature >= limits.derating_start_c:
        return ThermalState.DERATING
    if temperature >= limits.normal_below_c:
        return ThermalState.WARNING
    return ThermalState.NORMAL


@dataclass(frozen=True)
class ThermalLimits:
    normal_below_c: float
    warning_below_c: float
    derating_start_c: float
    thermal_stop_c: float
    rearm_below_c: float
    cooldown_seconds: float
    slope_window_seconds: float

    def __post_init__(self) -> None:
        values = (
            self.normal_below_c,
            self.warning_below_c,
            self.derating_start_c,
            self.thermal_stop_c,
            self.rearm_below_c,
            self.cooldown_seconds,
            self.slope_window_seconds,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("thermal limits must be finite")
        if not (
            self.normal_below_c
            < self.warning_below_c
            < self.derating_start_c
            < self.thermal_stop_c
        ):
            raise ValueError("thermal display/derating thresholds must increase")
        if not self.normal_below_c < self.rearm_below_c < self.thermal_stop_c:
            raise ValueError("rearm threshold must be below thermal stop")
        if self.cooldown_seconds <= 0.0 or self.slope_window_seconds <= 0.0:
            raise ValueError("thermal time windows must be positive")

    @classmethod
    def from_mapping(cls, value: Mapping) -> "ThermalLimits":
        required = {
            "normal_below_c",
            "warning_below_c",
            "derating_start_c",
            "thermal_stop_c",
            "rearm_below_c",
            "cooldown_seconds",
            "slope_window_seconds",
        }
        if not isinstance(value, Mapping) or not required.issubset(value):
            raise ValueError("thermal limits mapping is incomplete")
        if any(type(value[name]) not in {int, float} for name in required):
            raise ValueError("thermal limits mapping has non-numeric values")
        return cls(**{name: float(value[name]) for name in required})

    def as_mapping(self) -> dict[str, float]:
        return {
            "normal_below_c": self.normal_below_c,
            "warning_below_c": self.warning_below_c,
            "derating_start_c": self.derating_start_c,
            "thermal_stop_c": self.thermal_stop_c,
            "rearm_below_c": self.rearm_below_c,
            "cooldown_seconds": self.cooldown_seconds,
            "slope_window_seconds": self.slope_window_seconds,
        }


def load_thermal_limits(path: Path) -> ThermalLimits:
    """Load the one frozen thermal authority shared by every runtime layer."""

    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != THERMAL_CONFIG_SHA256:
        raise ValueError("thermal config SHA256 mismatch")
    try:
        document = yaml.safe_load(raw.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValueError("thermal config is not valid UTF-8 YAML") from exc
    if (
        not isinstance(document, Mapping)
        or document.get("schema") != "go-m8010-thermal-limits/1.0"
    ):
        raise ValueError("thermal config schema mismatch")
    return ThermalLimits.from_mapping(document)


@dataclass(frozen=True)
class ThermalSnapshot:
    state: ThermalState
    raw_temperature_c: Optional[float]
    median_temperature_c: Optional[float]
    slope_c_per_min: Optional[float]
    fault_latched: bool
    motion_allowed: bool
    derating_factor: float
    trip_activation_epoch: Optional[int]
    minimum_rearm_epoch: Optional[int]
    cooldown_remaining_s: Optional[float]


class MotorThermalManager:
    def __init__(self, limits: ThermalLimits) -> None:
        self.limits = limits
        self._samples: Deque[tuple[float, float]] = deque()
        self.state = ThermalState.OFFLINE
        self.raw_temperature_c: Optional[float] = None
        self.fault_latched = False
        self.trip_activation_epoch: Optional[int] = None
        self.minimum_rearm_epoch: Optional[int] = None
        self._cooldown_started_at: Optional[float] = None

    def _append_sample(self, now_s: float, temperature_c: float) -> None:
        self._samples.append((now_s, temperature_c))
        cutoff = now_s - self.limits.slope_window_seconds
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def _median(self) -> Optional[float]:
        if not self._samples:
            return None
        return float(statistics.median(value for _stamp, value in self._samples))

    def _slope(self) -> Optional[float]:
        if len(self._samples) < 2:
            return None
        origin = self._samples[0][0]
        x = [stamp - origin for stamp, _value in self._samples]
        y = [value for _stamp, value in self._samples]
        mean_x = statistics.fmean(x)
        mean_y = statistics.fmean(y)
        denominator = sum((item - mean_x) ** 2 for item in x)
        if denominator <= 1.0e-12:
            return None
        slope_c_per_s = sum(
            (item_x - mean_x) * (item_y - mean_y)
            for item_x, item_y in zip(x, y)
        ) / denominator
        return slope_c_per_s * 60.0

    def _classify_unlatched(self, temperature_c: float) -> ThermalState:
        if temperature_c < self.limits.normal_below_c:
            return ThermalState.NORMAL
        if temperature_c < self.limits.derating_start_c:
            return ThermalState.WARNING
        if temperature_c < self.limits.thermal_stop_c:
            return ThermalState.DERATING
        return ThermalState.THERMAL_STOP

    def _derating_factor(self) -> float:
        if self.state in {
            ThermalState.OFFLINE,
            ThermalState.THERMAL_STOP,
            ThermalState.COOLDOWN,
            ThermalState.WAIT_OPERATOR_CONFIRM,
        }:
            return 0.0
        # Safety derating follows the newest raw sample.  The median is kept
        # for operator display/trending only: smoothing must never hide an
        # immediately hot motor or delay a reduction in commanded effort.
        temperature = self.raw_temperature_c
        if temperature is None or temperature < self.limits.derating_start_c:
            return 1.0
        span = self.limits.thermal_stop_c - self.limits.derating_start_c
        return max(0.1, min(1.0, (self.limits.thermal_stop_c - temperature) / span))

    def force_trip(self, activation_epoch: int) -> None:
        if type(activation_epoch) is not int or activation_epoch < 0:
            raise ValueError("activation_epoch must be a non-negative integer")
        if not self.fault_latched:
            self.trip_activation_epoch = activation_epoch
            self.minimum_rearm_epoch = activation_epoch + 1
        self.fault_latched = True
        self.state = ThermalState.THERMAL_STOP
        self._cooldown_started_at = None

    def observe(
        self,
        *,
        now_s: float,
        temperature_c: Optional[float],
        continuity_valid: bool,
        activation_epoch: int,
    ) -> ThermalSnapshot:
        if not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("now_s must be finite and non-negative")
        if type(continuity_valid) is not bool:
            raise TypeError("continuity_valid must be bool")
        if type(activation_epoch) is not int or activation_epoch < 0:
            raise ValueError("activation_epoch must be a non-negative integer")
        if not continuity_valid or temperature_c is None:
            self.state = ThermalState.OFFLINE
            return self.snapshot(now_s)
        temperature = float(temperature_c)
        if not math.isfinite(temperature) or temperature < 0.0:
            self.state = ThermalState.OFFLINE
            return self.snapshot(now_s)
        self.raw_temperature_c = temperature
        self._append_sample(now_s, temperature)

        if temperature >= self.limits.thermal_stop_c:
            self.force_trip(activation_epoch)
            return self.snapshot(now_s)

        if self.fault_latched:
            if temperature < self.limits.rearm_below_c:
                if self._cooldown_started_at is None:
                    self._cooldown_started_at = now_s
                if now_s - self._cooldown_started_at >= self.limits.cooldown_seconds:
                    self.state = ThermalState.WAIT_OPERATOR_CONFIRM
                else:
                    self.state = ThermalState.COOLDOWN
            else:
                self._cooldown_started_at = None
                self.state = ThermalState.THERMAL_STOP
        else:
            self.state = self._classify_unlatched(temperature)
        return self.snapshot(now_s)

    def confirm_rearm(self, *, now_s: float, activation_epoch: int) -> bool:
        if (
            self.state is not ThermalState.WAIT_OPERATOR_CONFIRM
            or self.raw_temperature_c is None
            or self.raw_temperature_c >= self.limits.rearm_below_c
            or self.minimum_rearm_epoch is None
            or type(activation_epoch) is not int
            or activation_epoch < self.minimum_rearm_epoch
        ):
            return False
        self.fault_latched = False
        self.trip_activation_epoch = None
        self.minimum_rearm_epoch = None
        self._cooldown_started_at = None
        self.state = self._classify_unlatched(self.raw_temperature_c)
        return True

    def snapshot(self, now_s: float) -> ThermalSnapshot:
        remaining = None
        if self.fault_latched and self._cooldown_started_at is not None:
            remaining = max(
                0.0,
                self.limits.cooldown_seconds - (now_s - self._cooldown_started_at),
            )
        motion_allowed = bool(
            not self.fault_latched
            and self.state not in {ThermalState.OFFLINE, ThermalState.THERMAL_STOP}
        )
        return ThermalSnapshot(
            state=self.state,
            raw_temperature_c=self.raw_temperature_c,
            median_temperature_c=self._median(),
            slope_c_per_min=self._slope(),
            fault_latched=self.fault_latched,
            motion_allowed=motion_allowed,
            derating_factor=self._derating_factor(),
            trip_activation_epoch=self.trip_activation_epoch,
            minimum_rearm_epoch=self.minimum_rearm_epoch,
            cooldown_remaining_s=remaining,
        )


class ThermalFleetManager:
    """Couple domain trips while preserving a per-motor temperature record."""

    def __init__(self, limits: ThermalLimits) -> None:
        self.motors = {
            name: MotorThermalManager(limits) for name in MOTOR_NAMES
        }

    def _domain_for_motor(self, motor_name: str) -> str:
        for domain, motors in DOMAIN_MOTORS.items():
            if motor_name in motors:
                return domain
        raise KeyError(motor_name)

    def observe_motor(self, motor_name: str, **kwargs) -> ThermalSnapshot:
        if motor_name not in self.motors:
            raise KeyError(motor_name)
        snapshot = self.motors[motor_name].observe(**kwargs)
        # Propagate an actual over-temperature sample to every physical motor
        # in the coupled domain.  Merely observing an already-latched motor
        # during cooldown must not restart its peers' cooldown timers.
        if (
            snapshot.raw_temperature_c is not None
            and snapshot.raw_temperature_c
            >= self.motors[motor_name].limits.thermal_stop_c
        ):
            domain = self._domain_for_motor(motor_name)
            activation_epoch = snapshot.trip_activation_epoch
            if activation_epoch is None:
                activation_epoch = int(kwargs["activation_epoch"])
            for peer in DOMAIN_MOTORS[domain]:
                self.motors[peer].force_trip(activation_epoch)
        return self.motors[motor_name].snapshot(float(kwargs["now_s"]))

    def confirm_domain_rearm(
        self, domain: str, *, now_s: float, activation_epoch: int,
    ) -> bool:
        names = DOMAIN_MOTORS.get(domain)
        if names is None:
            raise KeyError(domain)
        if not all(
            self.motors[name].state is ThermalState.WAIT_OPERATOR_CONFIRM
            for name in names
        ):
            return False
        return all(
            self.motors[name].confirm_rearm(
                now_s=now_s, activation_epoch=activation_epoch,
            )
            for name in names
        )
