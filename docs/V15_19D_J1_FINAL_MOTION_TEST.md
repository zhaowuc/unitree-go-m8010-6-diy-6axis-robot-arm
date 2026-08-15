# V15.19D J1 Final Motion Commissioning

## Result

The practical J1 motion sequence completed successfully on 2026-08-15:

- `J1_MOTION_CONTROL = PASS`
- `J1_BIDIRECTIONAL_10DEG = PASS`
- `J1_COMMISSIONING = COMPLETE`
- final feedback mode: `BRAKE` (numeric mode `0`), confirmed by five consecutive valid frames
- J2 was not started

The executed route was `SESSION_LOCAL_ZERO -> +10 deg -> 0 deg -> -10 deg -> 0 deg`. There was no intermediate BRAKE. All four profiles completed naturally before their fixed-position holds.

## Frozen control configuration

| Item | Value |
|---|---:|
| Motor | GO-M8010-6, ID 0 |
| Gear ratio | `6.3299999237060547` |
| Raw-to-ROS sign | `+1` |
| Zero | 0.5 s BRAKE median `SESSION_LOCAL_ZERO` |
| Control mode | native SDK FOC mixed position control |
| `tau` | `0.0` |
| SDK-literal `kp` | `0.50` |
| SDK-literal `kd` | `0.05` |
| Command rate | 100 Hz |
| Velocity limit | `12 deg/s` |
| Acceleration limit | `40 deg/s^2` |
| Profile | discrete rest-to-rest trapezoid, q/dq from the same analytical state |
| Active deadline | 15 s, first FOC command through fifth confirmed BRAKE frame |

The driver clears the complete frozen SDK command object before populating it and validates the real 17-byte GO packet, reserved bit, fields and CRC. Offline golden vectors cover BRAKE, positive and negative q/dq FOC commands. The runner repeats packet, trajectory and safety self-tests before constructing the hardware driver.

`data.dq` was logged but was not used as control authority. Fast and slow velocity protection came from the quantized position-difference estimator.

## Motion result

| Metric | Result |
|---|---:|
| +10 peak | `9.903338492 deg` |
| +10 maximum `abs(qdot_slow)` | `16.867487422 deg/s` |
| +10 profile duration | `1.140078973 s` |
| First return error | `0.781019732 deg` |
| -10 peak | `-9.502416094 deg` |
| -10 maximum `abs(qdot_slow)` | `14.179528043 deg/s` |
| -10 profile duration | `1.139982489 s` |
| Final return error | `0.034712036 deg` |
| Maximum absolute J1 angle | `9.903338492 deg` |
| Maximum `abs(qdot_fast)` | `17.961543512 deg/s` |
| Maximum `abs(qdot_slow)` | `16.867487422 deg/s` |
| Maximum tracking error | `1.268503869 deg` |
| Maximum absolute torque feedback | `0.082031250` |
| Maximum temperature | `30 C` |
| `merror` frames | `0` |
| Invalid frames | `0` |
| Active duration reported by runner | `5.977038378 s` |

The actual joint speed exceeded the 12 deg/s reference because of plant response, but remained in the explicitly normal `0..20 deg/s` band. It did not enter any warning or brake condition.

All target and hold requirements passed:

- +10 target band `[9, 11] deg`, hold `0.309895615 s`
- first-zero band `abs(q) < 1 deg`, hold `0.200088713 s`
- -10 target band `[-11, -9] deg`, hold `0.300053291 s`
- final-zero band `abs(q) < 1 deg`, hold `0.200008566 s`

## Safety and operator observation

No software warning or hard gate fired. Maximum position, speed, tracking error, torque feedback and temperature stayed below their respective warning thresholds. Communication was 659/659 valid feedback frames with monotonically increasing sequence `0..658`, zero `merror`, and zero invalid frames.

The operator supplied the single immediate `READY_FOR_J1_FINAL_MOTION_TEST` confirmation, committed to continuous observation and immediate physical cutoff/reporting, and reported no abnormality during the uninterrupted run. Therefore:

- `POWER_SUPPLY_LIMITING = NO`
- collision observed: `NO`
- vibration observed: `NO`
- abnormal noise observed: `NO`
- manual power-off required: `NO`

## Evidence

The continuous CSV contains 662 physical lines: one header, 659 feedback frames and two event rows. Its SHA-256 is:

```text
dc738511024798b4d06040dcc13676c20822c600d13b35e51ea3f78b0751d0d9
```

The original remains at `/tmp/v15_19d_j1_final_20260815T110442Z.csv`. The persistent raw copy is outside Git at:

```text
/home/car/go-m8010-robot-arm-preservation/v15_19_j1/
20260815T110442Z_head-5e8ba916_j1-final-run/raw/
v15_19d_j1_final_20260815T110442Z.csv
```

Independent CSV verification confirmed:

- exact state order and stage counts
- four 114-interval profiles
- q/dq trapezoidal identity residual `1.75e-15 deg`
- motor q/dq mapping residuals below `9e-16 rad`
- mean/median feedback period `10.0101/9.9994 ms`
- five final valid BRAKE frames and final mode `0`

The raw CSV is intentionally not committed to Git.

## Implementation

- `tools/hardware/go_m8010_driver.cpp/.hpp`: frozen SDK packet checks, joint command mapping, telemetry and velocity estimators
- `tools/hardware/j1_final_trajectory.cpp/.hpp`: integer-interval rest-to-rest trapezoid and property tests
- `tools/hardware/v15_19d_j1_final_motion.cpp`: one-confirmation commissioning runner, safety state machine and reliable logging
- `hardware/config/go_m8010_j1.yaml`: frozen post-run J1 control and safety configuration record; the current runner uses compile-time constants and does not parse this file
- `hardware/v15_19d/j1_final_motion_test.json`: machine-readable result and evidence inventory

## Remaining nonblocking scope

- J1 absolute CAD zero remains `PENDING_PHYSICAL_ALIGNMENT`; this run intentionally used `SESSION_LOCAL_ZERO`.
- SDK `data.dq` semantics remain unresolved and are not used as motion-control authority.
- Full-range raw wrap/multi-turn behavior remains untested.
- J2 remains untouched.
