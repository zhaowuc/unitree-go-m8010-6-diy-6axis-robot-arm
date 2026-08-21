# V15.24A-FT — J2 Big-Arm-Installed Mechanical Coupling Gate

## Result

`J2_BIG_ARM_MECHANICAL_COUPLING = PASS`

The big arm was installed while the forearm remained removed. J2A and J2B completed dual HOLD and the commanded `0 → +5° → 0 → -5° → 0` route without motor error, invalid feedback, sync-limit violation, abnormal sound, vibration, or motor fighting.

Partial-load trajectory tracking did not meet every position threshold. This does not invalidate the mechanical-coupling result:

- `J2_BIG_ARM_PARTIAL_LOAD_MOTION = TRACKING_NOT_ACCEPTED`
- `COMMON_MODE_CONTROL_TUNING_PENDING = YES`
- `READY_FOR_FOREARM_INSTALLATION = YES`
- `J2_LOCAL_MOTION_AUTHORITY = NOT_GRANTED_FOR_FINAL_ASSEMBLY`

## Frozen hardware authority

- J2A: ID 0, `raw_to_ros_sign = -1`
- J2B: ID 1, `raw_to_ros_sign = +1`
- Sign relation: opposite
- Kp: 0.60 on both motors
- Kd: 0.05 on both motors
- Tff: 0
- Route speed: 10 deg/s
- Route acceleration: 30 deg/s²
- Control rate: 100 Hz

No ID, motor zero, READY_POSE, simulation authority, or J1/J3/J4/J5/J6 authority was modified.

## Run history and operator override

The original contract allowed one route execution. Run 1 was completed under that rule. The operator then explicitly overrode the no-retry clause and ordered a second identical test. An intermediate invocation without the required gated arguments returned `PHASE_NOT_ALLOWED` and caused no motor motion. Run 2 was then executed with the same controller and safety limits. Both raw CSVs are preserved; Run 2 is the authority run for the final reported metrics.

| Metric | Run 1 | Run 2 operator override |
|---|---:|---:|
| A0 raw (rad) | 12.029122352600098 | 11.869587898254395 |
| B0 raw (rad) | -4.216347694396973 | -4.046267509460449 |
| HOLD maximum e_sync | 0.006940° | 0.012150° |
| +5 logical actual | 4.866626° | 5.012851° |
| +5 logical error | 0.133374° | 0.012851° |
| +5 e_sync | 0.135372° | 0.123230° |
| First-center error | 1.234881° | 1.231410° |
| -5 logical actual | -3.762780° | -4.660086° |
| -5 logical error | 1.237220° | 0.339914° |
| -5 e_sync | 0.067685° | 0.116288° |
| Final-center error | 0.344516° | 0.217815° |
| Maximum motion e_sync | 0.227362° | 0.177032° |
| Partial-load tracking | FAIL | FAIL |
| Mechanical coupling numeric | PASS | PASS |
| Final dual BRAKE | PASS | PASS |

Run 2 failed only the first-center tracking threshold (`1.231410° > 0.75°`). Its +5, -5, and final-center position results passed. Every endpoint sync error stayed below 0.3°, maximum motion sync stayed below 0.7°, and the 1.0° hard-abort limit was never reached.

Both runs contain 715 data frames. Both have zero J2A/J2B `merror` frames and zero invalid feedback frames.

## Operator and final mechanical state

The operator reported the mechanical observation as `SAFE` and confirmed J2 24 V power off after the final BRAKE. The forearm was then reassembled with the frozen J3 flange reference aligned and cable routing restored.

No motion was performed after forearm reassembly. Final installed-load validation remains a separate future authority gate.

## Evidence

- `hardware/v15_24a_ft/j2_big_arm_coupling_run.csv` — original contract Run 1
- `hardware/v15_24a_ft/j2_big_arm_coupling_run2_operator_override.csv` — explicitly authorized Run 2
- `hardware/v15_24a_ft/j2_big_arm_coupling_inventory.json`
- `hardware/v15_24a_ft/SHA256SUMS`

`V15_24A_FT_J2_BIG_ARM_COUPLING_GATE = PASS`
