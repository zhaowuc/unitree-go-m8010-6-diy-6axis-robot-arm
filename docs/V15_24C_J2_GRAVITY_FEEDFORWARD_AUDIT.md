# V15.24C J2 gravity / feedforward authority audit

## Scope and frozen starting point

- Source HEAD: `ae3e5205722262790eeda5a9f39470ba2967b2d2`
- Branch: `agent/v15-24c-j2-gravity-feedforward-audit`
- Hardware motion, FOC, serial-port construction, and device I/O: **NO**
- Production simulation model modification: **NO**
- J2/J3 local-motion authority change: **NO**

This task is an offline controller, serializer, existing-evidence, and gravity-model audit. It does not grant new hardware authority.

## 1. Frozen Unitree SDK tau audit

The audited SDK is the official `unitreerobotics/unitree_actuator_sdk` repository at commit `5b79a42d81cd69adac1367db79efdb972a898fd1`.

The GO protocol header describes `RIS_Comd_t.tor_des` as desired joint-output torque in N·m with q8 encoding. The [official SDK README](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/main/README.md) states that commands are rotor-side and output-side commands require reducer conversion. The user-supplied official GO-M8010-6 use manual V1.2 resolves the terse header wording:

- Page 4 defines mixed-control `tau` as motor rotor output torque and defines `p`/`omega` as rotor position/velocity.
- Page 13 states that all commands target shaft 1 before the reducer, not output shaft 2, and gives the ratio as 6.33.
- Page 17 calls `tau_set` desired motor torque in N·m and specifies q8 scaling.
- Page 18 states that, unless specially noted, protocol quantities are rotor-side rather than output-side.

The PDF's SHA-256 is `1312eb4f9af0d5ebfe1e46376e6373df43330edc9222540e04e4ee4dd55e92be`; key pages were both text-extracted and visually rendered. The older official V1.0 manual (`b6dcadd37db9fa05c9e0704647346c7a9091aefaf3296ecf525712b3044ff446`) independently describes `T` as motor rotor output torque.

Therefore:

- Numeric quantity/unit evidence: N·m, q8, **PASS**.
- `tau` physical side: **MOTOR_ROTOR_TORQUE**.
- `DOCUMENTATION_CONFLICT`: **NO**; the product manual resolves the header ambiguity.
- `TAU_PHYSICAL_UNIT_AUTHORITY`: **PASS**.
- `J2_TAU_COMMAND_AUTHORITY`: **PASS**.

The real closed-library path `MotorCmd.tau -> MotorCmd::modify_data() -> ControlData_t.comd.tor_des` was exercised by `tools/hardware/v15_24c_j2_tau_serializer_self_test.cpp`. The program constructs no `SerialPort` and performs no device I/O.

Observed encoding:

```text
raw_count = trunc(tau_literal * 256), within the serializer's clamp
decoded_value = raw_count / 256
```

- Protocol storage: signed int16, nominal `[-32768, 32767]` counts or `[-128, 127.99609375]` N·m.
- Frozen serializer observed clamp: `[-32765, 32765]` counts or `[-127.98828125, 127.98828125]` N·m.
- LSB: `1/256 = 0.00390625` N·m.
- `tau=0 -> 0`; `+0.25 -> +64`; `-0.25 -> -64`.
- `+0.05 -> +12 -> +0.046875`; `-0.05 -> -12 -> -0.046875`.
- Zero, positive/negative symmetry, packet structure, and CRC: **PASS**.

The project-side generic driver name `tau_rotor_nm` now agrees with the product manual.

## 2. Torque conversion and dual-motor sign mapping

Frozen kinematic mapping:

```text
G = 6.329999923706055
sign_A = -1
sign_B = +1
q_rotor_i = q0_i + sign_i * G * q_J2
```

By virtual work, with the now-authoritative rotor-side SDK quantity:

```text
tau_J2 = G * (sign_A*tau_A_rotor + sign_B*tau_B_rotor)
tau_A_rotor = sign_A * tau_J2 / (2*G)
tau_B_rotor = sign_B * tau_J2 / (2*G)
```

For comparison only, an output-side API would instead use:

```text
tau_J2 = sign_A*tau_A_output + sign_B*tau_B_output
tau_A_output = sign_A * tau_J2 / 2
tau_B_output = sign_B * tau_J2 / 2
```

Thus a positive logical J2 torque requires a negative J2A rotor-torque command and a positive J2B rotor-torque command. The selected physical formula divides each equal logical share by the gear ratio.

## 3. Frozen MuJoCo gravity audit

Production MJCF:

`V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml`

SHA-256: `5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9`.

The XML intentionally compiles with zero gravity. Following the frozen V15.18A audit method, gravity was overridden only in the in-memory compiled model to `[0, 0, -9.81] m/s²`; `qvel=qacc=0`, then `mj_forward()` produced `qfrc_bias`. No time integration or actuation occurred and no model file was written.

For a revolute coordinate, `qfrc_bias[Jn]` is the positive generalized holding/compensation torque in N·m required at rest. The gravitational generalized force itself is `-qfrc_bias[Jn]`.

The V15.24B session zero is relative to the captured encoders and does not uniquely recover the six MuJoCo coordinates. `CURRENT_PHYSICAL_POSE_EXACT = NO`. The generated CSV is explicitly a `REPRESENTATIVE_MODEL_SWEEP`: J2 and J3 each use `{-30,-15,0,+15,+30}°`, while J1/J4/J5/J6 are fixed at 0°.

Results:

| Joint | Minimum holding torque | Maximum holding torque | Median | Sign change |
|---|---:|---:|---:|---|
| J2 | 8.442850988 N·m | 13.571141587 N·m | 12.518667930 N·m | NO |
| J3 | 2.136251271 N·m | 6.287493694 N·m | 5.708569865 N·m | NO |

At model mechanical zero, the already-frozen V15.18A evidence gives J2 `13.493267324 N·m` and J3 `6.209619430 N·m`; this is not claimed as the current exact physical pose.

Equal-share J2 values over the representative sweep are:

- Per-motor output-shaft share: `4.221425494...6.785570794 N·m`.
- Required SDK rotor-side Tff: `0.666891871...1.071970123 N·m` per motor before sign.

The rotor-side Tff is only `0.52...0.84%` of the serializer's approximately ±127.988 numeric clamp. The V1.0 product manual lists 23.7 N·m maximum torque for the geared actuator; divided by 6.33 this is approximately 3.744 N·m rotor-equivalent, so the sweep requires approximately `17.8...28.6%` of that listed maximum per motor. This does not establish continuous thermal authority or the actual current limit.

`J2_GRAVITY_MODEL_AUDIT = PASS`.

## 4. Existing V15.24B command and feedback analysis

All three CSVs reach exact logical references of `-5°` and `+5°`. For every active row:

```text
a_q_cmd = a0 + sign_A * G * q_ref
b_q_cmd = b0 + sign_B * G * q_ref
```

The maximum residual is exactly zero in the recorded double values for A and B in all runs. The q protocol counts are orders of magnitude inside the signed int32 q15 field. Therefore:

- `EXISTING_RUN_COMMAND_TARGET_CORRECT = YES`
- `POSITION_COMMAND_BUG = NOT_FOUND`

Endpoint torque feedback behavior in authoritative rotor-side N·m:

| Run | Kp | +5° A median | +5° B median | -5° A median | -5° B median |
|---|---:|---:|---:|---:|---:|
| RUN-A | 0.60 | -0.3203125 | +0.3359375 | +0.3281250 | -0.3203125 |
| RUN-A repeat | 0.60 | -0.3125000 | +0.3281250 | +0.3398438 | -0.3320313 |
| RUN-B | 0.70 | -0.3515625 | +0.3632813 | +0.3867188 | -0.3828125 |

A/B raw signs are opposite and align to the same logical effort. Endpoint values form sustained narrow bands. Their magnitude increases when Kp rises from 0.60 to 0.70; the protocol numeric limit is not approached. Converting the median paired feedback using `G*(sign_A*tau_A + sign_B*tau_B)` gives about `4.05...4.25 N·m` logical J2 effort at Kp 0.60 and `4.53...4.87 N·m` at Kp 0.70. These values are below the representative model's `8.44...13.57 N·m` J2 gravity hold envelope. This is consistent with a correct position command meeting a common-mode load and does not show a differential synchronization problem.

- Protocol-field saturation evidence: **NO**.
- Actual motor/current/internal-controller saturation: **INCONCLUSIVE**.
- Position-loop effort limiting/scaling: still a candidate.
- Mechanical brake/static friction: still a strong candidate because motion is small in both commanded directions.

The model shows a substantial J2 gravity term across the representative final-assembly sweep, while recorded controller effort remains well below that envelope. Therefore `GRAVITY_LOAD_CONTRIBUTION = SUPPORTED`, but `ROOT_CAUSE = GRAVITY` is not established. Bidirectional near-symmetry and the zero-Tff current-hold behavior still require exclusion of static friction/brake, internal limiting, and pose/model mismatch.

## 5. Feedforward decision and next experiment

Rotor-side model conversion is now authoritative, so a bounded position-control gravity-feedforward experiment is ready. This is distinct from zero-gravity/teach mode; no low-impedance or teach-mode design is authorized here.

The next single experiment is:

`NEXT_SINGLE_EXPERIMENT = J2_GRAVITY_FEEDFORWARD_BOUNDED_HOLD_AND_5DEG_TEST`

Frozen controls remain `Kp=.60`, `Kd=.05`, `10°/s`, `30°/s²`. Begin with zero-Tff current hold, then smoothly ramp paired commands for positive logical effort:

```text
J2A: 0 -> -0.05 literal  (encoded -12 counts = -0.046875 SDK units)
J2B: 0 -> +0.05 literal  (encoded +12 counts = +0.046875 SDK units)
```

First require the bounded current-position hold to pass. Then retain the same paired Tff for the existing `0 -> +5 -> 0 -> -5 -> 0` route. Abort on unexpected direction, rapid motion, `e_sync > 0.3°` during the feedforward hold (and retain the existing motion hard gates), nonzero `merror`, temperature/sound/operator abnormality, or feedback loss.

The bounded value is supported independently by the V1.2 manual's unloaded torque-mode example (`T=0.05`), the repository generic driver's existing ±0.05 guard, and its being below the existing endpoint feedback bands. Because the current exact model pose is unavailable, the actual encoded `0.046875 N·m` is reported as a representative percentage range:

- `4.373...7.029%` of each representative theoretical rotor-side gravity share.

This deliberately does not apply 100% compensation on the first hardware run.

## Final authority

- `J2_TAU_COMMAND_AUTHORITY = PASS`
- `J2_GRAVITY_MODEL_AUDIT = PASS`
- `J2_FEEDFORWARD_EXPERIMENT_READY = YES`
- `J2_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`
- `SIMULATION_MODIFIED = NO`
- `FINAL_TASK_RESULT = DIAGNOSIS_COMPLETE`

Primary evidence is in `hardware/v15_24c/`; its hashes are frozen in `hardware/v15_24c/SHA256SUMS`.
