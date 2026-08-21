# V15.22C J3 local motion commissioning — failed evidence freeze

## Final classification

`FINAL_TASK_RESULT = FAIL`

J3 communication and current-position HOLD passed, but the mandatory protocol `+3°` sign probe failed its endpoint acceptance. Active FOC had already begun, so V15.22C requires FAIL rather than BLOCKED and prohibits automatic retry, gain changes, gravity feedforward, or continuation to the bidirectional route.

`J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`

## Source authority

- Source branch: `agent/v15-22b-j4-local-motion`
- Source HEAD: `ae21327983e0567bf8a7ce4c3cb83442ae03b71e`
- Working branch: `agent/v15-22c-j3-local-motion`

All frozen J1, J2, J4, J5, J6, READY_POSE_V1, and simulation authorities remain unchanged.

## Shared bus and session

The FTDI FTASQA6F CHANNEL_2 / interface 02 bus at 4 Mbps passed a BRAKE-only preflight:

- J3 ID 3: 500/500 valid, merror 0, maximum temperature 34°C
- J4 ID 4: 500/500 valid, merror 0, maximum temperature 35°C
- J5 ID 5: 500/500 valid, merror 0, maximum temperature 35°C

The initial J3 BRAKE session captured 100/100 valid frames. Its median was `2.1947481632232666 rad` and standard deviation was `0.00013940117967902569 rad`. This is only `J3_SESSION_REFERENCE_V15_22C`; it is not READY_POSE, CAD zero, ROS zero, motor zero, or a reusable blind startup target.

## HOLD1

The first FOC frame used a freshly captured current position, with Kp 0.50, Kd 0.05, Tff 0, at 100 Hz. HOLD1 ran for 2 seconds and passed:

- Maximum output-equivalent drift: `0.016487407863457225°`
- Final 500 ms median error: `0.01475234556997298°`
- Maximum derived qdot_slow: `0.07576630676515314 deg/s`
- Operator gravity sag: NO
- merror/invalid frames: 0/0
- Final five-frame BRAKE: PASS

This result applies only to the tested pose and does not establish full-range gravity behavior.

## Mandatory +3° sign probe failure

The controller commanded the contract-mandated motor-protocol `+3°` rest-to-rest trapezoid and then returned to the freshly captured session position:

- Actual positive delta: `+0.9797458689656786°`
- Endpoint error: `2.0202541310343216°`
- Required endpoint error: `<= 1.5°`
- Return error: `0.8487087421105668°` (within the 1.0° return bound)
- merror/invalid frames: 0/0
- Final five-frame BRAKE: PASS

The endpoint acceptance failed. After the probe, the operator corrected the mechanical-clearance statement: the current pose does not allow protocol `+3°`; only approximately `-3°` is allowed. This is recorded as `OPERATOR_CORRECTED_MECHANICAL_CLEARANCE_CONSTRAINT`. The earlier `J3_PHYSICAL_CLEARANCE_PLUS_MINUS_10DEG=YES` must not be reused for this pose.

The result does not establish motor failure. It establishes that the V15.22C route cannot pass in the current mechanical pose under its mandatory `+3°` gate.

## Operator-authorized R1 at a changed pose

The operator subsequently changed the physical pose and explicitly authorized a separately preserved manual retry (`V15.22C-R1`). The retry controller accepted only HOLD and positive `+3°`; it did not gain ±5° authority.

R1 HOLD passed at a freshly captured raw position of `9.656047821044922 rad`:

- Maximum drift: `0.005213819031166577°`
- Final median error: `0.003470124586968484°`
- Operator gravity sag: NO
- Final five-frame BRAKE: PASS

The R1 `+3°` endpoint was within its endpoint tolerance, but the return was not:

- Actual positive delta: `+1.6158436599752326°`
- Endpoint error: `1.3841563400247674°` — PASS (`<= 1.5°`)
- Return error: `1.3468313151287503°` — FAIL (`> 1.0°`)
- merror/invalid frames: 0/0
- Final five-frame BRAKE: PASS

R1 therefore also failed. No further retry, HOLD2, SESSION 2, or bidirectional motion was executed.

## Sign and axis status

URDF and MuJoCo both define J3 on local +Y (`[0, 1, -2.77555756156289e-17]`), using the same local-axis convention as J4. The operator also stated that J3 and J4 share the same physical rotation convention. However, because the required sign probe failed and the exact post-probe ROS-direction answer was not frozen, `raw_to_ros_sign` remains pending and the bus configuration is not updated.

## Fail-closed actions

- No automatic retry was performed.
- No `-3°` substitute probe was performed.
- SESSION 2, HOLD2, and ±5° bidirectional motion were not performed.
- No gains or feedforward values were changed.
- J3 ended in BRAKE with five valid confirmation frames.
- J4 and J5 remained in BRAKE throughout.
- No ID, CALIBRATE, zero, EEPROM, READY_POSE, or simulation authority was modified.

## Frozen outcome

- `J3_COMMUNICATION = PASS`
- `J3_HOLD_CURRENT_AT_TESTED_POSE = PASS`
- `J3_LOCAL_MOTION = FAIL`
- `J3_BIDIRECTIONAL_5DEG = NOT_TESTED`
- `J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_RAW_TO_ROS_SIGN = PENDING`
- `J3_FULL_RANGE = NOT_TESTED`
- `J3_PHYSICAL_LIMITS = PENDING`
- `J3_CAD_ZERO = PENDING`
- `J3_ROS_ZERO = PENDING`
- `J3_FULL_COMMISSIONING = NOT_COMPLETE`
