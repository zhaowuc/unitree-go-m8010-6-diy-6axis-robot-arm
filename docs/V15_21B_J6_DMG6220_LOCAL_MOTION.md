# V15.21B — J6 DM-G6220 MIT Small-Motion Commissioning

## Result

```
J6_DMG6220_LINUX_MIT_MOTION = FAIL
J6_HOLD_CURRENT = FAIL
J6_SIGN_PROBE = NOT_RUN
J6_BIDIRECTIONAL_5DEG = NOT_RUN
J6_LOCAL_MOTION = FAIL
J6_LOCAL_MOTION_AUTHORITY = NOT_GRANTED
J6_FULL_RANGE = NOT_TESTED
J6_PHYSICAL_LIMITS = PENDING
J6_CAD_ZERO = PENDING
J6_ROS_ZERO = PENDING
J6_READY_REFERENCE = PENDING
J6_FULL_COMMISSIONING = NOT_COMPLETE
```

Source HEAD: `b9a604eaee7d02837db9234e30fe1ccd434a6f15`

Branch: `agent/v15-21b-j6-dmg6220-motion`

## Frozen hardware and control identity

- Adapter: DaMiao-Tech DM-USB2FDCAN, VID:PID `34b7:6877`
- USB serial: `EEE8D71AB573449FCAFE7B39BD222C75`
- Transport: `dmcan_sdk` / native libusb
- CAN: channel 0, Classic CAN, 1,000,000 bit/s
- Motor: DM-G6220, ID 1
- CTRL_MODE before motion: 1 / MIT
- CTRL_MODE modified: NO
- MIT command: Kp 10.0, Kd 2.0, Tff 0.0, 100 Hz

No set-zero, parameter write, ID write, mode write, CAD/ROS zero update, physical-limit update, or READY reference update occurred.

## Commissioning sequence and evidence

The first DISABLED session capture produced 100/100 valid samples and a session-local median of `1.0542076752880138 rad` (standard deviation `0.0002441443503471419 rad`). After FC enable, the HOLD CURRENT phase received an enabled feedback sample with `-6.032967032967029 rad/s`, exceeding the temporary `0.7 rad/s` velocity guard. The trajectory stopped immediately; the +3 degree trajectory was never entered. FD disable was sent and five subsequent feedback frames were all DISABLED.

The operator reported that the initial physical pose had constrained rotation, repositioned J6, and explicitly re-confirmed both clearance and emergency-disable gates. The first retry preflight was safely blocked before enable because the new position differed from the old session reference by `2.710260 deg`.

A new DISABLED session was therefore captured at the repositioned pose: 100/100 valid samples, median `1.1015106431677726 rad`, standard deviation `0.0002817759260337121 rad`. On the single explicitly authorized active retry, HOLD CURRENT again failed immediately after enable: the first enabled feedback sample reported `+6.010989010989007 rad/s`, while the commanded position remained the session reference. The +3 degree trajectory was again never entered. FD disable was sent and five subsequent frames were all DISABLED.

The retry telemetry recorded:

- actual TX rate: `100.0347582322921 Hz`
- maximum TX jitter: `0.26886584237217903 ms`
- maximum callback gap: `0.05039689131081104 ms`
- maximum feedback age: `90.88742802850902 ms`
- maximum absolute feedback velocity: `6.010989010989007 rad/s`
- torque protocol value: min/max `0.07570207570207543`
- MOS temperature: min/max `41 C`
- coil temperature: min/max `37 C`
- watchdog trips: `0`
- fault states observed: none
- operator abort: no
- final five-frame DISABLED verification: PASS

The short callback-gap value reflects the SDK's batched callback delivery; the maximum feedback-age measurement represents the approximately 90 ms period before the first enabled feedback batch.

## Failure interpretation

V15.21B entered active motion and failed the mandatory velocity guard during HOLD CURRENT twice. The second occurrence followed physical repositioning and a new session capture, so the first mechanical constraint does not fully explain the behavior. The evidence shows a fast enabled-state transient, but this task does not establish whether its root cause is physical motion, enable/hold timing, protocol feedback semantics, or SDK callback batching.

Per the commissioning contract, there was no automatic retry after the explicitly authorized retry, no gain tuning, no guard relaxation, and no +3/±5 degree motion. Diagnosis or any revised enable/hold strategy requires a separate authorized task.

## Authority preservation

J1, J3/J4/J5, READY_POSE_V1, simulation authority, and V15.21A read-only evidence remain unchanged. `protocol_to_ros_sign` remains PENDING. The session references remain `SESSION_LOCAL_ONLY` and are not motor, CAD, ROS, or READY zeros.
