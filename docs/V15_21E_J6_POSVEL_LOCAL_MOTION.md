# V15.21E — J6 DM-G6220 POS_VEL Local Motion Commissioning

## Authority and final result

- Source authority: `eb07deabed72f131a64f8dafd3543c7898c4e1d8`
- Work branch: `agent/v15-21e-j6-posvel-local-motion`
- `FINAL_TASK_RESULT = PASS`
- `J6_POSVEL_LOCAL_MOTION = PASS`
- `J6_BIDIRECTIONAL_5DEG = PASS`
- `J6_LOCAL_MOTION_AUTHORITY = PASS`

This authority is limited to the tested POS_VEL local range of ±5° at a 10°/s velocity limit and 100 Hz command refresh. `J6_FULL_RANGE = NOT_TESTED`, physical limits and zero references remain pending, and full commissioning is not complete.

The MIT authority remains unchanged: V15.21B MIT HOLD failed and V15.21C proved a real enable transient. V15.21D POS_VEL current-position enable/hold passed. MIT was not retested in V15.21E.

## Identity, mode, and session

- Adapter: DaMiao-Tech DM-USB2FDCAN, VID:PID `34b7:6877`
- USB serial: `EEE8D71AB573449FCAFE7B39BD222C75`
- CAN: channel 0, standard Classic CAN, 1 Mbps
- Motor Slave/ESC ID: 1; Master ID: 0
- Initial and final CTRL_MODE: `2 / POS_VEL`
- Runtime mode write required/count: `NO / 0`
- Persistence: `NOT_TESTED`

The newly captured V15.21E session contained 100/100 valid DISABLED feedback frames:

- median: `1.0976958876935985 rad`
- mean: `1.097730220492866 rad`
- standard deviation: `0.00020254160299776054 rad`
- range: `1.097314412146181` to `1.098077363241016 rad`

The session is `SESSION_LOCAL_ONLY`; it is not CAD zero, ROS zero, motor internal zero, READY reference, or an initialization angle authority.

## Command and guard contract

POS_VEL commands used CAN ID `0x101` and `<ff` little-endian payloads containing the absolute position target and a positive velocity limit. Both positive and negative motions used `+0.17453292519943295 rad/s` (10°/s); target position selected direction. The final absolute target was refreshed at 100 Hz without constructing an external point-by-point trajectory.

Only strict V15.21D normal feedback entered telemetry or safety decisions. Commissioning guards were ±6° command envelope, ±8° feedback envelope, 0.7 rad/s feedback speed, temperature below 60 °C, feedback age at most 150 ms, enabled state with no fault, and fail-closed FD handling.

## +3° sign probe

The first +3° round trip passed objectively, but the operator did not see it clearly. After explicit authorization, one identical +3° observation repeat was performed; the range and speed were not increased. Both attempts ended with verified DISABLED feedback.

The repeat is the sign-probe authority:

- session: `1.1110475318532078 rad`
- absolute +3° target: `1.1634074094130378 rad`
- initial HOLD: PASS
- endpoint delta: `+3.0818283784788965°`
- endpoint error: `0.0818283784788921°`
- return error: `0.6119942879248872°`
- actual TX rate: `99.98830092848273 Hz`
- maximum TX jitter: `0.385132385417819 ms`
- maximum feedback age: `100.18720990046859 ms`
- maximum absolute feedback velocity: `0.1868131868131826 rad/s`
- fault/watchdog: NONE / 0
- final DISABLED verification: PASS

The operator viewed from the J5 side toward the gripper and observed protocol-positive motion as counterclockwise. URDF and MuJoCo both define the J6 axis as approximately `+Z`; the TCP extends toward positive Z. From the operator's viewing direction, ROS-positive right-hand rotation appears clockwise. Therefore:

- operator ROS direction answer: `NO`
- `protocol_to_ros_sign = -1`
- sign authority: `OPERATOR_PHYSICAL_DIRECTION_CONFIRMATION` plus `URDF_MUJOCO_AXIS_REVIEW`

## ±5° bidirectional commissioning

A new 100/100 DISABLED session was captured before this round. Its median was `1.1259250782024868 rad`, within the required 2° proximity gate.

The exact route was `0 → +5° → 0 → −5° → 0`:

- second initial HOLD: PASS
- +5° endpoint delta: `+4.91781124225366°`
- +5° endpoint error: `0.08218875774633458°`
- first center return error: `0.0°`
- −5° endpoint delta: `−4.983382058816939°`
- −5° endpoint error: `0.01661794118305559°`
- final return error: `0.7212789821971886°`
- actual TX rate: `99.9954257070911 Hz`
- maximum TX jitter: `0.3187633119523525 ms`
- maximum feedback age: `99.9332049395889 ms`
- maximum absolute feedback velocity: `0.2087912087912116 rad/s`
- torque protocol range: `−0.026862026862026767` to `+0.012210012210012167`
- MOS/coil temperatures: `41 °C / 38 °C`
- fault/watchdog: NONE / 0
- final FD and five-frame DISABLED verification: PASS
- operator safe observation: YES

## Remaining limits

POS_VEL is the current preferred J6 low-level position-control path. The operator requested an initialization angle after this test; it is intentionally not defined here because session references are not zero or READY authority. A separate task must establish the initialization reference and its safe startup semantics.

Still unresolved: full range, physical limits, CAD zero, ROS zero, READY reference, CTRL_MODE power-cycle persistence, and full J6 commissioning. READY_POSE_V1, J1, J3/J4/J5, and simulation authority were not modified.
