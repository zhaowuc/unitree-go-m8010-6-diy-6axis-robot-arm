# V15.21D — J6 DM-G6220 POS_VEL Safe Enable / Current-Position Hold

## Authority and result

- Source authority: `349fa982726e73cb799113f96cbb462e98fc85ca`
- Work branch: `agent/v15-21d-j6-posvel-enable-hold`
- `J6_POSVEL_ENABLE_HOLD = PASS`
- `FINAL_TASK_RESULT = PASS`
- `J6_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`

This task tested only a hard-time-bounded current-position enable/hold in POS_VEL mode. It did not test commanded displacement, sign, bidirectional motion, limits, zero, READY pose, ROS 2, or a trajectory.

## Windows reference authority

The checked-in operator-verified Windows source defaults to `--mode pos_vel` with `--switch-mode` enabled. Its POS_VEL command uses CAN ID `0x100 + motor_id`, so J6 ID 1 uses `0x101`, with payload `struct.pack("<ff", position, velocity)`.

`WINDOWS_REFERENCE_MOTION = OPERATOR_VERIFIED_PASS` does not itself establish Ubuntu authority. The Ubuntu result in this document comes from the V15.21D raw evidence.

## Strict transport and classification

`dm_g6220_posvel_transport.py` intentionally exposes only POS_VEL enable, disable, `<ff` position/velocity command, and strict feedback-drain paths. It exposes no MIT command, arbitrary parameter write, set-zero, change-ID, or mode-switch operation. The one gated runtime mode write is isolated in `j6_posvel_mode_commissioning.py`.

Motion fields are decoded only when a frame is an RX callback (`dir=0`) on channel 0, standard Classic CAN, not extended, not CAN-FD, not RTR, DLC 8, CAN ID in `{0x000, 0x001}`, has the valid J6 ID/state encoding, and is not a parameter response. All other frames remain non-motion classifications and cannot enter motion or safety decoding.

## Runtime mode switch and DISABLED baseline

Initial read-only state:

- Adapter identity: `34b7:6877`, serial `EEE8D71AB573449FCAFE7B39BD222C75`
- J6 Slave/ESC ID: 1
- `CTRL_MODE_INITIAL = 1 (MIT)`
- Initial feedback: 5/5 `DISABLED`

After the exact operator gate, RID10 was written exactly once while the motor was DISABLED:

- Requested runtime value: `2 (POS_VEL)`
- RID10 readback: 2
- Flash/EEPROM save: not used
- Other RID writes: none
- Power-cycle persistence test: not run

POS_VEL DISABLED baseline evidence: `hardware/v15_21d/j6_posvel_disabled_baseline_20260820T095219Z.csv`

- Valid feedback: 100/100
- State: all `DISABLED`
- q median: `1.0976958876935985 rad`
- q mean: `1.097661554894331 rad`
- q standard deviation: `0.0002164348508812731 rad`
- q range: `1.097314412146181` to `1.098077363241016 rad`
- Velocity range: `-0.0549450549450512` to `+0.0329670329670364 rad/s`
- Temperature: MOS 41 °C, coil 38 °C
- Mode-switch-only q delta: `0.0 deg`

The baseline median became the session-local POS_VEL reference. It is not CAD zero, ROS zero, motor internal zero, READY reference, or a reusable blind startup command.

## POS_VEL micro-enable

The operator confirmed both clearance and emergency-stop gates. The sequence sent a POS_VEL preload while DISABLED, FC enable, immediate current-position hold, another identical hold near 10 ms, and FD at the hard deadline. Callback delivery never controlled or extended the enabled interval.

The first attempt met every objective safety criterion, but the operator did not observe it clearly. The operator explicitly authorized one identical observation repeat. No second RID10 write occurred. The repeat used a newly captured 100-frame DISABLED reference and is the final observation authority.

Authoritative repeat:

- Session q: `1.0976958876935985 rad`
- Preload/active target: `p=1.0976958876935985 rad`, `v=0.0 rad/s`
- POS_VEL CAN ID: `0x101`
- Preload payload: `4D818C3F00000000`
- Actual enabled window: `12.00195006094873 ms`
- Hard maximum: `15 ms`
- First active RX: CAN ID `0x000`, payload `118B3D7FE7FF2926`
- First active classification/state: `NORMAL_FEEDBACK`, `ENABLED`
- First active q: `1.0976958876935985 rad`
- First active velocity: `-0.0329670329670364 rad/s`
- Maximum active velocity by magnitude: `-0.0329670329670364 rad/s`
- Fault states: none
- Unexpected RX frames: 0
- FD sent: yes
- Final five feedback frames: `DISABLED` PASS
- q_post median: `1.0976958876935985 rad`
- q_post minus q_session: `0.0 deg`
- Operator visible jerk: no; operator also reported no movement and no abnormal sound
- `CTRL_MODE_FINAL = 2 (POS_VEL)`

## Direct MIT versus POS_VEL comparison

| Metric | MIT V15.21C | POS_VEL V15.21D |
|---|---:|---:|
| Enabled window | 12.00306392274797 ms | 12.00195006094873 ms |
| q_session | 1.0835812924391544 rad | 1.0976958876935985 rad |
| q_post − q_session | +0.8087067376150295 deg | 0.0 deg |
| Visible jerk | YES | NO |
| First active q | 1.0835812924391544 rad | 1.0976958876935985 rad |
| First active velocity | +0.010989010989007397 rad/s | −0.0329670329670364 rad/s |
| Maximum active velocity by magnitude | +1.83516483516484 rad/s | −0.0329670329670364 rad/s |
| Fault | NONE | NONE |
| Final disable | PASS | PASS |

## Acceptance and remaining authority

POS_VEL runtime mode readback was 2; the motor was DISABLED before enable; the enabled window was below 15 ms; no fault or unexpected RX occurred; the final five frames were DISABLED; the operator reported no jerk, movement, or abnormal sound; and the measured net displacement was 0.0°, within the 0.2° limit. Therefore `J6_POSVEL_ENABLE_HOLD = PASS`.

The following remain unresolved:

- no commanded-displacement, sign, or bidirectional test has been run;
- runtime RID10 persistence across power-cycle is not tested;
- CAD zero, ROS zero, physical limits, and J6 READY reference remain pending;
- the controller-level mechanism behind the MIT enable transient remains unresolved.

The motor was left DISABLED with `CTRL_MODE_FINAL = 2`. No automatic restoration to MIT was performed. `J6_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`.
