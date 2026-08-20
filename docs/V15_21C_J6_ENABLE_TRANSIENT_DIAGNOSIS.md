# V15.21C — J6 DM-G6220 Enable Transient Root-Cause Diagnosis

## Authority and result

- Source authority: `1583a0f84d35e8b0fcf0849b5f737a1fc5369dd5`
- Work branch: `agent/v15-21c-j6-enable-transient-diagnosis`
- Final diagnosis status: `DIAGNOSIS_COMPLETE`
- Root cause: `REAL_ENABLE_TRANSIENT`
- J6 local motion authority: `NOT_GRANTED`

This task diagnosed the V15.21B HOLD CURRENT failure. It did not commission motion. No sign probe, bidirectional trajectory, gain tuning, control-mode change, zero write, ID write, or parameter write was performed.

## Windows reference correction

The checked-in Windows reference `dm_g6220_run_100_cycle.py` defaults to `--mode pos_vel`, and mode switching is enabled by default. The operator-reported Windows run therefore establishes only:

- `WINDOWS_REFERENCE_MOTION = OPERATOR_VERIFIED_PASS`
- `DEFAULT_REFERENCE_MOTION_MODE = POS_VEL`
- `WINDOWS_MIT_KP10_KD2 = NOT_VERIFIED`

Kp 10 and Kd 2 are diagnostic constants in this task, not operator-verified Windows MIT gains and not tuned gains.

## Static transport and decoder audit

Audited sources:

- Installed Python package: `dmcan-sdk 1.0.4`, including `dmcan_def.py` and `dmcan_device.py`.
- Official `dmBots/dm-device-sdk` commit `36a8b3971dc35608701d128f353a32cad3ee6cdb`, including `C&C++/lib/v1.1.0/dmcan.h` and the v1.1.0 example.
- Official `dmBots/motor-sdk` commit `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a`, including `Python例程/u2can/DM_CAN.py`.
- Frozen V15.21A transport, V15.21B motion transport, and V15.21B raw evidence.

The SDK provides separate receive, sent, and error callbacks. The installed SDK defines `dir=0` as RX and `dir=1` as TX. The logger recorded matching TX-request and sent-callback entries separately from RX callbacks; no TX echo was decoded as feedback.

The V15.21B `_decode_feedback_payload()` is too broad: it relies primarily on payload length and `(data[0] & 0x0F) == motor_id`, without requiring a proven CAN ID, exact DLC, direction, or response type. An offline test showed it can accept a plausible payload on an unrelated CAN ID, while the strict classifier rejects it. Parameter replies also require explicit exclusion. This is a real decoder defect, but it is **not** the cause of the reproduced V15.21C physical transient: all relevant active frames passed the strict normal-feedback classification.

The official motor SDK maps ordinary feedback by Slave ID or Master ID, with a CAN-ID-zero fallback that uses the payload low nibble. RID7 reported Master ID 0 and the motor Slave/ESC ID is 1, so the diagnostic allowlist is `{0x000, 0x001}`. The observed normal feedback was standard Classic CAN ID `0x000`, DLC 8. Parameter response classification requires its proven payload signature (`payload[0:2]` is the Slave ID and `payload[2]` is `0x33` or `0x55`). Any frame that does not meet a proven class remains `UNKNOWN` and is never supplied to position, velocity, torque, or safety decoding.

No distinct FC enable-acknowledgement frame format was established from the official SDK. Its enable path reads through the ordinary motor-status receive path.

## DISABLED raw baseline

Evidence: `hardware/v15_21c/j6_raw_disabled_baseline_20260820T093200Z.csv`

- Refresh rate/duration: 100 Hz for 1 second
- Strict valid feedback: 100/100
- State: all `DISABLED`
- Observed normal-feedback CAN ID: `0x000`
- DLC: 8
- q median: `1.0828183413443195 rad`
- q mean: `1.0827496757457844 rad`
- q standard deviation: `0.00024354756541561474 rad`
- Strict classifier versus frozen read-only decoder disagreement on the formal baseline feedback: 0

The raw audit contained separate TX requests, RX callbacks, and sent callbacks. Callbacks were delivered in batches roughly 100 ms after transmission. Batching is therefore proven, but no stale-frame origin or incorrect RX/TX association was proven.

## Bounded micro-enable evidence

The operator confirmed both safety gates before each bounded diagnostic attempt. Each attempt used a newly captured session-local q reference, sent one MIT current-position preload while DISABLED, sent FC, sent the same current-position hold immediately and again near 10 ms, then sent FD on a hard deadline. Callback arrival was never used to extend the enabled interval.

The first attempt used a `12.000916991382837 ms` enabled window. It ended with five DISABLED frames and a post-minus-session displacement of `+0.04371387770892052 deg`. The operator did not observe this attempt clearly and explicitly requested one repeat. No reset, zero write, mode change, or trajectory was performed.

The authorized repeat is the diagnosis authority:

- q_session median: `1.0835812924391544 rad`
- Session valid DISABLED feedback: 100/100
- MIT preload sent while DISABLED: yes
- FC sent: yes
- First FD: `12.00306392274797 ms` after FC, below the 15 ms hard maximum
- Final five feedback frames: `DISABLED` PASS
- First active RX: CAN ID `0x000`, DLC 8, payload `118B188007FF2926`
- First active classification/state: `NORMAL_FEEDBACK`, enabled state 1
- First active q: `1.0835812924391544 rad`
- First active velocity: `+0.010989010989007397 rad/s`
- Maximum active-frame velocity observed in the bounded window: `+1.83516483516484 rad/s`
- First disabled-adjacent velocity after FD: `-3.08791208791209 rad/s`
- q_post median: `1.0976958876935985 rad`
- q_post minus q_session: `+0.01411459525444414 rad`, or `+0.8087067376150295 deg`
- Operator observation: visible jerk and audible sound

The full sequence and callback stream are retained in `j6_micro_enable_raw_20260820T093509Z.csv`; the post-disable samples are retained in `j6_post_disable_position_20260820T093509Z.csv`.

## Diagnosis

`ROOT_CAUSE = REAL_ENABLE_TRANSIENT`

The repeat produced a measurable 0.5–2 degree net displacement and the operator observed a visible jerk with sound. Its active frames are strict normal feedback frames, not an FC acknowledgement, parameter response, unknown frame, or TX echo. SDK callback batching exists, but the time-bounded TX sequence and the independent post-disable position measurement prove a real physical transient; batching is not the root cause classification.

This result does not establish the exact controller-level mechanism. The following remain unresolved:

- why the controller creates the enable transient despite a disabled-state current-position preload;
- how the bounded V15.21C transient relates quantitatively to V15.21B's approximately ±6 rad/s feedback;
- whether a future authorized task should use the Windows-reference POS_VEL path instead of MIT;
- how startup transient filtering and position-derived velocity authority should be designed.

No production motion decoder was changed. The strict classifier exists only in the new diagnostic tool. J6 remains DISABLED after the test, and `J6_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`.
