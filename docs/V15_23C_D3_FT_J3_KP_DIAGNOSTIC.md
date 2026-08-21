# V15.23C-D3-FT — J3 Kp 0.60 low-load repeatability

## Result

`V15.23C-D3-FT = PASS`.

RUN #1 and the single authorized exact repeat both passed every endpoint and center threshold. This establishes `J3_KP_060_REPEATABILITY=PASS`, `J3_LOW_LOAD_BIDIRECTIONAL_5DEG=PASS`, and `J3_LOW_LOAD_MOTION_AUTHORITY=PASS` for the disassembled low-load configuration only.

This result does not grant installed-load or final-assembly motion authority. The upper structure remained removed, J4 and J5 remained physically disconnected, and no J2 work was performed.

## Source and frozen starting authority

- Source branch: `agent/v15-23c-d2-j3-low-speed-diagnostic`
- Source HEAD: `dc6d329fae400c9ffc216f946c4113e45de9c6d7`
- D3 branch: `agent/v15-23c-d3-ft-j3-kp-diagnostic`
- `J3_LOW_LOAD_HOLD=PASS`
- `J3_LOW_LOAD_SIGN=PASS`
- `J3_RAW_TO_ROS_SIGN=+1`
- Starting `J3_LOW_LOAD_BIDIRECTIONAL_5DEG=FAIL_EXISTING_AUTHORITY`
- Starting `J3_LOW_LOAD_MOTION_AUTHORITY=NOT_GRANTED`

## Static audit

The vendor GO-M8010-6 command structure accepts Kp as an SDK literal, and its protocol header describes the `k_pos` field over the nominal `0.0–1.0` range. An in-memory serializer self-test encoded Kp `0.60` as count `768`, preserved Kd `0.05` as count `64`, produced a valid packet, and did not construct a serial port.

`KP_0_60_SDK_ENCODING_VALID=YES`.

Kp increased from `0.50` to `0.60`, a literal increase of 20%. This is not a claim of 20% additional torque: actual closed-loop output also depends on position error, Kd, controller behavior, and saturation.

The historic J1, J4, and J5 commissioning paths used Kp `0.50`. Kp `0.60` was not historically verified before this task. The J1-specific frozen production driver also retains its existing Kp `<=0.50` fast-authority guard and was not modified; D3 used the established direct-SDK J3 commissioning runner under this task's explicit experimental authority.

## Single-variable proof

The base controller was `tools/hardware/v15_23c_j3_low_load.cpp`, SHA-256 `150f1b8b384aab96c27df2a23827df32e1e8513946af43afceb90522986624b2`.

The D3 controller is `tools/hardware/v15_23c_d3_ft_j3_kp.cpp`, SHA-256 `66b9751bf296c4560613cd2d94fbb1bb92d120118ebba3e7bb7ce13f4f13cd26`.

The only changed control variable was Kp `0.50 → 0.60`. Kd `0.05`, Tff `0`, velocity `10°/s`, acceleration `30°/s²`, nominal command rate `100 Hz`, gear ratio `6.3299999237060547`, sign `+1`, route, HOLD durations, thresholds, envelopes, and mechanical configuration were unchanged. Other source differences are limited to D3 gate names, evidence path and labels, plus the offline Kp packet-encoding assertion.

`ONLY_KP_CHANGED=YES`.

## Mechanical and electrical topology

- Upper arm still removed: `YES`
- Low-load configuration unchanged: `YES`
- J3 flange reference intact: `YES`
- J2 reference marks intact: `YES`
- J3 ID3: `CONNECTED`
- J4: `PHYSICALLY_DISCONNECTED`
- J5: `PHYSICALLY_DISCONNECTED`
- J2 active work: `NO`

## RUN #1

BRAKE baseline: `100/100`, median `2.629441022872925 rad`, population standard deviation `0.0001226285823451973 rad`, temperature `32°C`, merror `0`, invalid `0`, final BRAKE `PASS`.

| Metric | Result | Limit | Status |
|---|---:|---:|---|
| Preflight maximum drift | 0.0034701246° | ≤1.0° | PASS |
| Preflight final median error | 0.0008675311° | ≤0.5° | PASS |
| +5 endpoint error | 0.2409830935° | ≤1.0° | PASS |
| First-center error | 0.2013298091° | ≤0.75° | PASS |
| -5 endpoint error | 0.2479233427° | ≤1.0° | PASS |
| Final-center error | 0.0347120361° | ≤0.75° | PASS |

The first and final centers were both `WITHIN_THRESHOLD`; the prior >0.75° residual plateau was not reproduced. Operator observation was safe. Actual command rate was `99.9985869734 Hz`; maximum TX jitter was `0.260333 ms`. Maximum qdot fast/slow was `14.8753280096 / 12.9459156594°/s`; maximum SDK dq was `1.9144124985 rad/s`. Feedback tau ranged from `-0.0703125` to `+0.05859375` SDK units. Temperature was `32°C`; merror and invalid frames were both zero. Final BRAKE passed.

`RUN_1=PASS`.

## RUN #2 — exact repeat

The repeat was explicitly authorized only after RUN #1 passed. A new SESSION 2 BRAKE baseline was captured rather than reusing the first session: `100/100`, median `2.6259894371032715 rad`, population standard deviation `0.00012204240397496529 rad`, temperature `33°C`, merror `0`, invalid `0`, final BRAKE `PASS`.

| Metric | Result | Limit | Status |
|---|---:|---:|---|
| Preflight maximum drift | 0.0034722826° | ≤1.0° | PASS |
| Preflight final median error | 0.0008675311° | ≤0.5° | PASS |
| +5 endpoint error | 0.2236259964° | ≤1.0° | PASS |
| First-center error | 0.2204219685° | ≤0.75° | PASS |
| -5 endpoint error | 0.2861076614° | ≤1.0° | PASS |
| Final-center error | 0.0052073449° | ≤0.75° | PASS |

Both centers were `WITHIN_THRESHOLD`; the >0.75° plateau was again not reproduced. Operator observation was safe. Actual command rate was `99.9969898226 Hz`; maximum TX jitter was `0.270657 ms`. Maximum qdot fast/slow was `14.6268616381 / 12.7244720651°/s`; maximum SDK dq was `2.3316562176 rad/s`. Feedback tau ranged from `-0.06640625` to `+0.06640625` SDK units. Temperature was `33°C`; merror and invalid frames were both zero. Final BRAKE passed.

`RUN_2=PASS`.

No RUN #3 was executed.

## Comparison and interpretation

The complete V15.23C / R1 / D2 / D3 comparison is stored in `hardware/v15_23c_d3_ft/j3_kp060_comparison.csv`.

At Kp `0.50`, historical 10°/s runs had inconsistent center return, and the 5°/s D2 run reproduced an approximately 0.9° first-center plateau. At Kp `0.60` and restored 10°/s velocity, both independent D3 sessions passed all four acceptance metrics and neither reproduced the >0.75° plateau.

The supported conclusion is `J3_KP_060_EFFECT=SUPPORTED_REPEATABLE` under the current low-load configuration. This does not prove a universal physical root cause and does not establish installed-load behavior.

## Frozen authority after D3-FT

- `J3_KP_060_REPEATABILITY=PASS`
- `J3_LOW_LOAD_BIDIRECTIONAL_5DEG=PASS`
- `J3_LOW_LOAD_MOTION_AUTHORITY=PASS`
- Frozen low-load control: Kp `0.60`, Kd `0.05`, Tff `0`, velocity `10°/s`, acceleration `30°/s²`, command rate `100 Hz`
- `J3_INSTALLED_LOAD_HOLD=NOT_TESTED`
- `J3_INSTALLED_LOAD_LOCAL_MOTION=NOT_TESTED`
- `J3_INSTALLED_LOAD_GRAVITY_BEHAVIOR=NOT_TESTED`
- `J3_LOCAL_MOTION_AUTHORITY=NOT_GRANTED_FOR_FINAL_ASSEMBLY`
- `J2_ACTIVE_WORK=NO`

Final safe state: final J3 BRAKE passed and the operator confirmed GO 24V power off.
