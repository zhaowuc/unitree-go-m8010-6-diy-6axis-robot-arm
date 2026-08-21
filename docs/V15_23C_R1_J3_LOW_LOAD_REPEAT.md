# V15.23C-R1 J3 Low-Load Bidirectional Exact Repeat

## Result

`FINAL TASK RESULT = FAIL`

This task executed the single authorized exact repeat of the V15.23C low-load bidirectional route. The original V15.23C runner and trajectory implementation were reused without gain, speed, acceleration, rate, threshold, or HOLD-duration changes. No automatic retry followed.

The V15.23C first-center failure was not reproduced: it passed in R1. R1 instead failed the final-center criterion. The frozen classification is therefore:

`J3_LOW_LOAD_BIDIRECTIONAL_REPEATABILITY = INCONSISTENT`

This evidence does not identify backlash, friction, gain, or trajectory as the specific physical cause.

## Source and configuration

- Authoritative source commit: `5fbb4bec58d4a06293912eb07f147e9e11c19c9d`
- Source tree: `b12d1b12ee21efd502e35c6826258a9df819ab73`
- Branch: `agent/v15-23c-r1-j3-low-load-repeat`
- Mechanical configuration unchanged: yes; upper arm remains removed.
- J3 ID3 connected; J4/J5 physically disconnected.
- Existing sign authority retained: `raw_to_ros_sign=+1`.
- Original controller source SHA256: `150f1b8b384aab96c27df2a23827df32e1e8513946af43afceb90522986624b2`.

The controller remained GO-M8010-6 FOC mixed position with SDK-literal `Kp=0.50`, `Kd=0.05`, `Tff=0`, gear ratio `6.3299999237060547`, nominal `100 Hz`, `10 deg/s`, and `30 deg/s^2`. Preflight HOLD remained `2.0 s`; endpoint/first-center HOLD remained `0.4 s`; final-center HOLD remained `0.5 s`.

## New baseline

The R1 BRAKE baseline was `100/100` valid. The newly measured session median/std were `2.7884001731872559 / 0.00011308245927179865 rad`. No old raw value was used as a blind target.

## Comparison

| Metric | V15.23C | V15.23C-R1 |
|---|---:|---:|
| +5 actual / error (deg) | 4.7086849937 / 0.2913150063 | 4.8267059163 / 0.1732940837 |
| First center error (deg) | 0.8435013972 — FAIL | 0.5276229481 — PASS |
| -5 actual / error (deg) | -4.5160348121 / 0.4839651879 | -4.3945416069 / 0.6054583931 |
| Final center error (deg) | 0.1674853042 — PASS | 0.8955705303 — FAIL |
| Max qdot fast (deg/s) | 15.8642726206 | 15.5063979870 |
| Max qdot slow (deg/s) | 13.0976497160 | 12.8128773053 |
| SDK dq max (rad/s) | 2.0125875473 | 2.2089374065 |
| Tau range | -0.06640625..0.05859375 | -0.06640625..0.06640625 |
| Temperature (C) | 34..34 | 33..33 |
| merror frames | 0 | 0 |
| Invalid frames | 0 | 0 |
| Operator observation | SAFE | SAFE |

R1 preflight HOLD passed with maximum drift `0.0034701246 deg` and final error `0.0017350623 deg`. The actual command rate was `99.9986916391 Hz` and maximum TX jitter was `0.245939 ms`.

## Safety and authority

The operator reported the one R1 route safe. J3 finished with five consecutive valid BRAKE frames, and the operator confirmed GO 24 V off. J2 active work was not used. The upper arm was not reassembled.

Because the final-center error was `0.8955705303 deg > 0.75 deg`:

- `J3_LOW_LOAD_BIDIRECTIONAL_5DEG = FAIL`
- `J3_LOW_LOAD_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_INSTALLED_LOAD_HOLD = NOT_TESTED`
- `J3_INSTALLED_LOAD_LOCAL_MOTION = NOT_TESTED`
- `J3_INSTALLED_LOAD_GRAVITY_BEHAVIOR = NOT_TESTED`
- `J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED_FOR_FINAL_ASSEMBLY`

V15.23C remains unchanged. No J3 PASS config was created or updated.
