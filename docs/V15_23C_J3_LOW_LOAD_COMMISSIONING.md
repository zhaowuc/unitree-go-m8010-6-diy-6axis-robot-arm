# V15.23C J3 Low-Load Commissioning

## Result

`FINAL TASK RESULT = FAIL`

The upper-arm removal and the J3 communication, HOLD, and sign-probe stages passed. The bidirectional test failed one mandatory numeric acceptance criterion: the first center return error was `0.8435013972 deg`, exceeding the `0.75 deg` limit. No retry, gain tuning, calibration, ID write, zero write, or additional hardware motion was used after this result.

## Authority boundary

- Complete assembly J3: the earlier V15.22C partial attempts remain preserved but are not authority.
- Disassembled low-load J3: HOLD and sign are individually verified; the complete low-load motion authority is **not granted** because the bidirectional acceptance did not fully pass.
- Final reassembly: installed-load HOLD, motion, gravity behavior, and final local-motion authority remain untested/not granted. The operator expects possible mechanical reassembly alignment error and will direct later precision alignment; no such correction was performed here.
- The raw references in this task are traceability evidence only. They are not CAD zero, ROS zero, motor internal zero, READY pose, or direct motion targets.

## Mechanical removal evidence

The GO 24 V supply was off before disassembly. External support, flange/index marks, the unique-hole reference, cable labels/orientation, strain relief, and fastener grouping were confirmed. The upper arm was removed only to the required J3 test stage. J2 was not disassembled beyond that stage. Post-removal photographs are held locally by the operator.

Post-disassembly topology was J3 connected, with J4 and J5 physically disconnected. The V15.23C controller therefore addressed only J3 and never polled or enabled J4/J5.

## Communication and traceability

- Post-disassembly J3 BRAKE baseline: `200/200` valid.
- Post-disassembly raw median: `2.6286740303039551 rad`.
- Pre-disassembly reference: `2.1815176010131836 rad`.
- Difference: `+0.44715642929077148 rotor rad`, or `+4.0474212463341175 deg` output-equivalent. This is traceability only.
- Low-load session median/std: `2.6286740303039551 / 0.00013108668599592783 rad`.

## Motion evidence

Control used GO-M8010-6 FOC mixed position mode, SDK-literal `Kp=0.50`, `Kd=0.05`, `Tff=0`, gear ratio `6.3299999237060547`, nominal `100 Hz`, `10 deg/s`, and `30 deg/s^2`.

HOLD1 passed: maximum output drift `0.0121497521 deg`, final median error `0.0086774695 deg`, and maximum slow position-derived velocity `0.0316001682 deg/s`. The operator reported no abnormal behavior.

The protocol-positive `+3 deg` sign probe passed numerically: actual `+2.4333126487 deg`, endpoint error `0.5666873513 deg`, return error `0.4043946806 deg`. URDF and MuJoCo both define J3 axis as approximately local `+Y`: `[0, 1, -2.77555756156289e-17]`. From the supplied photo-side observation, protocol positive moved clockwise/up and returned without abnormal sound. The operator confirmed protocol positive equals ROS positive, so `raw_to_ros_sign=+1` for this low-load sign evidence.

HOLD2 passed. In the bidirectional test, `+5 deg` and `-5 deg` endpoints and the final return passed, while the first center return failed:

- `+5`: actual `+4.7086849937 deg`, error `0.2913150063 deg` — PASS.
- First center: error `0.8435013972 deg` against `0.75 deg` — FAIL.
- `-5`: actual `-4.5160348121 deg`, error `0.4839651879 deg` — PASS.
- Final center: error `0.1674853042 deg` — PASS.

The operator observed clockwise `+5`, return, counterclockwise `-5`, return, with no collision, abnormal oscillation, sound, loss of control, cable pull, or structural instability. This safe physical observation does not override the numeric failure.

The run rate was `99.9990428159 Hz`, maximum TX jitter `0.189459 ms`, maximum fast/slow position-derived speed `15.8642726206/13.0976497160 deg/s`, maximum SDK `dq` `2.0125875473 rad/s`, torque range `[-0.06640625, 0.05859375]`, temperature `34..34 C`, nonzero-merror frames `0`, and invalid frames `0`.

## Final safe state

J3 finished with five consecutive valid BRAKE frames (`correct=true`, `merror=0`). The operator then confirmed GO 24 V power off. J2 FOC/dual enable, J4/J5 authority, J6 authority, READY_POSE, and simulation authority were untouched. Reassembly was not performed.

## Frozen status

- `J3_LOW_LOAD_HOLD = PASS`
- `J3_LOW_LOAD_SIGN = PASS`
- `J3_RAW_TO_ROS_SIGN = +1`
- `J3_LOW_LOAD_BIDIRECTIONAL_5DEG = FAIL`
- `J3_LOW_LOAD_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_INSTALLED_LOAD_HOLD = NOT_TESTED`
- `J3_INSTALLED_LOAD_LOCAL_MOTION = NOT_TESTED`
- `J3_INSTALLED_LOAD_GRAVITY_BEHAVIOR = NOT_TESTED`
- `J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED_FOR_FINAL_ASSEMBLY`
- `J3_CAD_ZERO = PENDING`
- `J3_ROS_ZERO = PENDING`
- `J3_PHYSICAL_LIMITS = PENDING`
- `J3_FULL_RANGE = NOT_TESTED`
- `motor internal zero = UNCHANGED`
