# V15.24E-FT J2 pose-aware gravity compensation

## Result

`FINAL_TASK_RESULT = FAIL`. `J2_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`.

## Session-local anchor and mapping

The operator matched a rendered MuJoCo pose to the unchanged physical assembly at the stable startup boundary B and explicitly authorized recording the current sliders; the anchor stores the exact gate semantic `MUJOCO_PHYSICAL_POSE_MATCHED=YES` with entry method `USER_CHAT_EXPLICIT_BOUNDARY_B_PLUS_5DEG_STAGE_ROUTE_AUTHORIZATION`. This is a session-local model/encoder anchor, not a permanent joint zero. Test center C is a relative command point at B+5 degrees, not a second model anchor. `CAD_ZERO = PENDING`, `ROS_ZERO = PENDING`, and `PERMANENT_ZERO_MODIFIED = NO`.

- Frozen model SHA256: `5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9`; MuJoCo `3.11.0`; runtime gravity `[0, 0, -9.81] m/s²`.
- Boundary-B q_anchor_model J2..J6: `2.96705972839, -2.96705972839, -0.216595360172, 0.815068760681, 0.00959931088597 rad`.
- Boundary-B J2 gravity torque / uncertainty envelope: `-0.734881534391 / [-1.25596211323, -0.655793849487] N·m`; sign robust: `YES` over 405 required samples spanning B-relative J2 0..10 degrees and all J3..J6 +/-5-degree corners.
- Real mapping per run uses 50 valid BRAKE frames: `qA=-1*(rawA-A0)/G`, `qB=+1*(rawB-B0)/G`, `qJ2=(qA+qB)/2`, with `G=6.3299999237060547`.

## Boundary-segmented route and protection

The exact command route is startup B -> C, then the formal C-relative `0 -> +5 -> 0 -> -5 -> 0`, then C -> B and BOTH BRAKE. In B-relative coordinates this is `0 -> 5 -> 10 -> 5 -> 0 -> 5 -> 0` degrees. `MOVE_TO_TEST_CENTER_PROFILE/ENDPOINT` and `RETURN_TO_BOUNDARY_PROFILE/ENDPOINT` are separate audited phases; every profile has 85 rows, the stage endpoint has 40 rows, and the return endpoint has 50 rows.

All active logical commands are audited fail-closed inside `[0, 10]` degrees relative to B. Sampled real qA, qB, and paired qJ2 are independently reconstructed from raw feedback and guarded inside `[-0.5, 12]` degrees after the B reference is established, including terminal BRAKE feedback. Stage-to-C and return-to-B errors must each be <=0.75 degrees, and their endpoint e_sync must be <=0.3 degrees.

`NO_ADDITIONAL_BOUNDARY_MARGIN = YES`: the lower command is exactly B (`0 deg`), with no inward offset. The `-0.5 deg` sampled-feedback abort is a reactive protection tolerance, not command margin and not proof that the mechanism can never cross B between 100 Hz samples. Therefore `ACTUAL_NO_BOUNDARY_CROSSING_ABSOLUTE_GUARANTEE = NO`.

## Real-time gravity and A/B torque conversion

At 100 Hz, the runner maps the latest measured relative J2 pose onto the frozen model anchor, holds J3..J6 at the visual anchor, sets qvel/qacc to zero, calls MuJoCo forward dynamics, and reads J2 `qfrc_bias`. It then commands `Tff_A=-alpha*tau_g/(2G)` and `Tff_B=+alpha*tau_g/(2G)`, applies the per-level cap, and audits the actual Q8 truncation and decoded values. The analyzer verified the prior-cycle pose mapping, formula, cap flags, opposite symmetry, Q8 encoding, feedback validity, merror, trajectory, and terminal brake for every CSV row.

## LEVEL-A alpha=.25

- Result: `FAIL` (numeric `FAIL`), alpha `0.25`.
- Operator observation: `SAFE`; drive-force observation: `HIGHER_DRIVE_FORCE_REQUIRED`.
- Model J2 torque range: `-0.760742346826` to `-0.734477252236 N·m`.
- J2A/J2B decoded Tff ranges: `0` to `0.01171875` / `-0.01171875` to `0 N·m`; clamp used: `NO`.
- Gravity hold displacement / max e_sync: `-0.00954715868952 / 0.0242995042595 deg`.
- Stage B->C actual/error/e_sync: `0.236041845195 / 4.76395815481 / 0.149260676031 deg`.
- C-relative +5 actual/error: `-4.2224503923 / 9.2224503923 deg` (absolute from B `0.777549607701 deg`); first-center error: `4.51837022994 deg`.
- C-relative -5 actual/error: `-4.90627750264 / 0.0937224973568 deg` (absolute from B `0.0937224973568 deg`); final-center error: `4.72751105645 deg`.
- Return C->B actual/error/e_sync: `0.0859125589985 / 0.0859125589985 / 0.0329769737646 deg`; acceptance `PASS`.
- Max route e_sync / directional asymmetry: `0.277696288465 / 0.683827110344 deg`.
- B-relative command min/max: `0 / 10 deg`; negative command rows: `0`.
- Sampled real qA/qB/qJ2 minima: `-0.00173506229348 / -0.022564441966 / -0.0121497521297 deg`; sampled boundary protection `PASS`.
- Valid/merror/final BOTH BRAKE/100 Hz: `PASS / PASS / PASS / PASS`.

## LEVEL-B alpha=.50

Not executed.


## Historical comparison and directional asymmetry

- V15.24B authoritative zero-Tff: +5/-5 errors `4.770899483 / 4.70408016236 deg`; asymmetry `0.0668193206382 deg`.
- V15.24D fixed Tff=.05: +5/-5 errors `4.55134828272 / 4.98958638918 deg`; asymmetry `0.43823810646 deg`.
- Auto-gravity assessment uses the C-relative +5/-5 endpoint metrics: better than zero-Tff `NO`, better than fixed Tff=.05 `NO`, directional asymmetry reduced `NOT_SUPPORTED`.
- The frozen Level-B eligibility threshold is a >=`0.05` degree endpoint-error improvement in both directions versus both historical runs, while Level A remains a formal position FAIL and all safety/sync/sign/center/clamp audits pass.

## Alpha strategy and scope

Level A uses alpha `.25`, a 1.0 s quintic ramp, and a `.30 N·m` per-motor cap. Level B is permitted only by the offline eligibility gate and uses alpha `.50`, a 1.5 s quintic ramp, and a `.60 N·m` cap. Alpha above `.50` is prohibited. Repeat status: `NO` / `NOT_EXECUTED`. Best validated alpha: `NOT_VALIDATED`.

This task tested position-control gravity feedforward only. `ZERO_GRAVITY_TEACH_MODE = NOT_IMPLEMENTED`. J3 work was not used. A full-state gravity controller is worth considering only if the final repeated authority is PASS; otherwise the unresolved torque/current limit, static friction/brake, model-load mismatch, and anchor accuracy must be isolated in a later task.
