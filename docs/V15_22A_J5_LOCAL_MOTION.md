# V15.22A — J5 local bidirectional motion

## Result

`FINAL TASK RESULT = PASS`

- `J5_LOCAL_MOTION = PASS`
- `J5_BIDIRECTIONAL_5DEG = PASS`
- `J5_LOCAL_MOTION_AUTHORITY = PASS`
- `J5_FULL_RANGE = NOT_TESTED`
- `J5_PHYSICAL_LIMITS = PENDING`
- `J5_CAD_ZERO = PENDING`
- `J5_ROS_ZERO = PENDING`
- `J5_FULL_COMMISSIONING = NOT_COMPLETE`

Only J5 entered FOC. J3 and J4 remained in BRAKE throughout. J6 was not
accessed. The frozen `READY_POSE_V1` raw reference was neither modified nor
used as a command target.

## Shared-bus and session authority

The authoritative FTDI path resolved to `/dev/ttyUSB2` with serial
`FTASQA6F`, interface `02`, and 4 Mbps. J3/ID3, J4/ID4, and J5/ID5 each returned
200/200 full-valid BRAKE frames with correct IDs, CRC, `data.correct=true`, and
`merror=0`.

The initial J5 session reference was captured from current feedback:

- median: `4.4880547523498535 rad`
- population standard deviation: `0.00012702168299342685 rad`

This is a session-local motor-side reference, not READY, CAD zero, ROS zero, or
motor internal zero.

## Does the J1 controller template apply directly to J5?

Yes, for the tested current posture and local ±5° range. J5 used the frozen J1
template without gain changes or feedforward:

- FOC mixed position control
- `Kp = 0.50`, `Kd = 0.05`, `Tff = 0`
- 100 Hz rest-to-rest trapezoidal `q+dq`
- output-equivalent velocity `10 deg/s`
- output-equivalent acceleration `30 deg/s²`

The first one-second current-position HOLD passed with maximum drift
`0.02776962884646172 deg`, final median error `0.024299504259493242 deg`, and
maximum slow position-derived speed `0.1783798955315092 deg/s`.

## Gravity behavior

The operator observed no gravity sag. The operator also clarified that the
current physical posture is not inherently sag-prone. Therefore the narrow
classification is:

`J5_GRAVITY_COMPENSATION_FOR_LOCAL_TEST = NOT_REQUIRED`

This does not establish that gravity feedforward is unnecessary at other J5
angles, with other arm configurations, or during full-range motion. No gravity
compensation or automatic gain adjustment was implemented.

## Sign authority

The repeated motor-positive +3° probe produced `+2.9817649764076664 deg`, with
endpoint error `0.018235023592333643 deg` and return error
`0.2256271553586623 deg`. The operator observed counterclockwise rotation from
the upper J5 output-flange side looking toward the joint center.

URDF and MuJoCo both define the J5 axis as approximately joint-frame +X:
`[1, 0, -1.59232002583387e-13]`. The visible J5 output flange and link5 geometry
are on the +X side. From +X looking toward the joint center, ROS positive is
counterclockwise by the right-hand rule. Therefore:

`J5_PROTOCOL_POSITIVE_IS_ROS_POSITIVE = YES`

`J5_RAW_TO_ROS_SIGN = +1`

Authority combines the operator's physical direction confirmation and the
URDF/MuJoCo J5-axis review. The first numerically valid probe is retained as
evidence; one identical +3° repeat was explicitly requested by the operator to
distinguish direction.

## Bidirectional ±5°

The second current-position HOLD passed. The motor-protocol route
`0 -> +5° -> 0 -> -5° -> 0` then produced:

- +5° delta: `4.564633820655985 deg`
- +5° endpoint error: `0.43536617934401534 deg`
- center return error: `0.23257172060795617 deg`
- -5° delta: `-4.958609495311545 deg`
- -5° endpoint error: `0.04139050468845529 deg`
- final return error: `0.19438308584987515 deg`
- actual command rate: `99.99623298263204 Hz`
- maximum fast/slow derived speed: `15.580329317598716 / 13.481965505603865 deg/s`
- torque feedback range: `-0.0546875 .. 0.0703125`
- temperature: `36 C`
- merror and invalid frames: `0 / 0`

The operator confirmed the motion was safe, with no collision, sustained
oscillation, obvious abnormal sound, uncontrolled motion, cable pull, or
obvious sag. J5 finished with five verified BRAKE frames, and J3/J4 remained in
BRAKE.
