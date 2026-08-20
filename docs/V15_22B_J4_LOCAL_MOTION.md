# V15.22B — J4 local bidirectional motion

## Result

`FINAL TASK RESULT = PASS`

- `J4_LOCAL_MOTION = PASS`
- `J4_BIDIRECTIONAL_5DEG = PASS`
- `J4_LOCAL_MOTION_AUTHORITY = PASS`
- `J4_FULL_RANGE = NOT_TESTED`
- `J4_PHYSICAL_LIMITS = PENDING`
- `J4_CAD_ZERO = PENDING`
- `J4_ROS_ZERO = PENDING`
- `J4_FULL_COMMISSIONING = NOT_COMPLETE`

Only J4 entered FOC. J3 and J5 remained in BRAKE throughout. J1, J2, J6,
the gripper, `READY_POSE_V1`, and simulation authority were not accessed or
modified.

## Shared bus and local-reference semantics

The authoritative transport was FTDI `FTASQA6F`, CHANNEL_2/interface `02`, at
4 Mbps. After an initial all-timeout attempt, the operator confirmed 24 V and
CHANNEL_2 connectivity and explicitly authorized one BRAKE-only retry. The
retry returned 200/200 valid BRAKE frames for each of J3/ID3, J4/ID4, and
J5/ID5, with correct IDs, CRC, `merror=0`, and reasonable temperatures.

The initial J4 session median was `0.60975879430770874 rad`, with population
standard deviation `0.0001153640391785415 rad`. Every active phase re-read the
current encoder position before its first FOC frame. No saved raw value was
used as an initialization target.

All session and passive raw references in this task are local-only. They are
not startup angles, READY references, CAD zero, ROS zero, motor internal zero,
or permanent multi-turn authority.

## HOLD and gravity behavior

J4 directly reused the frozen J1/J5 controller template without tuning:

- FOC mixed position control
- `Kp=0.50`, `Kd=0.05`, `Tff=0`
- 100 Hz rest-to-rest trapezoidal `q+dq`
- output-equivalent velocity `10 deg/s`
- output-equivalent acceleration `30 deg/s²`

The first 1.5-second HOLD passed with maximum drift
`0.0034712036058077152 deg`, final 500 ms median error `0 deg`, and maximum
slow position-derived speed `0.029929562016698281 deg/s`. The operator observed
no gravity sag. The second HOLD also passed.

The narrow classification is
`J4_GRAVITY_COMPENSATION_FOR_LOCAL_TEST = NOT_REQUIRED`. This does not establish
that gravity feedforward is unnecessary outside the tested posture and local
±5° range.

## Direction and sign authority

The operator initially allowed only clockwise active motion. A BRAKE-only
passive mapping therefore established that a clockwise manual movement made
raw feedback decrease. The large passive pose change was used only for sign
information and was never treated as a zero or active target. The operator
then confirmed that the new stable pose permitted bidirectional local motion.

Two identical +3° repeats were explicitly requested because the first motions
were not visually clear. The final observed motor-positive motion was
counterclockwise followed by a return to the current session. URDF and MuJoCo
both define J4 axis as approximately joint-frame +Y, and CAD places the visible
output flange on the positive-axis side. The operator confirmed:

`J4_PROTOCOL_POSITIVE_IS_ROS_POSITIVE = YES`

Therefore `J4_RAW_TO_ROS_SIGN = +1`. Authority combines operator physical
direction confirmation with URDF/MuJoCo J4-axis review.

## Bidirectional ±5°

The motor-protocol route `0 -> +5° -> 0 -> -5° -> 0` produced:

- +5° delta: `5.8021130505416592 deg`
- +5° endpoint error: `0.80211305054165916 deg`
- center return error: `0.9163977519332368 deg`
- -5° delta: `-5.0115461595642419 deg`
- -5° endpoint error: `0.011546159564241876 deg`
- final return error: `0.029504691139945961 deg`
- actual command rate: `99.997477286707692 Hz`
- maximum fast/slow derived speed: `14.553149598405476 / 13.802704964379792 deg/s`
- torque feedback range: `-0.1171875 .. 0.06640625`
- temperature: `34 .. 35 C`
- merror and invalid frames: `0 / 0`

The operator confirmed there was no collision, sustained oscillation, obvious
abnormal sound, uncontrolled motion, cable pull, or obvious sag. J4 finished
with five verified BRAKE frames; J3 and J5 remained in BRAKE.
