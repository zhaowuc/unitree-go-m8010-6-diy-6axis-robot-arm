# V15.21F — J6 startup and READY reference authority

## Result

`FINAL TASK RESULT = PASS`

- `J6_POSVEL_LOCAL_MOTION = PASS` remains frozen from V15.21E.
- `J6_PROTOCOL_TO_ROS_SIGN = -1` remains frozen.
- `J6_STARTUP_STATE_AUTHORITY = PASS`.
- `J6_READY_REFERENCE = FROZEN`.
- `J6_READY_RECOVERY_MOTION = NOT_TESTED`.
- `J6_CAD_ZERO = PENDING`.
- `J6_ROS_ZERO = PENDING`.
- `J6_PHYSICAL_LIMITS = PENDING`.
- `J6_FULL_COMMISSIONING = NOT_COMPLETE`.

No active motor motion, FC enable, zero write, ID write, parameter write, or
EEPROM/flash save was used in V15.21F.

## Power-cycle observation

The pre-power motor state was DISABLED. RID7/RID8/RID21 returned Master ID 0,
ESC ID 1, and PMAX 12.5. RID10 returned 2 / POS_VEL. The pre-power capture was
200/200 valid frames with a median position of `1.110284580758373 rad`.

The operator confirmed J6 24 V power off. During a 3.301-second read-only probe,
the USB adapter remained present while J6 returned zero normal feedback frames.
The operator reported that the physical pose did not move while power was off.

After power-on, J6 remained DISABLED and identity was unchanged. RID10 returned
1 / MIT, so the observed classification is:

`CTRL_MODE_POWER_CYCLE_PERSISTENCE = DOES_NOT_PERSIST`

No runtime mode write was performed. The post-power capture was 200/200 valid
frames with a median of `1.1099031052109556 rad`. The median difference was
`-0.0003814755474174092 rad` (`-0.02185693885446026 deg`). With the operator's
no-motion report, this supports only:

`J6_POSITION_READBACK_POWER_CYCLE_LOCAL_REPEATABILITY = OBSERVED_IN_THIS_TEST`

It does not establish single-turn, multi-turn, permanent-absolute, CAD-zero, or
ROS-zero authority.

## Startup policy

Every future startup must follow this order:

1. Discover the DM adapter and J6 ID1.
2. Require the motor to be DISABLED.
3. Read and verify identity and RID10.
4. If RID10 is not POS_VEL, perform a gated runtime switch only under explicit
   operator or system authority, while DISABLED, and verify RID10 readback.
5. Read the current position through strict feedback validation.
6. Preload that current position with velocity target zero.
7. Enable only under a separate motion authority, HOLD CURRENT, and validate.
8. A future validated trajectory may then move smoothly toward the READY
   reference.

`POWER ON -> blindly send saved READY raw` is forbidden.

## J6_READY_REFERENCE_V1

The operator manually set the desired physical READY wrist pose while the motor
was DISABLED. The capture returned 200/200 valid frames:

- median: `1.0381857022964827 rad`
- mean: `1.0383287556267642 rad`
- standard deviation: `0.00024556330948317094 rad`
- minimum: `1.0378042267490653 rad`
- maximum: `1.0389486533913175 rad`
- `protocol_to_ros_sign = -1`

`J6_READY_REFERENCE_V1` is a DM-G6220 protocol-position physical-pose reference.
It is not CAD zero, ROS zero, motor internal zero, or authority to send the raw
value blindly at startup. Automatic recovery from the current pose to this
reference was not tested.

The original J3/J4/J5 `READY_POSE_V1` authority was not modified.
