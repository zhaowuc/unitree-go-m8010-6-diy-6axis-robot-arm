# V15.19A J1 Encoder Sign Mapping

## Result

V15.19A-2B establishes the J1 encoder position sign without active motor motion:

- `J1_SIGN_MAPPING = PASS`
- `J1_RELATIVE_POSITION_MAPPING = PASS`
- `raw_to_ros_sign = +1`
- `J1_ABSOLUTE_ZERO = PENDING_PHYSICAL_ALIGNMENT`
- `J1_VELOCITY_FEEDBACK = UNTRUSTED_PENDING_SEMANTICS_VALIDATION`

The temporary relative zero is the operator-established
`J1_TEMP_REFERENCE_V2_SCREW_WITNESS`. It is not CAD mechanical zero and must
not be written to motor memory, URDF, or a ROS absolute-zero authority.

## Frozen J1 axis and ROS positive direction

The frozen V15.13 axis authority and URDF agree:

- parent: `base_link`
- child: `link1`
- J1 axis in the parent frame: `[0, -2.7755575615628914e-17, 1]`
- canonical direction: `base_link +Z`

By the right-hand rule, viewed from directly above the robot while looking
downward along `-base_link Z`, ROS `+J1` is counterclockwise motion of the J1
rotating part relative to the fixed base.

Authority sources:

- `V15_13_真实关节轴与相机上置零位.json`
  (`sha256:477be6df2a356fdf55e4b4704da4b2ed6c332446c2d651c54bad369ebf5be39f`)
- `完整工程_V15_13_交付/ros2_ws/src/go_m8010_arm_description/urdf/go_m8010_arm_v15_13.urdf.xacro`
  (`sha256:fec9863b45addbea15f66ff617e42d6c9a2c06170c240e8ce51361e2b3fe0fdf`)

## Position quantities

### RAW_POSITION

`data.q` is the persistent absolute raw position signal returned by the
GO-M8010-6 SDK. The sign test found that its displacement increases when J1 is
manually moved in ROS `+J1`, so `raw_to_ros_sign = +1`.

The SDK scale check is essential. At SDK commit
`5b79a42d81cd69adac1367db79efdb972a898fd1`,
`queryGearRatio(MotorType::GO_M8010_6)` returns
`6.3299999237060547`. The manual sign movement produced a raw displacement of
`1.7186378240585327 rad`, while the observed J1 output displacement was
`1.7186378240585327 / 6.3299999237060547 = 0.2715067685265174 rad`
or `15.55619194580482 deg`. This establishes that raw displacement uses the
motor-side angular scale.

### ROS SIGN

The operator moved J1 counterclockwise in the frozen ROS-positive direction.
The corresponding raw displacement was positive. Therefore:

```text
raw_to_ros_sign = +1
```

### RELATIVE_ROS_POSITION

For a continuous, locally unwrapped raw position series, the output-joint
relative position is:

```text
q_j1_relative =
    raw_to_ros_sign
    * unwrap_continuous(q_raw - q_raw_reference)
    / gear_ratio_motor_to_output
```

with:

```text
q_raw_reference = 1.0791579484939575 rad
raw_to_ros_sign = +1
gear_ratio_motor_to_output = 6.3299999237060547
```

The division by the gear ratio is mandatory. Omitting it would overstate J1
output motion by a factor of about 6.33.

`q_j1_relative = 0` means only that the screw witness is aligned. This mapping
does not establish absolute CAD pose.

### ABSOLUTE_ZERO

No CAD mechanical-zero alignment was performed. No motor zero, EEPROM value,
URDF authority, or ROS zero authority was modified. The only valid status is:

```text
J1_ABSOLUTE_ZERO = PENDING_PHYSICAL_ALIGNMENT
```

## Measurement evidence

All measurements were BRAKE-only at 50 Hz for 5 seconds, with 250/250 valid
responses, zero `merror`, zero gains, and zero commanded position, velocity,
and torque.

| Stage | Median raw position | Relevant result |
|---|---:|---|
| Screw reference V2 | `1.0791579484939575 rad` | temporary relative zero |
| Manual ROS `+J1` position | `2.7977957725524902 rad` | raw delta `+1.7186378240585327 rad`; output motion `+15.55619194580482 deg` |
| Return to screw reference | `1.0979492664337158 rad` | raw error `0.018791317939758304 rad`; output error `0.1700889766845781 deg`; PASS |

The required raw return-error limit was `< 0.02 rad`. The V2 screw-witness
return passed. An earlier run used an imprecise visual return reference and is
classified `INVALIDATED_BY_IMPRECISE_RETURN_REFERENCE`; it is not part of the
final sign authority.

Power-cycle persistence remains `PASS_HIGH_CONFIDENCE` with 3/3 valid cycles
and a maximum raw-position delta of `0.00019168853759765625 rad`. Its evidence
is `hardware/v15_19a/j1_power_cycle_persistence_retest.json`.

## Why velocity is not published

The position result does not validate `data.dq`. Static tests repeatedly show
near-zero finite-difference position velocity while `data.dq` reports varying
nonzero medians. Applying the position sign or the 6.33 position scale to
`data.dq` would be an unsupported inference.

Until a future controlled low-speed test compares continuous raw-position
finite differences against `data.dq`, the frozen status is:

```text
J1_VELOCITY_FEEDBACK = UNTRUSTED_PENDING_SEMANTICS_VALIDATION
data.dq used as ROS velocity = NO
```

## Safety record

- FOC used: NO
- calibration used: NO
- active motor motion used: NO
- motor internal zero modified: NO
- ID or EEPROM modified: NO
- ROS2 started: NO
- frames sent while J1 power was off: NO

Full-range raw wrapping and multi-turn behavior also remains to be validated
before unrestricted continuous-joint deployment.
