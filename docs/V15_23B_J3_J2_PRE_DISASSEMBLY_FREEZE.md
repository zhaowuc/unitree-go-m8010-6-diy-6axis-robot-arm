# V15.23B — J3/J2 pre-disassembly authority freeze

## Result

`FINAL TASK RESULT = PASS`

`PRE_DISASSEMBLY_REFERENCE_V1 = FROZEN`

`READY_FOR_UPPER_ARM_DISASSEMBLY = YES`

This result authorizes only the transition to a future, separately controlled disassembly task. No upper-arm disassembly, structural-fastener loosening, or active motor motion occurred in V15.23B.

## 1. Why J3 cannot be commissioned in the current assembly

The operator confirmed that the installed upper assembly prevents J3 from obtaining safe, reasonable local motion clearance. The prior full-assembly route produced only unaccepted partial attempts and did not establish motion authority. Reliable HOLD, sign, and bidirectional testing requires removal of the structure above J3.

Classification: `BLOCKED_BY_MECHANICAL_CONFIGURATION`.

## 2. Is upper-assembly removal required?

YES.

- `J3_CURRENT_COMMISSIONING_PATH = BLOCKED_BY_UPPER_ASSEMBLY`
- `J3_UPPER_ASSEMBLY_REMOVAL_REQUIRED = YES`

## 3. Is J3 a motor failure?

NO. J3 communication passed, and neither motor failure, RS485 failure, nor controller failure is established. The blocker is mechanical configuration.

## 4. Current J3 motion authority

- `J3_LOCAL_MOTION = NOT_TESTED_MECHANICALLY_BLOCKED`
- `J3_ACTIVE_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_RAW_TO_ROS_SIGN = PENDING`
- `J3_CAD_ZERO = PENDING`
- `J3_ROS_ZERO = PENDING`

For traceability, partial V15.22C attempts did physically occur before this blocker freeze, but they failed acceptance and granted no authority. Their evidence is preserved on `agent/v15-22c-j3-local-motion` at commit `b1c9c7988fc28ff0ce3627553f36a8d8eb5b0e3e`. Therefore this document does not claim that no historical FOC attempt occurred; it states that no accepted J3 local-motion test exists. V15.23B itself used no FOC or active motion.

## 5. J2 blocker

The frozen J2 blocker remains unchanged:

- `J2_DISASSEMBLY_REQUIRED = YES`
- `J2_ACTIVE_MOTION_AUTHORITY = NOT_GRANTED`

No J2 FOC, dual enable, ID write, zero write, or active discovery occurred.

## 6. J3/J4/J5 pre-disassembly raw references

The authorized BRAKE-only retry captured 200/200 valid frames per motor at 4 Mbps on FTDI FTASQA6F CHANNEL_2 / interface 02:

| Joint | ID | Raw median (rad) | Raw std (rad) | State |
|---|---:|---:|---:|---|
| J3 | 3 | 2.1815176010131836 | 0.0001254396791324235 | BRAKE |
| J4 | 4 | 0.75567907094955444 | 0.00011262245151247535 | BRAKE |
| J5 | 5 | 5.0590806007385254 | 0.00011007519035668067 | BRAKE |

These are `PRE_DISASSEMBLY_REFERENCE_ONLY`. They are not CAD zero, ROS zero, motor zero, READY_POSE, startup targets, or blind motion targets.

The first capture after USB reconnection was preserved but not accepted because each joint returned only 196/200 valid frames. The explicitly authorized retry passed with 200/200 valid frames, no timeout, no returned-ID mismatch, merror 0, and BRAKE mode throughout.

## 7. Were J2A/J2B raw values safely captured?

NO. Status: `NOT_CAPTURED_SAFE_PATH_NOT_ESTABLISHED`.

The repository and current wiring authority do not establish a safe J2 read-only bus path, J2A/J2B IDs, or physical-side mapping. No speculative discovery was performed, and no fake J2 CSV was created.

## 8. Unique flange and hole references

The operator confirmed:

- J3 flange index marked
- J2 flange index marked
- J3 unique-hole reference marked
- J2 unique-hole reference marked
- Upper-arm installation orientation marked
- Fastener groups labeled and grouping hardware prepared

The marks are intended to prevent 60°, 90°, 120°, or 180° symmetric-hole reassembly errors. No reliable fastener torque specification was found; `FASTENER_TORQUE_SPEC = PENDING`.

## 9. Cable reference

The operator confirmed the J2/J3 cable-routing reference is complete, including connector direction, RS485 routing, power routing, strain relief, and cable-clamp positions.

## 10. Photo reference

The operator confirmed the complete pre-disassembly photo set. Authority location is `OPERATOR_LOCAL`; no photos were copied into the repository, and this repository does not claim to contain them.

## 11. May the next upper-arm disassembly stage begin?

Authority result: YES, in a future separate task only.

All required encoder, marking, photo, cable, and hardware-grouping gates are complete. V15.23B stops here and performs no disassembly.

## Frozen future authority layers

After removal, successful low-load testing may establish only:

- `J3_LOW_LOAD_HOLD = PASS`
- `J3_LOW_LOAD_SIGN = PASS`
- `J3_LOW_LOAD_BIDIRECTIONAL_5DEG = PASS`

It must not establish installed-load authority. After full reassembly, J3 requires a separate current-position HOLD and ±2° revalidation before `J3_INSTALLED_LOAD_LOCAL_MOTION` or `J3_LOCAL_MOTION_AUTHORITY` can become PASS.
