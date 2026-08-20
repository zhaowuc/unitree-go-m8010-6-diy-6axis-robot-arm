# V15.20A-R2 J345 Shared Bus and READY Pose

## Result

`J345_SHARED_BUS = PASS`

`READY_POSE_V1 = FROZEN`

J3, J4, and J5 were commissioned as IDs 3, 4, and 5, restored to normal motor mode, assembled on one RS485 multidrop bus, and verified with BRAKE-only traffic. No FOC trajectory or active motor motion was used.

## Transport

- Channel: `CHANNEL_2`
- FTDI serial/interface: `FTASQA6F / 02`
- Stable port: `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0`
- Capture target: `/dev/ttyUSB2`
- Baudrate: `4000000`
- A/B polarity correction: `YES`
- Termination status: `UNKNOWN`

The shared-bus host entry was wired to the R1 working differential-pair orientation. Termination was not inferred from successful communication.

## ID ledger

| Joint | Original ID | Final ID | Motor mode restored |
|---|---:|---:|---|
| J3 | 0 | 3 | PASS |
| J4 | 0 | 4 | PASS |
| J5 | 4 | 5 | PASS |

J4 and J5 each encountered a fail-closed post-change validation interruption. Neither ID write was blindly repeated. Subsequent isolated recovery classification and completion evidence established normal BRAKE communication at the intended final ID.

## Shared discovery

Only IDs 3, 4, and 5 were queried, three times each. All 9 responses were full-valid with independently verified CRC, returned ID, BRAKE mode, `merror=0`, and 33°C feedback.

- CSV: `/tmp/v15_20a_r2_j345_shared_discovery_20260820T060343Z.csv`
- SHA256: `3bf5c8601f2d372152352fe20ae1f3eab720ca5b8f1790fcf2484d1fd6aa5a98`

## 100 Hz shared-bus validation

The test used sequential `J3 → J4 → J5` BRAKE-only request/response transactions for 500 cycles per motor.

| Joint | Valid | Actual Hz | Max RTT (ms) | Temperature (°C) | Output span (rad) |
|---|---:|---:|---:|---:|---:|
| J3 | 500/500 | 99.9401820157541 | 1.584172 | 33 | 0.00012116786385688799 |
| J4 | 500/500 | 99.9421520803818 | 1.359348 | 33–34 | 0.00009087589789266599 |
| J5 | 500/500 | 99.9412011865315 | 1.641500 | 34 | 0.00012113019899401671 |

All 1500 frames passed independent CRC and health checks. No timeout, collision suspicion, returned-ID mismatch, non-BRAKE response, or nonzero `merror` was observed.

- CSV: `/tmp/v15_20a_r2_j345_perf100_20260820T061201Z.csv`
- SHA256: `1954b37b7592fde7251f0f93f69e1888acf9ee9752316e9e1bfa84441ab14280`

## READY_POSE_V1 raw reference

These values are local-nearest-turn unwrapped motor SDK references. They are not CAD zero, ROS zero, motor internal zero, or authorized motion targets.

| Joint | ID | Median (rad) | Population std (rad) | Min (rad) | Max (rad) | Actual Hz | Valid |
|---|---:|---:|---:|---:|---:|---:|---:|
| J3 | 3 | 2.1399083137512207 | 0.00014659267438217439 | 2.1397163867950439 | 2.140291690826416 | 99.8071337479865 | 200/200 |
| J4 | 4 | 0.49758619070053101 | 0.00011549145169938258 | 0.49739444255828857 | 0.49796968698501587 | 99.8139984196254 | 200/200 |
| J5 | 5 | 4.3298625946044922 | 0.00009727130911748581 | 4.3298625946044922 | 4.3302459716796875 | 99.8118698228696 | 200/200 |

The READY capture contained 600/600 full-valid frames at 34°C with `merror=0`.

- CSV: `/tmp/v15_20a_r2_j345_ready_pose_v1_20260820T061733Z.csv`
- SHA256: `f8532b1bf67c0fc9d2151ea5a195b6d9dd76bc0da8c7a41287254c3f82070b54`

## Safety and unresolved items

- FOC trajectory used: `NO`
- Active motor motion used: `NO`
- J1 authority modified: `NO`
- J3/J4/J5 sign mapping: `PENDING`
- CAD/ROS/motor internal zero: `PENDING / NOT ESTABLISHED`
- Termination status: `UNKNOWN`
- J1 remained physically disconnected throughout J345 R2.
- Full-bus collision absence is not mathematically proven; the observed fixed-ID response set was clean.
- FOC, load, dynamic motion, URDF direction, and sign behavior remain untested.

Raw CSV evidence remains outside Git and is backed up separately on the Windows execution host.
