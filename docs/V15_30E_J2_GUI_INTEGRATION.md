# V15.30E J2 GUI control integration

## Result

`J2_GUI_POSITION_CONTROL = ENABLED_IN_VERIFIED_PLUS_MINUS_5_DEG_ENVELOPE`

The standard whole-arm GUI now forwards a selected, healthy J2 position or
hold request to the J2 fault-domain worker. The GUI and command router reject a
J2 target outside `[-5, +5] deg` relative to `SESSION_REFERENCE_V1`.

The installed-load V15.30E evidence records successful completion in both
directions. The accepted positive endpoint was `+5.137377 deg` with
`0.144053 deg` endpoint synchronization error. The bounded follow-up completed
the negative endpoint at `-4.472207 deg` with `0.151863 deg` endpoint
synchronization error and returned to `-0.314143 deg` with `0.295056 deg`
synchronization error. The two directions were completed in separate session
captures because the positive run stopped fail-closed at its stricter first
center gate. See `hardware/v15_30e_ft/j2_coupled_sync_result.json`.

## Frozen J2 worker contract

- J2A/J2B mapping: ID `0 / 1`, sign `-1 / +1`, gear ratio
  `6.329999923706055`.
- Position envelope: command `+/-5 deg`; feedback `+/-7 deg`.
- Profile: at most `5 deg/s` and `15 deg/s^2`.
- Gains: `Kp <= 1.00`, entered through a `0.60 -> target` ramp over `0.50 s`;
  `Kd <= 0.10`.
- Common-mode endpoint integral only: `+/-0.15 N.m` per rotor, Q8 encoded,
  with dwell, error, derived-velocity, slew and unwind gates.
- Reference governor predicted working limit: `0.50 N.m` per rotor.
- Predicted PD hard limit: `0.60 N.m` per rotor; feedback hard limit:
  `154/256 = 0.6015625 N.m` per rotor.
- Synchronization hard brake: `abs(qA - qB) > 0.5 deg`.
- J2 drag remains BRAKE because a gravity-compensated teach mode has not been
  validated for the installed arm.
- Lease expiry, invalid feedback, identity/mode mismatch, motor error,
  over-temperature, feedback/velocity/torque/synchronization envelope breach,
  or target timeout latches the J2 fault domain to dual BRAKE.

The integration does not write motor zero, motor ID, RID, Flash, or EEPROM.

## Verification

On the Ubuntu `car` host with the production Unitree SDK:

- strict C++17 optimized build with `-Wall -Wextra -Werror`: PASS;
- controller dry-run: PASS, serial port not opened, default mode BRAKE;
- GUI and hardware Python contract tests: `60 passed`;
- ROS2 packages `go_m8010_arm_hardware` and `go_m8010_arm_gui`: build PASS;
- live startup: J1, J2A, J2B, J3, J4, and J5 returned `merror=0`; observed
  idle J2 synchronization error was approximately `0.000...0.003 deg`.

J6 had no valid feedback during this GUI startup and therefore remained
unavailable; that degraded connection does not grant J6 active authority.
