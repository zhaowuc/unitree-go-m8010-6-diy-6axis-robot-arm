# V15.23D-R1-FT J2 Isolated Commissioning Recovery

## Result

`FINAL_TASK_RESULT = FAIL`

J2A communication and HOLD remain PASS. The formal visible-motion route failed its tracking thresholds at both allowed gains, Kp 0.50 and the single conditional Kp 0.60 retry. The task therefore stopped before J2B discovery or active testing.

## Permanent visible-motion policy

Commissioning actions that require visual direction or motion observation now require at least 5 degrees of commanded joint excursion. Historical +3 degree evidence is preserved as FAIL and is not reused as visual sign authority.

## J2A

- Physical label: `MARKED_MOTOR`
- ID: `0`, unchanged
- Communication: PASS
- Kp 0.50 HOLD: PASS
- Kp 0.50 formal route: FAIL
  - +5 actual/error: `3.653440 / 1.346560 deg`
  - first center error: `1.791140 deg`
  - -5 actual/error: `-3.235160 / 1.764840 deg`
  - final center error: `0.867800 deg`
- Kp 0.60 HOLD: PASS
- Kp 0.60 formal route: FAIL
  - +5 actual/error: `3.671664 / 1.328336 deg`
  - first center error: `1.534271 deg`
  - -5 actual/error: `-3.481615 / 1.518385 deg`
  - final center error: `0.704654 deg`
- Protocol positive physical direction from the frozen photo side view: `COUNTERCLOCKWISE`
- Protocol negative physical direction from the frozen photo side view: `CLOCKWISE`
- Protocol-to-ROS sign: `NOT_ESTABLISHED`
- Isolated motion authority: `FAIL`

The operator additionally requested two direction-only diagnostics. The `0 -> +10 -> 0` and `0 -> -10 -> 0` commands established visible physical directions but did not satisfy the formal tracking thresholds and do not grant commissioning authority.

## J2B and combined state

- J2B discovery: NOT RUN
- J2B ID: NOT ESTABLISHED
- ID write: NOT USED
- J2B isolated authority: NOT GRANTED
- J2 A/B sign relation: NOT ESTABLISHED
- Both motors active simultaneously: NO
- Dual HOLD: NOT TESTED
- Dual motion: NOT TESTED
- Ready for dual HOLD stage: NO

J2A finished with a valid five-frame BRAKE. The operator confirmed GO 24V OFF and J2A isolated. The upper arm remains removed; no reassembly was performed.
