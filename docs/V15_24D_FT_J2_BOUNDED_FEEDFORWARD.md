# V15.24D-FT J2 bounded gravity feedforward

## Result

`RUN1 = FAIL`. The zero-Tff hold, bounded Tff ramp/hold, communication, synchronization, and terminal BOTH BRAKE passed, but the +/-5 degree position thresholds did not. The contract therefore prohibited RUN2 and any automatic Tff increase.

## Frozen command

- J2A/J2B Tff literals: `-0.05 / +0.05 N.m` rotor-side.
- Actual serialized counts: `-12 / +12`; decoded: `-0.046875 / +0.046875 N.m`.
- Kp/Kd: `0.60 / 0.05`; route: `0 -> +5 -> 0 -> -5 -> 0 deg` at 10 deg/s, 30 deg/s^2, 100 Hz.
- The CSV proves a monotone symmetric 0.5 s Q8 ramp and no command outside +/-12 counts.

## RUN1

- A0/B0: `5.4504380226135254 / 2.4154500961303711 rad`.
- Feedforward hold displacement: `0.004338734753 deg` (`NEGLIGIBLE`); max hold e_sync: `0.010416847874 deg`.
- +5 actual/error/e_sync: `0.448651717277 / 4.551348282723 / 0.070287287188 deg`.
- First-center error: `0.360570330412 deg`.
- -5 actual/error/e_sync: `-0.010413610817 / 4.989586389183 / 0.030374380324 deg`.
- Final-center error: `0.050765678348 deg`; max route e_sync: `0.152732958656 deg`.
- Maximum absolute logical paired torque feedback: `4.673320256174 N.m`.
- Active invalid frames / nonzero merror frames: `0 / 0`; max temperature: `32 C`.
- Operator observation: `SAFE`; final five-pair BOTH BRAKE: `PASS`; operator-confirmed 24V OFF: `YES`.

## Comparison and diagnosis

Relative to the authoritative V15.24B Kp=.60 run, +5 error improved by `0.219551200275 deg`, while -5 error worsened by `0.285506226823 deg`. Relative to its operator repeat, the changes were an improvement of `0.089381604567 deg` and a worsening of `0.112812498660 deg`. Both comparisons show the same positive-direction bias, so `J2_TFF_005_EFFECT = DIRECTIONALLY_BIASED`.

The dedicated hold displacement was below the frozen 0.02 degree direction threshold, so `TFF_DIRECTION_AUTHORITY = INCONCLUSIVE` and `GRAVITY_CONTRIBUTION_CONFIDENCE = INCONCLUSIVE`. This result does not establish gravity as the root cause. Increasing fixed Tff automatically is prohibited and would risk worsening the negative direction.

## Authority and next task

- `J2_TFF_005_REPEATABILITY = NOT_TESTED_RUN1_FAIL`
- `J2_INSTALLED_LOAD_BIDIRECTIONAL_5DEG = FAIL`
- `J2_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`
- `NEXT_SINGLE_TASK = J2_INTERNAL_TORQUE_CURRENT_LIMIT_AND_POSITION_LOOP_EFFORT_AUDIT`
