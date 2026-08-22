# V15.24F-FT final-load J2/J3 drive authority

## Result

`FINAL_TASK_RESULT = FAIL`.

Both J2 levels passed session-center capture, current-position HOLD, communication, synchronization, logical-velocity, `merror`, and 100 Hz timing checks. Both then failed the first `+5 deg` position checkpoint and stopped fail-closed before the first-center, `-5 deg`, and final-center stages. No J2 repeat and no J3 installed-load run were permitted.

The Kp increase from `1.00` to `1.40` increased maximum logical torque feedback from `6.305273` to `9.049922 N.m`, but the measured `+5 deg` displacement decreased from `0.780152` to `0.640435 deg`. This strengthens `MECHANICAL_LOAD_OR_FRICTION_CANDIDATE`, but does not establish a unique root cause. Internal torque/current limiting, brake or drivetrain friction, power-supply behavior, and final mechanical load remain unresolved alternatives.

## Frozen authority and method

- Source HEAD: `5129a975d5c456aa04fed2e7ff47456a56785489`.
- Branch: `agent/v15-24f-ft-j2-drive-authority`; this document is contained in the final commit reported by the handoff.
- J2A/J2B: ID `0 / 1`, signs `-1 / +1`, gear ratio `6.3299999237060547`.
- `Kd=0.05`, `Tff=0`, velocity `10 deg/s`, acceleration `30 deg/s^2`, command rate `100 Hz`.
- Every run established a fresh `SESSION_CENTER` from 50 valid, static, dual-BRAKE frames. The center was not redefined during the route.
- Planned route: `0 -> +5 -> 0 -> -5 -> 0 deg`, relative to the real startup position.
- The permanent V15.24E center-gate rule was enforced: a failed checkpoint prohibited every later stage.
- Motor zero was not modified. MuJoCo was not modified or used for real-time feedforward.

## Active-run source traceability and post-run hardening

The exact J2 source used to build the active-run binary is preserved as `hardware/v15_24f_ft/j2_runner_active_source.cpp` with SHA256 `78ed9d8371356e7d5c618600c9b9919d042a6281f817a7f3d291c047e965ece0`; the active binary SHA256 is `b288285282445da1f3dbd0dc3fba17d74730488152eed483c4225fc494008ed6`.

The committed J2 runner SHA256 is `3c2c01662057eb867f7a9ceab8652953ebf01abcdb239ff19bba2b62fc7c786e`. Its only difference from the active-run source is a post-run safety hardening: endpoint synchronization now uses the tail-window maximum absolute `e_sync`, matching the analyzer, instead of the absolute signed median. The hardened source passed strict real-SDK optimized compilation, self-test, and UBSan self-test without constructing a serial port. The active evidence is not rewritten. This difference cannot change either recorded result because both runs failed the first checkpoint on position error, while both stricter endpoint synchronization values remained below `0.3 deg`.

## J2 Level A — Kp 1.00

- A0/B0: `2.453991413116455 / 5.420142173767090 rad`.
- HOLD: `PASS`; final/max e_sync `0.003472 / 0.008677 deg`; maximum logical motion `0.002603 deg`.
- `+5 deg` actual/error: `0.780152 / 4.219848 deg`; endpoint/max route e_sync `0.201330 / 0.220422 deg`.
- Maximum logical velocity: `9.330591 deg/s`.
- Maximum expected logical PD torque / logical torque feedback: `5.999228 / 6.305273 N.m`.
- 100 Hz timing: `PASS`; median/p95/max `9.999829 / 10.067981 / 10.159209 ms`.
- Result: `FAIL` at `PLUS_5_ENDPOINT` with termination reason `PLUS_5_ACCEPTANCE_FAILED_STOP_ROUTE`.
- First center, `-5 deg`, and final center: `NOT_EXECUTED_FAIL_CLOSED`.
- Final five-pair BOTH BRAKE: communication and strict-static result `PASS`; A/B span `0.060747 / 0.131908 deg`.

The failure met the frozen safe-tracking conditions for the single conditional Level B run: operator `SAFE`, HOLD `PASS`, `merror=0`, maximum e_sync below `0.7 deg`, maximum logical velocity below `25 deg/s`, and final BRAKE `PASS`.

## J2 Level B — Kp 1.40

- Executed: `YES`.
- A0/B0: `2.4164087772369385 / 5.465011119842529 rad`.
- HOLD: `PASS`; final/max e_sync `0.043390 / 0.045127 deg`; maximum logical motion `0.035581 deg`.
- `+5 deg` actual/error: `0.640435 / 4.359565 deg`; endpoint/max route e_sync `0.147526 / 0.147528 deg`.
- Maximum logical velocity: `7.997649 deg/s`.
- Maximum expected logical PD torque / logical torque feedback: `8.551584 / 9.049922 N.m`.
- 100 Hz timing: `PASS`; median/p95/max `10.000890 / 10.114065 / 10.171897 ms`.
- Result: `FAIL` at `PLUS_5_ENDPOINT` with termination reason `PLUS_5_ACCEPTANCE_FAILED_STOP_ROUTE`.
- First center, `-5 deg`, and final center: `NOT_EXECUTED_FAIL_CLOSED`.
- All five final rows commanded BOTH BRAKE and passed A/B send/receive, packet correctness, CRC, ID, mode, temperature, and `merror` checks.
- Strict final-BRAKE static result: `FAIL`; J2B span was `0.210007 deg`, exceeding the `0.200000 deg` limit. J2A span was `0.105871 deg`. This strict failure is retained and is not rewritten as a PASS.

## Repeat, J3, and combined gate

- J2 Kp 1.00 exact repeat: `NOT_EXECUTED_RUN1_FAIL`.
- J2 Kp 1.40 exact repeat: `NOT_EXECUTED_RUN1_FAIL`.
- J2 validated final installed-load Kp: `NONE`.
- `J2_INSTALLED_LOAD_BIDIRECTIONAL_5DEG = FAIL`.
- `J2_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`.
- J3 installed-load validation: `NOT_EXECUTED_J2_REPEAT_GATE_NOT_MET`.
- J3 Kp used, HOLD, endpoints, and repeat: `NOT_APPLICABLE`.
- `J3_LOCAL_MOTION_AUTHORITY = NOT_EVALUATED`.
- `FINAL_JOINT_LEVEL_COMMISSIONING_GATE = FAIL`.

## Torque-response interpretation

The Kp command rose by `1.4x`. Maximum expected logical PD torque rose by approximately `1.425x`, and maximum logical torque feedback rose by approximately `1.435x`. Therefore:

- `TORQUE_FEEDBACK_SCALED_WITH_KP = YES` for the recorded maximum-magnitude comparison.
- Mechanical movement remained very small and did not improve: Level B `+5 deg` actual was `0.139717 deg` lower than Level A.
- `J2_FAILURE_CLASSIFICATION = MECHANICAL_LOAD_OR_FRICTION_CANDIDATE`.
- `UNIQUE_ROOT_CAUSE_ESTABLISHED = NO`.

This evidence does not exclude an internal current/torque limit, brake or transmission friction, power-supply limitation, or final-assembly load. No Kp above `1.40` is authorized by this task.

## Final safety state

- Operator observation after both runs: `SAFE`; no abnormal noise, vibration, collision, or mechanical interlock was reported. The operator reported that substantial force was required to drive the assembly.
- Final five-frame BOTH BRAKE command and communication: `PASS`.
- Final strict BRAKE static acceptance: `FAIL` because J2B span was `0.210007 deg > 0.200000 deg`.
- Operator-confirmed 24 V power OFF: `YES`.
- Overall final BRAKE report remains `FAIL_STRICT_STATIC`, with the successful BRAKE commands and confirmed power-off recorded separately.

## Evidence

- `j2_kp100_run1.csv`: `fb9a1d01fb14b1d1d9ddd3dae38d80ad634292fa892a1a4e65c2966cbdb6d74e`
- `j2_kp100_run1.analysis.json`: `ef73b248a809e8cb0fe86b9fe3bea734406cb0d122e0b1237e524ef8516a2e0d`
- `j2_kp140_run1.csv`: `2e1fb90371abd0aa6000c825f1239a983d966dc85cc322e95939aeb32e64f0af`
- `j2_kp140_run1.analysis.json`: `7a1f2b4f3f3a2e957e08166f08e0199a3a5e8021c0e15e7a6d0879f162ebe9c0`
- The complete evidence and implementation manifest is `hardware/v15_24f_ft/SHA256SUMS`; `inventory.json` records the result semantics and primary artifact hashes.

## Required final report

1. Source HEAD: `5129a975d5c456aa04fed2e7ff47456a56785489`.
2. Branch / commit: `agent/v15-24f-ft-j2-drive-authority / SELF`; resolve `SELF` with the final `git rev-parse HEAD` reported by the handoff.
3. V15.24E center-gate issue fixed: `YES`.
4. Level A Kp: `1.00`.
5. Level A A0/B0: `2.453991413116455 / 5.420142173767090 rad`.
6. Level A HOLD: `PASS`.
7. Level A +5 actual/error: `0.780152 / 4.219848 deg`.
8. Level A first center: `NOT_EXECUTED_FAIL_CLOSED`.
9. Level A -5 actual/error: `NOT_EXECUTED_FAIL_CLOSED`.
10. Level A final center: `NOT_EXECUTED_FAIL_CLOSED`.
11. Level A max e_sync: `0.220422 deg`.
12. Level A max logical torque feedback: `6.305273 N.m`.
13. Level A result: `FAIL`.
14. Level B executed: `YES`.
15. Level B Kp: `1.40`.
16. Level B +5 actual/error: `0.640435 / 4.359565 deg`.
17. Level B first center: `NOT_EXECUTED_FAIL_CLOSED`.
18. Level B -5 actual/error: `NOT_EXECUTED_FAIL_CLOSED`.
19. Level B final center: `NOT_EXECUTED_FAIL_CLOSED`.
20. Level B max e_sync: `0.147528 deg`.
21. Level B max logical torque feedback: `9.049922 N.m`.
22. Level B result: `FAIL`.
23. J2 repeat used: `NO`.
24. J2 repeat result: `NOT_EXECUTED`.
25. Torque feedback scaled with Kp: `YES`.
26. J2 failure classification: `MECHANICAL_LOAD_OR_FRICTION_CANDIDATE_NON_EXCLUSIVE`.
27. J2 final validated Kp: `NONE`.
28. J2 local motion authority: `NOT_GRANTED`.
29. J3 executed: `NO`.
30. J3 Kp used: `NOT_APPLICABLE`.
31. J3 HOLD: `NOT_EXECUTED`.
32. J3 +5 error: `NOT_EXECUTED`.
33. J3 first center: `NOT_EXECUTED`.
34. J3 -5 error: `NOT_EXECUTED`.
35. J3 final center: `NOT_EXECUTED`.
36. J3 repeat result: `NOT_EXECUTED`.
37. J3 local motion authority: `NOT_EVALUATED`.
38. Final joint-level commissioning gate: `FAIL`.
39. Final BRAKE: `FAIL_STRICT_STATIC`; five-frame command/communication `PASS`.
40. 24 V off: `CONFIRMED`.
41. Motor zero modified: `NO`.
42. MuJoCo modified: `NO`.
43. Evidence SHA256: recorded above, in `inventory.json`, and in `hardware/v15_24f_ft/SHA256SUMS`.
44. Git status: final clean status is verified and reported by the handoff after commit and push.
45. Unresolved items: internal torque/current limit, brake or transmission friction, power supply, and mechanical final load.
46. Next single task: `J2_DRIVE_LIMITATION_DIAGNOSTIC` covering the unresolved candidates without increasing Kp above `1.40`.
47. Final task result: `FAIL`.
