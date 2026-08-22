# V15.30A-FT whole-arm real-to-sim state mirror

## Scope and source

- Requested authoritative source: `/home/car/go-m8010-robot-arm-v15-20a`
- Available implementation worktree: `D:\AI_GOM8010_6\go-m8010-robot-arm`
- Source HEAD: `9648aecc17486b0ca22a6595ad33cd3b1cb357c2` (exact match)
- Branch: `agent/v15-30a-ft-whole-arm-real-to-sim-mirror`
- Frozen MuJoCo model SHA256: `26f9e0efe10b7d208378d816540098625a34826069fdf7bbafc1db94cf073941`

The available host is Windows without WSL, ROS 2, `rclpy`, MuJoCo Python, or
connected motor buses. Software was implemented and pure mapping/static safety
tests were run. No physical feedback, motion, J3 validation, RViz, or MuJoCo
live result is claimed. Header-only CSV files intentionally show that no real
samples were captured.

## Frozen architecture

`whole_arm_state_node` is a state-only aggregator. The unique owner of each
hardware bus publishes raw feedback; the state node never opens serial/CAN and
has no actuator command path. It unwraps motor feedback, captures 25-frame
startup medians as `MIRROR_SESSION_REFERENCE_V1`, and produces exactly six ROS
joints. J2 uses the average of signed/geared J2A and J2B, while qA, qB, dqA,
dqB and e_sync remain available in `/whole_arm/hardware_state`.

The mirror subscribes `/joint_states`, sets each production-model qpos directly
to the operator-matched session pose plus measured relative motion, and calls
`mj_forward()`. PID, `mj_step`, actuator tracking and trajectory simulation are
absent. The URDF joint names are changed to lower case only in the runtime
robot-description string so `robot_state_publisher` consumes `joint1..joint6`;
the frozen geometry, axes, limits and meshes are unchanged.

## Offline verification

- `python -m compileall`: PASS
- state mapping tests: PASS
- J2 average/e_sync tests: PASS
- wrap/unwrapped-delta test: PASS
- exact lower-case joint-name test: PASS
- state node has no command/motion API static contract: PASS
- MuJoCo code contains direct qpos + `mj_forward`, and no `mj_step`/PID: PASS
- Total: 6/6 unit/static tests PASS

## Required live continuation

Run on the Ubuntu hardware worktree. Add the documented raw-feedback
publication to each existing unique bus owner; do not start a competing serial
reader. Fill `j2_blocker_operator_inventory.json`, visually match the initial
MuJoCo pose, set `pose_matched:=true`, then execute one authorized physical
joint at a time. J2 stays non-motion. J3 may be evaluated independently only
after operator clearance and external safety conditions are confirmed.

## Final report (current host)

1. source HEAD: `9648aecc17486b0ca22a6595ad33cd3b1cb357c2`
2. branch / commit: `agent/v15-30a-ft-whole-arm-real-to-sim-mirror` / **see final `git rev-parse HEAD` after commit**
3. all seven motor feedback sources readable: **NO — HARDWARE NOT CONNECTED**
4. six logical joints built: **YES — OFFLINE CONTRACT PASS**
5. `/joint_states` rate: **NOT_MEASURED** (configured 50 Hz)
6. robot_state_publisher: **FAIL_NOT_RUN_ENVIRONMENT**
7. RViz mirror: **FAIL_NOT_RUN_ENVIRONMENT**
8. MuJoCo direct qpos mirror: **FAIL_NOT_RUN_ENVIRONMENT** (implementation static test PASS)
9. mirror latency: **NOT_MEASURED**
10. session reference method: **MIRROR_SESSION_REFERENCE_V1, 25 fresh-frame median per motor; operator matched MuJoCo base pose + relative real motion**
11. J1 real→RViz→MuJoCo: **FAIL_NOT_RUN**
12. J4: **FAIL_NOT_RUN**
13. J5: **FAIL_NOT_RUN**
14. J6: **FAIL_NOT_RUN**
15. J3 visible mirror: **NOT_RUN**
16. J2 mirror state readable: **NO — HARDWARE NOT CONNECTED**
17. J2 active motion used: **NO**
18. J3 installed test executed: **NO**
19. J3 Kp: **NOT_USED**
20. J3 +5 error: **NOT_MEASURED**
21. J3 -5 error: **NOT_MEASURED**
22. J3 repeat: **NOT_EXECUTED**
23. J3_LOCAL_MOTION_AUTHORITY: **NOT_EVALUATED_FOR_INSTALLED_LOAD**
24. PSU rated V/A/W: **UNKNOWN / UNKNOWN / UNKNOWN**
25. J2 shared supply: **UNKNOWN**
26. cable gauge: **UNKNOWN**
27. manual supported movement classification: **NOT_PERFORMED**
28. idle voltage: **NOT_AVAILABLE**
29. loaded minimum voltage: **NOT_AVAILABLE**
30. power limitation classification: **UNKNOWN**
31. CAD zero: **PENDING**
32. ROS zero: **PENDING**
33. J2 active authority: **NOT_GRANTED**
34. zero gravity: **NOT_IMPLEMENTED**
35. evidence SHA256: `hardware/v15_30a_ft/SHA256SUMS`
36. git status: **CLEAN_AFTER_FINAL_COMMIT_REQUIRED**
37. unresolved blockers: Ubuntu/ROS2/MuJoCo/hardware live execution; four physical bus-owner feedback publishers; operator pose match; J3 installed-load validation; J2 PSU/cable/manual-resistance/loaded-voltage inventory; J2 active-drive root cause
38. NEXT_SINGLE_TASK: **ON UBUNTU, WIRE BUS-OWNER RAW FEEDBACK PUBLISHERS AND EXECUTE LIVE STATE/RVIZ/MUJOCO MIRROR ACCEPTANCE WITH J2 NON-MOTION**
39. FINAL_TASK_RESULT: **PARTIAL_PASS**

`FAIL_STRICT_STATIC` for the prior J2B 0.210007° span versus 0.200000° is
preserved in the operator inventory. It is not treated as a state-mirror
software blocker.
