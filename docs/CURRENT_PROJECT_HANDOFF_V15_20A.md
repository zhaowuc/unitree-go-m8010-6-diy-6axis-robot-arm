# CURRENT PROJECT HANDOFF — V15.20A

> Status snapshot for a new ChatGPT/Codex conversation. Read this file before
> proposing or executing any work. This document records the project state; it
> does not grant hardware-motion authority.

## 1. Handoff identity and truth model

- Robot: 6-DoF arm plus servo gripper.
- Main host: `car`, Ubuntu 22.04.5 LTS.
- ROS/planning project: ROS 2 Humble, MoveIt 2, `ros2_control`.
- Simulation: MuJoCo.
- Current tracked handoff branch:
  `agent/v15-20a-j345-shared-bus-ready-pose`.
- Source HEAD before this handoff document:
  `56a125d625c06e992231850bb03bad89315e0300`.
- Clean worktree containing the current baseline:
  `/home/car/go-m8010-robot-arm-v15-20a`.
- `/home/car/go-m8010-robot-arm` is a separate J1 worktree with preserved
  untracked hardware files. Do not clean, reset, stage, or overwrite it.

Information priority used here:

1. current tracked Git authority;
2. committed JSON/YAML/Markdown;
3. Git history;
4. preserved evidence and hashes;
5. operator reports.

`REPO_VERIFIED` means the statement is supported by tracked authority.
`OPERATOR_REPORTED` means it is a current physical-state report that the repo
cannot independently prove. If the two differ, current physical state follows
the operator report, while historical authority remains unchanged.

## 2. SSH access for the next conversation

> Sensitive access information. The password below is included because the
> operator explicitly requested a self-contained handoff. Treat this file and
> its Git history as credential-bearing material and rotate the password after
> handoff if repository readers should not retain host access.

- Windows SSH alias: `car` (`REPO-INDEPENDENT HOST VERIFICATION`).
- Direct IPv4 address: `192.168.3.112` (`HOST_VERIFIED`).
- SSH user: `car` (`HOST_VERIFIED`).
- SSH port: default `22` (no override is present in the `Host car` entry).
- SSH login password: `111111` (`OPERATOR_PROVIDED`; not repo-verified).
- Preferred command from the current Windows laptop: `ssh car`.
- Direct equivalent: `ssh car@192.168.3.112`.
- Host OS: `Ubuntu 22.04.5 LTS`.
- Do not assume that the SSH password is also sudo authority. Verify separately
  before any future privileged operation.

After connecting, begin with read-only checks:

```bash
cd /home/car/go-m8010-robot-arm-v15-20a
git status --porcelain
git branch --show-current
git rev-parse HEAD
```

At this handoff source baseline, the expected branch/HEAD are the values in
section 1 and the worktree is clean.

## 3. Hardware overview

| Logical joint | Motor hardware | Notes |
|---|---|---|
| J1 | Unitree GO-M8010-6 | One motor, historical ID 0 |
| J2 | Two mirror-mounted Unitree GO-M8010-6 motors | One logical joint; dual-motor commissioning not done |
| J3 | Unitree GO-M8010-6 | Current ID 3 |
| J4 | Unitree GO-M8010-6 | Current ID 4 |
| J5 | Unitree GO-M8010-6 | Current ID 5 |
| J6 | DaMiao DM-G6220 | Mechanical/simulation assets tracked; real-control evidence absent from this baseline |
| Gripper | Servo gripper, planned ESP32 PWM control | Real hardware control implementation not tracked |

The project contains a full six-axis simulation/planning stack, but the real
arm has not reached unified six-axis hardware control.

## 4. Three distinct state dimensions

Never collapse these columns into a single `PASS`:

- `HISTORICALLY_VERIFIED`: what a previous, preserved hardware task proved.
- `CURRENTLY_PHYSICALLY_CONNECTED`: what is physically online now.
- `CURRENT_MOTION_AUTHORITY`: what may actively move now.

## 5. Current physical topology

The connection facts in this section are `OPERATOR_REPORTED` unless explicitly
marked as tracked transport authority.

```text
car / Ubuntu 22.04.5
│
├── FTDI FTASQA6F CHANNEL_2 / interface 02
│   ├── J3 GO-M8010-6 ID3   CONNECTED
│   ├── J4 GO-M8010-6 ID4   CONNECTED
│   └── J5 GO-M8010-6 ID5   CONNECTED
│
├── FTDI CHANNEL_3 / interface 03
│   └── J1 historical transport authority only
│       CURRENTLY DISCONNECTED
│
├── J2
│   └── CURRENT CONNECTION UNKNOWN
│       two physical motors / one logical joint
│
├── J6 DM-G6220
│   └── CURRENTLY DISCONNECTED
│
└── gripper
    └── planned ESP32 + servo PWM
        CONTROL PENDING
```

Current J345 stable port (`REPO_VERIFIED`):

```text
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0
```

It resolved to `/dev/ttyUSB2` at capture time. Baudrate is 4,000,000.
J1's historical stable port is the corresponding `if03-port0`, but historical
CHANNEL_3 authority does **not** mean J1 is currently connected.

## 6. Current subsystem table

| Subsystem | Historical status | Current connection | Motion authority now | Pending |
|---|---|---|---|---|
| Simulation | `PURE_SIMULATION_PHASE = COMPLETE` | N/A | N/A | implicitfast limitation remains documented |
| Ubuntu runtime | portability changes tracked | host online | none implied | frozen simulation report still says target runtime verification pending |
| J1 | commissioning complete, bidirectional ±10° PASS | DISCONNECTED | NO | reconnect/transport revalidation before any use; CAD zero pending |
| J2 | simulation model only; real commissioning not started | UNKNOWN | NO | dual-motor sign, zero, synchronization, sharing, fault behavior |
| J3 | communication/shared-bus PASS, ID3 | CONNECTED | NO | sign, CAD/ROS zero, load/FOC commissioning |
| J4 | communication/shared-bus PASS, ID4 | CONNECTED | NO | sign, CAD/ROS zero, load/FOC commissioning |
| J5 | communication/shared-bus PASS, ID5 | CONNECTED | NO | sign, CAD/ROS zero, load/FOC commissioning |
| J6 | control complete only by operator report | DISCONNECTED | NO | recover evidence or revalidate before integration |
| Gripper | mechanical/simulation representation exists | UNKNOWN | NO | ESP32/servo hardware control implementation |
| READY_POSE_V1 | J3/J4/J5 raw reference frozen | J345 connected | NOT A MOTION TARGET | J1/J2/J6/gripper not included in frozen raw snapshot |
| ROS2 HardwareInterface | project exists | not established for full real arm | NO | unified hardware integration |
| `ros2_control` | simulation project exists | real-arm control not established | NO | hardware validation |
| MoveIt real hardware | planning project exists | not connected to unified hardware | NO | real-hardware integration and safety validation |

## 7. J1 frozen historical authority

Chapter status banner:

```text
CURRENT PHYSICAL CONNECTION: DISCONNECTED
DO NOT RUN J1 WITHOUT RECONNECTING AND REVALIDATING TRANSPORT.
```

Tracked authority commit:
`d7a35c82f6c7d0e9f5b323caa084027d154ab246`.

Tracked results:

- `J1_MOTION_CONTROL = PASS`.
- `J1_BIDIRECTIONAL_10DEG = PASS`.
- `J1_COMMISSIONING = COMPLETE`.
- Motor: GO-M8010-6, ID 0.
- Historical FTDI: CHANNEL_3 / interface 03.
- Gear ratio: `6.3299999237060547` motor-to-output.
- `raw_to_ros_sign: +1`.
- Control: Unitree SDK FOC mixed position control.
- SDK literals: `kp=0.50`, `kd=0.05`, `tau=0`.
- Rate: 100 Hz.
- Trajectory reference limits: `12 deg/s`, `40 deg/s^2`.
- Historical route: `0 → +10 → 0 → -10 → 0` degrees relative to
  `SESSION_LOCAL_ZERO`.

Measured historical result:

| Metric | Value |
|---|---:|
| +10 peak | `9.903338492 deg` |
| -10 peak | `-9.502416094 deg` |
| max fast estimated speed | `17.961543512 deg/s` |
| max slow estimated speed | `16.867487422 deg/s` |
| max tracking error | `1.268503869 deg` |
| final return error | `0.034712036 deg` |
| max temperature | `30 C` |
| `merror` frames | `0` |
| invalid frames | `0` |

Evidence CSV SHA-256:
`dc738511024798b4d06040dcc13676c20822c600d13b35e51ea3f78b0751d0d9`.

J1 pending items do not invalidate the historical PASS:

- absolute CAD/ROS zero: `PENDING_PHYSICAL_ALIGNMENT`;
- SDK `data.dq`: logged but `NOT CONTROL AUTHORITY`;
- full-range raw wrap/multi-turn behavior: `NOT FULLY VALIDATED`;
- current transport and power: must be physically re-established and checked.

## 8. J345 highest tracked authority

Tracked authority commit:
`56a125d625c06e992231850bb03bad89315e0300`.

```text
J345_SHARED_BUS = PASS
READY_POSE_V1 = FROZEN
```

- FTDI: CHANNEL_2 / interface 02 / serial `FTASQA6F`.
- Baudrate: 4 Mbps.
- A/B polarity: corrected to the verified working orientation.
- Termination status: `UNKNOWN`; communication success is not termination
  proof.
- J3: original ID 0, final ID 3, motor mode restored PASS.
- J4: original ID 0, final ID 4, motor mode restored PASS.
- J5: original ID 4, final ID 5, motor mode restored PASS.
- Shared discovery: 9/9 full-valid BRAKE responses.

100 Hz shared-bus evidence:

| Joint | Valid | Actual Hz | Max RTT | Temperature |
|---|---:|---:|---:|---:|
| J3 | 500/500 | `99.9401820157541` | `1.584172 ms` | 33 C |
| J4 | 500/500 | `99.9421520803818` | `1.359348 ms` | 33–34 C |
| J5 | 500/500 | `99.9412011865315` | `1.641500 ms` | 34 C |

No timeout, collision suspicion, returned-ID mismatch, non-BRAKE response,
or nonzero `merror` occurred in the 1,500-frame performance evidence.

J3/J4/J5 have communication authority only. Their real motion commissioning
has not started:

- sign: `PENDING` for all three;
- CAD zero: `PENDING`;
- ROS zero: `PENDING`;
- motor internal zero: not modified;
- FOC/load/gravity-loaded behavior: not tested.

## 9. READY_POSE_V1 semantics

> **READY_POSE_V1 IS NOT AN ACTIVE MOTION TARGET.**

Tracked semantic: `READY_POSE_V1_RAW_REFERENCE`.

| Joint | ID | motor SDK local-unwrapped raw median |
|---|---:|---:|
| J3 | 3 | `2.1399083137512207 rad` |
| J4 | 4 | `0.49758619070053101 rad` |
| J5 | 5 | `4.3298625946044922 rad` |

These values represent an operator-defined physical initialization posture.
They are not CAD zero, ROS zero, motor internal zero, or values that may be
blindly copied into `MotorCmd.q`. GO-M8010 feedback is single-turn absolute;
runtime local unwrapping does not create permanent multi-turn authority.

Future recovery architecture, for design context only:

```text
POWER ON
→ discover motors
→ BRAKE / safe state
→ read current encoders
→ reconstruct valid branches using limits and prior safe state
→ validate plausibility
→ hold current pose
→ require all participating joints healthy
→ smooth trajectory to READY_POSE_V1 under separate motion authority
→ READY
```

Forbidden startup behavior:
`POWER ON → blindly send saved raw encoder values`.

## 10. J2 status

The repo verifies that J2 is one logical degree of freedom driven by two
mirror-mounted GO-M8010-6 motors. J2 was not started or tested in J1
commissioning. Current physical connection is `UNKNOWN`.

Before both physical J2 motors may be actively enabled together, a dedicated
commissioning task must establish direction/sign, zero, synchronization,
current/torque sharing, limits, and fault behavior. Until then:

```text
DO NOT ENABLE BOTH J2 MOTORS TOGETHER.
```

## 11. J6 status

- Motor: DaMiao DM-G6220.
- Current physical connection: `DISCONNECTED` (`OPERATOR_REPORTED`).
- Historic control status: `OPERATOR_REPORTED_COMPLETE`.
- Tracked mechanical/simulation geometry: present.
- Repo-verified real control branch/commit/interface/ID/range evidence:
  `NOT FOUND IN CURRENT HANDOFF BASELINE`.

Therefore J6 is not ready for assumed integration. Recover its preserved
evidence or perform a separately authorized revalidation before treating it as
commissioned.

## 12. Gripper status

The current design is an ESP32-controlled PWM servo gripper
(`OPERATOR_REPORTED design plan`). The repo contains mechanical/simulation
gripper representation but no tracked real ESP32/servo control evidence.

```text
GRIPPER_CONTROL = PENDING_IMPLEMENTATION
```

## 13. Frozen simulation authority

Do not recompute or replace this authority during hardware handoff work.

- V15.15 mass and COM authority: complete.
- V15.16 Engineering rigid-body inertia: complete.
- V15.17 inertial deployment and production hash migration: complete.
- V15.18A static gravity/dynamics identities: complete.
- V15.18B passive gravity: `PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION`.
- `PURE_SIMULATION_PHASE = COMPLETE`.
- Frozen simulation commit/tag target:
  `cd8642764a7e7b580d1c001f63b3360cd6ddae14` /
  `v15.18b-final-simulation-handoff`.
- Ubuntu portability commit in current ancestry:
  `135729d`.
- Production MJCF SHA-256 recorded by the frozen report:
  `5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9`.

The remaining simulation limitation is numerical, not a continuous-time model
failure:

```text
NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED
CONTINUOUS_TIME_DYNAMICS_MODEL = PASS
PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION
```

The frozen handoff text still records
`UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST`. A later portability
patch exists, but no tracked replacement report in this baseline upgrades that
specific frozen runtime field to PASS.

## 14. Safety invariants

1. J1 is currently disconnected; never assume it is online.
2. J6 is currently disconnected; never assume it is online or integrated.
3. READY raw values are not motion targets.
4. Do not perform large FOC motion on J3/J4/J5 before sign and zero authority.
5. Do not actively enable both J2 motors before dual-motor commissioning.
6. `CALIBRATE`, EEPROM, ID, and internal-zero writes require separate explicit
   authority.
7. GO-M8010 rotor/joint conversions must use the frozen gear ratio
   `6.3299999237060547` and correct branch/sign semantics.
8. J345 CHANNEL_2 is the currently connected shared bus.
9. J1's historical CHANNEL_3 authority does not imply a currently online J1.
10. This handoff does not authorize the next task. The user and new
    conversation must first confirm shared understanding.

## 15. Work that must not be redone without contradictory evidence

- V15.15 Mass and COM.
- V15.16 inertia.
- V15.17 inertial deployment.
- V15.18 gravity validation and Ubuntu portability work.
- J1 gain exploration, torque experiments, and ±10° commissioning.
- J345 ID assignment and A/B diagnosis.
- J345 shared-bus discovery and 100 Hz test.
- J345 READY_POSE capture.

This list is an authority-preservation rule, not a proposed next-task plan.

## 16. Pending areas

- J1 physically disconnected; absolute CAD/ROS zero and full-range wrap remain
  unresolved.
- J2 current connection unknown and dual-motor commissioning absent.
- J3/J4/J5 sign, CAD/ROS zero, load, FOC, and gravity-loaded behavior pending.
- J345 termination status unknown; fixed-ID observations do not mathematically
  prove collision absence under every future topology.
- J6 real-control evidence missing from the current tracked baseline.
- Gripper real ESP32/servo implementation pending.
- Unified ROS2 HardwareInterface, `ros2_control`, MoveIt-to-real-arm, and
  six-axis safety integration pending.
- READY_POSE_V1 covers J3/J4/J5 raw references only and grants no motion.

No concrete next-step prompt, wiring action, gain test, or motion plan is
authorized by this section.

## 17. NEW CHAT STARTING INSTRUCTION

This is the current robot-arm project handoff document.

First read this entire file and the current repository. Do not immediately
plan or execute the next task. Your first reply should only:

1. restate the current hardware topology;
2. distinguish historically verified hardware from currently connected
   hardware;
3. restate all frozen authorities;
4. restate the current J1/J345/J6/J2/gripper states;
5. restate the exact READY_POSE_V1 semantics;
6. list pending items;
7. list every safety constraint and item that must not be redone.

Wait for the operator to confirm that your understanding is correct. Only then
may the operator and the new conversation jointly define a separate next task.
