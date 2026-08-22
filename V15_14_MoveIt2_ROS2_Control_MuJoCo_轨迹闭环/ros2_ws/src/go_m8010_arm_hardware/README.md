# V15.30A whole-arm state mirror

This package is a state source and visual mirror only. It has no trajectory,
MoveIt, GUI command, actuator command, HOLD, FOC, BRAKE, zero-write, or
zero-gravity API.

## Raw feedback ownership contract

GO-M8010 feedback is transaction based. Exactly one process must own each
physical bus. The bus-owning frozen controller or read process publishes
`std_msgs/msg/String` on `/whole_arm/motor_feedback_raw` after every valid
motor transaction. A publisher may send one or several samples per message:

```json
{
  "schema": "go-m8010-motor-feedback/1.0",
  "source_monotonic_ns": 123456789,
  "samples": [
    {
      "motor": "J2A",
      "position_rad": 2.45,
      "velocity_rad_s": 0.0,
      "temperature_c": 31,
      "merror": 0,
      "communication_ok": true
    }
  ]
}
```

The identical JSON datagram contract is accepted on UDP `127.0.0.1:15300` so
the frozen C++ Unitree runtime and Python 3.13 dmcan runtime do not have to load
the ROS 2 Python 3.10 ABI. The socket is loopback-only by default.

`position_rad` and `velocity_rad_s` are protocol/motor coordinates. The state
node applies the frozen gear and sign mapping. Required source names are
`J1,J2A,J2B,J3,J4,J5,J6`. J2A and J2B remain internal motor sources and never
appear as ROS joints.

Do not start a second GO-M8010 serial reader beside a motion controller. Add
the feedback publication at the existing bus owner instead. J2 is the only
joint required to remain in its safe BRAKE state for this task; this state node
does not change that state.

## Ubuntu build and launch

From the V15.14 ROS workspace:

```bash
colcon build --symlink-install --packages-select \
  go_m8010_arm_description go_m8010_arm_hardware
source install/setup.bash
```

First place the real arm in a safe static pose and visually match the MuJoCo
pose. The six angles below are an operator measurement, not CAD or ROS zero:

```bash
ros2 launch go_m8010_arm_hardware whole_arm_mirror.launch.py \
  model_path:=/home/car/go-m8010-robot-arm-v15-20a/mujoco_kinematic_v1/go_m8010_arm_v15_13_kinematic.xml \
  session_pose_deg:='J1_DEG,J2_DEG,J3_DEG,J4_DEG,J5_DEG,J6_DEG' \
  pose_matched:=true \
  evidence_directory:=/home/car/go-m8010-robot-arm-v15-20a/hardware/v15_30a_ft
```

For headless numerical QA only, `numeric_test_only:=true` permits a zero base
pose while recording `MUJOCO_SESSION_POSE_MATCHED=NO_NUMERIC_TEST_ONLY`. That
result may prove direct-qpos arithmetic and latency but must not be reported as
visual geometry/sign acceptance.

The state node waits for 25 valid fresh samples from all seven motors, captures
`MIRROR_SESSION_REFERENCE_V1`, then publishes exactly `joint1..joint6` in rad
and rad/s at 50 Hz. `robot_state_publisher` uses a runtime-only lower-case
joint-name view of the frozen URDF. Geometry, axes, limits, and mesh paths are
unchanged.

For software-only pipeline validation, start `mock_motor_feedback`. Evidence
from that node must be stored outside `hardware/v15_30a_ft` and must never be
reported as a real-arm result.

## Physical acceptance sequence

1. Keep J2 safely static and confirm J3 has mechanical clearance before any
   J3 test. Run the already committed V15.24F J3 installed-load runner only
   under its explicit operator safety gate.
2. Start the three state/mirror processes and confirm all seven source names
   are fresh with `merror=0`.
3. Use only frozen validated local controllers. Move one authorized joint at a
   time: J1, J4, J5, J6 and J3 only after installed-load PASS. Never command J2.
4. Compare real logical delta, `/joint_states`, RViz, and MuJoCo direction and
   approximate angle. Fix only state/axis mapping if visualization is wrong;
   never change the validated motor sign from this task.
5. Stop, close captures, fill the operator inventory, generate SHA256SUMS, then
   report measured rates and latency. Do not convert session reference into a
   permanent zero.
