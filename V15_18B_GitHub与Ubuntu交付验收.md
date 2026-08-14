# V15.18B GitHub 与 Ubuntu 最终交付验收

## 结论

```text
NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED
CONTINUOUS_TIME_DYNAMICS_MODEL = PASS
V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION
V15.18B FINAL_SIMULATION_AND_GITHUB_HANDOFF = PASS
UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST
PURE_SIMULATION_PHASE = COMPLETE
NEXT_PHASE = V15.19 REAL_HARDWARE_READONLY_BRINGUP
```

V15.18B2 没有放宽原有阈值，也没有修改 production MJCF、bridge、Mass、COM、Inertia、URDF、controller、MoveIt、`ros2_control`、joint、TF、collision 或 visual authority。原始 `implicitfast` 2 ms/1 ms FAIL 作为 `PRODUCTION_INTEGRATOR_DIAGNOSTIC` 原样保留；新增 0.5 ms 与 RK4 reference 证明误差随步长正常下降。limitation 的精确含义是：

`PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION`

Passive free-fall 不是当前 production `kinematic_position_tracking` 控制模式。本任务不修改 production timestep/integrator，也不启动 V15.19。

## 数值归因摘要

| 项目 | 结果 |
|---|---:|
| implicitfast 2 ms 最大归一化能量漂移 | 0.015933783414194082 |
| implicitfast 1 ms 最大归一化能量漂移 | 0.008016785126595418 |
| implicitfast 0.5 ms 最大归一化能量漂移 | 0.0040209537454217 |
| 能量比值 1 ms / 2 ms | 0.5031312977088626 |
| 能量比值 0.5 ms / 1 ms | 0.501566860271996 |
| 全轨迹 Dq(h,h/2) | 0.004432203935967438 rad |
| 全轨迹 Dq(h/2,h/4) | 0.0022151684234914537 rad |
| Dq 比值 | 0.49978937239672355 |
| 全轨迹 Dv(h,h/2) | 0.02068748344841409 rad/s |
| 全轨迹 Dv(h/2,h/4) | 0.010340517820967676 rad/s |
| Dv 比值 | 0.4998441616522663 |
| RK4 2 ms 最大归一化能量漂移 | 8.069505973993557e-10 |
| RK4 1 ms 最大归一化能量漂移 | 5.016045475439061e-11 |
| RK4 最大最终 q 差 | 9.282602464466549e-10 rad |
| RK4 最大最终 qvel 差 | 1.1041391800858946e-08 rad/s |
| initial qacc identity 最大相对误差 | 2.604371716984505e-15 |
| early-motion direction | 70/70 PASS |
| contact / active joint-limit event | 0 / 0 |
| determinism | BITWISE_DETERMINISTIC |

RK4 half-step 单调门在归一化域计算，并把 `1e-8 J` 按每个姿态的归一化分母换算；绝对 J 比较仅保留为诊断。六种 profile × 六个语义槽全部通过 finite、contact、limit、force isolation 与 continuity 门。

## 冻结产物

| 产物 | SHA-256 |
|---|---|
| `tools/audit_passive_gravity_v15_18b.py` | `bbba568b8b52694b8f995a63d6b0baa68840f343ccad59d0c016b6fa544a03f9` |
| `tools/validate_passive_gravity_v15_18b.py` | `95344b1c11bf7ecfe138434de93cab8fd327b969efa8289616fc5bc5ebb8c658` |
| `V15_18B_短时被动重力动力学验收.json` | `4cde5893981e8388d76997bef3f6cd05abf427b814bb16fe2de6cab42569d730` |
| `V15_18B_短时被动重力动力学验收.md` | `d9e759068598b252743d37c8d7c10d732b1516d7a098b3cc3455fc95abeff5f7` |

## Authority 与仓库交接

- production MJCF before/after SHA-256：`5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9`；
- production bridge modified：NO；
- Mass / COM / Inertia modified：NO；
- Git LFS required：NO，pointer count=0；
- missing referenced assets=0；runtime Windows absolute paths=0；
- case/NFC collision=0；broken symlink=0；
- 最大 tracked file 严格小于 GitHub 100 MB；
- `git fsck --full`、repository verifier、MuJoCo compile、V15.18A/V15.18B fresh-clone checks 必须全部 PASS。

最终 commit SHA 不写入包含它自身的报告，以避免自引用。报告规定：包含本文件的 commit 就是 final HEAD；远端目标分支、annotated tag peel 和真正 GitHub fresh-clone HEAD 必须在提交后都等于该 HEAD，并由外部 verifier 与最终交接记录确认。

Ubuntu runtime 在目标 Ubuntu 22.04/ROS 2 Humble 主机实际执行前保持 `PENDING_ON_TARGET_HOST`。该 pending 是合法非阻塞状态，不进入 hard unresolved；禁止伪报 Ubuntu runtime PASS。

## Hard unresolved items

`[]`

完成交接后停止，不自动开始 V15.19；等待用户另行授权 `V15.19 REAL_HARDWARE_READONLY_BRINGUP`。
