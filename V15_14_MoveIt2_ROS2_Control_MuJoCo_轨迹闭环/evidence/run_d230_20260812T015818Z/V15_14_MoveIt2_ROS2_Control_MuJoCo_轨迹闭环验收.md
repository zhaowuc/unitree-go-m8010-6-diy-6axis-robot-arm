# V15.14 MoveIt2 + ROS2 Control + MuJoCo 轨迹闭环验收

- 总状态：**PASS**
- 执行语义：`kinematic_position_tracking`（空载运动学；不宣称动力学有效）
- 标准接口：`/arm_controller/follow_joint_trajectory`
- 唯一公开状态源：`joint_state_broadcaster → /joint_states`
- 工具中心：`tcp_nominal`

## 多目标结果

| 目标 | 类型 | IK | 规划/拒绝 | MoveIt/MuJoCo碰撞一致 | 最大关节误差(rad) | TCP位置误差(m) | 结论 |
|---|---|---:|---:|---:|---:|---:|---|
| central | pose | True | True | True | 0.0 | 1.0284892470105896e-05 | PASS |
| high | pose | True | True | True | 0.0 | 6.084606561688496e-06 | PASS |
| left | pose | True | True | True | 0.0 | 1.6333750126235078e-05 | PASS |
| front | pose | True | True | True | 6.432490598706546e-16 | 1.5005003764102434e-05 | PASS |
| low | pose | True | True | True | 0.0 | 9.814562693974946e-06 | PASS |
| right | pose | True | True | True | 2.4492935982947064e-16 | 1.9728472360423486e-05 | PASS |
| back | pose | True | True | True | 0.0 | 1.5882419849449127e-05 | PASS |
| random_valid_seed_1514 | joint | True | True | True | 0.0 | 1.5338849464801818e-07 | PASS |
| near_joint_limit_clear | joint | True | True | True | 0.0 | 3.234215381011071e-07 | PASS |
| near_self_collision_clear | joint | True | True | True | 0.0 | 3.2822852773676575e-07 | PASS |
| outside_joint_limit_reject | joint | None | True | None | - | - | EXPECTED_REJECTION_PASS |
| self_collision_reject | joint | None | True | True | - | - | EXPECTED_REJECTION_PASS |

## 安全边界

- J1～J6 的 origin、axis、position limit 未修改。
- `tcp_nominal` 与冻结相机外参未修改。
- 速度 0.5 rad/s、加速度 1.0 rad/s² 仅为 V15.14 仿真执行包络，不是实机额定参数。
- MoveIt 使用 25 个组件碰撞代理（含 UpperArm Motion 专用代理）与 69 对已审计排除；没有全局关闭碰撞。
- MuJoCo bridge 对每个插值命令执行 0.25° 扫掠守卫，并且不提供私有 FollowJointTrajectory server。

## 汇总

```json
{
  "positive_target_count": 10,
  "expected_rejection_count": 2,
  "all_expectations_met": true,
  "ros_graph_pass": true,
  "ground_scene_pass": true,
  "startup_bridge_runtime_contract_pass": true,
  "post_run_bridge_runtime_contract_pass": true,
  "max_final_joint_error_rad": 6.432490598706546e-16,
  "max_tcp_position_error_m": 1.9728472360423486e-05,
  "max_tcp_orientation_error_rad": 0.0002641576493194355,
  "max_rviz_mujoco_joint_sync_error_rad": 2.220446049250313e-16,
  "max_trajectory_tracking_error_rad": 0.012058079000019317,
  "max_full_run_public_raw_sync_error_rad": 4.440892098500626e-16,
  "pass": true
}
```

## 最终聚合与回归

- 权威机器验收：`V15_14_FINAL_ACCEPTANCE.json`，状态 `PASS`，255/255 gates，0 失败。
- 10 个正向目标全部经“plan-only → JTC spline 预验证 → `/execute_trajectory` → `/arm_controller/follow_joint_trajectory`”执行成功。
- 2 个负向目标均为 `EXPECTED_REJECTION_PASS`。其中越界目标由 MoveIt 钳位到冻结边界后，QA 检出请求与结果不一致并禁止执行；这不是 planner 直接返回越界错误。
- fresh 503 姿态：overall/self/ground 碰撞布尔均 0 mismatch，self proxy pair set 0 mismatch，5 个负控全部 PASS。
- J1 连续关节 ±2π 标准 FJT 专项回归 PASS；bridge 分支归一化累计 460 次，无 fault、无 rejected command、无 watchdog timeout。
- V15.13 隔离回归继续 PASS，权威源执行前后哈希不变。

## 地面碰撞建模边界

MoveIt 与 MuJoCo 的地面几何并非逐代理完全同构：

- MoveIt：有限 `4 × 4 × 0.02 m` slab，`z ∈ [-0.11, -0.09] m`。
- MuJoCo：`z <= -0.09 m` 的无限 half-space。
- `UpperArm_Motion_Collision_Proxy` 在 MuJoCo 中按契约有意不参与 ground affinity。

因此 503 姿态中的 ground collision class 逐姿态完全一致，但 ground contact proxy pair 有 90/503 项诊断差异，不参与 PASS 门禁；自碰撞 proxy pair 仍要求并达到精确一致。详细证据见 `collision_cross_regression_503.json`。

## RViz / MuJoCo 同状态可视证据

补充可视验收在隔离 ROS domain 231 中完成。标准 FJT 成功到达非零姿态：

```text
[1.1239862708782487,
 -0.2787640209113898,
  1.7562373783755654,
  1.1168536669244002,
  0.9010670078695902,
  0.5253228012929325] rad
```

- `/joint_states` 唯一发布者：`/joint_state_broadcaster`。
- `/mujoco/joint_states_raw` 唯一发布者：`/mujoco_bridge`。
- 最终 public joint state、MuJoCo raw state、只读证据 renderer qpos 最大差：`0 rad`。
- bridge：`fault_latched=false`、`rejected_command_count=0`。
- renderer 使用冻结 MJCF SHA-256：`CE6EC828D32F445FF25FB4B3BA3167F7F0BEC61CDFCA5962EE4072DF4B022844`。
- 机器证据：`../visual_sync_d231_20260812T024535Z/V15_14_visual_sync_evidence.json`。
- 清晰 RViz RobotModel：`../visual_sync_d231_20260812T024535Z/V15_14_RViz_RobotModel_clear_same_final_state.png`。
- RViz TF 辅助图：`../visual_sync_d231_20260812T024535Z/V15_14_RViz_TF_same_final_state.png`。
- MuJoCo 同状态图：`../visual_sync_d231_20260812T024535Z/V15_14_MuJoCo_same_final_state.png`。

MuJoCo 图片是从实时权威 raw state 读取六轴值后，用同一冻结 MJCF 做的只读证据渲染；权威仿真状态始终由 `mujoco_bridge` 持有，未用 RViz 动画代替 MuJoCo 执行。

## 证据范围说明

最终权威结果仅使用本目录 `run_d230_20260812T015818Z/` 及上述 `visual_sync_d231_20260812T024535Z/` 补充可视证据。父级 `evidence/collision_cross_regression_503.json` 和 `evidence/debug_d217/qa.json` 是修复前的历史 FAIL/调试记录，不得作为最终验收结果。
