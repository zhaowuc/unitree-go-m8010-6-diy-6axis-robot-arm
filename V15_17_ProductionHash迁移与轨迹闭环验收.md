# V15.17D Production Hash 迁移与轨迹闭环验收

- 审计有效：`True`
- 状态：`PASS`
- 最终状态：`V15.17 INERTIAL_DEPLOYMENT = PASS`
- Source commit：`75f70ea4527dc90125a13c6d89efd789239367d0`
- Target branch：`agent/v15-17-production-hash-migration`
- Old MJCF SHA256：`ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844`
- New MJCF SHA256：`5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9`

## 20 项最终接受门

| # | Gate | Result | Failure code |
|---:|---|---|---|
| 1 | `git_base_branch_exact4_remote` | `PASS` | `N/A` |
| 2 | `old_new_production_model_hashes` | `PASS` | `N/A` |
| 3 | `semantic_only_six_accepted_inertials` | `PASS` | `N/A` |
| 4 | `bridge_only_model_anchor_migrated` | `PASS` | `N/A` |
| 5 | `old_hash_references_classified` | `PASS` | `N/A` |
| 6 | `mesh_1008_manifest_asset_authority` | `PASS` | `N/A` |
| 7 | `collision_contract_geoms_pairs_unchanged` | `PASS` | `N/A` |
| 8 | `fresh_colcon_current_install_binding` | `PASS` | `N/A` |
| 9 | `production_bridge_hash_authority_compile` | `PASS` | `N/A` |
| 10 | `six_body_independent_runtime_readback` | `PASS` | `N/A` |
| 11 | `moveit_load` | `PASS` | `N/A` |
| 12 | `ros2_control_load` | `PASS` | `N/A` |
| 13 | `standard_fjt_and_joint_state_authority` | `PASS` | `N/A` |
| 14 | `all_expected_success_targets` | `PASS` | `N/A` |
| 15 | `all_expected_rejection_targets` | `PASS` | `N/A` |
| 16 | `v15_14_numeric_thresholds` | `PASS` | `N/A` |
| 17 | `fresh_current_model_503_collision` | `PASS` | `N/A` |
| 18 | `tf_origin_axis_tcp_camera_unchanged` | `PASS` | `N/A` |
| 19 | `visual_limitation_gravity_controller_prohibitions` | `PASS` | `N/A` |
| 20 | `protected_toctou_and_hard_unresolved_empty` | `PASS` | `N/A` |

## 轨迹 attempt history

- Attempt 1：`FAIL`，原因 `OUTSIDE_JOINT_LIMIT_FROZEN_CLAMP_REJECTION_CONTRACT_FAILED`；10 个 expected-success 与 self-collision rejection 通过；outside-limit 路径触发 `1` 个 break-on-first 检测哨兵，未执行。
- Attempt 2：正式选定运行，`PASS`；repo/model/bridge/tools/protected snapshot 与 attempt 1 相同。

## Runtime / numeric facts

- `expected_success_targets`: `"10/10"`
- `expected_rejection_targets`: `"2/2"`
- `max_final_joint_error_rad`: `6.432490598706546e-16`
- `max_tcp_position_error_m`: `1.892103217369465e-05`
- `max_tcp_orientation_error_rad`: `0.00027038632003048476`
- `max_rviz_mujoco_joint_sync_error_rad`: `4.440892098500626e-16`
- `collision_regression`: `{"match": "503/503", "mismatch_count": 0}`
- `six_body_max_tensor_error`: `1.042904201715536e-15`

## 非阻塞限制与禁止项

- 保留：`NON_PHYSICAL_VISUAL_NUMERICAL_LIMITATION`（非物理、视觉层、亚微米 numerical artifact）。
- gravity enabled：`NO`。
- controller modified：`NO`。
- 未修改 friction / damping / armature / motor dynamics / controller tuning / joint limits。

## 最终 29 项报告

| # | Item | Value |
|---:|---|---|
| 1 | branch / commit | `{"branch":"agent/v15-17-production-hash-migration","head_contract":"BASE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD","source_commit":"75f70ea4527dc90125a13c6d89efd789239367d0"}` |
| 2 | old MJCF SHA | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` |
| 3 | new MJCF SHA | `"5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"` |
| 4 | semantic diff only inertial | `"YES"` |
| 5 | modified hash anchors | `["ACCEPTED_MODEL_SHA256"]` |
| 6 | mesh asset authority | `"PASS"` |
| 7 | collision contract | `"PASS"` |
| 8 | production bridge model hash | `"PASS"` |
| 9 | MuJoCo compile | `"PASS"` |
| 10 | six-body max tensor error | `1.042904201715536e-15` |
| 11 | MoveIt load | `"PASS"` |
| 12 | ros2_control load | `"PASS"` |
| 13 | FollowJointTrajectory action | `"PASS"` |
| 14 | expected-success targets | `"10/10"` |
| 15 | expected-rejection targets | `"2/2"` |
| 16 | max final joint error | `6.432490598706546e-16` |
| 17 | max TCP position error | `1.892103217369465e-05` |
| 18 | max TCP orientation error | `0.00027038632003048476` |
| 19 | max RViz/MuJoCo joint sync error | `4.440892098500626e-16` |
| 20 | collision regression | `{"match":"503/503","mismatch_count":0}` |
| 21 | J1-J6 origin unchanged | `"YES"` |
| 22 | J1-J6 axis unchanged | `"YES"` |
| 23 | TCP unchanged | `"YES"` |
| 24 | camera TF unchanged | `"YES"` |
| 25 | visual link6 limitation | `{"limitation":"NON_PHYSICAL_VISUAL_NUMERICAL_LIMITATION","still_present":"YES"}` |
| 26 | gravity enabled | `"NO"` |
| 27 | controller modified | `"NO"` |
| 28 | hard unresolved items | `[]` |
| 29 | final status | `"V15.17 INERTIAL_DEPLOYMENT = PASS"` |

## Hard unresolved

- `[]`

本报告不授权 commit/push；仅在全部 20 门通过且 hard unresolved=[] 时允许上层提交。
