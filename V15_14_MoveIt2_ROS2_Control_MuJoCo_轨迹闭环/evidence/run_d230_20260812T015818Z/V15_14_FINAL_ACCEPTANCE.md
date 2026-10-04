# V15.14 最终验收聚合报告

- 结论：**PASS**
- 生成时间（UTC）：`2026-08-12T02:19:36.061033+00:00`
- Gate：`255/255` PASS

## Gate 明细

| Gate | 结果 | 期望 | 实际 | 说明 |
|---|---:|---|---|---|
| `input.main_qa.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `input.collision_503.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `input.collision_static.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `input.collision_runtime.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `input.v15_13_summary.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `input.frozen_baseline.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `input.j1_branch.loaded` | **PASS** | `"readable UTF-8 JSON object"` | `"loaded"` |  |
| `main_qa.schema` | **PASS** | `"go-m8010-arm-v15.14-trajectory-closed-loop-qa/1.0"` | `"go-m8010-arm-v15.14-trajectory-closed-loop-qa/1.0"` |  |
| `main_qa.revision` | **PASS** | `"V15.14-MoveIt2-ros2_control-MuJoCo"` | `"V15.14-MoveIt2-ros2_control-MuJoCo"` |  |
| `main_qa.status` | **PASS** | `"PASS"` | `"PASS"` |  |
| `main_qa.scope.execution_mode` | **PASS** | `"kinematic_position_tracking"` | `"kinematic_position_tracking"` |  |
| `main_qa.scope.dynamics_invalid` | **PASS** | `false` | `false` |  |
| `main_qa.scope.simulation_only` | **PASS** | `true` | `true` |  |
| `main_qa.scope.tcp_frame` | **PASS** | `"tcp_nominal"` | `"tcp_nominal"` |  |
| `main_qa.target_manifest_hash` | **PASS** | `"f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"` | `"f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"` |  |
| `main_qa.mujoco_model_hash` | **PASS** | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` |  |
| `main_qa.ros_graph.pass` | **PASS** | `true` | `true` |  |
| `main_qa.ros_graph.follow_joint_trajectory_provider_pass` | **PASS** | `true` | `true` |  |
| `main_qa.ros_graph.controller_types_and_states_pass` | **PASS** | `true` | `true` |  |
| `main_qa.ros_graph.single_joint_state_source` | **PASS** | `true` | `true` |  |
| `main_qa.ros_graph.standard_topic_based_ros2_control_path` | **PASS** | `true` | `true` |  |
| `main_qa.ros_graph.no_private_moveit_mujoco_edge` | **PASS** | `true` | `true` |  |
| `main_qa.ground_scene` | **PASS** | `true` | `true` |  |
| `main_qa.startup.contract` | **PASS** | `true` | `true` |  |
| `main_qa.startup.controller_parameters` | **PASS** | `true` | `true` |  |
| `main_qa.startup.state_streams_fresh` | **PASS** | `true` | `true` |  |
| `main_qa.startup.fault_clear` | **PASS** | `false` | `false` |  |
| `main_qa.startup.watchdog_clear` | **PASS** | `0` | `0` |  |
| `main_qa.startup.j1_normalization_enabled` | **PASS** | `true` | `true` |  |
| `main_qa.startup.j1_normalization_joint` | **PASS** | `"J1"` | `"J1"` |  |
| `main_qa.startup.j1_normalization_method` | **PASS** | `"nearest_equivalent_to_current"` | `"nearest_equivalent_to_current"` |  |
| `main_qa.post_run.contract` | **PASS** | `true` | `true` |  |
| `main_qa.post_run.controller_parameters` | **PASS** | `true` | `true` |  |
| `main_qa.post_run.state_streams_fresh` | **PASS** | `true` | `true` |  |
| `main_qa.post_run.fault_clear` | **PASS** | `false` | `false` |  |
| `main_qa.post_run.watchdog_clear` | **PASS** | `0` | `0` |  |
| `main_qa.post_run.j1_normalization_enabled` | **PASS** | `true` | `true` |  |
| `main_qa.post_run.j1_normalization_joint` | **PASS** | `"J1"` | `"J1"` |  |
| `main_qa.post_run.j1_normalization_method` | **PASS** | `"nearest_equivalent_to_current"` | `"nearest_equivalent_to_current"` |  |
| `main_qa.aggregate.positive_count` | **PASS** | `10` | `10` |  |
| `main_qa.aggregate.rejection_count` | **PASS** | `2` | `2` |  |
| `main_qa.aggregate.all_expectations_met` | **PASS** | `true` | `true` |  |
| `main_qa.aggregate.ros_graph_pass` | **PASS** | `true` | `true` |  |
| `main_qa.aggregate.ground_scene_pass` | **PASS** | `true` | `true` |  |
| `main_qa.aggregate.startup_bridge_runtime_contract_pass` | **PASS** | `true` | `true` |  |
| `main_qa.aggregate.post_run_bridge_runtime_contract_pass` | **PASS** | `true` | `true` |  |
| `main_qa.aggregate.pass` | **PASS** | `true` | `true` |  |
| `main_qa.tests.count` | **PASS** | `12` | `12` |  |
| `main_qa.tests.unique_required_ids` | **PASS** | `["back","central","front","high","left","low","near_joint_limit_clear","near_self_collision_clear","outside_joint_limit_reject","random_valid_seed_1514","right","self_collision_reject"]` | `["back","central","front","high","left","low","near_joint_limit_clear","near_self_collision_clear","outside_joint_limit_reject","random_valid_seed_1514","right","self_collision_reject"]` |  |
| `main_qa.tests.expectations_and_verdicts` | **PASS** | `"10 execute_success=>PASS and 2 accepted rejections=>EXPECTED_REJECTION_PASS"` | `{"back":{"expected_outcome":"execute_success","verdict":"PASS"},"central":{"expected_outcome":"execute_success","verdict":"PASS"},"front":{"expected_outcome":"execute_success","verdict":"PASS"},"high":{"expected_outcome":"execute_success","verdict":"PASS"},"left":{"expected_outcome":"execute_success","verdict":"PASS...` |  |
| `collision_503.status` | **PASS** | `"PASS"` | `"PASS"` |  |
| `collision_503.revision` | **PASS** | `"V15.14-MoveIt-MuJoCo-503-pose-collision-cross-regression"` | `"V15.14-MoveIt-MuJoCo-503-pose-collision-cross-regression"` |  |
| `collision_503.model_not_modified` | **PASS** | `true` | `true` |  |
| `collision_503.expected_pose_count` | **PASS** | `503` | `503` |  |
| `collision_503.pose_count` | **PASS** | `503` | `503` |  |
| `collision_503.pose_count_gate` | **PASS** | `true` | `true` |  |
| `collision_503.frozen_pose_set_gate` | **PASS** | `true` | `true` |  |
| `collision_503.frozen_pose_set.expected_sha256` | **PASS** | `"4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"` | `"4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"` |  |
| `collision_503.frozen_pose_set.actual_sha256` | **PASS** | `"4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"` | `"4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"` |  |
| `collision_503.input_hash_gate` | **PASS** | `true` | `true` |  |
| `collision_503.match_count` | **PASS** | `503` | `503` |  |
| `collision_503.mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.boolean_mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.overall_boolean_mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.self_boolean_mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.ground_boolean_mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.pair_set_mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.self_pair_set_mismatch_count` | **PASS** | `0` | `0` |  |
| `collision_503.booleans_match` | **PASS** | `true` | `true` |  |
| `collision_503.overall_booleans_match` | **PASS** | `true` | `true` |  |
| `collision_503.self_booleans_match` | **PASS** | `true` | `true` |  |
| `collision_503.ground_booleans_match` | **PASS** | `true` | `true` |  |
| `collision_503.pair_sets_match` | **PASS** | `true` | `true` |  |
| `collision_503.self_pair_sets_match` | **PASS** | `true` | `true` |  |
| `collision_503.frozen_collision_counts` | **PASS** | `true` | `true` |  |
| `collision_503.expected_frozen_collision_counts` | **PASS** | `{"ground":118,"overall":182,"self":126}` | `{"ground":118,"overall":182,"self":126}` |  |
| `collision_503.count.moveit.overall` | **PASS** | `182` | `182` |  |
| `collision_503.count_gate.overall.moveit` | **PASS** | `182` | `182` |  |
| `collision_503.count.moveit.self` | **PASS** | `126` | `126` |  |
| `collision_503.count_gate.self.moveit` | **PASS** | `126` | `126` |  |
| `collision_503.count.moveit.ground` | **PASS** | `118` | `118` |  |
| `collision_503.count_gate.ground.moveit` | **PASS** | `118` | `118` |  |
| `collision_503.count.mujoco.overall` | **PASS** | `182` | `182` |  |
| `collision_503.count_gate.overall.mujoco` | **PASS** | `182` | `182` |  |
| `collision_503.count.mujoco.self` | **PASS** | `126` | `126` |  |
| `collision_503.count_gate.self.mujoco` | **PASS** | `126` | `126` |  |
| `collision_503.count.mujoco.ground` | **PASS** | `118` | `118` |  |
| `collision_503.count_gate.ground.mujoco` | **PASS** | `118` | `118` |  |
| `collision_503.count_gate.overall.pass` | **PASS** | `true` | `true` |  |
| `collision_503.count_gate.self.pass` | **PASS** | `true` | `true` |  |
| `collision_503.count_gate.ground.pass` | **PASS** | `true` | `true` |  |
| `collision_503.negative_control_count` | **PASS** | `5` | `5` |  |
| `collision_503.negative_controls` | **PASS** | `true` | `true` |  |
| `collision_503.accepted_model_unmodified` | **PASS** | `false` | `false` |  |
| `collision_503.parent_filter_enabled_by_guard` | **PASS** | `true` | `true` |  |
| `collision_503.model_hash` | **PASS** | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` |  |
| `collision_503.guard_hash` | **PASS** | `"648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"` | `"648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"` |  |
| `collision_503.validator_hash` | **PASS** | `"9be9fab3e5c7850f001c2e46b83aebf033a02feff2a9e27dbc15347db4098ea4"` | `"9be9fab3e5c7850f001c2e46b83aebf033a02feff2a9e27dbc15347db4098ea4"` |  |
| `collision_503.negative_controls_exact_all_pass` | **PASS** | `"five accepted self-collision controls at indices 43,73,75,83,87"` | `[{"category":"coupled_random_seed_1513","moveit_collision":true,"moveit_self_collision":true,"mujoco_collision":true,"mujoco_self_collision":true,"pass":true,"pose_index":43},{"category":"coupled_random_seed_1513","moveit_collision":true,"moveit_self_collision":true,"mujoco_collision":true,"mujoco_self_collision":tr...` |  |
| `collision_static.pass` | **PASS** | `true` | `true` |  |
| `collision_static.pair_partition.proxy_count` | **PASS** | `25` | `25` |  |
| `collision_static.pair_partition.all_unordered_pair_count` | **PASS** | `300` | `300` |  |
| `collision_static.pair_partition.runtime_full_pair_count` | **PASS** | `231` | `231` |  |
| `collision_static.pair_partition.runtime_excluded_pair_count` | **PASS** | `69` | `69` |  |
| `collision_static.pair_partition.motion_enabled_pair_count` | **PASS** | `2` | `2` |  |
| `collision_static.pair_partition.motion_excluded_pair_count` | **PASS** | `22` | `22` |  |
| `collision_static.runtime_token_coverage` | **PASS** | `true` | `true` |  |
| `collision_static.inherited_meshes` | **PASS** | `true` | `true` |  |
| `collision_static.explicit_geom_pairs` | **PASS** | `82` | `82` |  |
| `collision_static.generated_hash.contract` | **PASS** | `"6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"` | `"6D802909E44F238816F007EF33B57E1B57099C5522A473CD2DCC2705397AF9C9"` |  |
| `collision_static.generated_hash.mjcf` | **PASS** | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` | `"CE6EC828D32F445FF25FB4B3BA3167F7F0BEC61CDFCA5962EE4072DF4B022844"` |  |
| `collision_static.generated_hash.guard` | **PASS** | `"648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"` | `"648B9DE953A84BE46CD5F95E24DAF9F683F3025E73048DC068EF7BD8E5670C6A"` |  |
| `collision_runtime.top_pass` | **PASS** | `true` | `true` |  |
| `collision_runtime.pass` | **PASS** | `true` | `true` |  |
| `collision_runtime.nq` | **PASS** | `6` | `6` |  |
| `collision_runtime.npair` | **PASS** | `82` | `82` |  |
| `collision_runtime.expected_pairs` | **PASS** | `82` | `82` |  |
| `collision_runtime.frozen_pose_gate` | **PASS** | `true` | `true` |  |
| `collision_runtime.frozen_pose_count` | **PASS** | `503` | `503` |  |
| `collision_runtime.cases` | **PASS** | `"at least 3 runtime cases, every pass=true"` | `[{"minimum_motion_contact_distance_m":null,"motion_contact_count":0,"motion_token_pairs":[],"name":"zero","pass":true,"pose_deg":[0.0,0.0,0.0,0.0,0.0,0.0],"reason":"clear","safe":true},{"minimum_motion_contact_distance_m":-0.02705110262961133,"motion_contact_count":8,"motion_token_pairs":[["J1_Fixed_Collision_Proxy"...` |  |
| `v15_13_regression.schema` | **PASS** | `"go_m8010_v15_13_regression_summary_v1"` | `"go_m8010_v15_13_regression_summary_v1"` |  |
| `v15_13_regression.overall_status` | **PASS** | `"PASS"` | `"PASS"` |  |
| `v15_13_regression.source_not_written` | **PASS** | `false` | `false` |  |
| `v15_13_regression.source_hash_unchanged` | **PASS** | `true` | `true` |  |
| `v15_13_regression.xml_deterministic` | **PASS** | `true` | `true` |  |
| `v15_13_regression.contract_deterministic` | **PASS** | `true` | `true` |  |
| `v15_13_regression.required_test_set` | **PASS** | `["build_model","validate_joint_motion_semantics","validate_model","validate_virtual_camera"]` | `["build_model","validate_joint_motion_semantics","validate_model","validate_virtual_camera"]` |  |
| `v15_13_regression.all_tests_pass` | **PASS** | `"all four tests status=PASS and exit_code=0"` | `{"build_model":{"exit_code":0,"status":"PASS"},"validate_joint_motion_semantics":{"exit_code":0,"status":"PASS"},"validate_model":{"exit_code":0,"status":"PASS"},"validate_virtual_camera":{"exit_code":0,"status":"PASS"}}` |  |
| `frozen_baseline.schema` | **PASS** | `"go-m8010-arm-v15.14-frozen-geometry-baseline/2.0"` | `"go-m8010-arm-v15.14-frozen-geometry-baseline/2.0"` |  |
| `frozen_baseline.accepted_contract` | **PASS** | `true` | `true` |  |
| `frozen_baseline.all_frozen_joints` | **PASS** | `true` | `true` |  |
| `frozen_baseline.all_frozen_elements` | **PASS** | `true` | `true` |  |
| `frozen_baseline.runtime_token_coverage` | **PASS** | `true` | `true` |  |
| `frozen_baseline.inherited_meshes` | **PASS** | `true` | `true` |  |
| `frozen_baseline.generated_hash.collision_contract_sha256` | **PASS** | `"6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"` | `"6D802909E44F238816F007EF33B57E1B57099C5522A473CD2DCC2705397AF9C9"` |  |
| `frozen_baseline.generated_hash.mjcf_sha256` | **PASS** | `"ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"` | `"CE6EC828D32F445FF25FB4B3BA3167F7F0BEC61CDFCA5962EE4072DF4B022844"` |  |
| `frozen_baseline.generated_hash.guard_sha256` | **PASS** | `"648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"` | `"648B9DE953A84BE46CD5F95E24DAF9F683F3025E73048DC068EF7BD8E5670C6A"` |  |
| `frozen_baseline.position_limits_inherited` | **PASS** | `"inherited unchanged from V15.13"` | `"inherited unchanged from V15.13"` |  |
| `j1_branch.schema` | **PASS** | `"go-m8010-arm-v15.14-j1-continuous-branch-regression/1.0"` | `"go-m8010-arm-v15.14-j1-continuous-branch-regression/1.0"` |  |
| `j1_branch.pass` | **PASS** | `true` | `true` |  |
| `j1_branch.exception` | **PASS** | `null` | `null` |  |
| `j1_branch.contract.scope` | **PASS** | `"J1 continuous representation through standard FollowJointTrajectory"` | `"J1 continuous representation through standard FollowJointTrajectory"` |  |
| `j1_branch.contract.action` | **PASS** | `"/arm_controller/follow_joint_trajectory"` | `"/arm_controller/follow_joint_trajectory"` |  |
| `j1_branch.contract.joints` | **PASS** | `["J1","J2","J3","J4","J5","J6"]` | `["J1","J2","J3","J4","J5","J6"]` |  |
| `j1_branch.contract.segment_duration` | **PASS** | `"finite and >= 3.0 s"` | `4.0` |  |
| `j1_branch.contract.positive_branch_goal_rad` | **PASS** | `[6.283185307179586,6.403185307179586]` | `[6.283185307179586,6.403185307179586]` |  |
| `j1_branch.contract.negative_branch_goal_rad` | **PASS** | `[-6.163185307179586,-6.283185307179586]` | `[-6.163185307179586,-6.283185307179586]` |  |
| `j1_branch.contract.expected_physical_motion_rad` | **PASS** | `[0.0,0.12,0.0]` | `[0.0,0.12,0.0]` |  |
| `j1_branch.contract.threshold.branch_offset_tolerance_rad` | **PASS** | `0.02` | `0.02` |  |
| `j1_branch.contract.threshold.branch_max_adjustment_tolerance_rad` | **PASS** | `1e-09` | `1e-09` |  |
| `j1_branch.contract.threshold.raw_j1_jump_limit_rad` | **PASS** | `0.05` | `0.05` |  |
| `j1_branch.contract.threshold.raw_velocity_limit_rad_s` | **PASS** | `0.52` | `0.52` |  |
| `j1_branch.contract.threshold.bridge_acceleration_limit_rad_s2` | **PASS** | `1.02` | `1.02` |  |
| `j1_branch.contract.threshold.other_joint_limit_rad` | **PASS** | `1e-05` | `1e-05` |  |
| `j1_branch.contract.threshold.final_position_tolerance_rad` | **PASS** | `0.001` | `0.001` |  |
| `j1_branch.contract.threshold.controller_wrapped_tracking_limit_rad` | **PASS** | `0.03` | `0.03` |  |
| `j1_branch.contract.threshold.minimum_dynamic_sample_count` | **PASS** | `20` | `20` |  |
| `j1_branch.initial.pass` | **PASS** | `true` | `true` |  |
| `j1_branch.initial.all_exact_gates` | **PASS** | `["bridge_fault_not_latched","bridge_no_rejections","bridge_no_watchdog_timeout","bridge_primed","bridge_schema","fresh_adjustment_counter","j1_normalization_enabled","j1_normalization_joint","j1_normalization_method","mechanical_zero_position","publisher_message_types","raw_state_present","single_bridge_status_publi...` | `{"bridge_fault_not_latched":true,"bridge_no_rejections":true,"bridge_no_watchdog_timeout":true,"bridge_primed":true,"bridge_schema":true,"fresh_adjustment_counter":true,"j1_normalization_enabled":true,"j1_normalization_joint":true,"j1_normalization_method":true,"mechanical_zero_position":true,"publisher_message_type...` |  |
| `j1_branch.initial.zero_position` | **PASS** | `"six finite joints within 1e-3 rad of zero"` | `[0.0,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.initial.stationary_velocity` | **PASS** | `"six finite joints within 1e-5 rad/s of zero"` | `[0.0,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.initial.bridge.schema` | **PASS** | `"go-m8010-arm-v15.14-mujoco-bridge-status/1.0"` | `"go-m8010-arm-v15.14-mujoco-bridge-status/1.0"` |  |
| `j1_branch.initial.bridge.fault_latched` | **PASS** | `false` | `false` |  |
| `j1_branch.initial.bridge.command_stream_primed` | **PASS** | `true` | `true` |  |
| `j1_branch.initial.bridge.rejected_command_count` | **PASS** | `0` | `0` |  |
| `j1_branch.initial.bridge.moving_watchdog_timeout_count` | **PASS** | `0` | `0` |  |
| `j1_branch.initial.normalization.enabled` | **PASS** | `true` | `true` |  |
| `j1_branch.initial.normalization.joint_name` | **PASS** | `"J1"` | `"J1"` |  |
| `j1_branch.initial.normalization.method` | **PASS** | `"nearest_equivalent_to_current"` | `"nearest_equivalent_to_current"` |  |
| `j1_branch.initial.normalization.accepted_adjustment_count` | **PASS** | `0` | `0` |  |
| `j1_branch.segments.count` | **PASS** | `2` | `2` |  |
| `j1_branch.positive_2pi_branch.id` | **PASS** | `"positive_2pi_branch"` | `"positive_2pi_branch"` |  |
| `j1_branch.positive_2pi_branch.action` | **PASS** | `"/arm_controller/follow_joint_trajectory"` | `"/arm_controller/follow_joint_trajectory"` |  |
| `j1_branch.positive_2pi_branch.joints` | **PASS** | `["J1","J2","J3","J4","J5","J6"]` | `["J1","J2","J3","J4","J5","J6"]` |  |
| `j1_branch.positive_2pi_branch.two_point_goal` | **PASS** | `{"finish_j1_rad":6.403185307179586,"start_j1_rad":6.283185307179586}` | `[{"position_rad":[6.283185307179586,0.0,0.0,0.0,0.0,0.0],"time_from_start_s":0.0,"velocity_rad_s":[0.0,0.0,0.0,0.0,0.0,0.0]},{"position_rad":[6.403185307179586,0.0,0.0,0.0,0.0,0.0],"time_from_start_s":4.0,"velocity_rad_s":[0.0,0.0,0.0,0.0,0.0,0.0]}]` |  |
| `j1_branch.positive_2pi_branch.pass` | **PASS** | `true` | `true` |  |
| `j1_branch.positive_2pi_branch.all_exact_gates` | **PASS** | `["action_status_succeeded","bridge_accepted_moving_commands","bridge_adjusted_accepted_commands","bridge_fault_not_latched","bridge_guard_safe","bridge_max_adjustment_is_two_pi","bridge_observed_acceleration_within_envelope","bridge_observed_velocity_within_envelope","bridge_rejected_no_commands","bridge_watchdog_no...` | `{"action_status_succeeded":true,"bridge_accepted_moving_commands":true,"bridge_adjusted_accepted_commands":true,"bridge_fault_not_latched":true,"bridge_guard_safe":true,"bridge_max_adjustment_is_two_pi":true,"bridge_observed_acceleration_within_envelope":true,"bridge_observed_velocity_within_envelope":true,"bridge_r...` |  |
| `j1_branch.positive_2pi_branch.goal_accepted` | **PASS** | `true` | `true` |  |
| `j1_branch.positive_2pi_branch.action_status` | **PASS** | `4` | `4` |  |
| `j1_branch.positive_2pi_branch.controller_result` | **PASS** | `0` | `0` |  |
| `j1_branch.positive_2pi_branch.samples.command` | **PASS** | `">= 20"` | `223` |  |
| `j1_branch.positive_2pi_branch.samples.raw_state` | **PASS** | `">= 20"` | `445` |  |
| `j1_branch.positive_2pi_branch.samples.controller_state` | **PASS** | `">= 20"` | `122` |  |
| `j1_branch.positive_2pi_branch.pairing.expected_offset` | **PASS** | `6.283185307179586` | `6.283185307179586` |  |
| `j1_branch.positive_2pi_branch.pairing.sample_count` | **PASS** | `">= 20"` | `223` |  |
| `j1_branch.positive_2pi_branch.pairing.median` | **PASS** | `"within 0.02 rad of 6.283185307179586"` | `6.28380808557738` |  |
| `j1_branch.positive_2pi_branch.pairing.inliers` | **PASS** | `">= 0.95"` | `1.0` |  |
| `j1_branch.positive_2pi_branch.metric.max_consecutive_raw_j1_jump_rad` | **PASS** | `"finite and <= 0.05"` | `0.0009030511401533006` |  |
| `j1_branch.positive_2pi_branch.metric.max_abs_raw_velocity_rad_s` | **PASS** | `"finite and <= 0.52"` | `0.04499999999541572` |  |
| `j1_branch.positive_2pi_branch.metric.max_abs_other_joint_position_rad` | **PASS** | `"finite and <= 1e-05"` | `0.0` |  |
| `j1_branch.positive_2pi_branch.metric.max_abs_other_joint_velocity_rad_s` | **PASS** | `"finite and <= 1e-05"` | `0.0` |  |
| `j1_branch.positive_2pi_branch.metric.max_controller_wrapped_tracking_error_rad` | **PASS** | `"finite and <= 0.03"` | `0.0027007662012212604` |  |
| `j1_branch.positive_2pi_branch.metric.max_final_physical_position_error_rad` | **PASS** | `"finite and <= 0.001"` | `2.7755575615628914e-16` |  |
| `j1_branch.positive_2pi_branch.metric.bridge_max_observed_command_velocity_rad_s` | **PASS** | `"finite and <= 0.52"` | `0.04516268418008571` |  |
| `j1_branch.positive_2pi_branch.metric.bridge_max_observed_command_acceleration_rad_s2` | **PASS** | `"finite and <= 1.02"` | `0.04468977117717338` |  |
| `j1_branch.positive_2pi_branch.metric.expected_final` | **PASS** | `[0.12,0.0,0.0,0.0,0.0,0.0]` | `[0.12,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.positive_2pi_branch.metric.actual_final` | **PASS** | `"within 1e-3 rad of physical J1=0.12, others zero"` | `[0.12000000000000027,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.positive_2pi_branch.counter.j1_accepted_adjustment_count` | **PASS** | `"> 0"` | `222` |  |
| `j1_branch.positive_2pi_branch.counter.accepted_moving_command_count` | **PASS** | `"> 0"` | `200` |  |
| `j1_branch.positive_2pi_branch.counter.rejected_command_count` | **PASS** | `0` | `0` |  |
| `j1_branch.positive_2pi_branch.counter.moving_watchdog_timeout_count` | **PASS** | `0` | `0` |  |
| `j1_branch.positive_2pi_branch.bridge.max_adjustment` | **PASS** | `"2*pi +/- 1e-9 (6.283185307179586)"` | `6.283185307179587` |  |
| `j1_branch.positive_2pi_branch.bridge.fault_clear` | **PASS** | `false` | `false` |  |
| `j1_branch.positive_2pi_branch.bridge.rejections` | **PASS** | `0` | `0` |  |
| `j1_branch.positive_2pi_branch.bridge.watchdog` | **PASS** | `0` | `0` |  |
| `j1_branch.positive_2pi_branch.bridge.normalization.enabled` | **PASS** | `true` | `true` |  |
| `j1_branch.positive_2pi_branch.bridge.normalization.joint` | **PASS** | `"J1"` | `"J1"` |  |
| `j1_branch.positive_2pi_branch.bridge.normalization.method` | **PASS** | `"nearest_equivalent_to_current"` | `"nearest_equivalent_to_current"` |  |
| `j1_branch.negative_2pi_branch.id` | **PASS** | `"negative_2pi_branch"` | `"negative_2pi_branch"` |  |
| `j1_branch.negative_2pi_branch.action` | **PASS** | `"/arm_controller/follow_joint_trajectory"` | `"/arm_controller/follow_joint_trajectory"` |  |
| `j1_branch.negative_2pi_branch.joints` | **PASS** | `["J1","J2","J3","J4","J5","J6"]` | `["J1","J2","J3","J4","J5","J6"]` |  |
| `j1_branch.negative_2pi_branch.two_point_goal` | **PASS** | `{"finish_j1_rad":-6.283185307179586,"start_j1_rad":-6.163185307179586}` | `[{"position_rad":[-6.163185307179586,0.0,0.0,0.0,0.0,0.0],"time_from_start_s":0.0,"velocity_rad_s":[0.0,0.0,0.0,0.0,0.0,0.0]},{"position_rad":[-6.283185307179586,0.0,0.0,0.0,0.0,0.0],"time_from_start_s":4.0,"velocity_rad_s":[0.0,0.0,0.0,0.0,0.0,0.0]}]` |  |
| `j1_branch.negative_2pi_branch.pass` | **PASS** | `true` | `true` |  |
| `j1_branch.negative_2pi_branch.all_exact_gates` | **PASS** | `["action_status_succeeded","bridge_accepted_moving_commands","bridge_adjusted_accepted_commands","bridge_fault_not_latched","bridge_guard_safe","bridge_max_adjustment_is_two_pi","bridge_observed_acceleration_within_envelope","bridge_observed_velocity_within_envelope","bridge_rejected_no_commands","bridge_watchdog_no...` | `{"action_status_succeeded":true,"bridge_accepted_moving_commands":true,"bridge_adjusted_accepted_commands":true,"bridge_fault_not_latched":true,"bridge_guard_safe":true,"bridge_max_adjustment_is_two_pi":true,"bridge_observed_acceleration_within_envelope":true,"bridge_observed_velocity_within_envelope":true,"bridge_r...` |  |
| `j1_branch.negative_2pi_branch.goal_accepted` | **PASS** | `true` | `true` |  |
| `j1_branch.negative_2pi_branch.action_status` | **PASS** | `4` | `4` |  |
| `j1_branch.negative_2pi_branch.controller_result` | **PASS** | `0` | `0` |  |
| `j1_branch.negative_2pi_branch.samples.command` | **PASS** | `">= 20"` | `223` |  |
| `j1_branch.negative_2pi_branch.samples.raw_state` | **PASS** | `">= 20"` | `446` |  |
| `j1_branch.negative_2pi_branch.samples.controller_state` | **PASS** | `">= 20"` | `121` |  |
| `j1_branch.negative_2pi_branch.pairing.expected_offset` | **PASS** | `-6.283185307179586` | `-6.283185307179586` |  |
| `j1_branch.negative_2pi_branch.pairing.sample_count` | **PASS** | `">= 20"` | `223` |  |
| `j1_branch.negative_2pi_branch.pairing.median` | **PASS** | `"within 0.02 rad of -6.283185307179586"` | `-6.283798301393358` |  |
| `j1_branch.negative_2pi_branch.pairing.inliers` | **PASS** | `">= 0.95"` | `1.0` |  |
| `j1_branch.negative_2pi_branch.metric.max_consecutive_raw_j1_jump_rad` | **PASS** | `"finite and <= 0.05"` | `0.0009039185540504549` |  |
| `j1_branch.negative_2pi_branch.metric.max_abs_raw_velocity_rad_s` | **PASS** | `"finite and <= 0.52"` | `0.044999999987595136` |  |
| `j1_branch.negative_2pi_branch.metric.max_abs_other_joint_position_rad` | **PASS** | `"finite and <= 1e-05"` | `0.0` |  |
| `j1_branch.negative_2pi_branch.metric.max_abs_other_joint_velocity_rad_s` | **PASS** | `"finite and <= 1e-05"` | `0.0` |  |
| `j1_branch.negative_2pi_branch.metric.max_controller_wrapped_tracking_error_rad` | **PASS** | `"finite and <= 0.03"` | `0.0027004053734891223` |  |
| `j1_branch.negative_2pi_branch.metric.max_final_physical_position_error_rad` | **PASS** | `"finite and <= 0.001"` | `5.375258780371325e-16` |  |
| `j1_branch.negative_2pi_branch.metric.bridge_max_observed_command_velocity_rad_s` | **PASS** | `"finite and <= 0.52"` | `0.045266678609173494` |  |
| `j1_branch.negative_2pi_branch.metric.bridge_max_observed_command_acceleration_rad_s2` | **PASS** | `"finite and <= 1.02"` | `0.044753585580157886` |  |
| `j1_branch.negative_2pi_branch.metric.expected_final` | **PASS** | `[0.0,0.0,0.0,0.0,0.0,0.0]` | `[0.0,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.negative_2pi_branch.metric.actual_final` | **PASS** | `"within 1e-3 rad of physical J1=0.0, others zero"` | `[5.375258780371325e-16,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.negative_2pi_branch.counter.j1_accepted_adjustment_count` | **PASS** | `"> 0"` | `223` |  |
| `j1_branch.negative_2pi_branch.counter.accepted_moving_command_count` | **PASS** | `"> 0"` | `200` |  |
| `j1_branch.negative_2pi_branch.counter.rejected_command_count` | **PASS** | `0` | `0` |  |
| `j1_branch.negative_2pi_branch.counter.moving_watchdog_timeout_count` | **PASS** | `0` | `0` |  |
| `j1_branch.negative_2pi_branch.bridge.max_adjustment` | **PASS** | `"2*pi +/- 1e-9 (6.283185307179586)"` | `6.283185307179587` |  |
| `j1_branch.negative_2pi_branch.bridge.fault_clear` | **PASS** | `false` | `false` |  |
| `j1_branch.negative_2pi_branch.bridge.rejections` | **PASS** | `0` | `0` |  |
| `j1_branch.negative_2pi_branch.bridge.watchdog` | **PASS** | `0` | `0` |  |
| `j1_branch.negative_2pi_branch.bridge.normalization.enabled` | **PASS** | `true` | `true` |  |
| `j1_branch.negative_2pi_branch.bridge.normalization.joint` | **PASS** | `"J1"` | `"J1"` |  |
| `j1_branch.negative_2pi_branch.bridge.normalization.method` | **PASS** | `"nearest_equivalent_to_current"` | `"nearest_equivalent_to_current"` |  |
| `j1_branch.final.pass` | **PASS** | `true` | `true` |  |
| `j1_branch.final.all_exact_gates` | **PASS** | `["bridge_adjustments_observed","bridge_fault_not_latched","bridge_max_adjustment_is_two_pi","bridge_no_rejections","bridge_no_watchdog_timeout","raw_state_present","returned_to_mechanical_zero","stationary_velocity"]` | `{"bridge_adjustments_observed":true,"bridge_fault_not_latched":true,"bridge_max_adjustment_is_two_pi":true,"bridge_no_rejections":true,"bridge_no_watchdog_timeout":true,"raw_state_present":true,"returned_to_mechanical_zero":true,"stationary_velocity":true}` |  |
| `j1_branch.final.zero_position` | **PASS** | `"six finite joints within 1e-3 rad of zero"` | `[1.5503536808036467e-17,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.final.stationary_velocity` | **PASS** | `"six finite joints within 1e-5 rad/s of zero"` | `[0.0,0.0,0.0,0.0,0.0,0.0]` |  |
| `j1_branch.final.bridge.schema` | **PASS** | `"go-m8010-arm-v15.14-mujoco-bridge-status/1.0"` | `"go-m8010-arm-v15.14-mujoco-bridge-status/1.0"` |  |
| `j1_branch.final.bridge.fault_latched` | **PASS** | `false` | `false` |  |
| `j1_branch.final.bridge.rejected_command_count` | **PASS** | `0` | `0` |  |
| `j1_branch.final.bridge.moving_watchdog_timeout_count` | **PASS** | `0` | `0` |  |
| `j1_branch.final.normalization.adjustments` | **PASS** | `"> 0"` | `460` |  |
| `j1_branch.final.normalization.max_adjustment` | **PASS** | `"2*pi +/- 1e-9 (6.283185307179586)"` | `6.283185307179587` |  |

## 输入证据

| 名称 | 路径 | SHA-256 |
|---|---|---|
| `main_qa` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\evidence\run_d230_20260812T015818Z\QA_V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环.json` | `101959a6d6ea1dc1095cf692155428bf1e2f48a6ddd9377ec749fa7d7c0227e0` |
| `collision_503` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\evidence\run_d230_20260812T015818Z\collision_cross_regression_503.json` | `0518ba83e938a72abe46b6538d490ed2a91396c137ca4734eedc5c546fda9108` |
| `collision_static` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\evidence\collision_layer_validation_v15_14.json` | `e60648559cb1d24d40ed6295196c343e7bc87fd4239e5b355e0557628cda0494` |
| `collision_runtime` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\evidence\run_d230_20260812T015818Z\collision_layer_runtime_validation_v15_14.json` | `e635c8c9e7ae0713febf8ec738c06cf49d30f0f36ff015f3c4cbff27c12e31c6` |
| `v15_13_summary` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\evidence\v15_13_regression_20260811T231406Z\summary.json` | `670b96ba3f11dfaf4ad5425d0c1c4ac3a2f71312acc1bb4f5259e123bbc34b0e` |
| `frozen_baseline` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\config\frozen_geometry_baseline.json` | `9311eefea02477cc70b494dce7237e4497569a5d4f46ffa9dc79b78e9d588e76` |
| `j1_branch` | `D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15\V15_13_整机深度复核_相机上置机械零位\V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环\evidence\run_d230_20260812T015818Z\j1_continuous_branch_regression_v15_14.json` | `3840853756e857a610400e78a5b3b05259ad26ae545d85e3f96eece041d6761b` |
