# V15.17C link6 visual sameframe 运行时诊断

- 最终状态：**V15.17 INERTIAL_DEPLOYMENT = PASS_WITH_NONPHYSICAL_VISUAL_LIMITATION**
- 状态名称：`INERTIAL_DEPLOYMENT_ACCEPTED_WITH_NONPHYSICAL_VISUAL_LIMITATION`
- 审计有效：`True`
- CASE：`CASE_2`
- 分类：`VISUAL_ONLY_SUBMICRON_COMPILER_ARTIFACT`
- 工程裁决：`ENGINEERING_ACCEPTED_VISUAL_SUBMICRON_ARTIFACT`
- 对象：`visual__link6__001`；A=旧 fullinertia，B=当前 explicit principal-frame quat+diaginertia。
- 四姿态均由同一 MuJoCo 解释器加载 A/B，设置相同 qpos/qvel 后执行 `mj_forward`，验收使用 `MjData.geom_xpos/geom_xmat`，不使用 compiled local `model.geom_pos/geom_quat` 作为 world-pose 证据。
- production bridge 保持旧 hash 并继续 fail-closed；本任务不执行 hash migration。
- 重力保持关闭；本报告不宣称完整真实动力学验收。

## 精确 18 项最终事实

1. branch / commit: `{"branch": "agent/v15-17-inertial-deployment", "head_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD", "source_commit": "f4132c5514a67b5fc36ed63c7474b33a87fc4747"}`
2. visual__link6__001 OLD / NEW geom_sameframe: `{"new_explicit_principal": 0, "old_fullinertia": 2}`
3. compiled geom_pos delta: `0.0`
4. mechanical zero runtime geom_xpos delta: `{"euclidean_norm_m": 4.0266962935798585e-07, "max_abs_component_m": 3.037324224908211e-07}`
5. mechanical zero runtime rotation delta: `0.0`
6. J6 +0.5 runtime position / rotation delta: `{"position_delta_euclidean_norm_m": 4.026696293469835e-07, "position_delta_max_abs_component_m": 3.0503524697800266e-07, "rotation_delta_rad": 0.0}`
7. J6 -0.5 runtime position / rotation delta: `{"position_delta_euclidean_norm_m": 4.0266962932872377e-07, "position_delta_max_abs_component_m": 3.013112455790834e-07, "rotation_delta_rad": 0.0}`
8. nonzero six-axis pose runtime position / rotation delta: `{"position_delta_euclidean_norm_m": 4.0266962934214373e-07, "position_delta_max_abs_component_m": 3.066520122968486e-07, "rotation_delta_rad": 0.0, "source_sha256": "7a75ab2ee131b7f734346d45a11888ac913e3a49e8c3691aca2d6b998aee013b", "source_target_id": "V15_14_ACCEPTED_FOLLOW_JOINT_TRAJECTORY_FINAL_POSE"}`
9. link6 body runtime maximum position delta: `0.0`
10. link6 body runtime maximum rotation delta: `0.0`
11. collision geoms unchanged: `YES`
12. joint / TF unchanged: `YES`
13. artifact classification: `{"case": "CASE_2", "classification": "VISUAL_ONLY_SUBMICRON_COMPILER_ARTIFACT", "engineering_acceptance": "ENGINEERING_ACCEPTED_VISUAL_SUBMICRON_ARTIFACT", "limitation": "NON_PHYSICAL_VISUAL_NUMERICAL_LIMITATION", "visual_delta_scale": {"acceptance_metric": "EUCLIDEAN_POSITION_NORM; MAX_ABS_COMPONENT_REPORTED_FOR_0P304_MICROMETRE_LEGACY_COMPARABILITY", "max_abs_component_parts_per_million_of_bbox_diagonal": 6.19467501336044, "max_abs_component_relative_to_bbox_diagonal": 6.19467501336044e-06, "max_abs_component_relative_to_bbox_maximum_dimension": 8.761253830744555e-06, "maximum_runtime_position_delta_m": 4.0266962935798585e-07, "maximum_runtime_position_delta_micrometre": 0.40266962935798584, "maximum_runtime_position_max_abs_component_delta_m": 3.066520122968486e-07, "maximum_runtime_position_max_abs_component_delta_micrometre": 0.3066520122968486, "parts_per_million_of_bbox_diagonal": 8.134326179501345, "relative_to_bbox_diagonal": 8.134326179501346e-06, "relative_to_bbox_maximum_dimension": 1.1504541601775098e-05}}`
14. visual-only limitation accepted: `YES`
15. production bridge hash modified: `NO`
16. gravity enabled: `NO`
17. hard unresolved items: `[]`
18. final status: `{"case": "CASE_2", "production_hash_migration_pending": "YES", "v15_17c": "V15.17 INERTIAL_DEPLOYMENT = PASS_WITH_NONPHYSICAL_VISUAL_LIMITATION"}`

## 四姿态 runtime world-pose A/B

| 姿态 | q (rad) | visual Δposition norm (m) | visual max-abs component (m) | visual Δrotation (rad) | link6 body Δposition (m) | link6 body Δrotation (rad) | collision exact/count |
|---|---|---:|---:|---:|---:|---:|---:|
| `mechanical_zero` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | 4.0266962935798585e-07 | 3.037324224908211e-07 | 0.0 | 0.0 | 0.0 | 962/962 |
| `j6_plus_0p5` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.5]` | 4.026696293469835e-07 | 3.0503524697800266e-07 | 0.0 | 0.0 | 0.0 | 962/962 |
| `j6_minus_0p5` | `[0.0, 0.0, 0.0, 0.0, 0.0, -0.5]` | 4.0266962932872377e-07 | 3.013112455790834e-07 | 0.0 | 0.0 | 0.0 | 962/962 |
| `v15_14_accepted_fjt_final_pose` | `[1.1239862708782487, -0.2787640209113898, 1.7562373783755654, 1.1168536669244002, 0.9010670078695902, 0.5253228012929325]` | 4.0266962934214373e-07 | 3.066520122968486e-07 | 0.0 | 0.0 | 0.0 | 962/962 |

## sameframe / compiled-local 诊断

- OLD / NEW `geom_sameframe`：`2` / `0`
- compiled local `geom_pos` delta：`0.0` m
- compiled local `geom_quat` rotation delta：`0.0` rad
- compiled body `ipos` delta：`0.0` m
- compiled body `iquat` rotation delta：`2.1432261808676745` rad
- 上述 compiled-local 字段仅用于解释 sameframe 存储分类，未作为 runtime world-pose 验收量。

## Runtime maxima 与视觉尺度

- visual max position / rotation：`4.0266962935798585e-07` m / `0.0` rad
- link6 body max position / rotation：`0.0` m / `0.0` rad
- joint anchor / axis max：`0.0` m / `0.0`
- TCP/camera TF max position / rotation：`0.0` m / `0.0` rad
- collision max position / rotation：`0.0` m / `0.0` rad
- collision-active（961 active `collision__*` + ground，共 962/pose）exact：`3848/3848`；all exact=`True`
- 全部 963 个 `collision__*` tagged geoms（含两个 contype=conaffinity=0 UpperArm motion geoms，diagnostic）exact：`3852/3852`
- target visual 之外全部 1008 named geoms exact：`4032/4032`；all exact=`True`
- visual max Euclidean delta：`0.40266962935798584` μm
- visual max-abs component delta（与 V15.17B 的 0.304 μm 标量同口径）：`0.3066520122968486` μm
- max-abs component 相对 visual bbox diagonal：`6.19467501336044e-06`（`6.19467501336044` ppm）
- max-abs component 相对 visual bbox maximum dimension：`8.761253830744555e-06`

## Authority 与冻结历史

| Authority / history | SHA256 |
|---|---|
| Mass V1 | `658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a` |
| COM V2 | `1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae` |
| Inertia Engineering V1 | `c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401` |
| V15.17B JSON | `4c50f70b1a8b940e32be40d38b6ced25e99f49528046bf5a8f4d610cfdc36dde` |
| V15.17B Markdown | `a4d36555e4dd9467e4cdd2638c7866999f482e5ad9a2284fe5ebcb08ef0e8d06` |
| V15.14 accepted FJT evidence | `7a75ab2ee131b7f734346d45a11888ac913e3a49e8c3691aca2d6b998aee013b` |
| Runtime mesh manifest | `1174abcfef3b87ee77d4ce50af5afc1aa62d45759f1560ace361997e8c60b19d` |
| link6 visual mesh | `39d0a457706587ddb9b07d920d7916300c660791e7a61b3759f3be970cbc9d13` |

- 六体 frozen tensor compiled reconstruction：`True`
- production hash migration pending：`True`

## Hard unresolved items

- `[]`

## 模型边界

旧 fullinertia 模型仅用于同解释器 A/B runtime world-pose 诊断，不是 physics authority。V15.17C 未修改 inertia、MJCF、Xacro/URDF、geometry/mesh、production bridge、controller、armature、friction 或 damping；未执行 hash migration，未开启重力。V15.17B JSON/Markdown 保留为冻结历史证据。
