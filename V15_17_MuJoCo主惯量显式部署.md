# V15.17B MuJoCo 主惯量显式部署

- 最终状态：**V15.17B EXPLICIT_PRINCIPAL_INERTIA = FAIL**
- 状态名称：`EXPLICIT_PRINCIPAL_INERTIA_NOT_ACCEPTED`
- 审计有效：`True`
- 范围：`link2`、`link3`、`link4`、`link5`、`link6`、`gripper`
- 表示：`EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA`；冻结 authority 仍是 V15.16 owner-link-frame full tensor。
- production bridge：`BLOCKED_PENDING_HASH_MIGRATION`，不归类为 physics regression，也不执行 hash migration。
- 重力保持关闭；本报告不是完整真实动力学验收。

## 精确 24 项最终事实

1. branch / commit: `{"branch": "agent/v15-17-inertial-deployment", "head_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD", "source_commit": "f4132c5514a67b5fc36ed63c7474b33a87fc4747"}`
2. synthetic fullinertia error: `1.0258546895366918e-06`
3. synthetic quat+diaginertia error: `6.85083791643594e-16`
4. link2 principal moments / quat / compiled tensor error: `{"compiled_tensor_relative_frobenius_error": 6.85083791643594e-16, "input_quaternion_wxyz": [0.9956682729654489, -0.053405864390123244, -0.00327509762119186, 0.07603800098842532], "principal_moments_kg_m2": [0.0010041105324102276, 0.007363535125872096, 0.007408318752898626]}`
5. link3 principal moments / quat / compiled tensor error: `{"compiled_tensor_relative_frobenius_error": 9.399688243242385e-16, "input_quaternion_wxyz": [0.7003303447278717, 0.7026443310182399, 0.0889348412149297, -0.08898846194505229], "principal_moments_kg_m2": [0.0007863702207150335, 0.012163930853475871, 0.01266270072813982]}`
6. link4 principal moments / quat / compiled tensor error: `{"compiled_tensor_relative_frobenius_error": 3.223021180403743e-16, "input_quaternion_wxyz": [-0.692769943343069, 0.0892825384905402, 0.7106320975675021, -0.08426420252953525], "principal_moments_kg_m2": [0.00042205401921073743, 0.001535230761345874, 0.001785516906253784]}`
7. link5 principal moments / quat / compiled tensor error: `{"compiled_tensor_relative_frobenius_error": 3.4445972162808943e-16, "input_quaternion_wxyz": [-0.6083435682918202, 0.0008958574958809724, 0.7936684229227567, 0.0027811531109689916], "principal_moments_kg_m2": [0.0002696744802799555, 0.0007038103474563456, 0.0007229583936251443]}`
8. link6 principal moments / quat / compiled tensor error: `{"compiled_tensor_relative_frobenius_error": 3.430044546854164e-16, "input_quaternion_wxyz": [0.9991296568278667, -2.86131461404744e-15, 7.721002835442155e-14, -0.04171245433955482], "principal_moments_kg_m2": [1.151829969278952e-05, 1.1518299692790569e-05, 2.301583887297393e-05]}`
9. gripper principal moments / quat / compiled tensor error: `{"compiled_tensor_relative_frobenius_error": 7.944022324086645e-16, "input_quaternion_wxyz": [0.3437493074433223, 0.5735195693517792, 0.6648727695741156, 0.3329503228435606], "principal_moments_kg_m2": [0.00037497547647326, 0.0003938585140950338, 0.0004740673795944088]}`
10. maximum Python eigh reconstruction error: `1.3907442991267022e-15`
11. maximum input quaternion rotation error: `6.661338147750939e-16`
12. maximum compiled quaternion rotation error: `3.3306690738754696e-16`
13. maximum six-body tensor relative error: `9.399688243242385e-16`
14. six-body mass: `PASS`
15. six-body COM: `PASS`
16. URDF unchanged and PASS: `YES`
17. joint / TF / collision unchanged: `NO`
18. fullinertia still present: `NO`
19. explicit quat+diaginertia used: `YES`
20. MuJoCo compile: `PASS`
21. production bridge hash modified: `NO`
22. gravity enabled: `NO`
23. hard unresolved items: `["LINK6_LEGACY_FULLINERTIA_GEOM_SAMEFRAME_ARTIFACT"]`
24. final status: `{"production_hash_migration_pending": "YES", "v15_17b": "V15.17B EXPLICIT_PRINCIPAL_INERTIA = FAIL"}`

## Authority

| Authority | SHA256 |
|---|---|
| Mass V1 | `658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a` |
| COM V2 | `1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae` |
| Inertia Engineering V1 | `c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401` |

## 六体显式 principal-frame 回读

| Link | principal moments (kg m^2) | input quat WXYZ | Python eigh rel-F | compiled tensor rel-F | compiled R delta |
|---|---|---|---:|---:|---:|
| `link2` | `[0.0010041105324102276, 0.007363535125872096, 0.007408318752898626]` | `[0.9956682729654489, -0.053405864390123244, -0.00327509762119186, 0.07603800098842532]` | 8.445416784132799e-16 | 6.85083791643594e-16 | 1.1102230246251565e-16 |
| `link3` | `[0.0007863702207150335, 0.012163930853475871, 0.01266270072813982]` | `[0.7003303447278717, 0.7026443310182399, 0.0889348412149297, -0.08898846194505229]` | 1.0246599135432833e-15 | 9.399688243242385e-16 | 3.0357660829594124e-16 |
| `link4` | `[0.00042205401921073743, 0.001535230761345874, 0.001785516906253784]` | `[-0.692769943343069, 0.0892825384905402, 0.7106320975675021, -0.08426420252953525]` | 4.544526998155102e-16 | 3.223021180403743e-16 | 2.0122792321330962e-16 |
| `link5` | `[0.0002696744802799555, 0.0007038103474563456, 0.0007229583936251443]` | `[-0.6083435682918202, 0.0008958574958809724, 0.7936684229227567, 0.0027811531109689916]` | 3.970259914430953e-16 | 3.4445972162808943e-16 | 1.1102230246251565e-16 |
| `link6` | `[1.151829969278952e-05, 1.1518299692790569e-05, 2.301583887297393e-05]` | `[0.9991296568278667, -2.86131461404744e-15, 7.721002835442155e-14, -0.04171245433955482]` | 4.923609222735492e-16 | 3.430044546854164e-16 | 1.1102230246251565e-16 |
| `gripper` | `[0.00037497547647326, 0.0003938585140950338, 0.0004740673795944088]` | `[0.3437493074433223, 0.5735195693517792, 0.6648727695741156, 0.3329503228435606]` | 1.3907442991267022e-15 | 7.944022324086645e-16 | 3.3306690738754696e-16 |

## A/B generalized physics 诊断

- qpos max delta：`0.0`
- body pose max delta：`0.0`
- joint anchor/axis max delta：`0.0`
- all named geom pose max delta：`3.037324224908211e-07`
- collision geom pose max delta：`0.0`
- all-geom 字面硬门：`False`
- generalized mass matrix rel-F delta（diagnostic only）：`1.998269482119427e-09`

## Hard unresolved items

- `LINK6_LEGACY_FULLINERTIA_GEOM_SAMEFRAME_ARTIFACT`

## 模型边界

旧 fullinertia 模型仅用于回归与 generalized mass-matrix 诊断，不是 physics authority。当前未执行 production hash migration，未开启重力，也未修改 controller、armature、friction 或 damping。
