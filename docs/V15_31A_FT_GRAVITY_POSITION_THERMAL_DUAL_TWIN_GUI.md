# V15.31A-FT 全臂重力感知位置控制、热管理与双数字孪生 GUI

## 当前结论与安全边界

本轮按机械臂断电条件完成软件层实现与离线验证。未打开 24 V、未访问电机总线、未启动硬件 worker，也未执行 ROS 实机链路。因此，本文中的“已实现”只表示代码路径、离线契约和保护逻辑已经建立，不代表实机位置精度、重力方向、连续承载能力或热稳定性已经通过。

唯一 primary blocker 是机械臂断电：断电条件下既不能建立与下一次真实硬件会话及编码器分支严格绑定的 `MODEL_SESSION_ANCHOR_V2`，也不能完成方向、连续承载、热稳定和最终制动验收。重力节点 → Router → GO worker 的软件前馈链已经接通，但冻结 `gravity_control.yaml` 明确写入 `continuous_rotor_limits_authoritative: false`，其 SHA256 也被代码冻结；因此当前链路必然 fail-closed，`feedforward_nm` 与 `Tff` 保持为零，现实 POSITION 不会获得重力 authority。启动参数 `enabled_for_hardware` 默认仍为 `false`，单独改变该参数或比例值不能绕过连续力矩 authority。

`whole_arm_gravity_node` 本身仍不持有电机 socket，也不直接发布电机命令。它发布短时有效、会话绑定且防重放的重力状态；Router 是唯一允许把该状态转换成 worker authority 的组件，GO worker 再独立校验并执行第二级 slew。当前冻结配置使这条已接通的软件链在 Router 前稳定截止，而不是让节点直接驱动硬件。

在完成操作员姿态对齐、重力方向验证、分级前馈、逐轴 ±5° 验收和热稳定测试前，不得把本版本判为 FULL PASS，也不得据此给机械臂上电运动。

## 基线与冻结项

- 源分支：`snapshot/v15-30e-current-20260830`。
- 源提交：`093bebcce2aaa98dc75171a826e7714f0024b399`。
- 工作分支：`agent/v15-31a-ft-gravity-position-thermal-dual-twin-gui`。
- 冻结 production MuJoCo 模型 SHA256：`5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9`。
- 重力配置 SHA256：`307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d`。
- GO 减速比：`6.329999923706055`。
- 电机内部零位、CAD 零位、ROS 零位和冻结 MuJoCo 模型均未修改。
- 本阶段没有实现零重力／Teach 拖动、MoveIt、逆运动学、笛卡尔控制、视觉或手眼标定。

## 当前架构

GUI、状态汇聚、碰撞守卫、重力解算和电机 worker 彼此分责：

1. `whole_arm_state_node` 汇聚七颗物理电机反馈，生成六个逻辑关节的 `q_actual`、新鲜度、通信、温度、模式、J2 同步状态和轨迹反馈。
2. GUI 只用唯一一组滑条维护 `q_plan_target`；滑条编辑不会改变 `q_actual`，也不会创建或发送 `q_hardware_command`。
3. `workflow_contract` 从当前 `q_actual` 生成不可变的分段五次轨迹配方。预演与真机 worker 使用同一组起点、终点、时长、整数采样网格、分段顺序和轨迹 SHA256。
4. `whole_arm_mujoco_mirror` 异步完成路径限位与碰撞检查。仅 `execute` 类型的新鲜证明能授权 Router；用于多段候选预检查的 `plan_preview` 证明不能授权真机。
5. GUI 只有在完整预演检查通过后才签发一次性 `PLAN_TOKEN`；用户明确点击“下发到现实”后，GUI 才能建立 `q_hardware_command`。
6. `arm_gui_command_router` 再次验证会话、来源租约、激活 epoch、计划 manifest、逐段碰撞证明和不可变轨迹描述符，然后分别转发到 GO 与 J6 worker。
7. GO worker 控制 J1、J2A/J2B、J3、J4、J5；J6 worker继续使用已验证的 DM-G6220 `POS_VEL` 路径。各自持续回传轨迹样本索引、位置、温度、通信和故障状态。
8. `whole_arm_gravity_node` 使用独立的控制专用 `MjModel/MjData` 计算静态 `qfrc_bias`，发布诊断、六逻辑关节重力矩以及经 ramp／slew 后的六逻辑前馈向量；它不复用任何 GUI 渲染 `MjData`，也没有电机命令发布器。
9. Router 独立订阅重力状态，仅在模型／配置哈希、session、状态实例、序列、新鲜度、离散比例、连续力矩 authority 和转子限幅全部成立时，才紧邻 UDP 发送前注入 `gravity_authority` 与 `feedforward_nm`；GUI 不能自带或伪造这些字段。
10. GO worker 再次严格校验重力 authority，把逻辑 J2 前馈按既定符号映射到双电机，并使用 worker 内第二级 slew 后的 `Tff + position_PD + bounded_endpoint_correction` 形成线上命令。当前配置的连续力矩 authority 为 `false`，所以现实链路不会到达此非零分支。

四类状态在代码中独立保存：

- `q_actual`：真实编码器反馈；
- `q_plan_target`：用户编辑的虚拟目标；
- `q_plan_trajectory`：已经生成并用于虚拟预演的轨迹；
- `q_hardware_command`：一次性计划授权被明确消费后，真实硬件正在执行的目标。

## 虚拟先行与位置轨迹同源

计划生成器使用 rest-to-rest quintic。默认最大速度为 5°/s，最大加速度为 15°/s²，采样周期不大于 10 ms（100 Hz）。多关节目标按单移动轴分段，每个分段仍是完整六维姿态；未运动关节保持前一分段终点。每个分段的时长对齐到整数纳秒，worker 和计划视图都按同一个整数样本索引推进，并精确钉住起点和终点。

`PLAN_TOKEN` 绑定至少以下 authority：

- `session_id` 与 `state_instance_id`；
- 候选修订号、当前 `q_actual` 及其哈希；
- `q_plan_target` 及其哈希；
- 轨迹配方／分段 SHA256、profile、时长和采样周期；
- 关节限位哈希、冻结模型哈希和重力配置哈希；
- 完整预演检查哈希、签发时间与 nonce。

改变滑条、实际姿态漂移超过阈值、会话／状态实例改变、反馈或通信过期、温度不可执行、重力 authority 失效、碰撞证明失效、模型或配置 authority 改变，都会撤销旧 token。“下发到现实”是唯一能写入 `q_hardware_command` 的转换，且 token 只消费一次。普通运行 gate 未满足时不会提前消费 token；但任何 authority 改变后必须重新预演。

GO 与 J6 的 V1.3 POSITION 命令携带相同的不可变十字段轨迹描述符。worker 不在本地重新规划轨迹，而是按 `execute_at_monotonic_ns`、`duration_ns` 和 `interval_count` 选择完全相同的样本；同一来源和 activation epoch 内不得更换目标、token 或轨迹合同。旧 V1.2 `BRAKE` 仍兼容，但 production Router 默认拒绝 V1.2 POSITION。

整轨重力／负载证明由重力节点独立重建并哈希同一份 immutable recipe：对每个精确样本计算 `qfrc_bias`，同时用 `mj_inverse` 统计含速度／加速度的预测转子峰值，再分别检查 J2 50/50 分担、连续转子力矩、短时峰值、当前温度和到降额阈值的余量。证明必须同时绑定轨迹、session、状态实例、模型及重力／热配置哈希且新鲜；旧轨迹的 PASS/FAIL 不能影响新轨迹。当前连续力矩 authority 为 false，因此只能产生明确 `BLOCKED`，不会伪造轨迹热负载 PASS。

## 重力计算、前馈和物理限制

重力前馈不是免费能量，也不会减少机械臂维持某姿态所需的物理静态力矩。它能减少为了产生静态力矩而保留的位置误差、积分饱和和位置环与负载的无效对抗，从而改善保持与双向跟踪；它不能让超过电机连续力矩或散热能力的姿态被软件永久保持，也不能靠提高温度阈值解决持续过载。

重力解算的预定运行路径是：读取新鲜 `q_actual`，用有效 `MODEL_SESSION_ANCHOR_V2` 映射到模型绝对姿态，设置 `qvel=0`、`qacc=0`、重力为 `[0, 0, -9.81] m/s²`，执行 `mj_forward()`，再读取 J1～J6 对应的 `qfrc_bias`。这代表当前实际姿态下的静态 gravity/bias 力矩，不使用虚拟目标，也不使用 GUI renderer 的状态。

控制 authority 使用 `/whole_arm/hardware_state` 中原子绑定的 `position_rad`、`session_id`、`state_instance_id`、sequence 和单调时间戳；`/joint_states` 仅作为界面镜像的诊断交叉检查，不能替代带会话／序列身份的硬件姿态，也不能单独授权重力前馈。

`MODEL_SESSION_ANCHOR_V2` 是会话软件元数据，不是电机零位。它必须绑定冻结模型哈希、当前 `session_id`、`state_instance_id`、七电机方向与减速比、原始编码器参考、编码器分支、六轴逻辑参考以及模型绝对姿态。缺失、字段不全、哈希不符、会话或编码器分支变化时必须失效。

目前已经实现并离线覆盖了重力模型读取、静态解算、关节到转子映射、比例 ramp、转子力矩 slew limiter，以及重力节点 → Router → GO worker 的完整软件传递与多层校验。比例目标只接受 25%／50%／75%／100% 四档；启停必须平滑 ramp，不能一步跳变。当前不是“软件链未接入”，而是冻结配置明确拒绝连续承载 authority：`continuous_rotor_limits_authoritative=false` 会令重力状态报告 `BLOCKED_CONTINUOUS_LOAD_AUTHORITY`，Router 不签发 authority，GO 因而不能收到非零 Tff。

下一阶段只能在上电后取得有效 anchor，完成实机方向、连续承载和热验证，再通过受审计的新配置、相应新哈希及代码 authority 共同解除此门；禁止只把布尔值改成 `true`。现场首次验证仍须按 25% → 50% → 75% → 100% 逐档执行，并保留每档的操作员观察、同步、通信、温度和位置保持证据。

## GO 力矩语义与 J2 50/50 分配

`TORQUE_SEMANTICS_AUTHORITY = PASS`，其范围仅限字段物理语义、换算与 Q8 编解码的离线证据；它不代表连续力矩额定值已知，也不授权硬件重力前馈。

软件字段必须带侧别和单位：

- `tau_cmd_rotor_nm`：GO 电机转子侧命令力矩，单位 N·m；
- `tau_feedback_rotor_nm`：GO 电机转子侧反馈力矩，单位 N·m；
- `tau_joint_estimated_nm`：单电机反馈按方向和减速比换算的逻辑关节估算力矩，单位 N·m；
- `tau_j2_logical_total_nm`：J2A 与 J2B 两颗电机有符号合成的 J2 逻辑总力矩，单位 N·m。

冻结 GO SDK 的 `MotorCmd.tau` 和 `MotorData.tau` 都按转子侧 N·m 解释。线上格式为有符号 Q8：编码为朝零截断的 `tau_cmd_rotor_nm × 256`，有效命令计数限幅为 ±32765；反馈按 `raw_count / 256` 解码。3.2/4.07 N·m 的现场字段按转子侧反馈解释，不再重复乘减速比后当成同侧数据。历史约 `0.6015625 N·m` 是软件保护阈值，不是厂商连续额定力矩，也不能用来证明连续承载能力。

单 GO 电机的基本换算为：

```text
tau_rotor_nm = direction_sign × tau_joint_nm × load_share / G
tau_joint_estimated_nm = direction_sign × tau_feedback_rotor_nm × G
```

J2 的 MuJoCo 输出是一个逻辑关节总重力矩，必须平均分给两颗电机：

```text
tau_cmd_J2A_rotor_nm = -gravity_scale × tau_J2_joint_nm / (2G)
tau_cmd_J2B_rotor_nm = +gravity_scale × tau_J2_joint_nm / (2G)
```

禁止把完整 J2 总力矩同时给 J2A 和 J2B。J6 保持 DM-G6220 `POS_VEL`，该通道没有权威 torque 命令／反馈，因此 J6 对上述三个逐电机 torque 字段显式发布 `null`，而不是伪造零力矩测量。

[达妙官方 DM-G6220 页面](https://www.mdmbot.com/index.php?c=show&id=119) 标注额定力矩 1.3 N·m、峰值力矩 2.7 N·m。这些是 J6 电机的厂商参数，不能倒推 GO 电机的连续力矩，也不能替代整机持续占空比、散热、装配和实际温升验收。

## 位置控制与堵转保护

GO 位置控制的软件组成已经形成 `gravity_ff(q_actual) + position_PD + bounded_endpoint_correction`。共享轨迹、PD／有界近端修正、命令 authority、两级 Tff slew 和无进展保护均在同一 worker 路径内；但有效 anchor、实机方向和连续承载 authority 尚未建立，冻结配置会在 Router 前把现实 POSITION 失败关闭，因此不能把现状描述为“全臂重力位置控制已实机通过”。J6 继续使用 `POS_VEL`、当前位置 preload、目标刷新和到位后固定目标保持，不接收 GO 的 torque 字段。

GO 的 `NO_PROGRESS / SATURATION` watchdog 独立于温度保护：在 POSITION 中误差至少 2°、软件输出达到保护限／FOC 饱和且 3 秒内改善不足 0.25°（至少 100 个有效帧）时，锁存 `LOAD_LIMIT_NO_PROGRESS`，撤销轨迹并对对应故障域 BRAKE。worker 保持在线并继续反馈；必须先收到明确 release，再使用更高 activation epoch，于下一控制周期重置。这里使用的输出限只标记为 `SOFTWARE_GUARD_NOT_CONTINUOUS_RATING`，不得当作电机连续力矩 authority。

GO 只有在轨迹已经进入授权端点、误差连续保持在 0.5° 到位窗内不少于 0.5 秒且累计不少于 50 个有效反馈帧后，才确认稳定到位；任一帧漂出到位窗、端点状态撤销或反馈失效都会清零稳定窗口并撤销既有到位标志，旧的瞬时到位不能授权 HOLD。若从首次进入端点起 90 秒仍未形成上述稳定到位，`POSITION_ARRIVAL_TIMEOUT` 会触发同一位置安全锁存、立即补发整故障域 BRAKE、清除活动轨迹与积分状态，并原子回显触发原因。恢复同样要求明确 release、有效 BRAKE 反馈和更高 activation epoch，且只在下一控制周期生效。

J6 也把持续目标超时且位置没有足够改善归入锁存的 `LOAD_LIMIT_NO_PROGRESS`：撤销活动轨迹、验证并保持 DISABLED、worker 在线继续采集反馈。恢复同样需要明确 release、更高 activation epoch，并在下一控制周期 rearm；目标超时不能只打印日志后继续无限刷新 `POS_VEL`。

## 七电机热管理

软件为 J1、J2A、J2B、J3、J4、J5、J6 分别保留原始温度、窗口中位数、温升斜率、锁存状态和恢复 epoch，状态枚举为 `NORMAL`、`WARNING`、`DERATING`、`THERMAL_STOP`、`COOLDOWN`、`WAIT_OPERATOR_CONFIRM`，另有 `OFFLINE`。当前保守项目阈值为：45°C 以下正常，45～55°C 预警，55°C 进入热保护区，任一原始温度达到 60°C 立即热停机；exact v1.3 与 legacy/HOLD 在 55～60°C 的控制动作不同，不能笼统描述成所有轨迹都在线降额。这些阈值不是厂商连续额定证明。

GO worker 的 60°C 热停不会退出进程，而是让对应故障域 BRAKE 并继续采集反馈。重新允许动作必须同时满足：温度低于 55°C、连续冷却不少于 30 秒且不少于 500 个有效帧、操作员明确 release、使用更高 activation epoch，并在下一周期完成 rearm。J2A/J2B 属于同一故障域，任一过温会锁存双电机。J6 在 MOS 或线圈温度达到 60°C 时锁存故障、验证 DISABLED 后保持 worker 在线；七电机统一状态层与 GUI 会继续报告温度和故障。

GO 的 exact v1.3 五次轨迹把预演签名覆盖的 `q/dq` 样本视为不可变合同：执行中任一所属电机进入 ≥55°C 热保护区时，不再独立缩放 `q`、`dq`、`vmax` 或 `amax`，而是以 `EXACT_TRAJECTORY_DERATING_ABORT` 锁存整个故障域 BRAKE、撤销当前轨迹并要求重新预演。只有完成 release、冷却条件、更高 activation epoch 和下一周期 rearm 后，新的预演／token 才能再次执行。legacy profile 与 HOLD 仍保留动态 `vmax`／`amax`／`Kp`／`Kd` 降额并停止积分学习；重力静态前馈不随该降额缩小，因为缩小它会把同一物理静态负载重新推回位置误差与 PD 环。达到 60°C 时，任何命令类型都只能保持锁存 BRAKE。

J6 的 exact v1.3 五次轨迹采用相同的不变预演合同：MOS 或线圈温度进入 ≥55°C 时立即热锁存并验证 DISABLED，不在线独立缩放签名轨迹的 `q/dq`；完成 release、冷却、更高 activation epoch 和下一周期 rearm 后仍必须重新预演。legacy `POS_VEL` profile 与 HOLD 才在 55～60°C 连续动态降额。任一原始温度达到 60°C 时始终锁存热停机，worker 在线继续反馈。

共享热配置 `thermal_limits.yaml` 的冻结 SHA256 为 `1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467`；GO 与 J6 在打开串口／CAN 之前完成文件读取、格式检查和 SHA256 比对，不匹配即失败关闭。GO 回显同时提供逐电机 `thermal_state_by_motor`，并保留顶层 `thermal_state` 作为故障域汇总状态，避免单个电机状态被域级字段覆盖。

所有即时故障域 BRAKE 路径都会把该次 mode-0 事务返回的温度、故障码、位置、速度、力矩和返回模式一起提交到同一状态快照；只有已确认返回 mode-0 的电机才可回显 `brake_observed=true`。无有效确认时，相应逐电机模式必须是 `unknown` 且 `brake_observed=false`，不能用控制器的意图代替实际 BRAKE 观测。

`continuous_rotor_torque_limit_nm` 和 `short_peak_rotor_torque_limit_nm` 当前均为 `null`，冻结重力配置也因此明确设置 `continuous_rotor_limits_authoritative=false`。协议最大值、软件保护值和短时未过温都不能替代官方连续力矩或有界热试验 authority，因此预演中的“持续力矩／热负载可行性”仍不能在实机 gate 前宣称通过。

## 双 MuJoCo 与任务状态

GUI 同时创建两个独立渲染器：

- 左侧“计划／虚拟机械臂（不代表真机）”只读取 `q_plan_target`、预演轨迹以及真机执行时的同源计划样本；
- 右侧“现实机械臂数字孪生（仅编码器）”只读取新鲜 `q_actual`，现实未运动时不会被滑条或计划轨迹带动。

两个视图加载同一冻结模型并分别拥有自己的 `MjData`，可以独立旋转和缩放。每个 renderer 都由自己的单线程后台 executor 创建、使用和销毁 OpenGL context；渲染请求只保留一个最新待处理帧进行 coalescing，慢帧不会形成无界队列，也不在 Qt 事件线程执行 MuJoCo 渲染。精确分段轨迹生成和 planned-request JSON 序列化同样在 latest-generation 单线程 worker 完成；Qt 回调只冻结不可变快照并提交，完成后必须重新核对 generation、session、状态实例、候选、实际漂移、配置、反馈新鲜度和请求年龄才允许发布。窗口关闭使用非等待 shutdown，晚到结果不能改变工作流或获得授权。碰撞守卫和重力计算还各自使用隔离的模型数据，避免渲染状态进入安全计算或控制计算。

“当前任务状态”面板显示任务 ID、中文状态、当前阶段、进度、开始／已用／预计剩余时间、控制心跳、最后有效物理编码器反馈、目标、实际、最大误差、最高温度、热状态、整轨最大预测重力矩／电机力矩、限位／碰撞／热负载结果和失败原因。“最后反馈”来自 worker 最后一个有效物理帧的单调时间，无效数据报不会刷新它。正常 POSITION 轨迹、endpoint dwell 和明确的 90 秒到位超时分别显示，不会把整个正常执行阶段误报为“未到位”。GUI 以 200 ms 级刷新汇总，ROS 回调每次 Qt tick 有界排空；碰撞检查通过独立节点和请求／结果状态机执行，避免在滑条变化时同步控制真机。七行“物理电机状态”表分别显示 J1、J2A、J2B、J3、J4、J5、J6，并用中文文字和颜色同时表达离线、正常、温度预警、升温明显（50°C 以上的预警显示为橙色）、热降额、热停机、冷却中和等待确认；颜色不是唯一状态信息。

## 恢复初始化姿态

主界面只提供“恢复初始化姿态”。生产恢复 authority 必须是状态节点已经加载的同一份初始姿态文件：硬件参考类型为 `PERSISTENT_SOFTWARE_ZERO_V1`，文件内会话标识与 persistent zero 哈希绑定，且文件原始字节 SHA256 与状态流中的 `initial_pose_sha256` 一致。打包的默认 `initial_pose.json` 当前标记为无效，因此不能直接恢复。

authority 有效时，按钮只把初始姿态装入 `q_plan_target` 并启动与普通目标相同的虚拟预演；限位、碰撞、重力和热 gate 通过后，仍需用户明确下发，绝不会直接驱动真机。authority 无效、跨会话或哈希不一致时显示“初始化姿态需要重新设置”，不运动。这个软件恢复点也不是电机内部零位、CAD 零位或 ROS 零位。

## 断电条件下的验证结果与剩余限制

本轮只允许离线单元测试、静态契约检查、编译和不打开串口的 dry-run。终审发现的稳定到位门、exact v1.3 热保护语义、逐电机热状态／共享配置 authority、即时 BRAKE 原子回显、exact 轨迹 governor 中止、整轨负载证明、力矩字段契约、最后有效反馈与任务状态分层等问题已在软件工作树修复并登记于 bugfix inventory；它们仍只是断电条件下的实现与离线审查结果。离线检查可以覆盖滑条不直接下发、预演／执行描述符同源、token 失效规则、重力节点 → Router → GO 的 authority 传递与失败关闭、GO／J6 轨迹采样、torque Q8 与 J2 分配、热锁存、no-progress watchdog、双视图独立 `MjData`、GUI 轨迹构造与重力整轨解析的 latest-generation 后台单线程 coalescing，以及初始化恢复拒绝路径；所有 UTF-8 字节计数、JSON 解码、完整轨迹哈希重建、重放检查和逐样本 MuJoCo 评估均不在 100 Hz ROS 订阅回调中执行。它们不能代替编码器、模式回包和温度实测。

最终纯离线验证结果为：ROS 2 Humble 隔离工作区 `colcon build` 两个包通过，`colcon test` 共 578 项通过；分组 `pytest` 共 863 项及 86 个 subtests 通过、零失败；GO C++ 以 `-Wall -Wextra -Werror` 编译并完成内建 self-test；GO 无参数运行回显 `SERIAL_OPENED=NO`，J6 不带 `--execute` 运行回显 `CAN_OPENED=NO`。完整命令、环境、退出码和分组计数保存在 `hardware/v15_31a_ft/current_snapshot_audit.json`。

以下实机项目尚未执行，状态均应记为 `BLOCKED_POWER_OFF` 或 `PENDING`，不得写成 PASS：

- 建立并验证下一次会话的 `MODEL_SESSION_ANCHOR_V2`；
- 确认 J1～J5 重力前馈方向及 25%／50%／75%／100% 分级效果；
- J1～J6 的 `0 → +5° → 0 → -5° → 0` 位置精度与每端点 0.5 秒稳定；
- J2A/J2B 运动同步与端点同步误差；
- 小范围多关节轨迹和恢复初始化姿态；
- J2/J3 的 5、15、30 分钟代表性负载热稳定性；
- 真实终态 BRAKE／J6 DISABLED 以及断电后的完整 evidence。

是否需要机械配重当前不能判定，结论是 `UNKNOWN_PENDING_POWERED_VALIDATION`，不是 `NO`。只有在 anchor 映射、力矩方向、真实保持力矩和 30 分钟热趋势可用后，才能区分 `CONTROL_EXCESS_TORQUE` 与 `FUNDAMENTAL_CONTINUOUS_LOAD`，并计算需要减少的 J2 重力矩、配重质量、弹簧／气弹簧力或电机／减速比能力。若物理连续负载超限，即使软件控制改善，也必须报告需要机械补偿，不能伪造 `HEATING_FULLY_SOLVED`。

当前任务分类只能是 `PARTIAL_PASS`，`READY_FOR_NEXT_STAGE = false`；唯一 primary blocker 是机械臂断电，导致无法建立可用 `MODEL_SESSION_ANCHOR_V2` 并完成实机验收。
