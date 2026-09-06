# V15.31D：关节保持证据与通信恢复核查

核查日期：2026-09-06。时间以下均为 UTC；远端 `car` 的系统显示时区为 UTC-4。

## 基线与现存工作

- 本地工程目录：`D:\AI_GOM8010_6\go-m8010-robot-arm-v15-31a-ft`。
- 远端工程：`/home/car/worktrees/go-m8010-v15-31b-ft-afda94da4c0d`。
- 核查时远端为 detached `cbe8aa33d472d63c427ec20ac1a946c29b9c8d37`；未跟踪的 `hardware/v15_31b_ft/` 保持原状。
- 该目录含 9 月 5 日的上电只读、模型锚点和重力只读验证。未发现完成并封存的本轮六轴位置/长期热验收包。
- 7 颗电机对应 6 个关节：J2A/J2B 双电机同驱 J2，DM-G6220 为 J6。

## 设角、保持实际证据的范围

下表为历史实机记录，不能表示本次上电后的结果。各历史 PASS 对应当时的局部试验阈值，不等于当前 ±0.1° 严格验收通过。

| 关节 | 已记录实机结果 | 限制与出处 |
|---|---|---|
| J1 | 2026-08-15 双向 10° 完成；正向峰值 9.9033°，负向 -9.5024°，首回中误差 0.7810°，末回中 0.0347°；端点保持约 0.2–0.3 秒 | `hardware/v15_19d/j1_final_motion_test.json`；不是长期保持。最新 9 月 5 日 GUI 正向目标 5.02682°、中止帧约 4.758°，验收 PLUS_5 未完成，见下文 |
| J2 双电机 | 最终装配双向完成：+5° 实测 5.1374°、误差 0.1374°；-5° 实测 -4.4722°、误差 0.5278°；末回中误差 0.3141°；最大同步误差 0.3176°、最高 34°C | `hardware/v15_30e_ft/j2_coupled_sync_result.json`；由两次有界运行拼合双向完成，不是单次完整路线；无长期热保持通过结论 |
| J3 | 低载两次双向 5°完成，端点误差约 0.224–0.286°；预保持漂移约 0.0035° | `hardware/v15_23c_d3_ft/j3_kp060_inventory.json` 明确最终装配带载保持尚未测试，不能用低载结果替代 |
| J4 | 当前姿态保持漂移 0.00347°；双向 5°端点误差 +向 0.8021°、-向 0.0115°，首回中误差 0.9164° | `hardware/v15_22b/j4_motion_inventory.json`；历史局部试验，非全范围/精密保持 |
| J5 | 当前姿态保持漂移 0.02777°；双向 5°端点误差 +向 0.4354°、-向 0.0414°，末回中误差 0.1944° | `hardware/v15_22a/j5_motion_inventory.json`；记录说明测试姿态不易下垂，不能推为任意带载姿态 |
| J6 | POS_VEL 双向 5°完成，端点误差 0.0822°/0.0166°；末回中误差 0.7213°，终态 5 帧 DISABLED | `hardware/v15_21e/j6_posvel_local_motion_inventory.json`；不是六轴组合或长期保持验收 |

最新主动记录为远端 `.runtime/v15_31b_ft/current/attended_j1_gui.log`：2026-09-05 约 10:12 UTC，`ATTENDED_J1_FAIL`，目标 `0.08773453921788184 rad`，中止反馈 `0.08304391933934022 rad`，差约 `0.2688°`。内部验收原因 `POSITION_PLUS_ACTUAL_MOTION_BELOW_5_DEG`，GUI 同时记载 `feedback not fresh and healthy`。它证明确有运动尝试，没有证明整段或固定目标保持通过。

当前 GO 控制链具有 PD、重力前馈和有界保持积分；驱动内部到位标志仍使用 0.5° 内连续 0.5 秒/50 帧，超时 90 秒。`tools/validate_v15_31b_ft_evidence.py` 与主动验收器另行执行版本化 ±0.1° 契约。因此驱动“arrived”、路由接受、仿真成功都不能用作严格精度验收。多关节实机目标仍应按当前逐轴分段能力描述。

## 2026-09-06 07:26–07:28 UTC 实测通信异常

- GUI/state/router/mirror 在运行，GO 三域旧工作进程均为 `--execute --brake-only`；未见 J6 控制进程。
- `whole_arm_state_capture.csv` 最新帧 `healthy=0`：GO 六电机 `communication_ok=0`；J6 保留 `communication_ok=1` 但反馈年龄约 76,456,103 ms（21.2 小时），该 J6 数据不具实时效力。屏幕上的旧角度/温度也不能当作当前测量。
- J2 日志持续 `transport_ok=0`、返回 mode/id 为 255、位置/速度/力矩为 NaN。J1/J345 有 `motor does not reply`。
- `/proc/2111999/fd/5 -> /dev/ttyUSB6 (deleted)`（J1）；`/proc/2160366/fd/5 -> /dev/ttyUSB4 (deleted)`（J2）；`/proc/2160369/fd/7 -> /dev/ttyUSB5 (deleted)`（J345）。设备已经重新枚举，旧进程仍持有失效描述符。
- 只观测到 UDP 15300 的反馈接收端；无 GO 命令端口。旧 supervisor JSON 已过时，不能证明当前可控制。
- 本核查当时仅据反馈判断“通信失效”，没有推断 24V 状态。后续用户明确说明已发生断电重启并确认当前竖直、支撑可靠、接线正确；这属于用户提供的当前现场条件。

固定设备映射（不要使用会变化的 ttyUSB 数字）：

| 域 | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-` 后缀 | 反馈/命令 UDP |
|---|---|---|
| J1 | `if03-port0` | 15300 / 15310 |
| J2 | `if01-port0` | 15300 / 15312 |
| J345 | `if02-port0` | 15300 / 15313 |

旧日志位于远端 `.runtime/v15_31b_ft/current/manual_restore_j1_brake.log`、`manual_restore_j2_brake.log`、`manual_restore_j345_brake.log`。本调查没有删除或覆盖这些文件。

## 根因、修复与离线验证

`tools/hardware/v15_30a_gui_go_controller.cpp` 原先只在 `active_power_session` 为真时记录持续通信丢失并进入恢复；而 `--brake-only` 刻意不属于 active session。结果是拔插后永不重开原始反馈串口，启动时一次达到阈值的短暂失联也可能永久保留 transport latch。

这不是旧二进制猜测：原运行进程、磁盘二进制及相邻 `v15_30a_gui_go_controller.build.json` 一致，二进制 SHA256 为 `50a5d36f7c1aeeb03999f01df85dcb696061e3d73cf310e7c79a048c02bddeeb`，构建 manifest 绑定的当前原始源码 SHA256 为 `ffbe32889cdf8eb47a84dbd008801a520dc8b9859b86aaa71c808a984ba481b8`。

本次最小修复：

1. 持续失联在 BRAKE-only 下也进入原恢复函数；原始采集不要求不存在的 active-session 姿态许可。
2. 仍只发 BRAKE，仍需连续 5 帧健康且静止；不重选已有参考、不加载或改写持久零位，不清除非通信和热故障锁存。
3. 恢复途中再次丢失适配器时重新解析稳定 by-id，100 ms 到 2 s 有界退避，避免永久轮询失效的替换句柄。
4. BRAKE-only 恢复后始终不打开命令 UDP。带许可模式原来的姿态/相位/同步检查、旧指令作废和更高 epoch 要求保留。

已有 BRAKE-only 启动代码会建立用于诊断的内存参考；这不写电机零点或持久零位。本修复的恢复步骤不重新采集该参考。实机位置解释仍由 state 节点的保存参考与当前电源会话负责。

新增 `tools/hardware/test_v15_31d_brake_transport_recovery.cpp` 使用真实 SDK 编码器和脚本化串口，调用生产恢复函数：三域连续两次失效后均恢复；验证 FOC 发送次数为 0、参考不变、非通信/热故障保留；同时验证 active 模式不会绕过姿态/相位检查。测试不打开任何设备。

2026-09-06 07:38 UTC 验证结果：

- 生产源码以 `g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread` 严格编译通过；完整自检输出 `DRY_RUN=YES`、`SERIAL_OPENED=NO`。
- 新增 C++ 恢复测试同样严格编译通过，输出 `BRAKE_TRANSPORT_RECOVERY=PASS`、`SERIAL_OPENED=NO`。
- 既有 J2/GO-AUX 许可契约共 17 项测试通过；diff 格式检查通过。
- 编译依赖沿用 `/home/car/vendor/unitree_actuator_sdk/include`、`lib/libUnitreeMotorSDK_Linux64.so` 和系统 nlohmann/json；没有添加依赖。

临时测试目录为远端 `/tmp/go-m8010-v15-31d-brake-recovery-20260906`，包含本次源码、独立测试和候选二进制，不覆盖运行仓库或原 build manifest。候选生产二进制 SHA256 为 `431a2fbdad17b3ca6b6cc37bd944ad50cf176370e01cf96f4886013ae9511b56`，源码 SHA256 为 `7fc3ee8ce200e488798aee30e1dc361220240a2b09dc6a2c36214a516bc4a731`。这些属于离线验证，不代表实机拔插恢复或设角保持已通过。

在远端工程根目录可复现新增离线测试（测试输出到临时目录）：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -I/home/car/vendor/unitree_actuator_sdk/include tools/hardware/test_v15_31d_brake_transport_recovery.cpp /home/car/vendor/unitree_actuator_sdk/lib/libUnitreeMotorSDK_Linux64.so -Wl,-rpath,/home/car/vendor/unitree_actuator_sdk/lib -o /tmp/v15_31d_brake_recovery_test
/tmp/v15_31d_brake_recovery_test
```

## 后续现场恢复期间的再观察

主调试流程受控重启原 BRAKE-only 服务后，07:38:45 UTC 的实时 CSV 中 J2/J345 共 5 电机 `communication_ok=1`、merror=0、年龄约 6–9 ms；J1 年龄约 8 ms、merror=0，但 `communication_ok=0`，符合上述旧 transport latch 缺陷。J6 仍是 state 中的旧样本，不把它纳入实时六轴健康结论。

随后主调试流程把候选二进制用于新 `v15-31d-brake-j1` 服务（PID 2341675），沿用原 `--brake-only` 参数，不传入 zero-file，也不覆盖原二进制或 manifest。远端 `.runtime/v15_31d_recovery_20260906/hardware_after_j1.json` 和同名 YAML 已实际保存：J1 `communication_ok=true`、`fresh=true`、`age_ms=10.559139`、merror=0、31°C、`controller_mode=brake`、`controller_fault=false`；J2/J345 五颗电机同样新鲜、错误码为 0、BRAKE、无控制器故障，年龄约 5–9 ms。

因此可以记为“新二进制实机重新启动后反馈恢复通过”。没有做实机拔插复测，自动二次拔插恢复仍仅由 mock 测试证明；没有发出设角运动，不能记为当前六轴保持通过。此快照的总体 `telemetry_healthy=false` 与 `control_available_by_domain=false` 仍应保留：J6 尚未接入当前 state 会话，且这些只制动服务不具备命令端口或有效 supervisor 状态。

主调试流程另保存 `.runtime/v15_31d_recovery_20260906/j6_disabled_raw.json` 的独立 500 帧/50 秒只读采集，记录 5 帧终态 DISABLED。它不能把 state 中旧 J6 样本变为实时数据，后续需独立绑定新会话。本调查分工没有启动/停止实物驱动或发送运动命令；上述实机服务变更由主调试流程完成并保存证据。

## 新电源会话保留原几何参考

进一步追踪发现，旧竖直锚点 issuer 把本次 raw 均值映射为 J2 的逻辑 0 或 GO-AUX 的旧 initial_pose。该方式用于首次竖直初始化；在已有参考的断电恢复中再次使用，会把当前真实偏差藏进新参考。仅放宽逻辑角字段也不完整，因为 J2 的 driver/state/启动校验器要求旧逻辑 0 和 recovery hint 一致。

新增可选 `go-m8010-preserved-session-reference/1.0` 契约，保留原始 anchor 的 `session_reference_raw_rad`、`logical_position_rad`、方向和减速比完全不变。本次 fresh capture 只用于计算额外的 `startup_logical_position_rad`，并签发当前电源会话、主机 boot、worker hash 及 30 秒有效期的新许可。J2A、J2B 分别推导，不能把同步差平均或归零。

对两个现有 anchor issuer，保留原来的所有参数，并追加：

```text
--preserve-reference-file <原始有效session-anchor路径>
--expected-preserve-reference-sha256 <该原始anchor的SHA256>
```

不传这两个参数时，旧竖直初始化语义和文件格式保持不变。每次恢复均引用同一份原始几何 anchor；不建立递归恢复链。原始 source 路径和 SHA 写入新 anchor，Python issuer、state loader、启动校验器和 C++ loader 均核对 source hash、原几何字段、persistent/hints/initial_pose 父绑定，并重算 fresh raw 的角度。原始 anchor、persistent zero、校验 sidecar、hints 和 initial_pose 均不覆盖。

计算复用 state 的邻近转子圈数选择：保持原 reference，仅选择相邻 `2π` 原始编码器分支。原几何到新采样的偏差必须处在既有 2° 启动窗口内，采样两端同样检查。J2 采样同步差仍不得超过 0.5°。worker 的 50 帧启动复核以**本次新采样**为基准，允许的后续漂移仍为 0.25°、静止跨度仍为 0.20°；机械限位、温度、扭矩、碰撞、旧许可失效等限制没有放宽。

新契约的离线用例覆盖 J1 +0.256°、J3 -0.021°、J4 +0.020°、J5 -1.57°，及 J2A +0.007°/J2B +0.139°并保留 0.132°同步差；这些数值是为测试构造的输入，不能填作新的实机测量。用例同时加入一个 `2π` raw 周期，验证跨圈恢复、source/派生值篡改拒绝、2°邻近范围和 0.5°同步上限拒绝。新 anchor 的实际文件发布也已在临时目录验证，原校准文件字节不变。

独立复核补齐了历史证据验证：除原 anchor 自身 SHA 外，还读取其 `source_evidence.path`，检查实际原 raw 文件 SHA、schema、PASS、BRAKE/24V 记录、原现场确认作用域及会话绑定，并把内嵌 raw 身份、统计、worker 和采样记账与原文件对照。issuer 直接复用原完整 capture validator；state/启动/driver 消费者也独立检查原采样数、覆盖时间和静止跨度。旧 boot、旧电源会话和旧 TTL 不作为当前准入许可，只用作历史几何证据。

相关 113 项 Python 测试通过，包含 37 项现有 state mapping 和 32 项静态契约；新增 5 项测试还调用生产 C++ 两类 loader，数值一致且拒绝篡改。负例将错误原 raw hash、不可靠支撑确认、内嵌统计不一致、原捕获 FAIL、100 帧和 3 秒覆盖重新散列为新 source SHA，消费者仍拒绝。J2/AUX 的低采样/低覆盖拒绝也由另一代理独立复核。严格编译、原 BRAKE-only 两次断连 mock、自检 `SERIAL_OPENED=NO` 和 `bash -n start_arm_gui.sh` 均通过。

机端保留的 9 月 5 日两份真实原始 anchor 和原 raw 文件也经新 Python helper 与生产 C++ 离线消费者正向复核通过：J2 原 raw SHA `7879504b05750ee8612297ff63cd6456d386f62be16a7039294392b1ffd64aff`，GO-AUX 原 raw SHA `40b83d69485a02d55a65e2b9cf66c55d6d4de1e621d5b5f0fe64c7f67902ad3c`。没有打开串口，也没有把旧原始样本当作当前新会话采样。

新阶段隔离产物目录：`/tmp/go-m8010-v15-31d-preserve-reference-20260906`；最终生产二进制 SHA256 `9ca71f71d09dd91470d5e5297e957dafdfec986952867b99df4fe73e1cc40562`，对应 C++ 源码 SHA256 `62116e061dbe5141108602f7ad7f92cbbfa02e004c2abe964c03475140ef03e2`。该目录与前一阶段 brake-only 实机候选分开，未覆盖原产物。本节记录的是完成并可复现的离线恢复参考验证，实机新会话和动作/保持结果仍需主调试流程另存证据。

## UDP 原始反馈向 ROS 观察者发布

主调试流程在 `.runtime/v15_31d_hold_20260906` 已通过 GUI 请求实机 HOLD，保存的真实 state 流有 991 条观察记录（628 个独立 source 时间戳）、19.820182557 秒七电机均为 HOLD 的记录；六轴最大目标误差依次为 `[0.0242984, 0.0208283, 0.0052068, 0.0277696, 0.0104147, 0.0437139]°`，最大速度均小于 0.164°/s。这是当前位置、独立支撑条件下的短时保持观测，不能推为 ±5° 定位、完整动作路线、满载或长期热验收。随后 root 受控停止 active supervisor，原始终态记录 GO 三域 `FINAL_BRAKE=PASS`、J6 `J6_FINAL_DISABLED=PASS`。

此次独立 probe 的 J6 drive-state 始终为 null。只读 ROS 图与源码核对发现 `/whole_arm/motor_feedback_raw` 为 0 个 publisher；state 节点直接处理工作进程的 UDP 包，却从未向该 ROS 话题发布。安装的 Humble `create_subscription` 没有 `ignore_local_publications` 参数，Executor 丢弃 MessageInfo，Publisher 也未暴露 GID，不能假设可用本地发布者过滤 API。

原始 `.runtime/v15_31d_hold_20260906/evidence/current_pose_hold_probe.json` 仍为 **FAIL**：J6 原始话题无发布者导致 `drive_state` 恒为 null，probe 等待 20 秒后超时；该报告不得改写为 PASS。上述真实 state 保持观测与随后 primitive 终态证据是分别保存的事实，不能替代首轮 probe 缺失的独立闭环。raw 发布修复后的第二轮完整复测另存于独立目录，见下节。

修复保持 UDP 的原始 source/replay 检查及即时原子入库，成功后才将**原文** String 发布到既有 raw 话题（depth 100），不修改 source、sequence、时间戳或电机值。对本进程已发布原文保留最多 512 个一次性回声标记；自订阅只消耗一次精确匹配，额外重放仍由原验证器拒绝。原 ROS/mock 输入方式保留，控制反馈路径没有增加 DDS 跳转。无效 UDP 不转发，发布异常不会撤销已接收的数据。

相关 state-node 测试 12 项通过，覆盖先入库后发布、原文字节保留、自回声不重入、再次重放仍拒绝、外部 ROS 输入和无效 UDP。隔离真实 rclpy 验证使用 domain 179 和临时 UDP 端口，不接入生产 domain 30 或电机：400 个含 16 KB 附加测试数据的合成包以 400 Hz 发送，400 包入库、400 包原文送达观察者，invalid-payload 计数 0、待消费回声 0；UDP 入库 P95 1.9915 ms，观察端 P95 2.3439 ms、最大 2.7220 ms。这是消息链路测试，不能写作实机运动通过。

该修复的 `whole_arm_state_node.py` SHA256 为 `6cfcac48537164d2941db9464ff48c80b39227d56cbce15c977b9b7e8d414bdb`，验证副本位于 `/tmp/go-m8010-v15-31d-raw-observer-20260906/go_m8010_arm_hardware/whole_arm_state_node.py`。主调试流程随后在独立新目录完成第二轮实机验证，结果如下；原 HOLD 尝试及其缺失观察证据完整保留。

## 第二轮实机当前位置 HOLD：PASS

独立第二轮报告位于远端 `.runtime/v15_31d_hold_retry_20260906/evidence/current_pose_hold_probe.json`，原始状态为 **PASS**。完整报告已按字节保存到本地 `.codex-tmp/hold_evidence_20260906/attempt2_hold_probe.json`，SHA256 为 `ff98aa34c87519f4878ffbfc3759e7cf26975a438273f16dcd11705bab0fc3bd`。紧凑、可追溯的交付结果见 `hardware/v15_31d_recovery_20260906/hold_validation_summary.json`，包含完整报告和终态证据的远端/本地路径及 SHA。

按原报告的正式 HOLD 阶段独立重算：502 条观察记录对应 **500 个独立 source 样本，覆盖 10.019448704 秒**，最大 source 间隔 39.247807 ms。所有正式样本反馈新鲜、健康、七电机均 HOLD、J6 原始反馈配对为 ENABLED，冻结目标和会话身份未变，路由拒绝数为 0。演示门槛保持为误差 0.25°、速度 0.25°/s；未改严格 ±0.1° 验收契约。

| 关节 | 最大目标误差 ° | 峰峰漂移 ° | 首末漂移 ° | 最大速度 °/s |
|---|---:|---:|---:|---:|
| J1 | 0.015621 | 0.006942 | +0.003471 | 0.034689 |
| J2 | 0.009545 | 0.010414 | -0.006943 | 0.017354 |
| J3 | 0.006942 | 0.006942 | 0.000000 | 0.017356 |
| J4 | 0.024300 | 0.006942 | -0.003470 | 0.060648 |
| J5 | 0.010415 | 0.006940 | +0.001735 | 0.021684 |
| J6 | 0.043714 | 0.065571 | -0.021857 | 0.156178 |

整个 probe 的最高温度：J1 31°C、J2A 32°C、J2B 32°C、J3 31°C、J4 31°C、J5 31°C、J6 37°C；最大 J2 同步误差 0.172536°。probe 结束有 4 条配对终态观察，对应 J6 独立序号 9495、9497、9499，确认七电机制动且 J6 DISABLED。随后 root 受控停止 active supervisor 和 gravity-active，三域 GO 的 primitive `FINAL_BRAKE=PASS` 与 J6 `J6_FINAL_DISABLED=PASS` 均保存在 `evidence/worker_terminal_result.json`，该终态汇总 SHA 为 `86c1524f00f52b11c3dc8e23f7e698df1cca8b1f1ccf9030ceb6ad44eff01370`，含四份原始日志路径和 SHA。

本次模型 anchor 的 `model_absolute_joint_rad - logical_joint_reference_rad` 与固定模型偏移 `[0, 90, -14.40, 13.49, 47.94, 0]°` 并非完全相同。独立算得各轴残差约 `[+0.001736141, +0.000867531, -0.001735602, 0, +7.1e-15, -0.021856939]°`；最大差在 J6，为 0.021856939°。这些小采样差值原样保存，未修改模型、固定偏移、原始参考映射或校准来消除差异。

此次 PASS 仅覆盖**已有可靠独立支撑、当前位置、约 10 秒固定保持以及观察到的停止终态**。它没有验证 ±5° 双向设角、全范围定位、无支撑/满载悬停、长期热稳定或完整六轴零重力。第一轮原报告仍为 FAIL，SHA 为 `8b6f088476cbe008e6d0fb202de204698548c222d8321a24afe6a5afb9a9fd1b`；其独立 19.82 秒 state 观察与本次完整 probe PASS 分别记录。

## 首个真实两步动作组：单次 PASS

在独立目录 `.runtime/v15_31d_j1_demo_20260906`，现有 GUI 完成动作组保存、加载、虚拟预检与实际执行，原报告 `evidence/j1_action_group_demo.json` 为 **PASS**。本轮仅执行 **1 次往返、2 个步骤**，没有据此建立多轮重复性统计。紧凑证据见 `hardware/v15_31d_recovery_20260906/j1_action_group_summary.json`；约 11.6 MB 完整报告按原字节保存在本地 `.codex-tmp/hold_evidence_20260906/attempt3_j1_demo.json`，不将大日志加入 Git。

| 原始文件 | SHA256 |
|---|---|
| 完整报告 `evidence/j1_action_group_demo.json` | `c459d48dfb7a2c395b1416e8cc97290db7598491afbdd72aa4a80be7dcc8603e` |
| 实际保存并执行的 `run/j1_demo_action_group.json` | `4ee447a441cb4cbf8a78872654f9c401109c2906609bb0395b3380882d3e6662` |
| 执行日志 `run/action_group_20260906_094502_578991.jsonl` | `79188acd6ce3a89214b1b986a3170d964cf7caad2ac5b605fca96ff201fc185e` |
| 受控停止结果 `evidence/worker_terminal_result.json` | `0ed95e5361328de568dc4391384b3fad5a0c99f75f1a2e9c9a69173a18cff2fd` |

保存的动作组名为“J1 1°往返动作组示例”，两步速度设置均为 1°/s、到位后停留均为 0.5 秒；保存文件内容与 JSONL 中 `real_hardware` 的 `run_start.group` 完全一致，日志确认完成 2/2 步。两步夹爪动作均为 `none`，本次没有发送夹爪动作或验证夹持。

现有节点先完成 0、0.25、0.5、0.75、1.0 五级在线阶段，才进入位置动作。各级的真实样本均显示健康反馈、有效授权、温度条件满足和配对 J6 HOLD；每级都观察到完成标记，第 4 级完成后观察到位置许可。下表的“覆盖”是报告中该级观察时间范围，包含升阶交接，不能当作单独测得的稳态物理补偿收益。

| 阶段 | 系数 | 观察记录 | 独立 source 样本 | 观察覆盖秒 |
|---|---:|---:|---:|---:|
| 0 | 0.00 | 501 | 498 | 10.000066 |
| 1 | 0.25 | 612 | 580 | 12.219973 |
| 2 | 0.50 | 612 | 553 | 12.219936 |
| 3 | 0.75 | 611 | 592 | 12.200007 |
| 4 | 1.00 | 603 | 591 | 12.039996 |

去程预检 token 为 `63c414cc4f36e53f65f91893494c9bd4c0abcb0aa1d519b8135195cebbaeef38`，回程为 `6649f4b6ae66374609ed25a084b74b967e5f568078e04acb2ecbf3f2d8bc7e19`。**两次 recipe 都实际包含 J1 → J2 → J3 → J4 → J5 五个逐轴分段**，不能称为纯 J1 运动。J6 没有计划位移段，保持原目标。每个分段的 moving-joints、位移和独立 trajectory SHA 均在紧凑摘要及完整原报告中保留。

非 J1 计划修正远小于既有 0.25° 上限：J2 约 +0.007810°、J3 约 -0.001736°、J5 约 -0.006945°；J4 去程约 +0.020827°、回程约 +0.036447°。动作阶段实测非 J1 最大偏离起始值出现在 J4，约 0.045125°。

J1 名义去程目标为相对初始值 +1°，到位反馈实测偏移 **+1.1437610486°**；回到初始目标后的实测残差为 **+0.1440544101°**。它们符合本演示的去程 0.75–1.25°、回程误差不超过 0.25° 判据；本轮端点误差约 0.144°，没有达到或替代严格 ±0.1° 验收。整个流程含五级授权和终态观察共约 105.933 秒。

全过程最高温度依次为 J1 32°C、J2A 32°C、J2B 32°C、J3 31°C、J4 31°C、J5 32°C、J6 37°C；最大 J2 同步误差约 0.172539°，路由拒绝数为 0。probe 终态有 5 条配对制动观察，对应 3 个独立 J6 DISABLED 序号 23386、23390、23394。随后 root 受控停止 active supervisor 和 gravity-active，四域 primitive 终态均为 PASS，原日志路径/SHA 与末态文字随摘要保存。

本轮范围仍是用户确认的可靠独立支撑姿态、小幅往返及停止闭环，不代表无支撑满载、±5°/全范围定位、长期热性能、物理夹爪联动或六轴零重力通过。后续多轮循环若完成，应以独立运行和新证据追加；不把这次 1/1 单次结果扩写为重复性成功率。

## 正式入口三轮请求：在动作开始前受阻

正式入口会话 `.runtime/v15_31d_demo_20260906T110327-11adc7` 请求执行 3 轮，但原 `session_result.json` 最终为 **FAIL**，在只读 J6 静止采样阶段退出，**动作组执行轮数为 0**。原 J6 报告为 FAIL：500 帧覆盖 50.019081 秒，原始角度跨度 0.176623178454 rad，即约 **10.119763°**，未满足静止条件。采集发送的使能、位置/速度/力矩和电机零位写入命令数均为 0；末尾 5 帧均确认 DISABLED，通道已关闭。该跨度的具体来源尚未确认，不能据此假定是人工调整或自行运动。

清理原日志逐项核对：`boot_j1.log`、`boot_j2.log` 为 `FINAL_MODE=BRAKE / FINAL_BRAKE=PASS`；`boot_j345.log` 实际为 **`FINAL_MODE=UNCONFIRMED / FINAL_BRAKE=FAIL`**，不能写成三域终态全部确认。三域 FOC 发送数均为 0。紧凑失败证据与原文路径/SHA 保存在 `hardware/v15_31d_recovery_20260906/repeat_summary.json`，原报告和三份日志按字节保留在本地 `.codex-tmp/hold_evidence_20260906/`。没有再次循环启动；已验证的前一轮 1/1 实机 PASS 保持独立，未改写为 3/3 或重复性通过。
