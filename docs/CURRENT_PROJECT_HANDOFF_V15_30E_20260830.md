# V15.30E 机械臂控制工程交接文档

更新时间：2026-08-30 20:45（Asia/Shanghai）

远端核查时间：2026-08-30 08:38（America/New_York）

## 0. 先读结论

> **当前禁止把 Git 提交 `093bebc` 部署到真机，也禁止直接重新启动硬件 worker。**

- 用户最后明确报告：**24V 已断电，J2 已可靠支撑，机械臂未移动**。
- 当前真实温度未知；最后一份有效 J2 日志显示 J2A/J2B 为 `59–60°C`。断电时没有实时温度反馈，不能仅凭经过时间判断已冷却。
- Ubuntu 上 GUI、ROS 状态聚合器、命令路由器和 MuJoCo mirror 仍在运行，但 J1、J2、J345、J6 四个硬件 worker 均已退出。
- 反馈聚合端口 `127.0.0.1:15300` 仍由状态节点监听；控制端口 `15310/15311/15312/15313` 均无 worker 监听。
- J1/J2/J345 的终止日志均为 `FINAL_MODE=UNCONFIRMED`、`FINAL_BRAKE=FAIL`；J6 未确认 DISABLED 终态。因此只能依据现场断电和机械支撑判断当前物理安全，**不能声称软件已确认 BRAKE**。
- GitHub 新分支是中断时保存的 WIP 快照。它包含两个确定的 P0 缺陷：GUI 正常刷新可触发 `AttributeError`；J2 热锁只实现了一半，温度降到 59°C 后存在旧 epoch 自动恢复 FOC 的缺口。

## 1. 用户最终目标

1. 以用户照片中的整机竖直姿态作为本次软件初始化参考，不修改电机内部零位，不写 Flash/EEPROM。
2. 所有控制都遵循“虚拟机械臂先行”：先显示最终虚拟姿态并完成模型限位、整条路径和 2°碰撞余量检查，再由用户明确确认，最后现实机械臂沿已验证轨迹执行。
3. 删除虚拟机械臂滑条，只保留精确角度输入；现实反馈滑条可保留为只读状态显示。
4. 位置模式必须持续保持不可变目标，不能因外力位移而把反馈重采样成新目标；外力消失后应回到原设定角度。
5. GUI 必须明确显示“候选已编辑、等待硬件、正在解算、SAFE、UNSAFE、现实分段 n/m、到位保持、失败/超时”，不能让用户猜测程序是在解算还是卡住。
6. J2 在承重姿态下不能继续依靠长期饱和发热；必须保留 60°C 硬制动，并补齐热故障锁存、冷却资格和显式重新授权流程。
7. 单域故障不能被界面伪装成“仍可控”。现实新 POSITION 仍要求全六轴控制链、反馈和 HOLD 条件全部可信。

## 2. SSH 与主机信息

Windows SSH 配置文件：

`C:\Users\瞄准恶魔\.ssh\config`

有效配置：

```sshconfig
Host car
    HostName 192.168.3.112
    User car
    Port 22
    IdentityFile C:\Users\瞄准恶魔\.ssh\codex_ros2_ed25519
```

说明：配置文件中没有显式 `Port`，`ssh -G car` 展开的有效端口为默认值 `22`。密钥路径可记录，但**禁止把私钥内容、密码或 permit 内容提交到 Git**。

连接命令：

```powershell
ssh car
```

远端主机：

- hostname：`car`
- 用户/home：`car` / `/home/car`
- IPv4：`192.168.3.112`
- 系统：Ubuntu 22.04.5 LTS，x86_64
- kernel：`6.8.0-124-generic`
- 远端时区：`America/New_York`
- ROS 2：Humble

只读核查 SSH 生效配置：

```powershell
ssh -G car 2>$null | Select-String '^(hostname|user|port|identityfile) '
```

## 3. 工程、Git 与 GitHub

Windows 主工作区：

`D:\AI_GOM8010_6\go-m8010-robot-arm-v15-30a-gui`

Ubuntu 部署工作区：

`/home/car/go-m8010-robot-arm-v15-30a-gui`

GitHub：

- 仓库：<https://github.com/zhaowuc/go-m8010-robot-arm>
- WIP 分支：<https://github.com/zhaowuc/go-m8010-robot-arm/tree/snapshot/v15-30e-current-20260830>
- WIP 提交：<https://github.com/zhaowuc/go-m8010-robot-arm/commit/093bebcce2aaa98dc75171a826e7714f0024b399>
- 分支名：`snapshot/v15-30e-current-20260830`
- 提交：`093bebcce2aaa98dc75171a826e7714f0024b399`
- 父提交：`e0aabf5148874dfe0d0657faf9c6d6de452f6fd9`
- 提交标题：`wip: snapshot V15.30E GUI and hardware control work`

本地 tracked tree 在提交后干净；只剩未跟踪的 `.codex-tmp/` 和 `tmp/`，它们是运行/提取缓存，不应提交。

Ubuntu 部署目录是一个 linked worktree（`.git` 为 83 字节指针文件），当前仍停留在：

- 分支：`agent/v15-30a-ft-cn-gui-real-sim-sync`
- HEAD：`d8d0a2bd1284f4a9bef351edc0ee36e35df675c0`
- 至少 25 个 tracked 文件有现场修改

因此：

- 不要在 Ubuntu 上执行 `git reset --hard`、`git checkout -- .` 或直接 `git pull`。
- 不要用 GitHub WIP 分支整树覆盖远端。
- 修复应先在 Windows 新分支完成、测试和审查；部署时只同步已审查文件，并保留远端运行证据与 `.runtime`。

## 4. 当前物理和进程状态

用户最后确认的现场状态：

```text
24V 已断电；J2 已支撑；机械臂未移动。
```

以下 PID 只是 2026-08-30 08:38 EDT 的只读快照，接手时必须重新核对：

- supervisor：`785544`，`bash .../start_arm_gui.sh`
- ROS launch：`785737`
- whole-arm state：`785743`
- command router：`785745`
- MuJoCo mirror：`785747`
- 独立热重启 GUI：`927670`
- J1/J2/J345/J6 worker：**均不存在**

GUI `927670` 是孤儿化的独立重启进程，使用：

`/home/car/go-m8010-robot-arm-v15-30a-gui/.codex-tmp/gui_flicker_fix_JFfHAQmd/gui_params.yaml`

该参数文件记录：

```yaml
embedded_session_pose_deg: 0,90,-14.40,13.49,47.94,0
initial_pose_read_only: true
```

用户仍报告虚拟初始化姿态看起来明显弯曲；此问题尚未完成视觉复核。不得删除或覆盖现有竖直初始化数据，也不得用当前反馈重新定义零位。

控制/反馈 UDP：

| 用途 | 端口 | 当前状态 |
|---|---:|---|
| 状态节点接收 worker 反馈 | 15300 | 状态节点仍监听 |
| J1 command | 15310 | 无 worker |
| J6 command | 15311 | 无 worker |
| J2 command | 15312 | 无 worker |
| J3–J5 command | 15313 | 无 worker |

只读核查命令：

```bash
pgrep -af 'start_arm_gui|v15_30a_gui_go_controller|v15_30a_gui_j6_controller|arm_control_gui|whole_arm_state_node|command_router|mujoco_mirror'
ss -lunp | grep -E '15300|15310|15311|15312|15313'
```

24V 断开期间，不要运行 `start_arm_gui.sh`、`ros2 launch`、worker、诊断脚本或任何 UDP 发布工具。

## 5. 当前控制架构

```text
J1 / J2 / J345 / J6 worker
        │ feedback UDP :15300
        ▼
WholeArmStateNode
  ├─ /joint_states
  └─ /whole_arm/hardware_state
        ▼
MainWindow / ArmGuiNode
  ├─ /whole_arm/gui_targets
  ├─ /whole_arm/gui_mode
  ├─ /whole_arm/collision_guard_request
  └─ /whole_arm/gui_command
        ▼
CommandRouter
  ├─ 独立验证碰撞证明、source、sequence、epoch、target、mask
  └─ UDP :15310/:15311/:15312/:15313
        ▼
各硬件 worker
```

GUI 内部目标语义：

- `command_targets`：现实机械臂当前不可变的权威保持目标。
- `targets`：当前待证明/执行的单轴物理分段目标。
- `candidate_targets`：WIP 中新增的最终虚拟候选姿态。
- `requested_active_joint_mask`：当前申请主动保持的轴。
- `pending_target_joint_mask`：当前待执行证明的轴。
- `moving_joint_mask`：当前 POSITION 移动轴。

现实执行严格门保持为：六轴健康、全部 HOLD、状态年龄不超过约 250 ms、跟踪误差不超过 `0.25°`、速度不超过 `0.25°/s`。这些门不能为了“让 GUI 能动”而放宽。

当前轨迹证明设计：最终候选可一次编辑多个关节；现实执行再拆为单关节、每段最多 `30°`，每段都重新做 swept collision proof；每段到位后先进入精确 HOLD 屏障，再开始下一段。

模型会话相对限位来自冻结 3D 模型：

| 关节 | 相对范围 |
|---|---|
| J1 | -180° … +180° |
| J2 | -260° … +80° |
| J3 | -155.60° … +184.40° |
| J4 | -129.49° … +145.51° |
| J5 | -118.54° … +103.26° |
| J6 | -180° … +180° |

机械范围端点不静态缩小；路径碰撞检查单独应用 2°安全余量。

## 6. 2026-08-30 J2 过热/失力事件

GUI 目标约为：

```text
J2 target = -0.352906 rad ≈ -20.22°
```

事件中：

- J2 实际位置先到约 `-27.97°`，worker 退出后状态最终约为 `-34.60°`。
- 目标与实际最大已确认偏差约 `14.38°`。
- J2A/J2B 多次出现 `J2_TORQUE_FEEDBACK_SATURATED_KEEPING_FOC`。
- 反馈扭矩长期在约 `±3.2 Nm/转子`，峰值约 `4.07 Nm/转子`。
- J2A/J2B 温度到 `59–60°C`。
- 随后出现：

```text
J2_INVALID_FEEDBACK_BEGIN ... first_temp=60 ...
OWNED_DOMAIN_BRAKE_BEGIN bus=j2 reason=MOTOR_TEMPERATURE
DOMAIN_FAULT ...
TERMINAL_PATH=EXCEPTION
FINAL_MODE=UNCONFIRMED
FINAL_BRAKE=FAIL
REASON=J2_POWER_CONTINUITY_LOST
```

结论：发热不是“GUI 无用警告”造成的，而是 J2 在承重误差下长期饱和工作的真实热事件。现有系统没有经过验证的在线重力前馈；有界共同积分在误差大于 10°时不继续学习，recovery 分支还会逐周期清积分，因此不能依靠盲目提高 Kp、扭矩或温度门槛解决。

60°C 是当前项目硬保护阈值，不是已查到的厂家连续额定温度。厂家资料未给出可据此提高阈值的内部连续温度规格，所以：

- 不得删除、延时或提高 60°C 硬制动。
- 不得把“受到多大力量都绝不偏移”当成物理可保证合同；超过电机连续扭矩/散热能力时只能拒绝动作、减小负载或增加机械配重/支撑。
- 55°C 后只能限制新的运动能量并保留已验证的承重 HOLD；不能简单削减静态保持扭矩导致下坠。

## 7. 当前 Git WIP 已完成与未完成

### 7.1 已加入但尚未交付的框架

- 虚拟面板已删除虚拟滑条，只保留角度显示和 `QDoubleSpinBox` 精确输入。
- 新增 `candidate_targets`、候选 mask、预演状态、候选 SHA-256/session/state-instance 审批字段。
- 新增常驻工作流标签与进度条。
- 按钮已改名为“回到初始化姿态（仅虚拟预演）”和“确认执行现实轨迹”。
- 候选编辑可撤销旧审批，SAFE 结果可绑定候选 hash。
- 热反馈已初步拆成 `continuity_valid` 与 `actuation_safe`，纯 60°C 不再直接计入通信连续性丢失。
- 状态层新增 `worker_supervisor_status.json` 读取/校验框架和 `control_available_by_domain` 字段。

### 7.2 P0：GUI 当前必修问题

1. `VirtualWidgets` 已删除 `slider`，但 `_show_targets_on_virtual()` 仍访问 `widgets.slider`；正常反馈刷新即可 `AttributeError`。
2. `_return_initial_pose()` 仍要求全轴健康，并在加载目标后直接调用 `_execute_target()`；与“仅虚拟预演”相反，且绕过了按钮禁用状态。
3. `_execute_target()` 未在函数内部强制 `_preview_approval_matches_candidate()`；不能只依赖按钮是否 enabled。
4. `_tick()` 与 `_publish_command()` 仍把物理分段 `targets` 发送给 MuJoCo/`gui_targets`，执行时虚拟模型仍会从最终候选跳回当前分段。
5. 首帧反馈、HOLD 捕获、方向切换、停止、会话变化等路径尚未完整同步或明确保留 `candidate_targets`。
6. `waiting_hardware` 后硬件恢复不会自动重新发起预演。
7. 现实分段编号、POSITION、HOLD 屏障、完成、中止和超时尚未统一接入工作流进度。
8. 当前 `preview/pose_preview` 仍依赖新鲜全轴 HOLD 快照；若要真正支持断电时纯模型姿态检查，需要扩展 MuJoCo guard 合同，不能把无现实起点证明的点姿态检查冒充真机执行许可。

### 7.3 P0：热状态机当前必修问题

提交 `093bebc` 只完成了反馈有效性拆分：

- `continuity_valid`：transport、identity、返回 mode、有限 q/dq/tau。
- `actuation_safe`：再叠加 `merror==0` 与 `0 <= temp < 60`。
- terminal BRAKE 确认改为只依赖 continuity，因此过热时仍有机会确认 BRAKE。

但以下成员只有声明，没有接入运行逻辑或测试：

```text
kTemperatureWarning = 55
kThermalCooldownConsecutiveFrames = 500
ThermalInterlockState
thermal_fault_latched
thermal_warning_reported
```

当前严重缺口：60°C 已不再进入通用永久 fault latch，仅靠“最新温度仍 ≥60”保持 BRAKE。如果之后出现有效 59°C 反馈，旧 active command/旧 `activation_epoch` 可能自动恢复 FOC。

必须实现：

1. 任一 J2 电机首帧 `temp>=60`，立即锁存整个 J2 域、同周期双电机 BRAKE，并隔离 trip epoch。
2. worker 保持 BRAKE 轮询和热状态发布，不因纯过热退出。
3. 双电机均 `<55°C`、continuity 有效、`merror==0`、返回 BRAKE，连续 500 帧后只能进入 `rearm_ready`。
4. 冷却不得自动恢复 FOC；必须先收到明确释放/BRAKE，再收到高于 trip/minimum/rejected epoch 的新主动命令。
5. 清锁当周期仍全 BRAKE，下一周期才允许重新资格化 FOC。
6. 发布 `thermal_fault`、`thermal_rearm_ready` 和最低 rearm epoch。
7. 真实连续性丢失仍走终止路径，并要求新电源会话、重新采集和新 permit。

### 7.4 P0：worker 存活状态链未闭合

`state_model.py` 和 `whole_arm_state_node.py` 已加入 supervisor 状态文件读取器，但 `start_arm_gui.sh` 尚未生成/原子更新该状态文件。若直接部署这部分代码，上层会 fail-closed 认为全部域不可用；安全但不可操作。

还需完成：

- supervisor 周期性写入 instance、PID、每域 worker PID、进程存活、UDP owner、exit reason 和 monotonic timestamp。
- 路由状态回显实际接受的 source/sequence/epoch/mask；不能把“已执行 sendto”当 worker ACK。
- GUI 将当前命令身份与 router/worker 状态对账，并明确显示具体域不可用原因。
- worker dead 必须锁定现实执行；遥测 `fresh/hold` 仅代表短时最后样本，不能证明命令接收器存活。

## 8. 测试状态

提交前已完成：

```text
Python compileall: PASS
git diff --check: PASS
凭据格式扫描: PASS
```

对当前 GUI 逻辑测试的只读审计结果：

```text
83 passed, 12 failed in 30.32s
```

12 个失败主要来自测试替身/断言尚未适配 `QProgressBar`、删除虚拟滑条和新的 candidate/workflow 字段。当前没有完整覆盖新“虚拟先行”合同的测试。

至少补充：

- 删除滑条后的显示刷新不崩溃。
- 断电/硬件不可用时仍可编辑和渲染纯虚拟候选。
- 初始化按钮绝不调用 `_execute_target()` 或发布 POSITION。
- SAFE 必须精确匹配候选 hash/session/state；编辑、超时、会话变化必须作废旧 SAFE。
- 直接调用 `_execute_target()` 也不能绕过 SAFE。
- 硬件恢复后自动重新检查等待中的候选。
- 全分段过程中 MuJoCo 始终显示最终候选，物理分段单独展示。
- 60→59°C 不自动恢复；第 500 个冷却帧只进入 ready；缺 release/新 epoch 仍不可 FOC。
- 热锁期间继续发布 communication/fresh/BRAKE/thermal 状态。
- 真正 5 帧通信/身份/mode/数值失败仍走 terminal path。

在上述测试全绿、Linux 构建通过和代码审查完成前，不得部署。

## 9. 日志与证据

本次运行目录：

`/home/car/go-m8010-robot-arm-v15-30a-gui/logs/arm_gui/20260830T105341Z`

关键文件：

- `j1_controller.log`
- `j2_controller.log`
- `j345_controller.log`
- `j6_controller.log`
- `arm_gui_20260830_065345.csv`
- `arm_gui_20260830_074234.csv`
- `whole_arm_state_capture.csv`
- `joint_states_capture.csv`
- `mujoco_mirror_capture.csv`
- `mirror_validation.json`

汇总启动日志：

`/home/car/go-m8010-robot-arm-v15-30a-gui/.codex-tmp/position_mode_noise_fix_20260830/full_recovery.log`

独立 GUI 日志：

`/home/car/go-m8010-robot-arm-v15-30a-gui/.codex-tmp/gui_flicker_fix_JFfHAQmd/gui_restart.log`

ROS launch 日志：

`/home/car/.ros/log/2026-08-30-06-53-43-627288-car-785737/launch.log`

日志 CSV 仍可能因 ROS 进程存活而增长；持续增长不等于 worker 或控制链存活。

## 10. 建议接手顺序

### A. 保持断电，先完成离线修复

1. 从 `snapshot/v15-30e-current-20260830` 新建聚焦修复分支，不直接把 WIP 当成 release。
2. 修复 GUI 的 `widgets.slider` 崩溃和所有 candidate/approval/初始化/执行状态机缺口。
3. 补齐 J2 thermal latch/rearm 状态机；保留 60°C 同周期硬制动，禁止 59°C 自动恢复。
4. 补齐 supervisor worker 状态文件写入、路由/GUI 对账和域级不可用原因。
5. 完成单元测试、无硬件事件回放和 Linux 编译。
6. 仅在确认没有运行中的 worker 后执行 `./start_arm_gui.sh --prebuild-only`；该路径只允许构建/静态检查，不能发送硬件命令。
7. 审查构建产物哈希，并据此重新生成后续 permit；旧 permit 不得绑定新二进制。

### B. 只有用户再次明确现场状态后才进入硬件恢复

一次确认应包含：24V 状态、J2 支撑、无桌面/爪子/线束外力、未顶限位、温度已具备可信测量条件。

1. 正常停止旧 supervisor/孤儿 GUI，核实控制端口完全释放。
2. 由于所有 worker 已退出且 terminal BRAKE 未确认，建立新电源会话；不得复用旧 permit 或旧 epoch。
3. 先进行全域 BRAKE-only 启动和反馈/温度确认，不发送运动目标。
4. J2 必须满足热锁冷却与显式 rearm 合同；不满足时保持 BRAKE 和机械支撑。
5. 先验证虚拟候选、碰撞证明和 GUI 可观察状态，再进行极小、单向、低速 POSITION 验证。
6. 每次异常只停止，不自动重试；但不要重复要求用户提供同一电源周期的冗长口令，软件应把一次授权绑定到可机读 session/epoch。

## 11. 禁止事项

- 不得直接部署或运行 `093bebc`。
- 不得删除/提高 60°C 热保护。
- 不得把 `fresh/hold`、GUI 显示或 router `sendto` 当 worker ACK。
- 不得在无 matching SAFE proof 时执行现实 POSITION。
- 不得让“回到初始化姿态”按钮直接驱动真机。
- 不得把外力位移重采样为新的 HOLD 目标。
- 不得删除、覆盖或重新定义用户的竖直初始化位置。
- 不得写电机内部零位、Flash/EEPROM、其他 RID 或电机 ID。
- J6 RID10 仅允许在 DISABLED 已确认时做易失 POS_VEL 切换；不得持久写入。
- 不得复用已经消费或与旧 worker 哈希绑定的 permit。
- 不得对远端脏 worktree 执行破坏性 Git 命令。

## 12. 交接结论

当前机械臂依靠现场断电和机械支撑保持安全，软件控制链不完整。最新 GitHub 分支成功保存了全部当前工程工作，但它是 WIP 证据快照，不是可部署版本。下一位工程师应先完成 GUI 虚拟先行状态机、J2 热锁/rearm、worker 存活/ACK 三条链并使测试全绿；之后才可在新电源会话中从 BRAKE-only 恢复。任何“直接加大力矩、删除温度警告、绕过全轴 HOLD/碰撞证明”的处理都会重复当前失力与发热风险。
