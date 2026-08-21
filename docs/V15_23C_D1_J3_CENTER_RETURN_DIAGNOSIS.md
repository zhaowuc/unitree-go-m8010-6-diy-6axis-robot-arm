# V15.23C-D1 — J3 低负载回中精度离线根因诊断

## 结论

本任务仅分析既有源码和冻结 CSV，未上电、未通信、未产生硬件运动，也未修改任何控制参数或 authority。

两次失败都不是 center command 写错，而是 **command 正确、`dq_cmd=0`，反馈在约 0.84–0.90° 的非零误差处平台化**。没有发现 reference drift、旧 session reference 复用、相位切换错误、命令不连续或验收逻辑错误。最窄且有证据支持的主分类是：

`PRIMARY = POST_TRAJECTORY_STATIC_OFFSET_VARIABILITY`

`SECONDARY CANDIDATE = CONTROL_TRACKING_PRECISION_VARIABILITY`

精确物理机制仍不能仅凭两次运行确定。机械迟滞/静摩擦和轨迹到达动态可以作为候选解释，但均未被证明；固定方向 backlash 明确不受当前证据支持。

## 1. Input evidence identity

Authoritative source：

- source branch：`agent/v15-23c-r1-j3-low-load-repeat`
- source HEAD：`e4e2ade76de8cb78e8b5934de684805ac9f50ddc`
- source tree：`5c81d7b2f1d57ddd43b74d88713ada7cb04b6156`
- diagnosis branch：`agent/v15-23c-d1-j3-center-return-diagnosis`

主要输入：

| Evidence | SHA256 |
|---|---|
| `hardware/v15_23c/j3_low_load_bidirectional_5deg_20260820_225219.csv` | `1092689e67cd9efe7251a80615c6e62a5326719877c8a385f8c55d0f7603f9be` |
| `hardware/v15_23c/j3_low_load_commissioning_inventory.json` | `667d672b0608e88686c2c4734532e3dc4b382e85d85b61553e67eac3d0362568` |
| `docs/V15_23C_J3_LOW_LOAD_COMMISSIONING.md` | `8b43e0dc6c33538122b9dc3194f3d6b9234432bb24a8bb5608da9f9a931dd9db` |
| `hardware/v15_23c_r1/j3_repeat_bidirectional_5deg_20260820_231013.csv` | `cdedb58c6528eb0fd23a8575af3c6d13e6d9b765a07abb2bf299e802bcaabe11` |
| `hardware/v15_23c_r1/j3_repeat_inventory.json` | `952b6af7699b9e7e2fc32d2baec3c442268d279ad7e44331354d1d8fda2529ec` |
| `docs/V15_23C_R1_J3_LOW_LOAD_REPEAT.md` | `f43a45ad35a1d615a3c72493bd7a247e92437b1363434b0b7e56b459951cb144` |

原始 V15.23C / R1 CSV 未修改。

## 2. Controller source identity

实际 runner 为 `tools/hardware/v15_23c_j3_low_load.cpp`。重新计算 SHA256：

`150f1b8b384aab96c27df2a23827df32e1e8513946af43afceb90522986624b2`

与合同给出的 authority 完全一致，因此：

`SOFTWARE_PROTOCOL_EQUIVALENCE = PASS`

静态代码审计确认：

- 100 Hz；gear ratio `6.3299999237060547`；`raw_to_ros_sign=+1`。
- `Kp=0.50`、`Kd=0.05` 均为 SDK literal，`Tff=0`。
- 轨迹为 10°/s、30°/s²。
- `q_cmd_motor = session_reference_motor + gear_ratio × q_output`。
- `dq_cmd_motor = gear_ratio × dq_output`；所有 HOLD 显式命令 `dq=0`。
- `MotorCmd` 赋值和 SDK `modify_data` 编码路径与 J1/J4/J5 authority 一致。

## 3. Runner 与 phase timing 重建

每轮先采集 50 个新 session encoder sample 并取 median 作为该轮 reference，然后执行：

`SECOND_HOLD → +5_PROFILE → +5_HOLD → FIRST_CENTER_PROFILE → FIRST_CENTER_HOLD → -5_PROFILE → -5_HOLD → FINAL_CENTER_PROFILE → FINAL_CENTER_HOLD → BRAKE`

两轮 phase sample count 完全相同：capture 50、SECOND_HOLD 200、四段 profile 各 85、三个 0.4 s HOLD 各 40、FINAL_CENTER_HOLD 50、FINAL_BRAKE 5。

| Run | HOLD | Nominal (s) | Sample span (s) | 到下一 phase (s) |
|---|---|---:|---:|---:|
| V15.23C | +5 | 0.4 | 0.390068 | 0.400002 |
| V15.23C | first center | 0.4 | 0.390022 | 0.400000 |
| V15.23C | -5 | 0.4 | 0.390075 | 0.400132 |
| V15.23C | final center | 0.5 | 0.489896 | 0.491526 |
| R1 | +5 | 0.4 | 0.389801 | 0.399801 |
| R1 | first center | 0.4 | 0.390003 | 0.400000 |
| R1 | -5 | 0.4 | 0.389858 | 0.399991 |
| R1 | final center | 0.5 | 0.489987 | 0.491666 |

`sample span` 比 nominal 少一个约 10 ms 的采样间隔是离散采样定义所致；从第一帧到下一 phase 的边界时间与 nominal 一致，没有能解释 FAIL 的明显 timing 缺失。

## 4. Command target 与 phase transition

V15.23C 的新 session reference 为 `2.8054656982421875 motor rad`；R1 为 `2.788400173187256 motor rad`。每轮 FIRST/FINAL center 的最终 `q_cmd_motor` 都严格等于本轮 reference，换算后的最大 command-center error 为 `0.0°`。

未发现：

- old-session reuse；
- reference mutation / incremental accumulation；
- floating-point target drift；
- unwrap branch drift；
- sign application error。

全部 center HOLD 的 `max(abs(dq_cmd_motor_rad_s)) = 0.0`。所有 MOVE↔HOLD 边界的最大 `q_cmd` 跳变为 `0.0°`，最大 `dq_cmd` 跳变为 `0.0 motor rad/s`，也没有 one-tick wrong target、late update 或 off-by-one。

四段 profile 均为 85 samples，命令峰值 `|dq|=1.1047934031966051 motor rad/s`，到下一 phase 约 0.85 s；+5→center 与 -5→center 的命令生成、加减速和 endpoint 在软件侧对称。

实际 center-return 响应如下。arrival signed error 的符号在 +5→center 时为正、-5→center 时为负，所有 center HOLD 都没有跨越 target，故未观察到 overshoot；表中的 arrival magnitude 是尚未走完的 undershoot。

| Run / return | Profile actual qdot fast peak | Profile actual qdot slow peak | Arrival signed error | Overshoot | Settling |
|---|---:|---:|---:|---|---|
| V15.23C +5→center | 11.561622°/s | 10.997954°/s | +0.841766° | NO | plateau |
| V15.23C -5→center | 14.228538°/s | 12.055197°/s | -0.388775° | NO | continued toward center inside threshold |
| R1 +5→center | 12.179152°/s | 11.403541°/s | +0.565805° | NO | continued toward center inside threshold |
| R1 -5→center | 13.926788°/s | 11.765009°/s | -0.897306° | NO | plateau |

实际 peak velocity 随方向有差异，但哪一方向失败在两轮之间互换，故没有稳定的 `DIRECTIONAL_ASYMMETRY`。

## 5. Error-vs-time 与 threshold

门限为 `0.75°`。

| Run / center | Frozen result | Acceptance median abs error | Final signed error | Last-100ms signed slope | T_stable_100ms | Behavior |
|---|---|---:|---:|---:|---|---|
| V15.23C first | FAIL | 0.843501° | +0.843501° | +0.003145°/s | NOT_REACHED | PLATEAU |
| V15.23C final | PASS | 0.167485° | -0.150998° | +0.091505°/s | 0.000 s | within threshold |
| R1 first | PASS | 0.527623° | +0.465141° | -0.598524°/s | 0.000 s | within threshold |
| R1 final | FAIL | 0.895571° | -0.893835° | +0.006284°/s | NOT_REACHED | PLATEAU |

V15.23C failed first center 在最后 100 ms 的斜率只有 `+0.003145°/s`，且最后 200 ms 斜率为 `-0.002613°/s`；其 hold 内标准差只有 `0.001157°`。R1 failed final center 的最后 100/200 ms 斜率分别只有 `+0.006284°/s` 和 `+0.002087°/s`，标准差 `0.001389°`。这些量级相对于 0.84–0.90° 残差可忽略，因此两者都应判为非零偏差平台，不是仍有实际意义的持续收敛，也没有形成振荡。

原 acceptance 结论保持不变；没有用事后延长 HOLD 改判 PASS。

完整逐样本时间序列、所有 HOLD 的 0/50/100/... ms 最近样本及其实际时间差、窗口统计、qdot、SDK dq、tau、command/feedback 已写入派生 CSV。

## 6. 两轮机械响应与 tau 对比

V15.23C 在 +5→center 后失败、-5→center 后通过；R1 则在 +5→center 后通过、-5→center 后失败。故：

`FIXED_DIRECTIONAL_FAILURE_SUPPORTED = NO`

失败期间存在非零 tau feedback：

- V15.23C first center：signed error 约 `+0.8435°`，tau 范围 `-0.05078125..-0.03515625`，median `-0.04296875`。
- R1 final center：signed error 约 `-0.8956°`，tau 范围 `+0.04296875..+0.06640625`，median `+0.0546875`。

两者 tau 符号都与推动反馈回中心的方向一致，说明非零误差存在时控制输出并非消失；但本报告不把 protocol tau 未经 authority 换算为 Nm。

## 7. Encoder quantization

真实 feedback 中最常见的非零 encoder step 为：

- `0.000191688538 motor rad`
- `0.0017350622971260426 output deg`

0.75° 门限约等于 432.26 个该步长。因此 quantization 比失败残差小约三个数量级，不能解释 0.84–0.90° FAIL。

`ENCODER_QUANTIZATION_EXPLAINS_FAILURE = NO`

## 8. Static HOLD 直接对比

| Static current-position HOLD | Median abs error | Max abs error | Slow qdot peak |
|---|---:|---:|---:|
| V15.23C HOLD1 | 0.008677° | 0.012150° | 0.031600°/s |
| V15.23C HOLD2 | 0.006942° | 0.010413° | 0.033135°/s |
| R1 preflight HOLD | 0.001735° | 0.003470° | 0.039459°/s |

静态 current-position HOLD 是在电机已静止的位置捕获当前 encoder 并把同一点设为 target，因此几乎不需要消除先前运动留下的误差。post-trajectory center HOLD 则必须回到原 session reference；失败案例中 command 正确，但 feedback 在轨迹到达后保留约 0.8–0.9° residual，并在当前控制条件下平台化。这解释了“静态 HOLD 很准”和“回中重复性不稳定”并不矛盾。

## 9. J4/J5 历史对照

只读取既有 evidence：

- J4 V15.22B：first center median residual `0.916398°`，final center `0.029505°`；其 first center 也呈近似平台，和 J3 失败残差量级相似。
- J5 V15.22A：first/final center 分别为 `0.232572°` / `0.194383°`，未出现同量级残差。

因此 `J4/J5_SIMILAR_CENTER_RESIDUAL_OBSERVED = YES`（来自 J4），说明该现象并非 J3 独有；但 J5 没有同样表现，证据也不足以证明所有 GO 电机共享一个物理根因。

## 10. Proven、candidate 与 unsupported

PROVEN：

- 两轮 controller implementation 和 protocol parameters 相同。
- 四个 center target 都正确，center HOLD 的 `dq_cmd=0`。
- 未发现 reference drift、phase-transition bug 或 acceptance implementation bug。
- 两个 failed center 都是在正确命令下反馈未跟到门限，并在非零误差处平台化。
- 编码器量化不能解释失败。
- 失败方向在 repeat 中互换，不支持固定方向失败。
- static current-position HOLD 精度显著好于 post-trajectory center-return repeatability。

CANDIDATE：

- 机械迟滞或静摩擦可能参与形成轨迹后的静态残差。
- 轨迹到达动态可能影响低增益闭环是否跨入 0.75° 门限。

NOT PROVEN / NOT SUPPORTED：

- gearbox backlash 是根因；
- static friction 是根因；
- Kp/Kd 不足是根因；
- 固定方向 backlash；
- encoder artifact；
- software command/reference/phase bug。

## 11. Root-cause classification

`PRIMARY ROOT-CAUSE CATEGORY = POST_TRAJECTORY_STATIC_OFFSET_VARIABILITY`

`SECONDARY CANDIDATE = CONTROL_TRACKING_PRECISION_VARIABILITY`

这是一项有证据支持的行为层根因分类；精确的机械/控制物理机制仍为 `INCONCLUSIVE`，不作过度归因。

## 12. NEXT_SINGLE_EXPERIMENT

唯一建议：

`LOW_SPEED_SINGLE_VARIABLE_CENTER_RETURN_DIAGNOSTIC`

只降低 trajectory velocity；保持 Kp、Kd、Tff、acceleration、HOLD duration、0.75° threshold、动作路线和日志格式不变。其目的仅是隔离“轨迹到达动态”对平台残差的影响。本任务没有执行该实验。

## 13. Authority freeze

诊断没有 authority 升级：

- `J3_LOW_LOAD_HOLD = PASS`
- `J3_LOW_LOAD_SIGN = PASS`
- `J3_RAW_TO_ROS_SIGN = +1`
- `J3_LOW_LOAD_BIDIRECTIONAL_5DEG = FAIL`
- `J3_LOW_LOAD_BIDIRECTIONAL_REPEATABILITY = INCONSISTENT`
- `J3_LOW_LOAD_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_INSTALLED_LOAD_LOCAL_MOTION = NOT_TESTED`
- `J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED_FOR_FINAL_ASSEMBLY`
- `J2_ACTIVE_WORK = NO`
- `J2_ACTIVE_MOTION_AUTHORITY = NOT_GRANTED`

`HARDWARE_MOTION_USED = NO`
