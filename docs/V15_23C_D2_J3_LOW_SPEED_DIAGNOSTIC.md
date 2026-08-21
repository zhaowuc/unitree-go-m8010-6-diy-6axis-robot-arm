# V15.23C-D2 — J3 低速单变量回中诊断

## 结论

本任务按合同只执行了一次低速路线。唯一改变的控制变量是 trajectory max velocity：`10°/s → 5°/s`。Kp、Kd、Tff、acceleration、command rate、HOLD durations、route、acceptance thresholds、gear ratio、sign 和低负载机械配置全部保持不变。

结果：

- `LOW_SPEED_SINGLE_RUN_RESULT = FAIL`
- `LOW_SPEED_EFFECT = NOT_SUFFICIENT_IN_THIS_RUN`
- `0.8~0.9_DEG_PLATEAU_REPRODUCED = YES`

FIRST CENTER 的 median absolute error 为 `0.9025107794723276° > 0.75°`，且最后 100 ms slope 只有 `-0.0000132735°/s`，属于明确的非零 residual plateau。FINAL CENTER 为 `0.38877480385046415°`，通过。

降低 velocity 确实降低了整体实际速度峰值，但没有消除回中平台。因此本轮只证明 **5°/s 在本次运行中不足以避免该问题**；不能写成 velocity 完全无影响，也不能证明精确根因。

## 1. Source 与单变量审计

- authoritative source branch：`agent/v15-23c-d1-j3-center-return-diagnosis`
- source HEAD：`791a6a14998b0bf1fb8e2c6379851a97b265673b`
- D2 branch：`agent/v15-23c-d2-j3-low-speed-diagnostic`
- base source：`tools/hardware/v15_23c_j3_low_load.cpp`
- base SHA256：`150f1b8b384aab96c27df2a23827df32e1e8513946af43afceb90522986624b2`
- D2 source：`tools/hardware/v15_23c_d2_j3_low_speed.cpp`
- D2 SHA256：`7ad4ac5ceb39e02a920a524b794f43f785d46b81fa4ab72c90faddfbaa4d7eb6`
- compile/self-test：`PASS`
- `ONLY_VELOCITY_CHANGED = YES`

源代码逐行审计确认，唯一控制语义差异为 `kVmax: 10°/s → 5°/s`。其余差异仅是 D2 operator-gate token、evidence directory、baseline phase label 和 self-test/error label，不改变控制逻辑。

保持不变：

| Parameter | D1 / historical | D2 |
|---|---:|---:|
| Kp | 0.50 SDK literal | 0.50 SDK literal |
| Kd | 0.05 SDK literal | 0.05 SDK literal |
| Tff | 0 | 0 |
| Acceleration | 30°/s² | 30°/s² |
| Command rate | 100 Hz | 100 Hz |
| Gear ratio | 6.3299999237060547 | 6.3299999237060547 |
| raw_to_ros_sign | +1 | +1 |
| Preflight HOLD | 2.0 s | 2.0 s |
| Endpoint / first-center HOLD | 0.4 s | 0.4 s |
| Final-center HOLD | 0.5 s | 0.5 s |
| Center threshold | 0.75° | 0.75° |

## 2. Trajectory math audit

对每段 5° displacement，在 `vmax=5°/s`、`acceleration=30°/s²` 下：

- acceleration phase：`0.16666666666666666 s`
- cruise phase：`0.8333333333333334 s`
- deceleration phase：`0.16666666666666666 s`
- profile duration：`1.1666666666666667 s`
- 100 Hz runner samples：`118`

四段实际 profile 均为 118 samples；从 profile 首帧到下一 phase 为 `1.179831–1.180175 s`，符合离散 100 Hz 调度。没有为保持旧 duration 而修改 acceleration。

## 3. Mechanical、topology 与 safety

Operator 已确认：

- 上臂仍拆除，低负载配置未改变；
- J3 flange reference 和 J2 reference marks 完整；
- J3 ID3 connected；
- J4/J5 physically disconnected；
- ±10° clearance、emergency BRAKE、operator clear 均就绪；
- 测试后观察 SAFE：无碰撞、异常振动、明显异响、失控或线束拉扯；
- 测试后 GO 24V OFF。

J2 没有 discovery、FOC、dual enable、ID/zero write 或继续拆卸。

## 4. BRAKE baseline

BRAKE-only baseline：

- valid/total：`100/100`
- median：`2.676802635192871 rad`
- mean：`2.67690434217453 rad`
- std：`0.00013416469153143166 rad`
- min/max：`2.6766109466552734 / 2.6771862506866455 rad`
- temperature：`33°C`
- merror / invalid：`0 / 0`
- final BRAKE：`5/5 PASS`

Active route 又重新读取 50 帧并建立本轮 session reference `2.676994562149048 rad`，没有使用历史 raw target。

## 5. Preflight 与唯一 active route

Preflight current-position HOLD：

- duration：`2.0 s`，200 samples
- max drift：`0.008679627542778135°`
- final median error：`0.005207344918131188°`
- result：`PASS`

之后只执行一次：

`0 → +5° → 0 → -5° → 0`

没有额外 HOLD、retry、第二次 route 或事后延长 acceptance window。

## 6. D2 endpoint 与 center 结果

| Point | Actual / signed final | Acceptance error | Last-100ms slope | Behavior | Result |
|---|---:|---:|---:|---|---|
| +5 endpoint | `4.732982339953°` / `-0.268752722341°` | `0.267017660047°` | `-0.006321423562°/s` | stable endpoint | PASS |
| FIRST CENTER | final `+0.900775717179°` | `0.902510779472°` | `-0.000013273529°/s` | PLATEAU | FAIL |
| -5 endpoint | `-4.293877781365°` / `+0.702652094048°` | `0.706122218635°` | `-0.018896603990°/s` | stable endpoint | PASS |
| FINAL CENTER | final `-0.387039741557°` | `0.388774803850°` | `+0.018899777576°/s` | WITHIN_THRESHOLD | PASS |

FIRST CENTER：

- last-200ms slope：`-0.000523535591°/s`
- hold std：`0.000909367796°`
- `T_stable_100ms = NOT_REACHED`

FINAL CENTER：

- last-200ms slope：`+0.009123032500°/s`
- hold std：`0.001148815653°`
- `T_stable_100ms = 0.0 s`

FIRST CENTER 在整个 acceptance HOLD 中停留于约 `+0.90°`，不是仍有实际意义的收敛，也不是振荡。

## 7. 10°/s 与 5°/s 对比

| Metric | V15.23C 10°/s | R1 10°/s | D2 5°/s |
|---|---:|---:|---:|
| +5 error | 0.291315° | 0.173294° | 0.267018° |
| First center error | 0.843501° FAIL | 0.527623° PASS | 0.902511° FAIL |
| -5 error | 0.483965° | 0.605458° | 0.706122° |
| Final center error | 0.167485° PASS | 0.895571° FAIL | 0.388775° PASS |
| First center behavior | PLATEAU | WITHIN_THRESHOLD | PLATEAU |
| Final center behavior | WITHIN_THRESHOLD | PLATEAU | WITHIN_THRESHOLD |
| Max qdot fast | 15.864273°/s | 15.506398°/s | 11.764435°/s |
| Max qdot slow | 13.097650°/s | 12.812877°/s | 9.896335°/s |
| SDK dq max | 2.012588 rad/s | 2.208937 rad/s | 1.718063 rad/s |
| Tau range | -0.066406..0.058594 | -0.066406..0.066406 | -0.070312..0.066406 |
| Temperature | 34°C | 33°C | 33°C |
| merror / invalid | 0 / 0 | 0 / 0 | 0 / 0 |
| Operator observation | SAFE | SAFE | SAFE |

D2 的 measured qdot peaks 比两轮 10°/s evidence 低，但 FIRST CENTER 仍复现同量级平台。这不支持“只要降低到 5°/s 就能解决问题”，也不能从单次运行推导 velocity 完全没有贡献。

## 8. 合同问题回答

1. **5°/s 是否通过全部 center 标准？** 否。FIRST CENTER FAIL；FINAL CENTER PASS。
2. **是否仍出现 0.8–0.9° 平台残差？** 是。FIRST CENTER `0.902510779472°`，并明确平台化。
3. **实际 response 相比 10°/s 有何变化？** qdot fast/slow 峰值下降，但 endpoint/center residual 没有一致改善；FIRST CENTER 仍失败。
4. **是否支持 trajectory arrival dynamics 参与问题？** `INCONCLUSIVE`。本轮证明降低 velocity 单独不足，但单次 FAIL 不能证明 arrival dynamics 完全无影响。
5. **下一步是否值得直接做 5°/s exact repeat？** 否。合同规定低速单次 FAIL 后先根据 residual 重新诊断，而不是直接 repeat。

## 9. Classification 与下一单任务

- `LOW_SPEED_SINGLE_RUN_RESULT = FAIL`
- `LOW_SPEED_EFFECT = NOT_SUFFICIENT_IN_THIS_RUN`
- `0.8~0.9_DEG_PLATEAU_REPRODUCED = YES`
- `TRAJECTORY_ARRIVAL_DYNAMICS_CONTRIBUTION = INCONCLUSIVE`

建议的唯一下一任务：

`J3_KP_SINGLE_VARIABLE_CENTER_RETURN_DIAGNOSTIC_DESIGN`

该任务仅用于设计和审批下一项单变量诊断；本任务没有修改 Kp，也没有授权或执行更多运动。

## 10. Authority freeze

- `J3_LOW_LOAD_HOLD = PASS`
- `J3_LOW_LOAD_SIGN = PASS`
- `J3_RAW_TO_ROS_SIGN = +1`
- `J3_LOW_LOAD_BIDIRECTIONAL_5DEG = FAIL_EXISTING_AUTHORITY`
- `J3_LOW_LOAD_MOTION_AUTHORITY = NOT_GRANTED`
- `J3_INSTALLED_LOAD_LOCAL_MOTION = NOT_TESTED`
- `J3_LOCAL_MOTION_AUTHORITY = NOT_GRANTED_FOR_FINAL_ASSEMBLY`
- `J2_ACTIVE_WORK = NO`
- `J2_ACTIVE_MOTION_AUTHORITY = NOT_GRANTED`

`J3_AUTHORITY_CHANGED = NO`
