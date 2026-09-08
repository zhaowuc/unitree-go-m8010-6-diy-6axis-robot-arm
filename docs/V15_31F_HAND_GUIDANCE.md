# 整臂柔顺手导（V15.31F）

目标：内层位置闭环承担机械臂自身重量，外层根据估算外力产生有界参考；撤去手力后自动减速并保持新目标，再施力可以继续。鼠标按钮是启用开关，撤去手力与松开按钮是两个不同事件。

新增模式使用独立 `gui-command/1.5` 和手导运行包，保留旧单轴 `1.4`、普通位置轨迹和严格精度验收。新参考上限 30°/秒、距本次起始姿态每轴 10°、参考与实角距离 2°；这些上限不是已达到的速度或制动距离。虚拟惯量、阻尼和加速度仍会限制实际手感。J2 两电机按一个关节同步；J6 保持 POS_VEL，通过已核单位的电机力矩估计参与外环，不切 MIT。

用户确认只有底座固定，臂段没有外部支撑。新包与阶段确认记录这一事实，不声明 J2/J3 有支架承重。启动先建立当前角度 HOLD，再进行原重力阶梯与静止偏置采样；拖动期间不学习偏置，不伪造无人接触确认。正常结束先冻结各域已接受参考，收到同一保持确认后通过原碰撞预演/分段轨迹返回本次起始竖直姿态，确认到位后才停止驱动。回位未完成时保留有效 HOLD 并显示具体问题，不以 GUI 超时直接杀进程。通信、温度、同步、实际超速、原工作力矩限制、租约和包到期保护仍有效。

## 当前证据

- 真实 J6 只读检查：RID 7/8/10/21/22/23 为 0/1/2/12.5/45/10，KT_OUT=0、Gr=1；100 帧均失能，未写参数、未改变模式、未使能。见 [参数记录](../hardware/v15_31f_guidance_20260908/j6_protocol_torque_readonly.json)。力矩是按协议单位换算的电机估计，不是独立外力或电流测量。
- 用冻结质量/惯量模型执行 MuJoCo `mj_step`，通过位置内环和延迟、量化的电机力矩估算外力；已覆盖多轴同时推动、撤力、再次推动。模型 J6 使用明确假设的有界 POS_VEL 代理；它不验证真实内部限矩，也不验证 J2 双电机差动同步。
- 30°/秒配置的参考峰值仍受其他限制。突然撤力的模型试验中，J2/J3 约需 1 秒停稳，额外位移约 0.48°/1.10°；不能据末态收敛声称“瞬间停住”。实际手导效果尚待实机验证。

## 运行

离线动力学验证不会访问机械臂：

```bash
python3 tools/hardware/validate_hand_guidance_dynamics.py --speed-deg-s 30 --output /tmp/hand_guidance_dynamics.json
```

先使用只读参数工具生成新的 J6 参数记录（使用工程现有 J6 Python/USB 库环境）：

```bash
python tools/hardware/j6_dm_g6220/j6_protocol_torque_readonly.py --execute-readonly --samples 100 --output /tmp/j6_readback.json
```

先观察实际 HOLD 下的拟输出，实际保持目标不变；准备阶段保持手离开机械臂。将参数 SHA 替换为只读报告打印的 `readback_sha256`：

```bash
python3 tools/hardware/v15_31d_demo_session.py --execute --hand-guidance --guidance-shadow --guide-speed-deg-s 30 --base-fixed --vertical --hands-off --clearance --j6-torque-readback /tmp/j6_readback.json --expected-j6-torque-readback-sha256 <参数SHA>
```

静置观察完成后正常回位/停机并保存报告。实际拖动入口去掉 `--guidance-shadow`；工具栏就绪后按住“整臂柔顺拖动”，撤去手力会减速保持，松开按钮触发原生参考冻结和全轴 HOLD 确认。记录姿态在静止 HOLD 后进行；本会话仅记录，动作组执行保留在常规控制入口。结束按钮和窗口关闭会先回本次起始姿态。

`--power-cycled` 仅用于确实发生过的新上电事件，不因软件更新或重复运行而再次填写。
