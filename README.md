# GO-M8010 六自由度机械臂工程

这是 GO-M8010-6 + DM-G6220 六自由度机械臂的长期主工程。当前纯仿真交付分支为：

```text
branch: agent/v15-18b-final-simulation-handoff
status: V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION
PURE_SIMULATION_PHASE = COMPLETE
NEXT_PHASE = V15.19 REAL_HARDWARE_READONLY_BRINGUP
```

V15.18B 保留了 production `implicitfast` 2 ms/1 ms 的原始能量与绝对终值 FAIL。补充的 0.5 ms、多层全轨迹收敛及 RK4 2 ms/1 ms 参考证明误差随步长正常下降，根因分类为：

```text
NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED
CONTINUOUS_TIME_DYNAMICS_MODEL = PASS
V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION
UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST
```

该结论没有放宽旧阈值，也没有修改 production MJCF、bridge、Mass、COM、Inertia、URDF、controller 或碰撞/TF authority。limitation 的精确含义是：`PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION`。当前 production bridge 使用运动学位置跟踪；本次 passive free-fall 不是 production 控制模式，也不授权改变 production 积分器。

## 已通过的能力

- V15.13：整机几何、真实关节轴、机械零位、J1～J6 限位与碰撞契约；
- `tcp_nominal`、ROS 2 TF、MoveIt 2、`ros2_control` 与 MuJoCo 轨迹闭环；
- V15.15：实测质量账本与 COM V2；
- V15.16：Inertia Engineering V1；
- V15.17：URDF/MJCF inertial deployment 与 production bridge hash authority；
- V15.18A：21 个无碰撞姿态的静态重力、重力矩与动力学恒等式验收；
- V15.18B：6 个冻结姿态的短时被动重力仿真、implicitfast 三层步长收敛及 RK4 reference 验收。

软件运动学始终只有 `J1`～`J6` 六个自由度。`J2` 由两台镜像安装的 GO-M8010-6 电机共同驱动，但仍是一个逻辑关节，不能拆成两个独立运动学自由度。`tcp_nominal` 是 `ClosureAngle=0°` 时左右硅胶有效夹持面面积质心的中点：

```text
[-0.001187726400, 0.000060729026, 0.092493513872] m
```

## 主要目录

- `V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/`：production MJCF、ROS 2 Humble 工作区、MoveIt 2、`ros2_control`、MuJoCo bridge 与闭环验收证据；
- `mujoco_kinematic_v1/`：production MJCF 引用的 1008 个网格资产及冻结运动学资源；
- `rigid_links_v15_13/`：刚性 Link 定义、导出清单与网格；
- `tools/`：质量、COM、惯量、部署、静态重力、被动重力及仓库交接验证工具；
- `docs/UBUNTU22_04_HANDOFF_V15_18B.md`：Ubuntu 22.04 / VMware 接手、复现和可视化说明。

## 克隆与资产

本仓库的 CAD、STL、MJCF、URDF、图片和证据文件全部以普通 Git object 保存，**不使用 Git LFS**。普通 `git clone` 即会取得已提交的完整资产，不需要额外的大文件下载步骤。冻结文本的跨平台换行规则由仓库根目录 `.gitattributes` 管理。

新电脑应明确克隆 `agent/v15-18b-final-simulation-handoff`，并核对远端分支与 annotated tag `v15.18b-final-simulation-handoff` 指向同一最终提交。详细的目标机验证方法见[交接文档](docs/UBUNTU22_04_HANDOFF_V15_18B.md)。Ubuntu runtime 在目标主机实际执行前保持 `PENDING_ON_TARGET_HOST`，不得伪报 PASS；这一 pending 不是 hard unresolved。

## 安全边界

production MJCF 的默认重力仍为 `0 0 0`。V15.18A/V15.18B 只在验收进程内存中设置重力，不会把重力、阻尼、摩擦、armature、控制器或电机模型写回 production 模型。仿真参数不是实机电流、力矩或安全参数的替代品。

本任务只完成纯仿真冻结，不会自动开始 V15.19。下一次必须由用户明确授权 `V15.19 REAL_HARDWARE_READONLY_BRINGUP`。进入实机后，完成 CAN/串口枚举、电机 ID、方向、零位、限位、电流限制、急停和 J2 双电机一致性核对前，不得使能电机，更不得直接同时使能 J2 的两台电机。
