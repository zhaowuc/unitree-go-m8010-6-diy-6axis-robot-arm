# 基于宇树go-m8010-6关节电机的自组6轴机械臂

这是 GO-M8010-6 + DM-G6220 六自由度机械臂的长期主工程。当前主线为 V15.13，仓库保留机械设计、几何与碰撞验收、仿真集成资产以及后续 ROS 2 / MoveIt 2 开发所需的源文件。

## 主要内容

- FreeCAD 完整机械装配、刚性 Link 拆分工程和 STEP/STL 模型；
- J1～J6 真实关节中心、轴向、零位与机械位置限位定义；
- MuJoCo 空载运动学模型、物理网格地面、自碰撞守卫和临时关节控制界面；
- ROS 2 工作区、URDF/Xacro、视觉与碰撞网格；
- MoveIt 2 配置、关节限位和自碰撞策略；
- 装配预览、实物运动证据、几何/碰撞 QA 报告和复现工具。

## 机械与运动学约定

- 软件运动学保持六个机械自由度：`J1`～`J6`。
- `J2` 是一个机械自由度，由两台镜像安装的 GO-M8010-6 共同驱动；两台电机不是两个独立运动学关节，软件中仍只有一个 `J2`。
- `world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper`。
- `tool_reference` 是 J6 输出/夹爪安装中心。
- `tcp_nominal` 是 `ClosureAngle=0°` 时左右硅胶有效夹持面面积质心的中点，位于 gripper 局部坐标：

  ```text
  [-0.001187726400, 0.000060729026, 0.092493513872] m
  ```

  当前只在 MuJoCo 中作为固定名义参考点使用；动态 TCP、ROS TF 和 MoveIt TCP 尚未实现。

## 目录

- `机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd`：当前完整装配。
- `机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd`：刚性 Link 拆分工程。
- `mujoco_kinematic_v1/`：当前可运行的 MuJoCo 空载运动学版本。
- `rigid_links_v15_13/`：ROS 2 / MuJoCo 刚性 Link 网格与导出定义。
- `完整工程_V15_13_交付/ros2_ws/`：ROS 2、URDF/Xacro 与 MoveIt 2 工程。
- `完整工程_V15_13_交付/source_models/`：夹爪和相机源模型。
- `evidence/`、`装配预览图/`、`三视图资产_world/`：验收证据与展示资产。
- `完整工程_V15_13_交付/tools/`、`mujoco_kinematic_v1/windows_export_tools/`：构建、导出和验证工具。

## 当前工程边界

当前 MuJoCo 版本首先保证外形、尺寸、相对位置、J1～J6 运动学、机械限位和自碰撞关系。质量、质心、惯量、速度、力矩、阻尼、摩擦和负载参数仍需依据实测数据完善，不应把现有占位值用于最终动力学或实机力矩控制。

后续计划包括重力补偿、动态 TCP、ROS 2 控制链、MoveIt 2 规划完善以及 Gemini Pro 视觉抓取。

## 大型文件

FCStd、STEP/STP、STL、OBJ、DAE、GLB、MP4 等机械与媒体资产由 Git LFS 管理。克隆后请确保已安装 Git LFS，并执行：

```bash
git lfs install
git lfs pull
```


## 开源许可证

本项目采用 [MIT 许可证](LICENSE)。第三方模型、依赖和素材遵循各自的许可证。
