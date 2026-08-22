# V15.30A 全臂状态与 MuJoCo 直接镜像

该 ROS2 包把四个独立物理总线进程的本机 UDP 反馈聚合为六个逻辑关节，以 50 赫兹发布名称严格为 `joint1..joint6` 的 `/joint_states`，并用 MuJoCo `qpos + mj_forward` 直接镜像。包内不实现 MoveIt、零重力、逆运动学或电机内部设零。

物理源名为 `J1,J2A,J2B,J3,J4,J5,J6`。J2A/J2B 只在内部存在；对外 J2 位置是两个符号映射后的平均，同时发布同步误差。每个健康电机独立捕获 50 帧静止窗口，形成 `SESSION_REFERENCE_V1`；任一缺失源不会阻断其他关节显示。

完整构建、启动、模式和安全退出都由仓库根目录的 `./start_arm_gui.sh` 管理。请不要在旁边启动第二个串口或 CAN 读写进程。操作说明见 `docs/GUI_QUICK_START_CN.md`。
