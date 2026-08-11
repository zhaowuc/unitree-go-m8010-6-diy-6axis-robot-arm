# Ubuntu 22.04 VM 环境核验

核验主机：`codex@192.168.11.130`，系统：Ubuntu 22.04，主机名：`ubuntu22-vm`。

- ROS 2 Humble：已安装，`ROS_VERSION=2`、`ROS_DISTRO=humble`；
- ROS 包约 380 个，含 `ros-humble-desktop`、RViz2、Gazebo、`robot_state_publisher`、`joint_state_publisher`；
- ROS 2 Control：`controller_manager`、`joint_trajectory_controller` 已安装；
- Xacro、colcon：已安装；
- MoveIt 2：`moveit_ros_move_group` 与 `moveit_setup_assistant` 当前未安装；
- MuJoCo 环境：`/home/codex/mujoco_arm_env`，版本 `3.11.0`；
- 本次补充：`trimesh 4.7.4`、`coacd 1.0.11`、`fast-simplification 0.1.12`、`networkx 3.4.2`；
- Pillow 已恢复到 VM 原有的 `12.3.0`；
- 图形：`DISPLAY=:1`，VMware SVGA3D，OpenGL core 4.1，直接渲染可用。

VM 工程路径：`/home/codex/projects/mujoco_kinematic_v1`。
