# V15 刚性 Link 拆分

本目录不是“整机一个 STL”。主机械臂按以下刚体链导出：

`world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper`

- `visual/*.stl` 与 `collision/*.stl` 均在各自 Link 局部坐标系中，STL 数值单位为 mm。
- ROS 2 / MuJoCo 使用时 mesh scale 为 `0.001 0.001 0.001`。
- 每颗螺丝、垫圈和固定支架已经并入随动刚体，没有生成独立 Link。
- J1/J2 的完整 GO-M8010 STEP 已按工程中审计过的 BRep 固体编号拆成定子、输出转子和中性 B6808 显示件，未按外观猜测。
- `gripper.stl` 只包含与连接件、舵机壳体和固定框架相对静止的静态部分。
- 夹爪六个会相对运动的刚体位于 `gripper_internal_zero_reference/`；这些文件共用 `gripper` 零位坐标，仅用于后续建立夹爪内部关节，不得重新焊成一个刚体。
- `gripper_complete_zero_reference.stl` 只是零位总览，不能作为单个 URDF/MuJoCo 刚体。
- 质量、质心、惯量、速度/力矩上限、MuJoCo 设计接触排除和 tool0/TCP 尚无实测数据，未填写猜测值。
