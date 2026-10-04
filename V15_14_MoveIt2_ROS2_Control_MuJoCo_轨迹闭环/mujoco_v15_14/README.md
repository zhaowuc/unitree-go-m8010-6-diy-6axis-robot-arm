# V15.14 派生 MuJoCo 碰撞模型

本目录由冻结的 V15.13 空载运动学模型派生，补入 `UpperArm_Motion_Collision_Proxy`，不修改 V15.13 权威 XML、FreeCAD 装配、J1～J6、TCP 或相机外参。

## 来源与单位

- `UpperArm_Motion_Collision_Proxy` 从冻结 FCStd 只读提取，并变换到 `link2` 局部坐标系。
- 原始 STL 和分组件 STL 均使用毫米；MJCF `<mesh>` 通过 `scale="0.001 0.001 0.001"` 转换为米。
- 该代理有两个不连通实体，分别作为两个 MuJoCo mesh，避免单一凸包跨越实体间空隙。

## 确定性重建

在 `V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/tools` 中执行：

```powershell
D:\freecad\bin\freecadcmd.exe -c "exec(open('export_upperarm_motion_proxy_v15_14.py', encoding='utf-8').read())"
python generate_v15_14_description.py
python validate_v15_14_collision_layer.py
```

运行时 MuJoCo 验证需在已安装 `mujoco` Python 包的 Ubuntu/ROS2 环境执行最后一条命令。生成器会 fail-closed 校验冻结输入哈希、25 个代理 token、231 个启用 pair、69 个排除 pair，以及 URDF/MJCF 的 token 覆盖。

## 碰撞语义

`UpperArm_Motion_Collision_Proxy` 只参与以下两个 token pair：

- `J1_Fixed_Collision_Proxy` ↔ `UpperArm_Motion_Collision_Proxy`
- `J1_Moving_Collision_Proxy` ↔ `UpperArm_Motion_Collision_Proxy`

它与其余 22 个代理 pair 均被显式排除。MJCF 中 Motion geom 的通用 mask 为关闭状态，仅通过显式 `<pair>` 启用上述两组 token pair。
