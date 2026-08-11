# V15.13 MuJoCo 空载运动学版

本版只验收四项：

1. `base_link`、`link1`～`link6`、`gripper` 的外形、尺寸和零位相对位置；
2. J1～J6 真实旋转中心；
3. J1～J6 轴向与右手法则正方向；
4. 机械位置限位、非相邻刚体自碰撞和物理地面碰撞。

主模型：`go_m8010_arm_v15_13_kinematic.xml`

拓扑固定为：

`world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper`

J6 为有限旋转 `[-π,+π]`，`J6=0` 时 Gemini Pro 相机在上。夹爪在这个六轴首版中冻结于已验证的零位，以 fixed body 连接到 `link6`。

## 最终验证结果

- MuJoCo：`3.11.0`，模型可实际编译；
- 拓扑：`nq=6`、`nv=6`，J1～J6 顺序正确；
- Link 零位位姿、关节中心、轴向和 `+0.5°` 右手法则见证点全部通过；
- 视觉网格未减面，原三角面集保留；仅为 MuJoCo 单 STL `200,000` 面上限分成 45 个块；
- 24 个 FreeCAD 审计碰撞代理生成 961 个米制凸体；
- 保存零位的完整 231 对运行时矩阵无碰撞；
- 原 503 姿态“V15.13 末端变更相关碰撞对”回归完全一致；随机 150 位姿仍仅索引 `43, 73, 75, 83, 87` 碰撞，与 FreeCAD/VTK 基准一致；
- 新增全运行时碰撞对的逐轴 5° 深度扫描（386 点）：J1、J6 全清空；J2 有 31 个对地禁用点；J3 有 29 个对地、9 个自碰撞点（35 个姿态并集）；J4 有 12 个对地、1 个自碰撞点；J5 有 2 个自碰撞点；这些点均被运行时守卫及扫掠路径检查 fail-closed 拦截；
- 网格地面位于 `Z=-0.090 m`，由 `base_link` 最低支承面精确导出。固定底座不与地面求解，所有运动 Link 启用地面碰撞；
- 完整报告：`QA_MUJOCO_V15_13_空载运动学版.json`。
- 逐轴深度报告：`QA_MUJOCO_V15_13_逐轴运动语义与地面复核.json`。

Link 局部包围尺寸（mm）：

| Link | X | Y | Z |
|---|---:|---:|---:|
| base_link | 176.921 | 176.866 | 100.000 |
| link1 | 96.500 | 137.600 | 149.190 |
| link2 | 307.096 | 99.500 | 115.901 |
| link3 | 299.261 | 52.500 | 148.876 |
| link4 | 60.828 | 97.000 | 176.265 |
| link5 | 70.000 | 70.000 | 127.527 |
| link6 | 34.992 | 35.001 | 1.000 |
| gripper | 107.129 | 133.135 | 123.402 |

## Ubuntu 22.04 执行

```bash
source /home/codex/mujoco_arm_env/bin/activate
python build_model.py
python validate_model.py
python validate_joint_motion_semantics.py
MUJOCO_GL=egl python render_previews.py
```

交互查看：

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  /home/codex/mujoco_arm_env/bin/python launch_viewer.py
```

单点限位/碰撞检查（角度制 J1～J6）：

```bash
python kinematic_guard.py 0 0 0 0 0 0
```

`KinematicGuard.check_swept_deg()` 以默认最大 `0.25°` 插值步长检查一段运动。

临时滑条 GUI（会同时打开可用鼠标旋转视角的 MuJoCo 窗口）：

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw \
  /home/codex/mujoco_arm_env/bin/python temporary_joint_gui.py
```

GUI 中显示两套角度：

- `MuJoCo / CAD绝对角`：模型真实 `qpos`，以已核验 CAD 机械零位为 0；
- `临时0相对角`：本次 GUI 会话的滑条偏置。

先用滑条移动到安全姿态，再点“保存当前姿态为启动 0”，界面会明确显示该 GUI 启动 0 对应的六个 CAD 绝对角，并写入 `gui_default_zero.json`。再次打开 GUI 时，模型直接恢复到该姿态，六个相对滑条均为 0°。该操作不修改 MJCF、URDF、CAD 机械零位或实机编码器零位；“回 CAD 机械零位”只影响当前会话，不删除已保存的 GUI 启动 0。地面、自碰撞、机械限位或穿越不安全中间姿态时，目标会被拒绝并保持上一安全姿态。

当前几何精校后的 GUI 启动 0（CAD 绝对角）：`J1=-135.0°、J2=150.0°、J3=-74.402070913°、J4=-90.0°、J5=45.0°、J6=0.713491710°`。该解保留规整的 J1/J2/J4/J5，仅根据真实 J3、J4 轴心求解 J3，使 J3→J4 中心线竖直；再根据 Gemini Pro 碰撞代理的 102.896 mm 长轴求解 J6，使相机长边与物理地面平行，同时保持既有相机俯仰/观看分支。复算脚本为 `calibrate_gui_zero.py`，完整残差、限位、碰撞及扫掠结果见 `QA_MUJOCO_V15_13_GUI零位几何校准.json`。

如果从 FreeCAD 原始网格全量重建，再执行：

```bash
python prepare_assets.py
python prepare_visual_chunks.py
python build_model.py
python validate_model.py
python validate_joint_motion_semantics.py
```

`V15_13_自碰撞对矩阵契约.json` 是运行时/MoveIt ACM 的对级契约；`kinematic_guard.py` 会仅上报契约中有效的物理对，忽略同刚体、轴承座和设计接触。

## 物理边界

当前 `inertial` 仅为 MuJoCo 编译所需的数值占位，不是实测质量/质心/惯量。未实测速度、力矩、阻尼、摩擦及负载；不得将本版用于动力学、力矩控制或实机参数整定。
