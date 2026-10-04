# MuJoCo V15.13 staging

关节树、轴、原点、J2～J6 位置限位和 visual/collision 网格已写入模板。J6 为 `hinge`，`ref=0` 时相机在上，`range=-π +π`。

`tcp_nominal`、已冻结的 `camera_link`、仿真专用 `sim_camera_optical_frame` 与 MuJoCo renderer camera 已同步固化。renderer 的 `fovy=60` 仅是合成渲染设定，不宣称为 Gemini Pro 真实内参。

模板中的质量、质心、完整惯量张量和摩擦参数均为 `__...__` 占位符，必须由实测/标定值替换；没有替换前不是可运行的动力学模型。不要让 MuJoCo 依据高细节网格默认密度猜惯量。组合动作仍需使用碰撞检测，因为独立关节限位的笛卡尔积包含自碰撞姿态。
