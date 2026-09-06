本目录保存现场已验证的会话启动脚本。来源：2026-09-06 的
`.codex-tmp/hold_session_20260906`，原始 raw capture → 保留参考 →
只读证据 → 经验许可 → 绑定 worker 的顺序不变。

正式入口在仓库根目录运行：

```bash
python3 tools/hardware/v15_31d_demo_session.py
python3 tools/hardware/v15_31d_demo_session.py --self-test
python3 tools/hardware/v15_31d_demo_session.py --execute --cycles 3 --supported --vertical --hands-off --clearance
```

第一条只检查文件、模板语法并显示计划。最后一条的四个标志表示操作员确认
机械臂已独立可靠支撑、处于已确认的竖直姿态、已松手、活动范围净空。
同一会话建立一次真实重力阶梯，再执行指定的 1～3 次 J1 1°往返动作组；
其他轴的小修正按现有逐轴控制链记录，不能称为仅 J1 收到 POSITION。

未再次断电时复用原工具已完成的 POS_VEL 配置，不重复发起模式设定；
J6 worker 启动时仍读取实际 RID10，非 POS_VEL 会拒绝启用。

每次创建独立 `.runtime/v15_31d_demo_<时间及随机编号>` 和旁边的 `_scripts`
目录。前者保存原始证据和动作组报告，后者保存展开脚本、各阶段日志及
`session_result.json`。退出时停止本次拥有的 worker、重力节点和状态核心，
同时核对新 worker 日志的 GO `FINAL_BRAKE=PASS`、J6 `J6_FINAL_DISABLED=PASS`。
已有硬件 worker 或其他活动工程服务会被拒绝；仅已识别的旧演示纯状态核心
可以在预构建完成后停止。

当前四份校准/保留参考 SHA 由入口和原脚本核对；缺失文件会列出具体路径。
入口不会生成校准、不更新零位，也不移动旧证据。`J6_PYTHON` 的默认值与
`start_arm_gui.sh` 一致，可用既有环境变量指定另一个已配置的 J6 Python。
ROS 2 Humble、工作区依赖及桌面会话继续使用本机已有安装。

正常重复运行复用已完成的 J6 模式配置，实机 worker 仍须读取 RID10 并确认
POS_VEL 后才可启用。仅在确实发生了 J6 24V 断电重上电时增加 `--power-cycled`：
这时才调用原 commissioning 工具，使用新 PREFLIGHT_READY token 执行必要的
运行时模式切换，并确认最终 5 帧 DISABLED。默认不声称再次断过电，也不重复
申请已消费的模式设定。账本归档由原工具管理，不增加 Flash 或零位写入。

`fastdds_udp_only.xml` 是从当时实机已使用的 `/tmp/fastdds_udp_only.xml`
只读复制的原文。入口把它放到新的脚本目录，不依赖 `/tmp` 留存配置。
模板中的 `@@REPO@@`、`@@SESSION@@`、`@@SCRIPTS@@` 会经 shell 引号处理；
`@@UNIT@@` 只由入口生成。不要直接执行未展开的模板。

操作员确认整臂仍在原初始化构型附近、保持独立支撑，并授权自动回正时，可增加
`--supported-near-vertical-recovery`。普通启动仍保留原 ±2° 检查；该显式恢复模式
对新 BRAKE 采样逐电机验证原参考附近唯一圈数分支，软件准入上限为 5°，
不是外部角度测量结果。原参考、零位、双机同步限制和新鲜反馈复核均保留。
启动后先在实际位置建立 HOLD，通过原重力阶段，再以预演和 PLAN_TOKEN
逐轴回到原初始姿态；成功后才捕获新的真实 HOLD 起点并执行所选 1～3 轮往返。
仅大臂竖直而其他关节已翻转，不满足“整臂在原初始化构型附近”的条件。
