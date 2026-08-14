# Ubuntu 22.04 工程交接：V15.18B 最终纯仿真基线

本文档面向接手 GO-M8010 工程的新电脑和 VMware Ubuntu 虚拟机。最终纯仿真交付引用为：

```text
branch: agent/v15-18b-final-simulation-handoff
tag: v15.18b-final-simulation-handoff
status: V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION
```

V15.18B 保留 production `implicitfast` 2 ms/1 ms 的原始 FAIL 证据，同时增加 `implicitfast` 0.5 ms 与 RK4 2 ms/1 ms reference。三层能量和全轨迹误差单调收敛，RK4 reference 通过；冻结结论为：

```text
NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED
CONTINUOUS_TIME_DYNAMICS_MODEL = PASS
V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION
UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST
PURE_SIMULATION_PHASE = COMPLETE
NEXT_PHASE = V15.19 REAL_HARDWARE_READONLY_BRINGUP
```

该结论没有放宽旧阈值，也没有修改 production 模型；旧 FAIL 被明确标记为 `PRODUCTION_INTEGRATOR_DIAGNOSTIC`，而不是被删除或伪装成 PASS。limitation 表示 production `implicitfast` 2 ms 不可作为无驱动被动运动的能量守恒 reference。Ubuntu runtime 必须在目标 Ubuntu 22.04/ROS 2 Humble 主机上实际复验；在此之前只能报告 `PENDING_ON_TARGET_HOST`，且该 pending 不属于 hard unresolved。

## 1. 平台、资产与安全边界

- Ubuntu 22.04 LTS，目标环境是 VMware guest，不是 WSL；
- ROS 2 Humble；
- Python 3.10 或更高的兼容 Python 3 版本；
- MuJoCo 3.11.0；
- NumPy 2.2.6；
- production MJCF 默认重力为 `0 0 0`；V15.18A/V15.18B 只在离线验收进程内存中启用重力；
- 仓库使用普通 Git object，不使用 Git LFS；V15.18A 的普通克隆包含已提交的完整模型资产；
- production MJCF 必须保有 1008 个 mesh 文件引用；V15.14 ROS 工作区必须是包含 5 个包的自包含工作区；
- 不得从旧电脑复制 `build/`、`install/`、`log/`、`.venv/`、`__pycache__/` 或缓存目录。

本交付不会自动启动 V15.19。只有用户另行明确授权后，才可进入 `V15.19 REAL_HARDWARE_READONLY_BRINGUP`；该阶段也不得使能电机或写入控制器、固件和安全参数。

## 2. 新电脑取得最终纯仿真基线

在目标 Ubuntu 中克隆最终交付分支：

```bash
git -c core.autocrlf=false -c core.quotepath=false clone \
  --single-branch \
  --branch agent/v15-18b-final-simulation-handoff \
  https://github.com/zhaowuc/go-m8010-robot-arm.git

cd go-m8010-robot-arm
git config --local core.autocrlf false
git config --local core.quotepath false

test "$(git branch --show-current)" = \
  "agent/v15-18b-final-simulation-handoff"
test "$(git rev-parse HEAD)" = \
  "$(git ls-remote origin refs/heads/agent/v15-18b-final-simulation-handoff | cut -f1)"
test "$(git rev-parse HEAD)" = \
  "$(git ls-remote origin refs/tags/v15.18b-final-simulation-handoff^{} | cut -f1)"
git fsck --full
git status --porcelain
```

最后一条命令必须无输出。`.gitattributes` 管理跨平台换行；不要手工转换 byte-exact 冻结 authority 文件。不要安装、初始化或调用 Git LFS。

本文件、V15.18B audit/validator、bootstrap、requirements 和 handoff verifier 均属于最终交付内容。不要从聊天附件、旧诊断目录或未校验压缩包替代远端分支/标签。

## 3. Ubuntu 系统依赖

以下命令安装通用工具和 MuJoCo GUI 所需库，不会自动安装或启动 ROS 2：

```bash
sudo apt update
sudo apt install -y \
  git \
  python3 \
  python3-venv \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  libgl1 \
  libgl1-mesa-dri \
  libglx-mesa0 \
  libglfw3 \
  libglew2.2 \
  libosmesa6 \
  libx11-6 \
  libxcursor1 \
  libxi6 \
  libxinerama1 \
  libxrandr2 \
  mesa-utils \
  x11-utils
```

ROS 2 Humble 必须按 ROS 官方 Ubuntu 22.04 安装流程单独安装。至少需要 Humble、MoveIt 2、`ros2_control`、ROS 2 controllers、RViz 2、Xacro 和构建工具。bootstrap 不会静默安装或替换 ROS；找不到 `/opt/ros/humble/setup.bash` 时会明确停止。

第一次使用 rosdep 的机器执行：

```bash
sudo rosdep init
rosdep update
```

若 `rosdep init` 报告 sources list 已存在，只执行 `rosdep update`。

## 4. V15.18B 交接 bootstrap

以下命令在目标 Ubuntu 22.04 主机的 fresh clone 中运行：

```bash
bash tools/bootstrap_ubuntu22_04_v15_18b.sh
```

脚本会检查 Ubuntu 22.04、Python、ROS 2 Humble、Git、colcon 和 rosdep，创建或复用 Python 虚拟环境，构建自包含的五包 ROS 工作区，并运行仓库、MuJoCo 与 V15.18B 检查。它始终核对 V15.18A 冻结报告 SHA；如另外提供冻结的 V15.18A 历史解释器，还会运行完整 V15.18A validator。headless 数值验收不会启动 ROS 节点。

目标主机完整复验成功后，脚本应打印 V15.18B limitation PASS 与 `PURE_SIMULATION_PHASE=COMPLETE`。在目标主机尚未执行前，项目报告保持 `UBUNTU_RUNTIME_VERIFICATION=PENDING_ON_TARGET_HOST`；不得把静态检查冒充真实 Ubuntu runtime PASS。

虚拟环境可放在仓库外：

```bash
V15_18B_VENV_DIR="${PWD}/../go-m8010-v15-18b-venv" \
  bash tools/bootstrap_ubuntu22_04_v15_18b.sh
```

虚拟环境不得提交到 Git。

V15.18A 的 byte-exact 历史报告来自 MuJoCo 3.11.0 / NumPy 2.5.2 环境；不要用 B2 的 NumPy 2.2.6 环境冒充 A 的历史重放。若目标主机已经准备了该独立环境：

```bash
V15_18A_PYTHON=/absolute/path/to/v15_18a/bin/python \
V15_18B_VENV_DIR="${PWD}/../go-m8010-v15-18b-venv" \
  bash tools/bootstrap_ubuntu22_04_v15_18b.sh
```

未提供时脚本明确输出 `V15_18A_RUNTIME_CHECK=PENDING_ON_TARGET_HOST`，不会伪报 A 的目标机 runtime PASS。GitHub fresh-clone 交付本身仍必须在冻结环境中实际完成 V15.18A `--check`。

## 5. 手动 headless 验收

在完整 fresh clone 中执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --disable-pip-version-check \
  -r requirements-sim-v15_18b.txt

python tools/verify_repository_handoff_v15_18b.py
printf '%s  %s\n' \
  640e9104cabd0e2548e07bdd54d5cdb2c66dbdf86a7981eff4d8c12e55b9dfe2 \
  V15_18A_静态重力与重力矩验收.json | sha256sum --check -
python tools/audit_passive_gravity_v15_18b.py --check
python tools/validate_passive_gravity_v15_18b.py --check
```

如需手动运行完整 V15.18A 历史检查，使用独立解释器：

```bash
"${V15_18A_PYTHON}" tools/validate_static_gravity_v15_18.py \
  --check --mujoco-python "${V15_18A_PYTHON}"
```

仓库 verifier 会验证普通 Git/LFS 零依赖、单文件不超过 100 MB、case/NFC 冲突、symlink、submodule、runtime 绝对路径、冻结 authority SHA、shell LF/执行位、五个 ROS 包、全部 URDF/Xacro mesh 引用，以及 production MJCF 的 1008 个文件引用、资产可读性和 MuJoCo compile/readback。

`FINAL_HANDOFF_ARTIFACTS` 与总体 `REPOSITORY_HANDOFF` 必须为 `PASS`。`--allow-dirty` 仅用于开发期诊断，不能替代 clean fresh-clone 验收。

V15.18A 历史 validator 使用其冻结的数值环境；V15.18B 使用 `requirements-sim-v15_18b.txt` 中的 MuJoCo 3.11.0 / NumPy 2.2.6。bootstrap 会保护 V15.18A authority，并由 V15.18B 独立复核其 PASS 状态。

这些命令是 headless 数值/完整性验收。不要为它们启动 MoveIt、`ros2_control`、production bridge、RViz 或任何位置控制器。

## 6. ROS 2 工作区构建检查

```bash
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths \
  V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src \
  --ignore-src -r -y \
  --rosdistro humble

cd V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

本地诊断工作区必须能发现恰好这五个自包含包：

```bash
ros2 pkg prefix go_m8010_arm_description
ros2 pkg prefix go_m8010_arm_mujoco_bridge
ros2 pkg prefix go_m8010_arm_v15_14_description
ros2 pkg prefix go_m8010_arm_v15_14_moveit_config
ros2 pkg prefix go_m8010_arm_v15_14_qa
```

构建成功只证明软件包完整；B2 的数值归因结论仍必须由 audit 与独立 validator 给出。

## 7. 仿真闭环启动边界

在需要观察既有 V15.14 仿真闭环时，可从仓库根目录设置 repo-relative 模型路径：

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/install/setup.bash"

export GO_M8010_V15_14_MJCF="${REPO_ROOT}/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"

ros2 launch go_m8010_arm_v15_14_moveit_config v15_14_bringup.launch.py \
  mujoco_model_path:="${GO_M8010_V15_14_MJCF}" \
  use_mujoco_viewer:=true \
  use_rviz:=true
```

这里不是 B2 数值 authority。production MJCF 中 `gravity="0 0 0"` 仍是冻结默认值；不得为了演示写回重力，也不得加入阻尼、摩擦、armature、重力补偿、PID 或电机力矩模型。

## 8. VMware 控制台显示 V15.18B 诊断画面

应在 VMware 的 Ubuntu 桌面会话内打开 Terminal，使 `DISPLAY` 和图形授权指向虚拟机控制台。不要使用 WSL、Xvfb 或 `ssh -X` 把窗口转发到宿主机。

先检查：

```bash
printf 'DISPLAY=%s\n' "${DISPLAY:-<unset>}"
printf 'XAUTHORITY=%s\n' "${XAUTHORITY:-<unset>}"
xdpyinfo >/dev/null
```

若必须从 SSH 驱动同一 VMware console，SSH shell 必须复用同一 Ubuntu 桌面用户的本地 X 会话。不要写死用户名、显示号或 `/home/...`：

```bash
grep -Eqi '(microsoft|wsl)' /proc/sys/kernel/osrelease /proc/version && \
  { echo 'WSL is not accepted for this GUI witness' >&2; exit 1; }
test "$(systemd-detect-virt --vm)" = vmware

console_display="$(who | awk -v user="$(id -un)" '
  $1 == user {
    for (i = 2; i <= NF; ++i) {
      value = $i
      gsub(/[()]/, "", value)
      if (value ~ /^:[0-9]+$/) {print value; exit}
    }
  }')"
test -n "${console_display}"
export DISPLAY="${console_display}"

unset XAUTHORITY
xauth_file="$(find "${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  -maxdepth 2 -type f -name Xauthority -readable -print -quit)"
[[ -n "${xauth_file}" ]] && export XAUTHORITY="${xauth_file}"

printf 'DISPLAY=%s\n' "${DISPLAY:-<unset>}"
printf 'XAUTHORITY=%s\n' "${XAUTHORITY:-<unset>}"
xdpyinfo >/dev/null
```

SSH 应使用密钥认证。不要把口令写入脚本、文档、Git、shell 历史或报告；曾共享的临时口令应立即轮换。

确认 `xdpyinfo` 成功后，只读显示诊断轨迹：

```bash
V15_18B_VENV_DIR="${V15_18B_VENV_DIR:-${PWD}/.venv}"
source "${V15_18B_VENV_DIR}/bin/activate"
python tools/audit_passive_gravity_v15_18b.py --visualize --dwell-seconds 1 --cycles 1
```

或让 bootstrap 探测同一 VMware console：

```bash
bash tools/bootstrap_ubuntu22_04_v15_18b.sh --vm-gui
```

可视化只用于见证实际运动，不是数值 authority。若 `DISPLAY`、`XAUTHORITY` 或 `xdpyinfo` 失败，必须明确失败，不能退回 WSL/Xvfb，也不能声称 VMware GUI 已验证。

## 9. GitHub 交接验证

交付时必须同时满足：

- branch `agent/v15-18b-final-simulation-handoff` 的远端 SHA 与本地 HEAD 相同；
- annotated tag `v15.18b-final-simulation-handoff` 指向同一提交；
- 真正 GitHub fresh clone（不得使用 `--reference`、`--shared` 或本地目录复制）通过 `git fsck --full`；
- fresh clone 中 repository verifier、production MuJoCo compile、V15.18A 与 V15.18B 检查全部通过；
- Git LFS pointer 为 0，缺失引用资产为 0，runtime Windows absolute path 为 0。

Ubuntu runtime 是目标主机的后续复验门；若当前没有目标主机，保持 `PENDING_ON_TARGET_HOST`，不把它写入 hard unresolved，也不伪报 PASS。

## 10. 下一阶段边界

纯仿真阶段完成后，下一阶段名称为 `V15.19 REAL_HARDWARE_READONLY_BRINGUP`，但本交付不会启动它；必须等待用户另行明确授权。`J2` 是一个逻辑自由度、两台镜像电机共同驱动。只读发现阶段不得使能电机或写参数。

保持以下终态并停止：

```text
V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION
UBUNTU_RUNTIME_VERIFICATION = PENDING_ON_TARGET_HOST
PURE_SIMULATION_PHASE = COMPLETE
NEXT_PHASE = V15.19 REAL_HARDWARE_READONLY_BRINGUP
```
