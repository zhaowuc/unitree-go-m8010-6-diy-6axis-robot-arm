#!/usr/bin/env bash
# Open the existing GUI only. Session preparation and drive admission stay in
# their existing tools; the desktop entry supplies no physical attestations.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
mode="${1:-}"
if [[ $# -gt 1 || ( -n "$mode" && "$mode" != --install && "$mode" != --check ) ]]; then
  printf '用法：%s [--install|--check]\n' "${0##*/}" >&2
  exit 2
fi

notice() {
  printf '%s\n' "$1" >&2
  if [[ "$mode" != --check ]] && command -v zenity >/dev/null; then
    zenity --info --title='六轴机械臂控制' --text="$1" --width=440 2>/dev/null || true
  fi
}

if [[ "$mode" == --install ]]; then
  applications="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
  mkdir -p "$applications"
  python3 - "$repo" "$applications/go-m8010-arm-control.desktop" <<'PY'
from pathlib import Path
import sys
repo, destination = map(Path, sys.argv[1:])
launcher = str(repo / 'tools/start_arm_gui_menu.sh')
for old, new in [('\\', '\\\\'), ('"', '\\"'), ('`', '\\`'), ('$', '\\$'), ('%', '%%')]:
    launcher = launcher.replace(old, new)
text = (repo / 'tools/go-m8010-arm-control.desktop.in').read_text(encoding='utf-8')
destination.write_text(text.replace('@LAUNCHER@', launcher), encoding='utf-8')
print(destination)
PY
  desktop-file-validate "$applications/go-m8010-arm-control.desktop"
  update-desktop-database "$applications"
  exit 0
fi

existing="$(python3 - <<'PY'
from pathlib import Path
import os
gui = starting = False
for process in Path('/proc').glob('[0-9]*'):
    try:
        if process.stat().st_uid != os.getuid():
            continue
        args = [item.decode(errors='replace') for item in (process / 'cmdline').read_bytes().split(b'\0') if item]
    except (OSError, ProcessLookupError):
        continue
    names = {Path(arg).name for arg in args}
    gui |= bool(names & {'v15_31d_gui_j1_demo.py', 'arm_gui', 'arm_control_gui'}) or any(
        args[i:i+2] == ['-m', 'go_m8010_arm_gui.main_window'] for i in range(len(args)))
    starting |= ('v15_31d_demo_session.py' in names and '--execute' in args) or (
        'start_arm_gui.sh' in names and '--prebuild-only' not in args)
print('gui' if gui else 'starting' if starting else 'none')
PY
)"
if [[ "$mode" == --check ]]; then
  printf 'GUI_PROCESS_STATUS=%s\n' "$existing"
else
  # Keep menu double-clicks from racing the process check. Other existing GUI
  # carriers do not share this lock, so their real command lines are checked too.
  exec 9>"${XDG_RUNTIME_DIR:-/tmp}/go-m8010-arm-menu-${UID}.lock"
  if ! flock -n 9; then
    notice '机械臂控制窗口正在打开，请稍候。'
    exit 0
  fi
  if [[ "$existing" == gui ]]; then
    notice '机械臂控制窗口已经运行。请通过 Alt+Tab 或活动概览切回现有窗口。'
    exit 0
  fi
  if [[ "$existing" == starting ]]; then
    notice '机械臂会话正在启动，请稍候。'
    exit 0
  fi
fi

ws="$repo/V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
python="$repo/.venv/arm-gui/bin/python"
params="$repo/.runtime/v15_31b_ft/current/attended_gui_params.yaml"
for path in /opt/ros/humble/setup.bash "$ws/install/setup.bash" "$python" "$params"; do
  if [[ ! -f "$path" ]]; then notice "启动所需文件不存在：$path"; exit 1; fi
done
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export PYTHONPATH="$ws/src/go_m8010_arm_gui:$ws/src/go_m8010_arm_hardware:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$repo/tools/hardware/v15_31d_demo_templates/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export MUJOCO_GL=glfw
if [[ -z "${QT_QPA_PLATFORM:-}" ]]; then
  if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then export QT_QPA_PLATFORM=wayland;
  elif [[ -n "${DISPLAY:-}" ]]; then export QT_QPA_PLATFORM=xcb; fi
fi
mkdir -p "$repo/logs/arm_gui"
run="$(mktemp -d "$repo/logs/arm_gui/menu-$(date -u +%Y%m%dT%H%M%S)-XXXXXX")"
"$python" - "$repo" "$params" "$run" <<'PY'
from pathlib import Path
import importlib.util
import sys
import yaml
repo, source, run = map(Path, sys.argv[1:])
data = yaml.safe_load(source.read_text(encoding='utf-8'))
params = data['/**']['ros__parameters']
previous_root = str(Path(params['initial_pose_path']).parents[2])
for key, value in list(params.items()):
    if isinstance(value, str) and value.startswith(previous_root + '/'):
        params[key] = str(repo) + value[len(previous_root):]
params['initial_pose_read_only'] = True
params['log_directory'] = str(run)
for key in ('config_path', 'joint_limits_path', 'initial_pose_path', 'thermal_config_path', 'embedded_model_path'):
    if not Path(params[key]).is_file():
        raise FileNotFoundError(f'{key}: {params[key]}')
spec = importlib.util.find_spec('go_m8010_arm_gui.main_window')
if spec is None:
    raise RuntimeError('go_m8010_arm_gui.main_window is unavailable')
with (run / 'gui_params.yaml').open('x', encoding='utf-8') as stream:
    yaml.safe_dump(data, stream, allow_unicode=True)
print('GUI_ENTRY=' + str(spec.origin))
PY
command=("$python" -m go_m8010_arm_gui.main_window --ros-args --params-file "$run/gui_params.yaml")
if [[ "$mode" == --check ]]; then
  printf 'GUI_MENU_CHECK=PASS; NO_GUI_OR_ROS_NODE_STARTED\n'
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi
cd "$repo"
if ! "${command[@]}" >"$run/gui.log" 2>&1; then
  notice "控制窗口退出异常，启动日志：$run/gui.log"
  exit 1
fi
