from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "config"


def test_joint_limits_are_session_relative_and_bounded():
    limits = yaml.safe_load((CONFIG_DIRECTORY / "gui_joint_limits.yaml").read_text(encoding="utf-8"))
    assert limits["单位"] == "度"
    assert limits["参考"] == "SESSION_REFERENCE_V1"
    assert [limits[f"J{index}"] for index in range(1, 7)] == [[-10.0, 10.0]] * 6


def test_control_defaults_and_j2_blocker_are_frozen():
    config = yaml.safe_load((CONFIG_DIRECTORY / "arm_gui.yaml").read_text(encoding="utf-8"))
    assert config["控制"]["控制频率_赫兹"] == 100
    assert config["控制"]["最大速度_度每秒"] == 5.0
    assert config["控制"]["最大加速度_度每二次方秒"] == 20.0
    assert config["关节"]["J2"]["最终负载跟踪"] == "待解决"
    assert all(config["关节"][f"J{index}"].get("Tff", 0.0) == 0.0 for index in range(1, 7))
