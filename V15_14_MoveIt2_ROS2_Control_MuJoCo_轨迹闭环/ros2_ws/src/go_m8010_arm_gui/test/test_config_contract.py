from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "config"


def test_joint_limits_are_session_relative_and_bounded():
    limits = yaml.safe_load((CONFIG_DIRECTORY / "gui_joint_limits.yaml").read_text(encoding="utf-8"))
    assert limits["单位"] == "度"
    assert limits["参考"] == "SESSION_REFERENCE_V1"
    assert limits["J2"] == [-5.0, 5.0]
    assert [limits[f"J{index}"] for index in (1, 3, 4, 5, 6)] == [[-10.0, 10.0]] * 5


def test_control_defaults_and_j2_verified_controller_are_frozen():
    config = yaml.safe_load((CONFIG_DIRECTORY / "arm_gui.yaml").read_text(encoding="utf-8"))
    assert config["控制"]["控制频率_赫兹"] == 100
    assert config["控制"]["最大速度_度每秒"] == 5.0
    assert config["控制"]["最大加速度_度每二次方秒"] == 20.0
    assert config["关节"]["J2"]["最终负载跟踪"] == "V15.30E双向验证通过"
    assert config["关节"]["J2"]["Kd"] == 0.10
    assert config["关节"]["J2"]["Tff"] == "有界公共积分"
    assert config["映射"]["J2同步制动阈值_度"] == 0.5
    assert all(config["关节"][f"J{index}"].get("Tff", 0.0) == 0.0 for index in (1, 3, 4, 5, 6))
