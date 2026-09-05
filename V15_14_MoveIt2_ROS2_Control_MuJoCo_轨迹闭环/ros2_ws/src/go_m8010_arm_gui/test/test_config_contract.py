from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "config"


def test_joint_limits_are_the_full_historical_model_ranges_at_the_vertical_anchor():
    limits = yaml.safe_load((CONFIG_DIRECTORY / "gui_joint_limits.yaml").read_text(encoding="utf-8"))
    assert limits["单位"] == "度"
    assert limits["参考"] == "SESSION_REFERENCE_V1"
    session_limits = [limits[f"J{index}"] for index in range(1, 7)]
    assert session_limits == [
        [-180.0, 180.0],
        [-260.0, 80.0],
        [-155.6, 184.4],
        [-129.49, 145.51],
        [-118.54, 103.26],
        [-180.0, 180.0],
    ]
    vertical_anchor_deg = [0.0, 90.0, -14.40, 13.49, 47.94, 0.0]
    recovered_absolute = [
        [round(lower + anchor, 2), round(upper + anchor, 2)]
        for (lower, upper), anchor in zip(session_limits, vertical_anchor_deg)
    ]
    assert recovered_absolute == [
        [-180.0, 180.0],
        [-170.0, 170.0],
        [-170.0, 170.0],
        [-116.0, 159.0],
        [-70.6, 151.2],
        [-180.0, 180.0],
    ]


def test_control_defaults_and_fixed_hold_gain_envelope_are_frozen():
    config = yaml.safe_load((CONFIG_DIRECTORY / "arm_gui.yaml").read_text(encoding="utf-8"))
    assert config["控制"]["控制频率_赫兹"] == 100
    # The requested profile must not ride the independent 5 deg/s hard stop:
    # the attended J1 run measured 5.085 deg/s while following a 5 deg/s plan.
    assert config["控制"]["最大速度_度每秒"] == 3.0
    # The shared default is the most restrictive active joint limit: J2's
    # established 15 deg/s^2 guard.  This keeps preview/router/GO execution
    # compatible without weakening any worker-side protection.
    assert config["控制"]["最大加速度_度每二次方秒"] == 15.0
    assert config["控制"]["到位容差_度"] == 0.5
    assert config["控制"]["J6到位容差_度"] == 0.08
    assert [config["关节"][f"J{index}"]["Kp"] for index in range(1, 6)] == [
        1.50, 3.00, 2.00, 2.00, 1.50,
    ]
    assert [config["关节"][f"J{index}"]["Kd"] for index in range(1, 6)] == [
        0.15, 0.30, 0.15, 0.15, 0.12,
    ]
    assert config["关节"]["J2"]["移动Kp上限"] == 3.00
    assert config["关节"]["J2"]["移动Kd上限"] == 0.30
    assert config["关节"]["J2"]["Tff"] == "有界共同保持积分_不重置目标"
    assert config["关节"]["J2"]["共同积分上限_Nm每转子"] == 1.50
    assert config["关节"]["J2"]["移动预测工作限_Nm每转子"] == 1.75
    assert config["关节"]["J2"]["固定保持预测工作限_Nm每转子"] == 3.00
    assert [
        config["关节"][f"J{index}"]["保持积分上限_Nm每转子"]
        for index in (1, 3, 4, 5)
    ] == [0.35, 1.60, 0.75, 0.50]
    assert [
        config["关节"][f"J{index}"]["总工作限_Nm每转子"]
        for index in (1, 3, 4, 5)
    ] == [2.50, 3.00, 2.50, 2.00]
    assert config["映射"]["J2同步制动阈值_度"] == 0.5
    assert config["关节"]["J6"]["模式"] == "POS_VEL"
