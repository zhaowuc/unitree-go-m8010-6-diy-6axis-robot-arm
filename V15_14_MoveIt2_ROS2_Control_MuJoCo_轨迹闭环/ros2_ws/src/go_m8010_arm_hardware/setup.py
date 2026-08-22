from glob import glob
from setuptools import find_packages, setup


package_name = "go_m8010_arm_hardware"

setup(
    name=package_name,
    version="15.30.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GO-M8010 project",
    maintainer_email="maintainer@example.invalid",
    description="State-only whole-arm hardware state and direct-qpos mirror.",
    license="Proprietary project data",
    entry_points={
        "console_scripts": [
            "whole_arm_state_node = go_m8010_arm_hardware.whole_arm_state_node:main",
            "whole_arm_mujoco_mirror = go_m8010_arm_hardware.mujoco_mirror_node:main",
            "mock_motor_feedback = go_m8010_arm_hardware.mock_feedback_node:main",
        ]
    },
)
