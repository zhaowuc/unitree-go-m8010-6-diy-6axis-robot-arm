from glob import glob
from setuptools import find_packages, setup

package_name = "go_m8010_arm_gui"

setup(
    name=package_name,
    version="15.30.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="car",
    maintainer_email="car@example.com",
    description="六自由度机械臂全中文控制界面",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "arm_gui = go_m8010_arm_gui.main_window:main",
            "arm_gui_command_router = go_m8010_arm_gui.command_router:main",
        ],
    },
)
