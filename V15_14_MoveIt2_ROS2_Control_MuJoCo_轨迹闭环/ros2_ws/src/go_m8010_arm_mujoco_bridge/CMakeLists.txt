cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_mujoco_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/mujoco_bridge
  scripts/mujoco_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
