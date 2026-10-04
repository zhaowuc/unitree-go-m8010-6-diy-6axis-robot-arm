cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_v15_14_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/static_ground_scene.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_ground_scene
)

ament_package()
