cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_v15_14_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
