cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_v15_14_qa)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/v15_14_regression
  scripts/v15_14_regression.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
