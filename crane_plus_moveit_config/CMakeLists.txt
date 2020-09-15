cmake_minimum_required(VERSION 3.5)
project(crane_plus_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
