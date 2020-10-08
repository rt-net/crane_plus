cmake_minimum_required(VERSION 3.5)
project(crane_plus_ign)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
