cmake_minimum_required(VERSION 3.5)
project(crane_plus_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  gui
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
