cmake_minimum_required(VERSION 3.5)
project(crane_plus_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(crane_plus_control_node
  src/crane_plus_control_node.cpp src/crane_plus_interface.cpp src/crane_plus_driver.cpp)

ament_target_dependencies(crane_plus_control_node
    controller_interface
    controller_manager
    dynamixel_sdk
    hardware_interface
    rclcpp
)

install(TARGETS crane_plus_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
