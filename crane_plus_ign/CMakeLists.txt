cmake_minimum_required(VERSION 3.5)
project(crane_plus_ign)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(joint_trajectory_converter_component SHARED
  src/joint_trajectory_converter.cpp)
target_compile_definitions(joint_trajectory_converter_component
  PRIVATE "CRANE_PLUS_IGN_BUILDING_DLL")
ament_target_dependencies(joint_trajectory_converter_component
  control_msgs
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_nodes(joint_trajectory_converter_component "crane_plus_ign::JTrajectoryConverter")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(control_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclcpp_components)

ament_export_include_directories(include)
ament_export_libraries(
  joint_trajectory_converter_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  joint_trajectory_converter_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()
