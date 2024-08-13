// Copyright 2020 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference:
// https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes
// /run_move_group/src/run_move_group.cpp

#include <cmath>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("joint_values", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();

  auto joint_values = move_group_arm.getCurrentJointValues();
  double target_joint_value = to_radians(45.0);
  for (size_t i = 0; i < joint_values.size(); i++) {
    joint_values[i] = target_joint_value;
    move_group_arm.setJointValueTarget(joint_values);
    move_group_arm.move();
  }

  move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
