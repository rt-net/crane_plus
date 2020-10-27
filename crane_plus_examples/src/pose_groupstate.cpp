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

#include <string>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveItErrorCode = moveit::planning_interface::MoveItErrorCode;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using RobotStatePtr = moveit::core::RobotStatePtr;
using JointModelGroup = const moveit::core::JointModelGroup;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_groupstate");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("pose_groupstate", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  static const std::string PLANNING_GROUP = "arm";
  MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
  move_group.setMaxAccelerationScalingFactor(0.5);  // Set 0.0 ~ 1.0

  move_group.setNamedTarget("home");
  move_group.move();

  move_group.setNamedTarget("vertical");
  move_group.move();

  move_group.setNamedTarget("home");
  move_group.move();

  rclcpp::shutdown();
  return 0;
}
