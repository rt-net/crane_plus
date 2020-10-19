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
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveItErrorCode = moveit::planning_interface::MoveItErrorCode;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using RobotStatePtr = moveit::core::RobotStatePtr;
using JointModelGroup = const moveit::core::JointModelGroup;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_control");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

bool gripper_control(
  const double joint_angle,
  std::vector<double> & joint_group_positions, MoveGroupInterface & move_group)
{
  MoveGroupInterface::Plan my_plan;

  joint_group_positions[0] = joint_angle;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  if (move_group.plan(my_plan) != MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(LOGGER, "Plan failed.");
    return false;
  }

  if (move_group.move() != MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(LOGGER, "Move failed.");
    return true;
  }

  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("gripper_control", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  static const std::string PLANNING_GROUP = "gripper";
  MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  const JointModelGroup * joint_model_group =
    move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP);

  double wait_seconds = 10.0;
  RobotStatePtr current_state = move_group.getCurrentState(wait_seconds);
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  gripper_control(to_radians(30), joint_group_positions, move_group);
  gripper_control(to_radians(-30), joint_group_positions, move_group);
  gripper_control(to_radians(0), joint_group_positions, move_group);

  rclcpp::shutdown();
  return 0;
}
