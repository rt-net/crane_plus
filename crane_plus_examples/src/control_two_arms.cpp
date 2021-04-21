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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("control_two_arms");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arms_node = rclcpp::Node::make_shared("move_group_arms_node", node_options);
  auto move_group_right_arm_node = rclcpp::Node::make_shared("move_group_right_arm_node", node_options);
  auto move_group_left_arm_node = rclcpp::Node::make_shared("move_group_left_arm_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arms_node);
  executor.add_node(move_group_right_arm_node);
  executor.add_node(move_group_left_arm_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_both_arms(move_group_arms_node, "both_arms");
  move_group_both_arms.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_both_arms.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_right_arm(move_group_right_arm_node, "right_arm");
  move_group_right_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_right_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_left_arm(move_group_left_arm_node, "left_arm");
  move_group_left_arm.setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
  move_group_left_arm.setMaxAccelerationScalingFactor(0.5);  // Set 0.0 ~ 1.0


  move_group_both_arms.setNamedTarget("home");
  move_group_both_arms.move();


  // move_group_both_arms.setNamedTarget("home");
  // move_group_both_arms.move();

  // ----- Picking Preparation -----

  geometry_msgs::msg::Pose r_target_pose, l_target_pose;
  tf2::Quaternion q;
  r_target_pose.position.x = 0.0;
  r_target_pose.position.y = -0.2;
  r_target_pose.position.z = 0.24;

  l_target_pose.position.x = 0.0;
  l_target_pose.position.y = 0.2;
  l_target_pose.position.z = 0.14;
  q.setRPY(to_radians(0), to_radians(90), to_radians(0));
  r_target_pose.orientation = tf2::toMsg(q);
  l_target_pose.orientation = tf2::toMsg(q);

  move_group_both_arms.setPoseTarget(r_target_pose, "right_crane_plus_link4");
  move_group_both_arms.setPoseTarget(l_target_pose, "left_crane_plus_link4");
  move_group_both_arms.move();

  r_target_pose.position.z = 0.14;
  l_target_pose.position.z = 0.18;
  move_group_right_arm.setPoseTarget(r_target_pose);
  move_group_left_arm.setPoseTarget(l_target_pose);
  RCLCPP_INFO(LOGGER, "Before Move!");
  move_group_right_arm.move();
  move_group_left_arm.move();
  RCLCPP_INFO(LOGGER, "After Move!");

  r_target_pose.position.z = 0.24;
  l_target_pose.position.z = 0.14;
  move_group_right_arm.setPoseTarget(r_target_pose);
  move_group_left_arm.setPoseTarget(l_target_pose);
  RCLCPP_INFO(LOGGER, "Before AsyncMove!");
  move_group_right_arm.asyncMove();
  move_group_left_arm.asyncMove();
  RCLCPP_INFO(LOGGER, "After AsyncMove!");

  r_target_pose.position.z = 0.14;
  l_target_pose.position.z = 0.18;
  move_group_right_arm.setPoseTarget(r_target_pose);
  move_group_left_arm.setPoseTarget(l_target_pose);
  RCLCPP_INFO(LOGGER, "Before AsyncMove!");
  move_group_right_arm.asyncMove();
  move_group_left_arm.asyncMove();
  RCLCPP_INFO(LOGGER, "After AsyncMove!");

  move_group_both_arms.setNamedTarget("home");
  move_group_both_arms.move();

  rclcpp::shutdown();
  return 0;
}
