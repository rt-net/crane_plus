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

#include <thread>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GripperControl
{
public:
  // グリッパの開閉角度
  inline static const double GRIPPER_DEFAULT = 0.0;
  inline static const double GRIPPER_OPEN = angles::from_degrees(-30.0);
  inline static const double GRIPPER_CLOSE = angles::from_degrees(30.0);

  // ノードを受け取り、グリッパのMoveGroupInterfaceを初期化する
  explicit GripperControl(rclcpp::Node::SharedPtr node)
  {
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  }

  // グリッパを角度[rad]を指定して開閉する
  void move_gripper_angle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("gripper_control", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  GripperControl controller(node);

  // グリッパを閉じる
  controller.move_gripper_angle(GripperControl::GRIPPER_CLOSE);

  // グリッパを開く
  controller.move_gripper_angle(GripperControl::GRIPPER_OPEN);

  // グリッパを0度にする
  controller.move_gripper_angle(GripperControl::GRIPPER_DEFAULT);

  // 終了処理: rclcppを終了したのち、バックグラウンドスレッドを安全に回収する
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
