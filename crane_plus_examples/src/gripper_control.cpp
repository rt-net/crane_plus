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
#include <thread>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GripperController : public rclcpp::Node
{
public:
  explicit GripperController(const rclcpp::NodeOptions & node_options)
  : Node("gripper_control", node_options)
  {
  }

  // MoveGroupInterfaceはshared_from_this()を使うため、コンストラクタ後に呼び出す
  void initializeMoveGroup()
  {
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);      // Set 0.0 ~ 1.0
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  }

  // グリッパの開閉角度を設定して動かす
  void moveGripperAngle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
};

double toRadians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto gripper_controller = std::make_shared<GripperController>(node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([gripper_controller]() {
      rclcpp::spin(gripper_controller);
    });

  gripper_controller->initializeMoveGroup();

  // グリッパを閉じる
  gripper_controller->moveGripperAngle(toRadians(30.0));

  // グリッパを開く
  gripper_controller->moveGripperAngle(toRadians(-30.0));

  // グリッパを0度にする
  gripper_controller->moveGripperAngle(toRadians(0.0));

  // 終了処理: rclcppを終了したのち、バックグラウンドスレッドを安全に回収する
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
