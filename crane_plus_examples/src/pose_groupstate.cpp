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

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PoseGroupstateController : public rclcpp::Node
{
public:
  explicit PoseGroupstateController(const rclcpp::NodeOptions & node_options)
  : Node("pose_groupstate", node_options)
  {
  }

  // MoveGroupInterfaceはshared_from_this()を使うため、コンストラクタ後に呼び出す
  void initializeMoveGroup()
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm_tcp");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);      // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  }

  // SRDFに定義されている名前付きの姿勢に移動する
  void moveArmToNamedPose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

private:
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto arm_controller = std::make_shared<PoseGroupstateController>(node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([arm_controller]() {
      rclcpp::spin(arm_controller);
    });

  arm_controller->initializeMoveGroup();

  // SRDFに定義されている名前付き姿勢を順番に実行する
  arm_controller->moveArmToNamedPose("home");
  arm_controller->moveArmToNamedPose("vertical");
  arm_controller->moveArmToNamedPose("home");

  // 終了処理: rclcppを終了したのち、バックグラウンドスレッドを安全に回収する
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
