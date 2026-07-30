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
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlace
{
public:
  // グリッパの開閉角度
  inline static const double GRIPPER_DEFAULT = 0.0;
  inline static const double GRIPPER_OPEN = angles::from_degrees(-30.0);
  inline static const double GRIPPER_CLOSE = angles::from_degrees(10.0);

  // ノードを受け取り、アーム・グリッパのMoveGroupInterfaceを初期化する
  explicit PickAndPlace(rclcpp::Node::SharedPtr node)
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(node, "arm_tcp");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
    // IKの成功率を向上させるため、目標位置姿勢の許容範囲を小さく設定
    move_group_arm_->setGoalPositionTolerance(1e-5);
    move_group_arm_->setGoalOrientationTolerance(1e-4);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // アームを目標位置・姿勢（Pose）に動かす
  void move_arm_to_pose(const geometry_msgs::msg::Pose & pose)
  {
    move_group_arm_->setPoseTarget(pose);
    move_group_arm_->move();
  }

  // アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
  void control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);
    move_arm_to_pose(target_pose);
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
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_and_place", node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([node]() {rclcpp::spin(node);});

  PickAndPlace controller(node);

  // 物体上方のアプローチ位置のXYZ[m]とRPY[deg]
  const double APPROACH_X = 0.0;
  const double APPROACH_Y = -0.21;
  const double APPROACH_Z = 0.17;
  const double APPROACH_ROLL = 0.0;
  const double APPROACH_PITCH = 90.0;
  const double APPROACH_YAW = -90.0;

  // アプローチ・退避時の高さ
  const double LIFTING_HEIGHT = 0.05;

  // 掴む位置（ピック位置）のXYZ[m]とRPY[deg]
  const double PICK_X = 0.0;
  const double PICK_Y = -0.1;
  const double PICK_Z = 0.02;
  const double PICK_ROLL = 0.0;
  const double PICK_PITCH = 180.0;
  const double PICK_YAW = -90.0;

  // 置く位置（プレース位置）のXYZ[m]とRPY[deg]
  const double PLACE_X = 0.25;
  const double PLACE_Y = 0.0;
  const double PLACE_Z = 0.06;
  const double PLACE_ROLL = 0.0;
  const double PLACE_PITCH = 90.0;
  const double PLACE_YAW = 0.0;

  // 初期姿勢
  controller.move_arm_to_named_pose("vertical");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_DEFAULT);

  // ピック準備
  controller.move_arm_to_named_pose("home");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);
  // 物体上方へ移動
  controller.control_arm(APPROACH_X, APPROACH_Y, APPROACH_Z, APPROACH_ROLL, APPROACH_PITCH,
    APPROACH_YAW);
  // 物体直上まで降りる
  controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);

  // ピック動作
  // 掴む位置まで降りる
  controller.control_arm(PICK_X, PICK_Y, PICK_Z, PICK_ROLL, PICK_PITCH, PICK_YAW);
  // 掴む
  controller.move_gripper_angle(PickAndPlace::GRIPPER_CLOSE);
  // 持ち上げる
  controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW);

  // プレース動作
  // homeを経由してプレース位置へ移動
  controller.move_arm_to_named_pose("home");
  // 置く位置まで降ろす
  controller.control_arm(PLACE_X, PLACE_Y, PLACE_Z, PLACE_ROLL, PLACE_PITCH, PLACE_YAW);
  // 離す
  controller.move_gripper_angle(PickAndPlace::GRIPPER_OPEN);

  // 終了動作
  controller.move_arm_to_named_pose("home");
  controller.move_arm_to_named_pose("vertical");
  controller.move_gripper_angle(PickAndPlace::GRIPPER_DEFAULT);

  // 終了処理: rclcppを終了したのち、バックグラウンドスレッドを安全に回収する
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
