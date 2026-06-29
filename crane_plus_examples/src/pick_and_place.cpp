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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceController : public rclcpp::Node
{
public:
  explicit PickAndPlaceController(const rclcpp::NodeOptions & node_options)
  : Node("pick_and_place", node_options)
  {
    // グリッパの開閉角
    GRIPPER_DEFAULT = 0.0;
    GRIPPER_OPEN = toRadians(-30.0);
    GRIPPER_CLOSE = toRadians(10.0);

    // 物体の頭上の位置
    ABOVE_POSE = makePose(0.0, -0.21, 0.17, 0.0, 90.0, -90.0);
    // 物体を掴む直前・直後の位置（持ち上げた高さ）
    PRE_AND_POST_GRASP_POSE = makePose(0.0, -0.1, 0.05, 0.0, 180.0, -90.0);
    // 物体を掴む位置
    GRASP_POSE = makePose(0.0, -0.1, 0.02, 0.0, 180.0, -90.0);
    // 物体を置く位置
    RELEASE_POSE = makePose(0.25, 0.0, 0.06, 0.0, 90.0, 0.0);
  }

  // MoveGroupInterfaceはshared_from_this()を使うため、コンストラクタ後に呼び出す
  void initializeMoveGroup()
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "arm_tcp");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);      // Set 0.0 ~ 1.0
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
    // IKの成功率を向上させるため、目標位置姿勢の許容範囲を小さく設定
    move_group_arm_->setGoalPositionTolerance(1e-5);
    move_group_arm_->setGoalOrientationTolerance(1e-4);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);      // Set 0.0 ~ 1.0
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  }

  // SRDFに定義されている名前付きの姿勢に移動する
  void moveArmToNamedPose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // 指定した位置姿勢にアームを動かす
  void moveArmToPose(const geometry_msgs::msg::Pose & pose)
  {
    move_group_arm_->setPoseTarget(pose);
    move_group_arm_->move();
  }

  // グリッパの開閉角度を設定して動かす
  void setGripperAngle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // 目標位置姿勢の定数
  geometry_msgs::msg::Pose ABOVE_POSE;
  geometry_msgs::msg::Pose PRE_AND_POST_GRASP_POSE;
  geometry_msgs::msg::Pose GRASP_POSE;
  geometry_msgs::msg::Pose RELEASE_POSE;

  // グリッパ角度の定数
  double GRIPPER_DEFAULT;
  double GRIPPER_OPEN;
  double GRIPPER_CLOSE;

private:
  // x, y, z[m]とroll, pitch, yaw[deg]からPoseを生成する
  static geometry_msgs::msg::Pose makePose(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(toRadians(roll), toRadians(pitch), toRadians(yaw));
    pose.orientation = tf2::toMsg(q);
    return pose;
  }

  static double toRadians(const double deg_angle)
  {
    return deg_angle * M_PI / 180.0;
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto controller = std::make_shared<PickAndPlaceController>(node_options);

  // MoveGroupInterfaceのデッドロックを防ぐため、スピン処理を別スレッドで走らせる
  std::thread spin_thread([controller]() {
      rclcpp::spin(controller);
    });

  controller->initializeMoveGroup();

  // 初期姿勢
  controller->moveArmToNamedPose("vertical");
  controller->setGripperAngle(controller->GRIPPER_DEFAULT);

  // ピック準備
  controller->moveArmToNamedPose("home");
  controller->setGripperAngle(controller->GRIPPER_OPEN);
  controller->moveArmToPose(controller->ABOVE_POSE);
  controller->moveArmToPose(controller->PRE_AND_POST_GRASP_POSE);

  // ピック動作
  controller->moveArmToPose(controller->GRASP_POSE);
  controller->setGripperAngle(controller->GRIPPER_CLOSE);
  controller->moveArmToPose(controller->PRE_AND_POST_GRASP_POSE);

  // プレース準備
  controller->moveArmToNamedPose("home");

  // プレース動作
  controller->moveArmToPose(controller->RELEASE_POSE);
  controller->setGripperAngle(controller->GRIPPER_OPEN);

  // 終了動作
  controller->moveArmToNamedPose("home");
  controller->moveArmToNamedPose("vertical");
  controller->setGripperAngle(controller->GRIPPER_DEFAULT);

  // 終了処理: rclcppを終了したのち、バックグラウンドスレッドを安全に回収する
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
