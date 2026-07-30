// Copyright 2022 RT Corporation
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
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html

#include <chrono>
#include <cmath>
#include <memory>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  // 位置姿勢パラメータ（x, y, z [m], roll, pitch, yaw [deg]）
  struct PoseParams
  {
    double x, y, z, roll, pitch, yaw;
  };

  // グリッパの開閉角度
  inline static const double GRIPPER_DEFAULT = 0.0;
  inline static const double GRIPPER_OPEN = angles::from_degrees(-30.0);
  inline static const double GRIPPER_CLOSE = angles::from_degrees(10.0);

  // 待機姿勢の位置姿勢（x, y, z [m], roll, pitch, yaw [deg]）
  inline static const PoseParams STANDBY = {0.0, 0.0, 0.3, 0.0, 0.0, 0.0};

  // 把持アプローチ時の高さおよびピッチ角
  inline static const double GRASP_HEIGHT = 0.04;
  inline static const double GRASP_PITCH = 90.0;

  // 搬送時の中間姿勢
  inline static const PoseParams TRANSIT_1 = {0.12, 0.0, 0.17, 0.0, 90.0, 0.0};
  inline static const PoseParams TRANSIT_2 = {0.0, -0.12, 0.17, 0.0, 90.0, -90.0};

  // 置く位置（プレース位置）とその退避姿勢
  inline static const PoseParams PLACE = {0.0, -0.25, 0.05, 0.0, 90.0, -90.0};
  inline static const PoseParams PLACE_RETRACT = {0.0, -0.25, 0.10, 0.0, 90.0, -90.0};

  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm_tcp");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);

    // IKの成功率を向上させるため、目標位置姿勢の許容範囲を小さく設定
    move_group_arm_->setGoalPositionTolerance(1e-5);
    move_group_arm_->setGoalOrientationTolerance(1e-4);

    // SRDFに定義されている "home" の姿勢に移動
    move_arm_to_named_pose("home");

    // アームの可動範囲制限を設定してから待機姿勢に移動する
    set_constraints();
    control_arm(STANDBY);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 0.5秒ごとにon_timerを呼び出すタイマーを作成
    timer_ = this->create_wall_timer(500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

  // グリッパを角度[rad]を指定して開閉する
  void move_gripper_angle(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // アームを目標位置・姿勢（Pose）に動かす
  void move_arm_to_pose(const geometry_msgs::msg::Pose & pose)
  {
    move_group_arm_->setPoseTarget(pose);
    move_group_arm_->move();
  }

  // アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
  bool control_arm(
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
    move_group_arm_->setPoseTarget(target_pose);
    moveit::core::MoveItErrorCode result = move_group_arm_->move();
    return result.val == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // 位置姿勢パラメータ（PoseParams）を使ってアームを動かす
  bool control_arm(const PoseParams & pose)
  {
    return control_arm(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
  }

  // SRDFに定義された姿勢名でアームを動かす
  void move_arm_to_named_pose(const std::string & name)
  {
    move_group_arm_->setNamedTarget(name);
    move_group_arm_->move();
  }

  // アームの関節の一部に可動制限を設定する
  void set_constraints()
  {
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "crane_plus_joint1";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(100);
    joint_constraint.tolerance_below = angles::from_degrees(100);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "crane_plus_joint3";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(0);
    joint_constraint.tolerance_below = angles::from_degrees(180);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    move_group_arm_->setPathConstraints(constraints);
  }

  // 設定された関節可動制限をクリアする
  void clear_constraints()
  {
    move_group_arm_->clearPathConstraints();
  }

private:
  void on_timer()
  {
    // target_0（把持対象）のTFを取得
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", "target_0",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to target: %s",
        ex.what());
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const auto FILTERING_TIME = rclcpp::Duration(2s);
    const auto STOP_TIME_THRESHOLD = rclcpp::Duration(3s);
    const double DISTANCE_THRESHOLD = 0.01;
    const double TARGET_Z_MIN_LIMIT = 0.04;

    tf2::Stamped<tf2::Transform> tf_current;
    tf2::convert(tf_msg, tf_current);

    const auto tf_elapsed_time = now - rclcpp::Time(tf_msg.header.stamp, RCL_ROS_TIME);
    const auto tf_stop_time =
      now - rclcpp::Time(tf_past_.stamp_.time_since_epoch().count(), RCL_ROS_TIME);

    // 現在時刻から2秒以内に受け取ったtfを使用
    if (tf_elapsed_time > FILTERING_TIME) {
      return;
    }

    double tf_diff = (tf_past_.getOrigin() - tf_current.getOrigin()).length();

    // 把持対象の位置が停止していることを判定
    if (tf_diff > DISTANCE_THRESHOLD) {
      tf_past_ = tf_current;
      return;
    }

    // 把持対象が3秒以上停止している場合ピッキング動作開始
    if (tf_stop_time < STOP_TIME_THRESHOLD) {
      return;
    }

    // 把持対象が低すぎる場合は把持位置を調整
    if (tf_current.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
      tf_current.getOrigin().setZ(TARGET_Z_MIN_LIMIT);
    }

    picking(tf_current.getOrigin());
  }

  void picking(tf2::Vector3 target_position)
  {
    // 何かを掴んでいた時のためにハンドを開閉
    move_gripper_angle(GRIPPER_OPEN);

    double x = target_position.x();
    double y = target_position.y();
    double theta_deg = std::atan2(y, x) * 180.0 / M_PI;

    // ターゲットの正面に向ける
    control_arm(0.0, 0.0, STANDBY.z, 0, 0, theta_deg);

    // ピック動作（掴みに行く）
    if (!control_arm(x, y, GRASP_HEIGHT, 0, GRASP_PITCH, theta_deg)) {
      // アーム動作に失敗した場合は待機姿勢に戻る
      control_arm(STANDBY);
      return;
    }
    move_gripper_angle(GRIPPER_CLOSE);

    // プレース動作（移動して置く）
    control_arm(TRANSIT_1);
    control_arm(TRANSIT_2);
    control_arm(PLACE);
    move_gripper_angle(GRIPPER_OPEN);

    // 待機姿勢に戻る
    control_arm(PLACE_RETRACT);
    control_arm(STANDBY);
    move_gripper_angle(GRIPPER_DEFAULT);
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // タイマーとTFリスナーを持つため、MultiThreadedExecutorを使用する
  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_place_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_arm_node,
    move_group_gripper_node);

  exec.add_node(pick_and_place_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();

  pick_and_place_tf_node->clear_constraints();
  rclcpp::shutdown();
  return 0;
}
