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
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_place_tf_node")
  {
    using namespace std::placeholders;
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);

    // IKの成功率を向上させるため、目標位置姿勢の許容範囲を小さく設定
    move_group_arm_->setGoalPositionTolerance(1e-5);
    move_group_arm_->setGoalOrientationTolerance(1e-4);

    // SRDFに定義されている"home"の姿勢にする
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();

    // 可動範囲を制限する
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

    // 待機姿勢
    control_arm(0.0, 0.0, 0.17, 0, 0, 0);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

private:
  void on_timer()
  {
    // target_0のtf位置姿勢を取得
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
    const std::chrono::nanoseconds FILTERING_TIME = 2s;
    const std::chrono::nanoseconds STOP_TIME_THRESHOLD = 3s;
    const float DISTANCE_THRESHOLD = 0.01;
    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);
    const auto TF_ELAPSED_TIME = now.nanoseconds() - tf.stamp_.time_since_epoch().count();
    const auto TF_STOP_TIME = now.nanoseconds() - tf_past_.stamp_.time_since_epoch().count();

    // 現在時刻から2秒以内に受け取ったtfを使用
    if (TF_ELAPSED_TIME < FILTERING_TIME.count()) {
      double tf_diff = (tf_past_.getOrigin() - tf.getOrigin()).length();
      // 把持対象の位置が停止していることを判定
      if (tf_diff < DISTANCE_THRESHOLD) {
        // 把持対象が3秒以上停止している場合ピッキング動作開始
        if (TF_STOP_TIME > STOP_TIME_THRESHOLD.count()) {
          picking(tf.getOrigin());
        }
      } else {
        tf_past_ = tf;
      }
    }
  }

  void picking(tf2::Vector3 target_position)
  {
    const double GRIPPER_DEFAULT = 0.0;
    const double GRIPPER_OPEN = angles::from_degrees(-30.0);
    const double GRIPPER_CLOSE = angles::from_degrees(10.0);

    // 何かを掴んでいた時のためにハンドを開く
    control_gripper(GRIPPER_OPEN);

    // ロボット座標系（2D）の原点から見た把持対象物への角度を計算
    double x = target_position.x();
    double y = target_position.y();
    double theta_rad = std::atan2(y, x);
    double theta_deg = theta_rad * 180.0 / 3.1415926535;

    // 把持対象物に正対する
    control_arm(0.0, 0.0, 0.17, 0, 90, theta_deg);

    // 掴みに行く
    const double GRIPPER_OFFSET = 0.13;
    double gripper_offset_x = GRIPPER_OFFSET * std::cos(theta_rad);
    double gripper_offset_y = GRIPPER_OFFSET * std::sin(theta_rad);
    if (!control_arm(x - gripper_offset_x, y - gripper_offset_y, 0.04, 0, 90, theta_deg)) {
      // アーム動作に失敗した時はpick_and_placeを中断して待機姿勢に戻る
      control_arm(0.0, 0.0, 0.17, 0, 0, 0);
      return;
    }

    // ハンドを閉じる
    control_gripper(GRIPPER_CLOSE);

    // 移動する
    control_arm(0.0, 0.0, 0.17, 0, 90, 0);

    // 下ろす
    control_arm(0.0, -0.15, 0.05, 0, 90, -90);

    // ハンドを開く
    control_gripper(GRIPPER_OPEN);

    // 少しだけハンドを持ち上げる
    control_arm(0.0, -0.15, 0.10, 0, 90, -90);

    // 待機姿勢に戻る
    control_arm(0.0, 0.0, 0.17, 0, 0, 0);

    // ハンドを閉じる
    control_gripper(GRIPPER_DEFAULT);
  }

  // グリッパ制御
  void control_gripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // アーム制御
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
    // アーム動作の成否を取得
    moveit::core::MoveItErrorCode result = move_group_arm_->move();
    return result.val == moveit::core::MoveItErrorCode::SUCCESS;
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

  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_place_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_arm_node,
    move_group_gripper_node);
  exec.add_node(pick_and_place_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
