# Copyright 2025 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from crane_plus_examples_py.utils import plan_and_execute

from geometry_msgs.msg import PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)

import numpy as np

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException, TransformListener, TransformStamped
from tf2_ros.buffer import Buffer


class PickAndPlaceTf(Node):
    def __init__(self):
        super().__init__('pick_and_place_tf')
        self.logger = self.get_logger()

        # TFの受信に必要なBufferとListenerを初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # アームおよびグリッパ制御用のplanning componentを取得
        self.arm = self.crane_plus.get_planning_component('arm_tcp')
        self.gripper = self.crane_plus.get_planning_component('gripper')

        # ロボットモデルからRobotStateインスタンスを取得（グリッパの関節角設定等に使用）
        self.robot_model = self.crane_plus.get_robot_model()

        # 計画時のパラメータ（PlanRequestParameters）を設定
        self.arm_plan_request_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.gripper_plan_request_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')

        # アームとグリッパの最大速度・加速度スケーリングファクタを設定 (0.0 ~ 1.0)
        self.arm_plan_request_params.max_velocity_scaling_factor = 1.0
        self.arm_plan_request_params.max_acceleration_scaling_factor = 1.0

        self.gripper_plan_request_params.max_velocity_scaling_factor = 1.0
        self.gripper_plan_request_params.max_acceleration_scaling_factor = 1.0

        # グリッパの目標開閉角度を設定
        self.GRIPPER_DEFAULT = 0.0
        self.GRIPPER_OPEN = math.radians(-30.0)
        self.GRIPPER_CLOSE = math.radians(10.0)

        # 待機姿勢の位置姿勢（x, y, z [m], roll, pitch, yaw [deg]）
        self.STANDBY_POSITION = (0.0, 0.0, 0.3, 0.0, 0.0, 0.0)

        # 搬送時の中間姿勢およびプレース位置姿勢
        self.TRANSIT_POSE_1 = (0.12, 0.0, 0.17, 0.0, 90.0, 0.0)
        self.TRANSIT_POSE_2 = (0.0, -0.12, 0.17, 0.0, 90.0, -90.0)
        self.PLACE_POSE = (0.0, -0.25, 0.05, 0.0, 90.0, -90.0)
        self.PLACE_RETRACT_POSE = (0.0, -0.25, 0.10, 0.0, 90.0, -90.0)

        # 初期姿勢としてSRDFに定義されている 'home' の位置姿勢に移動
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_and_execute(
            self.crane_plus,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

        # アームの可動範囲制限（Path Constraints）を設定して、関節の動きを制限する
        constraints = Constraints()
        constraints.name = 'arm_constraints'

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'crane_plus_joint1'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(100)
        joint_constraint.tolerance_below = math.radians(100)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'crane_plus_joint3'
        joint_constraint.position = 0.0
        joint_constraint.tolerance_above = math.radians(0)
        joint_constraint.tolerance_below = math.radians(180)
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

        self.arm.set_path_constraints(constraints)

        # 初期位置としての待機姿勢に移動
        self.move_arm_to_pose(*self.STANDBY_POSITION)

        # 定期的にTFを監視してピッキングをトリガーするためのタイマーを設定（0.5秒周期）
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0（把持対象）のTFを取得
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'crane_plus_base', 'target_0', rclpy.time.Time()
            )
        except TransformException as ex:
            self.logger.info(f'Could not transform base_link to target: {ex}')
            return

        now_time = self.get_clock().now()
        FILTERING_TIME = rclpy.duration.Duration(seconds=2)
        STOP_TIME_THRESHOLD = rclpy.duration.Duration(seconds=3)
        DISTANCE_THRESHOLD = 0.01

        # ターゲットのTFが更新された時刻からの経過時間と、前回の位置から静止している時間を計算
        tf_time = rclpy.time.Time.from_msg(tf_msg.header.stamp)
        TF_ELAPSED_TIME = now_time - tf_time
        tf_past_time = rclpy.time.Time.from_msg(self.tf_past.header.stamp)
        TF_STOP_TIME = now_time - tf_past_time

        # 一定時間（2秒）以内に更新された有効なTFのみ処理する
        if TF_ELAPSED_TIME < FILTERING_TIME:
            tf_diff = np.linalg.norm(
                [
                    self.tf_past.transform.translation.x - tf_msg.transform.translation.x,
                    self.tf_past.transform.translation.y - tf_msg.transform.translation.y,
                    self.tf_past.transform.translation.z - tf_msg.transform.translation.z,
                ]
            )

            # 前回位置からの移動量が閾値未満（静止状態）であるかを判定
            if tf_diff < DISTANCE_THRESHOLD:
                # 一定時間（3秒）以上静止し続けている場合、ピッキング動作を開始
                if TF_STOP_TIME > STOP_TIME_THRESHOLD:
                    self.picking(tf_msg.transform.translation)
            else:
                # 移動した場合は過去のTF位置情報を更新
                self.tf_past = tf_msg

    def picking(self, target_position):
        # 1. 掴み動作の準備とターゲットへの正対
        self.move_gripper_angle(self.GRIPPER_OPEN)

        x = target_position.x
        y = target_position.y
        theta_rad = math.atan2(y, x)
        theta_deg = math.degrees(theta_rad)

        self.move_arm_to_pose(0.0, 0.0, 0.3, 0, 0, theta_deg)

        # 2. ターゲット位置へのアプローチと掴み動作
        if not self.move_arm_to_pose(x, y, 0.04, 0, 90.0, theta_deg):
            # アーム動作に失敗した場合は初期姿勢に戻る
            self.move_arm_to_pose(*self.STANDBY_POSITION)
            return

        self.move_gripper_angle(self.GRIPPER_CLOSE)

        # 3. 搬送および配置動作
        self.move_arm_to_pose(*self.TRANSIT_POSE_1)
        self.move_arm_to_pose(*self.TRANSIT_POSE_2)
        self.move_arm_to_pose(*self.PLACE_POSE)

        self.move_gripper_angle(self.GRIPPER_OPEN)

        # 4. 待機姿勢への復帰
        self.move_arm_to_pose(*self.PLACE_RETRACT_POSE)
        self.move_arm_to_pose(*self.STANDBY_POSITION)
        self.move_gripper_angle(self.GRIPPER_DEFAULT)

    def move_gripper_angle(self, angle):
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [angle])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_plus,
            self.gripper,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
        )

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        # 指定された位置姿勢（x, y, z [m]、roll, pitch, yaw [deg]）へアームを移動する
        POSITION_TOLERANCE = 0.00001
        ORIENTATION_TOLERANCE = 0.0001

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'crane_plus_base'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = rotation.as_quat()
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]

        goal_constraints = Constraints()
        goal_constraints.name = 'tolerance_goal'

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'crane_plus_base'
        position_constraint.link_name = 'crane_plus_link_tcp'
        tolerance_region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [POSITION_TOLERANCE]
        tolerance_region.primitives.append(primitive)
        tolerance_region.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = tolerance_region
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'crane_plus_base'
        orientation_constraint.link_name = 'crane_plus_link_tcp'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.weight = 1.0

        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[goal_constraints])

        return plan_and_execute(
            self.crane_plus,
            self.arm,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )


def main(args=None):
    rclpy.init(args=args)

    pick_and_place_tf_node = PickAndPlaceTf()

    rclpy.spin(pick_and_place_tf_node)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
