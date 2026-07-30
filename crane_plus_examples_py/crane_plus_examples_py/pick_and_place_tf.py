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
    # グリッパの開閉角度
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = math.radians(-30.0)
    GRIPPER_CLOSE = math.radians(10.0)

    # 待機姿勢の位置姿勢（x, y, z [m], roll, pitch, yaw [deg]）
    STANDBY = (0.0, 0.0, 0.3, 0.0, 0.0, 0.0)

    # 把持アプローチ時の高さおよびピッチ角
    GRASP_HEIGHT = 0.04
    GRASP_PITCH = 90.0

    # 搬送時の中間姿勢
    TRANSIT_1 = (0.12, 0.0, 0.17, 0.0, 90.0, 0.0)
    TRANSIT_2 = (0.0, -0.12, 0.17, 0.0, 90.0, -90.0)

    # 置く位置（プレース位置）とその退避姿勢
    PLACE = (0.0, -0.25, 0.05, 0.0, 90.0, -90.0)
    PLACE_RETRACT = (0.0, -0.25, 0.10, 0.0, 90.0, -90.0)

    def __init__(self):
        super().__init__('pick_and_place_tf')
        self.logger = self.get_logger()

        # TFリスナーの初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_past = TransformStamped()

        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='moveit_py')
        self.logger.info('MoveItPy instance created')

        # アーム・グリッパ制御用 planning component
        self.arm = self.crane_plus.get_planning_component('arm_tcp')
        self.gripper = self.crane_plus.get_planning_component('gripper')

        # ロボットモデルの取得（ジョイント目標値の設定に使用）
        self.robot_model = self.crane_plus.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.gripper_plan_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.gripper_plan_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.gripper_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        # SRDFに定義されている "home" の姿勢に移動
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_and_execute(
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

        # アームの可動範囲制限を設定してから待機姿勢に移動する
        self.set_constraints()
        self.control_arm(*self.STANDBY)

        # 0.5秒ごとにon_timerを呼び出すタイマーを作成
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のtf位置姿勢を取得
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

        tf_time = rclpy.time.Time.from_msg(tf_msg.header.stamp)
        tf_elapsed_time = now_time - tf_time
        tf_past_time = rclpy.time.Time.from_msg(self.tf_past.header.stamp)
        tf_stop_time = now_time - tf_past_time

        # 現在時刻から2秒以内に受け取ったtfを使用
        if tf_elapsed_time > FILTERING_TIME:
            return

        tf_diff = np.linalg.norm(
            [
                self.tf_past.transform.translation.x - tf_msg.transform.translation.x,
                self.tf_past.transform.translation.y - tf_msg.transform.translation.y,
                self.tf_past.transform.translation.z - tf_msg.transform.translation.z,
            ]
        )

        # 把持対象の位置が停止していることを判定
        if tf_diff > DISTANCE_THRESHOLD:
            self.tf_past = tf_msg
            return

        # 把持対象が3秒以上停止している場合ピッキング動作開始
        if tf_stop_time < STOP_TIME_THRESHOLD:
            return

        self.picking(tf_msg.transform.translation)

    def picking(self, target_position):
        # 何かを掴んでいた時のためにハンドを開閉
        self.move_gripper_angle(self.GRIPPER_OPEN)

        x = target_position.x
        y = target_position.y
        theta_deg = math.degrees(math.atan2(y, x))

        # ターゲットの正面に向ける
        self.control_arm(0.0, 0.0, self.STANDBY[2], 0, 0, theta_deg)

        # ピック動作（掴みに行く）
        if not self.control_arm(x, y, self.GRASP_HEIGHT, 0, self.GRASP_PITCH, theta_deg):
            # アーム動作に失敗した場合は待機姿勢に戻る
            self.control_arm(*self.STANDBY)
            return

        self.move_gripper_angle(self.GRIPPER_CLOSE)

        # プレース動作（移動して置く）
        self.control_arm(*self.TRANSIT_1)
        self.control_arm(*self.TRANSIT_2)
        self.control_arm(*self.PLACE)
        self.move_gripper_angle(self.GRIPPER_OPEN)

        # 待機姿勢に戻る
        self.control_arm(*self.PLACE_RETRACT)
        self.control_arm(*self.STANDBY)
        self.move_gripper_angle(self.GRIPPER_DEFAULT)

    def move_gripper_angle(self, angle):
        # グリッパを角度[rad]を指定して開閉する
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [angle])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_plus, self.gripper, self.logger,
            single_plan_parameters=self.gripper_plan_params,
        )

    def control_arm(self, x, y, z, roll, pitch, yaw):
        # アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
        # （IKの精度を向上させるため、位置・姿勢の許容範囲を設定）
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
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def set_constraints(self):
        # アームの関節の一部に可動制限を設定する
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

    def clear_constraints(self):
        # 設定された関節可動制限をクリアする
        self.arm.clear_path_constraints()


def main(args=None):
    rclpy.init(args=args)

    pick_and_place_tf_node = PickAndPlaceTf()

    rclpy.spin(pick_and_place_tf_node)

    pick_and_place_tf_node.clear_constraints()

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    pick_and_place_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
