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

from geometry_msgs.msg import Pose, PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint

import rclpy
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive


class PickAndPlace:
    # グリッパの開閉角度
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = math.radians(-30.0)
    GRIPPER_CLOSE = math.radians(10.0)

    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='pick_and_place')
        self.arm = self.crane_plus.get_planning_component('arm_tcp')
        self.gripper = self.crane_plus.get_planning_component('gripper')
        self.robot_model = self.crane_plus.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.arm_plan_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.gripper_plan_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.gripper_plan_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.gripper_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.logger = get_logger('pick_and_place')

    def move_arm_to_named_pose(self, name):
        # SRDFに定義されている名前付きの姿勢に移動する
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=name)
        plan_and_execute(
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def move_arm_to_pose(self, pose):
        # 位置姿勢の制約を設定してアームを動かす
        # （IKの精度を向上させるため、位置・姿勢の許容範囲を設定）
        POSITION_TOLERANCE = 0.00001
        ORIENTATION_TOLERANCE = 0.0001

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'crane_plus_base'
        target_pose.pose = pose

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
        plan_and_execute(
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def control_arm(self, x, y, z, roll, pitch, yaw):
        # アームを目標位置（x, y, z [m]）・姿勢（roll, pitch, yaw [deg]）に動かす
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.move_arm_to_pose(pose)

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


def main(args=None):
    rclpy.init(args=args)

    controller = PickAndPlace()

    # 物体上方のアプローチ位置のXYZ[m]とRPY[deg]
    APPROACH_X = 0.0
    APPROACH_Y = -0.21
    APPROACH_Z = 0.17
    APPROACH_ROLL = 0.0
    APPROACH_PITCH = 90.0
    APPROACH_YAW = -90.0

    # アプローチ・退避時の高さ
    LIFTING_HEIGHT = 0.05

    # 掴む位置（ピック位置）のXYZ[m]とRPY[deg]
    PICK_X = 0.0
    PICK_Y = -0.1
    PICK_Z = 0.02
    PICK_ROLL = 0.0
    PICK_PITCH = 180.0
    PICK_YAW = -90.0

    # 置く位置（プレース位置）のXYZ[m]とRPY[deg]
    PLACE_X = 0.25
    PLACE_Y = 0.0
    PLACE_Z = 0.06
    PLACE_ROLL = 0.0
    PLACE_PITCH = 90.0
    PLACE_YAW = 0.0

    # 初期姿勢
    controller.move_arm_to_named_pose('vertical')
    controller.move_gripper_angle(PickAndPlace.GRIPPER_DEFAULT)

    # ピック準備
    controller.move_arm_to_named_pose('home')
    controller.move_gripper_angle(PickAndPlace.GRIPPER_OPEN)
    # 物体上方へ移動
    controller.control_arm(
        APPROACH_X, APPROACH_Y, APPROACH_Z, APPROACH_ROLL, APPROACH_PITCH, APPROACH_YAW
    )
    # 物体直上まで降りる
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW)

    # ピック動作
    # 掴む位置まで降りる
    controller.control_arm(PICK_X, PICK_Y, PICK_Z, PICK_ROLL, PICK_PITCH, PICK_YAW)
    # 掴む
    controller.move_gripper_angle(PickAndPlace.GRIPPER_CLOSE)
    # 持ち上げる
    controller.control_arm(PICK_X, PICK_Y, LIFTING_HEIGHT, PICK_ROLL, PICK_PITCH, PICK_YAW)

    # プレース動作
    # homeを経由してプレース位置へ移動
    controller.move_arm_to_named_pose('home')
    # 置く位置まで降ろす
    controller.control_arm(PLACE_X, PLACE_Y, PLACE_Z, PLACE_ROLL, PLACE_PITCH, PLACE_YAW)
    # 離す
    controller.move_gripper_angle(PickAndPlace.GRIPPER_OPEN)

    # 終了動作
    controller.move_arm_to_named_pose('home')
    controller.move_arm_to_named_pose('vertical')
    controller.move_gripper_angle(PickAndPlace.GRIPPER_DEFAULT)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
