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

import copy
import math

from crane_plus_examples_py.utils import plan_and_execute

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

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


class PickAndPlaceController:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='pick_and_place')
        self.arm = self.crane_plus.get_planning_component('arm_tcp')
        self.gripper = self.crane_plus.get_planning_component('gripper')
        self.robot_model = self.crane_plus.get_robot_model()

        # プランニングパラメータの設定
        self.arm_plan_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.arm_plan_params.max_velocity_scaling_factor = 1.0      # Set 0.0 ~ 1.0
        self.arm_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.gripper_plan_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.gripper_plan_params.max_velocity_scaling_factor = 1.0      # Set 0.0 ~ 1.0
        self.gripper_plan_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.logger = get_logger('pick_and_place')

        # グリッパの開閉角
        self.GRIPPER_DEFAULT = 0.0
        self.GRIPPER_OPEN = math.radians(-30.0)
        self.GRIPPER_CLOSE = math.radians(10.0)

        # 物体を持ち上げる高さ
        LIFTING_HEIGHT = 0.03
        # 目標位置姿勢の定数
        self.ABOVE_POSE = self._make_pose(0.0, -0.21, 0.17, 0.0, 90.0, -90.0)
        self.GRASP_POSE = self._make_pose(0.0, -0.1, 0.02, 0.0, 180.0, -90.0)
        pre_post = copy.deepcopy(self.GRASP_POSE)
        pre_post.position.z = LIFTING_HEIGHT
        self.PRE_AND_POST_GRASP_POSE = pre_post
        self.RELEASE_POSE = self._make_pose(0.25, 0.0, 0.06, 0.0, 90.0, 0.0)

    @staticmethod
    def _make_pose(x, y, z, roll, pitch, yaw):
        # x, y, z[m]とroll, pitch, yaw[deg]からPoseを生成する
        quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        return Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )

    @staticmethod
    def _make_goal_constraints(pose):
        # 位置・姿勢の目標制約を生成する
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
        return goal_constraints

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
        goal_constraints = self._make_goal_constraints(pose)
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[goal_constraints])
        plan_and_execute(
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.arm_plan_params,
        )

    def set_gripper_angle(self, angle):
        # グリッパの目標角度を設定して動作計画・実行する
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

    controller = PickAndPlaceController()

    # 初期姿勢
    controller.move_arm_to_named_pose('vertical')
    controller.set_gripper_angle(controller.GRIPPER_DEFAULT)

    # ピック準備
    controller.move_arm_to_named_pose('home')
    controller.set_gripper_angle(controller.GRIPPER_OPEN)
    controller.move_arm_to_pose(controller.ABOVE_POSE)
    controller.move_arm_to_pose(controller.PRE_AND_POST_GRASP_POSE)

    # ピック動作
    controller.move_arm_to_pose(controller.GRASP_POSE)
    controller.set_gripper_angle(controller.GRIPPER_CLOSE)
    controller.move_arm_to_pose(controller.PRE_AND_POST_GRASP_POSE)

    # プレース準備
    controller.move_arm_to_named_pose('home')

    # プレース動作
    controller.move_arm_to_pose(controller.RELEASE_POSE)
    controller.set_gripper_angle(controller.GRIPPER_OPEN)

    # 終了動作
    controller.move_arm_to_named_pose('home')
    controller.move_arm_to_named_pose('vertical')
    controller.set_gripper_angle(controller.GRIPPER_DEFAULT)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
