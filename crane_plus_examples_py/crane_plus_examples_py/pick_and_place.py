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
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
import rclpy
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive


def set_goal_constraints(x, y, z, roll, pitch, yaw):
    # 位置姿勢の許容誤差
    POSITION_TOLERANCE = 0.00001
    ORIENTATION_TOLERANCE = 0.0001

    # 目標位置姿勢
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

    # 目標位置姿勢の制約設定
    goal_constraints = Constraints()
    goal_constraints.name = 'tolerance_goal'

    # 位置の制約設定
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = 'crane_plus_base'
    position_constraint.link_name = 'crane_plus_link4'
    tolerance_region = BoundingVolume()
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [POSITION_TOLERANCE]
    tolerance_region.primitives.append(primitive)
    tolerance_region.primitive_poses.append(target_pose.pose)
    position_constraint.constraint_region = tolerance_region
    position_constraint.weight = 1.0

    # 姿勢の制約設定
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = 'crane_plus_base'
    orientation_constraint.link_name = 'crane_plus_link4'
    orientation_constraint.orientation = target_pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
    orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
    orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
    orientation_constraint.weight = 1.0

    goal_constraints.position_constraints.append(position_constraint)
    goal_constraints.orientation_constraints.append(orientation_constraint)

    # 制約設定を返す
    return goal_constraints


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('pick_and_place')

    # instantiate MoveItPy instance and get planning component
    crane_plus = MoveItPy(node_name='pick_and_place')
    logger.info('MoveItPy instance created')

    # アーム制御用 planning component
    arm = crane_plus.get_planning_component('arm')
    # グリッパ制御用 planning component
    gripper = crane_plus.get_planning_component('gripper')

    robot_model = crane_plus.get_robot_model()

    arm_plan_request_params = PlanRequestParameters(
        crane_plus,
        'ompl_rrtc',
    )
    gripper_plan_request_params = PlanRequestParameters(
        crane_plus,
        'ompl_rrtc',
    )

    # 動作速度の調整
    arm_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    gripper_plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # グリッパの開閉角
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = math.radians(-30.0)
    GRIPPER_CLOSE = math.radians(10.0)

    # 物体を置く位置

    # SRDF内に定義されている'vertical'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperの開閉角を0度にする
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_DEFAULT])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # SRDF内に定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperを開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_OPEN])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 物体の上に腕を伸ばす
    arm.set_start_state_to_current_state()
    goal_constraints = set_goal_constraints(0.0, -0.09, 0.17, 0.0, 90.0, -90.0)
    arm.set_goal_state(motion_plan_constraints=[goal_constraints])
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 掴みに行く
    arm.set_start_state_to_current_state()
    goal_constraints = set_goal_constraints(0.0, -0.09, 0.14, 0.0, 180.0, -90.0)
    arm.set_goal_state(motion_plan_constraints=[goal_constraints])
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # ハンドを閉じる
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_CLOSE])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 持ち上げる
    arm.set_start_state_to_current_state()
    goal_constraints = set_goal_constraints(0.0, -0.09, 0.17, 0.0, 90.0, -90.0)
    arm.set_goal_state(motion_plan_constraints=[goal_constraints])
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # SRDF内に定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 下ろす
    arm.set_start_state_to_current_state()
    goal_constraints = set_goal_constraints(0.15, 0.0, 0.06, 0.0, 90.0, 0.0)
    arm.set_goal_state(motion_plan_constraints=[goal_constraints])
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # ハンドを開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_OPEN])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # SRDF内に定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # SRDF内に定義されている'vertical'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperの開閉角を0度にする
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [GRIPPER_DEFAULT])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
