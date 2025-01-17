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
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation as R


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
    arm_plan_request_params.max_acceleration_scaling_factor = 1.0    # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 1.0    # Set 0.0 ~ 1.0

    gripper_plan_request_params.max_acceleration_scaling_factor = 1.0    # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor = 1.0    # Set 0.0 ~ 1.0

    # グリッパの開閉角
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = math.radians(-30.0)
    GRIPPER_CLOSE = math.radians(10.0)

    # 物体を掴む位置
    rightward = R.from_euler('xyz', [0.0, 90.0, -90.0], degrees=True)
    quat = rightward.as_quat()
    PRE_GRASP_POSE = Pose(position=Point(x=0.0, y=-0.09, z=0.17),
                          orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))

    downward = R.from_euler('xyz', [0.0, 180.0, -90.0], degrees=True)
    quat = downward.as_quat()
    GRASP_POSE = Pose(position=Point(x=0.0, y=-0.09, z=0.14),
                      orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))

    # 物体を置く位置
    forward = R.from_euler('xyz', [0.0, 90.0, 0.0], degrees=True)
    quat = forward.as_quat()
    RELEASE_POSE = Pose(position=Point(x=0.15, y=0.0, z=0.06),
                        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))

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
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_plus_base'
    goal_pose.pose = PRE_GRASP_POSE
    arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='crane_plus_link4')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # 掴みに行く
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_plus_base'
    goal_pose.pose = GRASP_POSE
    arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='crane_plus_link4')
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
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_plus_base'
    goal_pose.pose = PRE_GRASP_POSE
    arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='crane_plus_link4')
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
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'crane_plus_base'
    goal_pose.pose = RELEASE_POSE
    arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='crane_plus_link4')
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
