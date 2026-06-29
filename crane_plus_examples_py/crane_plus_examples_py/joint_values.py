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

from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


class JointValuesController:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='joint_values')
        self.arm = self.crane_plus.get_planning_component('arm_tcp')
        self.robot_model = self.crane_plus.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        # プランニングパラメータの設定
        self.plan_request_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.plan_request_params.max_velocity_scaling_factor = 1.0      # Set 0.0 ~ 1.0
        self.plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.logger = get_logger('joint_values')

    def move_arm_to_named_pose(self, name):
        # SRDFに定義されている名前付きの姿勢に移動する
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=name)
        plan_and_execute(
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.plan_request_params,
        )

    def move_arm_to_joint_values(self, joint_values_dict):
        # 関節角度の辞書を指定してアームを動かす
        self.robot_state.joint_positions = joint_values_dict
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.crane_plus.get_robot_model().get_joint_model_group('arm_tcp'),
        )
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(
            self.crane_plus, self.arm, self.logger,
            single_plan_parameters=self.plan_request_params,
        )

    def get_current_joint_values(self, joint_names):
        # 現在の関節角度を名前付き辞書で取得する
        self.arm.set_start_state_to_current_state()
        current_state = self.arm.get_start_state()
        positions = current_state.get_joint_group_positions('arm_tcp')
        return dict(zip(joint_names, positions))


def main(args=None):
    rclpy.init(args=args)

    controller = JointValuesController()

    joint_names = [
        'crane_plus_joint1',
        'crane_plus_joint2',
        'crane_plus_joint3',
        'crane_plus_joint4',
    ]
    target_joint_value = math.radians(45.0)

    # verticalの姿勢から開始
    controller.move_arm_to_named_pose('vertical')

    # 各関節を順番に45度に動かす
    joint_values_dict = controller.get_current_joint_values(joint_names)
    for joint_name in joint_names:
        joint_values_dict[joint_name] = target_joint_value
        controller.move_arm_to_joint_values(joint_values_dict)

    # verticalの姿勢に戻る
    controller.move_arm_to_named_pose('vertical')

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
