# Copyright 2020 RT Corporation
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

import datetime

from crane_plus_examples_py.utils import euler_to_quaternion, plan_and_execute

import math

from geometry_msgs.msg import Pose

# moveit python library
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import Constraints, JointConstraint
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import numpy as np

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformListener, TransformStamped
from tf2_ros.buffer import Buffer


class PickAndPlaceTf(Node):
    def __init__(self, crane_plus):
        super().__init__('pick_and_place_tf_node')
        self.logger = get_logger('pick_and_place_tf')
        self.tf_past = TransformStamped()
        self.crane_plus = crane_plus
        self.crane_plus_arm = crane_plus.get_planning_component('arm')
        self.crane_plus_gripper = crane_plus.get_planning_component('gripper')
        # instantiate a RobotState instance using the current robot model
        self.robot_model = crane_plus.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        # planningのパラメータ設定
        # armのパラメータ設定用
        self.arm_plan_request_params = PlanRequestParameters(
            self.crane_plus,
            'ompl_rrtc',
        )
        # Set 0.0 ~ 1.0
        self.arm_plan_request_params.max_velocity_scaling_factor = 1.0

        # Set 0.0 ~ 1.0
        self.arm_plan_request_params.max_acceleration_scaling_factor = 1.0

        # gripperのパラメータ設定用
        self.gripper_plan_request_params = PlanRequestParameters(
            self.crane_plus,
            'ompl_rrtc',
        )
        # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_velocity_scaling_factor = 1.0

        # Set 0.0 ~ 1.0
        self.gripper_plan_request_params.max_acceleration_scaling_factor = 1.0

        # SRDFに定義されている'home'の姿勢にする
        self.crane_plus_arm.set_start_state_to_current_state()
        self.crane_plus_arm.set_goal_state(configuration_name='home')
        plan_and_execute(
            self.crane_plus,
            self.crane_plus_arm,
            # logger=None,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )

        # 可動範囲を制限する
        constraints = Constraints()
        constraints.name = 'arm_constraints'

        jointConstraint = JointConstraint()
        jointConstraint.joint_name = 'crane_plus_joint1'
        jointConstraint.position = 0.0
        jointConstraint.tolerance_above = math.radians(100)
        jointConstraint.tolerance_below = math.radians(100)
        jointConstraint.weight = 1.0
        constraints.joint_constraints.append(jointConstraint)

        jointConstraint.joint_name = 'crane_plus_joint3'
        jointConstraint.position = 0.0
        jointConstraint.tolerance_above = math.radians(0)
        jointConstraint.tolerance_below = math.radians(180)
        jointConstraint.weight = 1.0
        constraints.joint_constraints.append(jointConstraint)

        self.crane_plus_arm.set_path_constraints(constraints)

        # 待機姿勢
        self._control_arm(0.0, 0.0, 0.17, 0, 0, 0)

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        # target_0のtf位置姿勢を取得
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'target_0', Time())
        except tf2_ros.LookupException as ex:
            self.get_logger().info(
                f'Could not transform base_link to target: {ex}'
                )

        now = Time()
        FILTERING_TIME = datetime.timedelta(seconds=2)
        STOP_TIME_THRESHOLD = datetime.timedelta(seconds=3)
        DISTANCE_THRESHOLD = 0.01

        # 経過時間と停止時間を計算(nsec)
        # 経過時間
        TF_ELAPSED_TIME = now.nanoseconds - tf_msg.header.stamp.nanosec
        # 停止時間
        if self.tf_past is not None:
            TF_STOP_TIME = now.nanoseconds - self.tf_past.header.stamp.nanosec
        else:
            TF_STOP_TIME = now.nanoseconds

        # 現在時刻から2秒以内に受け取ったtfを使用 
        if TF_ELAPSED_TIME < FILTERING_TIME.total_seconds() * 1e9:
            tf_diff = np.sqrt((self.tf_past.transform.translation.x
                               - tf_msg.transform.translation.x) ** 2
                              + (self.tf_past.transform.translation.y
                              - tf_msg.transform.translation.y) ** 2
                              + (self.tf_past.transform.translation.z
                              - tf_msg.transform.translation.z) ** 2
                              )
            # 把持対象の位置が停止していることを判定
            if tf_diff < DISTANCE_THRESHOLD:
                # 把持対象が3秒以上停止している場合ピッキング動作開始
                if TF_STOP_TIME > STOP_TIME_THRESHOLD.total_seconds() * 1e9:
                    self._picking(tf_msg)
            else:
                self.tf_past = tf_msg

    def _picking(self, tf_msg):
        GRIPPER_DEFAULT = 0.0
        GRIPPER_OPEN = math.radians(-30.0)
        GRIPPER_CLOSE = math.radians(10.0)

        # 何かを掴んでいた時のためにハンドを開く
        self._control_gripper(GRIPPER_OPEN)

        # ロボット座標系（2D）の原点から見た把持対象物への角度を計算
        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        theta_rad = math.atan2(y, x)
        theta_deg = math.degrees(theta_rad)

        # 把持対象物に正対する
        self._control_arm(0.0, 0.0, 0.17, 0, 90, theta_deg)

        # 掴みに行く
        GRIPPER_OFFSET = 0.13
        gripper_offset_x = GRIPPER_OFFSET * math.cos(theta_rad)
        gripper_offset_y = GRIPPER_OFFSET * math.sin(theta_rad)
        if not self._control_arm(x - gripper_offset_x, y - gripper_offset_y,
                                 0.05, 0, 90, theta_deg):
            # アーム動作に失敗した時はpick_and_placeを中断して待機姿勢に戻る
            self._control_arm(0.0, 0.0, 0.17, 0, 0, 0)
            return

        # ハンドを閉じる
        self._control_gripper(GRIPPER_CLOSE)

        # 移動する
        self._control_arm(0.0, 0.0, 0.17, 0, 90, 0)

        # 下ろす
        self._control_arm(0.0, -0.15, 0.06, 0, 90, -90)

        # ハンドを開く
        self._control_gripper(GRIPPER_OPEN)

        # 少しだけハンドを持ち上げる
        self._control_arm(0.0, -0.15, 0.10, 0, 90, -90)

        # 待機姿勢に戻る
        self._control_arm(0.0, 0.0, 0.17, 0, 0, 0)

        # ハンドを閉じる
        self._control_gripper(GRIPPER_DEFAULT)

    # グリッパ制御
    def _control_gripper(self, angle):
        self.robot_state.set_joint_group_positions('gripper', [angle])
        self.crane_plus_gripper.set_start_state_to_current_state()
        self.crane_plus_gripper.set_goal_state(robot_state=self.robot_state)
        plan_and_execute(
            self.crane_plus,
            self.crane_plus_gripper,
            # logger=None,
            self.logger,
            single_plan_parameters=self.gripper_plan_request_params,
        )

    # アーム制御
    def _control_arm(self, x, y, z, roll, pitch, yaw):
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        q = euler_to_quaternion(math.radians(roll), math.radians(pitch),
                                math.radians(yaw))
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        self.crane_plus_arm.set_start_state_to_current_state()
        self.crane_plus_arm.set_goal_state(
            pose_stamped_msg=target_pose, pose_link='crane_plus_link4'
        )
        result = plan_and_execute(
            self.crane_plus,
            self.crane_plus_arm,
            # logger=None,
            self.logger,
            single_plan_parameters=self.arm_plan_request_params,
        )
        return result


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # MoveItPy初期化
    crane_plus = MoveItPy(node_name='moveit_py')

    # node生成
    pick_and_place_tf_node = PickAndPlaceTf(crane_plus)
    rclpy.spin(pick_and_place_tf_node)

    # MoveItPyの終了
    crane_plus.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == '__main__':
    main()
