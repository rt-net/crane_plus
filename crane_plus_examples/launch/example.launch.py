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

from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declare_use_camera = DeclareLaunchArgument(
        'use_camera', default_value='false', description='Use camera.'
    )

    moveit_config = MoveItConfigsBuilder('crane_plus').to_moveit_configs()

    description_loader = RobotDescriptionLoader()
    description_loader.use_camera = LaunchConfiguration('use_camera')

    declare_example_name = DeclareLaunchArgument(
        'example',
        default_value='gripper_control',
        description=(
            'Set an example executable name: '
            '[gripper_control, pose_groupstate, joint_values, pick_and_place]'
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=('Set true when using the gazebo simulator.'),
    )

    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='crane_plus_examples',
        executable=LaunchConfiguration('example'),
        output='screen',
        parameters=[
            {'robot_description': description_loader.load()},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            declare_use_camera,
            declare_use_sim_time,
            SetParameter(
                name='use_sim_time', value=LaunchConfiguration('use_sim_time')
            ),
            declare_example_name,
            example_node,
        ]
    )
