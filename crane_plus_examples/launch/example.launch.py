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

import os

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Use camera.'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_camera = LaunchConfiguration('use_camera')

    robot_description_semantic_config = load_file(
        'crane_plus_moveit_config', 'config/crane_plus.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('crane_plus_moveit_config', 'config/kinematics.yaml')

    declare_example_name = DeclareLaunchArgument(
        'example', default_value='gripper_control',
        description=('Set an example executable name: '
                     '[gripper_control, pose_groupstate, joint_values, pick_and_place]')
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    example_node = Node(name=[LaunchConfiguration('example'), '_node'],
                        package='crane_plus_examples',
                        executable=LaunchConfiguration('example'),
                        output='screen',
                        parameters=[{'robot_description': description_loader.load()},
                                    robot_description_semantic,
                                    kinematics_yaml])

    return LaunchDescription([
        declare_use_camera,
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        declare_example_name,
        example_node
    ])
