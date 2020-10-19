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
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
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
    # planning_context
    xacro_file = os.path.join(get_package_share_directory('crane_plus_description'),
                              'urdf', 'crane_plus.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file(
        'crane_plus_moveit_config', 'config/crane_plus.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('crane_plus_moveit_config', 'config/kinematics.yaml')

    gripper_control_node = Node(name='gripper_control_node',
                                package='crane_plus_examples',
                                executable='gripper_control',
                                output='screen',
                                parameters=[robot_description,
                                            robot_description_semantic,
                                            kinematics_yaml])

    return LaunchDescription([gripper_control_node])
