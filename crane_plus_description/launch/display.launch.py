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


from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_camera = DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            description='Set true to attach the camera model.'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_camera = LaunchConfiguration('use_camera')

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': description_loader.load()}])
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config_file = get_package_share_directory(
        'crane_plus_description') + '/launch/display.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file])

    ld = LaunchDescription()
    ld.add_action(declare_use_camera)
    ld.add_action(rsp)
    ld.add_action(jsp)
    ld.add_action(rviz_node)
    return ld
