# Copyright 2022 RT Corporation
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
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # world_file = os.path.join(get_package_share_directory(
    #     'crane_plus_gazebo'), 'worlds', 'table.world')

    # declare_arg_gui = DeclareLaunchArgument(
    #     'gui',
    #     default_value='true',
    #     description='Set to "false" to run headless.')

    # declare_arg_server = DeclareLaunchArgument(
    #     'server',
    #     default_value='true',
    #     description='Set to "false" not to run gzserver.')

    # xacro_file = os.path.join(get_package_share_directory('crane_plus_description'),
    #                           'urdf', 'crane_plus.urdf.xacro')
    # robot_description = {'robot_description': Command(
    #     ['xacro ', xacro_file, ' use_gazebo:=', 'false'])}

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[robot_description]
    # )

    # launch_ignition = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             [os.path.join(get_package_share_directory('ros_ign_gazebo'),
    #                           'launch', 'ign_gazebo.launch.py')]),
    #         launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])])
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(get_package_share_directory('crane_plus_description'))}

    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo', 'empty.sdf'],
            output='screen',
            additional_env=env,
            shell=True
        )
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'crane_plus',
                   '-allow_renaming', 'true'],
    )

    # gzserver = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory('gazebo_ros'),
    #             '/launch/gzserver.launch.py']),
    #         condition=IfCondition(LaunchConfiguration('server')),
    #         launch_arguments={'world': world_file}.items(),
    #     )

    # gzclient = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory('gazebo_ros'),
    #             '/launch/gzclient.launch.py']),
    #         condition=IfCondition(LaunchConfiguration('gui'))
    #     )

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={'xacro_use_gazebo': 'true'}.items(),
        )

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-entity', 'crane_plus', '-x', '0', '-y', '0',
    #                                '-z', '1.02', '-topic', '/robot_description'],
    #                     output='screen')

    # spawn_joint_state_controller = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner.py joint_state_controller'],
    #             shell=True,
    #             output='screen',
    #         )

    # spawn_arm_controller = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner.py crane_plus_arm_controller'],
    #             shell=True,
    #             output='screen',
    #         )

    # spawn_gripper_controller = ExecuteProcess(
    #             cmd=['ros2 run controller_manager spawner.py crane_plus_gripper_controller'],
    #             shell=True,
    #             output='screen',
    #         )

    return LaunchDescription([
        # launch_ignition,
        ign_gazebo,
        move_group,
        ignition_spawn_entity
        # node_robot_state_publisher
        # declare_arg_gui,
        # declare_arg_server,
        # gzserver,
        # gzclient,
        # move_group,
        # spawn_entity,
        # spawn_joint_state_controller,
        # spawn_arm_controller,
        # spawn_gripper_controller
    ])
