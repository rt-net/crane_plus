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
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Use camera.'
        )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=get_package_share_directory(
            'crane_plus_moveit_config'
        ) + '/config/moveit.rviz',
        description='Set the path to rviz configuration file.',
        condition=UnlessCondition(LaunchConfiguration('use_camera')),
        )

    declare_rviz_config_camera = DeclareLaunchArgument(
        'rviz_config',
        default_value=get_package_share_directory(
            'crane_plus_examples'
        ) + '/launch/camera_example.rviz',
        description='Set the path to rviz configuration file.',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(
            get_package_share_directory('crane_plus_gazebo'), 'worlds',
            'table.sdf'),
        description='Set world name.'
        )

    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'GZ_SIM_RESOURCE_PATH': os.path.dirname(
                get_package_share_directory('crane_plus_description')) + ':' +
           os.path.join(get_package_share_directory('crane_plus_gazebo'),
                        'models'),
           }

    gui_config = os.path.join(
        get_package_share_directory('crane_plus_gazebo'), 'gui', 'gui.config')
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    gz_sim = ExecuteProcess(
            cmd=['gz sim -r',
                 LaunchConfiguration('world_name'),
                 '--gui-config',
                 gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    gz_sim_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'crane_plus',
                   '-z', '1.015',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_camera = LaunchConfiguration('use_camera')
    description_loader.use_gazebo = 'true'
    description_loader.gz_control_config_package = 'crane_plus_control'
    description_loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={
                'loaded_description': description,
                'rviz_config': LaunchConfiguration('rviz_config')
                }.items()
        )

    spawn_joint_state_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner '
                     'joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner '
                     'crane_plus_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner '
                     'crane_plus_gripper_controller'],
                shell=True,
                output='screen',
            )

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    'image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                ],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        declare_use_camera,
        declare_rviz_config,
        declare_rviz_config_camera,
        declare_world_name,
        gz_sim,
        gz_sim_spawn_entity,
        move_group,
        spawn_joint_state_controller,
        spawn_arm_controller,
        spawn_gripper_controller,
        bridge
    ])
