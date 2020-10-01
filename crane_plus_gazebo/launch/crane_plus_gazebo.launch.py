
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('crane_plus_gazebo'),
        'worlds', 'table.world')
    xacro_file = os.path.join(get_package_share_directory('crane_plus_description'),
        'urdf', 'crane_plus.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toprettyxml(indent='  ')

    declare_arg_gui = DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.')

    declare_arg_server = DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.')

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'), '/launch/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={'world': world_file}.items(),
        )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'), '/launch/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        )

    rsp = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[{'robot_description' : robot_desc}])

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'demo', '-x', '0', '-y', '0', '-z', '1.02', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([
        declare_arg_gui,
        declare_arg_server,
        gzserver,
        gzclient,
        rsp,
        spawn_entity,
    ])
