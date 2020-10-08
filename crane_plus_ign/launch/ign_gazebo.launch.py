
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('crane_plus_ign'),
        'worlds', 'table.sdf')
    xacro_file = os.path.join(get_package_share_directory('crane_plus_description'),
        'urdf', 'crane_plus.urdf.xacro')
    urdf_text = xacro.process_file(xacro_file).toprettyxml(indent='  ')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(get_package_share_directory('crane_plus_description'))}

    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo', world_file],
            output='screen',
            additional_env=env,
            shell=True
        )

    spawn_entity = Node(package='ros_ign_gazebo', executable='create',
                        arguments=['-string', urdf_text, '-z', '1.02'],
                        output='screen')

    ign_joint_state_name = "/world/default/model/crane_plus/joint_state"
    msg_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        arguments=[ign_joint_state_name + '@sensor_msgs/msg/JointState[ignition.msgs.Model'],
                        remappings=[(ign_joint_state_name, '/joint_states')],
                        output='screen')

    return LaunchDescription([
        ign_gazebo,
        spawn_entity,
        msg_bridge
    ])