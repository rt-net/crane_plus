

# <launch>
#   <arg name="gui" default="true"/>
#   <arg name="state_rate" default="10"/>
#   <param name="rate" value="$(arg state_rate)"/>
#   <param name="robot_description"
#     command="$(find xacro)/xacro --inorder '$(find crane_x7_description)/urdf/crane_x7.urdf.xacro'"
#     />
#   <rosparam param="source_list">["joint_states_source"]</rosparam>
#   <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" if="$(arg gui)"/>
#   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg gui)"/>
#   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
#   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find crane_x7_description)/config/urdf.rviz"/>
# </launch>

import os

import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    pkg_share = FindPackageShare('crane_plus_description').find('crane_plus_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'crane_plus.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params])
    jsp = Node(
        # node_name='line_follower',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    return launch.LaunchDescription([rsp, jsp])
