from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('campusrover_description'),
        'urdf',
        'campusrover.urdf'
    )

    robot_desc = open(urdf_file, 'r').read()
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    Node(
    package='campusrover_description',
    executable='urdf_publisher.py',
    name='urdf_publisher',
    output='screen'
    )

    # joint_state_publisher_gui
    joint_state_publisher_node = Node(
        condition=LaunchConfigurationEquals('gui', 'true'),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    joint_state_publisher_node_no_gui = Node(
    condition=LaunchConfigurationEquals('gui', 'false'),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_desc}]
    )
    
    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_node_no_gui,
        robot_state_publisher_node
    ])
