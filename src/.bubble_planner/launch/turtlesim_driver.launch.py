from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_name='bubble_planner'
    tf_time_offset_arg = DeclareLaunchArgument('tf_time_offset', default_value='0.0')

    static_map = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_static_broadcaster0',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'])
    
    static_odom = Node(package='tf2_ros', executable='static_transform_publisher', name='tf_static_broadcaster1',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    turtlesim = Node(package='turtlesim', executable='turtlesim_node', name='turtlesim_node', output='screen')

    turtlesim_to_map = Node(package=pkg_name, executable='turtlesim_to_map', name='turtlesim_to_map', output='screen',
        parameters=[{'frame_id': 'map', 'child_frame_id': 'turtle1',
                     'tf_time_offset': LaunchConfiguration('tf_time_offset')}])

    global_path_generator = Node(package=pkg_name, executable='global_path_generator', name='global_path_generator', output='screen')
    
    robot_marker = Node(package=pkg_name, executable='robot_marker', name='robot_marker', output='screen')

    generated_odom = Node(package=pkg_name, executable='generated_sim_odom', name='generated_sim_odom', output='screen')

    return LaunchDescription([
        tf_time_offset_arg,
        static_map,
        static_odom,
        turtlesim,
        turtlesim_to_map,
        global_path_generator,
        robot_marker,
        generated_odom,
    ])
