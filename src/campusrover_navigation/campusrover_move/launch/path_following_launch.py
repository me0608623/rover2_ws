#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    debug_arg = LaunchConfiguration('debug', default='false')
    bag_filename_arg = LaunchConfiguration(
        'bag_filename',
        default=[EnvironmentVariable('HOME'), '/bags/campusrover_bag/20200409_itc_3f_2s.bag']
    )
    elevator_path_arg = LaunchConfiguration('elevator_path', default='/elevator_path')
    global_path_arg = LaunchConfiguration('global_path', default='/global_path')
    local_costmap_arg = LaunchConfiguration('local_costmap', default='/campusrover_local_costmap')
    cmd_vel_arg = LaunchConfiguration('cmd_vel', default='/input/nav_cmd_vel')
    dwa_obstacle_arg = LaunchConfiguration('enable_dwa_obstacle_avoidance', default='true')
    pullover_mode_arg = LaunchConfiguration('enable_pullover_mode', default='false')

    # Debug group (play bag, robot_state_publisher, rviz)
    debug_group = GroupAction([
        Node(
            package='rosbag2',
            executable='play',
            name='playbag',
            arguments=['--clock', LaunchConfiguration('bag_filename')],
            condition=IfCondition(debug_arg)
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            condition=IfCondition(debug_arg)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(
                get_package_share_directory('campusrover_move'),
                'rviz',
                'dp_planner.rviz'
            )],
            condition=IfCondition(debug_arg),
            output='screen',
        ),
    ])

    # Main path_following node
    path_following_node = Node(
        package='campusrover_move',
        executable='path_following',
        name='path_following',
        output='screen',
        parameters=[
            {'robot_frame': 'base_link'},
            {'arriving_range_dis': 0.1},
            {'arriving_range_angle': 0.05},
            {'max_linear_velocity': 0.5},
            {'max_angular_velocity': 1.0},
            {'target_point_dis': 0.6},
            {'threshold_occupied': 2.0},
            {'footprint_max_x': 1.0},
            {'footprint_min_x': -0.1},
            {'footprint_max_y': 0.1},
            {'footprint_min_y': -0.1},
            {'speed_pid_k': 0.8},
            {'min_angle_of_linear_profile': 0.1},
            {'max_angle_of_linear_profile': 0.8},
            {'obstacle_range': 0.3},
            {'enable_linear_depend_angular': True},
            {'enable_costmap_obstacle': True},
            {'direction_inverse': False},
            {'enable_dwa_obstacle_avoidance': dwa_obstacle_arg},
            {'enable_pullover_mode': pullover_mode_arg}
        ],
        remappings=[
            ('elevator_path', elevator_path_arg),
            ('global_path', global_path_arg),
            ('costmap', local_costmap_arg),
            ('cmd_vel', cmd_vel_arg)
        ]
    )

    # Optionally include dwa and pullover planners
    dwa_planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
            get_package_share_directory('campusrover_move'),
            'launch/dwa_planner_launch.py'
            )
        ),
        condition=IfCondition(dwa_obstacle_arg)
    )
    pullover_planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
            get_package_share_directory('campusrover_move'),
            'launch/pullover_path_planner.launch.py'
            )
        ),
        condition=IfCondition(pullover_mode_arg)
    )
    
    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
    )
    tf_node_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_footprint"]
    )

    return LaunchDescription([
        debug_group,
        path_following_node,
        dwa_planner_include,
        # pullover_planner_include
        # tf_node,
        # tf_node_1,
    ])