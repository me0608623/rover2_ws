from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    debug = LaunchConfiguration('debug')
    calibration_mode = LaunchConfiguration('calibration_mode')
    joy_cmd_vel_topic = LaunchConfiguration('joy_cmd_vel_topic')
    rvizconfig = LaunchConfiguration('rvizconfig')
    model = LaunchConfiguration('model')
    driver_param = os.path.join(
        get_package_share_directory('campusrover_driver'),
        'param',
        'driver_chgh.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('joy_cmd_vel_topic', default_value='/input/joy_cmd_vel'),
        DeclareLaunchArgument('rvizconfig', default_value='rviz/odom.rviz'),
        DeclareLaunchArgument('model', default_value='urdf/campusrover.urdf'),
        DeclareLaunchArgument('calibration_mode', default_value='false'),

        # Debug group
        GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': model}]
            ),
            Node(
                package='hector_trajectory_server',
                executable='hector_trajectory_server',
                name='hector_trajectory_server',
                remappings=[('trajectory', 'odom_path')],
                parameters=[
                    {'target_frame_name': 'odom'},
                    {'source_frame_name': 'base_link'},
                    {'trajectory_update_rate': 10},
                    {'trajectory_publish_rate': 5}
                ]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rvizconfig]
            ),
        ], condition=IfCondition(debug)),

        Node(
            package='campusrover_driver',
            executable='rover_driver',
            name='campusrover_driver',
            output='screen',
            remappings=[('/cmd_vel', '/output/cmd_vel')],
            parameters=[{'config_file': driver_param}]
        ),

        # joy
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(), '/joy/joy.launch.py'
            ])
        ),

        Node(
            package='campusrover_driver',
            executable='joy_to_twist',
            name='joy_to_twist',
            remappings=[('cmd_vel', joy_cmd_vel_topic)],
            parameters=[
                {'scale_linear': 0.6},
                {'scale_angular': 0.4},
                {'axis_linear': 1},
                {'axis_angular': 3},
                {'speed_up_vel': 0.5}
            ]
        ),

        Node(
            package='campusrover_driver',
            executable='lcr_cmd_vel_mux',
            name='lcr_cmd_vel_mux',
            remappings=[
                ('input/joy_cmd_vel', joy_cmd_vel_topic),
                ('input/nav_cmd_vel', 'input/nav_cmd_vel'),
                ('output/cmd_vel', 'output/cmd_vel')
            ],
            parameters=[
                {'joy_mode_button': 7},
                {'nav_mode_button': 0},
                {'stop_mode_button': 1},
                {'release_mode_button': 3}
            ]
        ),

        GroupAction([
            Node(
                package='campusrover_driver',
                executable='calibration',
                name='calibration',
                output='screen'
            )
        ], condition=IfCondition(calibration_mode)),
    ])