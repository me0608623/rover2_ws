from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    debug = LaunchConfiguration('debug', default='false')
    elevator_path = LaunchConfiguration('elevator_path', default='/elevator_path')
    global_path = LaunchConfiguration('global_path', default='/global_path')
    local_costmap = LaunchConfiguration('local_costmap', default='/campusrover_local_costmap')
    cmd_vel = LaunchConfiguration('cmd_vel', default='/input/nav_cmd_vel')

    # Debug nodes group
    debug_group = GroupAction(
        condition=IfCondition(debug),
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': PathJoinSubstitution([
                        FindPackageShare('campusrover_description'),
                        'urdf',
                        'campusrover.urdf'
                    ])
                }]
            ),
            Node(
                package='campusrover_sim_rviz',
                executable='obstacle_simulator',
                name='obstacle_simulator_node',
                output='screen',
                parameters=[
                    {'odom_frame_id': 'odom'},
                    {'robot_frame_id': 'base_link'}
                ]
            ),
            Node(
                package='campusrover_sim_rviz',
                executable='twist_odom_simulator_node',
                name='twist_odom_simulator_node',
                remappings=[
                    ('odom_gt', 'odom_gt'),
                    ('path_gt', 'path_gt'),
                    ('odom', 'odom'),
                    ('cmd_vel', 'input/nav_cmd_vel')
                ],
                parameters=[
                    {'odom_frame_id': 'odom'},
                    {'robot_frame_id': 'base_footprint'},
                    {'publish_tf': True}
                ]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('campusrover_move'),
                    'rviz',
                    'dwa_planner.rviz'
                ])],
                output='screen'
            )
        ]
    )

    # DWA planner node
    dwa_node = Node(
        package='campusrover_move',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        remappings=[
            ('elevator_path', elevator_path),
            ('global_path', global_path),
            ('costmap', local_costmap),
            ('cmd_vel', cmd_vel),
            ('odom', 'odom_gt')
        ],
        parameters=[{
            'robot_frame': 'base_link',
            'arriving_range_dis': 0.1,
            'arriving_range_angle': 0.05,
            'max_linear_acceleration': 5.0, #加速度限制 影響速度pair生成
            'max_angular_acceleration': 5.0,
            'max_linear_velocity': 0.5, #線速度限制(更上層還有demo的參數)
            'min_linear_velocity': -0.5,
            'max_angular_velocity': 0.5, #角速度限制(更上層還有demo的參數)
            'min_angular_velocity': -0.5,
            'target_point_dis': 3.0, #子目標點距離
            'threshold_occupied': 2.0,
            'footprint_max_x': 1.4,
            'footprint_min_x': 0.7,
            'footprint_max_y': 0.35,
            'footprint_min_y': -0.35,
            'obstacle_max_dis': 3.0,
            'obstacle_min_dis': 0.3,
            'obstable_cost_weight': 1.5, #1.5
            'target_dis_weight': 1.0, #1.0
            'velocity_weight': 1.0, #1.0
            'trajectory_num': 10,
            'trajectory_point_num': 10,
            'simulation_time': 6.0,
            'target_bias': 0.1,
            'min_angle_of_linear_profile': 0.1,
            'max_angle_of_linear_profile': 0.8,
            'enable_linear_depend_angular': True,
            'enable_costmap_obstacle': True,
            'direction_inverse': False
        }]
    )

    return LaunchDescription([
        debug_group,
        dwa_node
    ])
