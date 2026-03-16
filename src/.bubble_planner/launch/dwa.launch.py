import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share=get_package_share_directory('bubble_planner')
    
    turtlesim_launch_path = os.path.join(pkg_share, 'launch', 'turtlesim_driver.launch.py')
    turtlesim_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlesim_launch_path))
    
    costmap_launch_path = os.path.join(pkg_share, 'launch', 'costmap.launch.py')
    costmap_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(costmap_launch_path),
                                              launch_arguments={'debug': 'false', 'dwa_mode': 'true'}.items())
    
    # Define the node
    dwa_node = Node(
        package='campusrover_move',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        remappings=[
            ('elevator_path', '/elevator_path'),
            ('global_path', '/global_path'),
            ('costmap', '/costmap'),
            ('cmd_vel', '/turtle1/cmd_vel'),
            ('odom', 'odom')
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
            'simulation_time': 5.0,
            'target_bias': 0.1,
            'min_angle_of_linear_profile': 0.1,
            'max_angle_of_linear_profile': 0.8,
            'enable_linear_depend_angular': True,
            'enable_costmap_obstacle': True,
            'direction_inverse': False
        }])
        
    rviz_path=os.path.join(pkg_share, 'rviz', 'costmap.rviz')
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_path])
    
    


    return LaunchDescription([
        turtlesim_launch,
        costmap_launch,
        dwa_node,
        rviz
    ])
