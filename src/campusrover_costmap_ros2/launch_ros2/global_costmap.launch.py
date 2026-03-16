#!/usr/bin/env python3
"""
Campus Rover 全域成本地圖 ROS2 Launch 文件
功能：啟動全域成本地圖生成系統 (ROS2 版本)
包含：地圖服務器、RViz 可視化、全域成本地圖節點
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 宣告 launch 參數
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('campusrover_costmap_ros2'),
            'test_data',
            'stata.yaml'
        ]),
        description='地圖文件路徑'
    )
    
    # Foxy 版本地圖服務器節點 (使用傳統 map_server)
    map_server_node = Node(
        package='nav2_map_server',  # Foxy 中也可用，但可能需要調整
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # 生命週期管理器 (Foxy 版本兼容)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        parameters=[{
            'node_names': ['map_server'],
            'use_sim_time': False,
            'autostart': True
        }],
        output='screen'
    )
    
    # 全域成本地圖節點
    global_costmap_node = Node(
        package='campusrover_costmap_ros2',
        executable='global_costmap_node',
        name='global_costmap_node',
        output='screen',
        parameters=[{
            'costmap_resolution': 0.0,  # 0=使用原始地圖解析度
            'inflation_radius': 0.3,
            'cost_scaling_factor': 10.0
        }],
        remappings=[
            ('global_costmap', 'global_costmap'),
            ('map', 'map')
        ]
    )
    
    # # RViz 可視化節點
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', PathJoinSubstitution([
    #         FindPackageShare('campusrover_costmap_ros2'),
    #         'rviz',
    #         'global_costmap.rviz'
    #     ])]
    # )
    
    return LaunchDescription([
        map_file_arg,
        map_server_node,
        lifecycle_manager,
        global_costmap_node,
        # rviz_node
    ])
