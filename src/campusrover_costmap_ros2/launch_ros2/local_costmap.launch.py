#!/usr/bin/env python3
"""
Campus Rover 局部成本地圖 ROS2 Launch 文件
功能：啟動局部成本地圖生成系統 (ROS2 版本)
支援：調試模式、多種感測器輸入、可視化
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 宣告 launch 參數
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='是否啟用調試模式'
    )
    
    only_use_velodyne_arg = DeclareLaunchArgument(
        'only_use_velodyne',
        default_value='true',
        description='是否只使用 Velodyne 雷達'
    )
    
    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        default_value='~/bags/campusrover_bag/20200409_itc_3f_2s.bag',
        description='調試模式使用的數據包文件'
    )
    # 局部成本地圖參數文件路徑
    local_costmap_params = os.path.join(
        get_package_share_directory('campusrover_costmap_ros2'),
        'config',
        'local_costmap.yaml'
    )
    
    # 局部成本地圖節點
    local_costmap_node = Node(
        package='campusrover_costmap_ros2',
        executable='local_costmap_node',
        name='campusrover_costmap',
        output='screen',
        parameters=[local_costmap_params],
        remappings=[
            ('points2', LaunchConfiguration('pointcloud_topic', default='velodyne_points'))
            # ('points2', LaunchConfiguration('pointcloud_topic', default='laser_cloud'))
        ]
    )
    
    # 雷射轉點雲節點 (非 Velodyne 模式)
    laser_to_pointcloud_node = Node(
        package='campusrover_costmap_ros2',
        executable='laser_to_pointcloud2_node',
        name='laser_to_pointcloud2',
        condition=UnlessCondition(LaunchConfiguration('only_use_velodyne')),
        remappings=[
            ('laser', '/scan'),
            ('pointcloud', 'laser_cloud')
        ]
    )
    
    # 調試模式組
    debug_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('debug')),
        actions=[
            # 使用模擬時間參數
            Node(
                package='rclcpp',
                executable='parameter_blackboard',
                name='global_parameter_server',
                parameters=[{'use_sim_time': True}]
            ),
            
            # 播放數據包 (需要手動執行)
            # Node(
            #     package='rosbag2_play',
            #     executable='play',
            #     arguments=['--clock', LaunchConfiguration('bag_filename')]
            # ),
            
            # 靜態 TF 變換
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_broadcaster',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
            ),
            
            # RViz 可視化
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('campusrover_costmap_ros2'),
                    'rviz',
                    'campusrover_costmap.rviz'
                ])]
            )
        ]
    )
    
    return LaunchDescription([
        debug_arg,
        only_use_velodyne_arg,
        bag_filename_arg,
        local_costmap_node,
        laser_to_pointcloud_node,
        debug_group
    ])
