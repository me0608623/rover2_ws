import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home = os.environ.get('HOME', '/home')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value=os.path.join(home, 'spot/spot_data/bags/0822_spot_mot'))
    map_file_arg = DeclareLaunchArgument(
        'map_file_ros',
        default_value=os.path.join(home, 'spot/spot_data/maps/3F.yaml'))
    is_img_label_arg = DeclareLaunchArgument('is_img_label', default_value='false')

    # Play bag
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--clock', LaunchConfiguration('bag_file')],
        output='screen')

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file_ros'),
                      'use_sim_time': True}],
        remappings=[('map', 'global_map')],
    )

    # Costmap node
    costmap_node = Node(
        package='campusrover_costmap_ros2',
        executable='global_costmap_node',
        name='global_costmap_node',
        parameters=[{
            'costmap_resolution': 0.0,
            'inflation_radius': 0.8,
            'cost_scaling_factor': 10.0,
            'use_sim_time': True,
        }],
        remappings=[
            ('global_costmap', 'global_costmap'),
            ('map', 'global_map'),
        ],
    )

    # MOT node
    mot_node = Node(
        package='campusrover_mot',
        executable='campusrover_mot_node',
        name='campusrover_mot_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            'detection_area_min_x': -10.0,
            'detection_area_max_x': 10.0,
            'detection_area_min_y': -10.0,
            'detection_area_max_y': 0.0,
            'detection_area_min_z': -0.7,
            'detection_area_max_z': 0.5,
            'track_dead_time': 1.0,
            'track_older_age': 0.5,
            'cluster_dist': 0.35,
            'false_alarm_min': 10,
            'false_alarm_max': 3000,

            'weight_min_tolerate': 0.01,
            'cov_scale': 30.0,
            'inherit_ratio': 0.6,

            'history_length': 10,
            'anchor_dist_threshold': 0.1,
            'speed_threshold': 0.1,

            'trackers_update_period': 0.05,
            'label_update_period': 0.1,

            'map_frame': 'map',
            'laser_frame': 'hokuyo_link',
            'camera_frame': 'camera_link',

            'h_scale': 2.0,
            'v_scale': 3.0,
            'sync_tolerate': 0.08,
            'tf_tolerate': 0.1,

            'debug_mode': True,
            'is_use_laser': False,
            'is_map_filter': True,
            'is_img_label': LaunchConfiguration('is_img_label'),
            'only_dynamic_obstacle': False,
        }],
        remappings=[
            ('points', '/velodyne_filter_points'),
            ('/scan', '/hokuyo_scan'),
            ('image', '/zed_node/rgb/image_rect_color'),
            ('camera_info', '/zed_node/rgb/camera_info'),
        ],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory('campusrover_mot'), 'rviz', 'show.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        bag_file_arg,
        map_file_arg,
        is_img_label_arg,
        play_bag,
        map_server,
        costmap_node,
        mot_node,
        rviz_node,
    ])
