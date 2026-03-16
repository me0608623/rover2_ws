from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_img_label_arg = DeclareLaunchArgument('is_img_label', default_value='false')

    mot_node = Node(
        package='campusrover_mot',
        executable='campusrover_mot_node',
        name='campusrover_mot_node',
        output='screen',
        remappings=[
            ('points', '/velodyne_filter_points'),
            ('/scan', '/scan'),
            ('image', '/zed_node/rgb/image_rect_color'),
            ('camera_info', '/zed_node/rgb/camera_info'),
        ],
        parameters=[{
            'detection_area_min_x': -10.0,
            'detection_area_max_x': 10.0,
            'detection_area_min_y': -10.0,
            'detection_area_max_y': 10.0,
            'detection_area_min_z': -0.05,
            'detection_area_max_z': 0.5,
            'track_dead_time': 1.0,
            'track_older_age': 0.5,
            'cluster_dist': 0.35,
            'false_alarm_min': 10,
            'false_alarm_max': 3000,

            'weight_min_tolerate': 0.01,
            'cov_scale': 20.0,
            'inherit_ratio': 0.6,

            'history_length': 10,
            'anchor_dist_threshold': 0.1,
            'speed_threshold': 0.1,

            'trackers_update_period': 0.05,
            'label_update_period': 0.1,

            'map_frame': 'map',
            'laser_frame': 'scan',
            'camera_frame': 'camera_link',

            'h_scale': 2.0,
            'v_scale': 3.0,
            'sync_tolerate': 0.08,
            'tf_tolerate': 1.0,

            'debug_mode': False,
            'is_use_laser': False,
            'is_map_filter': True,
            'is_img_label': LaunchConfiguration('is_img_label'),
            'only_dynamic_obstacle': False,
        }],
    )

    return LaunchDescription([
        is_img_label_arg,
        mot_node,
    ])
