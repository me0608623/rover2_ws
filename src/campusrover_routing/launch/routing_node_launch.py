from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    enable_one_way_arg = DeclareLaunchArgument('enable_one_way', default_value='false')
    file_path1_arg = DeclareLaunchArgument(
        'file_path1',
        default_value=os.path.join(
            get_package_share_directory('campusrover_routing'),
            'share/node_module/3F_modul.csv'
        )
    )
    file_path2_arg = DeclareLaunchArgument(
        'file_path2',
        default_value=os.path.join(
            get_package_share_directory('campusrover_routing'),
            'share/node_module/3F_modul.csv'
        )
    )
    file_path3_arg = DeclareLaunchArgument(
        'file_path3',
        default_value=os.path.join(
            get_package_share_directory('campusrover_routing'),
            'share/node_module/3F_modul.csv'
        )
    )
    file_node_info_arg = DeclareLaunchArgument(
        'file_node_info',
        default_value=os.path.join(
            get_package_share_directory('campusrover_routing'),
            'share/node_module/3F_info.csv'
        )
    )
    # Debug group: include database handler and test node if debug is true
    debug_group = GroupAction([
        Node(
            package='campusrover_routing',
            executable='g_test',
            name='g_test',
            parameters=[
                {'origin': 'c1'},
                {'destination': 'e0'}
            ],
            condition=IfCondition(LaunchConfiguration('debug'))
        )
    ])
    
    mapinfo_db_handler_node = Node(
        package='campusrover_routing',
        executable='mapinfo_db_handler.py',
        name='mapinfo_db_handler',
        output='screen',
        parameters=[
            {'use_database': False},
            {'json_folder': os.path.join(
                get_package_share_directory('campusrover_routing'),'share/json/')}
        ]
    )

    # Main routing_engine_node
    routing_engine_node = Node(
        package='campusrover_routing',
        executable='routing_engine_node',
        name='routing_engine_node',
        output='screen',
        parameters=[
            {'enable_one_way': LaunchConfiguration('enable_one_way')},
            {'use_csv': False},
            {'path_orienation': False},
            {'file_path1': LaunchConfiguration('file_path1')},
            {'file_path2': LaunchConfiguration('file_path2')},
            {'file_path3': LaunchConfiguration('file_path3')},
            {'file_node_info': LaunchConfiguration('file_node_info')},
            {'connect_method': 'BezierCurve'},
            {'path_resolution': 0.05},
            {'bezier_length': 1.5},
            {'bezier_resolution': 0.01},
            {'BSpline_k': 3},
            {'BSpline_resolution': 0.001},
            {'path_frame': 'map'}
        ]
    )
    
    routes_visualization_node = Node(
        package='campusrover_routing',
        executable='routes_visualization',
        name='routes_visualization_node',
        output='screen',
        # parameters=[
        #     {'path_frame': 'map'}
        # ]
    )
    
    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
    )
    
    return LaunchDescription([
        debug_arg,
        enable_one_way_arg,
        file_path1_arg,
        file_path2_arg,
        file_path3_arg,
        file_node_info_arg,
        debug_group,
        routing_engine_node,
        mapinfo_db_handler_node,
        routes_visualization_node,
        tf_node,
    ])