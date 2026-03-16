import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ========================================================================================
    # 1. 參數宣告 (Declare Arguments)
    # ========================================================================================
    
    # 通用參數
    debug = LaunchConfiguration('debug')
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode')

    # --- From File 1 (Planner) Arguments ---
    elevator_path = LaunchConfiguration('elevator_path', default='/elevator_path')
    global_path = LaunchConfiguration('global_path', default='/global_path')
    local_costmap_topic = LaunchConfiguration('local_costmap', default='/campusrover_local_costmap')
    cmd_vel = LaunchConfiguration('cmd_vel', default='/input/nav_cmd_vel')

    # --- From File 2 (NDT Localization) Arguments ---
    base_frame = LaunchConfiguration("base_frame", default="base_link")
    odom_frame = LaunchConfiguration("odom_frame", default="odom")
    map_frame = LaunchConfiguration("map_frame", default="map") 
    ndt_resolution = LaunchConfiguration('ndt_resolution', default='1.0')
    step_size = LaunchConfiguration('step_size', default='0.1')
    trans_epsilon = LaunchConfiguration('trans_epsilon', default='0.00001')
    max_iterations = LaunchConfiguration('max_iterations', default='10')
    converged_param = LaunchConfiguration('converged_param_transform_probability', default='2.0')
    # pcd_path = LaunchConfiguration('pcd_path')
    # pcd_path_arg = DeclareLaunchArgument('pcd_path', default_value='~/humble_ws/ndt_ws/install/ndt_localizer/share/ndt_localizer/map/3F_314.pcd', 
    #                                      description='PCD map path for ndt_localizer map_loader')


    # --- From File 3 (Routing) Arguments ---
    enable_one_way = LaunchConfiguration('enable_one_way')
    enable_one_way_arg = DeclareLaunchArgument('enable_one_way', default_value='false')
    
    # Routing file paths (using default package paths)
    routing_pkg_share = get_package_share_directory('campusrover_routing')
    file_path1_arg = DeclareLaunchArgument('file_path1', default_value=os.path.join(routing_pkg_share, 'share/node_module/3F_modul.csv'))
    file_path2_arg = DeclareLaunchArgument('file_path2', default_value=os.path.join(routing_pkg_share, 'share/node_module/3F_modul.csv'))
    file_path3_arg = DeclareLaunchArgument('file_path3', default_value=os.path.join(routing_pkg_share, 'share/node_module/3F_modul.csv'))
    file_node_info_arg = DeclareLaunchArgument('file_node_info', default_value=os.path.join(routing_pkg_share, 'share/node_module/3F_info.csv'))

    # --- From File 4 (Costmap) Arguments ---
    only_use_velodyne = LaunchConfiguration('only_use_velodyne')
    only_use_velodyne_arg = DeclareLaunchArgument('only_use_velodyne', default_value='true', description='Only use Velodyne')
    
    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        default_value='~/bags/campusrover_bag/20200409_itc_3f_2s.bag',
        description='Bag file for debug mode'
    )
    
    use_bubble = LaunchConfiguration('use_bubble')
    use_bubble_arg = DeclareLaunchArgument(
        'use_bubble', 
        default_value='true', 
        description='If true, use bubble_planner; otherwise use dwa_planner'
    )

    demo_script_path = os.path.expanduser('~/rover2_ws/src/bubble_planner/demo_campusrover_bubblempc.py')
    
    # ========================================================================================
    # 2. 節點定義 (Node Definitions)
    # ========================================================================================

    # -----------------------------------------------------------------------
    # Part 1: DWA Planner & Simulation (From File 1)
    # -----------------------------------------------------------------------
    
    # 模擬器與機器人狀態 (Debug Group)
    planner_debug_group = GroupAction(
        condition=IfCondition(debug),
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': PathJoinSubstitution([
                        FindPackageShare('campusrover_description'), 'urdf', 'campusrover.urdf'
                    ])
                }]
            ),
            Node(
                package='campusrover_sim_rviz',
                executable='obstacle_simulator',
                name='obstacle_simulator_node',
                output='screen',
                parameters=[{'odom_frame_id': 'odom'}, {'robot_frame_id': 'base_link'}]
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
            # RVIZ (DWA version - Primary Visualization)
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_dwa',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('campusrover_move'), 'rviz', 'dwa_planner.rviz'
                ])],
                output='screen'
            )
        ]
    )
    
    bubble_pkg_share = get_package_share_directory('bubble_planner')
    bubble_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bubble_pkg_share, 'launch', 'bubble_planner.launch.py')
        ]),
        launch_arguments={'simulator': 'false'}.items(),
        condition=IfCondition(use_bubble)
    )
    
    dwa_node = Node(
        package='campusrover_move',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        condition=UnlessCondition(use_bubble),
        remappings=[
            ('elevator_path', elevator_path),
            ('global_path', global_path),
            ('costmap', local_costmap_topic),
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

    # -----------------------------------------------------------------------
    # Part 2: NDT Localization (From File 2)
    # -----------------------------------------------------------------------
    ndt_pkg_share = get_package_share_directory('ndt_localizer')
    
    pcd_path = LaunchConfiguration('pcd_path')
    pcd_path_arg = DeclareLaunchArgument('pcd_path', default_value=os.path.join(ndt_pkg_share, 'map/3F_314.pcd'),
                                          description='PCD map path for map_loader')
    map_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ndt_pkg_share, 'launch', 'map_loader_launch.py')]),
        launch_arguments={'pcd_path': pcd_path}.items()
    )
    
    points_downsample_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ndt_pkg_share, 'launch', 'points_downsample_launch.py')]),
    )
    
    tf_static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ndt_pkg_share, 'launch', 'tf_static_launch.py')]),
    )

    ndt_localizer_node = Node(
        package='ndt_localizer',
        executable='ndt_localizer_node',
        name='ndt_localizer_node',
        output='screen',
        parameters=[{
            'resolution': ndt_resolution,
            'step_size': step_size,
            'trans_epsilon': trans_epsilon,
            'max_iterations': max_iterations,
            'converged_param_transform_probability': converged_param,
            'debug': debug,
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame
        }],
        remappings=[
            ('ndt_pose', '/ndt_pose'),
            ('diagnostics', '/diagnostics')
        ]
    )

    points_downsample_launch_delayed = TimerAction(period=2.0, actions=[points_downsample_launch])
    map_loader_launch_delayed = TimerAction(period=3.0, actions=[map_loader_launch])

    # [已註解] NDT 專用的 RViz (避免與 DWA RViz 衝突)
    # ndt_rviz_node = Node(
    #     package='rviz2', executable='rviz2', name='rviz_ndt',
    #     arguments=['-d', os.path.join(ndt_pkg_share, 'cfgs', 'ndt_loc.rviz')],
    # )

    # -----------------------------------------------------------------------
    # Part 3: Routing Engine (From File 3)
    # -----------------------------------------------------------------------
    
    routing_debug_group = GroupAction([
        Node(
            package='campusrover_routing',
            executable='g_test',
            name='g_test',
            parameters=[{'origin': 'c1'}, {'destination': 'e0'}],
            condition=IfCondition(debug)
        )
    ])

    mapinfo_db_handler_node = Node(
        package='campusrover_routing',
        executable='mapinfo_db_handler.py',
        name='mapinfo_db_handler',
        output='screen',
        parameters=[
            {'use_database': False},
            {'json_folder': os.path.join(routing_pkg_share, 'share/json/')}
        ]
    )

    routing_engine_node = Node(
        package='campusrover_routing',
        executable='routing_engine_node',
        name='routing_engine_node',
        output='screen',
        parameters=[
            {'enable_one_way': enable_one_way},
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
        output='screen'
    )

    # World -> Map TF (從 File 3 來的)
    tf_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
    )

    # -----------------------------------------------------------------------
    # Part 4: Costmap (From File 4)
    # -----------------------------------------------------------------------
    costmap_pkg_share = get_package_share_directory('campusrover_costmap_ros2')
    local_costmap_params = os.path.join(costmap_pkg_share, 'config', 'local_costmap.yaml')

    local_costmap_node = Node(
        package='campusrover_costmap_ros2',
        executable='local_costmap_node',
        name='campusrover_costmap',
        output='screen',
        parameters=[local_costmap_params],
        remappings=[
            ('points2', LaunchConfiguration('pointcloud_topic', default='velodyne_points'))
        ]
    )

    laser_to_pointcloud_node = Node(
        package='campusrover_costmap_ros2',
        executable='laser_to_pointcloud2_node',
        name='laser_to_pointcloud2',
        condition=UnlessCondition(only_use_velodyne),
        remappings=[
            ('laser', '/scan'),
            ('pointcloud', 'laser_cloud')
        ]
    )

    # Costmap 的 Debug Group (原本包含播放 bag 與靜態 TF)
    costmap_debug_group = GroupAction(
        condition=IfCondition(debug),
        actions=[
            Node(
                package='rclcpp',
                executable='parameter_blackboard',
                name='global_parameter_server',
                parameters=[{'use_sim_time': True}]
            ),
            # File 4 的靜態 TF (Map->BaseLink)，這通常由 NDT 提供，若同時開啟 NDT 需注意衝突
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='map_broadcaster',
            #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
            # ),
            # [已註解] Costmap 專用 RViz
            # Node(
            #     package='rviz2', executable='rviz2', name='rviz_costmap',
            #     arguments=['-d', PathJoinSubstitution([
            #         FindPackageShare('campusrover_costmap_ros2'), 'rviz', 'campusrover_costmap.rviz'
            #     ])]
            # )
        ]
    )

    # -----------------------------------------------------------------------
    # Part 5: planner_data_collect 
    # -----------------------------------------------------------------------
    # test_pkg_share = get_package_share_directory('test_pkg')
    # logger_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(test_pkg_share, 'launch', 'logger.launch.py')]),
    #     launch_arguments={'bubble': use_bubble}.items()
    # )

    # -----------------------------------------------------------------------
    # Part 6: Demo Python Script (新增部分)
    # -----------------------------------------------------------------------
    # 使用 ExecuteProcess 直接執行 python3 指令
    demo_runner = ExecuteProcess(
        cmd=['python3', demo_script_path],
        output='screen',
        name='demo_campusrover_bubblempc'
    )
    
    # ========================================================================================
    # 3. 回傳 Launch Description
    # ========================================================================================
    return LaunchDescription([
        # Args
        debug_arg,
        enable_one_way_arg,
        file_path1_arg,
        file_path2_arg,
        file_path3_arg,
        file_node_info_arg,
        only_use_velodyne_arg,
        bag_filename_arg,
        use_bubble_arg,
        pcd_path_arg,

        # Part 1: Planner
        planner_debug_group, # 包含 DWA 的 RViz
        dwa_node,
        bubble_launch,

        # Part 2: Localization (NDT)
        ndt_localizer_node,
        tf_static_launch,
        points_downsample_launch_delayed,
        map_loader_launch_delayed,

        # Part 3: Routing
        routing_debug_group,
        routing_engine_node,
        mapinfo_db_handler_node,
        routes_visualization_node,
        tf_world_to_map,

        # Part 4: Costmap
        local_costmap_node,
        laser_to_pointcloud_node,
        costmap_debug_group,
        
        TimerAction(period=10.0, actions=[demo_runner])
    ])