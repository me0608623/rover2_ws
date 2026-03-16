import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 設定 UTF-8 編碼以支援中文輸出
if sys.stdout.encoding != 'utf-8':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

"""
========================================================================================
 Campus Rover Demo - 系統架構與通訊總覽
========================================================================================

當啟動此 launch 檔時，將會啟動以下核心模組 (Nodes) 及其對應的通訊關係：

1. 地圖服務器 (map_server):
   - 物理意義: 發布靜態 2D 佔用網格地圖 (Occupancy Grid Map)，供全域規劃器了解整個場域的固定障礙物。
   - 發布 (Publish): /map (通常為 nav_msgs/OccupancyGrid) 供 AIT* 讀取。

2. AIT* 全域規劃器 (aitstar_planner):
   - 物理意義: 從起始點到目標點，基於靜態地圖計算一條不會撞牆的總體路徑 (Global Path)。
   - 訂閱 (Subscribe): /map (地圖), /goal_pose (從 RViz 收到的目標點), TF (得知機器人現在位置)
   - 發布 (Publish): /global_path (發送給 DWA 做局部跟隨)

3. DWA 局部規劃器 (dwa_planner):
   - 物理意義: 根據全域路徑與「機器人眼前的即時障礙物」，在考慮機器人加減速限制下，計算出當下該前進或轉彎的實際速度指令。
   - 訂閱 (Subscribe): /global_path (全域路徑), /campusrover_local_costmap (局部障礙物地圖), TF/odom
   - 發布 (Publish): /input/nav_cmd_vel (Twist 速度指令，用來控制實體馬達或模擬車體)

4. NDT 定位系統 (ndt_localizer_node 等):
   - 物理意義: 透過 3D 光達目前的掃描點雲，與事先建好的 3D 點雲地圖對位 (Scan Matching)，藉此計算出機器人在地圖中的絕對座標，以修正輪子打滑造成的里程計 (odom) 誤差。
   - 訂閱 (Subscribe): 光達即時點雲、3D 點雲地圖
   - 發布 (Publish): /ndt_pose (目前絕對位置), 並發布 TF (map -> odom) 修正樹。

5. 障礙物地圖 (campusrover_costmap):
   - 物理意義: 將光達 (3D Velodyne 或 2D Laser) 掃描到的即時點雲資料，投影並膨脹成機器人周遭的 2D 障礙物影響範圍，避免機器人擦撞。
   - 訂閱 (Subscribe): /velodyne_points 或 /laser_cloud (光達點雲)
   - 發布 (Publish): /campusrover_local_costmap (供 DWA 讀取以避障)

6. 地圖資訊服務 (mapinfo_db_handler):
   - 物理意義: 提供特定樓層與 routing (如電梯、路口節點) 相關的業務邏輯資訊，可能透過 Service 讓其他模組查詢導航點。
========================================================================================
"""

def generate_launch_description():
    # ========================================================================================
    # 1. 參數宣告 (Declare Arguments)
    # ========================================================================================

    # 通用參數
    debug = LaunchConfiguration('debug')
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode')
    rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz visualization')

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
    converged_param = LaunchConfiguration('converged_param_transform_probability', default='1.5')

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

    # --- Map Server Argument ---
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/aa/maps/4v3F.yaml',
        description='靜態地圖路徑 (同時供 AIT* 規劃 + global_costmap 膨脹 + MOT 過濾)'
    )

    # ========================================================================================
    # 2. 節點定義 (Node Definitions)
    # ========================================================================================

    # -----------------------------------------------------------------------
    # Part 0: Map Server (發布 /map，供 AIT* + global_costmap + MOT 共用)
    # -----------------------------------------------------------------------
    map_server_node = Node(
        package='campusrover_demo',
        executable='simple_map_publisher',
        name='map_server',
        output='screen',
        parameters=[{
            'map_file': LaunchConfiguration('map_file')
        }]
    )

    # -----------------------------------------------------------------------
    # Part 1: AIT* 全域路徑規劃器
    # -----------------------------------------------------------------------
    aitstar_node = Node(
        package='aitstar_path_planner',
        executable='aitstar_planner_node',
        name='aitstar_planner',
        output='screen',
        # prefix=['python3.10'],  # 強制使用 Python 3.10 (註解掉以修復 PYTHONPATH 問題)
        parameters=[{
            'robot_radius': 0.3,
            'batch_size': 100,
            'max_iterations': 500,
            'goal_bias': 0.05,
            'rewire_factor': 1.1,
            'planning_frequency': 0.0,  # 禁用自動規劃，由目標點觸發
            'enable_smoothing': True,
            'visualize_tree': True,
            'obstacle_threshold': 50,
            'unknown_is_obstacle': False,
            'global_frame': 'map',
            'robot_frame': 'base_link',
            'replan_on_goal_change': True,
            'path_tolerance': 0.5,
        }]
    )

    # -----------------------------------------------------------------------
    # Part 2: DWA Planner & Simulation
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
            )
        ]
    )

    # RVIZ (可選啟動 - 載入 demo.rviz)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_demo',
        arguments=['-d', '/home/aa/rviz/demo.rviz'],
        output='screen',
        condition=IfCondition(rviz)
    )

    dwa_node = Node(
        package='campusrover_move',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
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
    # Part 3: NDT Localization
    # -----------------------------------------------------------------------
    ndt_pkg_share = get_package_share_directory('ndt_localizer')

    map_loader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ndt_pkg_share, 'launch', 'map_loader_launch.py')]),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items()
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
    # Part 4: Routing Engine (保留用於地圖資訊)
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

    # World -> Map TF
    tf_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_broadcaster",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "world", "--child-frame-id", "map"]
    )

    # -----------------------------------------------------------------------
    # Part 5: Costmap
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

    # -----------------------------------------------------------------------
    # Part 6: MOT 多物體追蹤
    # -----------------------------------------------------------------------
    # 流程: nav2_map_server(/map) → global_costmap_node(/global_costmap) → MOT (點雲過濾)
    global_costmap_node = Node(
        package='campusrover_costmap_ros2',
        executable='global_costmap_node',
        name='global_costmap_node',
        output='screen',
        parameters=[{
            'costmap_resolution': 0.0,
            'inflation_radius': 0.5,
            'cost_scaling_factor': 10.0,
        }],
        remappings=[
            ('map', '/map'),
            ('global_costmap', '/global_costmap'),
        ]
    )

    # 6b. MOT 節點
    mot_node = Node(
        package='campusrover_mot',
        executable='campusrover_mot_node',
        name='campusrover_mot_node',
        output='screen',
        remappings=[
            ('points', '/velodyne_points'),
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

            'debug_mode': True,
            'is_use_laser': False,
            'is_map_filter': True,
            'is_img_label': False,
            'only_dynamic_obstacle': False,
        }]
    )

    # Costmap 的 Debug Group
    costmap_debug_group = GroupAction(
        condition=IfCondition(debug),
        actions=[
            Node(
                package='rclcpp',
                executable='parameter_blackboard',
                name='global_parameter_server',
                parameters=[{'use_sim_time': True}]
            ),
        ]
    )

    # ========================================================================================
    # 3. 回傳 Launch Description
    # ========================================================================================
    return LaunchDescription([
        # 啟動訊息 (中文)
        LogInfo(msg='=' * 60),
        LogInfo(msg='🤖 Campus Rover Demo 啟動中...'),
        LogInfo(msg='=' * 60),
        LogInfo(msg='📋 組件清單:'),
        LogInfo(msg='  [1/7] 地圖服務器 (Map Server)'),
        LogInfo(msg='  [2/7] AIT* 全域路徑規劃'),
        LogInfo(msg='  [3/7] DWA 局部路徑規劃'),
        LogInfo(msg='  [4/7] NDT 定位系統'),
        LogInfo(msg='  [5/7] 地圖資訊服務'),
        LogInfo(msg='  [6/7] 障礙物地圖 (Costmap)'),
        LogInfo(msg='  [7/7] MOT 多物體追蹤'),
        LogInfo(msg='=' * 60),
        # Args
        debug_arg,
        enable_one_way_arg,
        file_path1_arg,
        file_path2_arg,
        file_path3_arg,
        file_node_info_arg,
        only_use_velodyne_arg,
        bag_filename_arg,
        map_file_arg,
        rviz_arg,

        # Part 0: 地圖服務器 (simple_map_publisher)
        LogInfo(msg='[啟動] 地圖服務器 (/map，供 AIT* + global_costmap + MOT 共用)...'),
        map_server_node,

        # Part 1: AIT* 全域路徑規劃器
        LogInfo(msg='[啟動] AIT* 全域路徑規劃器...'),
        aitstar_node,

        # Part 2: DWA 局部路徑規劃
        LogInfo(msg='[啟動] DWA 局部路徑規劃器...'),
        planner_debug_group,
        dwa_node,
        rviz_node,

        # Part 3: NDT 定位系統
        LogInfo(msg='[啟動] NDT 定位系統...'),
        ndt_localizer_node,
        tf_static_launch,
        LogInfo(msg='[延遲 2秒] 點雲降採樣...'),
        points_downsample_launch_delayed,
        LogInfo(msg='[延遲 3秒] 地圖載入器 (Map Loader)...'),
        map_loader_launch_delayed,

        # Part 4: 地圖資訊服務
        LogInfo(msg='[啟動] 地圖資訊服務...'),
        routing_debug_group,
        mapinfo_db_handler_node,
        tf_world_to_map,

        # Part 5: 障礙物地圖
        LogInfo(msg='[啟動] 障礙物地圖...'),
        local_costmap_node,
        laser_to_pointcloud_node,
        costmap_debug_group,

        # Part 6: MOT 多物體追蹤
        LogInfo(msg='[啟動] 全域成本地圖 (/map → /global_costmap)...'),
        global_costmap_node,
        LogInfo(msg='[啟動] MOT 多物體追蹤節點...'),
        mot_node,

        # 完成訊息
        LogInfo(msg='=' * 60),
        LogInfo(msg='✅ Campus Rover Demo 啟動完成!'),
        LogInfo(msg='💡 提示: 使用 RViz 2D Nav Goal 設定目標點'),
        LogInfo(msg='📌 AIT* 將自動規劃全域路徑'),
        LogInfo(msg='=' * 60),
    ])
