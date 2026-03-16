import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition  # <--- 匯入判斷式
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share=get_package_share_directory('bubble_planner')
    
    debug_arg=DeclareLaunchArgument('debug', default_value='true')
    debug_config = LaunchConfiguration('debug')
    dwa_arg=DeclareLaunchArgument('dwa_mode', default_value='true')
    dwa_config = LaunchConfiguration('dwa_mode')
    simulator_costmap_arg=DeclareLaunchArgument('simulator_costmap', default_value='true')
    simulator_cosstmap_config = LaunchConfiguration('simulator_costmap')
    
    target_frame=PythonExpression(["'map' if '", simulator_cosstmap_config, "' == 'true' else 'world'"])
    child_frame=PythonExpression(["'turtle1' if '", simulator_cosstmap_config, "' == 'true' else 'base_footprint'"])
    
    simulator_cosstmap_group = GroupAction(
        condition = IfCondition(simulator_cosstmap_config),
        actions=[
                Node(package='bubble_planner', executable='static_obs',    name='static_obs',    output='screen'),
                Node(package='bubble_planner', executable='dynamic_obs',   name='dynamic_obs',   output='screen'),
                Node(package='bubble_planner', executable='tracked_obs',   name='tracked_obs',   output='screen'),
                Node(package='bubble_planner', executable='costmap_merge', name='costmap_merge', output='screen')
                ])

    # no_costmap_converter_group = GroupAction(
    #     condition=UnlessCondition(dwa_config),
    #     actions=[
    #             Node(package='bubble_planner', executable='map_to_obstacle_simple', name='map_to_obstacle_simple', output='screen',
    #                  parameters=[{'robot_frame': child_frame}]),
    #             Node(package='bubble_planner', executable='mot_to_obstacle_simple', name='mot_to_obstacle_simple', output='screen',
    #                  parameters=[{'map_frame_id': target_frame}]),
    #             Node(package='bubble_planner', executable='wall_and_dynamic_obstacle_map', name='wall_and_dynamic_obstacle_map', output='screen'),
    #             Node(package='bubble_planner', executable='split_obstacles_node', name='split_obstacles_node', output='screen'),
    #             Node(package='bubble_planner', executable='obstacle_filter', name='obstacle_filter', output='screen')
    #             ])

    rviz_path=os.path.join(pkg_share, 'rviz', 'costmap.rviz')
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_path], condition=IfCondition(debug_config))
    
    
    return LaunchDescription([
        debug_arg,
        dwa_arg,
        simulator_costmap_arg,
        simulator_cosstmap_group,
        # no_costmap_converter_group,
        rviz
    ])