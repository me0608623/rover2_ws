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
    
    simulator_arg = DeclareLaunchArgument('simulator', default_value='true')
    simulator_config = LaunchConfiguration('simulator')
     
    target_frame = PythonExpression(["'map' if '", simulator_config, "' == 'true' else 'world'"])
    child_frame = PythonExpression(["'turtle1' if '", simulator_config, "' == 'true' else 'base_footprint'"])
    costmap = PythonExpression(["'/costmap' if '", simulator_config, "' == 'true' else '/campusrover_local_costmap'"])
    
    turtlesim_launch_path = os.path.join(pkg_share, 'launch', 'turtlesim_driver.launch.py')
    turtlesim_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(turtlesim_launch_path), 
                                                condition=IfCondition(simulator_config))
    
    costmap_launch_path = os.path.join(pkg_share, 'launch', 'costmap.launch.py')
    costmap_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(costmap_launch_path),
                                              launch_arguments={'simulator_costmap': simulator_config, 'debug': 'false', 'dwa_mode': 'false'}.items())
    
    local_path_node = Node(package='bubble_planner', executable='local_path_node', name='local_path_node', output='screen',
                           parameters=[{'child_frame': child_frame}, {'costmap_topic': costmap},
                                       {'max_local_len': 3.0}])
    
    regular_points_node = Node(package='bubble_planner', executable='regular_points_node', name='regular_points_node',   output='screen', 
                               parameters=[{'target_frame': target_frame}, {'child_frame': child_frame}, {'costmap_topic': costmap},
                                           {'clr_obs': 0.4}, {'per_dis': 0.3}])
    
    path_builder_node = Node(package='bubble_planner', executable='path_builder_node', name='path_builder_node',   output='screen', 
                             parameters=[{'target_frame': target_frame}, {'child_frame': child_frame}, {'costmap_topic': costmap},
                                         {'connect_radius': 0.4}])

    rviz_path=os.path.join(pkg_share, 'rviz', 'bubble_mpc_sim.rviz')
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_path],
                condition=IfCondition(simulator_config))
    
    
    return LaunchDescription([
        simulator_arg,
        turtlesim_launch,
        costmap_launch,
        local_path_node,
        regular_points_node,
        path_builder_node,
        rviz
    ])