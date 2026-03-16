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
    
    simulator_mpc_arg = DeclareLaunchArgument('simulator_mpc', default_value='true')
    simulator_mpc_config = LaunchConfiguration('simulator_mpc')
    
    target_frame = PythonExpression(["'map' if '", simulator_mpc_config, "' == 'true' else 'world'"])
    child_frame = PythonExpression(["'turtle1' if '", simulator_mpc_config, "' == 'true' else 'base_footprint'"])
    cmd_vel_arg = PythonExpression(["'/turtle1/cmd_vel' if '", simulator_mpc_config, "' == 'true' else '/input/nav_cmd_vel'"])

    # Define the node
    mpc_node = Node( package='bubble_planner', executable='bubble_mpc_node', name='bubble_mpc_node', output='screen',
                     parameters=[{'target_frame_id': target_frame}, {'child_frame_id': child_frame},
                                 {'cmd_vel_topic': cmd_vel_arg},
                                 {'input_path': '/bubble_path'}],
                    #  remappings=[
                    #      ('/bubble_path', '/test_bubble_path'), 
                    #      ('/bubble_info', '/test_bubble_info')] 
                     )

    return LaunchDescription([
        simulator_mpc_arg,
        mpc_node,
    ])
