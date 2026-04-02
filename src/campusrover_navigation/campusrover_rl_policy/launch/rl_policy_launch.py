"""Launch file for RL policy inference node.

Uses the IsaacLab conda Python (env_isaaclab) which has torch installed.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# Path to the conda env Python with torch
CONDA_PYTHON = '/home/aa/miniconda3/envs/env_isaaclab/bin/python'

# Isaac Sim internal rclpy (Python 3.11 compatible) for conda env
ISAAC_JAZZY_RCLPY = '/home/aa/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy'
ISAAC_JAZZY_LIB = '/home/aa/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib'
SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'rl_policy_node.py',
)
CHECKPOINT_DEFAULT = (
    '/home/aa/IsaacLab/logs/skrl/'
    'Isaac-Navigation-Charge-VLP16-Curriculum-NavRL-AC/'
    'rw_groundv8_openendedv1__seed1_nowalls_diag_v9/'
    'checkpoints/agent_23436.pt'
)


def generate_launch_description():
    checkpoint_arg = DeclareLaunchArgument(
        'checkpoint_path', default_value=CHECKPOINT_DEFAULT,
        description='Path to trained model checkpoint',
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cpu',
        description='PyTorch device (cuda:0 or cpu)',
    )
    deterministic_arg = DeclareLaunchArgument(
        'deterministic', default_value='true',
        description='Use argmax action (true) or sample (false)',
    )
    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel', default_value='/cmd_vel',
        description='Output cmd_vel topic',
    )
    odom_arg = DeclareLaunchArgument(
        'odom', default_value='/odom',
        description='Input odometry topic',
    )

    policy_node = ExecuteProcess(
        cmd=[
            CONDA_PYTHON, SCRIPT_PATH,
            '--ros-args',
            '-r', '__node:=rl_policy_node',
            '-r', ['cmd_vel:=', LaunchConfiguration('cmd_vel')],
            '-r', ['odom:=', LaunchConfiguration('odom')],
            '-p', ['checkpoint_path:=', LaunchConfiguration('checkpoint_path')],
            '-p', ['device:=', LaunchConfiguration('device')],
            '-p', ['deterministic:=', LaunchConfiguration('deterministic')],
            '-p', 'use_cadn:=true',
            '-p', 'goal_timeout:=300.0',
            '-p', 'goal_lookahead:=2.0',
        ],
        output='screen',
        additional_env={
            'PYTHONPATH': ISAAC_JAZZY_RCLPY + ':' + os.environ.get('PYTHONPATH', ''),
            'LD_LIBRARY_PATH': ISAAC_JAZZY_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', ''),
            'ROS_DISTRO': 'jazzy',
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        },
    )

    return LaunchDescription([
        checkpoint_arg,
        device_arg,
        deterministic_arg,
        cmd_vel_arg,
        odom_arg,
        policy_node,
    ])
