from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    # 用模式名分資料夾：bubble_mpc / dwa / teb / ...
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='bubble_mpc',
        description='Planner mode name used as bag subfolder (e.g., bubble_mpc, dwa)'
    )
    mode = LaunchConfiguration('mode')

    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_name = f"data_{timestamp}"

    bag_output_path = PathJoinSubstitution([
        '~/bags',
        mode,
        bag_name
    ])

    return LaunchDescription([
        mode_arg,

        # 先確保父資料夾存在（避免 ros2 bag 因為 bags/ 不存在而失敗）
        ExecuteProcess(
            cmd=['bash', '-lc', 'mkdir -p bags'],
            output='screen'
        ),

        # 在 launch 執行時印出真正的路徑（不是 LaunchConfiguration 物件）
        LogInfo(msg=[
            TextSubstitution(text='[Experiment Launch] Bag recording will be saved to: '),
            bag_output_path
        ]),

        # 開始錄 bag（存到本機 bags/<mode>/<timestamp>/）
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_output_path,
                '/planner_data_collector/all_data',
                '/planner_data_collector/resource_usage'
            ],
            output='screen'
        ),
    ])
