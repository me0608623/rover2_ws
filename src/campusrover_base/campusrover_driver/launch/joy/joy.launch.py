from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
            'coalesce_interval': 0.05,
        }],
        remappings=[
            ('joy', 'joy') 
        ]
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node
    ])
