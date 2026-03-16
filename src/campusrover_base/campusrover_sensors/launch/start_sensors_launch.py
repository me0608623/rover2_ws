#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os



def generate_launch_description():
    sensors_dir = get_package_share_directory('campusrover_sensors')

    yd_dir = get_package_share_directory('ydlidar_ros2_driver')
    yd_path = os.path.join(yd_dir, 'launch', 'ydlidar_launch.py')

    velodyne_dir = get_package_share_directory('velodyne')
    velodyne_path = os.path.join(velodyne_dir, 'launch', 'velodyne-all-nodes-VLP16-launch.py')

    realsense2_dir = get_package_share_directory('realsense2_camera')
    realsense2_path = os.path.join(realsense2_dir, 'launch', 'rs_launch.py')
    
    imu_dir = get_package_share_directory('handsfree_ros2_imu')
    imu_path = os.path.join(imu_dir, 'launch', 'handsfree_imu_launch.py')

    yd_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yd_path),
        launch_arguments={
            'params_file': os.path.join(sensors_dir, 'launch', 'config', 'ydlidar_front.yaml'), 
            'topic_name': 'scan_front',
            'node_name': 'ydlidar_front_node'
        }.items()
    )
    yd_back = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yd_path),
        launch_arguments={
            'params_file': os.path.join(sensors_dir, 'launch', 'config', 'ydlidar_back.yaml'), 
            'topic_name': 'scan_back',
            'node_name': 'ydlidar_back_node'
        }.items()
    )

    velodyne_points = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_path),
        launch_arguments={
            'params': os.path.join(sensors_dir, 'launch', 'config', 'velodyne_points.yaml'), 
        }.items()
    )

    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense2_path),
    )
    
    handsfree_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_path), 
        launch_arguments={ 
            'params_file': os.path.join(sensors_dir, 'launch', 'config', 'handsfree_imu.yaml'), 
        }.items()
    )
    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sensors_dir, 'launch', 'start_transform_launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui', default='false')
        }.items()
    )

    return LaunchDescription([
        yd_front,
        yd_back,
        velodyne_points,
        realsense2_camera,
        handsfree_imu,
        tf_static,
    ])

                                
