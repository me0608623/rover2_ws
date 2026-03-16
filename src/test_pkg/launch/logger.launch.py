import os
import psutil
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---------------------------------------------------------
    # 1. 定義監控目標與自動判斷邏輯
    # ---------------------------------------------------------
    target_nodes = {
        "local_path": "local_path_node",
        "regular_points": "regular_points_node",
        "path_builder": "path_builder_node",  
        "bubble_mpc": "bubble_mpc_node",
        "dwa_group": "dwa_planner"  
    }

    found_pids = []
    found_names = []
    
    print("\n[Experiment Launch] Scanning for processes...")
    
    # 掃描系統所有 Process
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            p_name = proc.info['name']
            p_cmd = proc.info['cmdline']
            p_pid = proc.info['pid']
            
            cmd_str = " ".join(p_cmd) if p_cmd else ""
            
            for tag, keyword in target_nodes.items():
                if tag in found_names: 
                    continue
                
                if keyword in p_name or keyword in cmd_str:
                    found_pids.append(p_pid)
                    found_names.append(tag)
                    print(f"  -> Found [{tag}]: PID {p_pid} (cmd: {p_name})")
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    if not found_pids:
        print("WARNING: No target nodes found! Monitoring self only.")
    
    # ---------------------------------------------------------
    # 2. 啟動 Node 與 Bag Recorder
    # ---------------------------------------------------------
    return LaunchDescription([
        Node(
            package='test_pkg', 
            executable='planner_data_collector',
            name='planner_data_collector',
            output='screen',
            parameters=[{
                'target_pids': found_pids,
                'target_names': found_names,
                'costmap_topic': '/campusrover_local_costmap', 
                'cmd_vel_topic': '/input/nav_cmd_vel', 
            }]
        )
    ])