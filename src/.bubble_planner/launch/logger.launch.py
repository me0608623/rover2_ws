import os
import psutil
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. 定義您要監控的 Node 關鍵字
    #    格式: {"顯示名稱": "系統Process名稱關鍵字"}
    # ---------------------------------------------------------
    target_nodes = {
        "local_path": "local_path_node",
        "regular_points": "regular_points_node",
        "path_builder": "path_builder_node",
        "bubble_mpc": "bubble_mpc_node",
        "dwa_group": "dwa_planner"  # [新增] 監控 dwa_node
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
            
            # 檢查 cmdline 是否包含我們的關鍵字 (比對名稱更準)
            cmd_str = " ".join(p_cmd) if p_cmd else ""
            
            for tag, keyword in target_nodes.items():
                # 防止重複加入 (同一 tag 只抓一個 PID，或根據需求修改)
                if tag in found_names: 
                    continue
                
                # 判斷關鍵字是否存在於程序名稱或指令中
                if keyword in p_name or keyword in cmd_str:
                    found_pids.append(p_pid)
                    found_names.append(tag)
                    print(f"  -> Found [{tag}]: PID {p_pid} (cmd: {p_name})")
                    
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    if not found_pids:
        print("WARNING: No target nodes found! Monitoring self only.")

    # ---------------------------------------------------------
    # 2. 啟動 Experiment Logger Node
    # ---------------------------------------------------------
    return LaunchDescription([
        Node(
            package='bubble_planner', 
            executable='experiment_logger',
            name='experiment_logger',
            output='screen',
            parameters=[{
                'target_pids': found_pids,    # 傳遞 PID 列表
                'target_names': found_names,  # 傳遞對應名稱
                'costmap_topic': '/campusrover_local_costmap', 
                'cmd_vel_topic': '/input/nav_cmd_vel', 
                }]
        )
    ])