#!/bin/bash
# RL Policy 完整啟動腳本
#
# 資料流：
#   map_server → /map
#     → global_costmap_node → /global_costmap
#       → MOT (is_map_filter:True)  → /tracked_label_obstacle
#   ndt_localizer → /ndt_pose → /tf (map→odom)
#   aitstar_planner → /global_path
#   rl_policy_node → /cmd_vel
#
# 使用方式：
#   ./launch_rl.sh                          # 實車模式（use_sim_time=false）
#   ./launch_rl.sh --sim                    # 模擬模式（use_sim_time=true）
#   ./launch_rl.sh --map /path/to/map.yaml  # 指定地圖
#   ./launch_rl.sh --sim --map /path/to/map.yaml

# ── 參數解析 ────────────────────────────────────────────────────────────────
USE_SIM_TIME="false"
MAP_FILE="/home/aa/maps/4v3F.yaml"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)      USE_SIM_TIME="true"; shift ;;
        --map)      MAP_FILE="$2"; shift 2 ;;
        *)          echo "Unknown argument: $1"; exit 1 ;;
    esac
done

WORKSPACE="/home/aa/rover2_ws"
PIDS=()

source /opt/ros/jazzy/setup.bash
source /home/aa/Documents/jazzy_ws/install/setup.bash 2>/dev/null || true
source "${WORKSPACE}/install/setup.bash"

# ── 檢查必要條件 ────────────────────────────────────────────────────────────────
echo "檢查傳感器數據源..."
sleep 2

MISSING_TOPICS=()

# 檢查必要的 Topics（不管來自實車還是 Isaac Sim）
if ! timeout 1 ros2 topic list 2>/dev/null | grep -q "velodyne_points"; then
    MISSING_TOPICS+=("velodyne_points")
fi

if ! timeout 1 ros2 topic list 2>/dev/null | grep -q "odom"; then
    MISSING_TOPICS+=("odom")
fi

if [ ${#MISSING_TOPICS[@]} -gt 0 ]; then
    echo ""
    echo "❌ 錯誤：缺少必要的傳感器數據"
    echo "未找到：${MISSING_TOPICS[*]}"
    echo ""
    echo "請檢查："
    if [ "${USE_SIM_TIME}" = "true" ]; then
        echo "  • Isaac Sim 是否在運行："
        echo "    cd ~/IsaacLab && ./isaaclab.sh -p scripts/reinforcement_learning/skrl/play_charge_ros2_bridge.py"
    else
        echo "  • 實車傳感器是否在線（velodyne_driver, 編碼器驅動等）"
    fi
    exit 1
fi

cleanup() {
    echo ""
    echo "Shutting down all nodes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "========================================"
echo " RL Policy 完整啟動"
echo " 模式: $([ "$USE_SIM_TIME" = "true" ] && echo "模擬 (sim time)" || echo "實車 (wall time)")"
echo " 地圖: ${MAP_FILE}"
echo "========================================"

# ── 清理殘留 process（避免 lifecycle_manager 遇到舊的 active map_server）──────
echo "Cleaning up leftover nodes..."
pkill -f "nav2_map_server" 2>/dev/null
pkill -f "lifecycle_manager" 2>/dev/null
pkill -f "ndt_localizer_node" 2>/dev/null
pkill -f "voxel_grid_filter_node" 2>/dev/null
pkill -f "campusrover_mot_node" 2>/dev/null
pkill -f "aitstar_planner" 2>/dev/null
pkill -f "rl_policy" 2>/dev/null
sleep 1

# ── 1+2. Map Server + Global Costmap（map_server/lifecycle 由 global_costmap.launch 統一管理）──
echo ""
echo "[1/7] map_server + global_costmap_node"
ros2 launch campusrover_costmap_ros2 global_costmap.launch.py \
    map_file:="${MAP_FILE}" \
    use_sim_time:="${USE_SIM_TIME}" &
PIDS+=($!)
sleep 2

# ── 3. NDT Localizer（定位：提供 map→odom tf）────────────────────────────────
echo "[3/7] ndt_localizer → /ndt_pose + tf"
ros2 launch ndt_localizer ndt_localizer_launch.py \
    use_sim_time:="${USE_SIM_TIME}" \
    leaf_size:=0.5 \
    crop_max_z:=2.0 \
    crop_min_z:=-2.0 \
    converged_param_transform_probability:=1.2 &
PIDS+=($!)
sleep 2

# ── 關閉 Isaac Sim viewport 釋放 VRAM（sim 模式下，用 RViz 觀察即可）────────
if [ "${USE_SIM_TIME}" = "true" ]; then
    echo "[GPU] 關閉 Isaac Sim viewport 釋放 VRAM..."
    timeout 10s python3 -c "
import socket, json, sys
s = socket.socket(); s.settimeout(5)
try:
    s.connect(('localhost', 8766))
    cmd = json.dumps({'type': 'execute_script', 'params': {'code': '''
import omni.kit.viewport.utility as vp_util
import carb.settings
for vp in vp_util.get_viewport_window_instances():
    vp.visible = False
s = carb.settings.get_settings()
s.set(\"/app/renderer/enabled\", False)
result = \"viewport disabled\"
'''}})
    s.sendall(cmd.encode())
    import time; time.sleep(2)
    s.recv(4096)
except: pass
finally: s.close()
" 2>/dev/null || true
    sleep 1
fi

# ── RViz（與 NDT 同時開，讓使用者可立即用 2D Pose Estimate 給初始位置）──────
echo "[RViz] 啟動 RViz（請用 2D Pose Estimate 給 NDT 初始位置）"
ros2 run rviz2 rviz2 -d /home/aa/demo.rviz \
    --ros-args -p use_sim_time:="${USE_SIM_TIME}" 2>/dev/null &
PIDS+=($!)

# ── 等待 NDT 載入 PCD map 和建立 TF（約 15 秒）────────────────────────────────
echo "等待 NDT 載入地圖和 TF 變換..."
echo "（RViz 中應能看到 /velodyne_points 點雲）"
sleep 15

# ── Local Costmap（/velodyne_points → /campusrover_local_costmap）─────────────
echo "[local] local_costmap_node → /campusrover_local_costmap"
ros2 launch campusrover_costmap_ros2 local_costmap.launch.py \
    pointcloud_topic:=/velodyne_points &
PIDS+=($!)
sleep 1

# ── 4. MOT（/velodyne_points + /global_costmap → /tracked_label_obstacle）────
echo "[4/7] campusrover_mot_node → /tracked_label_obstacle"
ros2 run campusrover_mot campusrover_mot_node \
    --ros-args \
    -r points:=/velodyne_points \
    -r /scan:=/scan \
    -p use_sim_time:="${USE_SIM_TIME}" \
    -p detection_area_min_x:=-10.0 \
    -p detection_area_max_x:=10.0 \
    -p detection_area_min_y:=-10.0 \
    -p detection_area_max_y:=10.0 \
    -p detection_area_min_z:=0.05 \
    -p detection_area_max_z:=0.5 \
    -p track_dead_time:=1.0 \
    -p cluster_dist:=0.35 \
    -p false_alarm_min:=15 \
    -p false_alarm_max:=3000 \
    -p min_publish_age:=0.3 \
    -p min_detection_count:=3 \
    -p history_length:=10 \
    -p speed_threshold:=0.1 \
    -p map_frame:=map \
    -p laser_frame:=scan \
    -p is_use_laser:=false \
    -p is_map_filter:=true \
    -p is_img_label:=false \
    -p only_dynamic_obstacle:=false \
    -p debug_mode:=false &
PIDS+=($!)
sleep 1

# ── 5. AIT* Global Planner（/odom + /map → /global_path）────────────────────
echo "[5/7] aitstar_planner → /global_path"
ros2 launch aitstar_path_planner aitstar_planner.launch.py \
    use_sim_time:="${USE_SIM_TIME}" \
    enable_visualization:=true &
PIDS+=($!)
sleep 2

# ── 6. RL Policy Node─────────────────────────────────────────────────────────
echo "[6/7] rl_policy_node → /cmd_vel"
ros2 launch campusrover_rl_policy rl_policy_launch.py &
PIDS+=($!)

echo ""
echo "========================================"
echo " 全部啟動完成！"
echo ""
echo " 模式          : $([ "$USE_SIM_TIME" = "true" ] && echo "模擬 (Isaac Sim)" || echo "實車")"
echo " Topics 確認："
echo "   ✓ /map                    ← map_server"
echo "   ✓ /global_costmap         ← global_costmap_node"
echo "   ✓ /ndt_pose               ← ndt_localizer (等待初始位置)"
echo "   ✓ /tracked_label_obstacle ← MOT"
echo "   ✓ /global_path            ← AIT* (收到 /goal_pose 後)"
echo "   ✓ /cmd_vel                ← RL policy"
echo ""
echo " === 使用步驟 ==="
echo " 1. 在 RViz 中使用「2D Pose Estimate」工具設置機器人初始位置"
echo "    （點雲應在座標軸處可見）"
echo ""
echo " 2. NDT 收到初始位置後會開始定位"
echo ""
echo " 3. 定位收斂後，發送導航目標："
echo "    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "      \"{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}}}\" --once"
echo ""
echo " 4. 監控 RL 策略輸出："
echo "    ros2 topic echo /cmd_vel"
echo ""
echo "========================================"

wait
