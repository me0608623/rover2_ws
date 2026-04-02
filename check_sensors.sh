#!/bin/bash
# 檢查傳感器數據可用性

set -e

source /opt/ros/jazzy/setup.bash

echo "========================================"
echo " 傳感器數據檢查"
echo "========================================"
echo ""

# 顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_topic() {
    local topic=$1
    local description=$2

    if timeout 1 ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} ${description}"
        echo "  Topic: ${topic}"

        # 嘗試取得一個消息
        if timeout 2 ros2 topic echo "${topic}" --once 2>/dev/null | head -3; then
            echo ""
        fi
        return 0
    else
        echo -e "${RED}✗${NC} ${description} 不可用"
        echo "  期望: ${topic}"
        echo ""
        return 1
    fi
}

check_tf() {
    local parent=$1
    local child=$2

    echo -n "  檢查 TF: ${parent} → ${child} ... "
    if timeout 2 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>/dev/null | head -1 | grep -q "Listening"; then
        echo -e "${GREEN}✓${NC}"
        timeout 1 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>/dev/null | head -5
        echo ""
        return 0
    else
        echo -e "${RED}✗${NC}"
        return 1
    fi
}

# 檢查基本 Topics
echo "=== 檢查基本 Topics ==="
check_topic "/velodyne_points" "LiDAR 點雲"
check_topic "/odom" "里程計"
check_topic "/clock" "模擬時間（用 use_sim_time=true）"
echo ""

# 檢查 TF
echo "=== 檢查 TF 樹 ==="
check_tf "map" "base_link"
check_tf "odom" "base_link"
check_tf "base_link" "velodyne"
echo ""

# 檢查可選 Topics
echo "=== 檢查可選 Topics ==="
if timeout 1 ros2 topic list 2>/dev/null | grep -q "tracked_label_obstacle"; then
    echo -e "${GREEN}✓${NC} 動態障礙物"
else
    echo -e "${YELLOW}◇${NC} 動態障礙物（可選）"
fi

if timeout 1 ros2 topic list 2>/dev/null | grep -q "global_path"; then
    echo -e "${GREEN}✓${NC} 全局路徑規劃"
else
    echo -e "${YELLOW}◇${NC} 全局路徑規劃（需要啟動 AIT*）"
fi

echo ""
echo "========================================"
echo " 檢查完成"
echo "========================================"
echo ""
echo "如果有 ✗ 標記的項目，請檢查："
echo "  • Isaac Sim 橋接是否運行："
echo "    cd ~/IsaacLab"
echo "    ./isaaclab.sh -p scripts/reinforcement_learning/skrl/play_charge_ros2_bridge.py"
echo "  • rover2_ws/launch_rl.sh 是否已啟動"
echo ""
