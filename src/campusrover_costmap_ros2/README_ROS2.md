# Campus Rover 成本地圖套件 ROS2 Foxy 版本 (campusrover_costmap_ros2)

## 概述

`campusrover_costmap_ros2` 是專為 Campus Rover 機器人設計的成本地圖生成套件，**專門針對 Ubuntu 20.04 + ROS2 Foxy** 環境優化。它提供了局部和全域成本地圖的生成功能，用於機器人導航系統的避障規劃。

**⚠️ 重要**: 此版本專為 **Ubuntu 20.04 LTS + ROS2 Foxy** 設計，如需其他 ROS2 版本，請參考兼容性指南。

## ROS2 版本變更

### 主要變更點

1. **ROS2 API 遷移**
   - `ros::NodeHandle` → `rclcpp::Node`
   - `ros::Publisher/Subscriber` → `rclcpp::Publisher/Subscription`
   - `ros::spin()` → `rclcpp::spin()`

2. **消息類型更新**
   - `nav_msgs::OccupancyGrid` → `nav_msgs::msg::OccupancyGrid`
   - `sensor_msgs::PointCloud2` → `sensor_msgs::msg::PointCloud2`
   - `sensor_msgs::LaserScan` → `sensor_msgs::msg::LaserScan`

3. **TF 系統升級**
   - `tf::TransformListener` → `tf2_ros::TransformListener`
   - `tf::TransformException` → `tf2::TransformException`

4. **參數系統**
   - `ros::NodeHandle::param()` → `rclcpp::Node::declare_parameter()`
   - YAML 配置文件格式更新

5. **Launch 系統**
   - XML Launch → Python Launch 文件
   - 更靈活的參數配置和條件啟動

## 套件結構

```
campusrover_costmap_ros2/
├── include/                        # 頭文件 (已更新為 ROS2)
│   ├── campusrover_costmap.h      # 局部成本地圖類定義
│   └── global_costmap.hpp         # 全域成本地圖類定義
├── src/                           # 源代碼 (已轉換為 ROS2)
│   ├── local_costmap.cpp          # 局部成本地圖實現
│   ├── local_costmap_node.cpp     # 局部成本地圖節點
│   ├── global_costmap.cpp         # 全域成本地圖實現
│   ├── global_costmap_node.cpp    # 全域成本地圖節點
│   └── laser_to_pointcloud2.cpp   # 雷射轉點雲節點
├── launch_ros2/                   # ROS2 Python Launch 文件
│   ├── local_costmap.launch.py    # 局部成本地圖啟動
│   └── global_costmap.launch.py   # 全域成本地圖啟動
├── config/                        # ROS2 參數配置文件
│   ├── local_costmap.yaml         # 局部成本地圖參數
│   └── global_costmap.yaml        # 全域成本地圖參數
├── launch/                        # ROS1 Launch 文件 (保留參考)
├── param/                         # ROS1 參數文件 (保留參考)
├── rviz/                          # RViz 配置
├── test_data/                     # 測試數據
├── CMakeLists.txt                 # ROS2 ament_cmake 配置
├── package.xml                    # ROS2 套件配置
├── README.md                      # ROS1 版本說明
└── README_ROS2.md                 # ROS2 版本說明 (本文件)
```

## 編譯和安裝

### 依賴項目

確保已安裝以下 ROS2 套件：
```bash
sudo apt install ros-foxy-rclcpp ros-foxy-std-msgs ros-foxy-sensor-msgs \
                 ros-foxy-nav-msgs ros-foxy-geometry-msgs ros-foxy-tf2 \
                 ros-foxy-tf2-ros ros-foxy-tf2-geometry-msgs \
                 ros-foxy-pcl-ros ros-foxy-pcl-conversions \
                 ros-foxy-laser-geometry ros-foxy-nav2-map-server \
                 ros-foxy-nav2-lifecycle-manager
```

### 編譯

```bash
cd ~/spot_ws
colcon build --packages-select campusrover_costmap_ros2
source install/setup.bash
```

## 使用方法

### 啟動局部成本地圖

```bash
# 標準模式
ros2 launch campusrover_costmap_ros2 local_costmap.launch.py

# 調試模式
ros2 launch campusrover_costmap_ros2 local_costmap.launch.py debug:=true

# 僅使用 Velodyne 雷達
ros2 launch campusrover_costmap_ros2 local_costmap.launch.py only_use_velodyne:=true
```

### 啟動全域成本地圖

```bash
# 使用預設測試地圖
ros2 launch campusrover_costmap_ros2 global_costmap.launch.py

# 使用自定義地圖
ros2 launch campusrover_costmap_ros2 global_costmap.launch.py map_file:=/path/to/your/map.yaml
```

### 使用參數文件

```bash
# 使用自定義參數啟動局部成本地圖
ros2 launch campusrover_costmap_ros2 local_costmap.launch.py \
    params_file:=$(ros2 pkg prefix campusrover_costmap_ros2)/share/campusrover_costmap_ros2/config/local_costmap.yaml
```

## ROS2 話題接口

### 訂閱的話題
- `/points2` (sensor_msgs/msg/PointCloud2): 輸入點雲數據
- `/laser` (sensor_msgs/msg/LaserScan): 輸入雷射掃描數據
- `/map` (nav_msgs/msg/OccupancyGrid): 輸入靜態地圖

### 發布的話題
- `/campusrover_local_costmap` (nav_msgs/msg/OccupancyGrid): 局部成本地圖
- `/global_costmap` (nav_msgs/msg/OccupancyGrid): 全域成本地圖
- `/pointcloud` (sensor_msgs/msg/PointCloud2): 轉換後的點雲

## 參數配置

### 局部成本地圖參數

可以通過 YAML 文件或 launch 文件設定：

```yaml
local_costmap_node:
  ros__parameters:
    base_frame: "base_link"
    map_size_width: 6.0
    map_size_height: 6.0
    map_resolution: 0.02
    max_obstacle_height: 2.0
    min_obstacle_height: -1.2
    inflation_radius: 0.15
    cost_scaling_factor: 10.0
    pcl_viz: false
```

### 全域成本地圖參數

```yaml
global_costmap_node:
  ros__parameters:
    costmap_resolution: 0.5
    inflation_radius: 0.3
    cost_scaling_factor: 10.0
```

## ROS2 特有功能

### 1. 參數動態重配置
```bash
# 查看當前參數
ros2 param list /local_costmap_node

# 動態修改參數
ros2 param set /local_costmap_node inflation_radius 0.2
```

### 2. 服務和動作 (未來擴展)
ROS2 版本為未來添加服務和動作接口預留了空間。

### 3. 生命週期管理
支援 ROS2 的生命週期節點管理（未來版本）。

## 調試和故障排除

### 查看節點狀態
```bash
ros2 node list
ros2 node info /local_costmap_node
```

### 查看話題
```bash
ros2 topic list
ros2 topic echo /campusrover_local_costmap
```

### 查看 TF 樹
```bash
ros2 run tf2_tools view_frames
```

## 與 ROS1 版本的差異

| 功能        | ROS1 版本      | ROS2 版本 |
|------      | -----------    |-----------|
| 節點繼承     | 無            | 繼承自 `rclcpp::Node` |
| 參數系統     | `ros::param`  | `declare_parameter()` + `get_parameter()` |
| TF 系統     | TF1           | TF2 | 
| Launch 格式 | XML           | Python   |
| 消息命名空間  | `nav_msgs::` | `nav_msgs::msg::` |
| 智能指針     | `boost::shared_ptr` | `std::shared_ptr` |
| 回調綁定     | `boost::bind` | `std::bind` |

## 已知問題和限制

1. **PCL 可視化**: 目前 PCL 可視化功能可能需要額外配置
2. **數據包回放**: ROS2 的 rosbag 格式與 ROS1 不同，需要轉換
3. **Nav2 整合**: 與 Nav2 導航堆疊的整合仍在開發中

## 開發計劃

- [ ] 添加 ROS2 服務接口
- [ ] 實現生命週期節點
- [ ] Nav2 costmap plugin 整合
- [ ] 參數動態重配置界面
- [ ] 單元測試和整合測試

## 維護者

**Campus Rover Team**  
Email: luke@campusrover.org

## 授權

MIT License

---

*ROS2 Foxy 版本 - 2024年*
