# Campus Rover 成本地圖套件 (campusrover_costmap)

## 概述

`campusrover_costmap` 是專為 Campus Rover 機器人設計的成本地圖生成套件。它提供了局部和全域成本地圖的生成功能，用於機器人導航系統的避障規劃。

## 主要功能

### 1. 局部成本地圖 (Local Costmap)
- 從即時感測器數據 (點雲) 生成局部成本地圖
- 支援多種點雲濾波和處理技術
- 提供多線程並行處理以提高效率
- 可選的 PCL 可視化功能

### 2. 全域成本地圖 (Global Costmap)
- 從靜態地圖生成全域成本地圖
- 支援地圖降採樣以適應不同解析度需求
- 障礙物膨脹處理確保安全導航

### 3. 感測器數據轉換
- 將 2D 雷射掃描數據轉換為點雲格式
- 支援多種感測器輸入 (2D LiDAR, 3D Velodyne)

## 套件結構

```
campusrover_costmap/
├── include/                     # 頭文件
│   ├── campusrover_costmap.h   # 局部成本地圖類定義
│   └── global_costmap.hpp      # 全域成本地圖類定義
├── src/                        # 源代碼
│   ├── local_costmap.cpp       # 局部成本地圖實現
│   ├── local_costmap_node.cpp  # 局部成本地圖節點
│   ├── global_costmap.cpp      # 全域成本地圖實現
│   ├── global_costmap_node.cpp # 全域成本地圖節點
│   └── laser_to_pointcloud2.cpp # 雷射轉點雲節點
├── launch/                     # 啟動文件
│   ├── start_costmap.launch    # 局部成本地圖啟動
│   └── global_costmap.launch   # 全域成本地圖啟動
├── param/                      # 參數文件
│   └── local_costmap.yaml      # 局部成本地圖參數
├── rviz/                       # RViz 配置
│   ├── campusrover_costmap.rviz
│   ├── global_costmap.rviz
│   └── autolabor_costmap.rviz
├── test_data/                  # 測試數據
│   ├── stata.pgm              # 測試地圖圖像
│   └── stata.yaml             # 測試地圖配置
├── CMakeLists.txt             # 編譯配置
├── package.xml                # 套件配置
└── README.md                  # 說明文件
```

## 核心類別

### CampusroverCostmap
局部成本地圖生成器，主要功能：
- **點雲處理流水線**: 座標變換 → 範圍裁剪 → 降採樣 → 成本計算
- **多線程處理**: 使用 8 個線程並行計算障礙物膨脹
- **濾波器**: 支援通過濾波器、裁剪盒濾波器、體素網格濾波器
- **機器人本體移除**: 自動移除機器人本體範圍內的點雲

### GlobalCostmap  
全域成本地圖生成器，主要功能：
- **地圖降採樣**: 支援不同解析度的地圖轉換
- **障礙物膨脹**: 對靜態地圖中的障礙物進行安全膨脹
- **成本計算**: 根據距離計算適當的成本值

## 節點說明

### 1. local_costmap_node
- **功能**: 生成局部成本地圖
- **輸入**: 點雲數據 (`points2`)
- **輸出**: 局部成本地圖 (`campusrover_local_costmap`)
- **特色**: 可選 PCL 可視化支援

### 2. global_costmap_node  
- **功能**: 生成全域成本地圖
- **輸入**: 靜態地圖 (`map`)
- **輸出**: 全域成本地圖 (`global_costmap`)

### 3. laser_to_pointcloud2_node
- **功能**: 雷射掃描轉點雲
- **輸入**: 雷射掃描 (`laser`)
- **輸出**: 點雲數據 (`pointcloud`)

## 使用方法

### 啟動局部成本地圖
```bash
# 標準模式
roslaunch campusrover_costmap start_costmap.launch

# 調試模式 (使用錄製數據)
roslaunch campusrover_costmap start_costmap.launch debug:=true

# 僅使用 Velodyne 雷達
roslaunch campusrover_costmap start_costmap.launch only_use_velodyne:=true
```

### 啟動全域成本地圖
```bash
# 使用預設測試地圖
roslaunch campusrover_costmap global_costmap.launch

# 使用自定義地圖
roslaunch campusrover_costmap global_costmap.launch map_file:=/path/to/your/map.yaml
```

## 參數配置

### 局部成本地圖參數
| 參數名稱 | 類型 | 預設值 | 說明 |
|---------|------|--------|------|
| `map_size_width` | double | 6.0 | 地圖寬度 (公尺) |
| `map_size_height` | double | 6.0 | 地圖高度 (公尺) |
| `map_resolution` | double | 0.02 | 地圖解析度 (公尺/像素) |
| `max_obstacle_height` | double | 2.0 | 障礙物最大高度 (公尺) |
| `min_obstacle_height` | double | -1.2 | 障礙物最小高度 (公尺) |
| `inflation_radius` | double | 0.02 | 膨脹半徑 (公尺) |
| `cost_scaling_factor` | double | 10.0 | 成本縮放因子 |
| `pcl_viz` | bool | false | 是否啟用 PCL 可視化 |

### 全域成本地圖參數
| 參數名稱 | 類型 | 預設值 | 說明 |
|---------|------|--------|------|
| `costmap_resolution` | double | 0.5 | 成本地圖解析度 (0=使用原始解析度) |
| `inflation_radius` | double | 0.3 | 膨脹半徑 (公尺) |
| `cost_scaling_factor` | double | 10.0 | 成本縮放因子 |

## 話題接口

### 訂閱的話題
- `/points2` (sensor_msgs/PointCloud2): 輸入點雲數據
- `/laser` (sensor_msgs/LaserScan): 輸入雷射掃描數據
- `/map` (nav_msgs/OccupancyGrid): 輸入靜態地圖

### 發布的話題
- `/campusrover_local_costmap` (nav_msgs/OccupancyGrid): 局部成本地圖
- `/global_costmap` (nav_msgs/OccupancyGrid): 全域成本地圖
- `/pointcloud` (sensor_msgs/PointCloud2): 轉換後的點雲

## 技術特點

### 1. 高效能處理
- **多線程並行**: 使用 8 個線程並行處理障礙物膨脹
- **體素網格降採樣**: 減少點雲數量，提高處理速度
- **靜態變數快取**: 避免重複計算，提升效率

### 2. 精確的機器人模型
- **本體尺寸**: 長 0.65m × 寬 1.0m × 高 1.9m
- **自動移除**: 自動移除機器人本體範圍內的點雲
- **安全膨脹**: 確保導航路徑的安全性

### 3. 靈活的配置
- **多感測器支援**: 2D LiDAR、3D Velodyne
- **可調參數**: 地圖尺寸、解析度、膨脹半徑等
- **調試模式**: 支援離線數據包測試

## 算法說明

### 障礙物膨脹算法
1. **障礙物檢測**: 從點雲或靜態地圖中提取障礙物位置
2. **距離計算**: 計算每個網格到最近障礙物的歐幾里得距離
3. **成本計算**: 使用指數衰減函數或固定值計算成本
4. **膨脹應用**: 在障礙物周圍設定適當的成本值

### 點雲處理流水線
1. **座標變換**: 將點雲從感測器座標系變換到機器人基礎座標系
2. **範圍裁剪**: 移除超出地圖範圍和機器人本體的點
3. **高度過濾**: 只保留指定高度範圍內的點
4. **降採樣**: 使用體素網格減少點的數量
5. **投影**: 將 3D 點雲投影到 2D 網格

## 依賴套件

- **ROS Core**: roscpp, rospy, std_msgs, sensor_msgs, nav_msgs
- **點雲處理**: PCL (Point Cloud Library), pcl_ros, pcl_conversions
- **幾何轉換**: laser_geometry, tf, tf2_ros
- **地圖服務**: map_server
- **可視化**: rviz, pcl_visualization

## 開發注意事項

### 已知問題
1. `computeMatDis` 函數中 `map_size_h` 使用了 `map_size_width_` 而非 `map_size_height_`
2. 全域成本地圖的 `computeCost` 函數目前使用固定值而非指數衰減
3. 局部成本地圖的成本計算被簡化為固定值 128

### 建議改進
1. 修正高度計算錯誤
2. 實現完整的指數衰減成本函數
3. 添加動態參數重配置支援
4. 優化多線程負載平衡

## 測試和調試

### 使用測試數據
套件包含 Stata 建築的測試地圖，可用於功能測試：
```bash
roslaunch campusrover_costmap global_costmap.launch
```

### 可視化
- **RViz**: 查看成本地圖和機器人狀態
- **PCL Viewer**: 查看原始點雲數據 (設定 `pcl_viz:=true`)

### 調試模式
啟用調試模式可以使用錄製的數據包進行離線測試：
```bash
roslaunch campusrover_costmap start_costmap.launch debug:=true
```

## 維護者

**Campus Rover Team**  
Email: luke@campusrover.org

## 授權

MIT License

---

*最後更新：2024年*
