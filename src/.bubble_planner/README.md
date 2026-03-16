# bubble_planner

ROS 2 Humble 的局部規劃與控制套件，採用 bubble-based 避障路線搭配 ACADOS MPC 進行軌跡追蹤，可在 turtlesim 模擬或實機底盤上運行。

## 系統架構
- 模擬/測試：`turtlesim_driver.launch.py` 啟動世界座標 (`world->map->odom` 靜態 TF)、turtlesim、模擬里程計、全域路徑產生器與模型 Marker。
- 環境/成本圖：`costmap.launch.py` 可啟動模擬障礙物來源（`static_obs`、`dynamic_obs`、`tracked_obs`、`costmap_merge`）產生 `/costmap`。若 `dwa_mode:=false` 則會啟用障礙物轉換鏈（`map_to_obstacle_simple`、`mot_to_obstacle_simple`、`wall_and_dynamic_obstacle_map`、`split_obstacles_node`、`obstacle_filter`）。
- 路徑規劃：
  - `local_path_node`：截取 `/global_path` 形成 `/local_path`，同時輸出 `/global_path_end` 供後續節點參考。
  - `regular_points_node`：基於成本圖在路徑左右取樣可行空間，發布 `/regular_points_left`、`/regular_points_right`。
  - `path_builder_node`：使用局部/全域路徑與左右候選點建立圖搜索，生成 bubble path `/bubble_path` 與資訊 `/bubble_info`，並輸出除錯 Marker。
- 控制：`bubble_mpc_node` 以 ACADOS 求解器追蹤 `/bubble_path`，輸出速度指令（模擬為 `/turtle1/cmd_vel`，實機可改 `cmd_vel_topic`），並發布 `/mpc_predict_path`、`/bubble_mpc_finish` 等狀態。
- 可視化：RViz 佈局位於 `rviz/`，規劃與 MPC 節點亦發布 Marker 便於除錯。

## 避障策略（Path Builder）
- 障礙物間距需至少 1.4 m 才可通行。
- 三個狀態：
  - **INTERCEPT**：以約 30° 平滑切回全域路徑。
  - **AVOIDANCE**：使用左右候選點建圖並以 Dijkstra 搜索繞障。
  - **MAINTAIN**：沿用並修剪上一輪路徑，避免路徑抖動。
- 狀態切換：每輪預先檢查 `intercept_path` 與 `maintained_path` 的碰撞；`INTERCEPT` 撞障改做 `AVOIDANCE`，`AVOIDANCE` 計算一次後下一輪進入 `MAINTAIN`，`MAINTAIN` 若切入路徑無撞則立即回切，若維持路徑撞障則回到 `AVOIDANCE`。
- 關鍵檢查：`checkCostmapCollision` 會檢查 bubble 連續性與周邊安全區；`checkLineClear` 以取樣方式確認節點連線不穿越障礙。

## 環境需求
- ROS 2 Humble、colcon。
- 已編譯的 ACADOS，並設定環境變數 `ACADOS_INSTALL_DIR`（例：`export ACADOS_INSTALL_DIR=$HOME/acados`）。

## 建置
1. `cd ~/rover_ws`
2. `export ACADOS_INSTALL_DIR=...`（每次 build 前確認）
3. `colcon build --packages-select bubble_planner`
4. `source install/setup.bash`

## 操作說明
### 模擬流程（預設）
1. 啟動規劃、成本圖與 RViz：
   ```bash
   ros2 launch bubble_planner bubble_planner.launch.py simulator:=true
   ```
   - 預設 frame：`map`/`turtle1`，成本圖來源為模擬障礙物。
2. 另開終端啟動 MPC：
   ```bash
   ros2 launch bubble_planner mpc.launch.py simulator_mpc:=true
   ```
   - 追蹤 `/bubble_path`，速度輸出至 `/turtle1/cmd_vel`。

### 實機/外部地圖
1. 準備必要資料：`/global_path`、`/odom`、`/campusrover_local_costmap`（或自訂成本圖）、正確的 TF (`world -> map -> odom -> base_footprint`)。
2. 啟動規劃：
   ```bash
   ros2 launch bubble_planner bubble_planner.launch.py simulator:=false
   ```
   - 會將 frame 設為 `world`/`base_footprint`，成本圖訂閱 `/campusrover_local_costmap`。
3. 如需障礙物轉換鏈（非模擬來源），可單獨啟動：
   ```bash
   ros2 launch bubble_planner costmap.launch.py simulator_costmap:=false dwa_mode:=false debug:=false
   ```
4. 啟動 MPC（依需求覆寫輸出 Topic）：
   ```bash
   ros2 launch bubble_planner mpc.launch.py simulator_mpc:=false cmd_vel_topic:=/input/nav_cmd_vel input_path:=/bubble_path
   ```
   - 亦可透過 `target_frame_id`、`child_frame_id` 變更 TF 來源。

### 常用主題
- 輸入：`/global_path` (nav_msgs/Path)、`/costmap` (OccupancyGrid)、`/odom`。
- 規劃輸出：`/local_path`、`/regular_points_left|right`、`/bubble_path`、`/bubble_info`、`/bubble_marker`。
- 控制輸出：`/mpc_predict_path`、`/bubble_mpc_finish`、`/mpc_finish`、`/cmd_vel_topic`。
- 服務：`/enable_bubble_mpc` (std_srvs/SetBool) 用於開關 MPC。

### 實驗數據蒐集
- 節點（手動）：`experiment_logger`
  - 啟動：`ros2 run bubble_planner experiment_logger`
  - 操作：`/experiment_logger/start`、`/experiment_logger/stop`，摘要發布 `/experiment_logger/summary`。
  - 參數：`odom_topic`, `costmap_topic`, `cmd_vel_topic`, `path_topic`, `costmap_frame`, `collision_threshold`, `collision_radius`, `obstacle_search_radius`, `resource_pid` 等。
- 節點（自動）：`experiment_logger_v2`
  - 啟動：`ros2 run bubble_planner experiment_logger_v2`
  - 邏輯：收到第一筆 `cmd_vel` 自動開始；`cmd_vel` 超過 `cmd_vel_timeout`（預設 1s）未更新自動結束；結束時計算終點距離/方位差，若距離 >0.3m 且航向差 >0.1rad 判定為失敗。可用 `/experiment_logger/start` 重新 armed、`/experiment_logger/stop` 強制結束。
- 指標（兩版本）：完成時間、實際行走里程、最近一次規劃路徑長、碰撞次數、最小障礙物距離、總旋轉量、平滑度指數（航向變化平方平均）、控制頻率（cmd_vel）、CPU 平均佔用、記憶體使用（即時/最大/平均）。預設 CPU/記憶體監測本節點，可透過 `resource_pid` 指定目標進程。

## RViz
- 模擬佈局：`rviz/bubble_mpc_sim.rviz`（`bubble_planner.launch.py` 在模擬時自動開啟）。
- 成本圖除錯：`rviz/costmap.rviz`（可透過 `debug:=true` 啟動）。

## 附註
- 若編譯時找不到 ACADOS，請確認 `ACADOS_INSTALL_DIR` 路徑與動態庫是否可見。
- TF 查詢失敗時，請確認 `simulator` 參數對應的 frame（模擬為 `turtle1`，實機為 `base_footprint`）。 
