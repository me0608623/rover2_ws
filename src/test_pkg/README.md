# 測試數據蒐集
- `ros2 launch test_pkg logger.launch.py bubble:=true or false`

## planner data collector
### 觸發條件 (Trigger Conditions)
- 開始條件
    1. 待命 (Arming)：按下遙控器 **A** 鍵 (Button 0)，節點進入 Armed 狀態。
    2. 觸發 (Trigger)：監聽到 Topic /input/nav_cmd_vel 發布**非零**的速度指令。
    
- 中止條件
    - 手動結束：按下遙控器 **B** 鍵 (Button 1)。
    - 長時間靜止：**Topic /input/nav_cmd_vel** 持續 1 秒 線速度與角速度皆為 0。
    - 通訊中斷：**Topic /input/nav_cmd_vel** 超過 1 秒 未收到任何訊息 (Timeout)。
---
### 蒐集數據列表 (Collected Data)
在結束時發布 **topic /planner_data_collector/all_data**
- 實驗結果與時間
    | 型態     | 名稱 | 說明 |
    | ------- | ---- | ---- |
    | bool    | arrive      | 導航成功判定 |
    | string  | stop_reason | 結束原因    |
    | float64 | time_spent  | 總耗時      |
- 路徑與運動指標
    | 型態     | 名稱 | 說明 |
    | ------- | ---- | ---- |
    | float64 | global_path_length   | 全局路徑長度 |
    | float64 | traveled_path_length | 實際行駛里程 |
    | float64 | total_rotation_rad   | 總旋轉量 |
    | float64 | smoothness_index     | 路徑平滑度指標 |
- 安全與精確度
    | 型態     | 名稱 | 說明 |
    | ------- | ---- | ---- |
    | float64 | min_distance_to_obstacle | 最近障礙物距離 |
    | float64 | distance_to_goal         | 終點距離誤差 |
    | float64 | yaw_difference_to_goal   | 終點角度誤差 |
- 控制頻率
    | 型態     | 名稱 | 說明 |
    | ------- | ---- | ---- |
    | float64 | avg_control_frequency | bubble 頻率 |
    | float64 | avg_cmd_vel_frequency | 速度指令頻率 |
- 系統資源消耗 (支援監控最多 4 個 Process)
    | 型態     | 名稱 | 說明 |
    | ------- | ---- | ---- |
    | float64 | avg_cpu_usage    | 平均 cpu 使用率 |
    | float64 | avg_memory_usage | 平均記憶體使用率 |
### **topic /planner_data_collector/resource_usage** 發布即時
| 型態     | 名稱 | 說明 |
| ------- | ---- | ---- |
| float64 | avg_cpu_usage         | 即時 cpu 使用率 |
| float64 | avg_memory_usage      | 即時記憶體使用率 |
| float64 | avg_control_frequency | 即時 bubble 頻率 |
| float64 | avg_cmd_vel_frequency | 即時速度指令頻率 |
    
## otherwise