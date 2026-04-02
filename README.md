# rover2_ws - RL Policy Navigation System

ROS2 导航栈，集成 RL 策略、NDT 定位、AIT* 规划和 MOT 目标跟踪。支持实车和 Isaac Sim 模拟。

## 快速开始

### Isaac Sim 模拟模式

**终端 1：启动 Isaac Sim**
```bash
cd ~/IsaacLab
./isaacrunmcp.sh
```

等待看到：
```
[IsaacSimBridge: Ready for simulation]
```

**终端 2：启动 ROS2 导航栈**
```bash
cd ~/rover2_ws
./launch_rl.sh --sim
```

### 实车模式

确保所有传感器驱动已启动，然后：
```bash
./launch_rl.sh
```

## 使用步骤

1. **RViz 中设置初始位置**
   - 使用「2D Pose Estimate」工具在地图上标记机器人位置
   - 点云应在该位置可见

2. **等待 NDT 定位收敛**
   - 监控：`ros2 topic echo /ndt_pose`

3. **发送导航目标**
   ```bash
   ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}}}" --once
   ```

4. **监控执行**
   ```bash
   # RL 策略输出
   ros2 topic echo /cmd_vel
   
   # 全局路径
   ros2 topic echo /global_path
   
   # 动态障碍物
   ros2 topic echo /tracked_label_obstacle
   ```

## 系统架构

```
数据流：
├─ map_server → /map
│  └─ global_costmap_node → /global_costmap
│     └─ MOT → /tracked_label_obstacle
├─ 传感器驱动/Isaac Sim → /odom + /velodyne_points
├─ ndt_localizer → /ndt_pose + /tf (map→odom)
├─ aitstar_planner → /global_path
└─ rl_policy_node → /cmd_vel (输出)
```

## 软件包说明

| 包 | 功能 |
|---|------|
| `campusrover_costmap_ros2` | 全局和局部成本地图 |
| `ndt_localizer` | NDT 点云定位 |
| `campusrover_mot` | 多目标跟踪（动态障碍物） |
| `aitstar_path_planner` | AIT* 全局路径规划 |
| `campusrover_rl_policy` | RL 策略推理节点 |

## 配置

### 地图文件
```bash
./launch_rl.sh --map /path/to/your/map.yaml
```

### NDT 参数（launch_rl.sh）
- `leaf_size`: 0.5（点云下采样）
- `crop_max_z`: 2.0（点云高度上限）
- `crop_min_z`: -2.0（点云高度下限）
- `converged_param_transform_probability`: 1.2（收敛阈值）

### RL 策略参数（src/campusrover_rl_policy/launch/rl_policy_launch.py）
- `device`: cpu（避免 VRAM 耗尽）
- `goal_timeout`: 300.0（秒）

## 故障排除

### Isaac Sim 未启动
```bash
# 检查 Isaac Sim 是否在运行
ros2 topic list | grep velodyne
```

### RViz 看不到点云
1. 检查 Fixed Frame 是否为 `map`
2. 检查 PointCloud2 display 是否启用
3. 检查 TF 树是否完整：`ros2 run tf2_tools view_frames.py`

### NDT 不收敛
- 使用 2D Pose Estimate 手动设置初始位置
- 确保点云质量足够（避免走廊等低特征区域）

## 关键改动

### TF 变换初始化
`src/ndt_localizer/launch/tf_static_launch.py` 不再发布静态 TF，由传感器层处理。

### 启动检查
`launch_rl.sh` 验证必要的 ROS Topics 可用：
- `/velodyne_points`
- `/odom`

如果缺失，显示清晰的错误提示。

### 初始化时间
启动时等待 15 秒让 NDT 加载地图和建立 TF 树。

## 环境要求

- ROS 2 Jazzy
- Isaac Sim（仅模拟模式）
- NVIDIA GPU（推荐 RTX 3090+）
- 至少 32GB RAM

## 已验证

- ✓ Isaac Sim 模拟启动
- ✓ RViz 可视化
- ✓ ROS2 Topics 发布
- ✓ TF 树完整
- ✓ RL 策略推理
