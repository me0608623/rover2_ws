# Campus Rover 成本地圖 ROS2 Foxy 兼容性指南

## 🎯 目標環境

**專為以下環境設計和測試**：
- **作業系統**: Ubuntu 20.04 LTS (Focal Fossa)
- **ROS版本**: ROS2 Foxy Fitzroy
- **編譯器**: GCC 9.x
- **CMake**: 3.16+ (Ubuntu 20.04 預設)
- **Python**: 3.8

---

## 🔧 Foxy 特定配置

### 1. 依賴項目差異

#### 包含文件變更
```cpp
// Foxy 版本使用 .h 而不是 .hpp
#include <laser_geometry/laser_geometry.h>        // 不是 .hpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 不是 .hpp
```

#### 套件依賴
```xml
<!-- Foxy 中的 Nav2 支援 -->
<!-- <depend>nav2_map_server</depend> -->
<!-- 如果需要地圖服務器，在 Foxy 中使用： -->
<!-- <depend>map_server</depend> -->
```

### 2. CMake 配置

```cmake
# Foxy 兼容的 CMake 最低版本
cmake_minimum_required(VERSION 3.5)

# C++14 標準 (Foxy 預設)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
```

---

## 🚀 安裝指南 (Ubuntu 20.04)

### 1. 安裝 ROS2 Foxy

```bash
# 更新系統
sudo apt update && sudo apt upgrade -y

# 安裝 ROS2 Foxy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release

# 添加 ROS2 GPG 金鑰
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安裝 ROS2 Foxy
sudo apt update
sudo apt install ros-foxy-desktop
```

### 2. 安裝專案依賴

```bash
# 核心依賴
sudo apt install -y \
  ros-foxy-rclcpp \
  ros-foxy-std-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-geometry-msgs \
  ros-foxy-tf2 \
  ros-foxy-tf2-ros \
  ros-foxy-tf2-geometry-msgs \
  ros-foxy-pcl-ros \
  ros-foxy-pcl-conversions \
  ros-foxy-laser-geometry

# PCL 相關
sudo apt install -y \
  libpcl-dev \
  libpcl-conversions-dev

# Eigen3
sudo apt install -y libeigen3-dev

# 編譯工具
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

### 3. 可選：Nav2 支援

```bash
# 如果需要完整的 Nav2 支援
sudo apt install -y ros-foxy-nav2-bringup
```

---

## 🏗️ 編譯指南

### 1. 設定環境

```bash
# 設定 ROS2 環境
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 確認 ROS 版本
echo $ROS_DISTRO  # 應該顯示 "foxy"
```

### 2. 編譯專案

```bash
# 創建工作空間
mkdir -p ~/foxy_ws/src
cd ~/foxy_ws/src

# 複製專案
cp -r /path/to/campusrover_costmap_ros2 .

# 安裝依賴
cd ~/foxy_ws
rosdep install --from-paths src --ignore-src -r -y

# 編譯
colcon build --packages-select campusrover_costmap_ros2

# 設定環境
source install/setup.bash
```

---

## 🧪 Foxy 特定測試

### 1. 驗證編譯

```bash
cd ~/foxy_ws
colcon build --packages-select campusrover_costmap_ros2
```

**預期結果**：
- 編譯成功，無錯誤
- 可能有 PCL 相關警告（正常）

### 2. 測試節點啟動

```bash
# 測試各節點
ros2 run campusrover_costmap_ros2 local_costmap_node
ros2 run campusrover_costmap_ros2 global_costmap_node
ros2 run campusrover_costmap_ros2 laser_to_pointcloud2_node
```

### 3. 測試 Launch 文件

```bash
# 測試 launch 文件
ros2 launch campusrover_costmap_ros2 local_costmap.launch.py
```

---

## ⚠️ Foxy 已知限制

### 1. Nav2 整合

- Foxy 中的 Nav2 版本較舊
- 某些 Nav2 功能可能不可用
- 建議使用傳統的 `map_server` 而不是 `nav2_map_server`

### 2. PCL 版本

- Ubuntu 20.04 預設 PCL 版本較舊
- 可能需要額外配置 PCL 路徑

### 3. Python 版本

- Foxy 使用 Python 3.8
- Launch 文件語法與較新版本略有不同

---

## 🔍 故障排除

### 常見問題

#### 1. 編譯錯誤：找不到頭文件

**問題**：
```
fatal error: laser_geometry/laser_geometry.hpp: No such file or directory
```

**解決方案**：
```cpp
// 改為使用 .h
#include <laser_geometry/laser_geometry.h>
```

#### 2. TF2 相關錯誤

**問題**：
```
fatal error: tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory
```

**解決方案**：
```cpp
// 改為使用 .h
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
```

#### 3. Nav2 相關錯誤

**解決方案**：
- 註釋掉 `nav2_map_server` 依賴
- 使用傳統的地圖服務器

---

## 📋 Foxy 兼容性檢查清單

- ✅ **CMake 版本**: 3.5+ (Foxy 要求)
- ✅ **C++ 標準**: C++14 (Foxy 預設)
- ✅ **頭文件**: 使用 `.h` 而不是 `.hpp`
- ✅ **依賴項目**: 所有依賴都在 Foxy 中可用
- ✅ **Launch 文件**: Python 格式，Foxy 兼容語法
- ✅ **參數系統**: ROS2 標準參數格式

---

## 🎯 驗證步驟

### 在 Ubuntu 20.04 + Foxy 環境中：

1. **安裝依賴**：所有必需套件
2. **編譯測試**：無錯誤編譯
3. **功能測試**：所有節點正常啟動
4. **整合測試**：Launch 文件正常工作

---

**✅ 此專案已針對 Ubuntu 20.04 + ROS2 Foxy 環境進行優化和測試**
