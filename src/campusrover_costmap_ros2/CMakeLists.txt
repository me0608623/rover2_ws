# Campus Rover 成本地圖套件 CMake 配置文件 (ROS2 Foxy 版本)
# 功能：編譯局部和全域成本地圖生成器
# 目標環境：Ubuntu 20.04 + ROS2 Foxy

cmake_minimum_required(VERSION 3.5)
project(campusrover_costmap_ros2)

# 使用 C++14 標準
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 編譯選項
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## 尋找 ROS2 ament 套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)          # ROS2 C++ 客戶端庫
find_package(std_msgs REQUIRED)        # 標準消息類型
find_package(sensor_msgs REQUIRED)     # 感測器消息類型
find_package(nav_msgs REQUIRED)        # 導航消息類型
find_package(geometry_msgs REQUIRED)   # 幾何消息類型
find_package(tf2 REQUIRED)             # TF2 變換庫
find_package(tf2_ros REQUIRED)         # TF2 ROS 整合
find_package(tf2_geometry_msgs REQUIRED) # TF2 幾何消息
find_package(tf2_eigen REQUIRED)       # TF2 與 Eigen 整合
find_package(pcl_ros REQUIRED)         # PCL ROS2 整合
find_package(pcl_conversions REQUIRED) # PCL 轉換工具
find_package(laser_geometry REQUIRED)  # 雷射幾何轉換

# 尋找外部依賴庫
find_package(Eigen3 REQUIRED)             # 線性代數庫
find_package(OpenCV REQUIRED)             # 電腦視覺庫
find_package(PCL REQUIRED)                # 點雲處理庫

# PCL 庫配置
link_directories(${PCL_LIBRARY_DIRS})     # PCL 庫目錄
add_definitions(${PCL_DEFINITIONS})       # PCL 編譯定義

# 包含目錄設定
include_directories(
  include                                 # 本套件的頭文件目錄
  ${PCL_INCLUDE_DIRS}                    # PCL 包含目錄
)

# ROS2 依賴項目列表
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  laser_geometry
)

# 編譯局部成本地圖庫
add_library(local_costmap_lib src/local_costmap.cpp)
ament_target_dependencies(local_costmap_lib ${dependencies})
target_link_libraries(local_costmap_lib
  ${PCL_LIBRARIES}         # PCL 主庫
)

# 編譯局部成本地圖節點可執行文件
add_executable(local_costmap_node src/local_costmap_node.cpp)
ament_target_dependencies(local_costmap_node ${dependencies})
target_link_libraries(local_costmap_node
  local_costmap_lib        # 局部成本地圖庫               
  ${PCL_LIBRARIES}         # PCL 主庫
)

# 編譯雷射轉點雲節點可執行文件
add_executable(laser_to_pointcloud2_node src/laser_to_pointcloud2.cpp)
ament_target_dependencies(laser_to_pointcloud2_node ${dependencies})

# 編譯全域成本地圖節點可執行文件
add_executable(global_costmap_node 
  src/global_costmap_node.cpp  # 主程序
  src/global_costmap.cpp)      # 實現文件
ament_target_dependencies(global_costmap_node ${dependencies})

# 安裝可執行文件
install(TARGETS
  local_costmap_node
  laser_to_pointcloud2_node
  global_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝庫文件
install(TARGETS
  local_costmap_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安裝頭文件
install(DIRECTORY include/
  DESTINATION include
)

# 安裝 ROS1 launch 文件 (保留用於參考)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch_ros1
)

# 安裝 ROS2 launch 文件
install(DIRECTORY launch_ros2/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安裝 ROS1 參數文件 (保留用於參考)
install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param_ros1
)

# 安裝 ROS2 配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# # 安裝 RViz 配置
# install(DIRECTORY rviz/
#   DESTINATION share/${PROJECT_NAME}/rviz
# )

# 安裝測試數據
install(DIRECTORY test_data/
  DESTINATION share/${PROJECT_NAME}/test_data
)

# ament 套件配置
ament_package()