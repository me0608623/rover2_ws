cmake_minimum_required(VERSION 3.13)
project(test_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 尋找依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(angles REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(campusrover_msgs REQUIRED)

# 2. 定義訊息與服務
set(msg_files
  "msg/TestData.msg"
  "msg/AllTestData.msg"
)

set(srv_files
  # "srv/YourService.srv"
)

# 產生訊息介面
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# 3. 整理依賴列表
set(TEST_PKG_DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  interactive_markers
  angles
  turtlesim
  campusrover_msgs
)

# 4. 自定義編譯函式 (修正依賴問題)
function(test_pkg_add_executable target)
  add_executable(${target} ${ARGN})
  # 連結外部 ROS 2 套件
  ament_target_dependencies(${target} ${TEST_PKG_DEPENDENCIES})
  
  # 重要：連結本套件產生的自定義訊息 (TestData.msg)
  # 這樣程式才能找到 #include "test_pkg/msg/test_data.hpp"
  rosidl_target_interfaces(${target} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  # 設定 include 路徑
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endfunction()

# ---------------- src (取消註解即可使用) ----------------
test_pkg_add_executable(experiment_logger src/experiment_logger.cpp)
test_pkg_add_executable(planner_data_collector src/planner_data_collector.cpp)

# ---------------- Install ----------------
# 注意：若有新增 executable，必須加入到 TARGETS 列表
install(TARGETS
  experiment_logger
  planner_data_collector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

# 5. 導出依賴
ament_export_dependencies(rosidl_default_runtime)
ament_package()