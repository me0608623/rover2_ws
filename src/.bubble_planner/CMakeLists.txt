cmake_minimum_required(VERSION 3.13)   # 升到 3.13 以使用 target_link_directories
project(bubble_planner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find necessary ROS2 packages
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

include_directories(
  include
)

set(msg_files
  "msg/MPCParameter.msg"
  "msg/ObstacleInfo.msg"
  )

set(srv_files
  # "msg/MPCParameter.msg"
  )

set(BUBBLE_PLANNER_DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
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
function(bubble_planner_add_executable target)
  add_executable(${target} ${ARGN})
  ament_target_dependencies(${target} ${BUBBLE_PLANNER_DEPENDENCIES})
endfunction()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)
rosidl_get_typesupport_target(bubble_planner_typesupport_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ACADOS related settings
set(ACADOS_INSTALL_DIR $ENV{ACADOS_INSTALL_DIR})
if(NOT ACADOS_INSTALL_DIR)
  message(FATAL_ERROR "請先設定環境變數 ACADOS_INSTALL_DIR（例如：export ACADOS_INSTALL_DIR=$HOME/acados）")
endif()

set(ACADOS_INC
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/blasfeo
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
)
set(ACADOS_LIB_DIR ${ACADOS_INSTALL_DIR}/lib)

# 生成器输出目录
set(GEN_DIR ${PROJECT_SOURCE_DIR}/bubble_mpc/bubble_mpc_model/c_generated_code)
set(MODEL_NAME bubble_model)

# 明确列出 acados 生成的源（根据你目录里的实际文件名）
set(GEN_SOURCES ${GEN_DIR}/acados_solver_${MODEL_NAME}.c)
file(GLOB GEN_MODEL "${GEN_DIR}/${MODEL_NAME}_model/*.c")
list(APPEND GEN_SOURCES ${GEN_MODEL})

# 代价/约束：如果存在也一并编入
if(EXISTS ${GEN_DIR}/${MODEL_NAME}_cost)
  file(GLOB GEN_COST "${GEN_DIR}/${MODEL_NAME}_cost/*.c")
  list(APPEND GEN_SOURCES ${GEN_COST})
endif()
if(EXISTS ${GEN_DIR}/${MODEL_NAME}_constraints)
  file(GLOB GEN_CONSTR "${GEN_DIR}/${MODEL_NAME}_constraints/*.c")
  list(APPEND GEN_SOURCES ${GEN_CONSTR})
endif()

list(REMOVE_ITEM GEN_SOURCES ${GEN_DIR}/main_${MODEL_NAME}.c) # 排除 main 文件

# ---------------- Simulator ----------------
bubble_planner_add_executable(turtlesim_to_map simulator/turtle_sim/turtlesim_to_map.cpp)
bubble_planner_add_executable(robot_marker simulator/turtle_sim/robot_marker.cpp)
bubble_planner_add_executable(global_path_generator simulator/turtle_sim/global_path_generator.cpp)
bubble_planner_add_executable(generated_sim_odom simulator/turtle_sim/generated_sim_odom.cpp)

bubble_planner_add_executable(static_obs simulator/costmap/static_obs.cpp)
bubble_planner_add_executable(dynamic_obs simulator/costmap/dynamic_obs.cpp)
bubble_planner_add_executable(tracked_obs simulator/costmap/tracked_obs.cpp)
bubble_planner_add_executable(costmap_merge_inflation simulator/costmap/costmap_merge_inflation.cpp)

# ---------------- Costmap converter ----------------
# bubble_planner_add_executable(obstacle_filter costmap_converter/obstacle_filter.cpp)
# bubble_planner_add_executable(split_obstacles_node costmap_converter/split_obstacles_node.cpp)
# bubble_planner_add_executable(mot_to_obstacle_simple costmap_converter/mot_to_obstacle_simple.cpp)
# bubble_planner_add_executable(map_to_obstacle_simple costmap_converter/map_to_obstacle_simple.cpp)
# bubble_planner_add_executable(wall_and_dynamic_obstacle_map costmap_converter/wall_and_dynamic_obstacle_map.cpp)

# ---------------- Path planner ----------------
bubble_planner_add_executable(local_path_node path_planner/local_path_node.cpp)
bubble_planner_add_executable(regular_points_node path_planner/regular_points_node.cpp)

bubble_planner_add_executable(path_builder_node 
  path_planner/path_build/path_builder_node.cpp
  path_planner/path_build/path_builder_core.cpp
  path_planner/path_build/path_builder_visualizer.cpp)

# ---------------- MPC + acados (ROS 2) ----------------

# 1. 使用自定義函數建立執行檔並自動連結 ROS 依賴
bubble_planner_add_executable(bubble_mpc_node
  bubble_mpc/bubble_mpc_node.cpp
  bubble_mpc/bubble_mpc_solver.cpp
  ${GEN_SOURCES}
)

# 2. 設定 Acados 相關屬性 (C99 標準、Include 路徑、連結庫)
set_source_files_properties(${GEN_SOURCES} PROPERTIES LANGUAGE C)
target_compile_options(bubble_mpc_node PRIVATE $<$<COMPILE_LANGUAGE:C>:-std=c99>)

target_include_directories(bubble_mpc_node PRIVATE
  include
  ${GEN_DIR} ${GEN_DIR}/${MODEL_NAME}_model ${GEN_DIR}/${MODEL_NAME}_cost ${GEN_DIR}/${MODEL_NAME}_constraints
  ${ACADOS_INC}
)

target_link_directories(bubble_mpc_node PRIVATE ${ACADOS_LIB_DIR})
target_link_libraries(bubble_mpc_node acados hpipm blasfeo m ${bubble_planner_typesupport_cpp})

# 3. 設定 RPATH 以便運行時找到 Acados 庫
set_target_properties(bubble_mpc_node PROPERTIES
  BUILD_RPATH "${ACADOS_LIB_DIR}"
  INSTALL_RPATH "${ACADOS_LIB_DIR}"
)

# ---------------- src ----------------
bubble_planner_add_executable(just_fellow src/just_fellow.cpp)
bubble_planner_add_executable(trajectory_recorder src/trajectory_recorder.cpp)
# bubble_planner_add_executable(demo_campus_node_3 src/demo_campus_node_3.cpp)

# ---------------- Install ----------------
install(TARGETS
  # Simulator
  turtlesim_to_map
  robot_marker
  global_path_generator
  generated_sim_odom
  static_obs
  dynamic_obs
  tracked_obs
  costmap_merge_inflation
  
  # Path planner
  local_path_node
  regular_points_node
  path_builder_node
  
  # MPC
  bubble_mpc_node
  
  # Src
  just_fellow
  trajectory_recorder
  # demo_campus_node_3
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
