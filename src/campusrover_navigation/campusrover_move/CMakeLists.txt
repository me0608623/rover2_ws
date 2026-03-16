cmake_minimum_required(VERSION 3.8)
project(campusrover_move)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(nav2_navfn_planner REQUIRED)
# find_package(nav2_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(campusrover_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(costmap_converter_msgs REQUIRED)

include_directories(
  include
)

add_executable(path_following src/path_following.cpp)
ament_target_dependencies(path_following
  rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros sensor_msgs control_toolbox costmap_converter_msgs tf2_geometry_msgs laser_geometry visualization_msgs campusrover_msgs
)

add_executable(dwa_planner src/dwa_planner.cpp)

ament_target_dependencies(dwa_planner
  rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3 visualization_msgs campusrover_msgs costmap_converter_msgs
)

# add_executable(dwa_planner src/dwa_planner.cpp)
# ament_target_dependencies(dwa_planner
#   rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros sensor_msgs navfn nav_core control_toolbox tf2_geometry_msgs laser_geometry campusrover_msgs
# )

# add_executable(pullover_path_planner src/pullover_path_planner.cpp)
# ament_target_dependencies(pullover_path_planner
#   rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros sensor_msgs navfn nav_core control_toolbox tf2_geometry_msgs laser_geometry campusrover_msgs
# )

install(TARGETS
  path_following
  dwa_planner
  # pullover_path_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()