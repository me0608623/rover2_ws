cmake_minimum_required(VERSION 3.8)
project(campusrover_mot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Ofast)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(campusrover_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(campusrover_mot_node src/mot_node.cpp)
ament_target_dependencies(campusrover_mot_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  nav_msgs
  campusrover_msgs
  laser_geometry
  image_geometry
  cv_bridge
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)
target_link_libraries(campusrover_mot_node ${PCL_LIBRARIES})

install(TARGETS
  campusrover_mot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  script/sim_mot.py
  script/show_now_pose.py
  script/sub_obs_to_csv.py
  script/target_pose_to_csv.py
  script/mot_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
