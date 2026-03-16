cmake_minimum_required(VERSION 3.8)
project(campusrover_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                # ROS2 C++ API
# find_package(rclpy REQUIRED)                 # ROS2 Python API（只有需要時才加）
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 如果有 msg/srv/action
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(campusrover_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

include_directories(
  include
)
# lcr_cmd_vel_mux node
add_executable(lcr_cmd_vel_mux src/lcr_cmd_vel_mux.cpp)
ament_target_dependencies(lcr_cmd_vel_mux
  rclcpp
  std_msgs 
  geometry_msgs 
  campusrover_msgs
  sensor_msgs
)

# calibration node
add_executable(calibration src/calibration.cpp)
ament_target_dependencies(calibration
  rclcpp
  campusrover_msgs
)

# joy_to_twist node
add_executable(joy_to_twist src/joy_to_twist.cpp)
ament_target_dependencies(joy_to_twist
  rclcpp
  geometry_msgs
  sensor_msgs
)

# hmi_joy_control_node
add_executable(hmi_joy_control_node src/hmi_joy_control.cpp)
ament_target_dependencies(hmi_joy_control_node
  rclcpp
  std_msgs
  sensor_msgs
  campusrover_msgs
)

# rover_driver node
add_executable(rover_driver src/rover_driver.cpp)
ament_target_dependencies(rover_driver
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  campusrover_msgs
)
target_link_libraries(rover_driver yaml-cpp)

### ROS TEST NODE
add_executable(rostest_control_valiation test/rostest_control_valiation.cpp)
ament_target_dependencies(rostest_control_valiation rclcpp std_msgs geometry_msgs)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs std_msgs)


#############
## Install ##
#############

install(TARGETS
  joy_to_twist
  lcr_cmd_vel_mux
  rover_driver
  calibration
  hmi_joy_control_node
  rostest_control_valiation
  odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
#############
## Testing ##
#############


# # catkin_make -DCATKIN_ENABLE_TESTING=1
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(utest_scurve_profile 
#     test/utest_scurve_profile.cpp)
#   target_link_libraries(utest_scurve_profile 
#     ${catkin_LIBRARIES})

#   catkin_add_gtest(utest_trapezoidal_profile 
#     test/utest_trapezoidal_profile.cpp)
#   target_link_libraries(utest_trapezoidal_profile 
#     ${catkin_LIBRARIES})
# endif()
