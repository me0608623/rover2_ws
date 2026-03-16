cmake_minimum_required(VERSION 3.8)
project(campusrover_routing)

# Use modern C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(campusrover_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


# Include directories globally (optional, you can also do per-target)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------
# Routing Engine Node
# ---------------------------
add_executable(routing_engine_node
  src/node_routing_engine/routing_engine_node.cpp
  src/node_routing_engine/routing_engine.cpp
  src/node_routing_engine/module_linkage.cpp
  src/node_routing_engine/node_connection.cpp
)

target_include_directories(routing_engine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(routing_engine_node
  rclcpp
  geometry_msgs
  nav_msgs
  campusrover_msgs
  visualization_msgs
  Eigen3
  builtin_interfaces
)

# ---------------------------
# Routes Visualization Node
# ---------------------------
add_executable(routes_visualization
  src/routes_visualization/routes_visualization.cpp
)

target_include_directories(routes_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(routes_visualization
  rclcpp
  geometry_msgs
  nav_msgs
  campusrover_msgs
  visualization_msgs  # 🔧 Added missing dependency!
  Eigen3
)

# ---------------------------
# Install Targets
# ---------------------------
install(TARGETS
  routing_engine_node
  routes_visualization
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# Install Scripts
# ---------------------------
install(PROGRAMS
  scripts/mapinfo_db_handler.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# Install Launch Files
# ---------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---------------------------
# Install Data if any (optional)
# ---------------------------
install(DIRECTORY
  share/
  DESTINATION share/${PROJECT_NAME}/share
)

ament_package()
