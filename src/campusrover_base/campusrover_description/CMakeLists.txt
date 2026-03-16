cmake_minimum_required(VERSION 3.5)
project(campusrover_description)

# 找到 ament_cmake
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

# 安裝 URDF、meshes、launch、rviz 資料夾
install(DIRECTORY urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

# 匯出 package
ament_package()
