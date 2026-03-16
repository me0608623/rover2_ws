cmake_minimum_required(VERSION 3.8)
project(campusrover_msgs)

# ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                            
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# ======== 定義 msg ========
set(msg_files
  "msg/ArmTablePoseStatus.msg"
  "msg/DoorStatus.msg"
  "msg/ElevatorControlStatus.msg"
  "msg/ElevatorInfo.msg"
  "msg/ElevatorInfoArray.msg"
  "msg/FloorStatus.msg"
  "msg/Encode.msg"
  "msg/ButtonCommand.msg"
  "msg/PoseWithId.msg"
  "msg/InDemoStatus.msg"
  "msg/Room.msg"
  "msg/RoomConnected.msg"
  "msg/RLObservationArray.msg"
  "msg/RLObservation.msg"
  "msg/State.msg"
  "msg/NodeInfo.msg"
  "msg/WorkingFloor.msg"
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  "msg/Trip.msg"
  "msg/LocationRoom.msg"
  "msg/HmiStatus.msg"
  "msg/CANbus.msg"
  "msg/ChargerStatus.msg"
  "msg/ButtonPose.msg"
  "msg/ButtonPoses.msg"
  "msg/BboxAngle.msg"
  "msg/BboxAngleArray.msg"
)

# ======== 定義 srv ========
set(srv_files
  "srv/DriveStatus.srv"
  "srv/DriveProfile.srv"
  "srv/ArmAction.srv"
  "srv/ArmMovePose.srv"
  "srv/InitFloor.srv"
  "srv/PressButton.srv"
  "srv/ElevatorControlInterface.srv"
  "srv/ElevatorStatusChecker.srv"
  "srv/ButtonStatus.srv"
  "srv/PlannerFunction.srv"
  "srv/MapInfo.srv"
  "srv/StatePlan.srv"
  "srv/MapClosestLocation.srv"
  "srv/ModuleInfo.srv"
  "srv/RoutingPath.srv"
  "srv/MapLoadInfo.srv"
  "srv/MapInitPose.srv"
  "srv/LocalizerSwitch.srv"
  "srv/LocalizerLauncher.srv"
  "srv/ArmTablePosition.srv"
  "srv/ArmTableHomeReturn.srv"
  "srv/ButtonParking.srv"
  "srv/ActionParking.srv"
  "srv/ActionCharging.srv"
  "srv/ButtonRecognize.srv"
  "srv/EncoderCount.srv"
  "srv/ImgLabel.srv"
  "srv/PullOverPathGenerator.srv"
  "srv/DemoPath.srv"
)


# 生成介面檔案 (msg/srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs vision_msgs sensor_msgs
)

###########
## Build ##
###########

# 必須有 runtime 相依性
ament_export_dependencies(rosidl_default_runtime)

ament_package()