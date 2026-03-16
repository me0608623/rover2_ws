#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>
#include <string>

#include <std_msgs/msg/Empty.hpp>
#include <std_msgs/msg/Bool.hpp>
#include <std_msgs/msg/String.hpp>

#include <campusrover_msgs/msg/LocationRoom.hpp>
#include <campusrover_msgs/FloorInfo.h>
#include <campusrover_msgs/DestinationWithMpc.h>

#include <campusrover_msgs/RoutingPath.h>
#include <campusrover_msgs/RoutingBezierPath.h>
#include <campusrover_msgs/BubbleMPC.h>
#include <campusrover_msgs/MapLoadInfo.h>
#include <campusrover_msgs/LocalizerSwitch.h>
#include <campusrover_msgs/ElevatorControlInterface.h>
#include <campusrover_msgs/MapClosestLocation.h>

#include <nav_msgs/msg/Path.hpp>
#include <std_srvs/srv/SetBool.hpp>
#include <std_srvs/srv/Empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/TransformStamped.hpp>

using namespace std;

class Demo
{
  enum DemoCase {CROSS_BUILDING, CROSS_FLOOR, SINGLE_FLOOR};
 private:
  campusrover_msgs::LocationRoom current_location_;

  ros::Publisher path_pub_, bezier_path_pub_, room_info_pub_, floor_info_pub_, destination_info_pub_;
  ros::Subscriber trip_sub_, path_completed_sub_, elevator_completed_sub_, path_navigation_sub_, change_map_finish_sub_, change_test_sub_, is_localization_sub_;
  ros::ServiceClient load_map_client_, localizer_client_, routing_client_, routing_bezier_client_, elevator_client_, closest_pt_client_, mpc_enable_client_, change_map_client_, get_localization_info_client_;

  bool path_completed_flag_, elevator_completed_flag_, change_map_finish_flag_, is_localization_flag_;

  void getParameters();

  void LoadMap(const string building, const string floor);
  void initLocalizer(const string building, const string floor, const string point);
  void initLocalizer(const campusrover_msgs::LocationRoom location);
  void offLocalizer();
  void executeChangeMap(const string building, const string floor, const string room);
  void CheckLocalization();
  DemoCase getTripCase(const campusrover_msgs::LocationRoom origin, const campusrover_msgs::LocationRoom destination);
  
  void executeCrossFloorTrip(const campusrover_msgs::LocationRoom& dest);
  void executeSingleFloorTrip(const string floor, const string o_room, const string d_room, const bool enable, const float robot_radius, const float away_dis);
  void executeTakeElevator(const string current_floor, const string dest_floor, const string elevator_id);
  
  void tripCallback(const campusrover_msgs::DestinationWithMpc::ConstPtr& msg);
  void changeMapFinishCallback(const std_msgs::Bool::ConstPtr& msg);
  void pathNavigationCallback(const campusrover_msgs::DestinationWithMpc::ConstPtr& msg);
  void pathCompletedCallback(const std_msgs::Bool::ConstPtr& msg);
  void elevatorCompletedCallback(const std_msgs::Bool::ConstPtr& msg);
  void isLocalizationCallback(const std_msgs::Bool::ConstPtr& msg);
  
  void changetestCallback(const std_msgs::Bool::ConstPtr& msg);
  void rosWaitToComplected(const bool& flag);
  
  void EnableBubbleMpcCallService(ros::ServiceClient &client, campusrover_msgs::BubbleMPC &srv);

  bool checkServicesExist();

 public:
  Demo(ros::NodeHandle &n);
  ~Demo();
  
  void UpdateLocationTimer(const ros::TimerEvent& event);
};