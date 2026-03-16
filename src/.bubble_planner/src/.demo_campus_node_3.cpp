#include <demo_campus_node_3.hpp>

void Demo::getParameters()
{
  ros::NodeHandle n_private("~"); 
  n_private.param<string>("start_building", current_location_.building, "itc_luntsai");
  n_private.param<string>("start_floor",    current_location_.floor,    "3");
  n_private.param<string>("start_point",    current_location_.room,     "r314");
}

void Demo::CheckLocalization()
{
  ROS_INFO("[Demo Campus Node 3] CheckLocalization");
  std_srvs::SetBool srv;
  srv.request.data = true ;
  if (get_localization_info_client_.call(srv))
  {
    ROS_INFO("[Demo Campus Node 3] Check Localization Service is called");
    is_localization_flag_ = false;
    rosWaitToComplected(is_localization_flag_);
    ROS_INFO("[Demo Campus Node 3] Check Localization Complete");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call Check Localization Service");
  }
}

void Demo::executeChangeMap(const string building, const string floor, const string room)
{
  ROS_INFO("[Demo Campus Node 3] ChangeMap to: %s, %s", floor.c_str(), room.c_str());
  campusrover_msgs::ChangeMap srv;
  srv.request.enable = true ;
  srv.request.build = building;
  srv.request.floor = floor;
  srv.request.room = room;
  if (change_map_client_.call(srv))
  {
    ROS_INFO("[Demo Campus Node 3] ChangeMap Service is called");
    change_map_finish_flag_ = false;
    rosWaitToComplected(change_map_finish_flag_);
    ROS_INFO("[Demo Campus Node 3] ChangeMap Complete");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call ChangeMap Service");
  }
}

void Demo::LoadMap(const string building, const string floor)
{
  ROS_INFO("[Demo Campus Node 3] Loading Map: %s, %s", building.c_str(), floor.c_str());
  campusrover_msgs::MapLoadInfo srv;
  srv.request.building = building;
  srv.request.floor = floor;
  if (load_map_client_.call(srv))
  {
    ROS_INFO("[Demo Campus Node 3] LoadMap Service is called");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call LoadMap Service");
  }
}

void Demo::initLocalizer(const string building, const string floor, const string point)
{
  static campusrover_msgs::LocalizerSwitch srv;
  srv.request.mode = srv.request.MODE_ENABLE;
  srv.request.building = building;
  srv.request.floor = floor;
  srv.request.init_point = point;

  if (localizer_client_.call(srv))
  {
    ROS_INFO("[Demo Campus Node 3] Localizer Service is called");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call Localizer Service");
  }
}

void Demo::initLocalizer(const campusrover_msgs::LocationRoom location)
{
  initLocalizer(location.building, location.floor, location.room);
}

void Demo::offLocalizer()
{
  static campusrover_msgs::LocalizerSwitch srv;
  srv.request.mode = srv.request.MODE_DISABLE;

  if (localizer_client_.call(srv))
  {
    ROS_INFO("[Demo Campus Node 3] Localizer Service is called");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call Localizer Service");
  }
}

Demo::DemoCase Demo::getTripCase(const campusrover_msgs::LocationRoom origin, const campusrover_msgs::LocationRoom destination)
{
  if (origin.building == destination.building)
  {
    if (origin.floor == destination.floor)
    {
      return SINGLE_FLOOR;
    }
    else
    {
      return CROSS_FLOOR;
    }
  }
  else
  {
    return CROSS_BUILDING;
  }
}

void Demo::executeCrossFloorTrip(const campusrover_msgs::LocationRoom& dest)
{
  executeSingleFloorTrip(current_location_.floor, current_location_.room, "e1_center_in", true, 0.35, 0.3);

  executeTakeElevator(current_location_.floor, dest.floor, "e1_left"); //e1_left 只是懶的改而已 沒用到

  ROS_INFO("change start");
  ros::Time start_time = ros::Time::now();
  ROS_INFO("executeChangeMap");
  executeChangeMap(dest.building, dest.floor, dest.room); //changeMap

  ros::Duration(0.5).sleep();

  ROS_INFO("LoadMap");  
  LoadMap(current_location_.building, dest.floor); //itc_luntsai, 1F
  
  ROS_INFO("initLocalizer");   
  initLocalizer(current_location_.building, dest.floor, "e1_center_localization");
  
  ROS_INFO("change finish");
  ros::Time end_time = ros::Time::now();
  ROS_INFO("change map use: %.3f secs", (end_time - start_time).toSec());

  CheckLocalization();

  executeSingleFloorTrip(dest.floor, "e1_center_localization", dest.room, true, 0.35, 0.3);
}

void Demo::executeSingleFloorTrip( const string received_floor, const string origin_room, const string dest_room, const bool enable, const float robot_radius, const float away_dis)
{
  campusrover_msgs::RoutingPath path;
  path.request.origin = origin_room;
  path.request.destination.push_back(dest_room);

  // campusrover_msgs::RoutingBezierPath bezier_path;
  // bezier_path.request.origin = origin_room;
  // bezier_path.request.destination.push_back(dest_room);
  // bezier_path.request.method = "BezierCurve";

  campusrover_msgs::FloorInfo floor_info;
  floor_info.current_floor = current_location_.floor ;
  floor_info.target_floor = received_floor ;
  floor_info_pub_.publish(floor_info);

  std_msgs::String room_info ;
  room_info.data = dest_room ;
  room_info_pub_.publish(room_info);

  // if (routing_bezier_client_.call(bezier_path))
  // {
  //   ROS_INFO("[Demo Campus Node 3] get routing_bezier");
  //   bezier_path_pub_.publish(bezier_path.response.routing[0]); 
  // }

  if (routing_client_.call(path))
  {
    ROS_INFO("[Demo Campus Node 3] Start trip ~~");
    path_pub_.publish(path.response.routing[0]); //傳給bubble mpc導航的code 然後先 enable mpc 然後再給出mpc goal's path

    campusrover_msgs::BubbleMPC bubble_mpc_enable;
    bubble_mpc_enable.request.enable = enable;
    bubble_mpc_enable.request.robot_radius = robot_radius;
    bubble_mpc_enable.request.away_dis = away_dis;
    EnableBubbleMpcCallService(mpc_enable_client_, bubble_mpc_enable);

    path_completed_flag_ = false;
    rosWaitToComplected(path_completed_flag_);
    ROS_INFO("[Demo Campus Node 3] Path Complected!");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call Routing Service");
  }
}

void Demo::executeTakeElevator(const string current_floor, const string dest_floor, const string elevator_id)
{
  // Request take elevator
  campusrover_msgs::ElevatorControlInterface elevator_srv;
  elevator_srv.request.enable_multi = false ;
  elevator_srv.request.elevator_id = elevator_id;
  elevator_srv.request.building = current_location_.building;
  elevator_srv.request.init_floor = stoi(current_floor);
  elevator_srv.request.target_floor = stoi(dest_floor);

  elevator_client_.waitForExistence();
  if (elevator_client_.call(elevator_srv))
  {
    elevator_completed_flag_ = false;
    rosWaitToComplected(elevator_completed_flag_);
    ROS_INFO("[Demo Campus Node 3] Elevator Complected!");
  }
  else
  {
    ROS_ERROR("[Demo Campus Node 3] Failed to call Elevator Service");
  }
}

void Demo::tripCallback(const campusrover_msgs::DestinationWithMpc::ConstPtr& dest)
{
  bool receive_enable = dest->enable ;
  float receive_robot_radius = dest->robot_radius ;
  float receive_away_dis = dest->away_dis ; 

  campusrover_msgs::LocationRoom destination;
  destination.building = dest->building;
  destination.floor = dest->floor;  
  destination.room = dest->room;

  destination_info_pub_.publish(destination);

  ROS_INFO("[Demo Campus Node 3] received destination: building: %s, floor: %s, room: %s",
           destination.building.c_str(), destination.floor.c_str(), destination.room.c_str());

  switch (getTripCase(current_location_, destination))
  {
  case CROSS_BUILDING:
    ROS_INFO("[Demo Campus Node 3] trip type -> cross build");
    // 
    break;

  case CROSS_FLOOR:
    ROS_INFO("[Demo Campus Node 3] trip type -> cross floor");
    executeCrossFloorTrip(destination);
      // 1. move to elevator button (e1_button_check)
      // 2. take elevator (3F --> 1F)
      // 3. change map and init point (e1_center_relocalization)
      // 4. move to "destination" (r104)
    break;

  case SINGLE_FLOOR:
    ROS_INFO("[Demo Campus Node 3] trip type -> single floor");
    executeSingleFloorTrip(destination.floor, current_location_.room, destination.room, receive_enable, receive_robot_radius, receive_away_dis);
      // move to "destination" (r317-1)
    break;
  
  default:
    break;
  }
}

void Demo::pathNavigationCallback(const campusrover_msgs::DestinationWithMpc::ConstPtr& msg)
{

  bool receive_enable = msg->enable;
  float receive_robot_radius = msg->robot_radius;
  float receive_away_dis = msg->away_dis;

  campusrover_msgs::LocationRoom destination;
  destination.building = msg->building;
  destination.floor = msg->floor;
  destination.room = msg->room;

  ROS_INFO("[Demo Campus Node 3] received path navagation: building: %s, floor: %s, room: %s",
          destination.building.c_str(), destination.floor.c_str(), destination.room.c_str());

  executeSingleFloorTrip(destination.floor, current_location_.room, destination.room, receive_enable, receive_robot_radius, receive_away_dis);

}

void Demo::EnableBubbleMpcCallService(ros::ServiceClient &client, campusrover_msgs::BubbleMPC &srv)
{
  ROS_INFO("[Demo Campus Node 3] received enable bubble mpc call service");
  while (!client.call(srv))
  {
    ROS_ERROR("[Demo Campus Node 3] Enable Bubble MPC : Failed to call service");
    ros::Duration(1.0).sleep();
  }
}

void Demo::isLocalizationCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    is_localization_flag_ = true;
  }
}

void Demo::changeMapFinishCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    change_map_finish_flag_ = true;
  }
}

void Demo::changetestCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO("executeChangeMap start");
    executeChangeMap("itc_luntsai", "1", "e1_center_localization"); //changeMap
    ROS_INFO("executeChangeMap finish");

    ros::Duration(0.5).sleep();

    ROS_INFO("LoadMap start");    
    LoadMap("itc_luntsai", "1"); //itc_luntsai, 1F
    ROS_INFO("LoadMap finish");  

    ROS_INFO("initLocalizer start");    
    initLocalizer("itc_luntsai", "1", "e1_center_localization");
    ROS_INFO("initLocalizer finish");        
    ROS_INFO("change finish");
    ros::Time end_time = ros::Time::now();
    ROS_INFO("change map use: %.3f secs", (end_time - start_time).toSec());
  }
}

void Demo::pathCompletedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    path_completed_flag_ = true;
  }
}

void Demo::elevatorCompletedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    elevator_completed_flag_ = true;
  }
}

void Demo::rosWaitToComplected(const bool& flag)
{
  while (!flag)
  {
    ros::Duration(0.1).sleep();
  }
}

Demo::Demo(ros::NodeHandle &n)
{
  getParameters();
  path_completed_flag_ = true;
  elevator_completed_flag_ = true;
  
  //publisher
  path_pub_                   = n.advertise<nav_msgs::Path>("global_path", 10, true);
  bezier_path_pub_            = n.advertise<nav_msgs::Path>("bezier_global_path", 10, true);
  room_info_pub_              = n.advertise<std_msgs::String>("room_info", 10, true);
  floor_info_pub_             = n.advertise<campusrover_msgs::FloorInfo>("/floor_info", 10, true);
  destination_info_pub_       = n.advertise<campusrover_msgs::LocationRoom>("/destination_info", 10, true);

  // service client
  load_map_client_              = n.serviceClient<campusrover_msgs::MapLoadInfo>("load_map");
  localizer_client_             = n.serviceClient<campusrover_msgs::LocalizerSwitch>("enable_localizer");
  routing_client_               = n.serviceClient<campusrover_msgs::RoutingPath>("generation_path");
  elevator_client_              = n.serviceClient<campusrover_msgs::ElevatorControlInterface>("elevator_controller");
  closest_pt_client_            = n.serviceClient<campusrover_msgs::MapClosestLocation>("get_closest_location");
  mpc_enable_client_            = n.serviceClient<campusrover_msgs::BubbleMPC>("enable_mpc");
  change_map_client_            = n.serviceClient<campusrover_msgs::ChangeMap>("kill_nodes");
  get_localization_info_client_ = n.serviceClient<std_srvs::SetBool>("get_localization_info");

  while(!checkServicesExist())
  {
    ros::Duration(0.1).sleep();
  }
  
  //subscriber
  trip_sub_                   = n.subscribe("cr_trip_luntsai",  10, &Demo::tripCallback,              this);
  path_completed_sub_         = n.subscribe("mpc_finish",       10, &Demo::pathCompletedCallback,     this);
  elevator_completed_sub_     = n.subscribe("elevator_finish",  10, &Demo::elevatorCompletedCallback, this);
  path_navigation_sub_        = n.subscribe("path_navigation",  10, &Demo::pathNavigationCallback,    this);
  change_map_finish_sub_      = n.subscribe("change_map_finish",10, &Demo::changeMapFinishCallback,   this);
  is_localization_sub_        = n.subscribe("is_localization",  10, &Demo::isLocalizationCallback,    this);
  //test
  change_test_sub_            = n.subscribe("change_test",      10, &Demo::changetestCallback,        this);

  cout << "[Demo Campus Node 3] Loading map: " << current_location_ << endl; 
  LoadMap(current_location_.building, current_location_.floor);
  cout << "[Demo Campus Node 3] initLocalizer: " << current_location_ << endl; 
  initLocalizer(current_location_);
}

bool Demo::checkServicesExist()
{
  ROS_INFO("[Demo Campus Node 3] checking load_submap service...");
  if (!load_map_client_.waitForExistence(ros::Duration(10, 0))){return false; }

  ROS_INFO("[Demo Campus Node 3] checking localizer service...");
  if (!localizer_client_.waitForExistence(ros::Duration(10, 0))){return false; }

  ROS_INFO("[Demo Campus Node 3] checking routing service...");
  if (!routing_client_.waitForExistence(ros::Duration(10, 0))){return false; }

  ROS_INFO("[Demo Campus Node 3] checking closest_pt service...");
  if (!closest_pt_client_.waitForExistence(ros::Duration(10, 0))){return false; }

  return true;
}

Demo::~Demo()
{
}

void Demo::UpdateLocationTimer(const ros::TimerEvent& event)
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("world", "base_footprint",ros::Time(0));
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    return;
  }
  campusrover_msgs::MapClosestLocation srv;
  srv.request.building = current_location_.building;
  srv.request.floor = current_location_.floor;
  srv.request.robot_pose.pose.position.x = transformStamped.transform.translation.x;
  srv.request.robot_pose.pose.position.y = transformStamped.transform.translation.y;
  if (closest_pt_client_.call(srv))
  {
    current_location_.room = srv.response.closest_point;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "demo_campus_node_3");
  ros::NodeHandle n;
  Demo demo(n);
  ros::Timer tracking_timer = n.createTimer(ros::Duration(0.5), &Demo::UpdateLocationTimer, &demo);

  ros::AsyncSpinner spinner(4); // 使用2个线程处理回调
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}
