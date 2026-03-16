#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <campusrover_msgs/srv/planner_function.hpp>
#include <campusrover_msgs/srv/elevator_status_checker.hpp>
#include <campusrover_msgs/srv/pull_over_path_generator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

#define M_PI 3.14159265358979323846

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PathFollowingNode : public rclcpp::Node
{
public:
  PathFollowingNode() : Node("path_following")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("robot_frame", "base_link");
    this->declare_parameter<double>("arriving_range_dis", 0.2);
    this->declare_parameter<double>("arriving_range_angle", 0.1);
    this->declare_parameter<double>("max_linear_velocity", 0.5);
    this->declare_parameter<double>("min_linear_velocity", 0.05);
    this->declare_parameter<double>("max_angular_velocity", 0.3);
    this->declare_parameter<double>("min_angular_velocity", 0.05);
    this->declare_parameter<double>("target_point_dis", 0.5);
    this->declare_parameter<double>("threshold_occupied", 10.0);
    this->declare_parameter<double>("footprint_max_x", 1.5);
    this->declare_parameter<double>("footprint_min_x", -0.5);
    this->declare_parameter<double>("footprint_max_y", 0.5);
    this->declare_parameter<double>("footprint_min_y", -0.5);
    this->declare_parameter<double>("obstacle_range", 0.3);
    this->declare_parameter<double>("obstacle_detect_max_dis", 1.5);
    this->declare_parameter<double>("obstacle_detect_min_dis", 0.8);
    this->declare_parameter<double>("back_obstacle_detect_max_dis", 2.3);
    this->declare_parameter<double>("speed_pid_k", 0.06);
    this->declare_parameter<double>("min_angle_of_linear_profile", 0.1);
    this->declare_parameter<double>("max_angle_of_linear_profile", 0.5);
    this->declare_parameter<bool>("enable_costmap_obstacle", true);
    this->declare_parameter<bool>("enable_dwa_obstacle_avoidance", true);
    this->declare_parameter<bool>("enable_pullover_mode", false);
    this->declare_parameter<bool>("direction_inverse", false);
    this->declare_parameter<bool>("enable_linear_depend_angular", false);

    this->get_parameter("robot_frame", robot_frame_);
    this->get_parameter("arriving_range_dis", arriving_range_dis_);
    this->get_parameter("arriving_range_angle", arriving_range_angle_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("min_linear_velocity", min_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("min_angular_velocity", min_angular_velocity_);
    this->get_parameter("target_point_dis", target_point_dis_);
    this->get_parameter("threshold_occupied", threshold_occupied_);
    this->get_parameter("footprint_max_x", footprint_max_x_);
    this->get_parameter("footprint_min_x", footprint_min_x_);
    this->get_parameter("footprint_max_y", footprint_max_y_);
    this->get_parameter("footprint_min_y", footprint_min_y_);
    this->get_parameter("obstacle_range", obstacle_range_);
    this->get_parameter("obstacle_detect_max_dis", obstacle_detect_max_dis_);
    this->get_parameter("obstacle_detect_min_dis", obstacle_detect_min_dis_);
    this->get_parameter("back_obstacle_detect_max_dis", back_obstacle_detect_max_dis_);
    this->get_parameter("speed_pid_k", speed_pid_k_);
    this->get_parameter("min_angle_of_linear_profile", min_angle_of_linear_profile_);
    this->get_parameter("max_angle_of_linear_profile", max_angle_of_linear_profile_);
    this->get_parameter("enble_costmap_obstacle", enble_costmap_obstacle_);
    this->get_parameter("enble_dwa_obstacle_avoidance", enble_dwa_obstacle_avoidance_);
    this->get_parameter("enble_pullover_mode", enble_pullover_mode_);
    this->get_parameter("direction_inverse", direction_inverse_);
    this->get_parameter("enable_linear_depend_angular", enable_linear_depend_angular_);

    // Publishers
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
    twist_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("twist_path", 20);
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_trajectories", 10);
    global_status_check_pub_ = this->create_publisher<std_msgs::msg::Empty>("reach_goal", 20);

    // Subscribers
    elevator_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "elevator_path", 10, std::bind(&PathFollowingNode::ElevatorPathCallback, this, _1));
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "global_path", 10, std::bind(&PathFollowingNode::GlobalPathCallback, this, _1));
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "costmap", 10, std::bind(&PathFollowingNode::CostmapCallback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&PathFollowingNode::OdomCallback, this, _1));
    custom_obst_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "obstacles", 1, std::bind(&PathFollowingNode::CB_customObstacle, this, _1));

    // Service servers and clients
    planner_function_srv_ = this->create_service<campusrover_msgs::srv::PlannerFunction>(
      "planner_function", std::bind(&PathFollowingNode::ServiceCallback, this, _1, _2));
    elevator_status_check_client_ = this->create_client<campusrover_msgs::srv::ElevatorStatusChecker>("elevator_status_checker");
    dwa_planner_client_ = this->create_client<campusrover_msgs::srv::PlannerFunction>("planner_function_dwa");
    pullover_planner_client_ = this->create_client<campusrover_msgs::srv::PullOverPathGenerator>("generate_pullover_path");

    // Timers
    timer_ = this->create_wall_timer(20ms, std::bind(&PathFollowingNode::TimerCallback, this));
    msgs_timer_ = this->create_wall_timer(1s, std::bind(&PathFollowingNode::MsgsTimerCallback, this));

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 其餘成員初始化
    // ...
  }

  // ROS 2 callback型態宣告
  void ElevatorPathCallback(const nav_msgs::msg::Path::SharedPtr path);
  void GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr path);
  void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void CB_customObstacle(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg);

  // 計時器
  void TimerCallback();
  void MsgsTimerCallback();

  // TF、路徑、邏輯相關
  void UpdateCampusRoverPoseFromTF();
  void check_arrive_point();
  void check_arrive_direction();
  void moving_to_target_point();
  void moving_to_target_direction();
  void TwistPublish(double x, double z);
  void TwistProfile(double &profile_linear_x, double &profile_angular_z);
  void PulloverFunction();
  void UpdateInputPath(const nav_msgs::msg::Path &path);
  void angle_normalize(double &angle);

  // Service callback
  bool ServiceCallback(
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> req,
    std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> res);

  // Elevator 狀態檢查 service client
  void ElevatorStatusCheckCallService(
    std::shared_ptr<campusrover_msgs::srv::ElevatorStatusChecker::Request> req);

  // Service client呼叫請用 async_send_request 並處理 future
  // TF2查詢請用 tf_buffer_->lookupTransform(...) 並處理 rclcpp::Time

private:
  // Publisher/Subscriber/Service/Timer/TF2
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr twist_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr global_status_check_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr elevator_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_;

  rclcpp::Service<campusrover_msgs::srv::PlannerFunction>::SharedPtr planner_function_srv_;
  rclcpp::Client<campusrover_msgs::srv::ElevatorStatusChecker>::SharedPtr elevator_status_check_client_;
  rclcpp::Client<campusrover_msgs::srv::PlannerFunction>::SharedPtr dwa_planner_client_;
  rclcpp::Client<campusrover_msgs::srv::PullOverPathGenerator>::SharedPtr pullover_planner_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr msgs_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 其餘成員變數請依原始程式宣告
  // ...

  nav_msgs::msg::Path following_path_;
  nav_msgs::msg::Path globle_path_; 
  nav_msgs::msg::Path elevator_path_; 
  nav_msgs::msg::Path pullover_path_;
  geometry_msgs::msg::PoseStamped target_pose_;
  nav_msgs::msg::OccupancyGrid costmap_data_;
  
  rclcpp::Time planner_start_time;

  geometry_msgs::msg::Pose robot_tf_pose_;
  geometry_msgs::msg::PoseArray obstacle_poses_;

  campusrover_msgs::srv::ElevatorStatusChecker elevator_status_checker_msg_;
  std_msgs::msg::Empty global_status_checker_msg_;

  std::string robot_frame_;
  int status_msg_;
  int path_sub_mode_;

  double arriving_range_dis_;
  double arriving_range_angle_;

  bool action_flag_ = false;
  bool get_following_path_ = false;
  bool get_costmap_data_ = false;
  bool get_obstacle_data_ = false;
  bool obstacle_stop_cmd_ = false;
  bool arriving_end_point_= false;
  bool arriving_end_direction_= false;
  bool enble_costmap_obstacle_;
  bool direction_inverse_= false;
  bool get_velocity_data_ = false;

  bool call_dwa_enable_srv_ = false;
  bool call_dwa_disable_srv_ = true;


  bool enable_linear_depend_angular_;
  bool enble_dwa_obstacle_avoidance_;
  bool enble_pullover_mode_;
  double max_angle_of_linear_profile_;
  double min_angle_of_linear_profile_;

  double threshold_occupied_;
  double footprint_max_x_;
  double footprint_min_x_;
  double footprint_max_y_;
  double footprint_min_y_;

  double robot_yaw_;
  double speed_pid_k_;
  double target_yaw_;

  double max_linear_velocity_;
  double min_linear_velocity_;
  double max_angular_velocity_;
  double min_angular_velocity_;
  double target_point_dis_;
  double obstacle_detect_max_dis_;
  double obstacle_detect_min_dis_;
  double back_obstacle_detect_max_dis_;
  double obstacle_range_;

  double active_angular_;

  double twist_linear_step_ = 0;
  double twist_angular_step_ = 0;

  double current_v_;
  double current_w_;
  

};

//-----------------------------------------------------------------------------------------------

void PathFollowingNode::UpdateCampusRoverPoseFromTF()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  double roll, pitch, yaw;
  double pre_yaw;

  try
  {
    transformStamped = tf_buffer_->lookupTransform(
      following_path_.header.frame_id,
      robot_frame_,
      rclcpp::Time(0,0,get_clock()->get_clock_type()),
      rclcpp::Duration::from_seconds(2.0));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "%s. Can't update pose from TF, will use the latest source point.", ex.what());
    return;
  }

  robot_tf_pose_.position.x = transformStamped.transform.translation.x;
  robot_tf_pose_.position.y = transformStamped.transform.translation.y;
  robot_tf_pose_.position.z = transformStamped.transform.translation.z;
  robot_tf_pose_.orientation.x = transformStamped.transform.rotation.x;
  robot_tf_pose_.orientation.y = transformStamped.transform.rotation.y;
  robot_tf_pose_.orientation.z = transformStamped.transform.rotation.z;
  robot_tf_pose_.orientation.w = transformStamped.transform.rotation.w;

  tf2::Quaternion q(
    robot_tf_pose_.orientation.x,
    robot_tf_pose_.orientation.y,
    robot_tf_pose_.orientation.z,
    robot_tf_pose_.orientation.w);
  tf2::Matrix3x3 m(q);

  m.getRPY(roll, pitch, yaw);
  if(!direction_inverse_)
    pre_yaw = yaw;
  else
  {
    pre_yaw = yaw + M_PI;
    angle_normalize(pre_yaw);
  }
  robot_yaw_ = pre_yaw;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::check_arrive_point()
{
  double dist = sqrt(pow(target_pose_.pose.position.x - robot_tf_pose_.position.x, 2)
              + pow(target_pose_.pose.position.y - robot_tf_pose_.position.y, 2));
  if(dist < arriving_range_dis_ )
  {
    arriving_end_point_ = true;
    twist_linear_step_ = 0;
  }else{
    arriving_end_point_ = false;
    arriving_end_direction_ = false;
  }
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::check_arrive_direction()
{
  double angle_error = target_yaw_ - robot_yaw_;
  angle_normalize(angle_error);

  if(std::abs(angle_error) < arriving_range_angle_)
  {
    arriving_end_direction_ = true;
    if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    {
      auto req = std::make_shared<campusrover_msgs::srv::ElevatorStatusChecker::Request>();
      req->node_name.data = "planner";
      req->status.data = arriving_end_direction_;
      ElevatorStatusCheckCallService(req);
    }
    else if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH)
    {
      global_status_check_pub_->publish(std_msgs::msg::Empty());
    }
    twist_angular_step_ = 0;
  }else{
    arriving_end_direction_ = false;
  }
}
//----------------------------------------------------------------------------------------------

void PathFollowingNode::TimerCallback()
{
  if(!action_flag_)
    return;

  if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_BUTTON_PARKING)
  {
    TwistPublish(max_linear_velocity_, active_angular_);
  }
  else
  {
    if (get_following_path_)
    {
      if (get_costmap_data_ || get_obstacle_data_|| !enble_costmap_obstacle_)
      {
        UpdateCampusRoverPoseFromTF();
        if(enble_pullover_mode_ && !call_dwa_enable_srv_)
        {
          PulloverFunction();
        }
        if(!arriving_end_direction_)
        {
          if(!arriving_end_point_)
          {
            check_arrive_point();
            if(arriving_end_point_)
            {
              moving_to_target_direction();
              RCLCPP_INFO(this->get_logger(), "1");
              return;
            }
            moving_to_target_point();
            RCLCPP_INFO(this->get_logger(), "2");
          }
          else
          {
            check_arrive_direction();
            if(arriving_end_direction_)
            {
              status_msg_ = 4;
              TwistPublish(0.0, 0.0);
              RCLCPP_INFO(this->get_logger(), "3");
              return;
            }
            moving_to_target_direction();
            RCLCPP_INFO(this->get_logger(), "4");
          }
        }
        else
        {
          check_arrive_point();
          status_msg_ = 4;
          RCLCPP_INFO(this->get_logger(), "5");
          TwistPublish(0.0, 0.0);
        }
      }
      else
      {
        status_msg_ = 2;
        RCLCPP_INFO(this->get_logger(), "6");
        TwistPublish(0.0, 0.0);
      }
    }
    else
    {
      status_msg_ = 1;
      RCLCPP_INFO(this->get_logger(), "7");
      TwistPublish(0.0, 0.0);
    }
  }
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::MsgsTimerCallback()
{
  if (status_msg_ == 1)
    RCLCPP_WARN(this->get_logger(), "path following : Without Global Path to follow, Waiting for the Path input");
  else if (status_msg_ == 2)
    RCLCPP_WARN(this->get_logger(), "path following : Without costmap input , Waiting for the costmap input");
  else if (status_msg_ == 3)
    RCLCPP_INFO(this->get_logger(), "path following : detect obstacle");
  else if (status_msg_ == 4)
    RCLCPP_INFO(this->get_logger(), "path following : Arrival the destination");
  status_msg_ = 0;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::moving_to_target_point()
{
  double x_p, y_p, dist_fp_p, closest_dist;
  int closest_id = 0;
  double looking_dist, ob_dist, front_ob_closest_dis = 0.0;
  int target_point_id = 0, front_ob_detect_id = 0;
  geometry_msgs::msg::PoseStamped target_pose;

  // 找最近點
  for (size_t cp = 0; cp < following_path_.poses.size(); cp++)
  {
    x_p = following_path_.poses[cp].pose.position.x;
    y_p = following_path_.poses[cp].pose.position.y;
    dist_fp_p = sqrt(pow(robot_tf_pose_.position.x - x_p,2) + pow(robot_tf_pose_.position.y - y_p,2));
    if (cp == 0 || dist_fp_p < closest_dist)
    {
      closest_dist = dist_fp_p;
      closest_id = cp;
    }
  }

  target_point_id = closest_id;
  front_ob_detect_id = closest_id;

  // 找目標點
  if (closest_id < static_cast<int>(following_path_.poses.size()) - 1)
  {
    for(int fp = closest_id; fp < static_cast<int>(following_path_.poses.size()); fp++)
    {
      looking_dist = sqrt(pow(following_path_.poses[fp].pose.position.x - robot_tf_pose_.position.x, 2) 
                        + pow(following_path_.poses[fp].pose.position.y - robot_tf_pose_.position.y, 2));
      if(looking_dist <= target_point_dis_)
        target_point_id = fp;
      if(looking_dist <= obstacle_detect_max_dis_)
        front_ob_detect_id = fp;
      else
        break;
    }
  }

  target_pose.header.frame_id = following_path_.header.frame_id;
  target_pose.pose.position = following_path_.poses[target_point_id].pose.position;
  target_pose.pose.orientation = following_path_.poses[target_point_id].pose.orientation;

  // 障礙物偵測
  bool detect_front_obstacle = false;
  for(int path_count = closest_id; path_count < front_ob_detect_id; path_count++)
  {
    geometry_msgs::msg::PoseStamped following_path_pose_b, following_path_pose_g;
    following_path_pose_g.header.frame_id = following_path_.header.frame_id;
    following_path_pose_g.pose.position = following_path_.poses[path_count].pose.position;
    following_path_pose_g.pose.orientation = following_path_.poses[path_count].pose.orientation;

    try {
      tf2::doTransform(following_path_pose_g, following_path_pose_b,
                      tf_buffer_->lookupTransform(
                          robot_frame_,                 // target
                          following_path_pose_g.header.frame_id,  // source
                          tf2::TimePointZero             // latest transform
                      ));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    }

    for(size_t ob_count = 0; ob_count < obstacle_poses_.poses.size(); ob_count++)
    {
      ob_dist = sqrt(pow(following_path_pose_b.pose.position.x - obstacle_poses_.poses[ob_count].position.x, 2) 
                   + pow(following_path_pose_b.pose.position.y - obstacle_poses_.poses[ob_count].position.y, 2));
      if(ob_dist < obstacle_range_)
      {
        front_ob_closest_dis = sqrt(pow(following_path_pose_b.pose.position.x, 2) 
                                 + pow(following_path_pose_b.pose.position.y, 2));
        detect_front_obstacle = true;
        break;
      }
    }
    if(detect_front_obstacle)
      break;
  }

  // 控制計算
  double direction_yaw = atan2(target_pose.pose.position.y - robot_tf_pose_.position.y,
                               target_pose.pose.position.x - robot_tf_pose_.position.x);
  double yaw_error = direction_yaw - robot_yaw_;
  angle_normalize(yaw_error);
  double ang_vel = yaw_error * speed_pid_k_;
  double len_vel = direction_inverse_ ? -max_linear_velocity_ : max_linear_velocity_;

  // 障礙物速度調整
  if(detect_front_obstacle)
  {
    if(front_ob_closest_dis <= obstacle_detect_min_dis_)
      len_vel = 0.0;
    else
      len_vel = len_vel * (front_ob_closest_dis - obstacle_detect_min_dis_) / (obstacle_detect_max_dis_ - obstacle_detect_min_dis_);
  }

  // 角度依賴線速度
  if(enable_linear_depend_angular_)
  {
    if(std::abs(yaw_error) >= max_angle_of_linear_profile_)
      len_vel = 0.0;
    else if(std::abs(yaw_error) >= min_angle_of_linear_profile_)
      len_vel = len_vel * (1.0 - ((std::abs(yaw_error) - min_angle_of_linear_profile_) / max_angle_of_linear_profile_));
  }

  // 目標點距離減速
  double min_reduce_dis = 0.8;
  double target_dist = sqrt(pow(following_path_.poses.back().pose.position.x - robot_tf_pose_.position.x, 2) 
                          + pow(following_path_.poses.back().pose.position.y - robot_tf_pose_.position.y, 2));
  if(target_dist < min_reduce_dis)
  {
    if(target_dist / min_reduce_dis > 1.0)
      len_vel = max_linear_velocity_;
    else if(target_dist / min_reduce_dis < 0.1)
      len_vel = min_linear_velocity_;
    else
      len_vel = len_vel * (target_dist / min_reduce_dis);
  }

  TwistProfile(len_vel, ang_vel);
  TwistPublish(len_vel, ang_vel);
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::moving_to_target_direction()
{
  double yaw_error = target_yaw_ - robot_yaw_;
  angle_normalize(yaw_error);

  double ang_vel = yaw_error * speed_pid_k_;

  if (ang_vel > 0 && ang_vel < min_angular_velocity_)
    ang_vel = min_angular_velocity_;
  else if (ang_vel < 0 && ang_vel > -min_angular_velocity_)
    ang_vel = -min_angular_velocity_;

  TwistPublish(0.0, ang_vel);
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::TwistPublish(double x, double z)
{
  geometry_msgs::msg::Twist pub_twist;

  if(z > max_angular_velocity_)
    pub_twist.angular.z = max_angular_velocity_;
  else if(z < -max_angular_velocity_)
    pub_twist.angular.z = -max_angular_velocity_;
  else
    pub_twist.angular.z = z;

  pub_twist.linear.x = x;

  // make_twist_path(pub_twist.linear.x, pub_twist.angular.z); // 如有需要可啟用
  twist_pub_->publish(pub_twist);
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::TwistProfile(double &profile_linear_x, double &profile_angular_z)
{
  const double step_linear_x_ = 0.01;
  const double step_angular_z_ = 0.02;
  const double vel_tolerance = 0.015;

  // 線速度平滑
  if(std::abs(profile_linear_x - twist_linear_step_) <= vel_tolerance)
  {
    twist_linear_step_ = profile_linear_x;
  }
  else
  {
    if (profile_linear_x > twist_linear_step_)
      twist_linear_step_ += step_linear_x_;
    else if (profile_linear_x < twist_linear_step_)
      twist_linear_step_ -= step_linear_x_ * 1.5;
    else
      twist_linear_step_ = profile_linear_x;
  }

  // 角速度平滑
  if(std::abs(profile_angular_z - twist_angular_step_) <= vel_tolerance)
  {
    twist_angular_step_ = profile_angular_z;
  }
  else
  {
    if (profile_angular_z > twist_angular_step_)
      twist_angular_step_ += step_angular_z_;
    else if (profile_angular_z < twist_angular_step_)
      twist_angular_step_ -= step_angular_z_;
    else
      twist_angular_step_ = profile_angular_z;
  }

  profile_linear_x = twist_linear_step_;
  profile_angular_z = twist_angular_step_;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::PulloverFunction()
{
  geometry_msgs::msg::PoseStamped globle_path_pose_g;
  geometry_msgs::msg::PoseStamped globle_path_pose_b;
  int gp_closest_id = 0;
  double dist_fp_p = 0.0;
  double closest_dist = 0.0;
  double looking_dist = 0.0;
  double ob_dist = 0.0;
  double back_ob_closest_dis = 0.0;
  int back_ob_detect_id = 0;

  bool detect_back_obstacle = false;
  static bool detect_back_obstacled = false;

  // 找最近點
  for (size_t cp = 0; cp < globle_path_.poses.size(); cp++)
  {
    dist_fp_p = sqrt(pow(robot_tf_pose_.position.x - globle_path_.poses[cp].pose.position.x,2) 
                    + pow(robot_tf_pose_.position.y - globle_path_.poses[cp].pose.position.y,2));
    if (cp == 0 || dist_fp_p < closest_dist)
    {
      closest_dist = dist_fp_p;
      gp_closest_id = cp;
    }
  }

  // 找偵測範圍內的點
  if(gp_closest_id > 0)
  {
    for(int p = gp_closest_id; p > 0; p--)
    {
      looking_dist = sqrt(pow(globle_path_.poses[p].pose.position.x - robot_tf_pose_.position.x, 2) 
                        + pow(globle_path_.poses[p].pose.position.y - robot_tf_pose_.position.y, 2));
      if(looking_dist <= back_obstacle_detect_max_dis_)
        back_ob_detect_id = p;
      else
        break;
    }
  }

  // 障礙物偵測
  for(int path_count = gp_closest_id; path_count > back_ob_detect_id; path_count--)
  {
    if(detect_back_obstacle)
      break;

    globle_path_pose_g.header.frame_id = globle_path_.header.frame_id;
    globle_path_pose_g.pose.position = globle_path_.poses[path_count].pose.position;
    globle_path_pose_g.pose.orientation = globle_path_.poses[path_count].pose.orientation;

    try
    {
      tf2::doTransform(globle_path_pose_g, globle_path_pose_b,
                      tf_buffer_->lookupTransform(
                          robot_frame_,                     // target frame
                          globle_path_pose_g.header.frame_id,  // source frame
                          tf2::TimePointZero                 // 最新 transform
                      ));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
      return;
    }


    for(size_t ob_count = 0; ob_count < obstacle_poses_.poses.size(); ob_count++)
    {
      ob_dist = sqrt(pow(globle_path_pose_b.pose.position.x - obstacle_poses_.poses[ob_count].position.x, 2) 
                   + pow(globle_path_pose_b.pose.position.y - obstacle_poses_.poses[ob_count].position.y, 2));
      if(ob_dist < obstacle_range_)
      {
        back_ob_closest_dis = sqrt(pow(globle_path_pose_b.pose.position.x, 2) 
                                 + pow(globle_path_pose_b.pose.position.y, 2));
        detect_back_obstacle = true;
        break;
      }
    }
  }

  //-----Pull Over Mode-----//
  static rclcpp::Time detect_back_ob_first_time;
  double detect_back_ob_continue_time = 0.0;
  static rclcpp::Time no_detect_back_ob_first_time;
  double no_detect_back_ob_continue_time = 0.0;
  static bool call_pullover_srv = false;

  //detect obstacle process
  if(detect_back_obstacle)
  {
    if(!detect_back_obstacled)
    {
      detect_back_ob_first_time = this->now();
      detect_back_obstacled = true;
    }

    rclcpp::Time time_now = this->now();
    detect_back_ob_continue_time = (time_now - detect_back_ob_first_time).seconds();

    if(detect_back_obstacled && detect_back_ob_continue_time > 2.0)
    {
      if(!call_pullover_srv)
      {
        auto pullover_srv = std::make_shared<campusrover_msgs::srv::PullOverPathGenerator::Request>();
        pullover_srv->pullover_forward_dis = 2.0;
        pullover_srv->pullover_shift_dis = 0.55;
        pullover_srv->reference_path = globle_path_;

        auto future = pullover_planner_client_->async_send_request(pullover_srv);
        // 等待結果（同步寫法，建議用非同步 callback）
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS)
        {
          auto response = future.get();
          pullover_path_ = response->pullover_path;
          path_sub_mode_ = campusrover_msgs::srv::PlannerFunction::Request::MODE_PULLOVER_PATH;
          UpdateInputPath(pullover_path_);
          RCLCPP_INFO(this->get_logger(), "Start Pull Over Mode");
          call_pullover_srv = true;
          return;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to call Pull Over Planner Service");
          return;
        }
      }
    }
  }
  else
  {
    if(detect_back_obstacled)
    {
      no_detect_back_ob_first_time = this->now();
      detect_back_obstacled = false;
    }

    rclcpp::Time time_now = this->now();
    no_detect_back_ob_continue_time = (time_now - no_detect_back_ob_first_time).seconds();

    if(call_pullover_srv)
    {
      if(no_detect_back_ob_continue_time > 3.0)
      {
        path_sub_mode_ = campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH;
        UpdateInputPath(globle_path_);
        RCLCPP_INFO(this->get_logger(), "Stop Pull Over Mode");
        call_pullover_srv = false;
        arriving_end_point_ = false;
        arriving_end_direction_ = false;
        return;
      }
    }
  }
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::UpdateInputPath(const nav_msgs::msg::Path &path)
{
  double roll, pitch, yaw;

  get_following_path_ = false;
  following_path_.poses.clear();

  if(path.poses.size() == 0)
    return;

  // ROS 2: stamp 為 builtin_interfaces::msg::Time，需轉換為 rclcpp::Time
  rclcpp::Time path_stamp(path.header.stamp);
  if(path_stamp < planner_start_time && path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    return;

  following_path_.header.frame_id = path.header.frame_id;
  following_path_.poses.clear();
  following_path_ = path;

  target_pose_.header.frame_id = path.header.frame_id;
  target_pose_.pose.position = path.poses.back().pose.position;
  target_pose_.pose.orientation = path.poses.back().pose.orientation;

  tf2::Quaternion q(
    target_pose_.pose.orientation.x,
    target_pose_.pose.orientation.y,
    target_pose_.pose.orientation.z,
    target_pose_.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  m.getRPY(roll, pitch, yaw);
  target_yaw_ = yaw;

  get_following_path_ = true;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::angle_normalize(double &angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::ElevatorPathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  if(path_sub_mode_ != campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    return;

  if(path->poses.size() == 0)
    return;

  elevator_path_.header.frame_id = path->header.frame_id;
  elevator_path_.poses.clear();
  elevator_path_ = *path;

  UpdateInputPath(elevator_path_);
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  if(path->poses.size() == 0)
    return;

  globle_path_.header.frame_id = path->header.frame_id;
  globle_path_.poses.clear();
  globle_path_ = *path;

  if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH)
  {
    UpdateInputPath(globle_path_);
  }
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  geometry_msgs::msg::Pose ob_pose;
  geometry_msgs::msg::PoseStamped ob_posestamped;
  geometry_msgs::msg::PoseStamped base_ob_pose;
  double data;
  double value;

  if(!enble_costmap_obstacle_)
    return;

  costmap_data_.info.resolution = map->info.resolution;
  obstacle_poses_.poses.clear();

  if (map->header.frame_id == robot_frame_)
  {
    for (size_t i = 0; i < map->data.size(); i++)
    {
      data = map->data[i];
      value = i;

      if (std::abs(data) > threshold_occupied_)
      {
        ob_pose.position.y = (std::floor(value / map->info.width) * map->info.resolution) + map->info.origin.position.y;
        ob_pose.position.x = ((value - (map->info.width * std::floor(value / map->info.width))) * map->info.resolution) + map->info.origin.position.x;

        if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ &&
            ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
        {
          obstacle_stop_cmd_ = true;
          status_msg_ = 3;
          break;
        }
        else
        {
          obstacle_stop_cmd_ = false;
        }
        obstacle_poses_.poses.push_back(ob_pose);
      }
    }
  }
  else
  {
    ob_posestamped.header.frame_id = map->header.frame_id;

    for (size_t i = 0; i < map->data.size(); i++)
    {
      data = map->data[i];

      if (std::abs(data) > threshold_occupied_)
      {
        value = i;

        ob_posestamped.pose.position.y = (std::floor(value / map->info.width) * map->info.resolution) + map->info.origin.position.y;
        ob_posestamped.pose.position.x = ((value - (map->info.width * std::floor(value / map->info.width))) * map->info.resolution) + map->info.origin.position.x;

        try {
            tf2::doTransform(ob_posestamped, base_ob_pose,
                            tf_buffer_->lookupTransform(
                                robot_frame_,                    // target frame
                                ob_posestamped.header.frame_id,  // source frame
                                tf2::TimePointZero               // 最新 transform
                            ));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return;
        }


        ob_pose.position.y = base_ob_pose.pose.position.y;
        ob_pose.position.x = base_ob_pose.pose.position.x;

        if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ &&
            ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
        {
          obstacle_stop_cmd_ = true;
          status_msg_ = 3;
          break;
        }
        else
        {
          obstacle_stop_cmd_ = false;
        }
        obstacle_poses_.poses.push_back(ob_pose);
      }
    }
  }

  get_costmap_data_ = true;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  current_v_ = odom->twist.twist.linear.x;
  current_w_ = odom->twist.twist.angular.z;

  if(!get_velocity_data_)
    RCLCPP_INFO(this->get_logger(), "path following : Get the Odom input!!");
  get_velocity_data_ = true;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::CB_customObstacle(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg)
{
  geometry_msgs::msg::Pose ob_pose;
  geometry_msgs::msg::PoseStamped obstacle_r;
  geometry_msgs::msg::PoseStamped obstacle_g;

  obstacle_poses_.poses.clear();

  if (obst_msg->header.frame_id == robot_frame_)
  {
    for(size_t i = 0; i < obst_msg->obstacles.size(); i++)
    {
      ob_pose.position.x = obst_msg->obstacles[i].polygon.points[0].x;
      ob_pose.position.y = obst_msg->obstacles[i].polygon.points[0].y;    
      obstacle_poses_.poses.push_back(ob_pose);
    }
  }
  else
  {
    for(size_t i = 0; i < obst_msg->obstacles.size(); i++)
    {
      obstacle_g.header.frame_id = obst_msg->obstacles[i].header.frame_id;
      obstacle_g.pose.position.x = obst_msg->obstacles[i].polygon.points[0].x;
      obstacle_g.pose.position.y = obst_msg->obstacles[i].polygon.points[0].y;
      try {
          tf2::doTransform(obstacle_g, obstacle_r,
                          tf_buffer_->lookupTransform(
                              robot_frame_,                  // target frame
                              obstacle_g.header.frame_id,    // source frame
                              tf2::TimePointZero             // 最新 transform
                          ));
      } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
          return;
      }


      ob_pose.position.x = obstacle_r.pose.position.x;
      ob_pose.position.y = obstacle_r.pose.position.y;    
      obstacle_poses_.poses.push_back(ob_pose);
    }
  }

  get_obstacle_data_ = true;
}
//-----------------------------------------------------------------------------------------------

bool PathFollowingNode::ServiceCallback(
  const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> req,
  std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> res)
{
  static int last_mode;

  action_flag_ = req->action.data;
  direction_inverse_ = req->direction_inverse.data;
  enble_costmap_obstacle_ = req->obstacle_avoidance.data;
  max_linear_velocity_ = req->speed_parameter.linear.x;
  max_angular_velocity_ = std::abs(req->speed_parameter.angular.z);
  active_angular_ = req->speed_parameter.angular.z;

  path_sub_mode_ = req->mode;

  RCLCPP_INFO(this->get_logger(), "recrvie planner function :");
  RCLCPP_INFO(this->get_logger(), "  action_flag : %d", action_flag_);
  RCLCPP_INFO(this->get_logger(), "  direction_inverse : %d", direction_inverse_);
  RCLCPP_INFO(this->get_logger(), "  enble_costmap_obstacle_ : %d", enble_costmap_obstacle_);
  // 可根據需求補充 speed_parameter 印出

  if(!action_flag_ || path_sub_mode_ != last_mode)
  {
    arriving_end_point_ = false;
    arriving_end_direction_ = false;
    get_costmap_data_ = false;
    twist_linear_step_ = 0;
    twist_angular_step_ = 0;
  }

  planner_start_time = this->now();

  if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH)
    UpdateInputPath(globle_path_);
  else if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    UpdateInputPath(elevator_path_);
  else if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_PULLOVER_PATH)
    UpdateInputPath(pullover_path_);

  if(!enble_costmap_obstacle_)
    obstacle_poses_.poses.clear();

  last_mode = path_sub_mode_;

  return true;
}
//-----------------------------------------------------------------------------------------------

void PathFollowingNode::ElevatorStatusCheckCallService(
  std::shared_ptr<campusrover_msgs::srv::ElevatorStatusChecker::Request> req)
{
  RCLCPP_INFO(this->get_logger(), "===========elevator planner status check=============");
  RCLCPP_INFO(this->get_logger(), "Request message: node_name=%s, status=%d", req->node_name.data.c_str(), req->status.data);

  auto future = elevator_status_check_client_->async_send_request(req);

  // 等待結果（同步寫法，建議用非同步 callback）
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "elevator planner status check : Failed to call service");
  }
}
//-----------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollowingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

