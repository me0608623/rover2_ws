#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iterator>
#include "rover_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

RoverDriver::RoverDriver(const YAML::Node &config, rclcpp::Node::SharedPtr nh):node_(nh)
{
  struct ifreq ifr;

  try
  {
    std::cout << "[rover_driver] Initializing Drvier ..." << std::endl;
    std::string can = config["can"]["device"].as<std::string>();
    
    wheel_base_ =  config["base"]["wheel_base_length"].as<double>();
    wheel_r_d_ =  config["base"]["right_wheel_diameter"].as<double>();
    wheel_l_d_ =  config["base"]["left_wheel_diameter"].as<double>();
    reduction_ratio_ =  config["base"]["reduction_ratio"].as<double>();
    encoder_res_ =  config["base"]["encoder_resolution"].as<int>();

    pub_tf_ = config["ros"]["publish_tf"].as<bool>();
    pub_odom_ = config["ros"]["publish_odom"].as<bool>();
    pub_rate_ = config["ros"]["publish_rate"].as<double>();
    odom_frame_ = config["ros"]["odom_frame"].as<std::string>();
    base_frame_ = config["ros"]["base_frame"].as<std::string>();

    acc_max_ = acc_max_default_ = config["driver"]["acc_max"].as<double>();
    acc_step_ = acc_step_default_ = config["driver"]["acc_step"].as<double>();
    dec_max_ = dec_max_default_ = config["driver"]["dec_max"].as<double>();
    dec_step_ = dec_step_default_ = config["driver"]["dec_step"].as<double>();
    vel_tolerance_ = vel_tolerance_default_ = config["driver"]["vel_tolerance"].as<double>();
    profile_omega_max_ = profile_omega_max_default_ = config["driver"]["profile_omega_max"].as<double>();
    profile_omega_step_ = profile_omega_step_default_ = config["driver"]["profile_omega_step"].as<double>();
    omega_tolerance_ = omega_tolerance_default_ = config["driver"]["omega_tolerance"].as<double>();
    max_speed_ = max_speed_default_ = config["driver"]["max_speed"].as<double>();

    use_profile_ = true;

    // odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 5);
     odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

    if ((s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
    {
      perror("Socket");
    }
    struct sockaddr_can addr;
    strcpy(ifr.ifr_name, can.c_str());
    ioctl(s_, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s_, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
    {
      perror("Bind error");
      return;
    }
    // Set CAN filter
    struct can_filter rfilter[2];
    rfilter[0].can_id   = (canid_t)LEFT_WHEEL;
	  rfilter[0].can_mask = 0xFF0;
	  rfilter[1].can_id   = (canid_t)RIGHT_WHEEL;
	  rfilter[1].can_mask = 0xFF0;
    struct timeval t = {0, 1000};
    setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    setsockopt(s_, SOL_SOCKET, SO_SNDTIMEO, &t, sizeof(t));
    setsockopt(s_, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));
    // Disable motors
    sendFrame(BROADCASTER, P2M_BROCAST, MOTOR, 0);
    // sendFrame(BROCASRER, P2M_BROCAST, SPEED_ACC_TIME, 0x1010);
    if (pub_tf_ || pub_odom_)
    {
      // set auto report time
      sendFrame(BROADCASTER, P2M_BROCAST, 
                CAN_REPORT_TIME, 1000/pub_rate_,
                CAN_AUTO_RP, 1);
    }
    else
    {
      sendFrame(BROADCASTER, P2M_BROCAST, 
                CAN_REPORT_TIME, 0);
    }
    // Enable motor
    sendFrame(BROADCASTER, P2M_BROCAST, MOTOR, 1);
    // ROS_INFO("[rover_driver] Driver Intitialed");
    RCLCPP_INFO(node_->get_logger(), "[rover_driver] Driver Initialized");
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

RoverDriver::~RoverDriver()
{
  closeSocket();
}

bool RoverDriver::sendFrame(
  CanAddr device, unsigned char *frame)
{
  struct can_frame cframe;
  cframe.can_id = device;
  cframe.can_dlc = 8;
  std::copy(frame, frame+8, cframe.data);
  if (write(s_, &cframe, sizeof(struct can_frame)) != sizeof(struct can_frame)) 
  {
    perror("Write error");
    return 1;
  }
  return 0;
}


bool RoverDriver::sendFrame(
  CanAddr device, FuncCode func_code, 
  RegAddr reg1, int value1, 
  RegAddr reg2, int value2)
{
  unsigned char frame[8];
  frame[0] = 0x00;
  frame[1] = func_code;
  frame[2] = reg1;
  frame[3] = value1 >> 8;
  frame[4] = value1 & 0x0000FF;
  frame[5] = reg2;
  frame[6] = value2 >> 8;
  frame[7] = value2 & 0x0000FF;
  return sendFrame(device, frame);
}

int RoverDriver::fromRPM(double rpm)
{
  return (int)(rpm/6000*16384);
}

void RoverDriver::getWheelsRpm(
    const double linear_vel, 
    const double angular_vel,
    double &rpm_l, double &rpm_r)
{
  double half_l = wheel_base_/2.0;
  rpm_l = -60*reduction_ratio_*(linear_vel-half_l*angular_vel)/(wheel_l_d_*M_PI);
  rpm_r =  60*reduction_ratio_*(linear_vel+half_l*angular_vel)/(wheel_r_d_*M_PI);
}

void RoverDriver::closeSocket()
{
  sendFrame(BROADCASTER, P2M_BROCAST, MOTOR, 0);
  sendFrame(BROADCASTER, P2M_BROCAST, CAN_REPORT_TIME, 0);
  if (close(s_) < 0) 
  {
    perror("Close error");
  }
}

void RoverDriver::driveStatusSrv(
  const std::shared_ptr<campusrover_msgs::srv::DriveStatus::Request> request,
  std::shared_ptr<campusrover_msgs::srv::DriveStatus::Response> response)
{
  typedef campusrover_msgs::srv::DriveStatus::Request DS;
  auto now = node_->get_clock()->now();
  std::stringstream ss;
  ss << "[" << now.seconds() << "] ";

  switch (request->drive_status)
  {
    case DS::DRIVE_STATUS_STOP:
      sendFrame(BROADCASTER, P2M_BROCAST, MOTOR, 1);
      {
        std::lock_guard<std::mutex> lock(srv_flag_mutex_);
        srv_flag_ = STOP;
      }
      ss << "Status: STOP";
      RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
      break;

    case DS::DRIVE_STATUS_RELEASE:
      {
        std::lock_guard<std::mutex> lock(srv_flag_mutex_);
        srv_flag_ = RELEASE;
      }
      ss << "Status: RELEASE";
      RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
      break;

    case DS::DRIVE_STATUS_CMD_CONTROL:
      sendFrame(BROADCASTER, P2M_BROCAST, MOTOR, 1);
      {
        std::lock_guard<std::mutex> lock(srv_flag_mutex_);
        srv_flag_ = CMD_CONTROL;
      }
      ss << "Status: CMD_CONTROL";
      RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
      break;

    default:
      ss << "Status: UNKNOWN";
      RCLCPP_WARN(node_->get_logger(), "%s", ss.str().c_str());
      break;
  }
}

void RoverDriver::driveEncoderSrv(
  const std::shared_ptr<campusrover_msgs::srv::EncoderCount::Request> req,
  std::shared_ptr<campusrover_msgs::srv::EncoderCount::Response> res)
{
  (void)req;
  res->left_count = left_encoder_count_;
  res->right_count = right_encoder_count_;
}

void RoverDriver::driveProfileSrv(
  const std::shared_ptr<campusrover_msgs::srv::DriveProfile::Request> req,
  std::shared_ptr<campusrover_msgs::srv::DriveProfile::Response> res)
{
  typedef campusrover_msgs::srv::DriveProfile::Request DP;
  switch (req->drive_profile)
  {
    case DP::DRIVE_PROFILE_NOUSE:
      use_profile_=false;
      std::cout << "DriveProfile: NO USE PROFILE" << std::endl;
    break;
    case DP::DRIVE_PROFILE_DEFAULT:
      use_profile_=true;
      acc_max_ = acc_max_default_;
      acc_step_ = acc_step_default_;
      dec_max_ = dec_max_default_;
      dec_step_ = dec_step_default_;
      vel_tolerance_ = vel_tolerance_default_;
      profile_omega_max_ = profile_omega_max_default_;
      profile_omega_step_ = profile_omega_step_default_;
      omega_tolerance_ = omega_tolerance_default_;
      std::cout << "DriveProfile: USE DEFAULT PROFILE" << std::endl;
    break;
    case DP::DRIVE_PROFILE_SET:
      use_profile_=true;
      acc_max_ = req->acc_max;
      acc_step_ = req->acc_step;
      dec_max_ = req->dec_max;
      dec_step_ = req->dec_step;
      vel_tolerance_ = req->vel_tolerance;
      profile_omega_max_ = req->profile_omega_max;
      profile_omega_step_ = req->profile_omega_step;
      omega_tolerance_ = req->omega_tolerance;
      std::cout << "DriveProfile: SET PROFILE" << std::endl;
      std::cout << "acc_max : " << acc_max_ << "\nacc_step : " << acc_step_ << "\ndec_max : " << dec_max_ << "\ndec_step : " << dec_step_ << "\nvel_tolerance : " << vel_tolerance_ << "\nprofile_omega_max : " << profile_omega_max_ << "\nprofile_omega_step : " << profile_omega_step_ << "\nomega_tolerance : " << omega_tolerance_  << std::endl;
    break;
    default:
    break;
  }
}
void RoverDriver::twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cmd_v_ = twist->linear.x;
  cmd_w_ = twist->angular.z;
}

double RoverDriver::getProfileSpeed(double t, double speed_tar, bool reset = false)
{
  static double acc_out = 0;
  static double vel_out = 0;
  double dt = t;
  double d_vel;

  if(reset == true)
  {
    acc_out = 0;
    vel_out = 0;
  }

  if (std::abs(speed_tar - vel_out) > vel_tolerance_) 
  {
    if (speed_tar > vel_out) 
    {
      acc_out = std::min(acc_out+acc_step_, acc_max_);
    } else {
      acc_out = std::max(acc_out-dec_step_, -dec_max_);
    }
    d_vel = acc_out*dt;
    vel_out += d_vel;
  }
  double vel_tolerance = std::max(vel_tolerance_, d_vel);
  if (std::abs(speed_tar - vel_out) <= vel_tolerance)
  {
    vel_out = speed_tar;
    acc_out = 0;
  }

  return vel_out;
}

double RoverDriver::getProfileOmega(double t, double angle_vel_tar)
{
  static double acc_out = 0;
  static double angle_vel_out = 0;
  double dt = t;
  if (std::abs(angle_vel_tar - angle_vel_out) > omega_tolerance_) 
  {
    if (angle_vel_tar > angle_vel_out) 
    {
      acc_out = std::min(acc_out+profile_omega_step_, profile_omega_max_);
    } else {
      acc_out = std::max(acc_out-profile_omega_step_, -profile_omega_max_);
    }
    angle_vel_out += acc_out*dt;
  }
  if (std::abs(angle_vel_tar - angle_vel_out) <= omega_tolerance_)
  {
    angle_vel_out = angle_vel_tar;
    acc_out = 0;
  }

  return angle_vel_out;
}

double RoverDriver::getLimit(double input ,double limit)
{
  double sign;
  if(input>0)sign=1;
  else sign=-1;

  if(std::abs(input)>limit)return limit*sign;
  else return input;
}

// void RoverDriver::pubTimerCallback(const ros::TimerEvent& event)
void RoverDriver::pubTimerCallback()
{
  static double rpm_r = 0;
  static double rpm_l = 0;
  double cmd_v, cmd_w;
  switch (srv_flag_)
  {
  case STOP:
    cmd_v = getProfileSpeed(0.1, 0);
    cmd_w = 0;
    getWheelsRpm(getLimit(cmd_v,max_speed_), cmd_w, rpm_l, rpm_r);
    sendFrame(LEFT_WHEEL, P2P_WRITE_REQ, SET_SPEED, fromRPM(rpm_l));
    sendFrame(RIGHT_WHEEL, P2P_WRITE_REQ, SET_SPEED, fromRPM(rpm_r)); 
    // sendFrame(BROADCASTER, P2M_BROCAST, SET_SPEED, fromRPM(0)); 
    break;
  case RELEASE:
    getProfileSpeed(0,0,true);
    sendFrame(BROADCASTER, P2M_BROCAST, MOTOR, 0);
    break;
  case CMD_CONTROL:
    if(use_profile_)
    {
      cmd_v = getProfileSpeed(0.1, cmd_v_);
      cmd_w = cmd_w_;
    }
    else
    {
      cmd_v = cmd_v_;
      cmd_w = cmd_w_;
    }

    getWheelsRpm(getLimit(cmd_v,max_speed_), cmd_w, rpm_l, rpm_r);
    sendFrame(LEFT_WHEEL, P2P_WRITE_REQ, SET_SPEED, fromRPM(rpm_l));
    sendFrame(RIGHT_WHEEL, P2P_WRITE_REQ, SET_SPEED, fromRPM(rpm_r)); 
    break;
  default:
    break;
  }
}

// void RoverDriver::readEncoderTimerCallback(const ros::TimerEvent& event)
void RoverDriver::readEncoderTimerCallback()
{
  struct can_frame frame;
  if (pub_tf_ || pub_odom_)
  {
    int nbytes = read(s_, &frame, sizeof(struct can_frame)); 
    if (nbytes > 0) {
      if (frame.can_id == LEFT_WHEEL || frame.can_id == RIGHT_WHEEL)
      {
        RegAddr reg1 = (RegAddr)frame.data[2];
        RegAddr reg2 = (RegAddr)frame.data[5];
        if (reg1 == OUTPUT_POS_H && reg2 == OUTPUT_POS_L) 
        {
          int counts = getCounts(frame.data[3], frame.data[4], frame.data[6], frame.data[7]);
          odomPublish((CanAddr)frame.can_id, counts); 
        }
      }
    }
  }
}

void RoverDriver::readMotorsInfo()
{
  struct can_frame frame;
  if (pub_tf_ || pub_odom_)
  {
    int nbytes = read(s_, &frame, sizeof(struct can_frame));
    if (nbytes > 0) {
      if (frame.can_id == LEFT_WHEEL || frame.can_id == RIGHT_WHEEL)
      {
        RegAddr reg1 = (RegAddr)frame.data[2];
        RegAddr reg2 = (RegAddr)frame.data[5];
        if (reg1 == OUTPUT_POS_H && reg2 == OUTPUT_POS_L) 
        {
          int counts = getCounts(frame.data[3], frame.data[4], frame.data[6], frame.data[7]);
          odomPublish((CanAddr)frame.can_id, counts); 
          // while (read(s_, &frame, sizeof(struct can_frame)) > 0) { }
        }
      }
    }
  }
}

int RoverDriver::getCounts(const unsigned char HH, const unsigned char HL, 
                           const unsigned char LH, const unsigned char LL)
{

  return ((HH<<24) | (HL<<16) | (LH<<8) | LL);
}

bool RoverDriver::isWheelsSync(const rclcpp::Time &cur_time)
{
  // std::cout << "start iswheelsync" << std::endl;
  static rclcpp::Time last_time(0, 0, cur_time.get_clock_type()); // 初始化成同 clock type
  double dt = (cur_time - last_time).seconds();
  last_time = cur_time;
  // std::cout<<"pass iswheelsync"<<std::endl;
  return dt < 0.025;
}

void RoverDriver::odomPublish(CanAddr device, const int count)
{
  static bool flag_l = false;
  static bool flag_r = false;
  static nav_msgs::msg::Odometry odom;
  static double dr_r, dr_l;
  static bool inited = false;
  static bool sync = false;
  static int count_l, count_r;
  static int last_count_l, last_count_r;
  static rclcpp::Time cur_time, last_time;
  static tf2_ros::TransformBroadcaster br(node_); 
  static geometry_msgs::msg::TransformStamped transformStamped;


  cur_time = node_->get_clock()->now();
  switch (device)
  {
  case LEFT_WHEEL:
    left_encoder_count_ = count;
    count_l = count;
    flag_l = true;
    sync = isWheelsSync(cur_time);
    break;
  case RIGHT_WHEEL:
    right_encoder_count_ = count;
    count_r = count;
    flag_r = true;
    sync = isWheelsSync(cur_time);
    break;
  default:
    break;
  }
  if (flag_l && flag_r && sync)
  {
    if (inited)
    {
      int d_c_l = -getCountsDiff(last_count_l, count_l);
      int d_c_r =  getCountsDiff(last_count_r, count_r);
      setOdom(odom, last_time, cur_time, d_c_l, d_c_r);
      if (pub_odom_) odom_pub_->publish(odom);
      if (pub_tf_)
      {
        transformStamped.header.frame_id = odom_frame_;
        transformStamped.child_frame_id = base_frame_;
        transformStamped.header.stamp = odom.header.stamp;
        transformStamped.transform.translation.x = odom.pose.pose.position.x;
        transformStamped.transform.translation.y = odom.pose.pose.position.y;
        transformStamped.transform.translation.z = odom.pose.pose.position.z;
        transformStamped.transform.rotation = odom.pose.pose.orientation;
        br.sendTransform(transformStamped);
      }
      last_count_l = count_l;
      last_count_r = count_r;
      flag_l = false;
      flag_r = false;
      return;
    }
    else
    {
      // odom.header.seq = 0;
      odom.header.frame_id = odom_frame_;
      odom.header.stamp = cur_time;
      odom.child_frame_id  = base_frame_;
      odom.pose.pose.orientation.x = 0;
      odom.pose.pose.orientation.y = 0;
      odom.pose.pose.orientation.z = 0;
      odom.pose.pose.orientation.w = 1;
      last_time = cur_time;
      last_count_l = count_l;
      last_count_r = count_r;
      inited = true;
    }
  }
  else if (flag_l || flag_r)
  {
    last_time = cur_time;
  }
}

int RoverDriver::getCountsDiff(const int count_a, const int count_b)
{
  int d_counts;
   if (count_b - count_a > 0)
   {
      d_counts = (count_b - count_a) < 0x01000000 ? 
                  (count_b - count_a):(count_b - count_a)-0x100000000;
   }
   else
   {
      d_counts = (count_a - count_b) < 0x01000000 ? 
                  (count_b - count_a):(count_b - count_a)+0x100000000;
   }
   return d_counts;
}

void RoverDriver::setOdom(nav_msgs::msg::Odometry &odom,
  const rclcpp::Time &last_time,
  const rclcpp::Time &cur_time,
  const int d_l_counts, 
  const int d_r_counts)
{
  static long long cur_counts_l = 0;
  static long long cur_counts_r = 0;
  static long long last_long_count_l = 0;
  static long long last_long_count_r = 0;
  static double last_theta = 0;
  const double dt = 0.05;
  cur_counts_l += d_l_counts;
  cur_counts_r += d_r_counts;
  double theta = (cur_counts_r*wheel_r_d_-cur_counts_l*wheel_l_d_)*M_PI/(wheel_base_*encoder_res_*reduction_ratio_);
  double d_theta = theta - last_theta;
  double ds_l = ((cur_counts_l - last_long_count_l)*M_PI*wheel_l_d_)/((double)encoder_res_*reduction_ratio_);
  double ds_r = ((cur_counts_r - last_long_count_r)*M_PI*wheel_r_d_)/((double)encoder_res_*reduction_ratio_);
  double ds = (ds_l + ds_r)/2.;
  double dx = ds*cos(d_theta);
  double dy = ds*sin(d_theta);
  double cos_th = cos(last_theta);
  double sin_th = sin(last_theta);
  // Transform theta to Quaternion
  tf2::Quaternion q_theta;
  q_theta.setRPY(0, 0, theta);
  // ROS msg setting
  odom.header.stamp = cur_time;
  odom.pose.pose.position.x += (dx*cos_th-dy*sin_th);
  odom.pose.pose.position.y += (dx*sin_th+dy*cos_th);
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf2::toMsg(q_theta);
  odom.twist.twist.linear.x = ds/dt;
  odom.twist.twist.angular.z = d_theta/dt;
  last_theta = theta;
  last_long_count_l = cur_counts_l;
  last_long_count_r = cur_counts_r;
}


double RoverDriver::getTfOdomPeriod()
{
  return 1.0/pub_rate_;
}

std::string g_config_file;

void GetParameters(rclcpp::Node::SharedPtr nh)
{
  nh->declare_parameter<std::string>("config_file", "");
  nh->get_parameter("config_file", g_config_file);

}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("rover_driver");

  YAML::Node config;
  GetParameters(nh);
  try 
  {
    std::cout << "[rover_driver] Loading config file: " << g_config_file << std::endl;
    config = YAML::LoadFile(g_config_file);
  } catch(YAML::Exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  if (!nh) {
  std::cerr << "nh is nullptr!" << std::endl;
  return 1;
  }
  RoverDriver rover_driver(config, nh);

  std::cout << "Start create service" << std::endl;
  auto drive_status_srv = nh->create_service<campusrover_msgs::srv::DriveStatus>(
    "drive_status",
    std::bind(&RoverDriver::driveStatusSrv, &rover_driver,
             std::placeholders::_1, std::placeholders::_2));
  auto drive_encoder_srv = nh->create_service<campusrover_msgs::srv::EncoderCount>(
    "drive_encoder",
    std::bind(&RoverDriver::driveEncoderSrv, &rover_driver,
              std::placeholders::_1, std::placeholders::_2));
  auto drive_profile_srv = nh->create_service<campusrover_msgs::srv::DriveProfile>(
    "drive_profile",
    std::bind(&RoverDriver::driveProfileSrv, &rover_driver,
              std::placeholders::_1, std::placeholders::_2));  

  auto twist_sub = nh->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1,
    std::bind(&RoverDriver::twistCallback, &rover_driver, std::placeholders::_1));

  std::cout << "Create service success" << std::endl;
  auto pub_timer = nh->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RoverDriver::pubTimerCallback, &rover_driver));

  auto readencoder_timer = nh->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&RoverDriver::readEncoderTimerCallback, &rover_driver));
    // Executor: 多執行緒 spinner 等效 ROS1 MultiThreadedSpinner(2)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(nh);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
