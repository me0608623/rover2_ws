#ifndef ROVER_DRIVER_HPP
#define ROVER_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include "campusrover_msgs/srv/drive_status.hpp"
#include "campusrover_msgs/srv/encoder_count.hpp"
#include "campusrover_msgs/srv/drive_profile.hpp"

#include <string>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <yaml-cpp/yaml.h>

#include <boost/thread/mutex.hpp>

// //ROS
// #include "campusrover_msgs/DriveStatus.h"
// #include "campusrover_msgs/EncoderCount.h"
// #include "campusrover_msgs/DriveProfile.h"
// #include "geometry_msgs/Twist.h"
// #include "nav_msgs/Odometry.h"

// class RoverDriver
// {
// private:
//   /* data */
//   // CANBUS devices ID
//   enum CanAddr {
//     BROADCASTER   = 0x00,
//     LEFT_WHEEL  = 0x01,
//     RIGHT_WHEEL = 0x02
//   };
//   // Function code for motor driver
//   enum FuncCode {
//     P2P_WRITE_REQ   = 0x1A,
//     P2P_WRITE_RES   = 0x1B,
//     P2P_WRITE_ERR   = 0x1C,
//     P2P_READ_REQ    = 0x2A,
//     P2P_READ_RES    = 0x2B,
//     P2P_READ_ERR    = 0x2C,
//     P2M_WRITE_RES   = 0x8A,
//     P2M_WRITE_REQ   = 0x8B,
//     P2M_WRITE_ERR   = 0x8C,
//     P2M_BROCAST     = 0x9A,
//     P2M_BROCAST_ERR = 0x9C
//   };
//   // Register address for motor driver
//   enum RegAddr {
//     MOTOR           = 0x00,
//     INPUT_MODE      = 0x02,
//     CAN_GP          = 0x0B,
//     CAN_REPORT_TIME = 0X0C,
//     CAN_ID          = 0x0D,
//     CAN_AUTO_RP     = 0x2E,
//     SPEED_ACC_TIME  = 0x0A,
//     OUTPUT_A        = 0xE2,
//     OUTPUT_RPM      = 0xE4,
//     OUTPUT_POS_H    = 0xE8,
//     OUTPUT_POS_L    = 0xE9,
//     CLR_ERR         = 0x4A,
//     ERR_STATUS      = 0xE3, 
//     SET_SPEED       = 0x06,
//     NONE            = 0xFF
//   };
//   enum srvFlag {
//     STOP, 
//     RELEASE,
//     CMD_CONTROL
//   };
//   int left_encoder_count_, right_encoder_count_;
//   int s_; 
//   int max_rpm_; // -2300rpm ~ 3600rpm
//   srvFlag srv_flag_;
//   boost::mutex srv_flag_mutex_;
//   double cmd_v_ = 0;
//   double cmd_w_ = 0;
//   boost::mutex cmd_mutex_;
//   // ROS
//   double wheel_base_ = 0.52; // wheels length in meter
//   double wheel_r_d_ = 0.27; // right wheel diameter in meters
//   double wheel_l_d_ = 0.27; // left wheel diameter in meters
//   double reduction_ratio_ = 10; // reduction ratio (one revolution on wheel = ? on motor)

//   int encoder_res_ = 10000; // encoder resolution in cpr (counts per revolution)


//   bool pub_tf_;
//   bool pub_odom_;
//   double pub_rate_;

//   std::string odom_frame_;
//   std::string base_frame_;

//   double acc_max_,acc_max_default; 
//   double acc_step_,acc_step_default;
//   double dec_max_,dec_max_default; 
//   double dec_step_,dec_step_default;
//   double vel_tolerance_,vel_tolerance_default;
//   double profile_omega_max_,profile_omega_max_default; 
//   double profile_omega_step_,profile_omega_step_default;
//   double omega_tolerance_,omega_tolerance_default;
//   double max_speed_,max_speed_default;

//   ros::Publisher odom_pub_;

// public:
//   RoverDriver(const YAML::Node config);
//   ~RoverDriver();
//   bool sendFrame(CanAddr device, unsigned char *frame);
//   bool sendFrame(CanAddr device, FuncCode func_code, 
//                  RegAddr reg1, int value1,
//                  RegAddr reg2 = NONE, int value2 = 0); 
//   int fromRPM(double rpm);
//   void getWheelsRpm(
//     const double linear_vel, 
//     const double angular_vel,
//     double &rpm_l, double &rpm_r);
//   void closeSocket();
//   bool driveStatusSrv(campusrover_msgs::DriveStatus::Request &req,
//                       campusrover_msgs::DriveStatus::Response &res);
//   bool driveEncoderSrv(campusrover_msgs::EncoderCount::Request &req,
//                       campusrover_msgs::EncoderCount::Response &res);

//   bool driveProfileSrv(campusrover_msgs::DriveProfile::Request &req,
//                       campusrover_msgs::DriveProfile::Response &res);

//   void twistCallback(const geometry_msgs::Twist &twist);

//   double getProfileSpeed(double t, double speed_tar, bool reset);
//   double getProfileOmega(double t, double angle_vel_tar);

//   void pubTimerCallback(const ros::TimerEvent& event);
//   void readEcoderTimerCallback(const ros::TimerEvent& event);
//   void readMotorsInfo();
//   int getCounts(const unsigned char HH, const unsigned char HL, 
//                 const unsigned char LH, const unsigned char LL);
//   bool isWheelsSync(const ros::Time cur_time);
//   void odomPublish(CanAddr device, const int count);
//   int getCountsDiff(const int count_a, const int count_b);
//   void setOdom(nav_msgs::Odometry &odom,
//                const ros::Time last_time,
//                const ros::Time cur_time,
//                const int d_l_counts, 
//                const int d_r_counts);
//   void setPublishers(ros::Publisher &odom_pub);
//   double getTfOdomPeriod();
//   bool use_profile;
//   double getLimit(double input ,double limit);
// };

class RoverDriver
{
private:
  rclcpp::Node::SharedPtr node_;

  enum CanAddr {
    BROADCASTER = 0x00,
    LEFT_WHEEL = 0x01,
    RIGHT_WHEEL = 0x02
  };

  enum FuncCode {
    P2P_WRITE_REQ = 0x1A,
    P2P_WRITE_RES = 0x1B,
    P2P_WRITE_ERR = 0x1C,
    P2P_READ_REQ = 0x2A,
    P2P_READ_RES = 0x2B,
    P2P_READ_ERR = 0x2C,
    P2M_WRITE_RES = 0x8A,
    P2M_WRITE_REQ = 0x8B,
    P2M_WRITE_ERR = 0x8C,
    P2M_BROCAST = 0x9A,
    P2M_BROCAST_ERR = 0x9C
  };

  enum RegAddr {
    MOTOR = 0x00,
    INPUT_MODE = 0x02,
    CAN_GP = 0x0B,
    CAN_REPORT_TIME = 0x0C,
    CAN_ID = 0x0D,
    CAN_AUTO_RP = 0x2E,
    SPEED_ACC_TIME = 0x0A,
    OUTPUT_A = 0xE2,
    OUTPUT_RPM = 0xE4,
    OUTPUT_POS_H = 0xE8,
    OUTPUT_POS_L = 0xE9,
    CLR_ERR = 0x4A,
    ERR_STATUS = 0xE3,
    SET_SPEED = 0x06,
    NONE = 0xFF
  };

  enum srvFlag {
    STOP,
    RELEASE,
    CMD_CONTROL
  };

  int left_encoder_count_ = 0;
  int right_encoder_count_ = 0;
  int s_ = -1;  // socket fd

  srvFlag srv_flag_ = STOP;
  std::mutex srv_flag_mutex_;

  double cmd_v_ = 0.0;
  double cmd_w_ = 0.0;
  std::mutex cmd_mutex_;

  // Parameters
  double wheel_base_ = 0.52;
  double wheel_r_d_ = 0.27;
  double wheel_l_d_ = 0.27;
  double reduction_ratio_ = 10.0;
  int encoder_res_ = 10000;

  bool pub_tf_ = false;
  bool pub_odom_ = false;
  double pub_rate_ = 10.0;

  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";

  double acc_max_, acc_max_default_;
  double acc_step_, acc_step_default_;
  double dec_max_, dec_max_default_;
  double dec_step_, dec_step_default_;
  double vel_tolerance_, vel_tolerance_default_;
  double profile_omega_max_, profile_omega_max_default_;
  double profile_omega_step_, profile_omega_step_default_;
  double omega_tolerance_, omega_tolerance_default_;
  double max_speed_, max_speed_default_;

  bool use_profile_ = true;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
public:
  explicit RoverDriver(const YAML::Node &config, rclcpp::Node::SharedPtr nh);
  ~RoverDriver();

  bool sendFrame(CanAddr device, unsigned char *frame);
  bool sendFrame(CanAddr device, FuncCode func_code,
                 RegAddr reg1,  int value1,
                 RegAddr reg2 = NONE, int value2 = 0);

  int fromRPM(double rpm);
  void getWheelsRpm(const double linear_vel,
                    const double angular_vel,
                    double &rpm_l, double &rpm_r);

  void closeSocket();

  // ROS2 Service callbacks
  void driveStatusSrv(const std::shared_ptr<campusrover_msgs::srv::DriveStatus::Request> request,
                      std::shared_ptr<campusrover_msgs::srv::DriveStatus::Response> response);

  void driveEncoderSrv(const std::shared_ptr<campusrover_msgs::srv::EncoderCount::Request> request,
                       std::shared_ptr<campusrover_msgs::srv::EncoderCount::Response> response);

  void driveProfileSrv(const std::shared_ptr<campusrover_msgs::srv::DriveProfile::Request> request,
                       std::shared_ptr<campusrover_msgs::srv::DriveProfile::Response> response);
  

  double getProfileSpeed(double t, double speed_tar, bool reset);
  double getProfileOmega(double t, double angle_vel_tar);

  // ROS2 Subscription callback
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist);

  // Timer callbacks (no arguments in ROS2)
  void pubTimerCallback();
  void readEncoderTimerCallback();

  void readMotorsInfo();

  int getCounts(const unsigned char HH, const unsigned char HL,
                const unsigned char LH, const unsigned char LL);

  bool isWheelsSync(const rclcpp::Time &cur_time);

  void odomPublish(CanAddr device, const int count);

  int getCountsDiff(const int count_a, const int count_b);

  void setOdom(nav_msgs::msg::Odometry &odom,
               const rclcpp::Time &last_time,
               const rclcpp::Time &cur_time,
               const int d_l_counts,
               const int d_r_counts);

  double getTfOdomPeriod();

  double getLimit(double input, double limit);


};

#endif