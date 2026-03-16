
#include <chrono>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "campusrover_msgs/srv/drive_status.hpp"
#include "campusrover_msgs/srv/drive_profile.hpp"

using namespace std::chrono_literals;

class LcrCmdVelMux : public rclcpp::Node
{
public:
  LcrCmdVelMux() : Node("lcr_cmd_vel_mux")
  {
    joy_mode_button_ = this->declare_parameter<int>("joy_mode_button", 7);
    nav_mode_button_ = this->declare_parameter<int>("nav_mode_button", 0);
    stop_mode_button_ = this->declare_parameter<int>("stop_mode_button", 1);
    release_mode_button_ = this->declare_parameter<int>("release_mode_button", 3);

    mode_ = 0;

    drive_status_client_ = this->create_client<campusrover_msgs::srv::DriveStatus>("drive_status");
    drive_profile_client_ = this->create_client<campusrover_msgs::srv::DriveProfile>("drive_profile");

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&LcrCmdVelMux::joy_callback, this, std::placeholders::_1));
    joy_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "input/joy_cmd_vel", 10, std::bind(&LcrCmdVelMux::joy_twist_callback, this, std::placeholders::_1));
    nav_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "input/nav_cmd_vel", 10, std::bind(&LcrCmdVelMux::nav_twist_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", 10);
    
    timer_ = this->create_wall_timer(50ms, std::bind(&LcrCmdVelMux::timer_callback, this));

    nav_twist_stamp_ = this->now();
    joy_twist_stamp_ = this->now();

  }

private:
  void pub_stop()
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_msg);
  }

  void timer_callback()
  {
    if (mode_ == 1)
    {
      pub_stop();
    }
    if (mode_ == 2)
    {
      double dt = (this->now() - joy_twist_stamp_).seconds();
      if (dt > 0.3)
      {
        pub_stop();
      }
    }
    if (mode_ == 3)
    {
      double dt = (this->now() - nav_twist_stamp_).seconds();
      if (dt > 0.3)
      {
        pub_stop();
      }
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    static int last_mode_ = -1;
    auto srv = std::make_shared<campusrover_msgs::srv::DriveStatus::Request>();
    auto srvP = std::make_shared<campusrover_msgs::srv::DriveProfile::Request>();

    if (joy->buttons[stop_mode_button_] == 1)
    {
      mode_ = 1;
      srv->drive_status = campusrover_msgs::srv::DriveStatus::Request::DRIVE_STATUS_STOP;
      srvP->drive_profile = campusrover_msgs::srv::DriveProfile::Request::DRIVE_PROFILE_DEFAULT;
    }
    else if (joy->buttons[release_mode_button_] == 1)
    {
      mode_ = 0;
      srv->drive_status = campusrover_msgs::srv::DriveStatus::Request::DRIVE_STATUS_RELEASE;
    }
    else if (joy->buttons[joy_mode_button_] == 1)
    {
      mode_ = 2;
      srv->drive_status = campusrover_msgs::srv::DriveStatus::Request::DRIVE_STATUS_CMD_CONTROL;
      srvP->drive_profile = campusrover_msgs::srv::DriveProfile::Request::DRIVE_PROFILE_DEFAULT;
    }
    else if (joy->buttons[nav_mode_button_] == 1)
    {
      mode_ = 3;
      pub_stop();
      srv->drive_status = campusrover_msgs::srv::DriveStatus::Request::DRIVE_STATUS_CMD_CONTROL;
      srvP->drive_profile = campusrover_msgs::srv::DriveProfile::Request::DRIVE_PROFILE_DEFAULT;
    }
    if (last_mode_ != mode_)
    {
      drive_status_client_->async_send_request(
        srv,
        [this](rclcpp::Client<campusrover_msgs::srv::DriveStatus>::SharedFuture future) {
          try {
            auto resp = future.get();
          } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "drive_status call failed: %s", e.what());
          }
        });

      drive_profile_client_->async_send_request(
        srvP,
        [this](rclcpp::Client<campusrover_msgs::srv::DriveProfile>::SharedFuture future) {
          try {
            auto resp2 = future.get();
          } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "drive_profile call failed: %s", e.what());
          }
        });

      last_mode_ = mode_;
    }

    //   // 等待 service up
    //   if (!drive_status_client_->wait_for_service(std::chrono::milliseconds(500))) {
    //     RCLCPP_WARN(this->get_logger(), "drive_status service not available");
    //   } else {
    //     auto future1 = drive_status_client_->async_send_request(srv);
    //     // 等待最多 1s
    //     if (future1.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    //       auto resp = future1.get();
    //       // optional: 檢查 resp
    //     } else {
    //       RCLCPP_WARN(this->get_logger(), "drive_status call timed out");
    //     }
    //   }

    //   if (!drive_profile_client_->wait_for_service(std::chrono::milliseconds(500))) {
    //     RCLCPP_WARN(this->get_logger(), "drive_profile service not available");
    //   } else {
    //     auto future2 = drive_profile_client_->async_send_request(srvP);
    //     if (future2.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    //       auto resp2 = future2.get();
    //     } else {
    //       RCLCPP_WARN(this->get_logger(), "drive_profile call timed out");
    //     }
    //   }
    //   last_mode_ = mode_;
    // }
  }

  void joy_twist_callback(const geometry_msgs::msg::Twist::SharedPtr joy_twist)
  {
    if (mode_ != 2)
      return;
    cmd_vel_pub_->publish(*joy_twist);
    joy_twist_stamp_ = this->now();
  }

  void nav_twist_callback(const geometry_msgs::msg::Twist::SharedPtr nav_twist)
  {
    if (mode_ != 3)
      return;
    cmd_vel_pub_->publish(*nav_twist);
    nav_twist_stamp_ = this->now();
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<campusrover_msgs::srv::DriveStatus>::SharedPtr drive_status_client_;
  rclcpp::Client<campusrover_msgs::srv::DriveProfile>::SharedPtr drive_profile_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time nav_twist_stamp_, joy_twist_stamp_;
  int joy_mode_button_, nav_mode_button_, stop_mode_button_, release_mode_button_;
  int mode_; //{release = 0, stop = 1, manual = 2, auto_mode = 3 };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LcrCmdVelMux>();
  rclcpp::spin(node); 
  rclcpp::shutdown();
  return 0;
}