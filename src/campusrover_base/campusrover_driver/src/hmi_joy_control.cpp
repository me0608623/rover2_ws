#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <campusrover_msgs/msg/hmi_status.hpp>
#include <campusrover_msgs/srv/arm_move_pose.hpp>
#include <campusrover_msgs/srv/arm_table_home_return.hpp>
#include <campusrover_msgs/srv/arm_table_position.hpp>

using namespace std::chrono_literals;

#define JOY_BTN_A 0
#define JOY_BTN_B 1
#define JOY_BTN_X 2
#define JOY_BTN_Y 3
#define JOY_BTN_LB 4
#define JOY_BTN_RB 5
#define JOY_BTN_BACK 6
#define JOY_BTN_START 7
#define JOY_BTN_POWER 8
#define JOY_BTN_BSL 9 // Button stick left
#define JOY_BTN_BSR 10 // Button stick right
#define JOY_AXIS_SLLR 0 // Stick left - left(-1) and right(1)
#define JOY_AXIS_SLTD 1 // Stick left - top(-1) and down(1)
#define JOY_AXIS_LT 2
#define JOY_AXIS_SRLR 3 // Stick right - left(-1) and right(1)
#define JOY_AXIS_SRTD 4 // Stick right - top(-1) and down(1)
#define JOY_AXIS_RT 5
#define JOY_AXIS_CROSS_LR 6 // cross key left(1)/right(-1)
#define JOY_AXIS_CROSS_TD 7 // cross key up(1)/down(-1)

class HmiJoyControl : public rclcpp::Node
{
public:
  HmiJoyControl() : Node("hmi_joy_control")
  {
    gesture_pub_ = this->create_publisher<std_msgs::msg::String>("gesture", 10);
    voice_pub_ = this->create_publisher<campusrover_msgs::msg::HmiStatus>("hmi_status", 10);

    arm_srv_ = this->create_client<campusrover_msgs::srv::ArmMovePose>("arm_move_to_pose");
    table_zero_srv_ = this->create_client<campusrover_msgs::srv::ArmTableHomeReturn>("zero_return");
    table_position_srv_ = this->create_client<campusrover_msgs::srv::ArmTablePosition>("table_position");

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&HmiJoyControl::JoyCallback, this, std::placeholders::_1));

    // Zero return at startup
    auto home_return_msg = std::make_shared<campusrover_msgs::srv::ArmTableHomeReturn::Request>();
    home_return_msg->flag = true;
    RCLCPP_INFO(this->get_logger(), "[HIH-JOY] checking zero service...");
    if (table_zero_srv_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_INFO(this->get_logger(), "[HIH-JOY] zero return calling");
      auto result = table_zero_srv_->async_send_request(home_return_msg);
      // Wait for result
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[HIH-JOY] zero return done");
      }
    }
  }

private:
  bool checkPressed(const sensor_msgs::msg::Joy &joy)
  {
    for (const auto btn: joy.buttons)
      if (btn) return true;
    for (const auto axis: joy.axes)
      if (axis != 0) return true;
    return false;
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    static std_msgs::msg::String gesture_status;
    static campusrover_msgs::msg::HmiStatus voice_status;
    auto arm_pose = std::make_shared<campusrover_msgs::srv::ArmMovePose::Request>();
    auto table_position_msg = std::make_shared<campusrover_msgs::srv::ArmTablePosition::Request>();

    if (!checkPressed(*joy)) { return; }

    if (!joy->buttons[JOY_BTN_LB])
    {
      if (joy->axes[JOY_AXIS_CROSS_TD] > 0)
      {
        gesture_status.data = "one";
        gesture_pub_->publish(gesture_status);
      }
      if (joy->axes[JOY_AXIS_CROSS_TD] < 0)
      {
        gesture_status.data = "ok";
        gesture_pub_->publish(gesture_status);
      }
      if (joy->axes[JOY_AXIS_CROSS_LR] < 0)
      {
        gesture_status.data = "two";
        gesture_pub_->publish(gesture_status);
      }
      if (joy->axes[JOY_AXIS_CROSS_LR] > 0)
      {
        gesture_status.data = "ok";
        gesture_pub_->publish(gesture_status);
      }
      if (joy->buttons[JOY_BTN_BACK] > 0)
      {
        table_position_msg->position.y = 0.2;
        table_position_msg->position.z = 0.6;
        if (table_position_srv_->wait_for_service(1s))
          table_position_srv_->async_send_request(table_position_msg);

        arm_pose->pose_name = "standby_pose_1";
        if (arm_srv_->wait_for_service(1s))
          arm_srv_->async_send_request(arm_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        arm_pose->pose_name = "standby_pose";
        if (arm_srv_->wait_for_service(1s))
          arm_srv_->async_send_request(arm_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
    else
    {
      if (joy->axes[JOY_AXIS_CROSS_TD] > 0)
      {
        voice_status.staus = 203;
        voice_pub_->publish(voice_status);
      }
      if (joy->axes[JOY_AXIS_CROSS_TD] < 0)
      {
        voice_status.staus = 207;
        voice_pub_->publish(voice_status);
      }
      if (joy->axes[JOY_AXIS_CROSS_LR] < 0)
      {
        voice_status.staus = 204;
        voice_pub_->publish(voice_status);
      }
      if (joy->axes[JOY_AXIS_CROSS_LR] > 0)
      {
        voice_status.staus = 205;
        voice_pub_->publish(voice_status);
      }
      if (joy->buttons[JOY_BTN_RB])
      {
        voice_status.staus = 206;
        voice_pub_->publish(voice_status);
      }
      if (joy->buttons[JOY_BTN_BACK] > 0)
      {
        table_position_msg->position.y = 0.2;
        table_position_msg->position.z = 0.1;
        if (table_position_srv_->wait_for_service(1s))
          table_position_srv_->async_send_request(table_position_msg);

        arm_pose->pose_name = "standby_pose_1";
        if (arm_srv_->wait_for_service(1s))
          arm_srv_->async_send_request(arm_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        arm_pose->pose_name = "laydown_pose";
        if (arm_srv_->wait_for_service(1s))
          arm_srv_->async_send_request(arm_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_pub_;
  rclcpp::Publisher<campusrover_msgs::msg::HmiStatus>::SharedPtr voice_pub_;
  rclcpp::Client<campusrover_msgs::srv::ArmMovePose>::SharedPtr arm_srv_;
  rclcpp::Client<campusrover_msgs::srv::ArmTableHomeReturn>::SharedPtr table_zero_srv_;
  rclcpp::Client<campusrover_msgs::srv::ArmTablePosition>::SharedPtr table_position_srv_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HmiJoyControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


