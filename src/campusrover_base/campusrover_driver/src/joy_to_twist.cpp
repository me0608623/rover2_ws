#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyToTwistNode : public rclcpp::Node
{
public:
  JoyToTwistNode() : Node("joy_to_twist")
  {
    this->declare_parameter<double>("scale_linear", 1.0);
    this->declare_parameter<double>("scale_angular", 1.0);
    this->declare_parameter<int>("axis_linear", 0);
    this->declare_parameter<int>("axis_angular", 1);
    this->declare_parameter<double>("speed_up_vel", 0.8);

    this->get_parameter("scale_linear", scale_linear_);
    this->get_parameter("scale_angular", scale_angular_);
    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);
    this->get_parameter("speed_up_vel", speed_up_vel_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyToTwistNode::joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    cmd_vel_msg.linear.x = joy->axes[axis_linear_] * scale_linear_;
    if (cmd_vel_msg.linear.x > 0)
      cmd_vel_msg.linear.x += joy->buttons[5] * speed_up_vel_; // press RB to speed up
    else if (cmd_vel_msg.linear.x < 0)
      cmd_vel_msg.linear.x -= joy->buttons[5] * speed_up_vel_; // press RB to speed up
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.z = joy->axes[axis_angular_] * scale_angular_;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    twist_pub_->publish(cmd_vel_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  double scale_linear_, scale_angular_;
  int axis_linear_, axis_angular_;
  double speed_up_vel_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToTwistNode>());
  rclcpp::shutdown();
  return 0;
}