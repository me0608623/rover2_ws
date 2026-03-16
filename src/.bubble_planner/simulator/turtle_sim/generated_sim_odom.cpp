#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

class GeneratedSimOdomNode : public rclcpp::Node
{
public:
  GeneratedSimOdomNode()
  : rclcpp::Node("odom_generator"),
    first_pose_received_(false)
  {
    const std::string pose_topic = this->declare_parameter<std::string>("pose_topic", "/turtle1/pose_stamped");
    const std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom");
    const std::string frame_id = this->declare_parameter<std::string>("frame_id", "odom");
    const std::string child_frame_id = this->declare_parameter<std::string>("child_frame_id", "turtle1");

    odom_frame_id_ = frame_id;
    child_frame_id_ = child_frame_id;

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic,
      rclcpp::QoS(10),
      std::bind(&GeneratedSimOdomNode::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const rclcpp::Time now = this->now();

    if (!first_pose_received_) {
      last_pose_ = *msg;
      last_time_ = now;
      first_pose_received_ = true;
      return;
    }

    const double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    const double dx = msg->pose.position.x - last_pose_.pose.position.x;
    const double dy = msg->pose.position.y - last_pose_.pose.position.y;
    const double distance = std::hypot(dx, dy);

    const double yaw_current = tf2::getYaw(msg->pose.orientation);
    double yaw_last = tf2::getYaw(last_pose_.pose.orientation);
    double delta_yaw = yaw_current - yaw_last;

    // wrap yaw to [-pi, pi]
    if (delta_yaw > kPi) {
      delta_yaw -= kTwoPi;
    } else if (delta_yaw < -kPi) {
      delta_yaw += kTwoPi;
    }

    const double linear_velocity = distance / dt;
    const double angular_velocity = delta_yaw / dt;

    last_pose_ = *msg;
    last_time_ = now;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose = msg->pose;
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;

    odom_pub_->publish(odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_time_;
  bool first_pose_received_;
  std::string odom_frame_id_;
  std::string child_frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeneratedSimOdomNode>());
  rclcpp::shutdown();
  return 0;
}
