#include <angles/angles.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode()
  : rclcpp::Node("pure_pursuit_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 0.6);
    linear_vel_ = this->declare_parameter<double>("linear_vel", 0.3);
    max_angular_ = this->declare_parameter<double>("max_angular", 1.5);
    goal_th_ = this->declare_parameter<double>("goal_threshold", 0.15);
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/rebuild_path");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "turtle1");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_, qos,
      std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", rclcpp::QoS(1));

    using namespace std::chrono_literals;
    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&PurePursuitNode::controlLoop, this));
  }

private:
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "PurePursuit: received empty path!");
      return;
    }

    path_ = *msg;
    prune_idx_ = 0U;
  }

  void controlLoop()
  {
    if (path_.poses.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;
    const double yaw = tf2::getYaw(transform.transform.rotation);

    size_t nearest = prune_idx_;
    double min_d2 = std::numeric_limits<double>::max();
    for (size_t i = prune_idx_; i < path_.poses.size(); ++i) {
      const double dx = path_.poses[i].pose.position.x - x;
      const double dy = path_.poses[i].pose.position.y - y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < min_d2) {
        min_d2 = d2;
        nearest = i;
      } else if (d2 > min_d2 && i > nearest + 10) {
        break;
      }
    }
    prune_idx_ = nearest;
    const double dist_to_goal = std::sqrt(min_d2);

    geometry_msgs::msg::Twist cmd;

    if (nearest >= path_.poses.size() - 1 && dist_to_goal < goal_th_) {
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Goal reached.");
      return;
    }

    size_t target = nearest;
    double accum_dist = 0.0;
    for (size_t i = nearest; i + 1 < path_.poses.size(); ++i) {
      const double dx =
        path_.poses[i + 1].pose.position.x - path_.poses[i].pose.position.x;
      const double dy =
        path_.poses[i + 1].pose.position.y - path_.poses[i].pose.position.y;
      accum_dist += std::hypot(dx, dy);
      if (accum_dist >= lookahead_dist_) {
        target = i + 1;
        break;
      }
    }

    const auto & target_pos = path_.poses[target].pose.position;
    const double dx = target_pos.x - x;
    const double dy = target_pos.y - y;
    const double alpha = angles::normalize_angle(std::atan2(dy, dx) - yaw);
    const double kappa = 2.0 * std::sin(alpha) / lookahead_dist_;

    double v = linear_vel_;
    if (nearest + 5 >= path_.poses.size()) {
      v *= 0.5;
    }

    double w = v * kappa;
    w = std::clamp(w, -max_angular_, max_angular_);

    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  nav_msgs::msg::Path path_;
  size_t prune_idx_{0U};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double lookahead_dist_{0.6};
  double linear_vel_{0.3};
  double max_angular_{1.5};
  double goal_th_{0.15};
  std::string path_topic_;
  std::string base_frame_;
  std::string map_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
