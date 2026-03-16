#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

#include <cmath>
#include <string>
#include <utility>

class TrajectoryRecorderNode : public rclcpp::Node
{
public:
  TrajectoryRecorderNode()
  : rclcpp::Node("trajectory_recorder"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cfg_.source = this->declare_parameter<std::string>("source", "tf");
    cfg_.odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom");
    cfg_.target_frame = this->declare_parameter<std::string>("target_frame", "world");
    cfg_.child_frame = this->declare_parameter<std::string>("child_frame", "base_footprint");
    cfg_.min_dist = this->declare_parameter<double>("min_dist", 0.03);
    cfg_.min_dyaw = this->declare_parameter<double>("min_dyaw", 0.05);
    cfg_.max_points = this->declare_parameter<int>("max_points", 5000);
    cfg_.latch_path = this->declare_parameter<bool>("latch_path", true);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (cfg_.latch_path) {
      qos.transient_local();
      qos.reliable();
    }

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory/path", qos);
    path_.header.frame_id = cfg_.target_frame;

    if (cfg_.source == "odom") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, rclcpp::QoS(50),
        std::bind(&TrajectoryRecorderNode::odomCallback, this, std::placeholders::_1));
    } else {
      using namespace std::chrono_literals;
      tf_timer_ = this->create_wall_timer(
        20ms, std::bind(&TrajectoryRecorderNode::tfTimer, this));
    }
  }

private:
  struct Config
  {
    std::string source;
    std::string odom_topic;
    std::string target_frame;
    std::string child_frame;
    double min_dist{0.03};
    double min_dyaw{0.05};
    int max_points{5000};
    bool latch_path{true};
  } cfg_;

  static double yawOf(const geometry_msgs::msg::Quaternion & q)
  {
    return tf2::getYaw(q);
  }

  static double dist2d(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b)
  {
    return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
  }

  static double normAng(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr od)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = od->header;
    pose.pose = od->pose.pose;
    addPose(pose);
  }

  void tfTimer()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        cfg_.target_frame, cfg_.child_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.02));

      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.header.frame_id = cfg_.target_frame;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      addPose(pose);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
    }
  }

  void addPose(geometry_msgs::msg::PoseStamped pose)
  {
    if (pose.header.frame_id != cfg_.target_frame) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          cfg_.target_frame, pose.header.frame_id,
          rclcpp::Time(pose.header.stamp), rclcpp::Duration::from_seconds(0.05));
        tf2::doTransform(pose, pose, transform);
      } catch (const tf2::TransformException &) {
        try {
          const auto transform = tf_buffer_.lookupTransform(
            cfg_.target_frame, pose.header.frame_id,
            tf2::TimePointZero, tf2::durationFromSec(0.05));
          tf2::doTransform(pose, pose, transform);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "TF transform failed: %s", ex.what());
          return;
        }
      }
    }

    if (!path_.poses.empty()) {
      const auto & last_pose = path_.poses.back().pose;
      const double dxy = dist2d(pose.pose, last_pose);
      const double dyaw =
        std::fabs(normAng(yawOf(pose.pose.orientation) - yawOf(last_pose.orientation)));
      if (dxy < cfg_.min_dist && dyaw < cfg_.min_dyaw) {
        return;
      }
    }

    path_.header.stamp = this->now();
    path_.poses.push_back(pose);

    if (cfg_.max_points > 0 && static_cast<int>(path_.poses.size()) > cfg_.max_points) {
      const auto to_remove = path_.poses.size() - static_cast<size_t>(cfg_.max_points);
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<std::ptrdiff_t>(to_remove));
    }

    pub_path_->publish(path_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  nav_msgs::msg::Path path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
