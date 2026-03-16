#include <rclcpp/rclcpp.hpp>
#include <campusrover_msgs/msg/tracked_obstacle_array.hpp>
#include <campusrover_msgs/msg/tracked_obstacle.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <cmath>

class MotToObstacleNode : public rclcpp::Node
{
public:
  MotToObstacleNode()
  : rclcpp::Node("mot_to_obstacle")
  {
    this->declare_parameter<std::string>("map_frame_id", "map");

    obstacle_pub_in_range_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "mot_obstacle", rclcpp::QoS(10).transient_local());
    marker_pub_in_range_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "mot_obstacle_visualizes_in_range", rclcpp::QoS(10).transient_local());

    obstacle_sub_ = this->create_subscription<campusrover_msgs::msg::TrackedObstacleArray>(
      "/tracked_label_obstacle", rclcpp::QoS(10),
      std::bind(&MotToObstacleNode::trackedObstacleCallback, this, std::placeholders::_1));
    elevator_strategy_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/elevator_strategy", rclcpp::QoS(10),
      std::bind(&MotToObstacleNode::elevatorStrategyCallback, this, std::placeholders::_1));
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(10),
      std::bind(&MotToObstacleNode::globalPathCallback, this, std::placeholders::_1));
  }

private:
  void deleteAllMarker()
  {
    visualization_msgs::msg::MarkerArray all_marker;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    all_marker.markers.push_back(marker);
    marker_pub_in_range_->publish(all_marker);
  }

  void elevatorStrategyCallback(const std_msgs::msg::Int16::SharedPtr mode)
  {
    elevator_strategy_mode_ = mode->data;
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      e1_center_in_flag_ = false;
      return;
    }

    const auto &last_pose = msg->poses.back().pose;
    const bool near_first =
      std::abs(last_pose.position.x - 103.1326) < 0.001 &&
      std::abs(last_pose.position.y + 0.57752) < 0.001;
    const bool near_second =
      std::abs(last_pose.position.x - 304.7249) < 0.001 &&
      std::abs(last_pose.position.y - 2.9157) < 0.001;

    e1_center_in_flag_ = (elevator_strategy_mode_ == 0) && (near_first || near_second);
  }

  void trackedObstacleCallback(const campusrover_msgs::msg::TrackedObstacleArray::SharedPtr msg)
  {
    if (!(elevator_strategy_mode_ == 0 && !e1_center_in_flag_)) {
      return;
    }

    deleteAllMarker();

    costmap_converter_msgs::msg::ObstacleArrayMsg obstacle_array;
    obstacle_array.header = msg->header;

    visualization_msgs::msg::MarkerArray markers_in_range;

    for (const auto &tracked_obs : msg->obstacles) {
      double dim = tracked_obs.dimensions.x;
      if (dim >= 1.5) {
        continue;
      }

      double radius = dim;
      if (dim >= 1.0 && dim < 1.5) {
        radius = 0.8 * dim;
      }

      costmap_converter_msgs::msg::ObstacleMsg obs;
      geometry_msgs::msg::Point32 p;
      p.x = tracked_obs.pose.position.x;
      p.y = tracked_obs.pose.position.y;
      p.z = tracked_obs.pose.position.z;
      obs.polygon.points.push_back(p);
      obs.radius = radius;
      obs.id = tracked_obs.id;
      obs.velocities.twist = tracked_obs.velocity;

      obstacle_array.obstacles.push_back(obs);

      visualization_msgs::msg::Marker marker_in;
      marker_in.header = msg->header;
      marker_in.ns = "in_range_obstacles";
      marker_in.id = tracked_obs.id;
      marker_in.type = visualization_msgs::msg::Marker::SPHERE;
      marker_in.action = visualization_msgs::msg::Marker::ADD;
      marker_in.pose = tracked_obs.pose;
      marker_in.scale.x = radius * 2.0;
      marker_in.scale.y = radius * 2.0;
      marker_in.scale.z = radius * 2.0;
      marker_in.color.r = 1.0f;
      marker_in.color.g = 0.0f;
      marker_in.color.b = 0.0f;
      marker_in.color.a = 1.0f;

      markers_in_range.markers.push_back(marker_in);
    }

    obstacle_pub_in_range_->publish(obstacle_array);
    marker_pub_in_range_->publish(markers_in_range);
  }

  int elevator_strategy_mode_{0};
  bool e1_center_in_flag_{false};

  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_in_range_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_in_range_;
  rclcpp::Subscription<campusrover_msgs::msg::TrackedObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr elevator_strategy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotToObstacleNode>());
  rclcpp::shutdown();
  return 0;
}
