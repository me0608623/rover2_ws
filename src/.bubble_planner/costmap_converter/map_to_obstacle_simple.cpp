#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int16.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
  constexpr double kNearTolerance = 1e-3;
} // namespace

class MapToObstacleNode : public rclcpp::Node
{
public:
  MapToObstacleNode()
      : rclcpp::Node("map_to_obstacle"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    resample_resolution_ = this->declare_parameter<double>("resample_resolution", 0.05);
    obstacle_frame_ = this->declare_parameter<std::string>("obstacle_frame", "map");
    wall_obstacle_radius_ = this->declare_parameter<double>("wall_obstalce_radius", 0.15);
    obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 0);
    detection_radius_ = this->declare_parameter<double>("detection_radius", 4.0);
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/static_costmap");
    global_path_topic_ = this->declare_parameter<std::string>("global_path_topic", "/global_path");
    elevator_strategy_topic_ = this->declare_parameter<std::string>("elevator_strategy_topic", "/elevator_strategy");
    robot_frame_ = this->declare_parameter<std::string>("robot_frame", "turtle1");

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("resample_map", latched_qos);
    obstacle_pub_all_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "wall_obstacle_all", latched_qos);
    obstacle_pub_in_range_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "wall_obstacle_in_range", latched_qos);
    marker_pub_all_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "wall_obstacle_visualizes_all", latched_qos);
    marker_pub_in_range_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "wall_obstacle_visualizes_in_range", latched_qos);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10),
        std::bind(&MapToObstacleNode::mapCallback, this, std::placeholders::_1));
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic_, rclcpp::QoS(10),
        std::bind(&MapToObstacleNode::globalPathCallback, this, std::placeholders::_1));
    elevator_strategy_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        elevator_strategy_topic_, rclcpp::QoS(10),
        std::bind(&MapToObstacleNode::elevatorStrategyCallback, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    local_timer_ = this->create_wall_timer(
        100ms, std::bind(&MapToObstacleNode::localObstacleTimer, this));
  }

private:
  void elevatorStrategyCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    elevator_strategy_mode_ = msg->data;
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg || msg->poses.empty())
    {
      e1_center_in_flag_ = false;
      return;
    }

    const auto &last_pose = msg->poses.back().pose;
    const bool near_first =
        std::abs(last_pose.position.x - 103.1326) < kNearTolerance &&
        std::abs(last_pose.position.y - (-0.57752)) < kNearTolerance;
    const bool near_second =
        std::abs(last_pose.position.x - 304.7249) < kNearTolerance &&
        std::abs(last_pose.position.y - 2.9157) < kNearTolerance;

    e1_center_in_flag_ = (elevator_strategy_mode_ == 0) && (near_first || near_second);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    const auto resampled = resampleMap(*msg);
    latest_resample_map_ = resampled;
    have_resample_map_ = true;

    map_pub_->publish(resampled);

    deleteAllMarker(marker_pub_all_);

    costmap_converter_msgs::msg::ObstacleArrayMsg wall_obs_all;
    wall_obs_all.header.frame_id = obstacle_frame_;
    wall_obs_all.header.stamp = this->now();

    visualization_msgs::msg::MarkerArray markers_all;
    markers_all.markers.reserve(resampled.info.width * resampled.info.height);

    const auto &info = resampled.info;
    const auto total_cells = static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
    if (resampled.data.size() != total_cells || total_cells == 0)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "map_to_obstacle: resampled occupancy grid has inconsistent size");
      return;
    }

    int marker_id = 0;
    for (uint32_t iy = 0; iy < info.height; ++iy)
    {
      for (uint32_t ix = 0; ix < info.width; ++ix)
      {
        const std::size_t index = static_cast<std::size_t>(iy) * info.width + ix;
        const auto cell = resampled.data[index];
        if (cell <= obstacle_threshold_)
        {
          continue;
        }

        geometry_msgs::msg::Point32 point;
        point.x = info.origin.position.x + (static_cast<double>(ix) + 0.5) * info.resolution;
        point.y = info.origin.position.y + (static_cast<double>(iy) + 0.5) * info.resolution;
        point.z = 0.0;

        costmap_converter_msgs::msg::ObstacleMsg obs;
        obs.radius = wall_obstacle_radius_;
        obs.polygon.points.clear();
        obs.polygon.points.push_back(point);
        wall_obs_all.obstacles.push_back(obs);

        visualization_msgs::msg::Marker marker;
        marker.header = wall_obs_all.header;
        marker.ns = "wall_obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        const double diameter = std::max(0.1, wall_obstacle_radius_ * 2.0);
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = diameter;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 200000000; // 0.2 秒
        markers_all.markers.push_back(marker);
      }
    }

    obstacle_pub_all_->publish(wall_obs_all);
    marker_pub_all_->publish(markers_all);
  }

  void localObstacleTimer()
  {
    if (!(elevator_strategy_mode_ == 0 && !e1_center_in_flag_))
    {
      return;
    }
    if (!have_resample_map_ || latest_resample_map_.data.empty())
    {
      return;
    }

    updateRobotPose();
    if (!have_robot_pose_)
    {
      return;
    }

    deleteAllMarker(marker_pub_in_range_);

    costmap_converter_msgs::msg::ObstacleArrayMsg wall_obs_in_range;
    wall_obs_in_range.header.frame_id = obstacle_frame_;
    wall_obs_in_range.header.stamp = this->now();

    visualization_msgs::msg::MarkerArray markers;

    const auto &info = latest_resample_map_.info;
    const auto total_cells = static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
    if (latest_resample_map_.data.size() != total_cells || total_cells == 0)
    {
      return;
    }

    int marker_id = 0;
    for (uint32_t iy = 0; iy < info.height; ++iy)
    {
      for (uint32_t ix = 0; ix < info.width; ++ix)
      {
        const std::size_t index = static_cast<std::size_t>(iy) * info.width + ix;
        const auto cell = latest_resample_map_.data[index];
        if (cell <= obstacle_threshold_)
        {
          continue;
        }

        geometry_msgs::msg::Point32 point;
        point.x = info.origin.position.x + (static_cast<double>(ix) + 0.5) * info.resolution;
        point.y = info.origin.position.y + (static_cast<double>(iy) + 0.5) * info.resolution;
        point.z = 0.0;

        const double distance = std::hypot(
            point.x - robot_pose_.pose.position.x,
            point.y - robot_pose_.pose.position.y);
        if (distance > detection_radius_)
        {
          continue;
        }

        costmap_converter_msgs::msg::ObstacleMsg obs;
        obs.radius = wall_obstacle_radius_;
        obs.polygon.points.clear();
        obs.polygon.points.push_back(point);
        wall_obs_in_range.obstacles.push_back(obs);

        visualization_msgs::msg::Marker marker;
        marker.header = wall_obs_in_range.header;
        marker.ns = "in_range_obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        const double diameter = std::max(0.1, wall_obstacle_radius_ * 2.0);
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = diameter;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 200000000; // 0.2 秒
        markers.markers.push_back(marker);
      }
    }

    obstacle_pub_in_range_->publish(wall_obs_in_range);
    marker_pub_in_range_->publish(markers);
  }

  nav_msgs::msg::OccupancyGrid resampleMap(const nav_msgs::msg::OccupancyGrid &original) const
  {
    if (resample_resolution_ <= 0.0 ||
        std::abs(resample_resolution_ - original.info.resolution) < 1e-9)
    {
      return original;
    }

    nav_msgs::msg::OccupancyGrid resampled;
    resampled.header = original.header;
    resampled.info = original.info;

    if (original.info.width == 0 || original.info.height == 0)
    {
      return resampled;
    }

    const double ratio = original.info.resolution / resample_resolution_;
    if (ratio <= 0.0)
    {
      return original;
    }

    resampled.info.resolution = resample_resolution_;
    resampled.info.width = static_cast<uint32_t>(std::max(
        1.0, std::round(static_cast<double>(original.info.width) * ratio)));
    resampled.info.height = static_cast<uint32_t>(std::max(
        1.0, std::round(static_cast<double>(original.info.height) * ratio)));
    resampled.data.clear();
    resampled.data.reserve(
        static_cast<std::size_t>(resampled.info.width) * resampled.info.height);

    for (uint32_t iy = 0; iy < resampled.info.height; ++iy)
    {
      const uint32_t src_y = std::min(
          static_cast<uint32_t>(iy / ratio),
          original.info.height - 1);
      for (uint32_t ix = 0; ix < resampled.info.width; ++ix)
      {
        const uint32_t src_x = std::min(
            static_cast<uint32_t>(ix / ratio),
            original.info.width - 1);
        const std::size_t index = static_cast<std::size_t>(src_y) * original.info.width + src_x;
        resampled.data.push_back(original.data[index]);
      }
    }

    return resampled;
  }

  void deleteAllMarker(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub)
  {
    if (!pub)
    {
      return;
    }
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    pub->publish(markers);
  }

  void updateRobotPose()
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
          obstacle_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
      robot_pose_.header = transform.header;
      robot_pose_.pose.position.x = transform.transform.translation.x;
      robot_pose_.pose.position.y = transform.transform.translation.y;
      robot_pose_.pose.position.z = transform.transform.translation.z;
      robot_pose_.pose.orientation = transform.transform.rotation;
      have_robot_pose_ = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "map_to_obstacle: TF lookup failed (%s)", ex.what());
      have_robot_pose_ = false;
    }
  }

  // Parameters
  double resample_resolution_{0.05};
  std::string obstacle_frame_{"map"};
  double wall_obstacle_radius_{0.15};
  int obstacle_threshold_{0};
  double detection_radius_{4.0};
  std::string map_topic_{"/static_costmap"};
  std::string global_path_topic_{"/global_path"};
  std::string elevator_strategy_topic_{"/elevator_strategy"};
  std::string robot_frame_{"turtle1"};

  // Publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_all_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_in_range_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_all_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_in_range_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr elevator_strategy_sub_;
  rclcpp::TimerBase::SharedPtr local_timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // State
  nav_msgs::msg::OccupancyGrid latest_resample_map_;
  bool have_resample_map_{false};
  geometry_msgs::msg::PoseStamped robot_pose_;
  bool have_robot_pose_{false};
  int elevator_strategy_mode_{0};
  bool e1_center_in_flag_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToObstacleNode>());
  rclcpp::shutdown();
  return 0;
}
