#include <rclcpp/rclcpp.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/path.hpp>

#include <string>

#include <cmath>
#include <chrono>

class WallAndDynamicObstacleMapNode : public rclcpp::Node
{
public:
  WallAndDynamicObstacleMapNode()
      : rclcpp::Node("wall_and_dynamic_obstacle_map")
  {
    wall_and_dynamic_obs_map_pub_ =
        this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("wall_and_dynamic_obs_map", 10);
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_and_dynamic_obstacle_markers", 10);

    wall_in_range_map_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "/wall_obstacle_in_range", rclcpp::QoS(10),
        std::bind(&WallAndDynamicObstacleMapNode::wallInRangeMapCallback, this, std::placeholders::_1));
    mot_in_range_map_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "/mot_obstacle", rclcpp::QoS(10),
        std::bind(&WallAndDynamicObstacleMapNode::motInRangeMapCallback, this, std::placeholders::_1));

    elevator_strategy_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/elevator_strategy", rclcpp::QoS(10),
        std::bind(&WallAndDynamicObstacleMapNode::elevatorStrategyCallback, this, std::placeholders::_1));
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path", rclcpp::QoS(10),
        std::bind(&WallAndDynamicObstacleMapNode::globalPathCallback, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    timer_in_range_map_ = this->create_wall_timer(
        100ms, std::bind(&WallAndDynamicObstacleMapNode::inRangeMapTimerCallback, this));
  }

private:
  void elevatorStrategyCallback(const std_msgs::msg::Int16::SharedPtr mode)
  {
    elevator_strategy_mode_ = mode->data;
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty())
    {
      e1_center_in_flag_ = false;
      return;
    }

    const auto &pose = msg->poses.back().pose;
    const bool near_a =
        std::fabs(pose.position.x - 103.1326) < 0.001 &&
        std::fabs(pose.position.y + 0.57752) < 0.001;
    const bool near_b =
        std::fabs(pose.position.x - 304.7249) < 0.001 &&
        std::fabs(pose.position.y - 2.9157) < 0.001;

    e1_center_in_flag_ = (elevator_strategy_mode_ == 0) && (near_a || near_b);
  }

  void wallInRangeMapCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    latest_wall_in_range_map_msg_ = *msg;
    flag_wall_in_range_map_ = true;
  }

  void motInRangeMapCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    latest_mot_in_range_map_msg_ = *msg;
    flag_mot_in_range_map_ = true;
  }

  void publishMarkersForObstacles(const costmap_converter_msgs::msg::ObstacleArrayMsg &obs_msg,
                                  const std::string &ns,
                                  float red, float green, float blue)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &obs : obs_msg.obstacles)
    {
      visualization_msgs::msg::Marker marker;
      marker.header = obs_msg.header;
      marker.ns = ns;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      if (!obs.polygon.points.empty())
      {
        marker.pose.position.x = obs.polygon.points[0].x;
        marker.pose.position.y = obs.polygon.points[0].y;
        marker.pose.position.z = obs.polygon.points[0].z;
      }
      else
      {
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
      }
      marker.pose.orientation.w = 1.0;
      const double scale = obs.radius > 0.0 ? obs.radius * 2.0 : 0.5;
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      marker.color.a = 0.8f;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      // Replace line 109
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 200000000; // 0.2 seconds
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }

  void inRangeMapTimerCallback()
  {
    if (!(elevator_strategy_mode_ == 0 && !e1_center_in_flag_))
    {
      return;
    }

    if (!flag_wall_in_range_map_ && !flag_mot_in_range_map_)
    {
      return;
    }

    costmap_converter_msgs::msg::ObstacleArrayMsg merged;
    merged.header.frame_id = latest_mot_in_range_map_msg_.header.frame_id;
    merged.header.stamp = this->now();

    if (flag_wall_in_range_map_)
    {
      merged.obstacles.insert(merged.obstacles.end(),
                              latest_wall_in_range_map_msg_.obstacles.begin(),
                              latest_wall_in_range_map_msg_.obstacles.end());
    }
    if (flag_mot_in_range_map_)
    {
      merged.obstacles.insert(merged.obstacles.end(),
                              latest_mot_in_range_map_msg_.obstacles.begin(),
                              latest_mot_in_range_map_msg_.obstacles.end());
    }

    wall_and_dynamic_obs_map_pub_->publish(merged);
    if (!merged.obstacles.empty())
    {
      publishMarkersForObstacles(merged, "map_obs", 1.0f, 0.0f, 0.0f);
    }
  }

  int elevator_strategy_mode_{0};
  bool e1_center_in_flag_{false};

  costmap_converter_msgs::msg::ObstacleArrayMsg latest_wall_in_range_map_msg_;
  bool flag_wall_in_range_map_{false};
  costmap_converter_msgs::msg::ObstacleArrayMsg latest_mot_in_range_map_msg_;
  bool flag_mot_in_range_map_{false};

  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr wall_and_dynamic_obs_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr wall_in_range_map_sub_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr mot_in_range_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr elevator_strategy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_in_range_map_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallAndDynamicObstacleMapNode>());
  rclcpp::shutdown();
  return 0;
}
