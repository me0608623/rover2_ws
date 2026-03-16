#include <rclcpp/rclcpp.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <string>

class ObstacleFilterNode : public rclcpp::Node
{
public:
  ObstacleFilterNode()
  : rclcpp::Node("obstacle_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/sim_odom", rclcpp::QoS(10),
      std::bind(&ObstacleFilterNode::odomCallback, this, std::placeholders::_1));
    obstacle_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "/split_obstacles_map", rclcpp::QoS(10),
      std::bind(&ObstacleFilterNode::obstacleCallback, this, std::placeholders::_1));

    pub_robot_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/filtered_obstacles_robot", 10);
    pub_odom_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/filtered_obstacles_odom", 10);
    pub_world_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/hallway_world_obs", 10);

    pub_markers_robot_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/filtered_obstacles_robot_markers", 10);
    pub_markers_odom_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/filtered_obstacles_odom_markers", 10);
    pub_markers_world_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hallway_world_obs_markers", 10);
  }

private:
  static constexpr double R_DETECT = 4.0;
  static constexpr double CAR_RADIUS = 0.35;
  static constexpr double CAR_RADIUS_OFFSET = 0.3;
  static constexpr double R_STATIC_KEEP_FORWARD = 4.0;
  static constexpr double R_STATIC_KEEP_BACKWARD = 1.0;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    is_reversing_ = (msg->twist.twist.linear.x < -0.125);
  }

  void obstacleCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr in)
  {
    geometry_msgs::msg::TransformStamped T_bl;
    try {
      T_bl = tf_buffer_.lookupTransform("turtle1", in->header.frame_id,
                                        tf2::TimePointZero, tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF to base_footprint failed: %s", ex.what());
      return;
    }

    costmap_converter_msgs::msg::ObstacleArrayMsg out_bl;
    out_bl.header.stamp = this->now();
    out_bl.header.frame_id = "turtle1";
    visualization_msgs::msg::MarkerArray markers_bl;
    int id_bl = 0;

    for (const auto &obs : in->obstacles) {
      if (obs.polygon.points.empty()) {
        continue;
      }

      geometry_msgs::msg::PointStamped pm, pb;
      pm.header.frame_id = in->header.frame_id;
      pm.header.stamp = this->now();
      pm.point.x = obs.polygon.points[0].x;
      pm.point.y = obs.polygon.points[0].y;
      pm.point.z = obs.polygon.points[0].z;
      tf2::doTransform(pm, pb, T_bl);
      const double x = pb.point.x;
      const double y = pb.point.y;
      const double r_center = std::hypot(x, y);
      const double sumR = CAR_RADIUS + obs.radius;
      const double r_edge = std::max(0.0, r_center - sumR);

      if (r_edge > R_DETECT) {
        continue;
      }

      geometry_msgs::msg::Vector3Stamped vm, vb;
      vm.header.frame_id = in->header.frame_id;
      vm.header.stamp = this->now();
      vm.vector = obs.velocities.twist.linear;
      tf2::doTransform(vm, vb, T_bl);
      const double vx = vb.vector.x;
      const double vy = vb.vector.y;

      const double speed = std::hypot(vx, vy);

      costmap_converter_msgs::msg::ObstacleMsg obs_bl = obs;
      obs_bl.header = out_bl.header;
      obs_bl.polygon.points[0].x = x;
      obs_bl.polygon.points[0].y = y;
      obs_bl.polygon.points[0].z = pb.point.z;

      if (speed < 0.2) {
        if (is_reversing_) {
          if (x < -R_STATIC_KEEP_BACKWARD) {
            continue;
          }
        } else {
          if (x > R_STATIC_KEEP_FORWARD) {
            continue;
          }
        }
      }

      out_bl.obstacles.push_back(obs_bl);

      visualization_msgs::msg::Marker m;
      m.header = out_bl.header;
      m.ns = "filtered_turtle1";
      m.id = id_bl++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = pb.point.z;
      m.pose.orientation.w = 1.0;
      const double scale = std::max(0.1, obs.radius * 2.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      m.lifetime = rclcpp::Duration::from_seconds(0.1);
      markers_bl.markers.push_back(m);
    }

    pub_robot_->publish(out_bl);
    pub_markers_robot_->publish(markers_bl);

    publishTransformed(out_bl, "odom", pub_odom_, pub_markers_odom_);
    publishTransformed(out_bl, "map", pub_world_, pub_markers_world_);
  }

  void publishTransformed(
    const costmap_converter_msgs::msg::ObstacleArrayMsg &base_obstacles,
    const std::string &target_frame,
    const rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr &pub_obs,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub_markers)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(target_frame, "turtle1",
                                             tf2::TimePointZero, tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF base_footprint->%s failed: %s", target_frame.c_str(), ex.what());
      return;
    }

    costmap_converter_msgs::msg::ObstacleArrayMsg out;
    out.header.stamp = base_obstacles.header.stamp;
    out.header.frame_id = target_frame;

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    for (const auto &base_obs : base_obstacles.obstacles) {
      geometry_msgs::msg::PointStamped bl_pt, tgt_pt;
      bl_pt.header = base_obstacles.header;
      bl_pt.point.x = base_obs.polygon.points[0].x;
      bl_pt.point.y = base_obs.polygon.points[0].y;
      bl_pt.point.z = base_obs.polygon.points[0].z;
      tf2::doTransform(bl_pt, tgt_pt, transform);

      costmap_converter_msgs::msg::ObstacleMsg obs_tgt = base_obs;
      obs_tgt.header.frame_id = target_frame;
      obs_tgt.header.stamp = base_obstacles.header.stamp;
      obs_tgt.polygon.points[0].x = tgt_pt.point.x;
      obs_tgt.polygon.points[0].y = tgt_pt.point.y;
      obs_tgt.polygon.points[0].z = tgt_pt.point.z;
      out.obstacles.push_back(obs_tgt);

      visualization_msgs::msg::Marker m;
      m.header = out.header;
      m.ns = target_frame == "odom" ? "filtered_odom" : "filtered_world";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = tgt_pt.point;
      m.pose.orientation.w = 1.0;
      const double scale = std::max(0.1, base_obs.radius * 2.5);
      m.scale.x = scale;
      m.scale.y = scale;
      m.scale.z = scale;
      if (target_frame == "odom") {
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
      } else {
        m.color.r = 0.0f;
        m.color.g = 0.0f;
        m.color.b = 1.0f;
      }
      m.color.a = 1.0f;
      m.lifetime = rclcpp::Duration::from_seconds(0.1);
      markers.markers.push_back(m);
    }

    pub_obs->publish(out);
    pub_markers->publish(markers);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool is_reversing_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_sub_;

  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_robot_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_odom_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_world_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_robot_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_odom_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_world_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
