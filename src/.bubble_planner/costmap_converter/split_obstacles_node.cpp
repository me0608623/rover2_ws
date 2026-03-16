#include <rclcpp/rclcpp.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

class SplitObstaclesNode : public rclcpp::Node
{
public:
  SplitObstaclesNode()
  : rclcpp::Node("split_obstacles_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    car_fwd_ = this->declare_parameter<double>("car_fwd", 0.3);
    car_bwd_ = this->declare_parameter<double>("car_bwd", 0.6);
    car_l_ = this->declare_parameter<double>("car_left", 0.35);
    car_r_ = this->declare_parameter<double>("car_right", 0.35);
    max_splits_ = this->declare_parameter<int>("max_splits", 5);
    min_radius_ = this->declare_parameter<double>("min_radius", 0.05);
    sample_res_ = this->declare_parameter<double>("sample_res", 0.05);

    split_obstacles_pub_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/split_obstacles", 10);
    split_obstacles_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/split_obstacles_marker", 10);
    split_obstacles_map_pub_ = this->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("/split_obstacles_map", 10);
    split_obstacles_marker_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/split_obstacles_map_marker", 10);
    candidate_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cand_point", 10);

    obstacle_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "/wall_and_dynamic_obs_map", rclcpp::QoS(10),
      std::bind(&SplitObstaclesNode::originalObstacleCallback, this, std::placeholders::_1));
  }

private:
  bool inRect(double x, double y) const
  {
    return x >= -car_bwd_ && x <= car_fwd_ && y >= -car_r_ && y <= car_l_;
  }

  double distToRect(double x, double y) const
  {
    double dx = 0.0;
    double dy = 0.0;

    if (x < -car_bwd_) {
      dx = -car_bwd_ - x;
    } else if (x > car_fwd_) {
      dx = x - car_fwd_;
    }

    if (y < -car_r_) {
      dy = -car_r_ - y;
    } else if (y > car_l_) {
      dy = y - car_l_;
    }

    return std::hypot(dx, dy);
  }

  void originalObstacleCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr input_obs)
  {
    geometry_msgs::msg::TransformStamped T_bl;
    try {
      T_bl = tf_buffer_.lookupTransform("turtle1", input_obs->header.frame_id,
                                        tf2::TimePointZero, tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF to turtle1 failed: %s", ex.what());
      return;
    }

    costmap_converter_msgs::msg::ObstacleArrayMsg out;
    out.header.stamp = this->now();
    out.header.frame_id = "turtle1";

    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (const auto &obs : input_obs->obstacles) {
      const int id_old = obs.id;
      const double cx0_map = obs.polygon.points.empty() ? 0.0 : obs.polygon.points[0].x;
      const double cy0_map = obs.polygon.points.empty() ? 0.0 : obs.polygon.points[0].y;
      const double r0 = std::max(obs.radius, min_radius_);

      geometry_msgs::msg::PointStamped pm, pbl;
      pm.header.frame_id = input_obs->header.frame_id;
      pm.header.stamp = this->now();
      pm.point.x = cx0_map;
      pm.point.y = cy0_map;
      pm.point.z = 0.0;
      tf2::doTransform(pm, pbl, T_bl);

      const double cx0 = pbl.point.x;
      const double cy0 = pbl.point.y;

      const double cx_clamped = std::clamp(cx0, -car_bwd_, car_fwd_);
      const double cy_clamped = std::clamp(cy0, -car_r_, car_l_);
      const double dx_rect = cx0 - cx_clamped;
      const double dy_rect = cy0 - cy_clamped;
      const bool overlap = (dx_rect * dx_rect + dy_rect * dy_rect) <= (r0 * r0);

      if (!overlap) {
        costmap_converter_msgs::msg::ObstacleMsg o2;
        o2.id = id_old;
        o2.radius = r0;
        o2.velocities.twist = obs.velocities.twist;
        geometry_msgs::msg::Point32 P;
        P.x = cx0;
        P.y = cy0;
        P.z = 0.0;
        o2.polygon.points = {P};
        out.obstacles.push_back(o2);
        continue;
      }

      std::vector<std::pair<double, double>> candidates = sampleCandidates(cx0, cy0, r0);

      visualization_msgs::msg::MarkerArray candidate_markers;
      candidate_markers.markers.reserve(candidates.size());
      for (const auto &c : candidates) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "turtle1";
        m.header.stamp = out.header.stamp;
        m.ns = "candidate_centers";
        m.id = marker_id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = c.first;
        m.pose.position.y = c.second;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        m.color.r = 1.0f;
        m.color.g = 0.0f;
        m.color.b = 0.0f;
        m.color.a = 0.6f;
        m.lifetime = rclcpp::Duration::from_seconds(0.1);
        candidate_markers.markers.push_back(m);
      }
      candidate_markers_pub_->publish(candidate_markers);

      std::vector<std::pair<double, double>> sel_centers;
      std::vector<double> sel_radii;

      for (int k = 0; k < max_splits_; ++k) {
        double best_r = 0.0;
        double best_x = 0.0;
        double best_y = 0.0;

        for (const auto &c : candidates) {
          const double x = c.first;
          const double y = c.second;

          double r_circle = r0 - std::hypot(x - cx0, y - cy0);
          if (r_circle < best_r) {
            continue;
          }

          double r_allowed = std::min(r_circle, distToRect(x, y));
          if (r_allowed < best_r) {
            continue;
          }

          for (size_t i = 0; i < sel_centers.size(); ++i) {
            const double dx = x - sel_centers[i].first;
            const double dy = y - sel_centers[i].second;
            const double dci = std::hypot(dx, dy) - sel_radii[i];
            r_allowed = std::min(r_allowed, dci);
            if (r_allowed < best_r) {
              break;
            }
          }

          if (r_allowed > best_r) {
            best_r = r_allowed;
            best_x = x;
            best_y = y;
          }
        }

        if (best_r <= min_radius_) {
          break;
        }

        sel_centers.emplace_back(best_x, best_y);
        sel_radii.push_back(best_r);

        costmap_converter_msgs::msg::ObstacleMsg o2;
        const int new_id = -1 * (id_old * 10000 + static_cast<int>(sel_centers.size()));
        o2.id = new_id;
        o2.radius = best_r;
        o2.velocities.twist = obs.velocities.twist;
        geometry_msgs::msg::Point32 p;
        p.x = best_x;
        p.y = best_y;
        p.z = 0.0;
        o2.polygon.points = {p};
        out.obstacles.push_back(o2);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = "turtle1";
        m.header.stamp = out.header.stamp;
        m.ns = "split_circles";
        m.id = new_id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = best_x;
        m.pose.position.y = best_y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = best_r * 2.0;
        m.scale.y = best_r * 2.0;
        m.scale.z = best_r * 2.0;
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;
        m.lifetime = rclcpp::Duration::from_seconds(0.11);
        markers.markers.push_back(m);
      }
    }

    split_obstacles_pub_->publish(out);
    split_obstacles_marker_pub_->publish(markers);

    geometry_msgs::msg::TransformStamped T_map;
    try {
      T_map = tf_buffer_.lookupTransform(input_obs->header.frame_id, "turtle1",
                                         tf2::TimePointZero, tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF turtle1->%s failed: %s", input_obs->header.frame_id.c_str(), ex.what());
      return;
    }

    costmap_converter_msgs::msg::ObstacleArrayMsg out_map;
    out_map.header.stamp = out.header.stamp;
    out_map.header.frame_id = input_obs->header.frame_id;

    for (const auto &o2 : out.obstacles) {
      geometry_msgs::msg::PointStamped pb, pm;
      pb.header.frame_id = "turtle1";
      pb.point.x = o2.polygon.points[0].x;
      pb.point.y = o2.polygon.points[0].y;
      pb.point.z = o2.polygon.points[0].z;
      tf2::doTransform(pb, pm, T_map);

      costmap_converter_msgs::msg::ObstacleMsg o_map = o2;
      geometry_msgs::msg::Point32 p32;
      p32.x = pm.point.x;
      p32.y = pm.point.y;
      p32.z = pm.point.z;
      o_map.polygon.points = {p32};
      out_map.obstacles.push_back(o_map);
    }

    visualization_msgs::msg::MarkerArray markers_map;
    for (const auto &m : markers.markers) {
      geometry_msgs::msg::PoseStamped pbs, pms;
      pbs.header.frame_id = "turtle1";
      pbs.pose = m.pose;
      tf2::doTransform(pbs, pms, T_map);

      visualization_msgs::msg::Marker m_map = m;
      m_map.header.frame_id = input_obs->header.frame_id;
      m_map.header.stamp = out.header.stamp;
      m_map.pose = pms.pose;

      if (m_map.id < 0) {
        m_map.color.r = 0.0f;
        m_map.color.g = 1.0f;
        m_map.color.b = 0.0f;
        m_map.color.a = 1.0f;
      }

      markers_map.markers.push_back(m_map);
    }

    split_obstacles_map_pub_->publish(out_map);
    split_obstacles_marker_map_pub_->publish(markers_map);
  }

  std::vector<std::pair<double, double>> sampleCandidates(double cx, double cy, double r0) const
  {
    std::vector<std::pair<double, double>> candidates;
    const int steps = static_cast<int>(std::ceil((2 * r0) / sample_res_));
    for (int ix = -steps; ix <= steps; ++ix) {
      for (int iy = -steps; iy <= steps; ++iy) {
        const double x = cx + ix * sample_res_;
        const double y = cy + iy * sample_res_;
        if (std::hypot(x - cx, y - cy) <= r0) {
          candidates.emplace_back(x, y);
        }
      }
    }
    return candidates;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr split_obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr split_obstacles_marker_pub_;
  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr split_obstacles_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr split_obstacles_marker_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_markers_pub_;

  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_sub_;

  double car_fwd_{0.3};
  double car_bwd_{0.6};
  double car_l_{0.35};
  double car_r_{0.35};
  int max_splits_{5};
  double min_radius_{0.05};
  double sample_res_{0.05};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SplitObstaclesNode>());
  rclcpp::shutdown();
  return 0;
}
