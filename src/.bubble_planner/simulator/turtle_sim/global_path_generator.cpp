#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
}  // namespace

struct Pt { double x, y; };
struct Vec { double x, y; };

static inline Vec operator+(const Vec &a, const Vec &b) { return {a.x + b.x, a.y + b.y}; }
static inline Vec operator-(const Vec &a, const Vec &b) { return {a.x - b.x, a.y - b.y}; }
static inline Vec operator*(const Vec &a, double s) { return {a.x * s, a.y * s}; }
static inline Vec operator/(const Vec &a, double s) { return {a.x / s, a.y / s}; }
static inline double dot(const Vec &a, const Vec &b) { return a.x * b.x + a.y * b.y; }
static inline double norm(const Vec &a) { return std::hypot(a.x, a.y); }
static inline Vec normalize(const Vec &a) { double n = norm(a); return n > 1e-9 ? a / n : Vec{0.0, 0.0}; }
static inline Vec fromPt(const Pt &p) { return {p.x, p.y}; }
static inline Pt toPt(const Vec &v) { return {v.x, v.y}; }
static inline double dist(const Pt &a, const Pt &b) { return std::hypot(a.x - b.x, a.y - b.y); }

class GlobalPathGeneratorCornerSmooth : public rclcpp::Node
{
public:
  GlobalPathGeneratorCornerSmooth()
  : rclcpp::Node("global_path_generator")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    spacing_ = this->declare_parameter<double>("spacing", 0.05);
    z_ = this->declare_parameter<double>("z", 0.0);
    rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 1.0);
    corner_trim_m_ = this->declare_parameter<double>("corner_trim_m", 0.8);
    tangent_gain_ = this->declare_parameter<double>("bezier_tangent_gain", 1.0);
    corner_min_angle_deg_ = this->declare_parameter<double>("corner_min_angle_deg", 5.0);
    emit_endpoints_ = this->declare_parameter<bool>("emit_endpoints", true);

    const std::vector<Pt> defaults{
      {5.5, 5.5},
      {5.5, 4.5},
      {1.5, 4.5},
      {1.5, 9.0},
      {9.5, 9.0},
      {9.5, 3.5},
    };
    waypoints_ = loadWaypointsParam(defaults);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", qos);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_path_marker", qos);

    buildPath();
    publishOnce();

    using namespace std::chrono_literals;
    const double safe_rate = std::max(0.1, rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / safe_rate),
      std::bind(&GlobalPathGeneratorCornerSmooth::publishOnce, this));
  }

private:
  std::vector<Pt> loadWaypointsParam(const std::vector<Pt> &fallback)
  {
    std::vector<double> flattened = this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
    if (flattened.empty()) {
      return fallback;
    }
    if (flattened.size() % 2 != 0) {
      RCLCPP_WARN(this->get_logger(), "Parameter 'waypoints' must contain an even number of entries. Using fallback path.");
      return fallback;
    }
    std::vector<Pt> out;
    out.reserve(flattened.size() / 2);
    for (std::size_t i = 0; i < flattened.size(); i += 2) {
      out.push_back(Pt{flattened[i], flattened[i + 1]});
    }
    return out;
  }

  void pushPose(double x, double y)
  {
    if (!path_.poses.empty()) {
      const auto &last = path_.poses.back().pose.position;
      if (std::abs(last.x - x) < 1e-6 && std::abs(last.y - y) < 1e-6) {
        return;
      }
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z_;
    ps.pose.orientation.w = 1.0;
    path_.poses.push_back(ps);
  }

  void appendLinear(const Pt &start, const Pt &end)
  {
    if (path_.poses.empty()) {
      pushPose(start.x, start.y);
    }

    const Vec s = fromPt(start);
    const Vec e = fromPt(end);
    const Vec diff = e - s;
    const double length = norm(diff);
    if (length < 1e-9) {
      pushPose(end.x, end.y);
      return;
    }

    const Vec dir = diff / length;
    double traveled = spacing_;
    while (traveled < length) {
      const Vec v = s + dir * traveled;
      Pt p = toPt(v);
      pushPose(p.x, p.y);
      traveled += spacing_;
    }

    pushPose(end.x, end.y);
  }

  void appendBezier(const Pt &P0, const Pt &C1, const Pt &C2, const Pt &P3)
  {
    const int segments = std::max(1, static_cast<int>(std::ceil(dist(P0, P3) / spacing_)));
    for (int i = 1; i <= segments; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(segments);
      const double u = 1.0 - t;
      const double b0 = u * u * u;
      const double b1 = 3.0 * u * u * t;
      const double b2 = 3.0 * u * t * t;
      const double b3 = t * t * t;
      const Pt p{
        b0 * P0.x + b1 * C1.x + b2 * C2.x + b3 * P3.x,
        b0 * P0.y + b1 * C1.y + b2 * C2.y + b3 * P3.y
      };
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = frame_id_;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = z_;
      ps.pose.orientation.w = 1.0;
      path_.poses.push_back(ps);
    }
  }

  void buildPath()
  {
    path_.header.frame_id = frame_id_;
    path_.poses.clear();
    if (waypoints_.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough waypoints to build a path.");
      return;
    }

    Pt emit_pt = waypoints_.front();
    if (emit_endpoints_) {
      pushPose(emit_pt.x, emit_pt.y);
    }

    const size_t N = waypoints_.size();
    for (size_t i = 1; i + 1 < N; ++i) {
      const Pt &Pm1 = waypoints_[i - 1];
      const Pt &P0 = waypoints_[i];
      const Pt &P1 = waypoints_[i + 1];

      Vec u_in = normalize(fromPt(P0) - fromPt(Pm1));
      Vec u_out = normalize(fromPt(P1) - fromPt(P0));

      if (norm(u_in) < 1e-9 || norm(u_out) < 1e-9) {
        appendLinear(emit_pt, P0);
        emit_pt = P0;
        continue;
      }

      double cosang = dot(u_in, u_out);
      cosang = std::min(1.0, std::max(-1.0, cosang));
      double ang_deg = std::acos(cosang) * kRadToDeg;
      if (ang_deg < corner_min_angle_deg_) {
        appendLinear(emit_pt, P0);
        emit_pt = P0;
        continue;
      }

      double Lin = dist(Pm1, P0);
      double Lout = dist(P0, P1);
      double trim_in = std::min(corner_trim_m_, 0.49 * Lin);
      double trim_out = std::min(corner_trim_m_, 0.49 * Lout);

      Pt A{P0.x - u_in.x * trim_in, P0.y - u_in.y * trim_in};
      Pt B{P0.x + u_out.x * trim_out, P0.y + u_out.y * trim_out};

      appendLinear(emit_pt, A);

      double tang_mag = tangent_gain_ * std::min(trim_in, trim_out);
      Pt C1{A.x + u_in.x * tang_mag / 3.0, A.y + u_in.y * tang_mag / 3.0};
      Pt C2{B.x - u_out.x * tang_mag / 3.0, B.y - u_out.y * tang_mag / 3.0};
      appendBezier(A, C1, C2, B);

      emit_pt = B;
    }

    if (emit_endpoints_) {
      appendLinear(emit_pt, waypoints_.back());
    } else {
      appendLinear(emit_pt, waypoints_.back());
    }

    setYawAlongPath();
  }

  void setYawAlongPath()
  {
    if (path_.poses.size() < 2) {
      return;
    }
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      size_t j = std::min(i + 1, path_.poses.size() - 1);
      double dx = path_.poses[j].pose.position.x - path_.poses[i].pose.position.x;
      double dy = path_.poses[j].pose.position.y - path_.poses[i].pose.position.y;
      if (i + 1 == path_.poses.size() - 1 && std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6 && i > 0) {
        dx = path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x;
        dy = path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y;
      }
      double yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      path_.poses[i].pose.orientation = tf2::toMsg(q);
    }
  }

  void publishMarker()
  {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = this->now();
    mk.ns = "global_path";
    mk.id = 1;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = 0.03;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.pose.orientation.w = 1.0;
    mk.points.clear();
    mk.points.reserve(path_.poses.size());
    for (const auto &ps : path_.poses) {
      geometry_msgs::msg::Point p;
      p.x = ps.pose.position.x;
      p.y = ps.pose.position.y;
      p.z = ps.pose.position.z;
      mk.points.push_back(p);
    }
    marker_pub_->publish(mk);
  }

  void publishOnce()
  {
    path_.header.stamp = this->now();
    path_pub_->publish(path_);
    publishMarker();
  }

  std::string frame_id_;
  double spacing_{0.05};
  double z_{0.0};
  double rate_hz_{1.0};
  double corner_trim_m_{0.5};
  double tangent_gain_{1.0};
  double corner_min_angle_deg_{5.0};
  bool emit_endpoints_{true};

  std::vector<Pt> waypoints_;
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPathGeneratorCornerSmooth>());
  rclcpp::shutdown();
  return 0;
}
