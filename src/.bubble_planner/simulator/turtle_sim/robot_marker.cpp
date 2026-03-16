#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>

class RobotMarkerNode : public rclcpp::Node
{
public:
  RobotMarkerNode()
  : rclcpp::Node("robot_visualization")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_marker", qos);

    body_marker_.header.frame_id = this->declare_parameter<std::string>("frame_id", "turtle1");
    body_marker_.ns = "robot";
    body_marker_.id = 0;
    body_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    body_marker_.action = visualization_msgs::msg::Marker::ADD;
    body_marker_.scale.x = 0.05;
    body_marker_.color.a = 1.0;
    body_marker_.color.r = 1.0;
    body_marker_.color.g = 0.0;
    body_marker_.color.b = 0.0;
    body_marker_.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = 0.35;  p1.y = 0.35;  p1.z = 0.0;
    p2.x = 0.35;  p2.y = -0.35; p2.z = 0.0;
    p3.x = -0.35; p3.y = -0.35; p3.z = 0.0;
    p4.x = -0.35; p4.y = 0.35;  p4.z = 0.0;

    body_marker_.points = {p1, p2, p3, p4, p1};

    arrow_marker_ = body_marker_;
    arrow_marker_.ns = "robot_orientation";
    arrow_marker_.id = 1;
    arrow_marker_.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker_.scale.x = 0.1;
    arrow_marker_.scale.y = 0.3;
    arrow_marker_.scale.z = 0.1;
    arrow_marker_.points.clear();

    geometry_msgs::msg::Point arrow_start, arrow_end;
    arrow_start.x = 0.0;
    arrow_start.y = 0.0;
    arrow_start.z = 0.0;
    arrow_end.x = 0.3;
    arrow_end.y = 0.0;
    arrow_end.z = 0.0;

    arrow_marker_.points.push_back(arrow_start);
    arrow_marker_.points.push_back(arrow_end);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(100ms, std::bind(&RobotMarkerNode::publishMarkers, this));
  }

private:
  void publishMarkers()
  {
    const auto stamp = this->now();
    body_marker_.header.stamp = stamp;
    arrow_marker_.header.stamp = stamp;

    marker_pub_->publish(body_marker_);
    marker_pub_->publish(arrow_marker_);
  }

  visualization_msgs::msg::Marker body_marker_;
  visualization_msgs::msg::Marker arrow_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
