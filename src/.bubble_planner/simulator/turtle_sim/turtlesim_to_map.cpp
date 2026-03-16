#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>

class TurtlesimToMapNode : public rclcpp::Node
{
public:
  static constexpr double HALF_PI = 1.5707963267948966;

  TurtlesimToMapNode()
      : rclcpp::Node("turtlesim_pose_to_pose_stamped")
  {
    tf_time_offset_sec_ = this->declare_parameter<double>("tf_time_offset", 0.0);
    if (tf_time_offset_sec_ < 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "tf_time_offset cannot be negative. Resetting to 0.");
      tf_time_offset_sec_ = 0.0;
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("turtle1/pose_stamped", 10);
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", rclcpp::QoS(10),
        std::bind(&TurtlesimToMapNode::poseCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(10),
        std::bind(&TurtlesimToMapNode::initialPoseCallback, this, std::placeholders::_1));

    teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

    go_to_min_srv_ = this->create_service<std_srvs::srv::Empty>(
        "go_to_min_pose",
        std::bind(&TurtlesimToMapNode::goToMinPose, this, std::placeholders::_1, std::placeholders::_2));
    go_to_max_srv_ = this->create_service<std_srvs::srv::Empty>(
        "go_to_max_pose",
        std::bind(&TurtlesimToMapNode::goToMaxPose, this, std::placeholders::_1, std::placeholders::_2));
    go_to_custom_srv_ = this->create_service<std_srvs::srv::Empty>(
        "go_to_3_6",
        std::bind(&TurtlesimToMapNode::goToCustomPose, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = this->now();
    pose_stamped.pose.position.x = msg->x;
    pose_stamped.pose.position.y = msg->y;
    pose_stamped.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, msg->theta);
    pose_stamped.pose.orientation = tf2::toMsg(q);

    pose_pub_->publish(pose_stamped);
    broadcastTf(msg, q, pose_stamped.header.stamp);
  }

  void broadcastTf(const turtlesim::msg::Pose::SharedPtr &msg, const tf2::Quaternion &q, const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = "map";

    if (tf_time_offset_sec_ > 0.0)
    {
      const rclcpp::Duration offset = rclcpp::Duration::from_seconds(tf_time_offset_sec_);
      if (stamp > (this->get_clock()->now() - offset))
      {
        transform_stamped.header.stamp = stamp - offset;
      }
      else
      {
        transform_stamped.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      }
    }
    else
    {
      transform_stamped.header.stamp = stamp;
    }

    transform_stamped.child_frame_id = "turtle1";
    transform_stamped.transform.translation.x = msg->x;
    transform_stamped.transform.translation.y = msg->y;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(transform_stamped);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (x < 0.0 || x > 10.0 || y < 0.0 || y > 10.0)
    {
      RCLCPP_WARN(this->get_logger(), "Initial pose is out of bounds: (%.2f, %.2f)", x, y);
      return;
    }

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = x;
    request->y = y;
    request->theta = yaw;

    callTeleportService(request, "initial pose");
  }

  bool callTeleportService(const turtlesim::srv::TeleportAbsolute::Request::SharedPtr &request, const std::string &context)
  {
    if (!teleport_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service teleport_absolute unavailable while processing %s", context.c_str());
      return false;
    }

    auto future = teleport_client_->async_send_request(request);
    const auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Teleport service timed out for %s", context.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Turtle teleported via %s to (%.2f, %.2f, %.2f)",
                context.c_str(), request->x, request->y, request->theta);
    return true;
  }

  bool goToMinPose(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = 1.0;
    request->y = 1.0;
    request->theta = HALF_PI;
    return callTeleportService(request, "go_to_min_pose");
  }

  bool goToMaxPose(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = 10.0;
    request->y = 10.0;
    request->theta = HALF_PI;
    return callTeleportService(request, "go_to_max_pose");
  }

  bool goToCustomPose(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = 3.0;
    request->y = 6.0;
    request->theta = HALF_PI;
    return callTeleportService(request, "go_to_3_6");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr go_to_min_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr go_to_max_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr go_to_custom_srv_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double tf_time_offset_sec_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimToMapNode>());
  rclcpp::shutdown();
  return 0;
}
