#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <string>
#include <vector>

#define M_PI 3.14159265358979323846  /* pi */

using namespace std;

// ros::Subscriber odom_sub_;

// double delta_time_;
// double accumulation_x_, accumulation_y_, accumulation_th_;

// bool pub_tf_;
// string source_frame_id_;
// string target_frame_id_;

// void get_parameters(ros::NodeHandle n_private)
// {
//   n_private.param<bool>("pub_tf", pub_tf_, true);
//   n_private.param<string>("source_frame_id", source_frame_id_, "odom");
//   n_private.param<string>("target_frame_id", target_frame_id_, "base_footprint");
// }

// void OdomCallback(nav_msgs::Odometry odom_data)
// {
//   static tf2_ros::TransformBroadcaster br_;
//   static geometry_msgs::TransformStamped transformStamped_;
//   static nav_msgs::Odometry odom_;
//   static ros::Time last_time_, now_; 

//   tf2::Quaternion q;
//   if(pub_tf_)
//   {
//     transformStamped_.header.stamp = odom_data.header.stamp;
//     transformStamped_.header.frame_id = source_frame_id_;
//     transformStamped_.child_frame_id = target_frame_id_;
//     transformStamped_.transform.translation.x = odom_data.pose.pose.position.x;
//     transformStamped_.transform.translation.y = odom_data.pose.pose.position.y;
//     transformStamped_.transform.translation.z = 0.0;
    
//     transformStamped_.transform.rotation.x = odom_data.pose.pose.orientation.x;
//     transformStamped_.transform.rotation.y = odom_data.pose.pose.orientation.y;
//     transformStamped_.transform.rotation.z = odom_data.pose.pose.orientation.z;
//     transformStamped_.transform.rotation.w = odom_data.pose.pose.orientation.w;

//     br_.sendTransform(transformStamped_);
//   }

// }



// //-----------------------------------------------------------------------------------------------//
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "odom_to_tf");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_private("~");
//   get_parameters(nh_private);
//   ros::Time::init();

//   odom_sub_ = nh.subscribe("odom", 10, OdomCallback);

//   ros::spin();

//   return 0;
// }


class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF() : Node("odom_to_tf")
  {
    this->declare_parameter<bool>("pub_tf", true);
    this->declare_parameter<std::string>("source_frame_id", "odom");
    this->declare_parameter<std::string>("target_frame_id", "base_footprint");

    this->get_parameter("pub_tf", pub_tf_);
    this->get_parameter("source_frame_id", source_frame_id_);
    this->get_parameter("target_frame_id", target_frame_id_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OdomToTF::OdomCallback, this, std::placeholders::_1));
  }

private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_data)
  {
    if (!pub_tf_) return;

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_data->header.stamp;
    transformStamped.header.frame_id = source_frame_id_;
    transformStamped.child_frame_id = target_frame_id_;
    transformStamped.transform.translation.x = odom_data->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_data->pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = odom_data->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_data->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_data->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_data->pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(transformStamped);
  }

  bool pub_tf_;
  std::string source_frame_id_;
  std::string target_frame_id_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToTF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}