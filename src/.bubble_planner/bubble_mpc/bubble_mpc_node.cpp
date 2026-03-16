#include "bubble_mpc_solver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <bubble_planner/msg/obstacle_info.hpp>
#include <bubble_planner/msg/mpc_parameter.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/exceptions.h>

#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class BubbleMpcNode : public rclcpp::Node
{
public:
  BubbleMpcNode()
      : Node("mpc_node"),
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
        bubble_mpc_(std::make_unique<BubbleMpcSolver>())
  {
    this->declare_parameter<std::string>("target_frame_id", "world");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<std::string>("cmd_vel_topic", "/input/nav_cmd_vel");
    this->declare_parameter<std::string>("input_path", "/bubble_path");

    this->get_parameter("target_frame_id", target_frame_);
    this->get_parameter("child_frame_id", child_frame_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
    this->get_parameter("input_path", input_path_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&BubbleMpcNode::odomCb, this, _1));
    global_path_end_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/global_path_end", 10, std::bind(&BubbleMpcNode::globalEndPoseCb, this, _1));
    elevator_strategy_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/elevator_strategy", 10, std::bind(&BubbleMpcNode::elevatorStrategyCallback, this, _1));
    bubble_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        input_path_, 10, std::bind(&BubbleMpcNode::bubblePathCb, this, _1));
    bubble_info_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/bubble_info", 10, std::bind(&BubbleMpcNode::bubbleInfoCb, this, _1));
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&BubbleMpcNode::costmapCb, this, _1));
    mpc_finish_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/bubble_mpc_finish_to_mpc", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        { mpcFinishCb(msg); });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 100);
    bubble_mpc_finish_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bubble_mpc_finish", 10);
    mpc_finish_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mpc_finish", 10);
    mpc_predict_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_predict_path", 10);
    mpc_turn_on_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bubble_mpc_turn_on", 10);  // compute_flag_ status
    mpc_87_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mpc_87", 10);
    obs_info_pub_ = this->create_publisher<bubble_planner::msg::ObstacleInfo>("/bubble_mpc_obs_info", 10);
    way_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/way_pose", 10);
    way_pose_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
    nearest_bubble_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/nearest_bubble", 10);

    enable_mpc_server_ = this->create_service<std_srvs::srv::SetBool>(
        "/enable_bubble_mpc", std::bind(&BubbleMpcNode::enableService, this, _1, _2));

    robot_pose_timer_ = this->create_wall_timer(10ms, std::bind(&BubbleMpcNode::robotPoseTimer, this));
    mpc_timer_ = this->create_wall_timer(50ms, std::bind(&BubbleMpcNode::mpcCompute, this));
  }

private:
  // Utility functions --------------------------------------------------------
  static double angle_normalize(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double orientationToYaw(const geometry_msgs::msg::Quaternion &orientation)
  {
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  static void rotate_to(double &x, double &y, double &rad)
  {
    double new_x = x * std::cos(rad) - y * std::sin(rad);
    double new_y = x * std::sin(rad) + y * std::cos(rad);
    x = new_x;
    y = new_y;
  }

  // base on base_link
  bool checkCostmapRegion(double min_x, double max_x, double min_y, double max_y, int lethal_threshold = 90)
  {
    if (costmap_.data.empty())
    {
      // Costmap 尚未接收到，保守起見可以回傳 false 或印出警告
      return false;
    }

    double resolution = costmap_.info.resolution;
    double origin_x = costmap_.info.origin.position.x;
    double origin_y = costmap_.info.origin.position.y;
    unsigned int width = costmap_.info.width;
    unsigned int height = costmap_.info.height;

    // 將物理座標轉換為 Grid Index
    // idx = (pos - origin) / resolution
    int min_idx_x = static_cast<int>((min_x - origin_x) / resolution);
    int max_idx_x = static_cast<int>((max_x - origin_x) / resolution);
    int min_idx_y = static_cast<int>((min_y - origin_y) / resolution);
    int max_idx_y = static_cast<int>((max_y - origin_y) / resolution);

    // 邊界檢查，防止超出地圖範圍
    min_idx_x = std::max(0, min_idx_x);
    max_idx_x = std::min(static_cast<int>(width) - 1, max_idx_x);
    min_idx_y = std::max(0, min_idx_y);
    max_idx_y = std::min(static_cast<int>(height) - 1, max_idx_y);

    // 遍歷區域內的每一個 Grid
    for (int y = min_idx_y; y <= max_idx_y; ++y)
    {
      for (int x = min_idx_x; x <= max_idx_x; ++x)
      {
        unsigned int index = y * width + x;
        int cost = costmap_.data[index];

        // cost -1 表示未知區域 (Unknown)，視需求決定是否當作障礙物
        // 這裡假設大於閾值 (例如 90) 為障礙物 (Lethal Obstacle = 100, Inscribed = 99)
        if (cost > lethal_threshold)
        {
          return true;
        }
      }
    }
    return false;
  }

  void updateObstacleStatus()
  {
    // 定義檢測範圍 (相對於 base_link, 單位: meters)
    // 請根據實際機器人尺寸與安全距離進行調整
    // 假設機器人中心在 (0,0)，且 Costmap frame_id 為 base_link

    // 前方檢測區
    const double front_min_x = 0.1;
    const double front_max_x = 0.8;
    const double front_min_y = -0.35;
    const double front_max_y = 0.35;

    // 後方檢測區
    const double back_min_x = -1.1;
    const double back_max_x = -0.1;
    const double back_min_y = -0.35;
    const double back_max_y = 0.35;

    // 左側檢測區
    const double left_min_x = -0.5;
    const double left_max_x = 0.5;
    const double left_min_y = 0.2; // 避開機器人本體
    const double left_max_y = 0.6;

    // 右側檢測區
    const double right_min_x = -0.5;
    const double right_max_x = 0.5;
    const double right_min_y = -0.6;
    const double right_max_y = -0.2; // 避開機器人本體

    obs_info_msg_.front = checkCostmapRegion(front_min_x, front_max_x, front_min_y, front_max_y);
    obs_info_msg_.back = checkCostmapRegion(back_min_x, back_max_x, back_min_y, back_max_y);

    // 如果 msg 定義中有 all，則設為前後都有 (參考舊程式邏輯)
    // 或者可以定義為任意方向有障礙物: obs_info_msg_.front || obs_info_msg_.back ...
    obs_info_msg_.all = obs_info_msg_.front && obs_info_msg_.back;

    obs_info_pub_->publish(obs_info_msg_);
  }

  void tryResumeFromNoBubble()
  {
    if (!paused_for_no_bubble_ || compute_flag_)
      return;

    const auto now = this->get_clock()->now();
    const auto stale_threshold = rclcpp::Duration::from_seconds(0.5);
    const bool info_ready = bubbles_info_received_ && !bubbles_info_.poses.empty() &&
                            (now - last_bubbles_info_time_) <= stale_threshold;
    const bool path_ready = bubble_path_received_ && bubble_path_.poses.size() >= 2 &&
                            (now - last_bubble_path_time_) <= stale_threshold;

    if (info_ready && path_ready)
    {
      RCLCPP_WARN(this->get_logger(), "[bubble_mpc_node] bubble data recovered, resuming mpc.");
      paused_for_no_bubble_ = false;
      compute_flag_ = true;
      first_time_mpc_flag_ = true;
    }
  }

  // Callbacks ----------------------------------------------------------------
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    linear_x_ = msg->twist.twist.linear.x;
    angular_z_ = msg->twist.twist.angular.z;
  }

  void elevatorStrategyCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    elevator_strategy_mode_ = msg->data;
  }

  void globalEndPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    global_path_end_pose_ = *msg;
    global_path_end_received_ = true;
  }

  void bubblePathCb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    received_bubble_path_ = *msg;
    bubble_path_received_ = true;
    last_bubble_path_time_ = this->get_clock()->now();
    if (received_bubble_path_.poses.size() < 2)
    {
      return;
    }

    double total_length = 0.0;
    for (size_t i = 0; i + 1 < received_bubble_path_.poses.size(); ++i)
    {
      const auto &p1 = received_bubble_path_.poses[i].pose.position;
      const auto &p2 = received_bubble_path_.poses[i + 1].pose.position;
      total_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
    }
    RCLCPP_INFO(this->get_logger(), "Total BUBBLE path length: %f", total_length);

    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < received_bubble_path_.poses.size(); ++i)
    {
      const auto &pt = received_bubble_path_.poses[i].pose.position;
      double dist = std::hypot(
          pt.x - robot_tf_pose_stamped_.pose.position.x,
          pt.y - robot_tf_pose_stamped_.pose.position.y);
      if (dist < min_dist)
      {
        min_dist = dist;
        nearest_idx = static_cast<int>(i);
      }
    }

    double dist_tail = 0.0;
    for (size_t i = nearest_idx; i + 1 < received_bubble_path_.poses.size(); ++i)
    {
      const auto &p1 = received_bubble_path_.poses[i].pose.position;
      const auto &p2 = received_bubble_path_.poses[i + 1].pose.position;
      dist_tail += std::hypot(p2.x - p1.x, p2.y - p1.y);
    }

    current_to_end_total_length_ = min_dist + dist_tail;
    RCLCPP_INFO(this->get_logger(),
                "Total distance from robot to end: %f", current_to_end_total_length_);

    bubble_path_ = received_bubble_path_;
    bubble_path_total_length_ = total_length;

    tryResumeFromNoBubble();
  }

  void bubbleInfoCb(const nav_msgs::msg::Path::SharedPtr bubbles)
  {
    bubbles_info_ = *bubbles;
    bubbles_info_received_ = true;
    last_bubbles_info_time_ = this->get_clock()->now();

    tryResumeFromNoBubble();
  }

  void costmapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) // frame_id = "base_link"
  {
    costmap_ = *msg;
  }

  void mpcFinishCb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      finish_from_callback_ = true;
    }
  }

  void enableService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    compute_flag_ = request->data;
    response->success = true;
    response->message = "receive enable msg";
  }

  void robotPoseTimer()
  {
    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(target_frame_, child_frame_, tf2::TimePointZero);

      robot_tf_pose_stamped_.header.frame_id = target_frame_;
      robot_tf_pose_stamped_.header.stamp = this->get_clock()->now();
      robot_tf_pose_stamped_.pose.position.x = transform_stamped.transform.translation.x;
      robot_tf_pose_stamped_.pose.position.y = transform_stamped.transform.translation.y;
      robot_tf_pose_stamped_.pose.position.z = transform_stamped.transform.translation.z;
      robot_tf_pose_stamped_.pose.orientation = transform_stamped.transform.rotation;

      bubble_planner::msg::MPCParameter para;
      para.header = robot_tf_pose_stamped_.header;
      para.child_frame_id = child_frame_;
      para.position_x = robot_tf_pose_stamped_.pose.position.x;
      para.position_y = robot_tf_pose_stamped_.pose.position.y;
      para.position_theta = orientationToYaw(robot_tf_pose_stamped_.pose.orientation);
      para.linear_x = linear_x_;
      para.angular_z = angular_z_;

      auto para_ptr = std::make_shared<bubble_planner::msg::MPCParameter>(para);
      bubble_mpc_->UpdateState(para_ptr);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "[bubble_mpc_node] %s. Can't update pose from TF", ex.what());
    }
  }

  void mpcCompute()
  {
    static bool time_recording_started = false;
    static rclcpp::Time mpc_start_time;

    static geometry_msgs::msg::Twist last_cmd_vel;
    static int mpc_87_count = 0;
    static int early_stop_count = 0;
    static bool first_time_distance = true;
    static geometry_msgs::msg::Point last_recorded_position;
    static double cumulative_distance = 0.0;
    auto reset_for_no_bubble_stop = [&]()
    {
      time_recording_started = false;
      first_time_mpc_flag_ = true;
      mpc_87_count = 0;
      early_stop_count = 0;
      first_time_distance = true;
      cumulative_distance = 0.0;
      last_cmd_vel = geometry_msgs::msg::Twist();
    };

    std_msgs::msg::Bool mpc_status;
    mpc_status.data = compute_flag_;
    mpc_turn_on_pub_->publish(mpc_status);

    updateObstacleStatus();

    // nearst bubble visualizer
    if (!bubbles_info_.poses.empty())
    {
      size_t nearest_idx = 0;
      double nearest_dist = std::numeric_limits<double>::max();

      // Safety check: ensure we have a valid robot pose before calculating distance
      // If robot_tf_pose_stamped_ hasn't been updated, it might be at (0,0,0)
      const auto &rp = robot_tf_pose_stamped_.pose.position;

      for (size_t i = 0; i < bubbles_info_.poses.size(); ++i)
      {
        const auto &bp = bubbles_info_.poses[i].pose.position;
        const double d = std::hypot(bp.x - rp.x, bp.y - rp.y);
        if (d < nearest_dist)
        {
          nearest_dist = d;
          nearest_idx = i;
        }
      }

      const size_t total = bubbles_info_.poses.size();
      const size_t start_idx = nearest_idx;
      const size_t end_idx = std::min(total, start_idx + static_cast<size_t>(3));

      static int last_published_count = 0;
      int published = 0;

      for (size_t idx = start_idx; idx < end_idx; ++idx)
      {
        const auto &pose = bubbles_info_.poses[idx].pose;
        double radius = pose.position.z;
        if (radius <= 0.0)
          radius = 0.05;

        visualization_msgs::msg::Marker m;
        m.header.frame_id = target_frame_;
        m.header.stamp = this->get_clock()->now();
        m.ns = "nearest_bubbles";
        m.id = published;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = pose.position.x;
        m.pose.position.y = pose.position.y;
        m.pose.position.z = 0.0; // Keep on ground
        m.pose.orientation.w = 1.0;

        m.scale.x = radius * 2.0;
        m.scale.y = radius * 2.0;
        m.scale.z = 0.05;

        if (published == 0)
        {
          m.color.r = 0.0f;
          m.color.g = 1.0f;
          m.color.b = 0.0f;
        } // Green
        else if (published == 1)
        {
          m.color.r = 1.0f;
          m.color.g = 1.0f;
          m.color.b = 0.0f;
        } // Yellow
        else
        {
          m.color.r = 0.0f;
          m.color.g = 1.0f;
          m.color.b = 1.0f;
        } // Cyan
        m.color.a = 0.7f;

        m.lifetime = rclcpp::Duration::from_seconds(0.2); // Short lifetime to auto-clear
        nearest_bubble_pub_->publish(m);
        ++published;
      }

      // Clear old markers
      for (int id = published; id < last_published_count; ++id)
      {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = target_frame_;
        del.header.stamp = this->get_clock()->now();
        del.ns = "nearest_bubbles";
        del.id = id;
        del.action = visualization_msgs::msg::Marker::DELETE;
        nearest_bubble_pub_->publish(del);
      }
      last_published_count = published;
    }
    else
    {
      // Clear all if no bubbles
      static int last_published_count = 3; // minimal cleanup
      for (int id = 0; id < last_published_count; ++id)
      {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = target_frame_;
        del.header.stamp = this->get_clock()->now();
        del.ns = "nearest_bubbles";
        del.id = id;
        del.action = visualization_msgs::msg::Marker::DELETE;
        nearest_bubble_pub_->publish(del);
      }
    }

    if (!compute_flag_)
      return;

    const auto now = this->get_clock()->now();
    const auto stale_threshold = rclcpp::Duration::from_seconds(0.5);
    const bool bubbles_info_stale =
        !bubbles_info_received_ || (now - last_bubbles_info_time_) > stale_threshold;
    const bool bubble_path_stale =
        !bubble_path_received_ || (now - last_bubble_path_time_) > stale_threshold;

    if (bubbles_info_stale || bubble_path_stale)
    {
      nav_msgs::msg::Path bubbles_info_status = bubbles_info_;
      nav_msgs::msg::Path bubble_path_status = bubble_path_;
      if (bubbles_info_stale)
      {
        bubbles_info_status.poses.clear();
      }
      if (bubble_path_stale)
      {
        bubble_path_status.poses.clear();
      }

      bubble_mpc_->UpdateNoBubbleStatus(bubble_path_status, bubbles_info_status);
      if (bubble_mpc_->IsNoBubbleTriggered())
      {
        RCLCPP_WARN(this->get_logger(), "[bubble_mpc_node] no_bubble triggered, stopping mpc.");
        cmd_vel_ = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(cmd_vel_);
        last_cmd_vel = cmd_vel_;
        compute_flag_ = false;
        paused_for_no_bubble_ = true;
        reset_for_no_bubble_stop();
      }
      return;
    }

    if (bubble_path_.poses.size() < 2)
    {
      bubble_mpc_->UpdateNoBubbleStatus(bubble_path_, bubbles_info_);
      if (bubble_mpc_->IsNoBubbleTriggered())
      {
        RCLCPP_WARN(this->get_logger(), "[bubble_mpc_node] no_bubble triggered, stopping mpc.");
        cmd_vel_ = geometry_msgs::msg::Twist();
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 87;
        cmd_vel_pub_->publish(cmd_vel_);
        last_cmd_vel = cmd_vel_;
        compute_flag_ = false;
        paused_for_no_bubble_ = true;
        reset_for_no_bubble_stop();
      }
      return;
    }

    // 只在 mpc 開始跑時第一次開啟計時
    if (!time_recording_started)
    {
      mpc_start_time = this->get_clock()->now();
      time_recording_started = true;
      RCLCPP_INFO(this->get_logger(), "[bubble_mpc_node] MPC start time: %.3f", mpc_start_time.seconds());
    }

    // 第一次 mpc 開始運作時，將機器人位置設為 last_recorded_position
    // 後續計算與前次機器人位置距離 cumulative_distance
    if (first_time_distance)
    {
      last_recorded_position = robot_tf_pose_stamped_.pose.position;
      first_time_distance = false;
    }
    else
    {
      double dx = robot_tf_pose_stamped_.pose.position.x - last_recorded_position.x;
      double dy = robot_tf_pose_stamped_.pose.position.y - last_recorded_position.y;
      cumulative_distance += std::hypot(dx, dy);
      last_recorded_position = robot_tf_pose_stamped_.pose.position;
      RCLCPP_INFO(this->get_logger(), "Cumulative distance traveled: %f", cumulative_distance);
    }

    geometry_msgs::msg::PoseStamped goal_pose = global_path_end_pose_;
    const bool goal_frame_ok =
        !goal_pose.header.frame_id.empty() && goal_pose.header.frame_id == target_frame_;
    if (!global_path_end_received_ || !goal_frame_ok)
    {
      if (!bubble_path_.poses.empty())
      {
        goal_pose = bubble_path_.poses.back();
        if (goal_pose.header.frame_id.empty())
        {
          goal_pose.header.frame_id = target_frame_;
        }
      }
    }

    // 更新一些資訊
    bubble_mpc_->UpdateInformation(
        bubble_path_, bubbles_info_,
        bubble_path_total_length_, cumulative_distance,
        goal_pose);

    if (bubble_mpc_->IsNoBubbleTriggered())
    {
      RCLCPP_WARN(this->get_logger(), "[bubble_mpc_node] no_bubble triggered, stopping mpc.");
      cmd_vel_ = geometry_msgs::msg::Twist();
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.linear.z = 87;
      cmd_vel_pub_->publish(cmd_vel_);
      last_cmd_vel = cmd_vel_;
      compute_flag_ = false;
      paused_for_no_bubble_ = true;
      reset_for_no_bubble_stop();
      return;
    }

    // 第一次跑 mpc 時初始化參數
    if (first_time_mpc_flag_)
    {
      bubble_mpc_->ResetParameter();
      bubble_mpc_->InitialParameter();
      first_time_mpc_flag_ = false;
      return;
    }

    double dis_to_goal = std::hypot(
        robot_tf_pose_stamped_.pose.position.x - goal_pose.pose.position.x,
        robot_tf_pose_stamped_.pose.position.y - goal_pose.pose.position.y);
    double th_to_goal = std::fabs(angle_normalize(
        orientationToYaw(robot_tf_pose_stamped_.pose.orientation) -
        orientationToYaw(goal_pose.pose.orientation)));

    RCLCPP_INFO(this->get_logger(), "[bubble_mpc_node] dis_to_goal: %.3f, th_to_goal: %.3f",
                dis_to_goal, th_to_goal);

    const double stop_dist = 0.05;
    const double stop_yaw = 0.01;
    const bool should_stop =
        (dis_to_goal < stop_dist && th_to_goal < stop_yaw) ||
        (dis_to_goal < 0.3 && th_to_goal < 0.1) ||
        (std::fabs(linear_x_) < 0.01 && std::fabs(angular_z_) < 0.01 &&
         cmd_vel_.linear.z != 87 && dis_to_goal < 0.50 && th_to_goal < 0.1) ||
        early_stop_count > 8 || finish_from_callback_;

    if (should_stop)
    {
      const double robot_yaw = orientationToYaw(robot_tf_pose_stamped_.pose.orientation);
      const double goal_yaw = orientationToYaw(goal_pose.pose.orientation);
      RCLCPP_INFO(this->get_logger(),
                  "[bubble_mpc_node] stop: dist=%.3f yaw_diff=%.3f "
                  "robot(x=%.3f,y=%.3f,yaw=%.3f) goal(x=%.3f,y=%.3f,yaw=%.3f)",
                  dis_to_goal, th_to_goal,
                  robot_tf_pose_stamped_.pose.position.x,
                  robot_tf_pose_stamped_.pose.position.y,
                  robot_yaw,
                  goal_pose.pose.position.x,
                  goal_pose.pose.position.y,
                  goal_yaw);

      last_cmd_vel = geometry_msgs::msg::Twist();
      cmd_vel_ = geometry_msgs::msg::Twist();
      compute_flag_ = false;

      rclcpp::Time mpc_end_time = this->get_clock()->now();
      auto elapsed = mpc_end_time - mpc_start_time;
      RCLCPP_INFO(this->get_logger(), "[bubble_mpc_node] MPC total time %.3f s", elapsed.seconds());
      time_recording_started = false;
      first_time_mpc_flag_ = true;
      mpc_87_count = 0;
      early_stop_count = 0;
      first_time_distance = true;
      cumulative_distance = 0.0;
      bubble_path_.poses.clear();
      finish_from_callback_ = false;

      std_msgs::msg::Bool finish_flag;
      finish_flag.data = true;
      bubble_mpc_finish_pub_->publish(finish_flag);
      mpc_finish_pub_->publish(finish_flag);
      return;
    }

    nav_msgs::msg::Path mpc_predict_path;
    geometry_msgs::msg::Point way_point;
    cmd_vel_ = bubble_mpc_->Compute(mpc_predict_path, way_point);

    if (cmd_vel_.linear.z == 87 && mpc_87_count < 3)
    {
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_);
      last_cmd_vel = cmd_vel_;
      ++mpc_87_count;

      std_msgs::msg::Bool flag;
      flag.data = false;
      mpc_87_pub_->publish(flag);
      return;
    }
    else if (cmd_vel_.linear.z == 87 && mpc_87_count >= 3)
    {
      ++mpc_87_count;
      last_cmd_vel.linear.x = 0.0;
      last_cmd_vel.angular.z = 0.0;
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_);

      std_msgs::msg::Bool flag;
      flag.data = true;
      mpc_87_pub_->publish(flag);
      ++early_stop_count;
      return;
    }

    const double slow_down_dist = 1.0;
    if (dis_to_goal < slow_down_dist)
    {
      const double scale = std::clamp(dis_to_goal / slow_down_dist, 0.0, 1.0);
      cmd_vel_.linear.x *= scale;
    }

    const double max_acc = 0.3;
    const double dt = 0.1;
    const double max_delta_v = max_acc * dt;

    double delta_linear = cmd_vel_.linear.x - last_cmd_vel.linear.x;
    if (delta_linear > max_delta_v)
    {
      cmd_vel_.linear.x = last_cmd_vel.linear.x + max_delta_v;
    }
    else if (delta_linear < -max_delta_v)
    {
      cmd_vel_.linear.x = last_cmd_vel.linear.x - max_delta_v;
    }

    double delta_angular = cmd_vel_.angular.z - last_cmd_vel.angular.z;
    if (delta_angular > max_delta_v)
    {
      cmd_vel_.angular.z = last_cmd_vel.angular.z + max_delta_v;
    }
    else if (delta_angular < -max_delta_v)
    {
      cmd_vel_.angular.z = last_cmd_vel.angular.z - max_delta_v;
    }

    if (std::fabs(cmd_vel_.linear.x) <= 0.001)
    {
      cmd_vel_.linear.x = 0.0;
    }
    if (std::fabs(cmd_vel_.angular.z) <= 0.001)
    {
      cmd_vel_.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd_vel_);
    last_cmd_vel = cmd_vel_;

    mpc_predict_path.header.frame_id = target_frame_;
    mpc_predict_path.header.stamp = this->get_clock()->now();
    mpc_predict_path_pub_->publish(mpc_predict_path);

    geometry_msgs::msg::PoseStamped way_pose;
    way_pose.header.frame_id = target_frame_;
    way_pose.header.stamp = this->get_clock()->now();
    way_pose.pose.position.x = way_point.x;
    way_pose.pose.position.y = way_point.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, way_point.z);
    way_pose.pose.orientation = tf2::toMsg(q);
    way_pose_pub_->publish(way_pose);

    visualization_msgs::msg::Marker arrow;
    arrow.header = way_pose.header;
    arrow.ns = "waypoint_arrow";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = way_pose.pose;
    arrow.scale.x = 0.5;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0f;
    arrow.lifetime = rclcpp::Duration::from_seconds(1.0);
    way_pose_marker_pub_->publish(arrow);

    std_msgs::msg::Bool finish_flag;
    finish_flag.data = false;
    bubble_mpc_finish_pub_->publish(finish_flag);
    mpc_finish_pub_->publish(finish_flag);
  }

  // Members ------------------------------------------------------------------
  std::string child_frame_{"base_footprint"};
  std::string target_frame_{"world"};
  std::string cmd_vel_topic_{"/input/nav_cmd_vel"};
  std::string input_path_{"input_path"};

  double linear_x_{0.0};
  double angular_z_{0.0};

  geometry_msgs::msg::PoseStamped robot_tf_pose_stamped_;
  geometry_msgs::msg::PoseStamped global_path_end_pose_;
  nav_msgs::msg::Path bubble_path_;
  nav_msgs::msg::Path received_bubble_path_;
  nav_msgs::msg::Path bubbles_, bubbles_info_;
  nav_msgs::msg::OccupancyGrid costmap_;
  bubble_planner::msg::ObstacleInfo obs_info_msg_;

  double bubble_path_total_length_{0.0};
  double current_to_end_total_length_{0.0};

  bool first_time_mpc_flag_{true};
  bool compute_flag_{false};
  bool paused_for_no_bubble_{false};
  int elevator_strategy_mode_{0};
  bool finish_from_callback_{false};
  bool global_path_end_received_{false};

  geometry_msgs::msg::Twist cmd_vel_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_path_end_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr elevator_strategy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr bubble_path_sub_, bubble_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mpc_finish_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bubble_mpc_finish_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mpc_finish_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_predict_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mpc_turn_on_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mpc_87_pub_;
  rclcpp::Publisher<bubble_planner::msg::ObstacleInfo>::SharedPtr obs_info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr way_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr way_pose_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_bubble_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_mpc_server_;

  rclcpp::TimerBase::SharedPtr robot_pose_timer_;
  rclcpp::TimerBase::SharedPtr mpc_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<BubbleMpcSolver> bubble_mpc_;

  bool bubbles_info_received_{false};
  bool bubble_path_received_{false};
  rclcpp::Time last_bubbles_info_time_;
  rclcpp::Time last_bubble_path_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BubbleMpcNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4u);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
