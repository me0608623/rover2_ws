#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2/time.h>

#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <limits>
#include <cmath>
#include <vector>
#include <optional>
#include <unistd.h>
#include <numeric>
#include <iomanip>

namespace
{
  double normalizeAngle(double ang)
  {
    while (ang > M_PI)
      ang -= 2.0 * M_PI;
    while (ang < -M_PI)
      ang += 2.0 * M_PI;
    return ang;
  }

  double pathLength(const nav_msgs::msg::Path &path)
  {
    double len = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
      const auto &a = path.poses[i - 1].pose.position;
      const auto &b = path.poses[i].pose.position;
      len += std::hypot(b.x - a.x, b.y - a.y);
    }
    return len;
  }
} // namespace

// 用於儲存單個 Process 的監控數據
struct ProcessMetrics
{
  int pid;
  std::string name;
  uint64_t last_proc_jiffies{0};
  double cpu_usage_sum{0.0};
  size_t samples{0};
  double max_mem_mb{0.0};
  double current_mem_kb{0.0};

  void reset()
  {
    last_proc_jiffies = 0;
    cpu_usage_sum = 0.0;
    samples = 0;
    max_mem_mb = 0.0;
    current_mem_kb = 0.0;
  }
};

class ExperimentLoggerV2 : public rclcpp::Node
{
public:
  ExperimentLoggerV2() : Node("experiment_logger_v2"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(tf_buffer_)
  {
    // Params
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("costmap_topic", "/costmap");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("path_topic", "/global_path"); // 改為監聽 global_path
    declare_parameter<std::string>("costmap_frame", "");
    declare_parameter<int>("collision_threshold", 90);
    declare_parameter<double>("collision_radius", 0.35);
    declare_parameter<double>("obstacle_search_radius", 5.0);
    declare_parameter<double>("cmd_vel_timeout", 1.0);

    // Multi-process monitoring params
    declare_parameter<std::vector<int64_t>>("target_pids", {});
    declare_parameter<std::vector<std::string>>("target_names", {});

    odom_topic_ = get_parameter("odom_topic").as_string();
    costmap_topic_ = get_parameter("costmap_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    explicit_costmap_frame_ = get_parameter("costmap_frame").as_string();
    collision_threshold_ = get_parameter("collision_threshold").as_int();
    collision_radius_ = get_parameter("collision_radius").as_double();
    obstacle_search_radius_ = get_parameter("obstacle_search_radius").as_double();
    cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();

    // Initialize Process Monitors
    std::vector<int64_t> pids = get_parameter("target_pids").as_integer_array();
    std::vector<std::string> names = get_parameter("target_names").as_string_array();

    if (pids.empty())
    {
      // Default to self if no PIDs provided
      ProcessMetrics pm;
      pm.pid = static_cast<int>(::getpid());
      pm.name = "logger_self";
      monitored_processes_.push_back(pm);
      RCLCPP_WARN(get_logger(), "No target_pids provided. Monitoring self (PID: %d)", pm.pid);
    }
    else
    {
      for (size_t i = 0; i < pids.size(); ++i)
      {
        ProcessMetrics pm;
        pm.pid = static_cast<int>(pids[i]);
        // 如果名稱數量不夠，自動補齊
        pm.name = (i < names.size()) ? names[i] : ("proc_" + std::to_string(pm.pid));
        monitored_processes_.push_back(pm);
        RCLCPP_INFO(get_logger(), "Monitoring Target: %s (PID: %d)", pm.name.c_str(), pm.pid);
      }
    }

    cpu_core_count_ = std::max(1u, std::thread::hardware_concurrency());

    // Subscribers & Publishers
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 50, std::bind(&ExperimentLoggerV2::odomCb, this, std::placeholders::_1));
    sub_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, 5, std::bind(&ExperimentLoggerV2::costmapCb, this, std::placeholders::_1));
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, 5, std::bind(&ExperimentLoggerV2::pathCb, this, std::placeholders::_1));
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 50, std::bind(&ExperimentLoggerV2::cmdVelCb, this, std::placeholders::_1));

    pub_summary_ = create_publisher<std_msgs::msg::String>("/experiment_logger/summary", 1);

    srv_start_ = create_service<std_srvs::srv::Trigger>(
        "/experiment_logger/start",
        std::bind(&ExperimentLoggerV2::startSrv, this, std::placeholders::_1, std::placeholders::_2));
    srv_stop_ = create_service<std_srvs::srv::Trigger>(
        "/experiment_logger/stop",
        std::bind(&ExperimentLoggerV2::stopSrv, this, std::placeholders::_1, std::placeholders::_2));

    resource_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&ExperimentLoggerV2::resourceTick, this));

    RCLCPP_INFO(get_logger(), "ExperimentLogger v2 ready. Waiting for start command or cmd_vel.");
  }

private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_summary_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;
  rclcpp::TimerBase::SharedPtr resource_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string odom_topic_, costmap_topic_, cmd_vel_topic_, path_topic_;
  std::string explicit_costmap_frame_;
  int collision_threshold_;
  double collision_radius_;
  double obstacle_search_radius_;
  double cmd_vel_timeout_;

  // State
  bool running_{false};
  bool armed_{true};
  rclcpp::Time start_time_;

  geometry_msgs::msg::Point last_pos_;
  bool has_last_pos_{false};
  double last_yaw_{0.0};
  bool has_last_yaw_{false};
  double prev_heading_{0.0};
  bool has_prev_heading_{false};
  size_t heading_samples_{0};

  double traveled_dist_{0.0};
  double planned_path_length_{0.0}; // 只會被賦值一次
  double total_rotation_{0.0};
  double smoothness_sum_{0.0};
  double min_obstacle_dist_{std::numeric_limits<double>::infinity()};

  nav_msgs::msg::OccupancyGrid costmap_;
  bool has_costmap_{false};
  std::mutex costmap_mtx_;

  bool in_collision_{false};
  int collision_count_{0};

  rclcpp::Time first_cmd_vel_time_;
  rclcpp::Time last_cmd_vel_time_;
  size_t cmd_vel_count_{0};

  geometry_msgs::msg::PoseStamped last_robot_pose_;
  bool has_last_robot_pose_{false};
  geometry_msgs::msg::PoseStamped global_end_pose_;
  bool has_global_end_pose_{false};

  // Resource usage (Multi-Process)
  std::vector<ProcessMetrics> monitored_processes_;
  unsigned int cpu_core_count_{1};
  uint64_t last_system_total_jiffies_{0};

  // Callbacks ---------------------------------------------------------------
  void startSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    resetMetrics();
    armed_ = true;
    running_ = false;
    resp->success = true;
    resp->message = "Armed: waiting for cmd_vel to start";
    RCLCPP_INFO(get_logger(), "Armed. Waiting for cmd_vel to start.");
  }

  void stopSrv(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    if (!running_)
    {
      resp->success = false;
      resp->message = "Not running";
      return;
    }
    const auto result = finishExperiment("manual_stop");
    resp->success = true;
    resp->message = result;
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!running_)
      return;

    const auto &p = msg->pose.pose.position;
    const double yaw = tf2::getYaw(msg->pose.pose.orientation);

    if (has_last_pos_)
    {
      const double dx = p.x - last_pos_.x;
      const double dy = p.y - last_pos_.y;
      const double dist = std::hypot(dx, dy);
      traveled_dist_ += dist;

      if (dist > 1e-4)
      {
        const double heading = std::atan2(dy, dx);
        if (has_prev_heading_)
        {
          const double d_heading = normalizeAngle(heading - prev_heading_);
          smoothness_sum_ += d_heading * d_heading;
          ++heading_samples_;
        }
        prev_heading_ = heading;
        has_prev_heading_ = true;
      }

      if (has_last_yaw_)
      {
        total_rotation_ += std::abs(normalizeAngle(yaw - last_yaw_));
      }
    }

    last_pos_ = p;
    has_last_pos_ = true;
    last_yaw_ = yaw;
    has_last_yaw_ = true;
    last_robot_pose_.header = msg->header;
    last_robot_pose_.pose = msg->pose.pose;
    has_last_robot_pose_ = true;

    updateCollisionAndDistance(msg->header.frame_id, msg->pose.pose);
  }

  void pathCb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty())
      return;

    global_end_pose_ = msg->poses.back();
    has_global_end_pose_ = true;

    double len = pathLength(*msg);
    if (len > 0.0)
    {
      planned_path_length_ = len;
    }
  }

  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr)
  {
    const auto t = now();
    if (!running_ && armed_)
    {
      resetMetrics();
      armed_ = false;
      running_ = true;
      start_time_ = t;
      RCLCPP_INFO(get_logger(), "Experiment auto-started by cmd_vel.");
    }

    if (!running_)
      return;

    if (cmd_vel_count_ == 0)
      first_cmd_vel_time_ = t;
    last_cmd_vel_time_ = t;
    ++cmd_vel_count_;
  }

  void costmapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(costmap_mtx_);
    costmap_ = *msg;
    has_costmap_ = !costmap_.data.empty();
  }

  void resourceTick()
  {
    if (!running_)
      return;

    // 1. 讀取系統總 Jiffies (只讀一次，供所有 Process 計算分母)
    uint64_t current_total_jiffies = readSystemTotalJiffies();
    if (current_total_jiffies == 0)
      return;

    double total_delta = 0.0;
    if (last_system_total_jiffies_ > 0 && current_total_jiffies > last_system_total_jiffies_)
    {
      total_delta = static_cast<double>(current_total_jiffies - last_system_total_jiffies_);
    }

    // 2. 遍歷所有監控的 Process
    for (auto &pm : monitored_processes_)
    {
      uint64_t proc_jiffies = readProcessJiffies(pm.pid);

      // CPU 計算
      if (total_delta > 0.0 && pm.last_proc_jiffies > 0 && proc_jiffies > pm.last_proc_jiffies)
      {
        double proc_delta = static_cast<double>(proc_jiffies - pm.last_proc_jiffies);
        // %CPU = (Process Delta / System Delta) * Cores * 100
        double cpu = 100.0 * (proc_delta / total_delta) * cpu_core_count_;
        pm.cpu_usage_sum += cpu;
        pm.samples++;
      }
      pm.last_proc_jiffies = proc_jiffies;

      // Memory 計算
      double rss_kb = 0.0;
      if (readMemoryKb(pm.pid, rss_kb))
      {
        pm.current_mem_kb = rss_kb/1024.0;
        if (rss_kb > pm.max_mem_mb)
          pm.max_mem_mb = rss_kb/1024.0;
      }
    }

    last_system_total_jiffies_ = current_total_jiffies;

    // Timeout Check
    if (running_)
    {
      const auto now_time = now();
      if (last_cmd_vel_time_ != rclcpp::Time(0, 0, get_clock()->get_clock_type()) &&
          (now_time - last_cmd_vel_time_).seconds() > cmd_vel_timeout_)
      {
        finishExperiment("cmd_vel_timeout");
      }
    }
  }

  // Helpers -----------------------------------------------------------------
  void resetMetrics()
  {
    running_ = false;
    has_last_pos_ = false;
    has_last_yaw_ = false;
    has_prev_heading_ = false;
    heading_samples_ = 0;
    traveled_dist_ = 0.0;
    // planned_path_length_ 不重置，因為 Global Path 可能只發一次且在 Start 之前
    total_rotation_ = 0.0;
    smoothness_sum_ = 0.0;
    min_obstacle_dist_ = std::numeric_limits<double>::infinity();
    in_collision_ = false;
    collision_count_ = 0;
    cmd_vel_count_ = 0;
    first_cmd_vel_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_cmd_vel_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    last_system_total_jiffies_ = 0;
    has_last_robot_pose_ = false;

    // Reset all process monitors
    for (auto &pm : monitored_processes_)
    {
      pm.reset();
    }
  }

  double computeControlFreq() const
  {
    if (cmd_vel_count_ <= 1)
      return 0.0;
    const double dt = (last_cmd_vel_time_ - first_cmd_vel_time_).seconds();
    return dt > 0.0 ? static_cast<double>(cmd_vel_count_ - 1) / dt : 0.0;
  }

  std::optional<std::pair<double, double>> computeGoalError()
  {
    if (!has_last_robot_pose_ || !has_global_end_pose_)
      return std::nullopt;
    // ... (維持原有邏輯) ...
    // 為節省篇幅省略詳細 Transform 實作，假設與原檔相同
    const auto target_frame = global_end_pose_.header.frame_id;
    geometry_msgs::msg::PoseStamped robot_in_goal = last_robot_pose_;
    try
    {
      robot_in_goal = tf_buffer_.transform(last_robot_pose_, target_frame, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
      return std::nullopt;
    }

    const auto &rp = robot_in_goal.pose.position;
    const auto &gp = global_end_pose_.pose.position;
    const double dist = std::hypot(rp.x - gp.x, rp.y - gp.y);
    const double robot_yaw = tf2::getYaw(robot_in_goal.pose.orientation);
    const double goal_yaw = tf2::getYaw(global_end_pose_.pose.orientation);
    const double yaw_diff = std::abs(normalizeAngle(robot_yaw - goal_yaw));
    return std::make_pair(dist, yaw_diff);
  }

  std::string finishExperiment(const std::string &reason)
  {
    running_ = false;
    armed_ = true;

    const double duration = (now() - start_time_).seconds();
    const double control_freq = computeControlFreq();
    const double smoothness_idx = heading_samples_ > 0 ? smoothness_sum_ / static_cast<double>(heading_samples_) : 0.0;
    const double min_obs = std::isfinite(min_obstacle_dist_) ? min_obstacle_dist_ : -1.0;

    bool success = true;
    double goal_dist = -1.0;
    double goal_yaw_diff = -1.0;
    if (auto err = computeGoalError())
    {
      goal_dist = err->first;
      goal_yaw_diff = err->second;
      if (goal_dist > 0.3 && goal_yaw_diff > 0.1)
        success = false;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "result=" << (success ? "success" : "fail")
        << " reason=" << reason
        << " time_sec=" << duration
        << " planned_path_m=" << planned_path_length_
        << " traveled_path_m=" << traveled_dist_
        << " collisions=" << collision_count_
        << " min_obstacle_distance_m=" << min_obs
        << " total_rotation_rad=" << total_rotation_
        << " smoothness_index=" << smoothness_idx
        << " control_freq_hz=" << control_freq;

    if (goal_dist >= 0.0)
    {
      oss << " goal_dist_m=" << goal_dist << " goal_yaw_diff_rad=" << goal_yaw_diff;
    }

    // --- 輸出個別 Node 資源使用量 ---
    double total_avg_cpu = 0.0;
    double total_max_mem = 0.0;

    for (const auto &pm : monitored_processes_)
    {
      double avg_cpu = pm.samples > 0 ? pm.cpu_usage_sum / pm.samples : 0.0;

      oss << " cpu_" << pm.name << "=" << avg_cpu
          << " mem_" << pm.name << "_max=" << pm.max_mem_mb;

      total_avg_cpu += avg_cpu;
      total_max_mem += pm.max_mem_mb;
    }

    // --- 輸出總計資源 ---
    oss << " cpu_total=" << total_avg_cpu
        << " mem_total_max=" << total_max_mem; // 這裡簡單加總各 Node 最大值，作為保守估計

    const std::string summary = oss.str();

    std_msgs::msg::String msg;
    msg.data = summary;
    pub_summary_->publish(msg);

    RCLCPP_INFO(get_logger(), "Experiment summary: %s", summary.c_str());
    return summary;
  }

  void updateCollisionAndDistance(const std::string &pose_frame, const geometry_msgs::msg::Pose &pose)
  {
    // ... (維持原檔邏輯，省略以節省空間) ...
    nav_msgs::msg::OccupancyGrid costmap_copy;
    {
      std::lock_guard<std::mutex> lk(costmap_mtx_);
      if (!has_costmap_)
        return;
      costmap_copy = costmap_;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = pose_frame;
    ps.header.stamp = now();
    ps.pose = pose;
    geometry_msgs::msg::PoseStamped target_pose;
    std::string target_frame = explicit_costmap_frame_.empty() ? costmap_copy.header.frame_id : explicit_costmap_frame_;
    try
    {
      target_pose = tf_buffer_.transform(ps, target_frame, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &)
    {
      return;
    }

    const double min_dist = computeMinObstacleDistance(costmap_copy, target_pose.pose.position);
    if (min_dist < min_obstacle_dist_)
      min_obstacle_dist_ = min_dist;

    const bool collided = checkCollision(costmap_copy, target_pose.pose.position);
    if (collided && !in_collision_)
      ++collision_count_;
    in_collision_ = collided;
  }

  double computeMinObstacleDistance(const nav_msgs::msg::OccupancyGrid &map, const geometry_msgs::msg::Point &p) const
  {
    // ... (維持原檔邏輯) ...
    const double res = map.info.resolution;
    const int r_cells = std::ceil(obstacle_search_radius_ / res);
    const int cx = (p.x - map.info.origin.position.x) / res;
    const int cy = (p.y - map.info.origin.position.y) / res;
    double best = 999.0;
    for (int dy = -r_cells; dy <= r_cells; ++dy)
    {
      for (int dx = -r_cells; dx <= r_cells; ++dx)
      {
        int x = cx + dx, y = cy + dy;
        if (x >= 0 && x < (int)map.info.width && y >= 0 && y < (int)map.info.height)
        {
          if (map.data[y * map.info.width + x] > collision_threshold_)
          {
            double d = std::hypot(dx * res, dy * res);
            if (d < best)
              best = d;
          }
        }
      }
    }
    return best;
  }

  bool checkCollision(const nav_msgs::msg::OccupancyGrid &map, const geometry_msgs::msg::Point &p) const
  {
    // ... (維持原檔邏輯) ...
    const double res = map.info.resolution;
    const int r_cells = std::max(1, (int)std::ceil(collision_radius_ / res));
    const int cx = (p.x - map.info.origin.position.x) / res;
    const int cy = (p.y - map.info.origin.position.y) / res;
    for (int dy = -r_cells; dy <= r_cells; ++dy)
    {
      for (int dx = -r_cells; dx <= r_cells; ++dx)
      {
        int x = cx + dx, y = cy + dy;
        if (x >= 0 && x < (int)map.info.width && y >= 0 && y < (int)map.info.height)
        {
          if (map.data[y * map.info.width + x] > collision_threshold_)
            return true;
        }
      }
    }
    return false;
  }

  // /proc helpers (Refactored) ----------------------------------------------
  uint64_t readSystemTotalJiffies()
  {
    std::ifstream fs("/proc/stat");
    if (!fs)
      return 0;
    std::string line;
    if (std::getline(fs, line))
    { // First line is "cpu ..."
      std::istringstream iss(line);
      std::string lbl;
      iss >> lbl;
      uint64_t val, sum = 0;
      while (iss >> val)
        sum += val;
      return sum;
    }
    return 0;
  }

  uint64_t readProcessJiffies(int pid)
  {
    std::ostringstream path;
    path << "/proc/" << pid << "/stat";
    std::ifstream ps(path.str());
    if (!ps)
      return 0;
    std::string line;
    if (!std::getline(ps, line))
      return 0;

    size_t rparen = line.rfind(')');
    if (rparen == std::string::npos)
      return 0;

    std::istringstream iss(line.substr(rparen + 2));
    std::string val;
    // skip fields: state(0), ppid(1)...
    // utime is 14th field (index 13 in 1-based, index 11 here after split)
    for (int i = 0; i < 11; ++i)
      iss >> val;

    uint64_t u = 0, s = 0;
    iss >> u >> s;
    return u + s;
  }

  bool readMemoryKb(int pid, double &rss_kb)
  {
    std::ostringstream path;
    path << "/proc/" << pid << "/status";
    std::ifstream fs(path.str());
    if (!fs)
      return false;
    std::string line;
    while (std::getline(fs, line))
    {
      if (line.rfind("VmRSS:", 0) == 0)
      {
        std::istringstream iss(line.substr(6));
        iss >> rss_kb;
        return true;
      }
    }
    return false;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExperimentLoggerV2>());
  rclcpp::shutdown();
  return 0;
}