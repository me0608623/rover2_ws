#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

#include <mutex>
#include <algorithm>
#include <string>
#include <vector>
#include <utility>
#include <cmath>

class CostmapMergeNode : public rclcpp::Node
{
public:
  CostmapMergeNode() : Node("costmap_merge"),
                       tf_buffer_(this->get_clock()),
                       tf_listener_(tf_buffer_)
  {
    // ---- Parameters ----
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/costmap");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "turtle1");

    static_topic_ = this->declare_parameter<std::string>("static_topic", "/static_costmap");
    dynamic_topic_ = this->declare_parameter<std::string>("dynamic_topic", "/dynamic_costmap");

    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    map_width_ = this->declare_parameter<int>("map_width", 0);    // cells
    map_height_ = this->declare_parameter<int>("map_height", 0);  // cells
    map_size_ = this->declare_parameter<double>("map_size", 6.0); // meters (square)
    origin_x_ = this->declare_parameter<double>("origin_x", 0.0);
    origin_y_ = this->declare_parameter<double>("origin_y", 0.0);
    use_center_size_ = this->declare_parameter<bool>("use_map_size_square", true);

    rate_hz_ = this->declare_parameter<double>("rate_hz", 10.0);

    combine_method_ = this->declare_parameter<std::string>("combine_method", "max"); // max | overwrite_dynamic | overwrite_static
    unknown_overwrites_ = this->declare_parameter<bool>("unknown_overwrites", false);

    // ---- Inflation (like local_costmap) ----
    enable_inflation_ = this->declare_parameter<bool>("enable_inflation", true);
    inflation_radius_ = this->declare_parameter<double>("inflation_radius", 0.5);
    cost_scaling_factor_ = this->declare_parameter<double>("cost_scaling_factor", 5.0);
    lethal_threshold_ = this->declare_parameter<int>("lethal_threshold", 100); // treat >= threshold as obstacle

    // ---- QoS ----
    auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto qos_stat = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); // 靜態地圖常用 Transient Local
    auto qos_dyn = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // ---- Subscriptions ----
    auto qos_static = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // Volatile
    sub_static_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        static_topic_, qos_static,
        std::bind(&CostmapMergeNode::onStatic, this, std::placeholders::_1));

    sub_dynamic_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        dynamic_topic_, qos_dyn,
        std::bind(&CostmapMergeNode::onDynamic, this, std::placeholders::_1));

    // ---- Publisher ----
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, qos_pub);

    // ---- Timer ----
    using namespace std::chrono_literals;
    int period_ms = std::max(10, static_cast<int>(1000.0 / std::max(1.0, rate_hz_)));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&CostmapMergeNode::runOnce, this));

    RCLCPP_INFO(this->get_logger(),
                "costmap_merge started. out:%s frame:%s res=%.3f method=%s unknown_overwrites=%s",
                output_topic_.c_str(), output_frame_.c_str(), resolution_,
                combine_method_.c_str(), unknown_overwrites_ ? "true" : "false");
  }

private:
  // ===== Callbacks =====
  void onStatic(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    static_grid_ = *msg;
    have_static_ = true;
  }
  void onDynamic(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    dynamic_grid_ = *msg;
    have_dynamic_ = true;
  }

  // ===== Merge Timer =====
  void runOnce()
  {
    nav_msgs::msg::OccupancyGrid out;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_static_ && !have_dynamic_)
      {
        // 兩張都沒有就先不發
        return;
      }

      // 準備 output info
      prepareOutputInfo(out.info);

      // 先預設全 unknown(-1)
      out.data.assign(out.info.width * out.info.height, -1);

      // 逐 cell 投影到輸入坐標系去取值後合併
      for (uint32_t iy = 0; iy < out.info.height; ++iy)
      {
        for (uint32_t ix = 0; ix < out.info.width; ++ix)
        {
          const size_t out_idx = iy * out.info.width + ix;

          // output cell -> world (output_frame)
          double wx = out.info.origin.position.x + (static_cast<double>(ix) + 0.5) * out.info.resolution;
          double wy = out.info.origin.position.y + (static_cast<double>(iy) + 0.5) * out.info.resolution;

          // 從各輸入 grid 取值
          const int v_static = sampleGridInInput(static_grid_, have_static_, wx, wy, output_frame_);
          const int v_dynamic = sampleGridInInput(dynamic_grid_, have_dynamic_, wx, wy, output_frame_);

          // 合併
          out.data[out_idx] = combine(v_static, v_dynamic);
        }
      }
    }


    // Apply inflation ring (same idea as local_costmap)
    if (enable_inflation_ && inflation_radius_ > 0.0)
    {
      applyInflation(out);
    }

    out.header.stamp = this->now();
    out.header.frame_id = output_frame_;
    pub_->publish(out);
  }

  // ===== Utils =====
  void prepareOutputInfo(nav_msgs::msg::MapMetaData &info)
  {
    info.resolution = resolution_;

    if (use_center_size_)
    {
      // 用 map_size（正方形）推寬高
      const double size_m = std::max(0.1, map_size_);
      uint32_t w = static_cast<uint32_t>(std::round(size_m / resolution_));
      uint32_t h = static_cast<uint32_t>(std::round(size_m / resolution_));
      info.width = std::max<uint32_t>(1, w);
      info.height = std::max<uint32_t>(1, h);
      info.origin.position.x = origin_x_ - 0.5 * size_m; // 左下角
      info.origin.position.y = origin_y_ - 0.5 * size_m;
    }
    else
    {
      // 直接用 map_width/height（cells）與 origin
      info.width = (map_width_ > 0) ? static_cast<uint32_t>(map_width_) : 1u;
      info.height = (map_height_ > 0) ? static_cast<uint32_t>(map_height_) : 1u;
      info.origin.position.x = origin_x_;
      info.origin.position.y = origin_y_;
    }

    info.origin.position.z = 0.0;
    info.origin.orientation.x = 0.0;
    info.origin.orientation.y = 0.0;
    info.origin.orientation.z = 0.0;
    info.origin.orientation.w = 1.0;
  }

  // 將 output_frame 下的 (wx,wy) 投影到某張輸入 grid 的 frame，取值
  // 若沒有該 grid 或取不到回傳 -128（表示「沒有來源」）；格外用 -1 表 unknown。
  int sampleGridInInput(const nav_msgs::msg::OccupancyGrid &grid,
                        bool have_grid,
                        double wx_out, double wy_out,
                        const std::string &output_frame)
  {
    if (!have_grid)
      return -128;

    // 如同 ROS1 的 time=0（最接近的 transform）
    geometry_msgs::msg::TransformStamped T;
    try
    {
      // 我們有一個點在 output_frame，要表達到輸入 grid 的 frame：
      // target = grid.header.frame_id, source = output_frame
      T = tf_buffer_.lookupTransform(grid.header.frame_id, output_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF failed %s -> %s: %s",
                           output_frame.c_str(), grid.header.frame_id.c_str(), ex.what());
      return -128;
    }

    geometry_msgs::msg::PointStamped p_out, p_in;
    p_out.header.frame_id = output_frame;
    p_out.point.x = wx_out;
    p_out.point.y = wy_out;
    p_out.point.z = 0.0;

    tf2::doTransform(p_out, p_in, T);

    // world -> cell index in input grid
    int ix = static_cast<int>(std::floor((p_in.point.x - grid.info.origin.position.x) / grid.info.resolution));
    int iy = static_cast<int>(std::floor((p_in.point.y - grid.info.origin.position.y) / grid.info.resolution));

    if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.info.width) || iy >= static_cast<int>(grid.info.height))
    {
      return -128; // 超出範圍，表示此來源沒有提供
    }

    const int idx = iy * static_cast<int>(grid.info.width) + ix;
    int v = static_cast<int>(grid.data[idx]);
    return v; // -1=unknown, 0..100=占據率
  }

  // 合併兩個來源的取樣值（-128 表示沒有來源）
  int combine(int v_static, int v_dynamic) const
  {
    // 先處理「完全沒有來源」
    const bool has_s = (v_static != -128);
    const bool has_d = (v_dynamic != -128);
    if (!has_s && !has_d)
      return -1; // 看不到任何東西，輸出 unknown

    // 如果 unknown 需要覆蓋（-1 優先）
    if (unknown_overwrites_)
    {
      if ((has_d && v_dynamic == -1) || (has_s && v_static == -1))
        return -1;
    }

    // 正常合併
    if (combine_method_ == "overwrite_dynamic")
    {
      // dynamic 優先，其次 static，最後 unknown
      if (has_d && v_dynamic >= 0)
        return v_dynamic;
      if (has_s && v_static >= 0)
        return v_static;
      return -1;
    }
    else if (combine_method_ == "overwrite_static")
    {
      if (has_s && v_static >= 0)
        return v_static;
      if (has_d && v_dynamic >= 0)
        return v_dynamic;
      return -1;
    }
    else
    { // "max"
      int best = -1;
      if (has_s && v_static >= 0)
        best = std::max(best, v_static);
      if (has_d && v_dynamic >= 0)
        best = std::max(best, v_dynamic);
      // 若兩者皆 <0（unknown），則輸出 unknown
      return (best >= 0) ? best : -1;
    }
  }


  // ===== Inflation helpers (copied conceptually from local_costmap) =====
  static constexpr int8_t LETHAL_OBSTACLE = 100;
  static constexpr int8_t INSCRIBED_INFLATED_OBSTACLE = 99;
  static constexpr int8_t NO_INFORMATION = -1;

  int8_t computeCost(const double dis_meters) const
  {
    if (dis_meters < 0.0)
    {
      return 0;
    }

    // Nav2-style inflation (inscribed_radius treated as 0.0 here)
    // cost = (INSCRIBED_INFLATED_OBSTACLE - 1) * exp(-cost_scaling_factor * distance)
    double cost = (static_cast<double>(INSCRIBED_INFLATED_OBSTACLE) - 1.0) *
                  std::exp(-1.0 * cost_scaling_factor_ * dis_meters);

    cost = std::max(0.0, std::min(cost, static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)));
    return static_cast<int8_t>(cost);
  }

  void applyInflation(nav_msgs::msg::OccupancyGrid &grid) const
  {
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    if (width <= 0 || height <= 0)
      return;

    const double res = grid.info.resolution;
    if (res <= 0.0)
      return;

    const int radius_cells = static_cast<int>(inflation_radius_ / res);
    if (radius_cells <= 0)
      return;

    // 1) collect obstacle cells (>= lethal_threshold_)
    std::vector<std::pair<int, int>> obstacles;
    obstacles.reserve(1024);
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        const int idx = y * width + x;
        const int v = static_cast<int>(grid.data[idx]);
        if (v >= lethal_threshold_)
        {
          obstacles.emplace_back(x, y);
          // normalize to "lethal" like local_costmap
          grid.data[idx] = LETHAL_OBSTACLE;
        }
      }
    }

    // 2) inflate around each obstacle
    for (const auto &obs : obstacles)
    {
      const int ox = obs.first;
      const int oy = obs.second;

      for (int y = oy - radius_cells; y <= oy + radius_cells; ++y)
      {
        if (y < 0 || y >= height)
          continue;

        for (int x = ox - radius_cells; x <= ox + radius_cells; ++x)
        {
          if (x < 0 || x >= width)
            continue;

          const double dx = (x - ox) * res;
          const double dy = (y - oy) * res;
          const double dist = std::hypot(dx, dy);

          if (dist > inflation_radius_)
            continue;

          const int idx = y * width + x;
          const int8_t cost = computeCost(dist);

          // only overwrite if new cost is higher
          if (grid.data[idx] < cost)
          {
            grid.data[idx] = cost;
          }
        }
      }
    }
  }
private:
  // Params
  std::string output_topic_;
  std::string output_frame_;
  std::string static_topic_;
  std::string dynamic_topic_;
  double resolution_{0.05};
  int map_width_{0};
  int map_height_{0};
  double map_size_{6.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  bool use_center_size_{true};
  double rate_hz_{10.0};
  std::string combine_method_;
  bool unknown_overwrites_{false};

  // Inflation params
  bool enable_inflation_{true};
  double inflation_radius_{0.55};
  double cost_scaling_factor_{10.0};
  int lethal_threshold_{100};

  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_static_, sub_dynamic_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // buffers
  std::mutex mtx_;
  nav_msgs::msg::OccupancyGrid static_grid_, dynamic_grid_;
  bool have_static_{false}, have_dynamic_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapMergeNode>());
  rclcpp::shutdown();
  return 0;
}
