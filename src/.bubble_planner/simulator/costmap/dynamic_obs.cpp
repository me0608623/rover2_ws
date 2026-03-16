#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

struct DynObs
{
  std::string id;
  bool enabled{false};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double r{0.20};
};

class DynObsGridMapOnly : public rclcpp::Node
{
public:
  DynObsGridMapOnly()
  : rclcpp::Node("dynamic_obs")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    map_len_ = this->declare_parameter<double>("map_length", 11.0);
    map_wid_ = this->declare_parameter<double>("map_width", 11.0);
    res_ = this->declare_parameter<double>("resolution", 0.05);
    rate_hz_ = this->declare_parameter<double>("rate_hz", 10.0);
    use_unknown_ = this->declare_parameter<bool>("use_unknown", false);

    std::vector<std::string> ids = this->declare_parameter<std::vector<std::string>>(
      "obstacle_ids",
      {"o1", "o2", "o3", "o4", "o5", "o6", "o7", "o8", "o9", "o10"});
    loadObstaclesFromParams(ids);

    obstacles_ = {
      DynObs{"o1", true, 3.0, 8.5, 0.0, 0.0, 0.05},
      DynObs{"o2", true, 4.0, 9.5, 0.0, 0.0, 0.05},
      DynObs{"o3", true, 5.0, 8.5, 0.0, 0.0, 0.05},
      DynObs{"o4", true, 6.5, 8.0, 0.1, 0.3, 0.15},
      DynObs{"o5", true, 7.0, 9.5, 0.4, 0.4, 0.15},
      DynObs{"o6", true, 8.5, 8.0, 0.3, 0.2, 0.15},
      DynObs{"o7", true, 3.0, 7.0, 0.3, 0.2, 0.15},
      DynObs{"o8", true, 9.0, 8.0, 0.1, 0.2, 0.15},
      DynObs{"o9", true, 1.0, 9.0, 0.2, -0.4, 0.15},
      DynObs{"o10", true, 7.5, 5.5, 0.2, 0.1, 0.15},
    };

    buildGridMeta();
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("dynamic_costmap", 1);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
      std::bind(&DynObsGridMapOnly::onTimer, this));
    last_stamp_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "dynamic_obs: frame=%s, grid %.2fx%.2f @%.3f, N=%zu",
                frame_id_.c_str(), map_wid_, map_len_, res_, obstacles_.size());
  }

private:
  void loadObstaclesFromParams(const std::vector<std::string> &ids)
  {
    obstacles_.clear();
    for (const auto &id : ids) {
      DynObs o;
      o.id = id;
      o.enabled = this->declare_parameter<bool>("obstacles." + id + ".enabled", o.enabled);
      o.x = this->declare_parameter<double>("obstacles." + id + ".x", o.x);
      o.y = this->declare_parameter<double>("obstacles." + id + ".y", o.y);
      o.vx = this->declare_parameter<double>("obstacles." + id + ".vx", o.vx);
      o.vy = this->declare_parameter<double>("obstacles." + id + ".vy", o.vy);
      o.r = this->declare_parameter<double>("obstacles." + id + ".r", o.r);
      obstacles_.push_back(o);
    }
  }

  void buildGridMeta()
  {
    grid_.header.frame_id = frame_id_;
    grid_.info.resolution = res_;
    grid_.info.width = static_cast<uint32_t>(std::round(map_wid_ / res_));
    grid_.info.height = static_cast<uint32_t>(std::round(map_len_ / res_));
    grid_.info.origin.position.x = 0.0;
    grid_.info.origin.position.y = 0.0;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(grid_.info.width * grid_.info.height, use_unknown_ ? -1 : 0);
  }

  void setCell(uint32_t ix, uint32_t iy, int8_t val)
  {
    if (ix >= grid_.info.width || iy >= grid_.info.height) {
      return;
    }
    auto &cell = grid_.data[iy * grid_.info.width + ix];
    cell = std::max<int8_t>(cell, val);
  }

  void advance(double dt)
  {
    const double minX = 0.0;
    const double maxX = map_wid_;
    const double minY = 0.0;
    const double maxY = map_len_;

    for (auto &o : obstacles_) {
      if (!o.enabled) {
        continue;
      }

      o.x += o.vx * dt;
      o.y += o.vy * dt;

      if (o.x > maxX - o.r) {
        o.x = maxX - o.r;
        o.vx = -std::abs(o.vx);
      }
      if (o.x < minX + o.r) {
        o.x = minX + o.r;
        o.vx = std::abs(o.vx);
      }

      if (o.y > maxY - o.r) {
        o.y = maxY - o.r;
        o.vy = -std::abs(o.vy);
      }
      if (o.y < minY + o.r) {
        o.y = minY + o.r;
        o.vy = std::abs(o.vy);
      }
    }
  }

  void rasterize()
  {
    std::fill(grid_.data.begin(), grid_.data.end(), use_unknown_ ? -1 : 0);

    const double ox = grid_.info.origin.position.x;
    const double oy = grid_.info.origin.position.y;
    const double r = grid_.info.resolution;

    for (const auto &o : obstacles_) {
      if (!o.enabled) {
        continue;
      }

      const double cx = o.x;
      const double cy = o.y;
      const double rad = o.r;

      int min_ix = static_cast<int>(std::floor((cx - rad - ox) / r));
      int max_ix = static_cast<int>(std::floor((cx + rad - ox) / r));
      int min_iy = static_cast<int>(std::floor((cy - rad - oy) / r));
      int max_iy = static_cast<int>(std::floor((cy + rad - oy) / r));

      min_ix = std::max(0, min_ix);
      min_iy = std::max(0, min_iy);
      max_ix = std::min<int>(grid_.info.width - 1, max_ix);
      max_iy = std::min<int>(grid_.info.height - 1, max_iy);

      const double r2 = rad * rad;
      for (int iy = min_iy; iy <= max_iy; ++iy) {
        for (int ix = min_ix; ix <= max_ix; ++ix) {
          const double px = ox + (ix + 0.5) * r;
          const double py = oy + (iy + 0.5) * r;
          const double dx = px - cx;
          const double dy = py - cy;
          if (dx * dx + dy * dy <= r2) {
            setCell(ix, iy, 100);
          }
        }
      }
    }
  }

  void onTimer()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const rclcpp::Time now = this->now();
    const double dt = std::max(0.0, (now - last_stamp_).seconds());
    last_stamp_ = now;

    advance(dt);
    rasterize();

    grid_.header.stamp = now;
    pub_->publish(grid_);
  }

  std::string frame_id_;
  double map_len_{0.0};
  double map_wid_{0.0};
  double res_{0.0};
  double rate_hz_{10.0};
  bool use_unknown_{false};

  std::vector<DynObs> obstacles_;
  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_stamp_;
  std::mutex mtx_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynObsGridMapOnly>());
  rclcpp::shutdown();
  return 0;
}
