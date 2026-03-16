#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <campusrover_msgs/msg/tracked_obstacle.hpp>
#include <campusrover_msgs/msg/tracked_obstacle_array.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

class DynamicCostmapTracker : public rclcpp::Node
{
public:
  DynamicCostmapTracker()
  : rclcpp::Node("tracked_obs")
  {
    costmap_topic_ = this->declare_parameter<std::string>("costmap_topic", "/dynamic_costmap");
    tracked_topic_ = this->declare_parameter<std::string>("tracked_topic", "/tracked_label_obstacle");
    tracked_height_ = this->declare_parameter<double>("tracked_height", 1.70);
    default_label_ = this->declare_parameter<int>("default_label", 1);
    dynamic_speed_thresh_ = this->declare_parameter<double>("dynamic_speed_threshold", 0.05);
    occupancy_threshold_ = this->declare_parameter<int>("occupancy_threshold", 80);
    min_cluster_cells_ = this->declare_parameter<int>("min_cluster_cells", 4);
    max_match_distance_ = this->declare_parameter<double>("max_match_distance", 1.0);

    max_match_distance_sq_ = max_match_distance_ * max_match_distance_;

    tracked_pub_ = this->create_publisher<campusrover_msgs::msg::TrackedObstacleArray>(tracked_topic_, 1);
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1),
      std::bind(&DynamicCostmapTracker::costmapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "tracked_obs: listening to '%s' and publishing on '%s'",
                costmap_topic_.c_str(), tracked_topic_.c_str());
  }

private:
  struct ClusterDetection
  {
    double cx{0.0};
    double cy{0.0};
    int min_ix{0};
    int max_ix{0};
    int min_iy{0};
    int max_iy{0};
    std::size_t cell_count{0};
  };

  struct StoredCluster
  {
    uint32_t id{0};
    double cx{0.0};
    double cy{0.0};
    double width{0.0};
    double length{0.0};
  };

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    const auto &info = msg->info;
    if (info.width == 0 || info.height == 0 || info.resolution <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tracked_obs: received invalid occupancy grid");
      return;
    }
    const std::size_t expected_size = static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
    if (msg->data.size() != expected_size) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tracked_obs: occupancy grid size mismatch (expected %zu, got %zu)",
                           expected_size, msg->data.size());
      return;
    }

    const auto clusters = extractClusters(*msg);

    campusrover_msgs::msg::TrackedObstacleArray out;
    out.header = msg->header;

    const double min_dt = 1e-3;
    double dt = 0.0;
    bool can_compute_velocity = false;
    if (prev_stamp_.nanoseconds() != 0) {
      dt = (rclcpp::Time(msg->header.stamp) - prev_stamp_).seconds();
      if (dt >= min_dt) {
        can_compute_velocity = true;
      }
    }

    std::vector<StoredCluster> new_states;
    new_states.reserve(clusters.size());
    std::vector<bool> prev_matched(prev_clusters_.size(), false);

    for (const auto &cluster : clusters) {
      std::size_t best_idx = prev_clusters_.size();
      double best_dist2 = max_match_distance_sq_ + 1.0;

      for (std::size_t i = 0; i < prev_clusters_.size(); ++i) {
        if (prev_matched[i]) {
          continue;
        }
        const double dx = cluster.cx - prev_clusters_[i].cx;
        const double dy = cluster.cy - prev_clusters_[i].cy;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < best_dist2 && dist2 <= max_match_distance_sq_) {
          best_dist2 = dist2;
          best_idx = i;
        }
      }

      uint32_t id = 0;
      double vx = 0.0;
      double vy = 0.0;

      if (best_idx < prev_clusters_.size()) {
        prev_matched[best_idx] = true;
        id = prev_clusters_[best_idx].id;
        if (can_compute_velocity) {
          const double inv_dt = 1.0 / dt;
          vx = (cluster.cx - prev_clusters_[best_idx].cx) * inv_dt;
          vy = (cluster.cy - prev_clusters_[best_idx].cy) * inv_dt;
        }
      } else {
        id = next_id_++;
      }

      const bool is_dynamic = std::hypot(vx, vy) > dynamic_speed_thresh_;
      const int label = is_dynamic ? default_label_ : 0;

      const double width = (cluster.max_ix - cluster.min_ix + 1) * info.resolution;
      const double length = (cluster.max_iy - cluster.min_iy + 1) * info.resolution;

      campusrover_msgs::msg::TrackedObstacle tracked;
      tracked.header = msg->header;
      tracked.id = id;
      tracked.label = label;
      tracked.is_dynamic = is_dynamic;
      tracked.pose.position.x = cluster.cx;
      tracked.pose.position.y = cluster.cy;
      tracked.pose.position.z = tracked_height_;
      tracked.pose.orientation.w = 1.0;
      tracked.velocity.linear.x = vx;
      tracked.velocity.linear.y = vy;
      tracked.dimensions.x = width;
      tracked.dimensions.y = length;
      tracked.dimensions.z = tracked_height_;

      out.obstacles.push_back(tracked);

      StoredCluster state;
      state.id = id;
      state.cx = cluster.cx;
      state.cy = cluster.cy;
      state.width = width;
      state.length = length;
      new_states.push_back(state);
    }

    tracked_pub_->publish(out);
    prev_clusters_.swap(new_states);
    prev_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  std::vector<ClusterDetection> extractClusters(const nav_msgs::msg::OccupancyGrid &grid) const
  {
    std::vector<ClusterDetection> clusters;

    const auto width = static_cast<int>(grid.info.width);
    const auto height = static_cast<int>(grid.info.height);
    const double resolution = grid.info.resolution;
    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;

    std::vector<uint8_t> visited(grid.data.size(), 0);
    const int neighbor_offsets[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    auto index = [width](int ix, int iy) {
      return iy * width + ix;
    };

    for (int iy = 0; iy < height; ++iy) {
      for (int ix = 0; ix < width; ++ix) {
        const int idx = index(ix, iy);
        const int8_t occ = grid.data[idx];
        if (occ < occupancy_threshold_ || visited[idx]) {
          continue;
        }

        ClusterDetection cluster;
        cluster.min_ix = cluster.max_ix = ix;
        cluster.min_iy = cluster.max_iy = iy;

        double sum_x = 0.0;
        double sum_y = 0.0;

        std::queue<std::pair<int, int>> q;
        q.emplace(ix, iy);
        visited[idx] = 1;

        while (!q.empty()) {
          const auto cell = q.front();
          q.pop();

          const int cx = cell.first;
          const int cy = cell.second;
          const int cidx = index(cx, cy);

          const double wx = origin_x + (static_cast<double>(cx) + 0.5) * resolution;
          const double wy = origin_y + (static_cast<double>(cy) + 0.5) * resolution;

          sum_x += wx;
          sum_y += wy;
          cluster.cell_count += 1;
          cluster.min_ix = std::min(cluster.min_ix, cx);
          cluster.max_ix = std::max(cluster.max_ix, cx);
          cluster.min_iy = std::min(cluster.min_iy, cy);
          cluster.max_iy = std::max(cluster.max_iy, cy);

          for (const auto &offset : neighbor_offsets) {
            const int nx = cx + offset[0];
            const int ny = cy + offset[1];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
              continue;
            }
            const int nidx = index(nx, ny);
            if (visited[nidx]) {
              continue;
            }
            const int8_t nocc = grid.data[nidx];
            if (nocc < occupancy_threshold_) {
              continue;
            }
            visited[nidx] = 1;
            q.emplace(nx, ny);
          }
        }

        if (static_cast<int>(cluster.cell_count) < min_cluster_cells_) {
          continue;
        }

        cluster.cx = sum_x / static_cast<double>(cluster.cell_count);
        cluster.cy = sum_y / static_cast<double>(cluster.cell_count);
        clusters.push_back(cluster);
      }
    }

    return clusters;
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<campusrover_msgs::msg::TrackedObstacleArray>::SharedPtr tracked_pub_;

  std::vector<StoredCluster> prev_clusters_;
  rclcpp::Time prev_stamp_;
  uint32_t next_id_{1};
  std::mutex mtx_;

  std::string costmap_topic_;
  std::string tracked_topic_;
  double tracked_height_{1.70};
  int default_label_{1};
  double dynamic_speed_thresh_{0.05};
  int occupancy_threshold_{80};
  int min_cluster_cells_{4};
  double max_match_distance_{1.0};
  double max_match_distance_sq_{1.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicCostmapTracker>());
  rclcpp::shutdown();
  return 0;
}
