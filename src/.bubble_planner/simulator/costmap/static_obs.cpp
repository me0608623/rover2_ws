#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

struct StaticObstacle
{
  double x;
  double y;
  bool enabled{true};
};

namespace
{

  std::vector<StaticObstacle> turtleBack(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 3.0, start_y = 5.2;
    double end_x = 3.5, end_y = 5.9;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> turtleDown(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 5.2, start_y = 3.5;
    double end_x = 5.7, end_y = 4.2;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> turtleInside(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 4.7, start_y = 5.0;
    double end_x = 6.0, end_y = 6.1;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> turtleFrontUp(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 7.0, start_y = 5.9;
    double end_x = 7.1, end_y = 6.55;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> turtleFrontDown(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 7.0, start_y = 4.45;
    double end_x = 7.1, end_y = 5.1;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> turtleFront(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 7.0, start_y = 5.3;
    double end_x = 7.7, end_y = 6.1;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> bigWall(double res)
  {
    std::vector<StaticObstacle> out;
    double start_x = 7.0, start_y = 4.5;
    double end_x = 7.7, end_y = 6.6;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

  std::vector<StaticObstacle> Wall(double map_size, double default_radius)
  {
    std::vector<StaticObstacle> out;

    // (x, map_size) top
    for (double x = 0; x <= map_size + 1e-6; x += default_radius)
      out.push_back({x, map_size});

    // (x, 0) bottom
    for (double x = 2.5; x <= 8.0 + 1e-6; x += default_radius)
      out.push_back({x, 8.0});

    // (0,y) left
    for (double y = default_radius; y <= map_size + 1e-6; y += default_radius)
      out.push_back({0, y});

    // (map_size,y) right
    for (double y = default_radius; y <= map_size + 1e-6; y += default_radius)
      out.push_back({map_size, y});

    return out;
  }

  std::vector<StaticObstacle> DoYouself(double start_x, double start_y, double end_x, double end_y, double res)
  {
    std::vector<StaticObstacle> out;

    for (double x = start_x; x <= end_x + 1e-6; x += res)
    {
      for (double y = start_y; y <= end_y + 1e-6; y += res)
        out.push_back({x, y});
    }
    return out;
  }

} // namespace

class StaticObsNode : public rclcpp::Node
{
public:
  StaticObsNode()
      : rclcpp::Node("static_obs")
  {
    map_size_ = this->declare_parameter<double>("map_size", 11.0);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    inflation_radius_ = this->declare_parameter<double>("inflation_radius", 0.3);

    auto default_coords = std::vector<double>{5.0, 8.3, 5.5, 8.7};
    auto doyouself_coords = this->declare_parameter<std::vector<double>>("doyouself_coords", default_coords);
    start_x_ = doyouself_coords[0];
    start_y_ = doyouself_coords[1];
    end_x_ = doyouself_coords[2];
    end_y_ = doyouself_coords[3];

    loadObstacles();
    initializeMap();

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("static_costmap", 10);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(100ms, std::bind(&StaticObsNode::publishCostmap, this));
  }

private:
  void loadObstacles()
  {
    bool g1 = this->declare_parameter<bool>("turtle_back", false);
    bool g2 = this->declare_parameter<bool>("turtle_down", false);
    bool g3 = this->declare_parameter<bool>("turtle_inside", false);
    bool g4 = this->declare_parameter<bool>("turtle_front_up", false);
    bool g5 = this->declare_parameter<bool>("turtle_front_down", false);
    bool g6 = this->declare_parameter<bool>("turtle_front", false);
    bool g7 = this->declare_parameter<bool>("big_wall", false);
    bool g8 = this->declare_parameter<bool>("wall", true);
    bool g9 = this->declare_parameter<bool>("DoYouself", false);

    static_obstacles_.clear();
    if (g1)
    {
      const auto v = turtleBack(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g2)
    {
      const auto v = turtleDown(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g3)
    {
      const auto v = turtleInside(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g4)
    {
      const auto v = turtleFrontUp(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g5)
    {
      const auto v = turtleFrontDown(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g6)
    {
      const auto v = turtleFront(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g7)
    {
      const auto v = bigWall(resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g8)
    {
      const auto v = Wall(map_size_, resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
    if (g9)
    {
      const auto v = DoYouself(start_x_, start_y_, end_x_, end_y_, resolution_);
      static_obstacles_.insert(static_obstacles_.end(), v.begin(), v.end());
    }
  }

  void initializeMap()
  {
    map_.header.frame_id = "map";
    map_.info.resolution = resolution_;
    map_.info.width = static_cast<uint32_t>(std::round(map_size_ / resolution_));
    map_.info.height = static_cast<uint32_t>(std::round(map_size_ / resolution_));
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    map_.data.assign(map_.info.width * map_.info.height, 0);
  }

  void publishCostmap()
  {
    std::fill(map_.data.begin(), map_.data.end(), 0);

    for (const auto &obstacle : static_obstacles_)
    {
      const int center_x = static_cast<int>((obstacle.x - map_.info.origin.position.x) / resolution_);
      const int center_y = static_cast<int>((obstacle.y - map_.info.origin.position.y) / resolution_);
      const int obstacle_cells = 1;

      for (int dy = -obstacle_cells; dy <= obstacle_cells; ++dy)
      {
        for (int dx = -obstacle_cells; dx <= obstacle_cells; ++dx)
        {
          const int nx = center_x + dx;
          const int ny = center_y + dy;
          if (nx < 0 || ny < 0 ||
              nx >= static_cast<int>(map_.info.width) || ny >= static_cast<int>(map_.info.height))
          {
            continue;
          }

          const int index = ny * map_.info.width + nx;
          map_.data[index] = 100;
        }
      }
    }

    map_.header.stamp = this->now();
    map_pub_->publish(map_);
  }

  double map_size_{0.0};
  double resolution_{0.05};
  double inflation_radius_{0.0};

  double start_x_, start_y_, end_x_, end_y_;

  std::vector<StaticObstacle> static_obstacles_;
  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticObsNode>());
  rclcpp::shutdown();
  return 0;
}
