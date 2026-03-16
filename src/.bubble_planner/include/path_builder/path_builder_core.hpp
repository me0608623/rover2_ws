#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <memory>
#include <type_traits>
#include <chrono>

namespace bsp
{
    struct Vec2
    {
        double x, y;
    };
    struct Edge
    {
        int to;
        double w;
    };
    using Graph = std::vector<std::vector<Edge>>;
    struct CandidatePath
    {
        std::vector<int> indices;
    };
    struct PathGeometry
    {
        std::vector<Vec2> pts;
    };
    struct PathScore
    {
        double total;
        bool is_backward;
    };
}

class PathBuilderCore
{
public:
    PathBuilderCore(tf2_ros::Buffer &tf_buffer, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

    enum class BuilderState
    {
        INIT,      // 剛啟動
        INTERCEPT, // State 1: 30度切入
        AVOIDANCE, // State 2: 避障規劃
        MAINTAIN   // State 3: 沿用+剪裁+延伸
    };

    struct Params
    {
        std::string target_frame_ = "world";
        std::string child_frame_ = "base_footprint";
        std::string costmap_topic_ = "/costmap";
        double default_lookahead = 4.0;
        int collision_cost_threshold = 90;
        double connect_radius = 0.90;
        int candidate_count = 3;

        // Weighting params
        double obs_influence_dist = 1.5;
        double obs_weight_max = 200.0;

        // Bubble Params
        double bubble_dis_away_ = 0.5;
        double robot_radius_ = 0.35;
        double bubble_max_search_dis_ = 1.0;
        double end_curve_dist = 1.0;
        double end_curve_min_yaw_diff = 1.0471975512; // 60 deg in rad
        double end_curve_ctrl_ratio = 0.35;
    };

    ~PathBuilderCore();

    void setParams(const Params &params);

    // Data Update Methods
    void updateGlobalPath(const nav_msgs::msg::Path &path);
    void updateLocalPath(const nav_msgs::msg::Path &path);
    void updateGlobalEndPose(const geometry_msgs::msg::PoseStamped &pose);
    void updateCostmap(const nav_msgs::msg::OccupancyGrid &map);
    nav_msgs::msg::Path densifiedPath(const nav_msgs::msg::Path &path);

    // Split Regular Points
    void updateLeftPoints(const std::vector<bsp::Vec2> &points);
    void updateRightPoints(const std::vector<bsp::Vec2> &points);

    bool getRobotPose(geometry_msgs::msg::PoseStamped &pose);

    // Main Compute Function
    nav_msgs::msg::Path computePath(const geometry_msgs::msg::PoseStamped &robot_pose);

    // Getters for Visualization/Debug
    BuilderState getCurrentState() const { return current_state_; }
    std::vector<bsp::PathGeometry> getLastCandidates() const { return last_candidates_; }
    size_t getLastBestIndex() const { return last_best_index_; }
    bsp::PathScore getLastScore() const { return last_score_; }
    geometry_msgs::msg::Point getActiveObstacle() const { return active_obs_center_; } // 雖然邏輯改了，還是保留給Vis用

    // Bubble
    std::pair<double, double> disToObstacle(geometry_msgs::msg::PoseStamped &pose_state, nav_msgs::msg::OccupancyGrid &map, double max_search_dis = 1);
    bool createBubble(const geometry_msgs::msg::PoseStamped &robot_pose,
                      nav_msgs::msg::Path &check_points, nav_msgs::msg::Path &out_bubbles,
                      double robot_radius, double away_dis);
    nav_msgs::msg::Path getLastBubblePath() const { return last_bubble_path_; }

private:
    rclcpp::Logger logger_;
    tf2_ros::Buffer &tf_buffer_;
    rclcpp::Clock::SharedPtr clock_;
    Params p_;

    // Data
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Path local_path_;
    geometry_msgs::msg::PoseStamped global_end_pose_;
    nav_msgs::msg::OccupancyGrid costmap_;

    std::vector<bsp::Vec2> points_left_;
    std::vector<bsp::Vec2> points_right_;

    // State Machine
    BuilderState current_state_ = BuilderState::INIT;
    nav_msgs::msg::Path last_output_path_;
    nav_msgs::msg::Path last_bubble_path_;
    bool first_run_ = true;

    // For Vis & Debug
    geometry_msgs::msg::Point active_obs_center_;
    std::vector<bsp::PathGeometry> last_candidates_;
    size_t last_best_index_ = 0;
    bsp::PathScore last_score_;

    // Core Logic
    nav_msgs::msg::Path runStateIntercept(const geometry_msgs::msg::PoseStamped &robot_pose);
    nav_msgs::msg::Path runStateAvoidance(const geometry_msgs::msg::PoseStamped &robot_pose);
    nav_msgs::msg::Path runStateMaintain(const geometry_msgs::msg::PoseStamped &robot_pose);

    // Helpers
    nav_msgs::msg::Path generateGraphPath(const std::vector<bsp::Vec2> &points, const bsp::Vec2 &start, const bsp::Vec2 &goal, double robot_yaw);
    nav_msgs::msg::Path pruneAndExtendPath(const nav_msgs::msg::Path &old_path, const nav_msgs::msg::Path &new_tail, const geometry_msgs::msg::PoseStamped &robot);
    bool buildIntercept30Deg(const geometry_msgs::msg::PoseStamped &robot_pose, nav_msgs::msg::Path &out_path);

    // Collision & Math
    bool checkCostmapCollision(const nav_msgs::msg::Path &path, size_t start, size_t end, geometry_msgs::msg::Point &out_hit);
    bool checkLineClear(const bsp::Vec2 &p1, const bsp::Vec2 &p2);
    double getObstacleDist(const bsp::Vec2 &p, double max_dist);
    int getPointCost(const bsp::Vec2 &p);
    bool worldToMap(double wx, double wy, int &mx, int &my) const;
    size_t findNearestIndex(const nav_msgs::msg::Path &path, const geometry_msgs::msg::Point &pt);
    size_t findLookaheadIndex(const nav_msgs::msg::Path &path, size_t start, double dist);

    // Graph
    std::vector<bsp::CandidatePath> generateCandidates(bsp::Graph &graph, const std::vector<bsp::Vec2> &nodes, int start, int goal, int count);
    std::vector<int> runDijkstra(const bsp::Graph &graph, int start, int goal);
};
