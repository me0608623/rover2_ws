#include "path_builder/path_builder_core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <tf2/time.h>

using namespace std;

namespace
{
    const char *stateName(PathBuilderCore::BuilderState state)
    {
        switch (state)
        {
        case PathBuilderCore::BuilderState::INIT:
            return "INIT";
        case PathBuilderCore::BuilderState::INTERCEPT:
            return "INTERCEPT";
        case PathBuilderCore::BuilderState::AVOIDANCE:
            return "AVOIDANCE";
        case PathBuilderCore::BuilderState::MAINTAIN:
            return "MAINTAIN";
        }
        return "UNKNOWN";
    }
} // namespace

// Helper Math
inline double length(const bsp::Vec2 &a, const bsp::Vec2 &b) { return std::hypot(a.x - b.x, a.y - b.y); }
double distToSegment(const geometry_msgs::msg::Point &p, const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    double abx = b.x - a.x, aby = b.y - a.y;
    double apx = p.x - a.x, apy = p.y - a.y;
    double ab_sq = abx * abx + aby * aby;
    if (ab_sq < 1e-6)
        return std::hypot(apx, apy);
    double t = std::max(0.0, std::min(1.0, (apx * abx + apy * aby) / ab_sq));
    return std::hypot(p.x - (a.x + t * abx), p.y - (a.y + t * aby));
}
double bspPolylineDistance(const bsp::Vec2 &p, const bsp::PathGeometry &geom)
{
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < geom.pts.size(); ++i)
    {
        geometry_msgs::msg::Point pp, pa, pb;
        pp.x = p.x;
        pp.y = p.y;
        pa.x = geom.pts[i - 1].x;
        pa.y = geom.pts[i - 1].y;
        pb.x = geom.pts[i].x;
        pb.y = geom.pts[i].y;
        best = std::min(best, distToSegment(pp, pa, pb));
    }
    return best;
}

PathBuilderCore::PathBuilderCore(tf2_ros::Buffer &tf_buffer, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
    : logger_(logger), tf_buffer_(tf_buffer), clock_(clock) {}
PathBuilderCore::~PathBuilderCore() = default;

// ================= PARAM =================
void PathBuilderCore::setParams(const Params &params) { p_ = params; }

// ================= DATA UPDATE =================
bool PathBuilderCore::getRobotPose(geometry_msgs::msg::PoseStamped &pose)
{
    try
    {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(p_.target_frame_, p_.child_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
        pose.header = tf.header;
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(logger_, "GetRobotPose Failed: %s", ex.what());
        return false;
    }
}
void PathBuilderCore::updateGlobalPath(const nav_msgs::msg::Path &path) { global_path_ = path; }
void PathBuilderCore::updateLocalPath(const nav_msgs::msg::Path &path) { local_path_ = path; }
void PathBuilderCore::updateGlobalEndPose(const geometry_msgs::msg::PoseStamped &pose) { global_end_pose_ = pose; }
void PathBuilderCore::updateCostmap(const nav_msgs::msg::OccupancyGrid &map) { costmap_ = map; }
void PathBuilderCore::updateLeftPoints(const std::vector<bsp::Vec2> &points) { points_left_ = points; }
void PathBuilderCore::updateRightPoints(const std::vector<bsp::Vec2> &points) { points_right_ = points; }

// ================= DIJKSTRA =================
std::vector<int> PathBuilderCore::runDijkstra(const bsp::Graph &graph, int start, int goal)
{
    // Standard Dijkstra implementation (same as provided before)
    int N = graph.size();
    std::vector<double> dist(N, std::numeric_limits<double>::infinity());
    std::vector<int> parent(N, -1);
    using Q = std::pair<double, int>;
    std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
    dist[start] = 0;
    pq.push({0, start});
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u])
            continue;
        if (u == goal)
            break;
        for (const auto &e : graph[u])
        {
            if (d + e.w < dist[e.to])
            {
                dist[e.to] = d + e.w;
                parent[e.to] = u;
                pq.push({dist[e.to], e.to});
            }
        }
    }
    std::vector<int> path;
    if (parent[goal] == -1 && start != goal)
        return path;
    for (int v = goal; v != -1; v = parent[v])
        path.push_back(v);
    std::reverse(path.begin(), path.end());
    return path;
}

// ================= SMALL FUNCTION =================
nav_msgs::msg::Path PathBuilderCore::densifiedPath(const nav_msgs::msg::Path &input_path_)
{
    nav_msgs::msg::Path out_path;
    out_path.poses.clear();
    out_path.header.frame_id = input_path_.header.frame_id;

    if (input_path_.poses.size() >= 2)
    {
        const auto &p0 = input_path_.poses[0].pose.position;
        size_t k = 1;
        while (k < input_path_.poses.size())
        {
            const auto &pk = input_path_.poses[k].pose.position;
            if (std::fabs(pk.x - p0.x) > 1e-6 || std::fabs(pk.y - p0.y) > 1e-6)
                break;
            ++k;
        }

        if (k < input_path_.poses.size())
        {
            const auto &p1 = input_path_.poses[k].pose.position;
            double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
        }
    }
    if (!input_path_.poses.empty())
    {
        geometry_msgs::msg::PoseStamped first_pose = input_path_.poses[0];
        first_pose.pose.position.z = 0;
        out_path.poses.push_back(first_pose);
    }

    if (input_path_.poses.size() < 2)
    {
        return out_path;
    }

    for (size_t i = 0; i < input_path_.poses.size() - 1; ++i)
    {
        const auto &p1 = input_path_.poses[i].pose.position;
        const auto &p2 = input_path_.poses[i + 1].pose.position;
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        auto orientation_msg = tf2::toMsg(q);
        double dist = std::hypot(dx, dy);
        if (dist < 1e-6)
            continue;
        int steps = std::max(1, (int)(dist / 0.1));

        for (int j = 1; j <= steps; ++j)
        {
            double t = (double)j / steps;
            geometry_msgs::msg::PoseStamped interp;
            interp.header = input_path_.header;
            interp.pose.position.x = p1.x + dx * t;
            interp.pose.position.y = p1.y + dy * t;
            interp.pose.position.z = 0;
            interp.pose.orientation = orientation_msg; // 所有點使用線段的朝向
            out_path.poses.push_back(interp);
        }
    }
    return out_path;
}

nav_msgs::msg::Path PathBuilderCore::pruneAndExtendPath(const nav_msgs::msg::Path &old_path, const nav_msgs::msg::Path &new_tail, const geometry_msgs::msg::PoseStamped &robot)
{
    nav_msgs::msg::Path res;
    res.header = old_path.header;
    res.header.stamp = clock_->now();

    if (old_path.poses.empty())
        return new_tail;

    // 1. Prune: Find closest point index
    size_t closest_idx = findNearestIndex(old_path, robot.pose.position);

    // 保留 closest_idx 之後的點 (包含 closest，防止斷層)
    for (size_t i = closest_idx; i < old_path.poses.size(); ++i)
    {
        res.poses.push_back(old_path.poses[i]);
    }

    // 2. Extend: Find where to attach local_path
    if (new_tail.poses.empty())
        return res;
    if (res.poses.empty())
        return new_tail;

    geometry_msgs::msg::Point tail_start = res.poses.back().pose.position;
    size_t attach_idx = findNearestIndex(new_tail, tail_start);

    // 從 attach_idx + 1 開始補，避免重疊過多
    for (size_t i = attach_idx + 1; i < new_tail.poses.size(); ++i)
    {
        res.poses.push_back(new_tail.poses[i]);
    }

    return res;
}

bool PathBuilderCore::buildIntercept30Deg(const geometry_msgs::msg::PoseStamped &robot_pose,
                                          nav_msgs::msg::Path &out_path)
{
    out_path.header.stamp = clock_->now();
    out_path.header.frame_id = p_.target_frame_;

    if (global_path_.poses.empty() || local_path_.poses.empty())
        return false;

    size_t robot_idx = findNearestIndex(global_path_, robot_pose.pose.position);
    geometry_msgs::msg::Point local_goal = local_path_.poses.back().pose.position;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    geometry_msgs::msg::PoseStamped goal_pose;
    bool have_goal_pose = false;
    bool goal_from_global_path = false;
    if (!global_end_pose_.header.frame_id.empty() && global_end_pose_.header.frame_id == p_.target_frame_)
    {
        goal_pose = global_end_pose_;
        have_goal_pose = true;
    }
    else if (!global_path_.poses.empty())
    {
        goal_pose = global_path_.poses.back();
        goal_pose.header.frame_id = p_.target_frame_;
        have_goal_pose = true;
        goal_from_global_path = true;
    }

    if (have_goal_pose)
    {
        if (goal_from_global_path && global_path_.poses.size() >= 2)
        {
            const auto &p_last = global_path_.poses.back().pose.position;
            const auto &p_prev = global_path_.poses[global_path_.poses.size() - 2].pose.position;
            double yaw = std::atan2(p_last.y - p_prev.y, p_last.x - p_prev.x);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            goal_pose.pose.orientation = tf2::toMsg(q);
        }

        double dist_robot_goal = std::hypot(goal_pose.pose.position.x - robot_pose.pose.position.x,
                                            goal_pose.pose.position.y - robot_pose.pose.position.y);
        double dist_local_goal = std::hypot(goal_pose.pose.position.x - local_goal.x,
                                            goal_pose.pose.position.y - local_goal.y);
        double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
        double yaw_diff = std::atan2(std::sin(goal_yaw - robot_yaw), std::cos(goal_yaw - robot_yaw));

        // Near goal with large yaw mismatch: build a smooth bezier approach.
        if (dist_robot_goal <= p_.end_curve_dist && dist_local_goal <= p_.end_curve_dist &&
            std::abs(yaw_diff) >= p_.end_curve_min_yaw_diff)
        {
            out_path.poses.clear();
            out_path.poses.push_back(robot_pose);

            bsp::Vec2 p0{robot_pose.pose.position.x, robot_pose.pose.position.y};
            bsp::Vec2 p3{goal_pose.pose.position.x, goal_pose.pose.position.y};
            double ctrl_dist = dist_robot_goal * p_.end_curve_ctrl_ratio;
            bsp::Vec2 p1{p0.x + ctrl_dist * std::cos(robot_yaw), p0.y + ctrl_dist * std::sin(robot_yaw)};
            bsp::Vec2 p2{p3.x - ctrl_dist * std::cos(goal_yaw), p3.y - ctrl_dist * std::sin(goal_yaw)};

            int steps = std::max(2, (int)std::ceil(dist_robot_goal / 0.1));
            for (int i = 1; i <= steps; ++i)
            {
                double t = (double)i / steps;
                double omt = 1.0 - t;
                double omt2 = omt * omt;
                double t2 = t * t;

                bsp::Vec2 pt;
                pt.x = omt2 * omt * p0.x + 3.0 * omt2 * t * p1.x + 3.0 * omt * t2 * p2.x + t2 * t * p3.x;
                pt.y = omt2 * omt * p0.y + 3.0 * omt2 * t * p1.y + 3.0 * omt * t2 * p2.y + t2 * t * p3.y;

                bsp::Vec2 d;
                d.x = 3.0 * omt2 * (p1.x - p0.x) + 6.0 * omt * t * (p2.x - p1.x) + 3.0 * t2 * (p3.x - p2.x);
                d.y = 3.0 * omt2 * (p1.y - p0.y) + 6.0 * omt * t * (p2.y - p1.y) + 3.0 * t2 * (p3.y - p2.y);

                geometry_msgs::msg::PoseStamped p;
                p.header = out_path.header;
                p.pose.position.x = pt.x;
                p.pose.position.y = pt.y;

                double yw = std::atan2(d.y, d.x);
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yw);
                p.pose.orientation = tf2::toMsg(q);
                out_path.poses.push_back(p);
            }

            if (!out_path.poses.empty())
            {
                out_path.poses.back().pose.orientation = goal_pose.pose.orientation;
            }
            return true;
        }
    }

    // 30度切入判斷用 heading
    size_t look_k = findLookaheadIndex(global_path_, robot_idx, 1.0);
    double gx = global_path_.poses[look_k].pose.position.x;
    double gy = global_path_.poses[look_k].pose.position.y;
    double path_heading = std::atan2(gy - robot_pose.pose.position.y, gx - robot_pose.pose.position.x);

    double ang_diff = std::atan2(std::sin(path_heading - robot_yaw), std::cos(path_heading - robot_yaw));

    // 你原本的條件：abs(ang_diff) <= 60deg 才允許切入
    if (std::abs(ang_diff) > (60.0 * M_PI / 180.0))
        return false;

    size_t global_end = findNearestIndex(global_path_, local_goal);

    out_path.poses.clear();
    out_path.poses.push_back(robot_pose);

    double dist_err = std::hypot(global_path_.poses[robot_idx].pose.position.x - robot_pose.pose.position.x,
                                 global_path_.poses[robot_idx].pose.position.y - robot_pose.pose.position.y);

    double req_dist = dist_err / std::tan(30.0 * M_PI / 180.0);

    size_t merge_idx = robot_idx;
    double acc = 0.0;
    for (size_t k = robot_idx; k + 1 < global_path_.poses.size() && k < global_end; ++k)
    {
        double sl = std::hypot(global_path_.poses[k + 1].pose.position.x - global_path_.poses[k].pose.position.x,
                               global_path_.poses[k + 1].pose.position.y - global_path_.poses[k].pose.position.y);
        acc += sl;
        if (acc >= req_dist)
        {
            merge_idx = k + 1;
            break;
        }
    }
    if (acc < req_dist)
        merge_idx = global_end;

    // 直線插值到 merge_idx
    geometry_msgs::msg::Point ps = robot_pose.pose.position;
    geometry_msgs::msg::Point pe = global_path_.poses[merge_idx].pose.position;
    double app_len = std::hypot(pe.x - ps.x, pe.y - ps.y);
    int steps = std::max(2, (int)(app_len / 0.1));

    for (int i = 1; i <= steps; ++i)
    {
        double t = (double)i / steps;
        geometry_msgs::msg::PoseStamped p;
        p.header = out_path.header;
        p.pose.position.x = ps.x + t * (pe.x - ps.x);
        p.pose.position.y = ps.y + t * (pe.y - ps.y);

        double yw = std::atan2(pe.y - ps.y, pe.x - ps.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yw);
        p.pose.orientation = tf2::toMsg(q);
        out_path.poses.push_back(p);
    }

    for (size_t i = merge_idx; i <= global_end && i < global_path_.poses.size(); ++i)
        out_path.poses.push_back(global_path_.poses[i]);

    return true;
}

size_t PathBuilderCore::findNearestIndex(const nav_msgs::msg::Path &path, const geometry_msgs::msg::Point &pt)
{
    double min_d = std::numeric_limits<double>::max();
    size_t idx = 0;
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        double d = std::hypot(path.poses[i].pose.position.x - pt.x, path.poses[i].pose.position.y - pt.y);
        if (d < min_d)
        {
            min_d = d;
            idx = i;
        }
    }
    return idx;
}

size_t PathBuilderCore::findLookaheadIndex(const nav_msgs::msg::Path &path, size_t start, double dist)
{
    if (start >= path.poses.size())
        return path.poses.size() - 1;
    double acc = 0;
    for (size_t i = start; i + 1 < path.poses.size(); ++i)
    {
        acc += std::hypot(path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x,
                          path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y);
        if (acc >= dist)
            return i + 1;
    }
    return path.poses.size() - 1;
}

// ================= BUBBLE =================
// nav_msgs::Path bubbles
// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// geometry_msgs/PoseStamped[] poses
//   std_msgs/Header header
//     uint32 seq -> pass checkpoint num
//     time stamp
//     string frame_id -> bubble frame_id
//   geometry_msgs/Pose pose
//     geometry_msgs/Point position
//       float64 x -> bubble x
//       float64 y -> bubble y
//       float64 z -> bubble radius
//     geometry_msgs/Quaternion orientation
//       float64 x -> obstacle position x
//       float64 y -> obstacle position y
//       float64 z -> obstacle orientation z(yaw)
//       float64 w -> obstacle orientation w(yaw)
bool PathBuilderCore::createBubble(const geometry_msgs::msg::PoseStamped &robot_pose,
                                   nav_msgs::msg::Path &check_points, nav_msgs::msg::Path &out_bubbles,
                                   double robot_radius, double away_dis)
{
    bool elevator_mode = false;
    out_bubbles.poses.clear();
    out_bubbles.header = check_points.header;

    if (check_points.poses.empty())
        return false;
    else
        check_points = densifiedPath(check_points);

    geometry_msgs::msg::PoseStamped planning_bubble;
    geometry_msgs::msg::PoseStamped last_planning_bubble, last_last_planning_bubble, next_bubble;
    double last_planning_bubble_radius; // initial value
    size_t initial_check_point_size;

    nav_msgs::msg::Path bubbles;
    std::vector<int> progress_calculate;

    // initialize first bubble
    geometry_msgs::msg::PoseStamped initial_pose_ = robot_pose;
    planning_bubble = robot_pose;
    planning_bubble.pose.position.z = 0;
    last_planning_bubble = planning_bubble;

    // check point loop
    for (int check_point_idx = 0; check_point_idx < static_cast<int>(check_points.poses.size()); check_point_idx++)
    {
        double dis_last_bubble_checkpoint = 0.0;

        // if robot in bubble set robot pose is planning bubble, robot out of bubble continue next checkpoint
        if (check_point_idx == 0)
        {
            double dis_first_checkpoint_bubble = std::hypot(check_points.poses.front().pose.position.x - planning_bubble.pose.position.x,
                                                            check_points.poses.front().pose.position.y - planning_bubble.pose.position.y);
            if (dis_first_checkpoint_bubble < planning_bubble.pose.position.z)
                continue;
            else
                planning_bubble.pose.position = check_points.poses.front().pose.position;
        }
        else if (check_point_idx == check_points.poses.size() - 1) // end_point
        {
            planning_bubble.pose = check_points.poses.back().pose;

            double dis_last = std::hypot(planning_bubble.pose.position.x - last_planning_bubble.pose.position.x,
                                         planning_bubble.pose.position.y - last_planning_bubble.pose.position.y);

            if (dis_last <= 0.3)
            {
                if (!bubbles.poses.empty())
                    bubbles.poses.pop_back();
            }

            pair<double, double> end_bubble_info = disToObstacle(planning_bubble, costmap_, p_.bubble_max_search_dis_);
            if (elevator_mode)
                planning_bubble.pose.position.z = end_bubble_info.first - robot_radius * 0.1;
            else 
                planning_bubble.pose.position.z = end_bubble_info.first - robot_radius + 0.05;
            if (end_bubble_info.second == 87)
            {
                planning_bubble.pose.orientation.x = 0.0;
                planning_bubble.pose.orientation.y = 0.0;
            }
            else
            {
                planning_bubble.pose.orientation.x = planning_bubble.pose.position.x + end_bubble_info.first * std::cos(end_bubble_info.second);
                planning_bubble.pose.orientation.y = planning_bubble.pose.position.y + end_bubble_info.first * std::sin(end_bubble_info.second);
            }
            bubbles.poses.push_back(planning_bubble);
        }
        else
            planning_bubble.pose.position = check_points.poses[check_point_idx].pose.position;

        dis_last_bubble_checkpoint = std::hypot(planning_bubble.pose.position.x - last_planning_bubble.pose.position.x,
                                                planning_bubble.pose.position.y - last_planning_bubble.pose.position.y);

        if (dis_last_bubble_checkpoint < last_planning_bubble.pose.position.z && check_point_idx != check_points.poses.size() - 1)
        {
            continue;
        }
        else if (dis_last_bubble_checkpoint >= last_planning_bubble.pose.position.z && check_point_idx != check_points.poses.size() - 1)
        {
            pair<double, double> dis_theta = disToObstacle(planning_bubble, costmap_, p_.bubble_max_search_dis_);
            double dis_bubble_obs = dis_theta.first;
            double bubble_angle = dis_theta.second;

            // int loop_count = 0;
            double clearance = robot_radius + away_dis;

            // second times keep away obstacle
            if (dis_bubble_obs <= clearance)
            {
                planning_bubble.pose.position.x -= (clearance - dis_bubble_obs) * std::cos(bubble_angle);
                planning_bubble.pose.position.y -= (clearance - dis_bubble_obs) * std::sin(bubble_angle);

                dis_theta = disToObstacle(planning_bubble, costmap_, p_.bubble_max_search_dis_);
                dis_bubble_obs = dis_theta.first;
                bubble_angle = dis_theta.second;
                if (dis_bubble_obs < clearance)
                {
                    // RCLCPP_WARN(
                    //     rclcpp::get_logger("PathBuilderCore"),
                    //     "createBubble: second check still collision: dis=%.3f <= clearance=%.3f, return false",
                    //     dis_bubble_obs, clearance);
                    return false; // 此路線不可用
                }
            }

            // 避免使用未初始化的 last_last_planning_bubble：只有在至少已經有 2 顆 bubble 時才做轉角過大濾除
            if (bubbles.poses.size() > 2)
            {
                double theta_last = std::atan2(planning_bubble.pose.position.y - last_planning_bubble.pose.position.y,
                                               planning_bubble.pose.position.x - last_planning_bubble.pose.position.x);
                double theta_last_last = std::atan2(last_planning_bubble.pose.position.y - last_last_planning_bubble.pose.position.y,
                                                    last_planning_bubble.pose.position.x - last_last_planning_bubble.pose.position.x);
                double angle_diff = theta_last - theta_last_last;
                angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
                if (std::abs(angle_diff) > ((2.0 * M_PI) / 3.0))
                {
                    RCLCPP_WARN(rclcpp::get_logger("PathBuilderCore"), "createBubble: bubbles return, return false");
                    continue;
                }
            }

            if (elevator_mode)
                planning_bubble.pose.position.z = dis_bubble_obs - robot_radius * 0.1;
            else 
                planning_bubble.pose.position.z = dis_bubble_obs - robot_radius + 0.05;
            if (bubbles.poses.size() > 2)
            {
                double dis_bubbles = std::hypot(planning_bubble.pose.position.x - last_planning_bubble.pose.position.x, planning_bubble.pose.position.y - last_planning_bubble.pose.position.y);
                if (dis_bubbles > (planning_bubble.pose.position.z + last_planning_bubble_radius) * 1.0)
                {
                    RCLCPP_WARN(rclcpp::get_logger("PathBuilderCore"), "createBubble: bubbles not continue, (%.2f,%.2f), dis:%.2f", check_points.poses[check_point_idx].pose.position.x, check_points.poses[check_point_idx].pose.position.y, dis_bubbles);
                    return false;
                }
            }

            last_planning_bubble_radius = planning_bubble.pose.position.z;

            if (bubble_angle == 87)
            {
                planning_bubble.pose.orientation.x = 0.0;
                planning_bubble.pose.orientation.y = 0.0;
            }
            else
            {
                planning_bubble.pose.orientation.x = planning_bubble.pose.position.x + dis_bubble_obs * std::cos(bubble_angle);
                planning_bubble.pose.orientation.y = planning_bubble.pose.position.y + dis_bubble_obs * std::sin(bubble_angle);
            }
            bubbles.poses.push_back(planning_bubble);
            last_last_planning_bubble = last_planning_bubble;
            last_planning_bubble = planning_bubble;
        }
    }
    bubbles.header.frame_id = check_points.header.frame_id;
    bubbles.header.stamp = clock_->now();
    out_bubbles = bubbles;
    return true;
}

std::pair<double, double> PathBuilderCore::disToObstacle(geometry_msgs::msg::PoseStamped &pose_state, nav_msgs::msg::OccupancyGrid &map, double max_search_dis)
{
    geometry_msgs::msg::PoseStamped map_frame_pose;
    geometry_msgs::msg::PoseStamped before_pose;
    before_pose.header = pose_state.header;
    before_pose.pose = pose_state.pose;
    if (before_pose.header.frame_id != map.header.frame_id)
    {
        try
        {
            map_frame_pose = tf_buffer_.transform(before_pose, map.header.frame_id, tf2::durationFromSec(0.5));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(logger_, "disToObstacle: map transform failed: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            map_frame_pose = before_pose;
        }
    }
    else
    {
        map_frame_pose = before_pose;
    }
    int robot_row = int((map_frame_pose.pose.position.x - costmap_.info.origin.position.x) / costmap_.info.resolution);
    int robot_col = int((map_frame_pose.pose.position.y - costmap_.info.origin.position.y) / costmap_.info.resolution);
    double search_grid = 1;
    double min_dis = numeric_limits<int>::max();
    double theta = 87;
    bool hit = false;

    while (!hit)
    {
        for (int i = robot_row - search_grid; i <= robot_row + search_grid; i++)
        {
            for (int j = robot_col - search_grid; j <= robot_col + search_grid; j++)
            {
                if (i != robot_row - search_grid && i != robot_row + search_grid && j != robot_col - search_grid && j != robot_col + search_grid)
                    continue;
                if (i < 0 || i >= costmap_.info.width || j < 0 || j >= costmap_.info.height)
                    continue;

                if (int(costmap_.data[j * costmap_.info.width + i]) >= p_.collision_cost_threshold)
                {
                    hit = true;
                    double dis = sqrt(pow(robot_row - i, 2) + pow(robot_col - j, 2)) * costmap_.info.resolution;

                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        theta = atan2((j - robot_col), (i - robot_row));
                    }
                }
            }
        }

        search_grid += 1;
        if (search_grid > (max_search_dis / costmap_.info.resolution))
        {
            min_dis = max_search_dis;
            break;
        }
    }

    if (hit) // 找到障礙物
    {
        search_grid = min_dis / costmap_.info.resolution;
        for (int i = robot_row - search_grid; i <= robot_row + search_grid; i++)
        {
            for (int j = robot_col - search_grid; j <= robot_col + search_grid; j++)
            {
                if (i < 0 || i >= costmap_.info.width || j < 0 || j >= costmap_.info.height)
                    continue;

                if (int(costmap_.data[j * costmap_.info.width + i]) >= p_.collision_cost_threshold)
                {
                    double dis = sqrt(pow(robot_row - i, 2) + pow(robot_col - j, 2)) * costmap_.info.resolution;

                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        theta = atan2((j - robot_col), (i - robot_row));
                    }
                }
            }
        }
    }

    if (theta != 87)
    {
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, theta);

        geometry_msgs::msg::PoseStamped before_yaw, after_yaw;
        before_yaw.header.frame_id = map.header.frame_id;
        before_yaw.header.stamp = pose_state.header.stamp;
        before_yaw.pose.orientation.z = myQuaternion.getZ();
        before_yaw.pose.orientation.w = myQuaternion.getW();

        try
        {
            after_yaw = tf_buffer_.transform(before_yaw, pose_state.header.frame_id, tf2::durationFromSec(0.5));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(logger_, "disToObstacle: yaw transform failed: %s", ex.what());
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            after_yaw = before_yaw;
        }
        tf2::Quaternion q_odom_pose(after_yaw.pose.orientation.x, after_yaw.pose.orientation.y, after_yaw.pose.orientation.z, after_yaw.pose.orientation.w);
        theta = tf2::getYaw(q_odom_pose);
    }
    pair<double, double> dis_angle(min_dis, theta);
    return dis_angle;
}

// ================= PATH BUILD =================

nav_msgs::msg::Path PathBuilderCore::generateGraphPath(const std::vector<bsp::Vec2> &points, const bsp::Vec2 &start, const bsp::Vec2 &goal, double robot_yaw)
{
    nav_msgs::msg::Path res_path;
    res_path.header.frame_id = p_.target_frame_;
    res_path.header.stamp = clock_->now();

    if (points.empty())
        return res_path;

    std::vector<bsp::Vec2> nodes;
    nodes.reserve(points.size() + 2);
    nodes.push_back(start);
    for (const auto &p : points)
        nodes.push_back(p);
    nodes.push_back(goal);

    bsp::Graph graph(nodes.size());
    int N = nodes.size();

    // Pre-calculate obstacle distances for Weighting Logic
    std::vector<double> obs_dists(N);
    for (int i = 0; i < N; ++i)
        obs_dists[i] = getObstacleDist(nodes[i], p_.obs_influence_dist);

    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            double d = length(nodes[i], nodes[j]);
            bool connect = false;

            if (d <= p_.connect_radius)
            {
                if (checkLineClear(nodes[i], nodes[j]))
                    connect = true;
            }

            if (connect)
            {
                double w = d;

                // Cost Stacking (Keep original logic as well, or replace? keeping as secondary)
                // === Exponential decay weighting ===
                double avg_obs = 0.5 * (obs_dists[i] + obs_dists[j]);

                // 只在影響距離內加權（外面就視為 w_min）
                if (avg_obs < p_.obs_influence_dist)
                {
                    double d = std::max(0.0, avg_obs);

                    // alpha 讓 d = obs_influence_dist 時接近 obs_weight_min
                    const double eps = 1.0; // 你可以改小一點(0.5)讓衰退更「貼近」min
                    double alpha = std::log((p_.obs_weight_max - 1) / eps) / p_.obs_influence_dist;

                    double factor = 1 + (p_.obs_weight_max - 1) * std::exp(-alpha * d);

                    w *= factor;
                }
                else
                    w *= 1; // 通常 obs_weight_min=1 就等於不變

                graph[i].push_back({j, w});
                graph[j].push_back({i, w});
            }
        }
    }

    auto idxs = runDijkstra(graph, 0, N - 1);
    if (idxs.empty())
        return res_path;

    // Convert indices to Path
    for (size_t i = 0; i < idxs.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = res_path.header;
        ps.pose.position.x = nodes[idxs[i]].x;
        ps.pose.position.y = nodes[idxs[i]].y;

        // Calulate Yaw
        tf2::Quaternion q;
        if (i + 1 < idxs.size())
        {
            double y = std::atan2(nodes[idxs[i + 1]].y - nodes[idxs[i]].y, nodes[idxs[i + 1]].x - nodes[idxs[i]].x);
            q.setRPY(0, 0, y);
        }
        else if (i > 0)
        {
            double y = std::atan2(nodes[idxs[i]].y - nodes[idxs[i - 1]].y, nodes[idxs[i]].x - nodes[idxs[i - 1]].x);
            q.setRPY(0, 0, y);
        }
        else
            q.setRPY(0, 0, 0);
        ps.pose.orientation = tf2::toMsg(q);
        res_path.poses.push_back(ps);
    }

    // Store candidate for visualization (Optional, just storing the last generated one)
    bsp::PathGeometry geom;
    for (int id : idxs)
        geom.pts.push_back(nodes[id]);
    last_candidates_.clear();
    last_candidates_.push_back(geom);

    return res_path;
}

std::vector<bsp::CandidatePath> PathBuilderCore::generateCandidates(bsp::Graph &graph, const std::vector<bsp::Vec2> &nodes, int start, int goal, int count) { return {}; }

// Need to implement getObstacleDist logic again or ensure it's copied
double PathBuilderCore::getObstacleDist(const bsp::Vec2 &p, double max_dist)
{
    geometry_msgs::msg::Point query_pt;
    query_pt.x = p.x;
    query_pt.y = p.y;
    query_pt.z = 0;
    if (costmap_.header.frame_id != p_.target_frame_ && !costmap_.header.frame_id.empty())
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(costmap_.header.frame_id, p_.target_frame_, rclcpp::Time(0));
            tf2::doTransform(query_pt, query_pt, tf);
        }
        catch (...)
        {
            return 0.0;
        }
    }
    int mx, my;
    if (!worldToMap(query_pt.x, query_pt.y, mx, my))
        return 0.0;

    int r = std::ceil(max_dist / costmap_.info.resolution);
    double min_d2 = max_dist * max_dist;
    bool found = false;
    for (int dy = -r; dy <= r; ++dy)
    {
        for (int dx = -r; dx <= r; ++dx)
        {
            int nx = mx + dx, ny = my + dy;
            if (nx < 0 || ny < 0 || nx >= costmap_.info.width || ny >= costmap_.info.height)
                continue;
            if (costmap_.data[ny * costmap_.info.width + nx] >= 100)
            {
                double d2 = (dx * dx + dy * dy) * std::pow(costmap_.info.resolution, 2);
                if (d2 < min_d2)
                {
                    min_d2 = d2;
                    found = true;
                }
            }
        }
    }
    return found ? std::sqrt(min_d2) : max_dist;
}

// ... (Other helpers like worldToMap, getPointCost, checkLineClear, Dijkstra, etc. are same as previous version) ...

int PathBuilderCore::getPointCost(const bsp::Vec2 &p)
{
    geometry_msgs::msg::Point query_pt;
    query_pt.x = p.x;
    query_pt.y = p.y;
    query_pt.z = 0;
    if (costmap_.header.frame_id != p_.target_frame_ && !costmap_.header.frame_id.empty())
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(costmap_.header.frame_id, p_.target_frame_, rclcpp::Time(0));
            tf2::doTransform(query_pt, query_pt, tf);
        }
        catch (...)
        {
            return -1;
        }
    }
    int mx, my;
    if (worldToMap(query_pt.x, query_pt.y, mx, my))
    {
        return costmap_.data[my * costmap_.info.width + mx];
    }
    return -1;
}

bool PathBuilderCore::checkLineClear(const bsp::Vec2 &p1, const bsp::Vec2 &p2)
{
    double dist = length(p1, p2);
    double step_size = std::max(0.05, (double)costmap_.info.resolution);
    int steps = std::ceil(dist / step_size);
    for (int i = 0; i <= steps; ++i)
    {
        double t = (double)i / steps;
        bsp::Vec2 pt;
        pt.x = p1.x + t * (p2.x - p1.x);
        pt.y = p1.y + t * (p2.y - p1.y);
        int cost = getPointCost(pt);
        if (cost >= p_.collision_cost_threshold || cost == -1)
            return false;
    }
    return true;
}

// Keep existing methods needed for compilation...
bool PathBuilderCore::worldToMap(double wx, double wy, int &mx, int &my) const
{
    if (costmap_.info.resolution <= 0)
        return false;
    mx = std::floor((wx - costmap_.info.origin.position.x) / costmap_.info.resolution);
    my = std::floor((wy - costmap_.info.origin.position.y) / costmap_.info.resolution);
    return (mx >= 0 && my >= 0 && mx < (int)costmap_.info.width && my < (int)costmap_.info.height);
}

// Stub for compilation if needed

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ================= MAIN COMPUTE (STATE MACHINE) =================

nav_msgs::msg::Path PathBuilderCore::computePath(const geometry_msgs::msg::PoseStamped &robot_pose)
{
    nav_msgs::msg::Path output_path;
    output_path.header.stamp = clock_->now();
    output_path.header.frame_id = p_.target_frame_;

    if (global_path_.poses.empty() || local_path_.poses.empty())
    {
        if (!last_bubble_path_.poses.empty())
        {
            nav_msgs::msg::Path cached = last_bubble_path_;
            cached.header.stamp = clock_->now();
            return cached;
        }
        return output_path;
    }

    if (first_run_)
    {
        current_state_ = BuilderState::INTERCEPT;
        first_run_ = false;
    }

    // =========================
    // 1) 每輪都先算判斷條件
    // =========================

    // --- intercept candidate ---
    nav_msgs::msg::Path intercept_path; // 回主線
    nav_msgs::msg::Path intercept_bubbles;
    bool intercept_ok = buildIntercept30Deg(robot_pose, intercept_path);

    // 以 createBubble 判斷 intercept_path 是否可通過
    bool intercept_passable = false;
    if (intercept_ok && !intercept_path.poses.empty())
    {
        intercept_passable = createBubble(robot_pose, intercept_path, intercept_bubbles,
                                          p_.robot_radius_, p_.bubble_dis_away_);
        // if (!intercept_passable)
        // {
        //     RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Can't choose intercept path!");
        // }
    }
    bool intercept_has_collision = !intercept_passable; // true -> 需要避障, false -> 可通行

    // debug hit point: 僅用於視覺化，不影響決策
    geometry_msgs::msg::Point intercept_hit;

    // --- maintained candidate (State3 output): last_output prune/extend local_tail ---
    nav_msgs::msg::Path maintained_path;
    if(!last_output_path_.poses.empty()) maintained_path = pruneAndExtendPath(last_output_path_, local_path_, robot_pose);
    else maintained_path = local_path_;


    nav_msgs::msg::Path maintained_bubbles;
    bool maintained_passable = false;
    if (!maintained_path.poses.empty())
    {
        maintained_passable = createBubble(robot_pose, maintained_path, maintained_bubbles,
                                           p_.robot_radius_, p_.bubble_dis_away_);
        if (!maintained_passable)
        {
            RCLCPP_WARN(logger_, "maintained_passable create fauled!");
        }
    }
    bool maintained_has_collision = !maintained_passable; // true -> 需要避障, false -> 可通行

    // debug hit point: 僅用於視覺化，不影響決策
    geometry_msgs::msg::Point maintained_hit;

    // debug vis point
    if (maintained_has_collision)
        active_obs_center_ = maintained_hit;
    else if (intercept_ok && !intercept_path.poses.empty() && intercept_has_collision)
        active_obs_center_ = intercept_hit;

    // =========================
    // 2) 決定「本輪要執行的狀態」(action_state)
    //    這一步讓狀態三能「當輪」切回狀態一並輸出 intercept
    // =========================
    BuilderState action_state = current_state_;

    switch (current_state_)
    {
    case BuilderState::INTERCEPT:
        action_state = intercept_has_collision ? BuilderState::AVOIDANCE : BuilderState::INTERCEPT;
        break;

    case BuilderState::AVOIDANCE:
        action_state = BuilderState::AVOIDANCE; // 本輪就做避障
        break;

    case BuilderState::MAINTAIN:
    default:
        if (!intercept_has_collision && intercept_ok && !intercept_path.poses.empty())
            action_state = BuilderState::INTERCEPT; // ★ 當輪立刻回狀態一輸出 intercept
        else if (maintained_has_collision)
            action_state = BuilderState::AVOIDANCE; // 當輪立刻避障
        else
            action_state = BuilderState::MAINTAIN; // 繼續維持
        break;
    }

    // =========================
    // 3) 依 action_state 輸出路徑 + 可通行 bubble
    // =========================
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "State current=%s action=%s",
                         stateName(current_state_), stateName(action_state));
    nav_msgs::msg::Path output_bubbles;
    bool bubble_ok = false;
    bool used_precomputed_bubbles = false;
    switch (action_state)
    {
    case BuilderState::INTERCEPT:
        if (intercept_ok && !intercept_path.poses.empty())
        {
            output_path = intercept_path;
            if (intercept_passable)
            {
                output_bubbles = intercept_bubbles;
                bubble_ok = true;
                used_precomputed_bubbles = true;
            }
        }
        else
        {
            output_path = local_path_;
        }
        break;

    case BuilderState::AVOIDANCE:
    {
        const auto t0 = std::chrono::steady_clock::now();
        output_path = runStateAvoidance(robot_pose);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        RCLCPP_INFO(logger_, "[PathBuilder] Avoidance compute time: %.3f ms", ms);
    }
    break;

    case BuilderState::MAINTAIN:
    default:
        output_path = maintained_path;
        if (maintained_passable)
        {
            output_bubbles = maintained_bubbles;
            bubble_ok = true;
            used_precomputed_bubbles = true;
        }
        break;
    }

    // 若本輪輸出的路徑不是預先計算過 bubbles（或預先計算不可通行），就針對 output_path 再算一次
    if (!used_precomputed_bubbles)
    {
        bubble_ok = createBubble(robot_pose, output_path, output_bubbles,
                                 p_.robot_radius_, p_.bubble_dis_away_);
        if (!bubble_ok)
        {
            RCLCPP_WARN(logger_, "bubble still bad!");
        }
    }

    // 儲存最後一條「可通行」的 bubble 路線供輸出使用
    if (bubble_ok)
    {
        last_bubble_path_ = output_bubbles;
        last_bubble_path_.header.stamp = clock_->now();
    }

    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "Bubble size: %zu", output_bubbles.poses.size());

    nav_msgs::msg::Path publish_path;
    if (!last_bubble_path_.poses.empty())
    {
        publish_path = last_bubble_path_;
        publish_path.header.stamp = clock_->now();
    }
    else
    {
        publish_path = output_bubbles;
        if (publish_path.header.frame_id.empty())
            publish_path.header = output_path.header;
        publish_path.header.stamp = clock_->now();
    }

    // =========================
    // 4) 更新「下一輪狀態」(next_state) 與 last_output
    //    規格：狀態二做完後下一輪一定進狀態三
    // =========================
    BuilderState next_state = action_state;
    if (action_state == BuilderState::AVOIDANCE)
        next_state = BuilderState::MAINTAIN;

    current_state_ = next_state;
    last_output_path_ = output_path;
    return publish_path;
}

// ================= STATE IMPLEMENTATIONS =================

// State 1: 30度切入 (原邏輯)
nav_msgs::msg::Path PathBuilderCore::runStateIntercept(const geometry_msgs::msg::PoseStamped &robot_pose)
{
    nav_msgs::msg::Path out;
    if (buildIntercept30Deg(robot_pose, out))
        return out;
    return local_path_;
}

// State 2: 避障規劃
nav_msgs::msg::Path PathBuilderCore::runStateAvoidance(const geometry_msgs::msg::PoseStamped &robot_pose)
{
    if (local_path_.poses.empty())
        return last_output_path_;

    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    bsp::Vec2 start = {local_path_.poses.front().pose.position.x, local_path_.poses.front().pose.position.y};
    bsp::Vec2 goal = {local_path_.poses.back().pose.position.x, local_path_.poses.back().pose.position.y};

    // 1. Generate Paths from Left and Right Graphs
    nav_msgs::msg::Path path_left = generateGraphPath(points_left_, start, goal, robot_yaw);
    nav_msgs::msg::Path path_right = generateGraphPath(points_right_, start, goal, robot_yaw);

    // 2. Evaluation Helper
    auto evalPath = [&](const nav_msgs::msg::Path &p) -> double
    {
        if (p.poses.empty())
            return std::numeric_limits<double>::max();

        // Dynamic Cost (Placeholder)
        double dynamic_cost = 0.0;

        // Length Cost
        double len = 0;
        for (size_t i = 0; i < p.poses.size() - 1; ++i)
        {
            len += std::hypot(p.poses[i + 1].pose.position.x - p.poses[i].pose.position.x,
                              p.poses[i + 1].pose.position.y - p.poses[i].pose.position.y);
        }

        // Heading Cost (First segment)
        double head_cost = 0;
        if (p.poses.size() > 1)
        {
            double dy = p.poses[1].pose.position.y - p.poses[0].pose.position.y;
            double dx = p.poses[1].pose.position.x - p.poses[0].pose.position.x;
            double yaw = std::atan2(dy, dx);
            double diff = std::abs(std::atan2(std::sin(yaw - robot_yaw), std::cos(yaw - robot_yaw)));
            head_cost = diff * 5.0; // 加權
        }

        return len + head_cost + dynamic_cost;
    };

    double score_l = evalPath(path_left);
    double score_r = evalPath(path_right);

    // 3. Selection
    // 如果某一邊沒路 (empty)，分數會是 max
    if (path_left.poses.empty() && path_right.poses.empty())
    {
        RCLCPP_WARN(logger_, "[PathBuilder] Both sides blocked! Using Local Path.");
        return local_path_;
    }

    if (score_l < score_r)
    {
        RCLCPP_INFO(logger_, "[PathBuilder] Avoidance: Selected LEFT ( %.2f vs %.2f)", score_l, score_r);
        return path_left;
    }
    else
    {
        RCLCPP_INFO(logger_, "[PathBuilder] Avoidance: Selected RIGHT ( %.2f vs %.2f)", score_r, score_l);
        return path_right;
    }
}

// State 3: 沿用 + 剪裁 + 延伸
nav_msgs::msg::Path PathBuilderCore::runStateMaintain(const geometry_msgs::msg::PoseStamped &robot_pose)
{
    // 如果上一輪路徑是空的，沒得剪裁，直接回傳 local 或 global
    if (last_output_path_.poses.empty())
        return local_path_;

    // 1. Prune & Extend
    nav_msgs::msg::Path maintained = pruneAndExtendPath(last_output_path_, local_path_, robot_pose);

    // 2. Fail-safe: 如果剪裁完剩太少，或延伸失敗，導致路徑過短
    if (maintained.poses.size() < 5)
    {
        // RCLCPP_WARN(this->get_logger(), "[PathBuilder] Maintained path too short, falling back to local.");
        // return local_path_;
        // 根據需求 "繼續補上local_path"，pruneAndExtend 應該已經盡力補了。
        // 如果還是很短，可能就是很短。
    }

    return maintained;
}
