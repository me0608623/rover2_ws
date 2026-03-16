#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/time.h> // [修正] 必須引用此標頭檔才能使用 tf2::TimePointZero

#include <cmath>
#include <limits>
#include <utility>
#include <string>
#include <chrono>
#include <memory> // [修正] unique_ptr 需要

using namespace std;
using namespace std::chrono_literals;

class LocalPathNode : public rclcpp::Node
{
public:
    LocalPathNode() : Node("local_path_node")
    {
        this->declare_parameter<std::string>("global_path_topic", "/global_path");
        this->declare_parameter<std::string>("costmap_topic", "/costmap");
        this->declare_parameter<std::string>("child_frame", "base_link");
        this->declare_parameter<double>("max_local_len", 5.0);
        this->declare_parameter<double>("point_spacing", 0.5);
        this->declare_parameter<double>("clearance", 0.1);
        this->declare_parameter<int>("obstacle_threshold", 100);
        this->declare_parameter<bool>("treat_minus128_as_obstacle", true);
        this->declare_parameter<double>("lookahead_dist_on_global", 1.0);
        this->declare_parameter<double>("collision_check_step", 0.05);
        this->declare_parameter<double>("cross_obstacle_extend", 2.0);
        this->declare_parameter<bool>("show_global_end_arrow", true);
        this->declare_parameter<double>("arrow_len", 0.6);
        this->declare_parameter<double>("arrow_shaft_d", 0.05);
        this->declare_parameter<double>("arrow_head_d", 0.12);
        this->declare_parameter<double>("arrow_head_len", 0.18);
        this->declare_parameter<double>("proj_backtrack_slack", 0.8);
        this->declare_parameter<double>("proj_corner_slack", 0.85);

        global_path_topic_ = this->get_parameter("global_path_topic").as_string();
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        max_local_len_ = this->get_parameter("max_local_len").as_double();
        point_spacing_ = this->get_parameter("point_spacing").as_double();
        clearance_ = this->get_parameter("clearance").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
        treat_minus128_as_obstacle_ = this->get_parameter("treat_minus128_as_obstacle").as_bool();
        lookahead_dist_on_global_ = this->get_parameter("lookahead_dist_on_global").as_double();
        collision_check_step_ = this->get_parameter("collision_check_step").as_double();
        cross_obstacle_extend_ = this->get_parameter("cross_obstacle_extend").as_double();
        show_global_end_arrow_ = this->get_parameter("show_global_end_arrow").as_bool();
        arrow_len_ = this->get_parameter("arrow_len").as_double();
        arrow_shaft_d_ = this->get_parameter("arrow_shaft_d").as_double();
        arrow_head_d_ = this->get_parameter("arrow_head_d").as_double();
        arrow_head_len_ = this->get_parameter("arrow_head_len").as_double();
        proj_backtrack_slack_ = this->get_parameter("proj_backtrack_slack").as_double();
        proj_corner_slack_ = this->get_parameter("proj_corner_slack").as_double();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic_, 1, std::bind(&LocalPathNode::globalPathCb, this, std::placeholders::_1));
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>( 
            costmap_topic_, 1, std::bind(&LocalPathNode::costmapCb, this, std::placeholders::_1));

        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 1);
        global_path_end_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/global_path_end", 10);
        debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/local_debug_markers", 1);
        global_end_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/global_path_end_marker", 1);
        
        timer_ = this->create_wall_timer(50ms, std::bind(&LocalPathNode::onTimer, this));
    }

private:
    // ===== ROS =====
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_path_end_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_end_marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // ===== 參數 =====
    std::string global_path_topic_, costmap_topic_, child_frame_;
    double max_local_len_{5.0}, point_spacing_{0.1}, clearance_{0.1};
    int obstacle_threshold_{100};
    bool treat_minus128_as_obstacle_{true};
    double lookahead_dist_on_global_{1.0}, collision_check_step_{0.05};
    double cross_obstacle_extend_{2.0};
    double proj_backtrack_slack_{0.2};
    double proj_corner_slack_{0.05};

    // ===== 狀態 =====
    nav_msgs::msg::Path latest_global_path_;
    bool have_global_path_ = false;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;

    bool show_global_end_arrow_ = true; 
    double arrow_len_ = 0.6;            
    double arrow_shaft_d_ = 0.05;       
    double arrow_head_d_ = 0.12;        
    double arrow_head_len_ = 0.18;      
    double last_projection_s_ = 0.0;
    bool have_last_projection_s_ = false;

    // ===== 回呼 =====
    void globalPathCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        latest_global_path_ = *msg;
        have_global_path_ = !msg->poses.empty();
    }

    // [修正] 這裡原本 Subscription 定義的是 nav_msgs::msg::OccupancyGrid，但 create_subscription 模板參數可能會寫錯
    void costmapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_costmap_ = *msg;
        have_costmap_ = latest_costmap_.info.width > 0 && latest_costmap_.info.height > 0 && !latest_costmap_.data.empty();
    }

    // ===== 主流程 =====
    void onTimer()
    {
        if (!have_global_path_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[local_straight] No global_path yet.");
            return;
        }

        const std::string target_frame = latest_global_path_.header.frame_id;

        // 1) 取得機器人目前位姿
        geometry_msgs::msg::PoseStamped robot_in_target;
        if (!lookupPose(child_frame_, target_frame, robot_in_target))
        {
            // [修正] Throttle 在 ROS2 需要 Clock 和 毫秒
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[local_straight] TF lookup robot pose failed (%s -> %s).",
                                 child_frame_.c_str(), target_frame.c_str());
            return;
        }

        // 2) 從 global_path 取得「前向方向」
        std::pair<double, double> dir = estimateForwardDirOnPath(robot_in_target, latest_global_path_, lookahead_dist_on_global_);
        const double dn = std::hypot(dir.first, dir.second);
        if (dn < 1e-9)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[local_straight] direction degenerated, skip.");
            return;
        }
        dir.first /= dn;
        dir.second /= dn;

        // 3) 往前生成理想終點
        double s_hint = have_last_projection_s_ ? last_projection_s_ : 0.0;
        double s_robota = projectWithForwardPreference(latest_global_path_,
                                                       robot_in_target.pose.position,
                                                       s_hint,
                                                       have_last_projection_s_);
        last_projection_s_ = s_robota;
        have_last_projection_s_ = true;
        const double total_len = pathTotalLength(latest_global_path_);
        const double s_target = std::min(total_len, s_robota + max_local_len_);
        geometry_msgs::msg::Point desired_end = pointAtSAlongGlobal(latest_global_path_, s_target);
        desired_end.z = robot_in_target.pose.position.z;

        // 4) 依 costmap 檢查碰撞/間隙
        geometry_msgs::msg::Point safe_end = adjustEndByObstacles(robot_in_target.pose.position, desired_end);

        // 4.5) 距離判斷
        geometry_msgs::msg::Point final_end;
        const geometry_msgs::msg::Point &global_goal = latest_global_path_.poses.back().pose.position;
        const double dist_to_goal = std::hypot(global_goal.x - robot_in_target.pose.position.x,
                                               global_goal.y - robot_in_target.pose.position.y);
        if (dist_to_goal <= max_local_len_)
        {
            final_end = global_goal; 
        }
        else
        {
            final_end = safe_end; 
        }

        // 5) 輸出 Local Path
        nav_msgs::msg::Path local_path;
        local_path.header.stamp = this->now();
        local_path.header.frame_id = target_frame;

        double yaw = std::atan2(final_end.y - robot_in_target.pose.position.y,
                                final_end.x - robot_in_target.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        geometry_msgs::msg::PoseStamped s, e;
        s.header = local_path.header;
        s.pose.position.x = robot_in_target.pose.position.x;
        s.pose.position.y = robot_in_target.pose.position.y;
        s.pose.position.z = 0.0;
        s.pose.orientation = tf2::toMsg(q);

        e.header = local_path.header;
        e.pose.position.x = final_end.x;
        e.pose.position.y = final_end.y;
        e.pose.position.z = 0.0;
        e.pose.orientation = tf2::toMsg(q);

        local_path.poses.clear();
        local_path.poses.push_back(s);
        local_path.poses.push_back(e);

        double s_robot = s_robota;
        double s_end = 0.0;
        projectAlongGlobal(latest_global_path_, e.pose.position, s_end);
        const double path_len = std::max(0.0, s_end - s_robot);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[local_straight] local_path length (along global) = %.3f m", path_len);

        // Debug visualization
        {
            visualization_msgs::msg::MarkerArray arr;
            geometry_msgs::msg::Point proj_pt = pointAtSAlongGlobal(latest_global_path_, s_robot);

            visualization_msgs::msg::Marker m_proj;
            m_proj.header.frame_id = target_frame;
            m_proj.header.stamp = this->now();
            m_proj.ns = "local_straight_debug";
            m_proj.id = 1;
            m_proj.type = visualization_msgs::msg::Marker::SPHERE;
            m_proj.action = visualization_msgs::msg::Marker::ADD;
            m_proj.pose.orientation.w = 1.0;
            m_proj.pose.position = proj_pt;
            m_proj.scale.x = m_proj.scale.y = m_proj.scale.z = 0.12;
            m_proj.color.r = 0.0f;
            m_proj.color.g = 1.0f;
            m_proj.color.b = 0.0f;
            m_proj.color.a = 0.9f;
            m_proj.lifetime = rclcpp::Duration::from_seconds(0.2); // [修正] Duration寫法
            arr.markers.push_back(m_proj);

            visualization_msgs::msg::Marker m_des = m_proj;
            m_des.id = 2;
            m_des.pose.position = desired_end;
            m_des.color.r = 0.2f;
            m_des.color.g = 0.6f;
            m_des.color.b = 1.0f;
            m_des.color.a = 0.95f;
            arr.markers.push_back(m_des);

            visualization_msgs::msg::Marker m_tri;
            m_tri.header.frame_id = target_frame;
            m_tri.header.stamp = this->now();
            m_tri.ns = "local_straight_debug";
            m_tri.id = 10;
            m_tri.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m_tri.action = visualization_msgs::msg::Marker::ADD;
            m_tri.pose.orientation.w = 1.0;
            m_tri.scale.x = 0.03;
            m_tri.color.r = 1.0f;
            m_tri.color.g = 0.2f;
            m_tri.color.b = 0.2f;
            m_tri.color.a = 0.9f;
            m_tri.lifetime = rclcpp::Duration::from_seconds(0.2);
            m_tri.points.clear();
            m_tri.points.push_back(robot_in_target.pose.position);
            m_tri.points.push_back(proj_pt);
            m_tri.points.push_back(desired_end);
            m_tri.points.push_back(robot_in_target.pose.position);
            arr.markers.push_back(m_tri);

            debug_marker_pub_->publish(arr);
        }

        if (show_global_end_arrow_ && have_global_path_ && latest_global_path_.poses.size() >= 1)
        {
            const auto &end_ps = latest_global_path_.poses.back();
            double yaw = tf2::getYaw(end_ps.pose.orientation);

            geometry_msgs::msg::Point A = end_ps.pose.position; 
            geometry_msgs::msg::Point B = A;
            B.x += arrow_len_ * std::cos(yaw);
            B.y += arrow_len_ * std::sin(yaw);

            visualization_msgs::msg::Marker mk;
            mk.header.frame_id = latest_global_path_.header.frame_id;
            mk.header.stamp = this->now();
            mk.ns = "global_path_end";
            mk.id = 0;
            mk.type = visualization_msgs::msg::Marker::ARROW;
            mk.action = visualization_msgs::msg::Marker::ADD;

            mk.points.clear();
            mk.points.push_back(A);
            mk.points.push_back(B);

            mk.scale.x = arrow_shaft_d_;
            mk.scale.y = arrow_head_d_;
            mk.scale.z = arrow_head_len_;

            mk.color.r = 0.0f;
            mk.color.g = 1.0f;
            mk.color.b = 0.0f;
            mk.color.a = 1.0f;
            mk.lifetime = rclcpp::Duration::from_seconds(0.01); // 10ms

            global_end_marker_pub_->publish(mk);
        }

        local_path_pub_->publish(local_path);
        global_path_end_pub->publish(latest_global_path_.poses.back());
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "pub local path");
    }

    // ===== 工具：TF 抓取位姿 =====
    bool lookupPose(const std::string &from_frame, const std::string &to_frame, geometry_msgs::msg::PoseStamped &out_pose)
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            // [修正] 使用 tf2::TimePointZero 取代 ros::Time(0)，且 tf_buffer_ 為 unique_ptr 需用 -> 存取
            tf = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero, std::chrono::milliseconds(100));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[local_straight] %s", ex.what());
            return false;
        }

        out_pose.header = tf.header;
        out_pose.pose.position.x = tf.transform.translation.x;
        out_pose.pose.position.y = tf.transform.translation.y;
        out_pose.pose.position.z = tf.transform.translation.z;
        out_pose.pose.orientation = tf.transform.rotation;
        return true;
    }

    // ===== 工具：估計 global_path 的前向方向 =====
    static double sqrDist(const geometry_msgs::msg::PoseStamped &a, const geometry_msgs::msg::PoseStamped &b)
    {
        const double dx = a.pose.position.x - b.pose.position.x;
        const double dy = a.pose.position.y - b.pose.position.y;
        return dx * dx + dy * dy;
    }

    std::pair<double, double> estimateForwardDirOnPath(const geometry_msgs::msg::PoseStamped &robot,
                                                       const nav_msgs::msg::Path &gpath,
                                                       double lookahead_dist)
    {
        int idx_min = -1;
        double best = std::numeric_limits<double>::infinity();
        for (int i = 0; i < (int)gpath.poses.size(); ++i)
        {
            double d = sqrDist(robot, gpath.poses[i]);
            if (d < best)
            {
                best = d;
                idx_min = i;
            }
        }
        if (idx_min < 0)
            return {0.0, 0.0};

        int j = idx_min;
        double acc = 0.0;
        while (j + 1 < (int)gpath.poses.size())
        {
            double dx = gpath.poses[j + 1].pose.position.x - gpath.poses[j].pose.position.x;
            double dy = gpath.poses[j + 1].pose.position.y - gpath.poses[j].pose.position.y;
            double ds = std::hypot(dx, dy);
            acc += ds;
            if (acc >= lookahead_dist)
                break;
            ++j;
        }

        if (j + 1 >= (int)gpath.poses.size())
        {
            if ((int)gpath.poses.size() >= 2)
            {
                const auto &p0 = gpath.poses[gpath.poses.size() - 2].pose.position;
                const auto &p1 = gpath.poses.back().pose.position;
                return {p1.x - p0.x, p1.y - p0.y};
            }
            const auto &pend = gpath.poses.back().pose.position;
            return {pend.x - robot.pose.position.x, pend.y - robot.pose.position.y};
        }

        const auto &pa = gpath.poses[j].pose.position;
        const auto &pb = gpath.poses[j + 1].pose.position;
        return {pb.x - pa.x, pb.y - pa.y};
    }

    double projectWithForwardPreference(const nav_msgs::msg::Path &gpath,
                                        const geometry_msgs::msg::Point &p,
                                        double hint_s,
                                        bool has_hint) const
    {
        const double backtrack_slack = std::max(1e-3, proj_backtrack_slack_);
        const double corner_slack = std::max(1e-3, proj_corner_slack_);

        double best_cost = std::numeric_limits<double>::infinity();
        double best_s = 0.0;

        const size_t N = gpath.poses.size();
        if (N == 0) return 0.0;
        if (N == 1) return 0.0;

        double acc = 0.0;
        for (size_t i = 0; i + 1 < N; ++i)
        {
            const auto &a = gpath.poses[i].pose.position;
            const auto &b = gpath.poses[i + 1].pose.position;
            const double vx = b.x - a.x;
            const double vy = b.y - a.y;
            const double seg_len2 = vx * vx + vy * vy;
            if (seg_len2 < 1e-12) continue;
            const double seg_len = std::sqrt(seg_len2);

            const double wx = p.x - a.x;
            const double wy = p.y - a.y;
            const double t_raw = (wx * vx + wy * vy) / seg_len2;
            double t = t_raw;
            if (t < 0.0) t = 0.0;
            else if (t > 1.0) t = 1.0;

            const double proj_x = a.x + t * vx;
            const double proj_y = a.y + t * vy;
            const double dx = p.x - proj_x;
            const double dy = p.y - proj_y;
            const double d2 = dx * dx + dy * dy;

            const double s_candidate = acc + t * seg_len;
            double cost = d2;

            if (has_hint)
            {
                const double back = hint_s - s_candidate;
                if (back > 0.0)
                {
                    const double penalty = back / backtrack_slack;
                    cost += penalty * penalty;
                }
            }
            const double ahead = std::max(0.0, t_raw - 1.0) * seg_len;
            if (ahead > 0.0)
            {
                const double penalty = ahead / corner_slack;
                cost += penalty * penalty;
            }
            const double before = std::max(0.0, -t_raw) * seg_len;
            if (before > 0.0)
            {
                const double penalty = before / corner_slack;
                cost += penalty * penalty;
            }

            if (cost < best_cost)
            {
                best_cost = cost;
                best_s = s_candidate;
            }

            acc += seg_len;
        }

        return best_s;
    }

    void projectAlongGlobal(const nav_msgs::msg::Path &gpath,
                            const geometry_msgs::msg::Point &p,
                            double &out_s) const
    {
        out_s = 0.0;
        const size_t N = gpath.poses.size();
        if (N == 0 || N == 1) return;

        double best_d2 = std::numeric_limits<double>::infinity();
        double best_s = 0.0;
        double acc = 0.0;
        for (size_t i = 0; i + 1 < N; ++i)
        {
            const auto &a = gpath.poses[i].pose.position;
            const auto &b = gpath.poses[i + 1].pose.position;
            const double vx = b.x - a.x;
            const double vy = b.y - a.y;
            const double wx = p.x - a.x;
            const double wy = p.y - a.y;
            const double seg_len2 = vx * vx + vy * vy;
            double t = 0.0;
            if (seg_len2 > 1e-12)
            {
                t = (wx * vx + wy * vy) / seg_len2;
                if (t < 0.0) t = 0.0;
                else if (t > 1.0) t = 1.0;
            }
            const double proj_x = a.x + t * vx;
            const double proj_y = a.y + t * vy;
            const double dx = p.x - proj_x;
            const double dy = p.y - proj_y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)
            {
                best_d2 = d2;
                const double seg_len = std::sqrt(seg_len2);
                best_s = acc + t * seg_len;
            }
            acc += std::sqrt(seg_len2);
        }
        out_s = best_s;
    }

    double pathTotalLength(const nav_msgs::msg::Path &gpath) const
    {
        double acc = 0.0;
        for (size_t i = 1; i < gpath.poses.size(); ++i)
        {
            const auto &a = gpath.poses[i - 1].pose.position;
            const auto &b = gpath.poses[i].pose.position;
            acc += std::hypot(b.x - a.x, b.y - a.y);
        }
        return acc;
    }

    geometry_msgs::msg::Point pointAtSAlongGlobal(const nav_msgs::msg::Path &gpath, double s) const
    {
        geometry_msgs::msg::Point out;
        const size_t N = gpath.poses.size();
        if (N == 0) { out.x=out.y=out.z=0.0; return out; }
        if (N == 1) return gpath.poses.front().pose.position;

        double total = 0.0;
        for (size_t i = 1; i < N; ++i)
        {
            const auto &a = gpath.poses[i - 1].pose.position;
            const auto &b = gpath.poses[i].pose.position;
            total += std::hypot(b.x - a.x, b.y - a.y);
        }
        if (s <= 0.0) return gpath.poses.front().pose.position;
        if (s >= total) return gpath.poses.back().pose.position;

        double acc = 0.0;
        for (size_t i = 0; i + 1 < N; ++i)
        {
            const auto &a = gpath.poses[i].pose.position;
            const auto &b = gpath.poses[i + 1].pose.position;
            const double seg = std::hypot(b.x - a.x, b.y - a.y);
            if (acc + seg >= s)
            {
                const double t = (s - acc) / std::max(seg, 1e-12);
                out.x = a.x + t * (b.x - a.x);
                out.y = a.y + t * (b.y - a.y);
                out.z = 0.0;
                return out;
            }
            acc += seg;
        }
        return gpath.poses.back().pose.position;
    }

    // ===== OccupancyGrid 索引/查詢 =====
    bool worldToMapIndex(const nav_msgs::msg::OccupancyGrid &grid,
                         double x_cmap, double y_cmap, int &mx, int &my) const
    {
        tf2::Transform T_origin;
        tf2::Quaternion q;
        q.setX(grid.info.origin.orientation.x);
        q.setY(grid.info.origin.orientation.y);
        q.setZ(grid.info.origin.orientation.z);
        q.setW(grid.info.origin.orientation.w);
        T_origin.setOrigin(tf2::Vector3(grid.info.origin.position.x, grid.info.origin.position.y, 0));
        T_origin.setRotation(q);

        tf2::Vector3 p_cmap(x_cmap, y_cmap, 0);
        tf2::Vector3 p_grid = T_origin.inverse() * p_cmap; 

        const double res = grid.info.resolution;
        mx = (int)std::floor(p_grid.x() / res);
        my = (int)std::floor(p_grid.y() / res);

        if (mx < 0 || my < 0 || mx >= (int)grid.info.width || my >= (int)grid.info.height)
            return false;
        return true;
    }

    inline bool isObstacleCell(int8_t v) const
    {
        if (treat_minus128_as_obstacle_ && v == -128)
            return true; 
        return v >= obstacle_threshold_;
    }

    bool occupiedWithinRadius(const nav_msgs::msg::OccupancyGrid &grid,
                              double x_cmap, double y_cmap, double radius) const
    {
        int cx, cy;
        if (!worldToMapIndex(grid, x_cmap, y_cmap, cx, cy))
            return false; 

        const double res = grid.info.resolution;
        const int r = std::max(1, (int)std::ceil(radius / res));

        for (int dy = -r; dy <= r; ++dy)
        {
            for (int dx = -r; dx <= r; ++dx)
            {
                int mx = cx + dx;
                int my = cy + dy;
                if (mx < 0 || my < 0 || mx >= (int)grid.info.width || my >= (int)grid.info.height)
                    continue;
                const double wx = (dx * res);
                const double wy = (dy * res);
                if (std::hypot(wx, wy) > radius)
                    continue;
                int idx = my * grid.info.width + mx;
                if (idx >= 0 && idx < (int)grid.data.size())
                {
                    if (isObstacleCell(grid.data[idx]))
                        return true;
                }
            }
        }
        return false;
    }

    geometry_msgs::msg::Point adjustEndByObstacles(const geometry_msgs::msg::Point &start_target_frame,
                                                   const geometry_msgs::msg::Point &desired_end_target_frame)
    {
        geometry_msgs::msg::Point out = desired_end_target_frame;
        if (!have_costmap_)
            return out;

        const std::string cmap_frame = latest_costmap_.header.frame_id;

        geometry_msgs::msg::Point start_cmap = transformPoint(start_target_frame, latest_global_path_.header.frame_id, cmap_frame);
        geometry_msgs::msg::Point end_cmap = transformPoint(desired_end_target_frame, latest_global_path_.header.frame_id, cmap_frame);

        const double L = std::hypot(end_cmap.x - start_cmap.x, end_cmap.y - start_cmap.y);
        if (L < 1e-9)
            return out;

        const double step_m = std::max(0.01, std::min(collision_check_step_, L));
        const int K = std::max(1, (int)std::floor(L / step_m));

        auto pointAt = [&](double a)
        {
            geometry_msgs::msg::Point p;
            p.x = start_cmap.x + a * (end_cmap.x - start_cmap.x);
            p.y = start_cmap.y + a * (end_cmap.y - start_cmap.y);
            p.z = 0.0;
            return p;
        };

        bool in_block = false;
        bool hit = false;
        bool cross = false;
        double exit_a = -1.0;

        for (int i = 0; i <= K; ++i)
        {
            const double a = (double)i / (double)K;
            const auto p = pointAt(a);
            const bool occ = occupiedWithinRadius(latest_costmap_, p.x, p.y, clearance_);

            if (!in_block && occ)
            {
                in_block = true;
                hit = true;
            }
            else if (in_block && !occ)
            {
                exit_a = a;
                cross = true;
                break; 
            }
        }

        if (cross)
            return desired_end_target_frame;

        if (hit && cross_obstacle_extend_ > 1e-6)
        {
            const double total_len = L + std::max(0.0, cross_obstacle_extend_);
            const int K_ext = std::max(1, (int)std::floor(total_len / step_m));
            for (int i = K + 1; i <= K_ext; ++i)
            {
                const double s = std::min(total_len, i * step_m);
                const double a = s / L;
                const auto p = pointAt(a);
                const bool occ = occupiedWithinRadius(latest_costmap_, p.x, p.y, clearance_);
                if (in_block && !occ)
                {
                    exit_a = a;
                    cross = true;
                    break;
                }
            }
            if (cross)
            {
                const auto cross_cmap = pointAt(exit_a);
                out = transformPoint(cross_cmap, cmap_frame, latest_global_path_.header.frame_id);
                return out;
            }
        }

        return out;
    }

    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point &pin,
                                             const std::string &from_frame,
                                             const std::string &to_frame)
    {
        geometry_msgs::msg::PointStamped in_ps, out_ps;
        in_ps.header.frame_id = from_frame;
        // [修正] 使用 rclcpp::Time(0) 或 this->now()，tf2::TimePointZero 用於查詢
        in_ps.header.stamp = rclcpp::Time(0);
        in_ps.point = pin;

        try
        {
            // [修正] tf_buffer_ 是 unique_ptr，使用 ->，且使用 tf2::TimePointZero
            geometry_msgs::msg::TransformStamped T = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero, std::chrono::milliseconds(100));
            tf2::doTransform(in_ps, out_ps, T);
            return out_ps.point;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[local_straight] %s (return original point)", ex.what());
            return pin; 
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPathNode>());
    rclcpp::shutdown();
    return 0;
}
