#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/time.h>

#include <mutex>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class RegularPointsNode : public rclcpp::Node
{
public:
    RegularPointsNode() : Node("regular_points_node")
    {
        // 1. 宣告參數
        this->declare_parameter<double>("clr_obs", 0.1);
        this->declare_parameter<double>("max_range", 3.0); // 左右展開寬度
        this->declare_parameter<double>("per_dis", 0.1);   // 點間距
        this->declare_parameter<std::string>("child_frame", "base_link");
        this->declare_parameter<std::string>("costmap_topic", "/costmap");

        clr_obs_ = this->get_parameter("clr_obs").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        per_dis_ = this->get_parameter("per_dis").as_double();
        child_frame_ = this->get_parameter("child_frame").as_string();
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();

        // 2. 初始化 Callback Group (關鍵：使用 Reentrant 避免死鎖)
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = callback_group_;

        // 3. TF 初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 4. 訂閱 (加入 sub_opts)
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path", 1,
            std::bind(&RegularPointsNode::localPathCb, this, std::placeholders::_1),
            sub_opts);

        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 1,
            std::bind(&RegularPointsNode::costmapCb, this, std::placeholders::_1),
            sub_opts);

        // 5. 發布
        left_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/regular_points_left", 1);
        right_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/regular_points_right", 1);
        pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/regular_points", 1);

        // 6. Timer (加入 callback_group_)
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&RegularPointsNode::tick, this),
            callback_group_);

        RCLCPP_INFO(this->get_logger(), "RegularPointsNode started with ReentrantCallbackGroup.");
    }

private:
    // ROS 介面
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_pub_, right_pub_, pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_; 

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // 數據緩衝區 (受 Mutex 保護)
    std::mutex mtx_;
    std::string target_frame_ = "map";
    std::string child_frame_;
    std::string costmap_topic_;
    nav_msgs::msg::Path local_path_temp_;
    bool has_local_path_ = false;
    nav_msgs::msg::OccupancyGrid costmap_;
    bool has_costmap_ = false;

    // 參數變數
    double clr_obs_, max_range_, per_dis_;

    // ---------------- Callbacks ----------------

    void localPathCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
            return;
        std::lock_guard<std::mutex> lk(mtx_);
        local_path_temp_ = *msg;
        target_frame_ = msg->header.frame_id;
        has_local_path_ = true;
    }

    void costmapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        costmap_ = *msg;
        has_costmap_ = costmap_.info.width > 0 && costmap_.info.height > 0 && !costmap_.data.empty();
    }

    // ---------------- Core Logic ----------------

    void tick()
    {
        nav_msgs::msg::Path path_copy;
        nav_msgs::msg::OccupancyGrid costmap_copy;
        bool has_costmap = false;
        bool should_gen = false;
        std::string current_target_frame;

        // 1. 快速複製數據 (Minimize Lock Time)
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (has_local_path_ && !local_path_temp_.poses.empty())
            {
                path_copy = local_path_temp_;
                costmap_copy = costmap_;
                has_costmap = has_costmap_;
                current_target_frame = target_frame_;
                should_gen = true;
            }
        }

        // 2. 執行計算 (No Lock here)
        if (should_gen)
        {
            generate(path_copy, costmap_copy, has_costmap, current_target_frame);
        }
    }

    void generate(const nav_msgs::msg::Path &path_data,
                  const nav_msgs::msg::OccupancyGrid &costmap_data,
                  bool has_costmap,
                  const std::string &frame_id)
    {
        // A. 獲取 Robot 位置 (TF)
        double robot_x, robot_y;
        try
        {
            // 使用 timeout 避免瞬間拿不到資料就報錯，且因為是 Reentrant，這裡等待不會卡死 TF callback
            if (!tf_buffer_->canTransform(frame_id, child_frame_, tf2::TimePointZero, 50ms))
            {
                // 只印一次警告避免洗版
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Wait for TF %s -> %s timeout", frame_id.c_str(), child_frame_.c_str());
                return;
            }
            geometry_msgs::msg::TransformStamped t =
                tf_buffer_->lookupTransform(frame_id, child_frame_, tf2::TimePointZero);
            robot_x = t.transform.translation.x;
            robot_y = t.transform.translation.y;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Error: %s", ex.what());
            return;
        }

        const auto &poses = path_data.poses;
        if (poses.size() < 2)
            return;

        // B. 尋找最近的路徑點 (Start Index)
        size_t start_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t search_limit = std::min((size_t)200, poses.size());

        for (size_t i = 0; i < search_limit; ++i)
        {
            double dx = poses[i].pose.position.x - robot_x;
            double dy = poses[i].pose.position.y - robot_y;
            double dsq = dx * dx + dy * dy;
            if (dsq < min_dist_sq)
            {
                min_dist_sq = dsq;
                start_idx = i;
            }
        }

        bool can_use_costmap = has_costmap && !costmap_data.data.empty();
        bool need_tf = false;
        tf2::Transform path_to_costmap;
        path_to_costmap.setIdentity();
        if (can_use_costmap)
        {
            if (costmap_data.header.frame_id != frame_id)
            {
                need_tf = true;
                if (!tf_buffer_->canTransform(costmap_data.header.frame_id, frame_id, tf2::TimePointZero, 50ms))
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Wait for TF %s -> %s timeout", frame_id.c_str(), costmap_data.header.frame_id.c_str());
                    can_use_costmap = false;
                }
                else
                {
                    geometry_msgs::msg::TransformStamped t =
                        tf_buffer_->lookupTransform(costmap_data.header.frame_id, frame_id, tf2::TimePointZero);
                    path_to_costmap = tf2::Transform(tf2::Quaternion(
                                                         t.transform.rotation.x,
                                                         t.transform.rotation.y,
                                                         t.transform.rotation.z,
                                                         t.transform.rotation.w),
                                                     tf2::Vector3(t.transform.translation.x,
                                                                  t.transform.translation.y,
                                                                  t.transform.translation.z));
                }
            }
        }

        // C. 準備輸出容器
        geometry_msgs::msg::PoseArray out_left, out_right, out_all;
        out_left.header.frame_id = frame_id;
        out_left.header.stamp = this->now();
        out_right.header = out_left.header;
        out_all.header = out_left.header;

        double accumulated_dist = 0.0;

        // D. 沿路徑生成點
        for (size_t i = start_idx; i < poses.size() - 1; ++i)
        {
            double x1 = poses[i].pose.position.x;
            double y1 = poses[i].pose.position.y;
            double x2 = poses[i + 1].pose.position.x;
            double y2 = poses[i + 1].pose.position.y;

            double dx = x2 - x1;
            double dy = y2 - y1;
            double seg_len = std::hypot(dx, dy);

            if (seg_len < 1e-3)
                continue;

            double dir_x = dx / seg_len;
            double dir_y = dy / seg_len;
            double nx = -dir_y; // 法向量 (向右旋轉90度，視座標系定義而定，此處假設為右手)
            double ny = dir_x;

            auto quat = yawToQuat(std::atan2(dy, dx));

            // 計算從這段線段的哪裡開始鋪點
            double current_pos_on_seg = (per_dis_ - accumulated_dist);
            if (current_pos_on_seg < 0)
                current_pos_on_seg = 0;

            while (current_pos_on_seg <= seg_len)
            {
                double cx = x1 + current_pos_on_seg * dir_x;
                double cy = y1 + current_pos_on_seg * dir_y;

                expandPoints(cx, cy, nx, ny, quat, robot_x, robot_y, dir_x, dir_y,
                             out_left, out_right, out_all, costmap_data, can_use_costmap, need_tf, path_to_costmap);

                current_pos_on_seg += per_dis_;
            }
            accumulated_dist = seg_len - (current_pos_on_seg - per_dis_);
        }

        left_pub_->publish(out_left);
        right_pub_->publish(out_right);
        pub_->publish(out_all);
    }

    void expandPoints(double cx, double cy, double nx, double ny,
                      const geometry_msgs::msg::Quaternion &quat,
                      double rx, double ry, double dx, double dy,
                      geometry_msgs::msg::PoseArray &left,
                      geometry_msgs::msg::PoseArray &right,
                      geometry_msgs::msg::PoseArray &all,
                      const nav_msgs::msg::OccupancyGrid &costmap_data,
                      bool can_use_costmap,
                      bool need_tf,
                      const tf2::Transform &path_to_costmap)
    {
        // 左右展開
        for (double offset = -max_range_; offset <= max_range_ + 1e-9; offset += per_dis_)
        {
            double px = cx + offset * nx;
            double py = cy + offset * ny;

            // 1. 障礙物過濾
            if (isObstacleCostmap(px, py, costmap_data, can_use_costmap, need_tf, path_to_costmap))
                continue;

            // 2. 背後點過濾 (寬鬆檢查：-0.5m)
            // dot product < 0 代表在機器人後方，給予一點寬容度避免轉彎時斷裂
            if ((px - rx) * dx + (py - ry) * dy < -0.5)
                continue;

            geometry_msgs::msg::Pose p;
            p.position.x = px;
            p.position.y = py;
            p.position.z = 0.0;
            p.orientation = quat;

            bool added = false;
            // 判斷左右 (offset 0 為中心)
            if (std::abs(offset) < 0.001)
            {
                left.poses.push_back(p);
                right.poses.push_back(p);
                added = true;
            }
            else if (offset > 0.0) // 假設 nx 指向左邊
            {
                left.poses.push_back(p);
                added = true;
            }
            else // offset < 0
            {
                right.poses.push_back(p);
                added = true;
            }

            if (added)
                all.poses.push_back(p);
        }
    }

    bool isObstacleCostmap(double x, double y,
                           const nav_msgs::msg::OccupancyGrid &grid,
                           bool can_use_costmap,
                           bool need_tf,
                           const tf2::Transform &path_to_costmap) const
    {
        if (!can_use_costmap || grid.data.empty())
            return false;

        double gx = x;
        double gy = y;
        if (need_tf)
        {
            tf2::Vector3 p = path_to_costmap * tf2::Vector3(x, y, 0.0);
            gx = p.x();
            gy = p.y();
        }
        return occupiedWithinRadius(grid, gx, gy, std::max(0.0, clr_obs_));
    }

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
        return v >= 99;
    }

    bool occupiedWithinRadius(const nav_msgs::msg::OccupancyGrid &grid,
                              double x_cmap, double y_cmap, double radius) const
    {
        int cx, cy;
        if (!worldToMapIndex(grid, x_cmap, y_cmap, cx, cy))
            return false;

        const int idx = cy * grid.info.width + cx;
        if (radius <= 1e-6)
        {
            if (idx >= 0 && idx < (int)grid.data.size())
                return isObstacleCell(grid.data[idx]);
            return false;
        }

        const double res = grid.info.resolution;
        const int r = std::max(1, (int)std::ceil(radius / res));
        const double r_sq = radius * radius;

        for (int dy = -r; dy <= r; ++dy)
        {
            for (int dx = -r; dx <= r; ++dx)
            {
                int mx = cx + dx;
                int my = cy + dy;
                if (mx < 0 || my < 0 || mx >= (int)grid.info.width || my >= (int)grid.info.height)
                    continue;
                const double wx = dx * res;
                const double wy = dy * res;
                if ((wx * wx + wy * wy) > r_sq)
                    continue;
                int cidx = my * grid.info.width + mx;
                if (cidx >= 0 && cidx < (int)grid.data.size())
                {
                    if (isObstacleCell(grid.data[cidx]))
                        return true;
                }
            }
        }
        return false;
    }

    geometry_msgs::msg::Quaternion yawToQuat(double yaw) const
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RegularPointsNode>();

    // 使用 MultiThreadedExecutor 以支援並行 Callback
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
