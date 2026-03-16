#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <mutex>
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

struct LineSegment
{
    double x1, y1;
    double x2, y2;
    double len_sq;
};

class RegularPointsNode : public rclcpp::Node
{
public:
    RegularPointsNode() : Node("regular_points_node")
    {
        // 1. 宣告參數
        this->declare_parameter<double>("clr_obs", 0.1);
        this->declare_parameter<double>("max_range", 2.0); // 左右展開寬度
        this->declare_parameter<double>("per_dis", 0.1);   // 點間距
        this->declare_parameter<double>("gap_dist_limit", 0.8);
        this->declare_parameter<double>("gap_filter_width", 0.3);
        this->declare_parameter<double>("obs_proximity_limit", 0.6);
        this->declare_parameter<std::string>("child_frame", "base_link");

        clr_obs_ = this->get_parameter("clr_obs").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        per_dis_ = this->get_parameter("per_dis").as_double();
        gap_dist_limit_ = this->get_parameter("gap_dist_limit").as_double();
        gap_filter_width_ = this->get_parameter("gap_filter_width").as_double();
        obs_proximity_limit_ = this->get_parameter("obs_proximity_limit").as_double();
        child_frame_ = this->get_parameter("child_frame").as_string();

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

        obs_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
            "/wall_and_dynamic_obs_map", 1,
            std::bind(&RegularPointsNode::obsCb, this, std::placeholders::_1),
            sub_opts);
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 1,
            std::bind(&RegularPointsNode::costmapCb, this, std::placeholders::_1),
            sub_opts);

        // 5. 發布
        left_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/regular_points_left", 1);
        right_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/regular_points_right", 1);
        pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/regular_points", 1);

        // 6. Timer (加入 callback_group_)
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&RegularPointsNode::tick, this),
            callback_group_);

        RCLCPP_INFO(this->get_logger(), "RegularPointsNode started with ReentrantCallbackGroup.");
    }

private:
    // ROS 介面
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obs_sub_;
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
    nav_msgs::msg::Path local_path_temp_;
    std::vector<costmap_converter_msgs::msg::ObstacleMsg> obstacles_;
    bool has_local_path_ = false;

    // 參數變數
    double clr_obs_, max_range_, per_dis_;
    double gap_dist_limit_, gap_filter_width_, obs_proximity_limit_;

    // 計算用變數 (不需要 Mutex，因為只在 generate 內部使用副本)
    std::vector<LineSegment> active_narrow_gaps_;

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

    void obsCb(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        // 即使是空陣列也要更新，代表障礙物消失
        std::lock_guard<std::mutex> lk(mtx_);
        obstacles_ = msg->obstacles;
    }

    void costmapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->data.empty()) return;
        std::lock_guard<std::mutex> lk(mtx_);

        
    }

    // ---------------- Core Logic ----------------

    void tick()
    {
        nav_msgs::msg::Path path_copy;
        std::vector<costmap_converter_msgs::msg::ObstacleMsg> obs_copy;
        bool should_gen = false;
        std::string current_target_frame;

        // 1. 快速複製數據 (Minimize Lock Time)
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (has_local_path_ && !local_path_temp_.poses.empty())
            {
                path_copy = local_path_temp_;
                obs_copy = obstacles_;
                current_target_frame = target_frame_;
                should_gen = true;
            }
        }

        // 2. 執行計算 (No Lock here)
        if (should_gen)
        {
            generate(path_copy, obs_copy, current_target_frame);
        }
    }

    void generate(const nav_msgs::msg::Path &path_data,
                  const std::vector<costmap_converter_msgs::msg::ObstacleMsg> &obs_data,
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

        // C. 更新窄縫列表 (使用當前的障礙物副本)
        updateNarrowGaps(obs_data);

        // D. 準備輸出容器
        geometry_msgs::msg::PoseArray out_left, out_right, out_all;
        out_left.header.frame_id = frame_id;
        out_left.header.stamp = this->now();
        out_right.header = out_left.header;
        out_all.header = out_left.header;

        double accumulated_dist = 0.0;

        // E. 沿路徑生成點
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
                             out_left, out_right, out_all, obs_data);

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
                      const std::vector<costmap_converter_msgs::msg::ObstacleMsg> &obs_data)
    {
        // 左右展開
        for (double offset = -max_range_; offset <= max_range_ + 1e-9; offset += per_dis_)
        {
            double px = cx + offset * nx;
            double py = cy + offset * ny;

            // 1. 障礙物過濾
            if (isObstacle(px, py, obs_data))
                continue;

            // 2. 窄縫過濾
            if (isInsideNarrowGap(px, py))
                continue;

            // 3. 背後點過濾 (寬鬆檢查：-0.5m)
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

    void updateNarrowGaps(const std::vector<costmap_converter_msgs::msg::ObstacleMsg> &current_obs)
    {
        active_narrow_gaps_.clear();
        if (current_obs.size() < 2)
            return;

        for (size_t i = 0; i < current_obs.size(); ++i)
        {
            // 只檢查多邊形障礙物的第一個點(簡化版)，若需精確需檢查邊緣距離
            if (current_obs[i].polygon.points.empty())
                continue;

            for (size_t j = i + 1; j < current_obs.size(); ++j)
            {
                if (current_obs[j].polygon.points.empty())
                    continue;

                double x1 = current_obs[i].polygon.points[0].x;
                double y1 = current_obs[i].polygon.points[0].y;
                double x2 = current_obs[j].polygon.points[0].x;
                double y2 = current_obs[j].polygon.points[0].y;

                double dist_sq = std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2);
                double dist = std::sqrt(dist_sq);

                // 估算兩障礙物邊緣的間隙：中心距 - 半徑1 - 半徑2
                // (前提是 obstacle msg 填寫了正確的 radius，若無 radius 則此算法需調整)
                // 若沒有 radius，則假設為點障礙物
                double r1 = current_obs[i].radius > 0 ? current_obs[i].radius : 0.0;
                double r2 = current_obs[j].radius > 0 ? current_obs[j].radius : 0.0;

                double gap = dist - r1 - r2;

                if (gap < gap_dist_limit_ && gap > 0.05) // 太小的gap可能是重疊，視需求過濾
                {
                    active_narrow_gaps_.push_back({x1, y1, x2, y2, dist_sq});
                }
            }
        }
    }

    bool isObstacle(double x, double y, const std::vector<costmap_converter_msgs::msg::ObstacleMsg> &obs_data) const
    {
        if (obs_data.empty())
            return false;

        for (const auto &obs : obs_data)
        {
            if (obs.polygon.points.empty())
                continue;

            double ox = obs.polygon.points[0].x;
            double oy = obs.polygon.points[0].y;
            double dist = std::hypot(x - ox, y - oy);

            // 使用障礙物半徑 + 安全距離
            double safe_r = (obs.radius > 0 ? obs.radius : 0.1) + clr_obs_;

            if (dist < safe_r)
                return true;
        }
        return false;
    }

    bool isInsideNarrowGap(double x, double y) const
    {
        if (active_narrow_gaps_.empty())
            return false;

        double half_w_sq = std::pow(gap_filter_width_ / 2.0, 2);

        for (const auto &seg : active_narrow_gaps_)
        {
            // 計算點到線段的投影
            double apx = x - seg.x1;
            double apy = y - seg.y1;
            double abx = seg.x2 - seg.x1;
            double aby = seg.y2 - seg.y1;

            // t 是投影點在線段上的比例 (0~1)
            double t = (apx * abx + apy * aby) / seg.len_sq;

            if (t >= 0.0 && t <= 1.0)
            {
                double closest_x = seg.x1 + t * abx;
                double closest_y = seg.y1 + t * aby;

                // 檢查點到線段的垂直距離平方
                if (std::pow(x - closest_x, 2) + std::pow(y - closest_y, 2) < half_w_sq)
                {
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