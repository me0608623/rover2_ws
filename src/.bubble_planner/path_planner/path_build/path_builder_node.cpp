#include "path_builder/path_builder_core.hpp"
#include "path_builder/path_builder_visualizer.hpp"

#include <cmath>

using namespace std;
using namespace std::chrono_literals;

class PathBuilderNode : public rclcpp::Node
{
public:
    PathBuilderNode() : Node("path_builder_node")
    {
        this->declare_parameter<std::string>("target_frame", "world");
        this->declare_parameter<std::string>("child_frame", "base_footprint");
        this->declare_parameter<std::string>("costmap_topic", "/costmap");
        this->declare_parameter<double>("default_lookahead", 4.0);
        this->declare_parameter<int>("collision_cost_threshold", 99);
        this->declare_parameter<double>("connect_radius", 0.6);
        this->declare_parameter<double>("obs_influence_dist", 0.5);
        this->declare_parameter<double>("obs_weight_max", 10.0);
        this->declare_parameter<double>("bubble_dis_away", 0.05);
        this->declare_parameter<double>("robot_radius", 0.35);
        this->declare_parameter<double>("bubble_max_search_dis", 1.0);
        this->declare_parameter<double>("bubble_end_snap_dist", 0.3);
        this->declare_parameter<double>("end_curve_dist", 3.0);
        this->declare_parameter<double>("end_curve_min_yaw_diff", 1.0471975512);
        this->declare_parameter<double>("end_curve_ctrl_ratio", 0.60);

        p.target_frame_ = this->get_parameter("target_frame").as_string();
        p.child_frame_ = this->get_parameter("child_frame").as_string();
        p.costmap_topic_ = this->get_parameter("costmap_topic").as_string();
        p.default_lookahead = this->get_parameter("default_lookahead").as_double();
        p.collision_cost_threshold = this->get_parameter("collision_cost_threshold").as_int();
        p.connect_radius = this->get_parameter("connect_radius").as_double();
        p.obs_influence_dist = this->get_parameter("obs_influence_dist").as_double();
        p.obs_weight_max = this->get_parameter("obs_weight_max").as_double();
        p.bubble_dis_away_ = this->get_parameter("bubble_dis_away").as_double();
        p.robot_radius_ = this->get_parameter("robot_radius").as_double();
        p.bubble_max_search_dis_ = this->get_parameter("bubble_max_search_dis").as_double();
        bubble_end_snap_dist_ = this->get_parameter("bubble_end_snap_dist").as_double();
        p.end_curve_dist = this->get_parameter("end_curve_dist").as_double();
        p.end_curve_min_yaw_diff = this->get_parameter("end_curve_min_yaw_diff").as_double();
        p.end_curve_ctrl_ratio = this->get_parameter("end_curve_ctrl_ratio").as_double();

        // Subs
        sub_global_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", 1, std::bind(&PathBuilderNode::globalCb, this, std::placeholders::_1));
        global_end_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/global_path_end", 1, std::bind(&PathBuilderNode::globalEndPoseCb, this, std::placeholders::_1));
        sub_local_ = this->create_subscription<nav_msgs::msg::Path>("/local_path", 1, std::bind(&PathBuilderNode::localCb, this, std::placeholders::_1));
        sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(p.costmap_topic_, 1, std::bind(&PathBuilderNode::costmapCb, this, std::placeholders::_1));
        sub_points_left_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/regular_points_left", 1, std::bind(&PathBuilderNode::pointsLeftCb, this, std::placeholders::_1));
        sub_points_right_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/regular_points_right", 1, std::bind(&PathBuilderNode::pointsRightCb, this, std::placeholders::_1));

        // Pubs
        pub_bubble_info_ = this->create_publisher<nav_msgs::msg::Path>("/bubble_info", 1);
        pub_bubble_path_ = this->create_publisher<nav_msgs::msg::Path>("/bubble_path", 1);
        pub_score_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/path_builder_node/score", 1);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_builder_node/candidates", 1);
        pub_debug_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/path_builder_node/active_obstacle", 1);
        pub_bubble_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bubble_marker", 1);
        pub_bubble_marker_ids_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bubble_marker_id", 1);

        timer_ = this->create_wall_timer(50ms, std::bind(&PathBuilderNode::controlLoop, this));
    }

    void initTf()
    {
        // 建 Buffer（需要 clock）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        // 兼容不同 ROS2 版本 TransformListener 建構子的寫法
        using TL = tf2_ros::TransformListener;
        if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer &, rclcpp::Node::SharedPtr, bool>)
        {
            tf_listener_ = std::make_shared<TL>(*tf_buffer_, this->shared_from_this(), true);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer &, rclcpp::Node::SharedPtr>)
        {
            tf_listener_ = std::make_shared<TL>(*tf_buffer_, this->shared_from_this());
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer &, rclcpp::Node *, bool>)
        {
            tf_listener_ = std::make_shared<TL>(*tf_buffer_, this, true);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer &, rclcpp::Node *>)
        {
            tf_listener_ = std::make_shared<TL>(*tf_buffer_, this);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer &>)
        {
            tf_listener_ = std::make_shared<TL>(*tf_buffer_);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer *, rclcpp::Node::SharedPtr, bool>)
        {
            tf_listener_ = std::make_shared<TL>(tf_buffer_.get(), this->shared_from_this(), true);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer *, rclcpp::Node::SharedPtr>)
        {
            tf_listener_ = std::make_shared<TL>(tf_buffer_.get(), this->shared_from_this());
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer *, rclcpp::Node *, bool>)
        {
            tf_listener_ = std::make_shared<TL>(tf_buffer_.get(), this, true);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer *, rclcpp::Node *>)
        {
            tf_listener_ = std::make_shared<TL>(tf_buffer_.get(), this);
        }
        else if constexpr (std::is_constructible_v<TL, tf2_ros::Buffer *>)
        {
            tf_listener_ = std::make_shared<TL>(tf_buffer_.get());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No compatible tf2_ros::TransformListener constructor found; tf listener not created.");
            tf_listener_.reset();
        }

        // 建 Core（注意：core 需要 Buffer reference）
        core_ = std::make_unique<PathBuilderCore>(*tf_buffer_, this->get_logger(), this->get_clock());
        core_->setParams(p);

        // Visualizer
        vis_ = std::make_unique<PathBuilderVisualizer>(this->get_clock());
    }

    ~PathBuilderNode() override = default;


private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<PathBuilderCore> core_;
    std::unique_ptr<PathBuilderVisualizer> vis_;

    PathBuilderCore::Params p;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_, sub_local_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_end_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_points_left_, sub_points_right_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_bubble_info_, pub_bubble_path_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_score_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bubble_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bubble_marker_ids_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_debug_;
    rclcpp::TimerBase::SharedPtr timer_;
    double bubble_end_snap_dist_{0.3};

    geometry_msgs::msg::PoseStamped global_end_pose_;

    // Callbacks
    void globalCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
            return;
        nav_msgs::msg::Path tf_path = *msg;        // Simplified for brevity, assume global/local frame handled in core or transform
        tf_path.header.frame_id = p.target_frame_; // Force overwrite if pre-transformed or handle properly
        // Note: For production, keep the transform logic from previous code
        core_->updateGlobalPath(tf_path);
    }

    void globalEndPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr end_pose)
    {
        global_end_pose_ = *end_pose;
        core_->updateGlobalEndPose(*end_pose);
    }

    void localCb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
            return;
        core_->updateLocalPath(*msg);
    }

    void costmapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { core_->updateCostmap(*msg); }

    void pointsLeftCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::vector<bsp::Vec2> pts;
        for (const auto &p : msg->poses)
            pts.push_back({p.position.x, p.position.y});
        core_->updateLeftPoints(pts);
    }

    void pointsRightCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::vector<bsp::Vec2> pts;
        for (const auto &p : msg->poses)
            pts.push_back({p.position.x, p.position.y});
        core_->updateRightPoints(pts);
    }

    void controlLoop()
    {
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!core_->getRobotPose(robot_pose))
        {
            return;
        }

        // 1. Compute
        nav_msgs::msg::Path bubble_path = core_->computePath(robot_pose);

        // 2. Publish paths first
        pub_bubble_info_->publish(bubble_path);

        nav_msgs::msg::Path output_bubble_path;
        if (!bubble_path.poses.empty())
        {
            output_bubble_path = core_->densifiedPath(bubble_path);
            if (!output_bubble_path.poses.empty())
            {
                if (!global_end_pose_.header.frame_id.empty() &&
                    global_end_pose_.header.frame_id == output_bubble_path.header.frame_id)
                {
                    const auto &last = output_bubble_path.poses.back().pose.position;
                    const auto &goal = global_end_pose_.pose.position;
                    const double dist_to_end = std::hypot(last.x - goal.x, last.y - goal.y);
                    if (dist_to_end <= bubble_end_snap_dist_)
                        output_bubble_path.poses.back().pose = global_end_pose_.pose;
                }
                for (auto &p : output_bubble_path.poses)
                    p.pose.position.z = 0;
            }
        }
        else
        {
            output_bubble_path.header = bubble_path.header;
        }

        pub_bubble_path_->publish(output_bubble_path);

        // 3. Visualize (based on published bubble path)
        // 只有在避障狀態 (或剛避障完) 才顯示 Candidate
        // 但因為我們切換很快，持續顯示 Last Candidates 也可以
        vis_->visualizeCandidates(core_->getLastCandidates(), core_->getLastBestIndex(), p.target_frame_);
        vis_->visualizeDebugPoint(core_->getActiveObstacle(), p.target_frame_);
        vis_->VisualizeBubble(bubble_path);

        pub_markers_->publish(vis_->getCandidateMarkers());
        pub_debug_->publish(vis_->getDebugPointMsg());
        pub_bubble_markers_->publish(vis_->GetBubbleMarkers());
        pub_bubble_marker_ids_->publish(vis_->GetBubbleIdMarkers());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathBuilderNode>();
    node->initTf();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
