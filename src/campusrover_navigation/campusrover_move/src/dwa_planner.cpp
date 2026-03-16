#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include "std_msgs/msg/empty.hpp"

#include <campusrover_msgs/srv/elevator_status_checker.hpp>
#include "campusrover_msgs/srv/planner_function.hpp"



#include <algorithm>
#define M_PI 3.14159265358979323846
using namespace std;

class DWANode : public rclcpp::Node
{
public:
    DWANode() : Node("dwa_planner_node")
    {
        // Declare & Get Parameters
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<double>("arriving_range_dis", 0.2);
        this->declare_parameter<double>("arriving_range_angle", 0.1);
        this->declare_parameter<double>("max_linear_acceleration", 3.0);
        this->declare_parameter<double>("n_max_linear_acceleration", -1.0);
        this->declare_parameter<double>("max_angular_acceleration", 8.0);
        this->declare_parameter<double>("max_linear_velocity", 1.0);
        this->declare_parameter<double>("min_linear_velocity", 0.0);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<double>("min_angular_velocity", 1.0);
        this->declare_parameter<double>("target_point_dis", 1.0);
        this->declare_parameter<double>("threshold_occupied", 10.0);
        this->declare_parameter<double>("footprint_max_x", 1.5);
        this->declare_parameter<double>("footprint_min_x", -0.5);
        this->declare_parameter<double>("footprint_max_y", 0.5);
        this->declare_parameter<double>("footprint_min_y", -0.5);
        this->declare_parameter<double>("speed_pid_k", 0.1);
        this->declare_parameter<double>("min_angle_of_linear_profile", 0.1);
        this->declare_parameter<double>("max_angle_of_linear_profile", 0.5);
        this->declare_parameter<bool>("enable_costmap_obstacle", false);
        this->declare_parameter<bool>("direction_inverse", false);
        this->declare_parameter<bool>("enable_linear_depend_angular", false);
        this->declare_parameter<int>("trajectory_num", 44);
        this->declare_parameter<int>("trajectory_point_num", 10);
        this->declare_parameter<double>("delta_t", 0.05);
        this->declare_parameter<double>("simulation_time", 2.0);
        this->declare_parameter<double>("obstable_cost_weight", 1.2);
        this->declare_parameter<double>("target_dis_weight", 1.0);
        this->declare_parameter<double>("velocity_weight", 1.0);
        this->declare_parameter<double>("obstacle_max_dis", 2.0);
        this->declare_parameter<double>("obstacle_min_dis", 0.5);
        this->declare_parameter<double>("statu_v", 0.0);
        this->declare_parameter<double>("statu_w", 0.0);
        this->declare_parameter<double>("target_bias", 0.3);

        // Get parameters
        this->get_parameter("robot_frame", robot_frame_);
        this->get_parameter("arriving_range_dis", arriving_range_dis_);
        this->get_parameter("arriving_range_angle", arriving_range_angle_);
        this->get_parameter("max_linear_acceleration", max_linear_acceleration_);
        this->get_parameter("n_max_linear_acceleration", n_max_linear_acceleration_);
        this->get_parameter("max_angular_acceleration", max_angular_acceleration_);
        this->get_parameter("max_linear_velocity", max_linear_velocity_);
        this->get_parameter("min_linear_velocity", min_linear_velocity_);
        this->get_parameter("max_angular_velocity", max_angular_velocity_);
        this->get_parameter("min_angular_velocity", min_angular_velocity_);
        this->get_parameter("target_point_dis", target_point_dis_);
        this->get_parameter("threshold_occupied", threshold_occupied_);
        this->get_parameter("footprint_max_x", footprint_max_x_);
        this->get_parameter("footprint_min_x", footprint_min_x_);
        this->get_parameter("footprint_max_y", footprint_max_y_);
        this->get_parameter("footprint_min_y", footprint_min_y_);
        this->get_parameter("speed_pid_k", speed_pid_k_);
        this->get_parameter("min_angle_of_linear_profile", min_angle_of_linear_profile_);
        this->get_parameter("max_angle_of_linear_profile", max_angle_of_linear_profile_);
        this->get_parameter("enable_costmap_obstacle", enable_costmap_obstacle_);
        this->get_parameter("direction_inverse", direction_inverse_);
        this->get_parameter("enable_linear_depend_angular", enable_linear_depend_angular_);
        this->get_parameter("trajectory_num", trajectory_num_);
        this->get_parameter("trajectory_point_num", trajectory_point_num_);
        this->get_parameter("delta_t", delta_t_);
        this->get_parameter("simulation_time", simulation_time_);
        this->get_parameter("obstable_cost_weight", obstable_cost_weight_);
        this->get_parameter("target_dis_weight", target_dis_weight_);
        this->get_parameter("velocity_weight", velocity_weight_);
        this->get_parameter("obstacle_max_dis", obstacle_max_dis_);
        this->get_parameter("obstacle_min_dis", obstacle_min_dis_);
        this->get_parameter("statu_v", statu_v_);
        this->get_parameter("statu_w", statu_w_);
        this->get_parameter("target_bias", target_bias_);

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "路徑規劃器節點已初始化。");
        // Subscribers
        elevator_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "elevator_path", 10, std::bind(&DWANode::ElevatorPathCallback, this, std::placeholders::_1));
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_path", 10, std::bind(&DWANode::GlobalPathCallback, this, std::placeholders::_1));
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "costmap", 10, std::bind(&DWANode::CostmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DWANode::OdomCallback, this, std::placeholders::_1));
        
        // Publishers
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        twist_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("twist_path", 20);
        path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dwa_trajectories", 10);
        global_status_check_pub_ = this->create_publisher<std_msgs::msg::Empty>("reach_goal", 10);
        target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dwa_target_marker", 10);

        // === Service ===
        service_ = this->create_service<campusrover_msgs::srv::PlannerFunction>(
            "planner_function_dwa",
            std::bind(&DWANode::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Service client
        elevator_status_check_client_ =
            this->create_client<campusrover_msgs::srv::ElevatorStatusChecker>("elevator_status_checker");

        // Timers
        auto delta_tms = delta_t_ * 1000.0;
        auto timer_period_ms = static_cast<long long>(delta_tms);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), std::bind(&DWANode::TimerCallback, this));
        msgs_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&DWANode::MsgsTimerCallback, this));

        InitialAccTrajectorySampling();
        
        last_path_pub_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "DWA 規劃器節點已初始化。");

    }


private:
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr elevator_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr twist_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr global_status_check_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    nav_msgs::msg::Path globle_path_;
    geometry_msgs::msg::PoseStamped target_pose_;
    nav_msgs::msg::OccupancyGrid costmap_data_;

    geometry_msgs::msg::Pose robot_tf_pose_;
    geometry_msgs::msg::PoseArray obstacle_poses_;

    // Service server
    rclcpp::Service<campusrover_msgs::srv::PlannerFunction>::SharedPtr service_;

    // Service client
    rclcpp::Client<campusrover_msgs::srv::ElevatorStatusChecker>::SharedPtr elevator_status_check_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr msgs_timer_;

    std::vector<Eigen::Vector2d> acc_pairs_;
    std::vector<Eigen::Vector2d> twist_pairs_;
    std::string robot_frame_;

    // Mutex for costmap data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
    std::mutex costmap_mutex_;

    int status_msg_;
    int path_sub_mode_;

    double arriving_range_dis_;
    double arriving_range_angle_;

    bool action_flag_ = false;
    bool get_globle_path_ = false;
    bool get_costmap_data_ = false;
    bool obstacle_stop_cmd_ = false;
    bool arriving_end_point_= false;
    bool arriving_end_direction_= false;
    bool enable_costmap_obstacle_;
    bool direction_inverse_= false;
    bool get_velocity_data_ = true;
    bool get_obstacle_data_ = false;

    double current_v_;
    double current_w_;

    bool enable_linear_depend_angular_;
    double max_angle_of_linear_profile_;
    double min_angle_of_linear_profile_;

    double threshold_occupied_;
    double footprint_max_x_;
    double footprint_min_x_;
    double footprint_max_y_;
    double footprint_min_y_;
    double obstacle_max_dis_;
    double obstacle_min_dis_;

    double robot_yaw_;
    double speed_pid_k_;
    double target_yaw_;

    double max_linear_acceleration_;
    double n_max_linear_acceleration_;
    double max_angular_acceleration_;
    double max_linear_velocity_;
    double min_linear_velocity_;
    double max_angular_velocity_;
    double min_angular_velocity_;
    double target_point_dis_;

    double active_angular_;

    int trajectory_num_;
    int trajectory_point_num_;
    double simulation_time_;
    double delta_t_;
    double statu_v_;
    double statu_w_;

    //objective function
    double obstable_cost_weight_;
    double target_dis_weight_;
    double velocity_weight_;

    double target_bias_;

    //visualizePath發布限制
    rclcpp::Time last_path_pub_time_;
    const double path_pub_interval_ = 0.5; // 2Hz → 0.5s

    void UpdateCampusRoverPoseFromTF()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
        transformStamped = tf_buffer_->lookupTransform(globle_path_.header.frame_id, robot_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
        RCLCPP_WARN(this->get_logger(), "Cannot get TF transform: %s", ex.what());
        return;
        }

        robot_tf_pose_.position.x = transformStamped.transform.translation.x;
        robot_tf_pose_.position.y = transformStamped.transform.translation.y;
        robot_tf_pose_.position.z = transformStamped.transform.translation.z;
        robot_tf_pose_.orientation = transformStamped.transform.rotation;

        tf2::Quaternion q;
        tf2::fromMsg(robot_tf_pose_.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (!direction_inverse_)
        robot_yaw_ = yaw;
        else
        {
        robot_yaw_ = yaw + M_PI;
        angle_normalize(robot_yaw_);
        }
    }

    void InitialAccTrajectorySampling()
    {
        for (int i = 0; i < trajectory_num_; i++)
        {
            for (int j = 0; j < trajectory_num_ * 2.0; j++)
            {
                Eigen::Vector2d acc_pair(
                    -max_linear_acceleration_ + i * (2.0 * max_linear_acceleration_ / (trajectory_num_ - 1)),
                    -max_angular_acceleration_ + j * (2.0 * max_angular_acceleration_ / ((trajectory_num_ * 2.0) - 1)));
                acc_pairs_.push_back(acc_pair);
            }
        }

    }

    void check_arrive_point()
    {
        double dist = sqrt(pow(target_pose_.pose.position.x - robot_tf_pose_.position.x, 2) +
                        pow(target_pose_.pose.position.y - robot_tf_pose_.position.y, 2));

        arriving_end_point_ = (dist < arriving_range_dis_);
    }

    void check_arrive_direction()
    {
        double angle_error = target_yaw_ - robot_yaw_;
        angle_normalize(angle_error);
        arriving_end_direction_ = (fabs(angle_error) < arriving_range_angle_);
    }
    void TimerCallback()
    {
        if (!action_flag_)
            return;

        // if (obstacle_stop_cmd_)
        // {
        //   TwistPublish(0.0, 0.0);
        //   return;
        // }

        if (path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_BUTTON_PARKING)
        {
            TwistPublish(max_linear_velocity_, active_angular_);
        }
        else
        {
            if (get_globle_path_)
            {
                if (get_velocity_data_)
                {
                    if (get_costmap_data_ || get_obstacle_data_ || !enable_costmap_obstacle_)
                    {
                        UpdateCampusRoverPoseFromTF();

                        if (!arriving_end_direction_)//方向未到達
                        {
                            if (!arriving_end_point_)//位置未到達
                            {
                                check_arrive_point();//確認位置是否到達

                                if (arriving_end_point_)//到達位置
                                {
                                    moving_to_target_direction();//移動至目標方向
                                    return;
                                }

                                moving_to_target_point();//位置未到達->移動至目標位置
                            }
                            else//位置到達
                            {
                                check_arrive_direction();//確認方向是否到達

                                if (arriving_end_direction_)//方向到達
                                {
                                    status_msg_ = 4;
                                    TwistPublish(0.0, 0.0);
                                    return;
                                }

                                moving_to_target_direction();//方向未到達->移動至目標方向
                            }
                        }
                        else//方向到達
                        {
                            status_msg_ = 4;
                            TwistPublish(0.0, 0.0);
                        }
                    }
                    else//無costmap資料
                    {
                        status_msg_ = 2;
                        TwistPublish(0.0, 0.0);
                    }
                }
                else//無odom資料
                {
                    status_msg_ = 5;
                    TwistPublish(0.0, 0.0);
                }
            }
            else//無路徑資料
            {
            status_msg_ = 1;
            TwistPublish(0.0, 0.0);
            }
        }
    }

    //------------------------------------------------------------

    void MsgsTimerCallback()
    {
    switch (status_msg_)
    {
    case 1:
        RCLCPP_WARN(this->get_logger(), "dwa_planner : Without Global Path to follow, Waiting for Path input");
        break;
    case 2:
        RCLCPP_WARN(this->get_logger(), "dwa_planner : Without costmap input, Waiting for costmap input");
        break;
    case 3:
        RCLCPP_INFO(this->get_logger(), "dwa_planner : detect obstacle");
        break;
    case 4:
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Arrival the destination");
        break;
    case 5:
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Without odom input, Waiting for odom input");
        break;
    default:
        // RCLCPP_DEBUG(this->get_logger(), "dp planner : moving");
        break;
    }

    status_msg_ = 0;
    }

    void moving_to_target_point()
    {
        // === 1. 找出 Global Path 上的最近點與目標點 ===
        // [修正 1] 移除 static，改為局部變數，確保每次重新計算
        double x_p, y_p;
        double dist_fp_p;
        double closest_dist = std::numeric_limits<double>::max();
        int closest_id = -1;

        double x_fp = 0.0, y_fp = 0.0;
        double looking_dist;
        size_t target_point_id = 0; // 使用 size_t 確保與 vector 索引一致

        // 檢查路徑是否存在
        if (globle_path_.poses.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Global path is empty!");
            TwistPublish(0.0, 0.0);
            return;
        }

        for (size_t cp = 0; cp < globle_path_.poses.size(); cp++)
        {
            x_p = globle_path_.poses[cp].pose.position.x;
            y_p = globle_path_.poses[cp].pose.position.y;
            dist_fp_p = std::hypot(robot_tf_pose_.position.x - x_p, robot_tf_pose_.position.y - y_p);
            if (cp == 0)
            {
            closest_dist = dist_fp_p;
            closest_id = cp;
            }
            else if (dist_fp_p < closest_dist)
            {
            closest_dist = dist_fp_p;
            closest_id = cp;
            }
        }

        //std::cout << "closest_id : " << closest_id<<" new_path_ : " << new_path_<<'\n';
        if (closest_id >= globle_path_.poses.size() - 1)
        {
            target_point_id = closest_id;
        }
        else
        {
            //find the target point from globle path for following

            for(int fp = closest_id;fp<globle_path_.poses.size();fp++)
            {
            target_point_id = fp;
            x_fp = globle_path_.poses[fp].pose.position.x;
            y_fp = globle_path_.poses[fp].pose.position.y;
            looking_dist = sqrt(pow(x_fp - globle_path_.poses[closest_id].pose.position.x, 2) 
                                + pow(y_fp - globle_path_.poses[closest_id].pose.position.y, 2));
            
            if(looking_dist >= target_point_dis_)
            {
                break;
            }
            }
            // cout<< "closest_id : " << closest_id<<"target_point_id"<<target_point_id<<endl;
        }

        //設定目標點
        geometry_msgs::msg::PoseStamped target_pose;
        // [關鍵修正] 強制設定時間戳為 0，確保 TF 查詢使用的是「最新」的轉換關係，避免逾時
        target_pose.header.stamp = rclcpp::Time(0); 
        target_pose.header.frame_id = globle_path_.header.frame_id;
        target_pose.pose.position.x = globle_path_.poses[target_point_id].pose.position.x;
        target_pose.pose.position.y = globle_path_.poses[target_point_id].pose.position.y;
        target_pose.pose.position.z = globle_path_.poses[target_point_id].pose.position.z;

        // --- 新增：發佈目標點 Marker 到 RViz 2 ---
        visualization_msgs::msg::Marker target_marker;
        target_marker.header.frame_id = target_pose.header.frame_id; // 使用 global path 的 frame
        target_marker.header.stamp = this->now();
        target_marker.ns = "dwa_targets";
        target_marker.id = 0;
        target_marker.type = visualization_msgs::msg::Marker::SPHERE; // 可以選擇 CUBE, ARROW, SPHERE 等

        // 設定 Marker 的大小
        target_marker.scale.x = 0.2; // 20 cm 寬
        target_marker.scale.y = 0.2; // 20 cm 高
        target_marker.scale.z = 0.2; // 20 cm 深

        // 設定 Marker 的顏色 (R, G, B, A)
        target_marker.color.r = 1.0f; // 紅色
        target_marker.color.g = 0.0f;
        target_marker.color.b = 0.0f;
        target_marker.color.a = 0.8f; // 80% 不透明

        // 設定 Marker 的位置 (直接使用 target_pose 的位置)
        target_marker.pose = target_pose.pose;

        // 發佈 Marker
        target_marker_pub_->publish(target_marker);

        //Objective Function
        double ob_closest_dist;
        double ob_dis;
        double step_dis, step_theta, theta;
        double s_x,s_y;
        Eigen::Vector3d function_value;
        std::vector<Eigen::Vector3d> objective_function_values;;
        double t_x, t_y;
        double tg_angle;
        geometry_msgs::msg::PoseStamped target_points_b;
        double max_ob_score;
        double max_tg_angle;
        double max_velocity;
        double best_reward = -1.0;
        int best_id;
        double objective_function_sum;

        nav_msgs::msg::OccupancyGrid::SharedPtr map_snapshot;
        {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            if (!current_costmap_) // 確保地圖不是空的
            {
                RCLCPP_WARN(this->get_logger(), "DWA: Costmap not received yet!");
                return; // 這週期不跑了
            }
            map_snapshot = current_costmap_; // 複製一份指標
        }

        // --- 準備 TF 轉換 (只需做一次) ---
        // [關鍵修正] 在迴圈外先轉換好目標點，避免在迴圈內或每次呼叫都可能因為 TF 延遲而卡住
        try
        {
            // 這裡使用較短的 timeout (0.05s)，因為我們已經用了 Time(0)，應該要是瞬時的
            // 如果失敗，就不要卡住整個迴圈，直接 return
            if (tf_buffer_->canTransform(robot_frame_, target_pose.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.05))) {
                tf_buffer_->transform(target_pose, target_points_b, robot_frame_);
            } else {
                // 如果不能轉換，可能是 TF 樹還沒建立好，或者時間戳差異太大
                // 這裡選擇 return，因為沒有目標方向就無法評分
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "DWA: Waiting for TF transform...");
                TwistPublish(0.0, 0.0);
                return;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform error: %s", ex.what());
            TwistPublish(0.0, 0.0);
            return;
        }

        // 1. 計算目標方向（世界座標系）
        double dx = x_fp - robot_tf_pose_.position.x;
        double dy = y_fp - robot_tf_pose_.position.y;
        double target_theta = atan2(dy, dx);

        // 2. 計算角度差並正規化到 [-pi, pi]
        double target_angle_diff = target_theta - robot_yaw_;
        while (target_angle_diff > M_PI) target_angle_diff -= 2.0*M_PI;
        while (target_angle_diff < -M_PI) target_angle_diff += 2.0*M_PI;
        double target_angle = atan2(target_points_b.pose.position.y, target_points_b.pose.position.x);
        angle_normalize(target_angle);
        //接近終點減速距離
        double min_reduce_dis = 2.0;
        double target_dist_for_slowdown = std::hypot(globle_path_.poses.back().pose.position.x - robot_tf_pose_.position.x, 
                                                globle_path_.poses.back().pose.position.y - robot_tf_pose_.position.y);
        static double current_max_linear_velocity = max_linear_velocity_;
        static double current_min_linear_velocity = min_linear_velocity_;
        static bool ReverseMode = false;
        static bool ClosingMode = false;
        
        if(target_dist_for_slowdown < min_reduce_dis)
        {
            // 簡化並確保減速因子的安全範圍 [0.1, 1.0]
            double scale = std::max(0.4, target_dist_for_slowdown / min_reduce_dis); 
            current_max_linear_velocity = max_linear_velocity_ * scale;
            current_min_linear_velocity = min_linear_velocity_ * scale;
            if(!ClosingMode)
            {
                RCLCPP_INFO(this->get_logger(), "Approaching Target Mode: Activating slowdown.");
                ClosingMode = true;
            }
        }
        else if(abs(target_angle_diff)> (M_PI/2))
        {
            current_max_linear_velocity = max_linear_velocity_*0.3;
            current_min_linear_velocity = min_linear_velocity_*0.3;
            if(!ReverseMode)
            {
                RCLCPP_INFO(this->get_logger(), "Reverse Mode Active: Reducing linear velocity limits.");
                ReverseMode = true;
            }
        }
        else if(abs(target_angle_diff)<= (M_PI/3)&&(target_dist_for_slowdown > min_reduce_dis))
        {
            current_max_linear_velocity = max_linear_velocity_;
            current_min_linear_velocity = min_linear_velocity_;
            if(ReverseMode || ClosingMode)
            {
                RCLCPP_INFO(this->get_logger(), "Normal Mode Active: Restoring linear velocity limits.");
                ReverseMode = false;
                ClosingMode = false;
            }
        }


        twist_pairs_.clear();

        // 設定候選角速度中心為目標方向差的一部分，K 控制引導強度
        double K = target_bias_; // 0~1，越大越偏向目標方向
        double w_center = K * target_angle_diff;

        for (int i = 0; i < acc_pairs_.size(); i++)
        {
            // 線速度仍然依據當前速度 + 加速度生成
            double v_potential = statu_v_ + acc_pairs_[i](0)  * delta_t_;
            double v_final = std::clamp(v_potential, current_min_linear_velocity, current_max_linear_velocity);

            // 角速度以 w_center 為中心生成候選，並加上加速度影響
            double w_potential = w_center + acc_pairs_[i](1)  * delta_t_;
            double w_final = std::clamp(w_potential, min_angular_velocity_, max_angular_velocity_);

            Eigen::Vector2d twist_pair(v_final, w_final);
            twist_pairs_.push_back(twist_pair);
        }

        for(int p=0; p < twist_pairs_.size(); p++)
        {
            //Obstacle Distance Function
            // [新邏輯]
            double total_trajectory_cost = 0.0;
            bool trajectory_is_valid = true;
            double sim_step_time = simulation_time_ / trajectory_point_num_;
            // 處理 v=0 (原地旋轉) 的情況
            if (std::abs(twist_pairs_[p](0)) < 1e-6) // 如果 v 接近 0
            {
                // RCLCPP_INFO(this->get_logger(), "v is zero, in-place rotation");
                step_dis = 0.0; // 原地旋轉，一步的距離是 0
                step_theta = twist_pairs_[p](1) * sim_step_time;
            }
            else // v > 0 (正常前進)
            {
                // RCLCPP_INFO(this->get_logger(), "v is non-zero, normal movement");
                step_dis = twist_pairs_[p](0) * sim_step_time;
                step_theta = twist_pairs_[p](1) * sim_step_time;
            }
            
            s_x = s_y = 0;
            theta = 0.0;
            for (int j = 0; j < trajectory_point_num_; j++)
            {
                s_x += step_dis*cos(theta);
                s_y += step_dis*sin(theta);
                theta += step_theta;

                // [新邏輯：O(1) 查表，超快]
                double point_cost = get_cost_at_point(s_x, s_y, map_snapshot);
                // RCLCPP_INFO(this->get_logger(), "point_cost = %f", point_cost);
                if (point_cost == -1.0)
                {
                    // 這條軌跡撞牆或出界了
                    total_trajectory_cost = -1.0; // 標記為無效
                    trajectory_is_valid = false;
                    break; // 這個小迴圈 (j) 可以停了
                }

                // 累加這條軌跡上所有點的 cost
                total_trajectory_cost += point_cost;
            }

            if (!trajectory_is_valid) //評分
            {
                function_value(0) = -1.0; // 判此軌跡 (p) 出局
            }
            else
            {
                // 我們要的是「總代價最低」的軌跡
                // 為了讓評分統一 (分數越高越好)，我們用 1 / (1 + cost)
                // (1.0 + ... 是為了避免除以 0)
                function_value(0) = 1.0 / (1.0 + total_trajectory_cost);
                // RCLCPP_INFO(this->get_logger(), "function_value(0) = %f", function_value(0));
            }
            
            if(p == 0)
            {
                max_ob_score = function_value(0); // 這裡存的是「分數」，不是「距離」
            }
            else
            {
                if( function_value(0) > max_ob_score)
                {
                    max_ob_score = function_value(0);
                }
            }

            // // ... (前面的 tf_transform 和 atan2) ...
            // double target_angle = atan2(target_points_b.pose.position.y
            //                             ,target_points_b.pose.position.x);
            // angle_normalize(target_angle); // atan2 已經在 [-pi, pi]，這行可選

            // // 1. 計算這條軌跡 p 預測的最終航向 (在 robot_frame_ 中)
            // double final_heading = twist_pairs_[p](1) * simulation_time_;
            
            // // 2. 計算角度差
            // double diff = target_angle - final_heading;

            // // 3. (修正 1) *先* 正規化角度差
            // angle_normalize(diff); // 確保 diff 在 [-pi, pi] 之間
            
            // // 4. (修正 1) *後* 取絕對值，得到成本
            // double tg_angle = abs(diff); // 成本總是大於 0

            // // 5. 設定 p 軌跡的航向成本
            // function_value(1) = tg_angle;

            // // 6. (加回來) 更新 max_tg_angle，使用與 max_velocity 同樣的邏輯
            // if(p == 0)
            // {
            //     max_tg_angle = tg_angle;
            // }
            // else
            // {
            //     if( tg_angle > max_tg_angle)
            //     {
            //         max_tg_angle = tg_angle;
            //     }
            // }

            // 軌跡終點 B 座標 (假設機器人起始於 (0,0,0) 的 robot_frame_ 中):
            double end_x_b = s_x;
            double end_y_b = s_y;

            // 目標點 B 在 robot_frame_ 中的座標:
            double target_x_b = target_points_b.pose.position.x;
            double target_y_b = target_points_b.pose.position.y;
            
            // 計算軌跡終點與目標點 B 之間的直線距離
            double goal_distance = std::hypot(target_x_b - end_x_b, target_y_b - end_y_b);

            // 越接近目標，分數越高 (將距離轉為分數)
            // 1.0 / (1.0 + distance)
            function_value(1) = 1.0 / (1.0 + goal_distance);
            
            // 更新 max_tg_angle (現在它代表 max_goal_proximity_score)
            if(p == 0)
            {
                max_tg_angle = function_value(1);
            }
            else
            {
                if( function_value(1) > max_tg_angle)
                {
                    max_tg_angle = function_value(1);
                }
            }

            //Velocity Function
            function_value(2) = twist_pairs_[p](0);

            if(p == 0)
            {
                max_velocity = twist_pairs_[p](0);
            }
            else
            {
                if(twist_pairs_[p](0) > max_velocity)
                {
                    max_velocity = twist_pairs_[p](0);
                }
            } 

            // cout<< "v1:"<<objective_function_values[p](0)<< " v2:"<<objective_function_values[p](1)<<endl;

            objective_function_values.push_back(function_value);
        }


        bool best_id_founded = false;

        double norm_ob_score = (max_ob_score > 0.0) ? max_ob_score : 1.0;
        double norm_tg_angle = (max_tg_angle > 0.0) ? max_tg_angle : 1.0;
        double norm_velocity = (max_velocity > 0.0) ? max_velocity : 1.0;

        for(size_t f=0;f<objective_function_values.size();f++)
        {
            if(objective_function_values[f](0) >= 0)
            {
                double score_obs = obstable_cost_weight_ * (objective_function_values[f](0)/norm_ob_score);
                double score_path = target_dis_weight_ * ((objective_function_values[f](1)/norm_tg_angle));
                double score_vel = velocity_weight_ * (objective_function_values[f](2)/norm_velocity);

                objective_function_sum = score_obs + score_path + score_vel;
                // RCLCPP_INFO(this->get_logger(), "score_obs = %f", score_obs);
                if(objective_function_sum > best_reward)
                {
                    best_reward = objective_function_sum;
                    best_id = f;
                    best_id_founded = true;
                }
            }

        }

        if(best_id_founded) 
        {
            // DWA 找到了最佳解
            double best_v = twist_pairs_[best_id](0);
            double best_w = twist_pairs_[best_id](1);
            
            TwistPublish(best_v, best_w);
            // --- 狀態更新 ---
            statu_v_ = best_v; 
            statu_w_ = best_w;
        }
        else
        {
            // DWA 宣告失敗 (所有路徑都撞牆)
            // 執行「恐慌」：原地煞車
            TwistPublish(0.0, 0.0);
            
            statu_v_ = 0.0;
            statu_w_ = 0.0;
        }
        // RCLCPP_INFO(this->get_logger(), "Best ID: %d, Reward: %.3f", best_id, best_reward);
        if(best_id_founded)
        {
            VisualizePath(twist_pairs_, simulation_time_, best_id);
        }
    }
    void moving_to_target_direction()
    {
        double yaw_error = target_yaw_ - robot_yaw_;
        angle_normalize(yaw_error);

        double ang_vel = yaw_error * speed_pid_k_;
        
        if (ang_vel > 0 && ang_vel < 0.1)
            ang_vel = 0.1;
        else if (ang_vel < 0 && ang_vel > -0.1)
            ang_vel = -0.1;

        TwistPublish(0.0, ang_vel);
    }

    void TwistPublish(double x, double z)
    {
        geometry_msgs::msg::Twist pub_twist;

        // 限制角速度
        if (z > max_angular_velocity_)
            pub_twist.angular.z = max_angular_velocity_;
        else if (z < -max_angular_velocity_)
            pub_twist.angular.z = -max_angular_velocity_;
        else
            pub_twist.angular.z = z;

        // 線速度依角速度做調整
        if (std::abs(z) > max_angular_velocity_ * 0.7)
            pub_twist.linear.x = x * (1 - ((std::abs(z) - (max_angular_velocity_ * 0.7)) / (2.0 * max_angular_velocity_)));
        else
            pub_twist.linear.x = x;

        // 發佈
        twist_pub_->publish(pub_twist);
    }

    void angle_normalize(double &angle)
    {
        if (angle > M_PI)
            angle = -2 * M_PI + angle;
        else if (angle < -M_PI)
            angle = 2 * M_PI + angle;
    }

    void ElevatorPathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        if(path_sub_mode_ != campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
            return;

        if(path->poses.empty())
            return;

        globle_path_.header.frame_id = path->header.frame_id;
        globle_path_.poses.clear();

        for (size_t i = 0; i < path->poses.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose = path->poses[i].pose;
            globle_path_.poses.push_back(pose);

            if(i == path->poses.size() - 1)
            {
            target_pose_.header.frame_id = path->header.frame_id;
            target_pose_.pose = path->poses[i].pose;
            }
        }

        tf2::Quaternion q(target_pose_.pose.orientation.x,
                            target_pose_.pose.orientation.y,
                            target_pose_.pose.orientation.z,
                            target_pose_.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        target_yaw_ = yaw;

        get_globle_path_ = true;
    }

    void GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        if(path->poses.empty())
            return;

        globle_path_.header.frame_id = path->header.frame_id;
        globle_path_.poses.clear();

        for (size_t i = 0; i < path->poses.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose = path->poses[i].pose;
            globle_path_.poses.push_back(pose);

            if(i == path->poses.size() - 1)
            {
            target_pose_.header.frame_id = path->header.frame_id;
            target_pose_.pose = path->poses[i].pose;
            }
        }

        tf2::Quaternion q(target_pose_.pose.orientation.x,
                            target_pose_.pose.orientation.y,
                            target_pose_.pose.orientation.z,
                            target_pose_.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        target_yaw_ = yaw;

        if(!get_globle_path_)
            RCLCPP_INFO(this->get_logger(), "dwa_planner : Get the global path input!!");

        get_globle_path_ = true;
    }

    void CostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        // RCLCPP_INFO(this->get_logger(), "CostmapCallback enable_costmap_obstacle : %d", enable_costmap_obstacle_);
        if(!enable_costmap_obstacle_)
            return;

        // 鎖上門，確保 DWA 迴圈不會同時讀取
        std::lock_guard<std::mutex> lock(costmap_mutex_); 

        // 把新地圖存起來
        current_costmap_ = map; 

        if(!get_costmap_data_ && current_costmap_){
            RCLCPP_INFO(this->get_logger(), "topic: %s", map->header.frame_id.c_str());
            RCLCPP_INFO(this->get_logger(), "dwa_planner : Get the Costmap input!!");
            get_costmap_data_ = true;
        }
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        current_v_ = odom->twist.twist.linear.x;
        current_w_ = odom->twist.twist.angular.z;

        if(!get_velocity_data_)
            RCLCPP_INFO(this->get_logger(), "dwa_planner : Get the Odom input!!");

        get_velocity_data_ = true;
    }

    //-----------------------------------------------------------------------------------------------
    double get_cost_at_point(double x, double y, const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
    {
        // // --- 0. [除錯] 第一次呼叫時印出地圖資訊 ---
        // static bool map_info_logged = false;
        // if (!map_info_logged && map) // 檢查 map 是否為空get_cost_at_point
        // {
        //     RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: --- Costmap Info (First Call) ---");
        //     RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: Frame ID: %s", map->header.frame_id.c_str());
        //     RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: Resolution: %.4f", map->info.resolution);
        //     RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: Width: %d, Height: %d", map->info.width, map->info.height);
        //     RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: Origin X: %.2f, Y: %.2f", 
        //         map->info.origin.position.x, map->info.origin.position.y);
        //     RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: ------------------------------------");
        //     map_info_logged = true;
        // }
        // --- 結束除錯 ---


        // --- 1. 把 (x, y) 公尺座標 轉成 (mx, my) 格子座標 ---
        double res = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;
        
        // 檢查 res 是否為 0，避免除以零
        if (res == 0.0) {
            RCLCPP_ERROR(this->get_logger(), "DWA_DEBUG: Costmap resolution is ZERO!");
            return -1.0;
        }

        unsigned int mx = static_cast<unsigned int>((x - origin_x) / res);
        unsigned int my = static_cast<unsigned int>((y - origin_y) / res);

        // RCLCPP_INFO(this->get_logger(), "x,y = %f, %f ; mx,my = %u, %u", x, y, mx, my);

        // --- 2. 檢查是否在地圖範圍內 ---
        if (mx >= map->info.width || my >= map->info.height)
        {
            // 軌跡點在 costmap 之外，這很常發生，暫時用 INFO 印出
            // RCLCPP_INFO(this->get_logger(), "DWA_DEBUG: Point (%.2f, %.2f) -> Cell (%u, %u) is OUT OF BOUNDS", x, y, mx, my);
            return -1.0; 
        }

        // --- 3. 查表：把 (mx, my) 轉成 1D 陣列的索引 ---
        int index = my * map->info.width + mx;
        
        // map->data 是 int8_t[] (signed char)
        int8_t data = map->data[index]; // 讀取代價 (0-100, -1 for unknown)

        // --- 4. 判斷代價 ---
        // (注意：nav2 costmap: 99=INSCRIBED, 100=LETHAL. 
        //  這裡用 99 當作「撞到」的門檻比較安全)
        if (data > 99 || data < 0) // data < 0 是 "unknown" (-1)
        {
            return -1.0; // 撞牆或未知區域，判此軌跡為無效
        }
        // RCLCPP_WARN(this->get_logger(), "DWA_DEBUG: GET COST! Point (%.2f, %.2f) -> Cell (%u, %u) -> Cost: %d", x, y, mx, my, data);
        return static_cast<double>(data); // 回傳 0-98 之間的代價
    }
   
    void VisualizePath(const std::vector<Eigen::Vector2d> twist_pairs, double dt, int best_trajectory_id)
    {
        rclcpp::Time now = this->now();

        // 節流發布
        if ((now - last_path_pub_time_).seconds() < path_pub_interval_)
            return;

        last_path_pub_time_ = now;
        visualization_msgs::msg::MarkerArray all_rollOuts;

        int v_samples = this->trajectory_num_; 

        // 安全檢查
        if (v_samples == 0 || twist_pairs.empty())
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "v_samples (trajectory_num_) or twist_pairs is zero. Cannot visualize paths.");
            return;
        }

        // 2. 檢查總數是否匹配
        if (twist_pairs.size() % v_samples != 0)
        {
            RCLCPP_WARN(this->get_logger(), 
                "Path visualization mismatch: twist_pairs.size() (%ld) is not a multiple of v_samples (trajectory_num_) (%d). Visualization might be incorrect.", 
                twist_pairs.size(), v_samples);
        }
        
        // 3. 計算 w_samples (角速度量)
        int w_samples = twist_pairs.size() / v_samples; //

        if (w_samples == 0) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Calculated w_samples is zero. Cannot visualize.");
            return;
        }

        // 4. 找出 "最後一個 v" 的起始索引
        // v 的索引從 0 到 (v_samples - 1)
        // 我們要的是最後一個 v，其索引為 (v_samples - 1)
        int last_v_index = v_samples - 1;       // 例如: 10 - 1 = 9
        int start_index = last_v_index * w_samples; // 例如: 9 * 20 = 180


        std::set<size_t> visualize_indices;

        // 5. 將這最後一個 v 對應的所有 w 索引加入
        //    (索引範圍: 180 到 180 + 20 - 1 = 199)
        for (int i = 0; i < w_samples; ++i)
        {
            size_t index_to_add = start_index + i;
            visualize_indices.insert(index_to_add);
        }
        // --- 修正結束 ---


        // 確保最佳軌跡包含 (這行不變)
        visualize_indices.insert(best_trajectory_id);

        for (int i : visualize_indices)
        {
            visualization_msgs::msg::Marker lane;
            lane.header.frame_id = robot_frame_;
            // lane.header.stamp = now;
            lane.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
            lane.ns = "dwa_paths";
            lane.id = i;
            lane.type = visualization_msgs::msg::Marker::LINE_STRIP;
            lane.action = visualization_msgs::msg::Marker::ADD;
            lane.scale.x = 0.02;
            lane.lifetime = rclcpp::Duration::from_seconds(0.7);
            // RCLCPP_INFO(this->get_logger(), "i = %d, twist_pairs = %f, %f", i, twist_pairs[i](0), twist_pairs[i](1));
            double step_dis = (twist_pairs[i](0)  * dt) / trajectory_point_num_;
            double step_theta = (twist_pairs[i](1)  * dt) / trajectory_point_num_;

            double s_x = 0.0, s_y = 0.0, theta = step_theta;
            for (int j = 0; j < trajectory_point_num_; j++)
            {
                geometry_msgs::msg::Point p;
                s_x += step_dis * cos(theta);
                s_y += step_dis * sin(theta);
                p.x = s_x;
                p.y = s_y;
                p.z = 0.0;
                lane.points.push_back(p);
                theta += step_theta;
            }

            if (i == best_trajectory_id)
            {
                lane.color.r = 0.0; lane.color.g = 1.0; lane.color.b = 0.0; lane.color.a = 1.0;
            }
            else
            {
                lane.color.r = 1.0; lane.color.g = 0.0; lane.color.b = 1.0; lane.color.a = 0.3;
            }

            all_rollOuts.markers.push_back(lane);
        }

        path_marker_pub_->publish(all_rollOuts);
    }



    //-----------------------------------------------------------------------------------------------
    bool ServiceCallback(const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> req,
                        std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> res)
    {
        static int last_mode = 0;

        action_flag_ = req->action.data;
        direction_inverse_ = req->direction_inverse.data;
        enable_costmap_obstacle_ = req->obstacle_avoidance.data;
        max_linear_velocity_ = req->speed_parameter.linear.x;
        min_linear_velocity_ = -req->speed_parameter.linear.x;
        max_angular_velocity_ = req->speed_parameter.angular.z;
        min_angular_velocity_ = -req->speed_parameter.angular.z;

        path_sub_mode_ = req->mode;

        RCLCPP_INFO(this->get_logger(), "Receive planner function request: action=%d, mode=%d", action_flag_, path_sub_mode_);

        // if(!action_flag_ || path_sub_mode_ != last_mode)
        // {
        //     arriving_end_point_ = false;
        //     arriving_end_direction_ = false;
        //     get_costmap_data_ = false;
        // }
        arriving_end_point_ = false;
        arriving_end_direction_ = false;
        get_costmap_data_ = false;

        last_mode = path_sub_mode_;
        
        return true;
    }

    //-----------------------------------------------------------------------------------------------
    void ElevatorStatusCheckCallService(rclcpp::Client<campusrover_msgs::srv::ElevatorStatusChecker>::SharedPtr client,
                                        campusrover_msgs::srv::ElevatorStatusChecker::Request &srv)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        RCLCPP_INFO(this->get_logger(), "Request message:\n");
        
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call elevator status service");
        }

        auto result_future = client->async_send_request(std::make_shared<campusrover_msgs::srv::ElevatorStatusChecker::Request>(srv));
        // 注意：在 ROS2 你可以用回調或 future 取得結果
    }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DWANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
