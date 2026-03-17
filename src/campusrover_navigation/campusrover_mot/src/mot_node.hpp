#ifndef MOT_NODE_HPP
#define MOT_NODE_HPP

#include "mot.hpp"

#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <campusrover_msgs/srv/img_label.hpp>
#include <campusrover_msgs/msg/tracked_obstacle.hpp>
#include <campusrover_msgs/msg/tracked_obstacle_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>

using namespace std;

// Helper: apply geometry_msgs::msg::Transform to pcl::PointCloud using Eigen
inline void transformPointCloudWithTransform(
    const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
    pcl::PointCloud<pcl::PointXYZ> &cloud_out,
    const geometry_msgs::msg::Transform &transform)
{
    Eigen::Affine3d tf_eigen = Eigen::Affine3d::Identity();
    tf_eigen.translation() << transform.translation.x,
                              transform.translation.y,
                              transform.translation.z;
    Eigen::Quaterniond q(transform.rotation.w,
                         transform.rotation.x,
                         transform.rotation.y,
                         transform.rotation.z);
    tf_eigen.rotate(q);
    pcl::transformPointCloud(cloud_in, cloud_out, tf_eigen);
}

class MotNode : public rclcpp::Node {
public:
    MotNode();

private:
    // --- Algorithm objects ---
    ExtendedObjectTrackingParam eot_param;
    TrackerParam t_param;

    ExtendedObjectTracking<MeanCovTracker> *eot;
    std::mutex eot_mutex;

    Eigen::MatrixXd env_data, env_data_, now_pose, now_pose_;
    std::mutex env_data_mutex;

    sensor_msgs::msg::PointCloud2 env_laser_pcl;
    std::mutex env_laser_pcl_mutex;

    sensor_msgs::msg::Image env_img, env_img_;
    std::mutex env_img_mutex;

    nav_msgs::msg::OccupancyGrid costmap;
    std::mutex costmap_mutex;

    bool is_pcl_laser_sync = false, is_pcl_img_sync = false, is_init_pcm = false, enable_mot = true;

    // --- ROS param ---
    bool debug_mode, is_use_laser, is_map_filter, is_img_label, only_dynamic_obstacle;
    double min_publish_age;
    int min_detection_count;
    double h_scale, v_scale;
    double sync_tolerate, tf_tolerate;
    double laser_data_update_time = 0.0, env_data_update_time = 0.0;
    double trackers_update_period, label_update_period;
    string map_frame, laser_frame, camera_frame;
    geometry_msgs::msg::TransformStamped map_2_camera_tf;
    image_geometry::PinholeCameraModel pcm;

    // --- ROS interfaces ---
    rclcpp::Publisher<campusrover_msgs::msg::TrackedObstacleArray>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_1;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tracked_cloud;
    rclcpp::Client<campusrover_msgs::srv::ImgLabel>::SharedPtr cli;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub3;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub4;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub5;

    rclcpp::TimerBase::SharedPtr trackers_update_timer;
    rclcpp::TimerBase::SharedPtr label_update_timer;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // --- Methods ---
    void get_param();

    double get_dim(std::pair<BoundaryPt, BoundaryPt> pt_h, std::pair<BoundaryPt, BoundaryPt> pt_v, Eigen::MatrixXd mean);
    void map_filter(const nav_msgs::msg::OccupancyGrid &map, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);

    void img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void laser_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void pcl_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void enable_mot_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void trackers_update_callback();
    void label_update_callback();
};

// ===== Implementation =====

void MotNode::get_param(){
    this->declare_parameter<double>("detection_area_min_x", 0.0);
    this->declare_parameter<double>("detection_area_max_x", 20.0);
    this->declare_parameter<double>("detection_area_min_y", -20.0);
    this->declare_parameter<double>("detection_area_max_y", 20.0);
    this->declare_parameter<double>("detection_area_min_z", -1.0);
    this->declare_parameter<double>("detection_area_max_z", 3.0);
    this->declare_parameter<double>("track_dead_time", 2.0);
    this->declare_parameter<double>("track_older_age", 2.0);
    this->declare_parameter<double>("cluster_dist", 1.0);
    this->declare_parameter<int>("false_alarm_min", 10);
    this->declare_parameter<int>("false_alarm_max", 1000);

    this->declare_parameter<double>("weight_min_tolerate", 0.01);
    this->declare_parameter<double>("cov_scale", 6.0);
    this->declare_parameter<double>("inherit_ratio", 0.8);

    this->declare_parameter<int>("history_length", 10);
    this->declare_parameter<double>("anchor_dist_threshold", 0.1);
    this->declare_parameter<double>("speed_threshold", 0.1);

    this->declare_parameter<double>("trackers_update_period", 0.05);
    this->declare_parameter<double>("label_update_period", 0.1);

    this->declare_parameter<string>("map_frame", "map");
    this->declare_parameter<string>("laser_frame", "laser");
    this->declare_parameter<string>("camera_frame", "camera");

    this->declare_parameter<double>("h_scale", 2.0);
    this->declare_parameter<double>("v_scale", 1.5);
    this->declare_parameter<double>("sync_tolerate", 0.05);
    this->declare_parameter<double>("tf_tolerate", 0.1);

    this->declare_parameter<bool>("debug_mode", true);
    this->declare_parameter<bool>("is_use_laser", false);
    this->declare_parameter<bool>("is_map_filter", false);
    this->declare_parameter<bool>("is_img_label", false);
    this->declare_parameter<bool>("only_dynamic_obstacle", true);
    this->declare_parameter<double>("min_publish_age", 0.3);
    this->declare_parameter<int>("min_detection_count", 3);

    eot_param.detection_area[0] = this->get_parameter("detection_area_min_x").as_double();
    eot_param.detection_area[1] = this->get_parameter("detection_area_max_x").as_double();
    eot_param.detection_area[2] = this->get_parameter("detection_area_min_y").as_double();
    eot_param.detection_area[3] = this->get_parameter("detection_area_max_y").as_double();
    eot_param.detection_area[4] = this->get_parameter("detection_area_min_z").as_double();
    eot_param.detection_area[5] = this->get_parameter("detection_area_max_z").as_double();
    eot_param.track_dead_time   = this->get_parameter("track_dead_time").as_double();
    eot_param.track_older_age   = this->get_parameter("track_older_age").as_double();
    eot_param.cluster_dist      = this->get_parameter("cluster_dist").as_double();
    eot_param.false_alarm[0]    = this->get_parameter("false_alarm_min").as_int();
    eot_param.false_alarm[1]    = this->get_parameter("false_alarm_max").as_int();

    eot_param.weight_min_tolerate = this->get_parameter("weight_min_tolerate").as_double();
    eot_param.cov_scale           = this->get_parameter("cov_scale").as_double();
    eot_param.inherit_ratio       = this->get_parameter("inherit_ratio").as_double();

    t_param.history_length        = this->get_parameter("history_length").as_int();
    t_param.anchor_dist_threshold = this->get_parameter("anchor_dist_threshold").as_double();
    t_param.speed_threshold       = this->get_parameter("speed_threshold").as_double();

    trackers_update_period = this->get_parameter("trackers_update_period").as_double();
    label_update_period    = this->get_parameter("label_update_period").as_double();

    map_frame    = this->get_parameter("map_frame").as_string();
    laser_frame  = this->get_parameter("laser_frame").as_string();
    camera_frame = this->get_parameter("camera_frame").as_string();

    h_scale       = this->get_parameter("h_scale").as_double();
    v_scale       = this->get_parameter("v_scale").as_double();
    sync_tolerate = this->get_parameter("sync_tolerate").as_double();
    tf_tolerate   = this->get_parameter("tf_tolerate").as_double();

    debug_mode            = this->get_parameter("debug_mode").as_bool();
    is_use_laser          = this->get_parameter("is_use_laser").as_bool();
    is_map_filter         = this->get_parameter("is_map_filter").as_bool();
    is_img_label          = this->get_parameter("is_img_label").as_bool();
    only_dynamic_obstacle = this->get_parameter("only_dynamic_obstacle").as_bool();
    min_publish_age       = this->get_parameter("min_publish_age").as_double();
    min_detection_count   = this->get_parameter("min_detection_count").as_int();

    eot_param.split_false_alarm[1] = eot_param.false_alarm[1];
}

double MotNode::get_dim(std::pair<BoundaryPt, BoundaryPt> pt_h, std::pair<BoundaryPt, BoundaryPt> pt_v, Eigen::MatrixXd mean){
    double dist_h1 = pow(pt_h.first.pt_3X1(0, 0) - mean(0, 0), 2) + pow(pt_h.first.pt_3X1(1, 0) - mean(1, 0), 2) + pow(pt_h.first.pt_3X1(2, 0) - mean(2, 0), 2);
    double dist_h2 = pow(pt_h.second.pt_3X1(0, 0) - mean(0, 0), 2) + pow(pt_h.second.pt_3X1(1, 0) - mean(1, 0), 2) + pow(pt_h.second.pt_3X1(2, 0) - mean(2, 0), 2);
    double dist_v1 = pow(pt_v.first.pt_3X1(0, 0) - mean(0, 0), 2) + pow(pt_v.first.pt_3X1(1, 0) - mean(1, 0), 2) + pow(pt_v.first.pt_3X1(2, 0) - mean(2, 0), 2);
    double dist_v2 = pow(pt_v.second.pt_3X1(0, 0) - mean(0, 0), 2) + pow(pt_v.second.pt_3X1(1, 0) - mean(1, 0), 2) + pow(pt_v.second.pt_3X1(2, 0) - mean(2, 0), 2);

    double dist_h = (dist_h1 > dist_h2) ? dist_h1 : dist_h2;
    double dist_v = (dist_v1 > dist_v2) ? dist_v1 : dist_v2;

    return (dist_h > dist_v) ? sqrt(dist_h) : sqrt(dist_v);
}

void MotNode::map_filter(const nav_msgs::msg::OccupancyGrid &map, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out){
    uint32_t width = map.info.width;
    uint32_t height = map.info.height;
    float res = map.info.resolution;
    geometry_msgs::msg::Point offset = map.info.origin.position;

    int size = cloud_in.points.size();
    for(int i = 0; i < size; i++){
        int idx_row = (cloud_in.points[i].y-offset.y)/res;
        int idx_col = (cloud_in.points[i].x-offset.x)/res;

        if(idx_col >= 0 && idx_col < (int)width && idx_row >= 0 && idx_row < (int)height){
            int costmap_idx = idx_row*width+idx_col;

            if(map.data[costmap_idx] == 0 || map.data[costmap_idx] == -1){
                cloud_out.points.push_back(cloud_in.points[i]);
            }
        }
    }
    cloud_out.width  = cloud_out.points.size();
    cloud_out.height = 1;
}

void MotNode::img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    if(fabs(env_data_update_time - rclcpp::Time(msg->header.stamp).seconds()) <= sync_tolerate){
        env_img_mutex.lock();
        env_img = *msg;
        env_img_mutex.unlock();
        is_pcl_img_sync = true;
    }else{
        is_pcl_img_sync = false;
    }
}

void MotNode::laser_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    laser_geometry::LaserProjection projector;
    env_laser_pcl_mutex.lock();
    laser_data_update_time = rclcpp::Time(msg->header.stamp).seconds();
    projector.projectLaser(*msg, env_laser_pcl);
    env_laser_pcl_mutex.unlock();
}

void MotNode::pcl_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    bool is_get_tf = true;
    geometry_msgs::msg::TransformStamped tf_stamped;
    geometry_msgs::msg::TransformStamped tf_stamped_laser;

    try{
        tf_stamped = tf_buffer->lookupTransform(map_frame, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(tf_tolerate));
        if(is_use_laser){
            tf_stamped_laser = tf_buffer->lookupTransform(msg->header.frame_id, laser_frame, tf2::TimePointZero, tf2::durationFromSec(tf_tolerate));
        }
        if(is_img_label){
            map_2_camera_tf = tf_buffer->lookupTransform(camera_frame, map_frame, tf2::TimePointZero, tf2::durationFromSec(tf_tolerate));
        }

    }catch(tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        is_get_tf = false;
    }

    if(is_get_tf){
        Eigen::MatrixXd matrix_data;
        pcl::PointCloud<pcl::PointXYZ> pcl_data, laser_data, laser_data_, box_filter_data, map_filter_data;

        env_data_update_time = rclcpp::Time(msg->header.stamp).seconds();
        pcl::fromROSMsg(*msg, pcl_data);
        if(is_use_laser){
            if(fabs(laser_data_update_time - env_data_update_time) <= sync_tolerate){
                pcl::fromROSMsg(env_laser_pcl, laser_data);
                transformPointCloudWithTransform(laser_data, laser_data_, tf_stamped_laser.transform);
                for(size_t i = 0; i < laser_data_.size(); i++){
                    pcl_data.push_back(laser_data_[i]);
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "NO sync pcl & laser");
            }
        }

        BoxFilter::pcl_box_filter(pcl_data, box_filter_data, eot_param.detection_area);
        transformPointCloudWithTransform(box_filter_data, pcl_data, tf_stamped.transform);

        if(is_map_filter){
            costmap_mutex.lock();
            this->map_filter(costmap, pcl_data, map_filter_data);
            costmap_mutex.unlock();

        }else{
            map_filter_data = pcl_data;
        }

        DataTransform::pcl2matrix(map_filter_data, matrix_data);

        now_pose = Eigen::MatrixXd::Zero(3, 1);
        env_data_mutex.lock();
        env_data = matrix_data;
        now_pose << tf_stamped.transform.translation.x,
                    tf_stamped.transform.translation.y,
                    tf_stamped.transform.translation.z;
        env_data_mutex.unlock();

        if(debug_mode){
            sensor_msgs::msg::PointCloud2 pub_msg;
            pcl::toROSMsg(map_filter_data, pub_msg);
            pub_msg.header = msg->header;
            pub_msg.header.frame_id = map_frame;
            pub_1->publish(pub_msg);
        }
    }else{
        env_data_mutex.lock();
        env_data = Eigen::MatrixXd::Zero(0, 3);
        env_data_mutex.unlock();
    }
}

void MotNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
    if(!is_init_pcm){
        pcm.fromCameraInfo(msg);
        is_init_pcm = true;
    }
}

void MotNode::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    costmap_mutex.lock();
    costmap = *msg;
    costmap_mutex.unlock();
}

void MotNode::enable_mot_callback(const std_msgs::msg::Bool::SharedPtr msg){
    enable_mot = msg->data;
}

void MotNode::trackers_update_callback(){
    if(enable_mot){
        env_data_mutex.lock();
        env_data_ = env_data;
        now_pose_ = now_pose;
        env_data_mutex.unlock();

        env_img_mutex.lock();
        env_img_ = env_img;
        env_img_mutex.unlock();

        if(env_data_.rows() != 0){
            eot_mutex.lock();
            eot->update(env_data_, now_pose_, this->now().seconds(), trackers_update_period, false);
            eot_mutex.unlock();
        }

        campusrover_msgs::msg::TrackedObstacleArray toa;
        double cur_time = this->now().seconds();
        for(size_t i = 0; i < eot->trackers.size(); i++){
            // 過濾未確認的 tracker：必須存活足夠久且被偵測足夠多次
            double age = cur_time - eot->trackers[i].born_time;
            if(age < min_publish_age || eot->trackers[i].detection_count < min_detection_count){
                continue;
            }

            if(only_dynamic_obstacle){
                if(eot->trackers[i].status == ObjectStatus::DYNAMIC){
                    campusrover_msgs::msg::TrackedObstacle to;

                    to.id = eot->trackers[i].uid;
                    to.label = eot->trackers[i].label;
                    to.pose.position.x = eot->trackers[i].mean_3X1(0, 0);
                    to.pose.position.y = eot->trackers[i].mean_3X1(1, 0);
                    to.pose.position.z = eot->trackers[i].mean_3X1(2, 0);
                    to.velocity.linear.x = eot->trackers[i].speed_2X1(0,0);
                    to.velocity.linear.y = eot->trackers[i].speed_2X1(1,0);

                    to.dimensions.x = get_dim(eot->trackers[i].h_fov_range, eot->trackers[i].v_fov_range, eot->trackers[i].mean_3X1);

                    toa.obstacles.push_back(to);
                    toa.header.frame_id = map_frame;
                    toa.header.stamp = this->now();
                }
            }else{
                campusrover_msgs::msg::TrackedObstacle to;

                to.id = eot->trackers[i].uid;
                to.label = eot->trackers[i].label;
                to.pose.position.x = eot->trackers[i].mean_3X1(0, 0);
                to.pose.position.y = eot->trackers[i].mean_3X1(1, 0);
                to.pose.position.z = eot->trackers[i].mean_3X1(2, 0);
                to.velocity.linear.x = eot->trackers[i].speed_2X1(0,0);
                to.velocity.linear.y = eot->trackers[i].speed_2X1(1,0);
                to.is_dynamic = eot->trackers[i].status == ObjectStatus::DYNAMIC;

                to.dimensions.x = get_dim(eot->trackers[i].h_fov_range, eot->trackers[i].v_fov_range, eot->trackers[i].mean_3X1);

                toa.obstacles.push_back(to);
                toa.header.frame_id = map_frame;
                toa.header.stamp = this->now();
            }
        }

        pub->publish(toa);

        // Publish tracked obstacle points as blue colored point cloud
        {
            pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
            eot_mutex.lock();
            for(size_t i = 0; i < eot->trackers.size(); i++){
                uint8_t r = 0, g = 0, b = 255;  // blue for static
                if(eot->trackers[i].status == ObjectStatus::DYNAMIC){
                    r = 255; g = 0; b = 0;  // red for dynamic
                }
                const auto &pts = eot->trackers[i].points;
                for(int j = 0; j < pts.rows(); j++){
                    pcl::PointXYZRGB p;
                    p.x = pts(j, 0);
                    p.y = pts(j, 1);
                    p.z = pts(j, 2);
                    p.r = r; p.g = g; p.b = b;
                    colored_cloud.push_back(p);
                }
            }
            eot_mutex.unlock();

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(colored_cloud, cloud_msg);
            cloud_msg.header.frame_id = map_frame;
            cloud_msg.header.stamp = this->now();
            pub_tracked_cloud->publish(cloud_msg);
        }
    }
}

void MotNode::label_update_callback(){
    static int now_idx = 0;

    if(is_pcl_img_sync && is_init_pcm && eot->trackers.size() > 0 && cli->service_is_ready()){
        int now_uid, count = 0;
        bool is_call_server = true;
        Eigen::MatrixXd pt_h_v(4,3);
        pcl::PointCloud<pcl::PointXYZ> pts, pts_;

        eot_mutex.lock();
        for(size_t i = 0; i < eot->trackers.size(); i++){
            now_idx = (now_idx < (int)eot->trackers.size()) ? now_idx : 0;

            if(eot->trackers[now_idx].status != ObjectStatus::DYNAMIC){
                now_idx++;
                if(i == eot->trackers.size()-1){
                    is_call_server = false;
                }
            }else{
                now_uid = eot->trackers[now_idx].uid;
                pt_h_v << eot->trackers[now_idx].h_fov_range.first.pt_3X1.transpose(),
                        eot->trackers[now_idx].h_fov_range.second.pt_3X1.transpose(),
                        eot->trackers[now_idx].v_fov_range.first.pt_3X1.transpose(),
                        eot->trackers[now_idx].v_fov_range.second.pt_3X1.transpose();

                break;
            }
        }
        eot_mutex.unlock();
        now_idx++;

        if(is_call_server){
            DataTransform::matrix2pcl(pt_h_v, pts, false);

            transformPointCloudWithTransform(pts, pts_, map_2_camera_tf.transform);

            cv::Point3d cv_pt;
            std::vector<cv::Point2d> cv_pixel;

            for(int i = 0; i < 4; i++){
                cv_pt.x = -pts_[i].y;
                cv_pt.y = -pts_[i].z;
                cv_pt.z = pts_[i].x;
                cv_pixel.push_back(pcm.project3dToPixel(cv_pt));
            }

            int u0 = (int)round((cv_pixel[0].x+cv_pixel[1].x)/2 + (cv_pixel[0].x-cv_pixel[1].x)*h_scale/2);
            int u1 = (int)round((cv_pixel[0].x+cv_pixel[1].x)/2 - (cv_pixel[0].x-cv_pixel[1].x)*h_scale/2);
            int v0 = (int)round((cv_pixel[2].y+cv_pixel[3].y)/2 + (cv_pixel[2].y-cv_pixel[3].y)*v_scale/2);
            int v1 = (int)round((cv_pixel[2].y+cv_pixel[3].y)/2 - (cv_pixel[2].y-cv_pixel[3].y)*v_scale/2);

            if(u0 >= 0 && u1 < (int)env_img.width && v0 >=0 && v1 < (int)env_img.height){
                u0 = (u0 < (int)env_img.width) ? u0 : (int)env_img.width-1;
                u1 = (u1 >= 0) ? u1 : 0;
                v0 = (v0 < (int)env_img.height) ? v0 : (int)env_img.height-1;
                v1 = (v1 >= 0) ? v1 : 0;

                if(u0-u1 >= 32 && v0-v1 >= 32){
                    cv_bridge::CvImagePtr cv_ptr, cv_ptr_show;

                    cv_ptr = cv_bridge::toCvCopy(env_img_, env_img_.encoding);

                    if(debug_mode){
                        cv_ptr_show = cv_bridge::toCvCopy(env_img_, env_img_.encoding);
                    }

                    cv::Mat temp_img = cv_ptr->image(cv::Range(v1, v0), cv::Range(u1, u0));
                    cv_ptr->image = temp_img;

                    sensor_msgs::msg::Image temp_img_;
                    cv_ptr->toImageMsg(temp_img_);

                    auto request = std::make_shared<campusrover_msgs::srv::ImgLabel::Request>();
                    request->source_img = temp_img_;

                    auto future = cli->async_send_request(request);
                    auto status = future.wait_for(std::chrono::seconds(2));

                    if(status == std::future_status::ready){
                        auto response = future.get();
                        if(response->result.detections.size() > 0){
                            int label = 0;
                            try { label = std::stoi(response->result.detections[0].results[0].hypothesis.class_id); } catch(...) {}
                            std::string name = response->result.detections[0].header.frame_id;
                            cv::Point pt;

                            eot_mutex.lock();
                            for(size_t i = 0; i < eot->trackers.size(); i++){
                                if(now_uid == eot->trackers[i].uid){
                                    eot->trackers[i].label = label;
                                    eot->trackers[i].name = name;
                                    pt.x = (u1+u0)/2;
                                    pt.y = (v1+v0)/2;
                                    break;
                                }
                            }
                            eot_mutex.unlock();

                            if(debug_mode){
                                cv::circle(cv_ptr_show->image, pt, 2, cv::Scalar(0, 0, 255), 2);
                                cv::putText(cv_ptr_show->image, name, pt, cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

                                sensor_msgs::msg::Image pub_img;
                                cv_ptr_show->toImageMsg(pub_img);
                                pub_2->publish(pub_img);
                            }
                        }
                    }else{
                        RCLCPP_ERROR(this->get_logger(), "failed to call service, which is get label");
                    }
                }
            }
        }
    }else{
        if(!is_init_pcm){
            RCLCPP_INFO(this->get_logger(), "NO init camera info");
        }

        if(!is_pcl_img_sync){
            RCLCPP_INFO(this->get_logger(), "NO sync pcl & img");
        }
    }
}

MotNode::MotNode() : Node("mot_node"){
    get_param();

    pcm = image_geometry::PinholeCameraModel();

    eot = new ExtendedObjectTracking<MeanCovTracker>(this->now().seconds(), eot_param, t_param);

    // TF
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Publishers
    pub = this->create_publisher<campusrover_msgs::msg::TrackedObstacleArray>("tracked_label_obstacle", 10);
    pub_tracked_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tracked_obstacle_cloud", 10);
    if(debug_mode){
        pub_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_filter", 10);
        pub_2 = this->create_publisher<sensor_msgs::msg::Image>("/labeled_img", 10);
    }

    // Service client
    cli = this->create_client<campusrover_msgs::srv::ImgLabel>("img_lable_srv");

    // Subscribers
    sub0 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", 1, std::bind(&MotNode::pcl_sub_callback, this, std::placeholders::_1));
    if(is_use_laser){
        sub1 = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&MotNode::laser_sub_callback, this, std::placeholders::_1));
    }
    if(is_img_label){
        sub2 = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 1, std::bind(&MotNode::img_sub_callback, this, std::placeholders::_1));
        sub3 = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", 10, std::bind(&MotNode::camera_info_callback, this, std::placeholders::_1));
    }
    if(is_map_filter){
        sub4 = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap", 10, std::bind(&MotNode::costmap_callback, this, std::placeholders::_1));
    }
    sub5 = this->create_subscription<std_msgs::msg::Bool>(
        "/enable_mot", 10, std::bind(&MotNode::enable_mot_callback, this, std::placeholders::_1));

    // Timers
    trackers_update_timer = this->create_wall_timer(
        std::chrono::duration<double>(trackers_update_period),
        std::bind(&MotNode::trackers_update_callback, this));
    if(is_img_label){
        label_update_timer = this->create_wall_timer(
            std::chrono::duration<double>(label_update_period),
            std::bind(&MotNode::label_update_callback, this));
    }

    RCLCPP_INFO(this->get_logger(), "campusrover_mot_node started");
}

#endif
