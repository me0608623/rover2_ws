/**
 * @file campusrover_costmap.h
 * @brief Campus Rover 局部成本地圖生成器頭文件 (ROS2 Foxy 版本)
 * @description 此文件定義了 CampusroverCostmap 類，用於從點雲數據生成局部成本地圖
 * @target Ubuntu 20.04 + ROS2 Foxy
 * @author Campus Rover Team
 * @date 2024
 */

#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <memory>

// ROS2 核心
#include <rclcpp/rclcpp.hpp>

// PCL 相關
#include <pcl_conversions/pcl_conversions.h>  
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


#include <pcl/filters/passthrough.h>  // 通過濾波器，用於高度過濾
#include <pcl/filters/crop_box.h>     // 裁剪盒濾波器，用於範圍限制
#include <pcl/filters/voxel_grid.h>   // 體素網格濾波器，用於降採樣

// TF2 - 座標變換相關
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// ROS2 消息類型
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>    // 占用網格地圖消息
#include <sensor_msgs/msg/point_cloud2.hpp>  // 點雲消息

// Eigen - 線性代數庫
#include <Eigen/Dense>
#include <Eigen/Core>

// PCL 轉換
#include <pcl_conversions/pcl_conversions.h>

// 多線程處理的線程數量
#define NUM_THRE1AD 8

// 熱力圖顯示相關常數
static const double HEAT_MIN = 1.6;      // 熱力圖最小值
static const double HEAT_MAX = 20;       // 熱力圖最大值
static const double COLOR_MAP_SIZE = 20; // 顏色映射大小

// 成本地圖中的標準值定義 (遵循 ROS costmap_2d 標準)
// static const unsigned char NO_INFORMATION = 255;           // 未知區域
// static const unsigned char LETHAL_OBSTACLE = 254;          // 致命障礙物
// static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 254; // 膨脹障礙物
static const unsigned char FREE_SPACE = 0;                 // 自由空間

// 點雲數據類型定義
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;        // 基本 XYZ 點雲
typedef PointCloud::Ptr PointCloudPtr;                    // 點雲智能指針
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;  // 帶顏色的點雲

// PCL 可視化相關全域變數 (只宣告，不定義)
extern std::vector<PointCloudPtr> g_cloud_viz_list;  // 用於可視化的點雲列表
extern bool g_pcl_viz;                               // 是否啟用 PCL 可視化

/**
 * @class CampusroverCostmap
 * @brief Campus Rover 局部成本地圖生成器 (ROS2 版本)
 * @description 從點雲數據生成局部成本地圖，用於機器人導航避障
 */
class CampusroverCostmap : public rclcpp::Node
{
  public:
    /**
     * @brief 建構函數
     * @description 初始化 ROS2 節點和所有參數
     */
    CampusroverCostmap();

  private:
    // ROS2 發布者和訂閱者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;  // 點雲數據訂閱者
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;    // 成本地圖發布者
    
    // TF2 相關
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                   // TF2 監聽器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                               // TF2 緩衝區
    
    // ROS 參數 - 通用設定
    std::string base_frame_;               // 基礎座標系名稱
    
    // ROS 參數 - 地圖設定
    double map_size_width_;                // 地圖寬度 (公尺)
    double map_size_height_;               // 地圖高度 (公尺)
    double map_update_frequency_;          // 地圖更新頻率 (Hz)
    double map_resolution_;                // 地圖解析度 (公尺/像素)
    
    // ROS 參數 - 障礙物檢測設定
    double max_obstacle_height_;           // 障礙物最大高度 (公尺)
    double min_obstacle_height_;           // 障礙物最小高度 (公尺)
    
    // ROS 參數 - 膨脹設定
    double inflation_radius_;              // 膨脹半徑 (公尺)
    double cost_scaling_factor_;           // 成本縮放因子

    /**
     * @brief 點雲數據回調函數 (ROS2 版本)
     * @param msg 接收到的點雲消息
     * @description 處理點雲數據並生成成本地圖
     */
    void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief 發布點雲數據 (ROS2 版本)
     * @param pub ROS2 發布者
     * @param pt 要發布的點
     */
    void pubPointcloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, pcl::PointXYZ& pt);

    /**
     * @brief 通過濾波器 (已棄用，使用 cropBoxFilter 替代)
     * @param cloud_in 輸入點雲
     * @param cloud_out 輸出點雲
     * @description 根據高度範圍過濾點雲
     */
    void passThroughFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out);

    /**
     * @brief 裁剪盒濾波器
     * @param cloud_in 輸入點雲
     * @param cloud_out 輸出點雲
     * @description 移除機器人本體範圍內的點以及超出地圖範圍的點
     */
    void cropBoxFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out);

    /**
     * @brief 體素網格濾波器
     * @param cloud_in 輸入點雲
     * @param cloud_out 輸出點雲
     * @description 對點雲進行降採樣，減少計算量
     */
    void voxelGridFliter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out);

    /**
     * @brief 將點雲轉換為成本地圖 (ROS2 版本)
     * @param cloud_in 輸入點雲
     * @param occupancy_grid 輸出的占用網格地圖
     * @description 將過濾後的點雲轉換為 2D 成本地圖格式
     */
    void toCostmap( PointCloudPtr &cloud_in, nav_msgs::msg::OccupancyGrid &occupancy_grid);

    /**
     * @brief 獲取成本矩陣
     * @param obstacle_list 障礙物索引列表
     * @param width 地圖寬度
     * @param height 地圖高度
     * @param num_thread 線程數量
     * @return 成本矩陣
     * @description 使用多線程計算障礙物膨脹後的成本矩陣
     */
    Eigen::MatrixXd getCostMat(const std::vector<Eigen::Vector2i> obstacle_list, 
                               int width, int height, int num_thread);
    
    /**
     * @brief 計算矩陣距離
     * @param obstacle_list 障礙物列表
     * @param dst_mat 目標矩陣
     * @param min_i 最小索引
     * @param max_i 最大索引
     * @description 計算每個網格到最近障礙物的距離 (多線程工作函數)
     */
    void computeMatDis(const std::vector<Eigen::Vector2i> obstacle_list, 
                       Eigen::MatrixXd &dst_mat,
                       int min_i, int max_i);

    /**
     * @brief 計算成本值
     * @param dis 到障礙物的距離
     * @return 成本值
     * @description 根據距離計算對應的成本值，距離越近成本越高
     */
    // double computeCost(const double dis);
    int8_t computeCost(const double dis);

    nav_msgs::msg::OccupancyGrid costmap_; 
};