/**
 * @file laser_to_pointcloud2.cpp
 * @brief 雷射掃描轉點雲節點 (ROS2 Foxy 版本)
 * @description 將雷射掃描數據轉換為點雲格式，供成本地圖生成器使用
 * @target Ubuntu 20.04 + ROS2 Foxy
 */

#include <rclcpp/rclcpp.hpp>
#include <laser_geometry/laser_geometry.hpp>    // Foxy 版本使用 .h
#include <sensor_msgs/msg/laser_scan.hpp>     // 雷射掃描消息
#include <sensor_msgs/msg/point_cloud2.hpp>  // 點雲消息

/**
 * @class LaserToPointCloud2Node
 * @brief 雷射轉點雲節點類 (ROS2 版本)
 */
class LaserToPointCloud2Node : public rclcpp::Node
{
public:
  /**
   * @brief 建構函數
   */
  LaserToPointCloud2Node() : Node("laser_to_pointcloud2_node")
  {
    // 初始化發布者和訂閱者
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "laser", 10,
      std::bind(&LaserToPointCloud2Node::scanCallback, this, std::placeholders::_1));
  }

private:
  /**
   * @brief 雷射掃描回調函數 (ROS2 版本)
   * @param scan_in 接收到的雷射掃描數據
   * @description 將雷射掃描數據投影為點雲並發布
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in)
  {
    static int div_time = 0;  // 分頻計數器 (目前未使用)
    
    // 將雷射掃描投影為點雲
    sensor_msgs::msg::PointCloud2 cloud;
    projector_.projectLaser(*scan_in, cloud);
    
    // 發布轉換後的點雲
    cloud_pub_->publish(cloud);
    div_time = 0;
  }

  // 成員變數
  laser_geometry::LaserProjection projector_;                                        // 雷射投影器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;           // 點雲發布者
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;          // 雷射掃描訂閱者
};

/**
 * @brief 主函數 (ROS2 版本)
 * @param argc 命令行參數數量
 * @param argv 命令行參數陣列
 * @return 程序退出碼
 * @description 初始化雷射轉點雲節點
 */
int main(int argc, char** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  
  // 創建並運行節點
  auto node = std::make_shared<LaserToPointCloud2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}