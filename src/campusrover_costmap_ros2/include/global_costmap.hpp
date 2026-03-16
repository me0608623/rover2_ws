/**
 * @file global_costmap.hpp
 * @brief Campus Rover 全域成本地圖生成器頭文件 (ROS2 版本)
 * @description 此文件定義了 GlobalCostmap 類，用於從靜態地圖生成全域成本地圖
 * @author Campus Rover Team
 * @date 2024
 */

#ifndef GLABAL_COSTMAP_HPP
#define GLABAL_COSTMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>

/**
 * @class GlobalCostmap
 * @brief 全域成本地圖生成器 (ROS2 版本)
 * @description 從靜態地圖生成帶有膨脹障礙物的全域成本地圖，用於全域路徑規劃
 */
class GlobalCostmap : public rclcpp::Node
{
  // 成本地圖標準值定義
  enum Cost {LETHAL=254, FREE_SPACE=0, UNKNOW=-1};
private:
  nav_msgs::msg::OccupancyGrid costmap_;     // 內部成本地圖數據
  
  /**
   * @brief 獲取降採樣的成本地圖 (ROS2 版本)
   * @param original_map 原始地圖
   * @param downsampled_map 降採樣後的地圖
   * @param target_resolution 目標解析度
   * @description 將高解析度地圖降採樣到指定解析度
   */
  void getDownSampledCostmap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr original_map,
    nav_msgs::msg::OccupancyGrid &downsampled_map,
    const double target_resolution);
    
  /**
   * @brief 設定障礙物索引 (ROS2 版本)
   * @param obstacle_indexes 障礙物索引列表
   * @param costmap 成本地圖
   * @description 從成本地圖中提取所有障礙物的索引位置
   */
  void setObstacleIndexes(
    std::vector<std::pair<int, int>> &obstacle_indexes, 
    const nav_msgs::msg::OccupancyGrid &costmap );
    
  /**
   * @brief 計算成本地圖 (ROS2 版本)
   * @param costmap 成本地圖
   * @param obstacle_indexes 障礙物索引
   * @param inflation_radius 膨脹半徑
   * @param cost_scaling_factor 成本縮放因子
   * @description 對障礙物進行膨脹處理，生成最終的成本地圖
   */
  void computeCostmap(nav_msgs::msg::OccupancyGrid &costmap,
    std::vector<std::pair<int, int>> &obstacle_indexes,
    const double inflation_radius, 
    const double cost_scaling_factor);
    
  /**
   * @brief 計算成本值
   * @param dis 到障礙物的距離
   * @return 成本值
   * @description 根據距離計算對應的成本值
   */
  double computeCost(const double dis);
  
public:
  double inflation_radius_;           // 膨脹半徑 (公尺)
  double cost_scaling_factor_;        // 成本縮放因子
  double costmap_resolution_;         // 成本地圖解析度 (公尺/像素)
  
  // ROS2 發布者、訂閱者和定時器
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;    // 全域地圖訂閱者
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;   // 全域成本地圖發布者
  rclcpp::TimerBase::SharedPtr republish_timer_;                                     // 定期重發定時器
  
  /**
   * @brief 建構函數 (ROS2 版本)
   */
  GlobalCostmap();
  
  /**
   * @brief 解構函數
   */
  ~GlobalCostmap();
  
  /**
   * @brief 地圖更新回調函數 (ROS2 版本)
   * @param map 接收到的地圖消息
   * @description 當接收到新的靜態地圖時，生成對應的成本地圖
   */
  void mapUpdateCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
};

#endif // GLABAL_COSTMAP_HPP