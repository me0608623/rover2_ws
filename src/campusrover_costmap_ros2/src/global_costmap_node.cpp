/**
 * @file global_costmap_node.cpp
 * @brief 全域成本地圖節點主程序 (ROS2 版本)
 * @description 全域成本地圖節點的主入口，負責初始化和啟動全域成本地圖生成器
 */

#include <global_costmap.hpp>

/**
 * @brief 主函數 (ROS2 版本)
 * @param argc 命令行參數數量
 * @param argv 命令行參數陣列
 * @return 程序退出碼
 * @description 初始化 ROS2 節點，創建全域成本地圖生成器並啟動服務
 */
int main(int argc, char **argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  
  // 創建全域成本地圖生成器節點
  auto global_costmap_node = std::make_shared<GlobalCostmap>();
  
  // 啟動 ROS2 事件循環
  rclcpp::spin(global_costmap_node);
  rclcpp::shutdown();
  return 0;
}
