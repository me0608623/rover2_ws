/**
 * @file local_costmap_node.cpp
 * @brief 局部成本地圖節點主程序 (ROS2 版本)
 * @description 局部成本地圖節點的主入口，負責初始化和啟動局部成本地圖生成器
 */

#include "campusrover_costmap.h"

#include <pcl/visualization/cloud_viewer.h>    // PCL 點雲可視化
#include <pcl/visualization/pcl_visualizer.h>  // PCL 可視化器

/**
 * @brief PCL 可視化回調函數
 * @param viz PCL 可視化器引用
 * @description 在 PCL 視窗中顯示點雲數據和機器人模型
 */
void PclVizCallback(pcl::visualization::PCLVisualizer& viz)
{
  static bool first_time = true;
  
  // 首次執行時添加機器人模型和參考線
  if(first_time)
  {
    pcl::PointXYZ p1(0, 0, 0);      // 原點
    pcl::PointXYZ p2(5, 0, 0);      // X 軸方向 5 公尺處
    
    // 添加機器人本體的立方體表示 (長1.6m, 寬0.9m, 高1.1m)
    viz.addCube(0, 1.6, -0.45, 0.45, 0, 1.1);
    
    // 添加 X 軸參考線 (紅色)
    viz.addLine (p1, p2, 255, 0,0);
    first_time = false;
  }
  
  // 顯示所有待可視化的點雲
  for (int i = 0; i < g_cloud_viz_list.size(); i++)
  {
    PointCloudPtr cloud_pass = g_cloud_viz_list[i];
    
    // 設定點雲顯示屬性
    viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    
    // 設定點雲顏色為白色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (cloud_pass, 255, 255, 255);
    
    if(cloud_pass == NULL) return;
    
    // 更新或添加點雲到可視化器
    if (!viz.updatePointCloud (cloud_pass, color, "cloud"))
    {
      viz.addPointCloud (cloud_pass,color, "cloud");
      viz.resetCameraViewpoint ("cloud");  // 重置攝像機視角
    }
  }
  
  // 清空可視化列表，準備下一幀
  g_cloud_viz_list.clear();
}

/**
 * @brief 主函數 (ROS2 版本)
 * @param argc 命令行參數數量
 * @param argv 命令行參數陣列
 * @return 程序退出碼
 * @description 初始化局部成本地圖節點，可選啟動 PCL 可視化
 */
int main(int argc, char** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  
  // 創建局部成本地圖生成器節點
  auto campusrover_costmap = std::make_shared<CampusroverCostmap>();
  
  // 如果啟用 PCL 可視化，創建可視化視窗
  if(g_pcl_viz)
  {
    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL Viewer");
    viewer_->runOnVisualizationThread (std::bind(&PclVizCallback, std::placeholders::_1), "g_pcl_vizcb");
  }
  
  // 啟動 ROS2 事件循環
  rclcpp::spin(campusrover_costmap);
  rclcpp::shutdown();
  return 0;
}
