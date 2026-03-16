/**
 * @file global_costmap.cpp
 * @brief 全域成本地圖實現 (ROS2 版本)
 * @description 實現從靜態地圖生成全域成本地圖的功能
 */

#include <global_costmap.hpp>

/**
 * @brief GlobalCostmap 建構函數 (ROS2 版本)
 * @description 初始化全域成本地圖生成器
 */
GlobalCostmap::GlobalCostmap() : Node("global_costmap_node")
{
  // 宣告 ROS2 參數
  this->declare_parameter("costmap_resolution", 0.5);
  this->declare_parameter("inflation_radius", 1.0);
  this->declare_parameter("cost_scaling_factor", 10.0);
  
  // 讀取參數值
  costmap_resolution_ = this->get_parameter("costmap_resolution").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
  cost_scaling_factor_ = this->get_parameter("cost_scaling_factor").as_double();
  
  // QoS: TRANSIENT_LOCAL 以匹配 nav2_map_server 的發布策略
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // 初始化發布者和訂閱者
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap", map_qos);
  global_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", map_qos,
    std::bind(&GlobalCostmap::mapUpdateCallback, this, std::placeholders::_1));

  // 定期重發 costmap，讓 VOLATILE 訂閱者（RViz、MOT）也能收到
  republish_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
    if (!costmap_.data.empty()) {
      costmap_.header.stamp = this->get_clock()->now();
      global_costmap_pub_->publish(costmap_);
    }
  });
}

/**
 * @brief GlobalCostmap 解構函數
 * @description 清理資源
 */
GlobalCostmap::~GlobalCostmap()
{
}

/**
 * @brief 獲取降採樣的成本地圖 (ROS2 版本)
 * @param original_map 原始高解析度地圖
 * @param downsampled_map 輸出的降採樣地圖
 * @param target_resolution 目標解析度
 * @description 將高解析度地圖降採樣到指定解析度，減少計算負擔
 */
void GlobalCostmap::getDownSampledCostmap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr original_map,
  nav_msgs::msg::OccupancyGrid &downsampled_map,
  const double target_resolution)
{
  // 計算降採樣比例
  double ratio = original_map->info.resolution/target_resolution;
  uint32_t down_sample_width = original_map->info.width*ratio; 
  uint32_t down_sample_height = original_map->info.height*ratio;
  
  // 設定新地圖的基本信息
  downsampled_map.info.resolution = target_resolution;
  downsampled_map.info.width = down_sample_width;
  downsampled_map.info.height = down_sample_height;
  
  // 初始化所有網格為自由空間
  for (uint32_t i = 0; i < down_sample_height; i++)
  {
    for (uint32_t j = 0; j < down_sample_width; j++)
    {
      downsampled_map.data.push_back(Cost::FREE_SPACE);
    }
  }
  
  // 將原始地圖的障礙物映射到新地圖
  for (uint32_t i = 0; i < original_map->info.height; i++)
  {
    for (uint32_t j = 0; j < original_map->info.width; j++)
    {
      // 計算在新地圖中的對應位置
      int down_i = i * ratio;
      int down_j = j * ratio;
      
      // 如果原始位置是障礙物，在新地圖中標記為障礙物
      if (original_map->data[i*original_map->info.width+j] > 0)
      {
        uint32_t idx = down_i*downsampled_map.info.width+down_j;
        downsampled_map.data[idx] = 100;  // 標記為障礙物
      }
    }
  }
}

/**
 * @brief 設定障礙物索引 (ROS2 版本)
 * @param obstacle_indexes 輸出的障礙物索引列表
 * @param costmap 輸入的成本地圖
 * @description 掃描整個地圖，找出所有障礙物的網格位置
 */
void GlobalCostmap::setObstacleIndexes(
    std::vector<std::pair<int, int>> &obstacle_indexes, 
    const nav_msgs::msg::OccupancyGrid &costmap )
{
  uint32_t h = costmap.info.height;
  uint32_t w = costmap.info.width;
  
  // 遍歷整個地圖，尋找障礙物
  for (uint32_t i = 0; i < h; i++)
  {
    for (uint32_t j = 0; j < w; j++)
    {
      // 如果網格值大於 80，視為障礙物 (ROS 占用網格標準)
      if (costmap.data[i*w+j] > 80)
      {
        obstacle_indexes.push_back(std::make_pair<int, int>(i, j));
      }
    }
  }
}

/**
 * @brief 計算全域成本地圖 (ROS2 版本)
 * @param costmap 要處理的成本地圖
 * @param obstacle_indexes 障礙物索引列表
 * @param inflation_radius 膨脹半徑
 * @param cost_scaling_factor 成本縮放因子
 * @description 對每個障礙物進行膨脹處理，在其周圍區域設定適當的成本值
 */
void GlobalCostmap::computeCostmap(nav_msgs::msg::OccupancyGrid &costmap,
  std::vector<std::pair<int, int>> &obstacle_indexes,
  const double inflation_radius, 
  const double cost_scaling_factor )
{
  const double map_res = costmap.info.resolution;    // 地圖解析度
  const int num_rad = inflation_radius / map_res;    // 膨脹半徑 (網格單位)
  const int max_h = costmap.info.height;             // 地圖高度
  const int max_w = costmap.info.width;              // 地圖寬度
  
  // 對每個障礙物進行膨脹處理
  for (std::pair<int, int> obstacle_idx: obstacle_indexes)
  {
    int cy = obstacle_idx.first;   // 障礙物 Y 座標
    int cx = obstacle_idx.second;  // 障礙物 X 座標
    
    // 計算膨脹區域的邊界 (確保不超出地圖範圍)
    uint32_t bw_min = std::max(0, cx-num_rad);
    uint32_t bw_max = std::min(max_w, cx+num_rad);
    uint32_t bh_min = std::max(0, cy-num_rad);
    uint32_t bh_max = std::min(max_h, cy+num_rad);
    
    // 遍歷膨脹區域內的所有網格
    for (uint32_t i = bh_min; i < bh_max; i++)
    {
      for (uint32_t j = bw_min; j < bw_max; j++)
      {
        // 只對自由空間進行成本計算
        if (costmap.data[i*max_w+j] <= Cost::FREE_SPACE)
        {
          // 計算到障礙物的實際距離
          double dy =  (cy-i)*map_res;
          double dx =  (cx-j)*map_res;
          double dis = std::sqrt(std::pow(dx, 2)+std::pow(dy, 2));
          
          // 如果在膨脹半徑內，設定成本值
          if (dis < inflation_radius_) 
            costmap.data[i*max_w+j] = computeCost(dis);
        }
      }
    }
  }
}

/**
 * @brief 計算成本值
 * @param dis 到障礙物的距離
 * @return 成本值
 * @description 目前簡化為固定值，註解中保留了指數衰減的計算方式
 */
double GlobalCostmap::computeCost(const double dis)
{
  // 注意：原始指數衰減公式被註解，目前使用固定值
  // 原始公式：return exp(-1.0 * cost_scaling_factor_ * (dis - inflation_radius_)) * (Cost::LETHAL - 1);
  return 254;  // 固定返回最大成本值
}

/**
 * @brief 地圖更新回調函數 (ROS2 版本)
 * @param map 接收到的靜態地圖
 * @description 當接收到新的靜態地圖時，處理並生成對應的全域成本地圖
 */
void GlobalCostmap::mapUpdateCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // 複製地圖的基本信息
  costmap_.header = map->header;
  costmap_.info = map->info;
  
  // 根據目標解析度決定是否需要降採樣
  if (costmap_resolution_ > map->info.resolution)
  {
    // 如果目標解析度較低，進行降採樣
    getDownSampledCostmap(map, costmap_, costmap_resolution_);
  }
  else
  {
    // 直接使用原始地圖數據
    costmap_.data = map->data;
  }
  
  // 提取所有障礙物位置
  std::vector<std::pair<int, int>> obstacle_idx;
  setObstacleIndexes(obstacle_idx, costmap_);
  
  // 計算膨脹後的成本地圖
  computeCostmap(costmap_, obstacle_idx, inflation_radius_, costmap_resolution_);
  
  // 發布生成的全域成本地圖
  global_costmap_pub_->publish(costmap_);
}
