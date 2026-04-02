#include "campusrover_costmap.h"
#include <limits> // 為了 numeric_limits

//PCL 可視化相關全域變數定義
std::vector<PointCloudPtr> g_cloud_viz_list;
bool g_pcl_viz = false;

// [Gemini 修正]：定義 Nav2 標準代價
static const int8_t LETHAL_OBSTACLE = 100;
static const int8_t INSCRIBED_INFLATED_OBSTACLE = 99;
static const int8_t NO_INFORMATION = -1;

/**
 * @brief CampusroverCostmap 建構函數 (ROS2 版本)
 * @description 初始化所有 ROS2 參數、發布者和訂閱者
 */
CampusroverCostmap::CampusroverCostmap() : Node("local_costmap_node")
{
  // 宣告 ROS2 參數
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("map_size_width", 10.0);
  this->declare_parameter("map_size_height", 10.0);
  this->declare_parameter("map_update_frequency", 5.0); // 注意：這個參數在此程式中未被使用
  this->declare_parameter("map_resolution", 0.1);
  this->declare_parameter("pcl_viz", false);
  this->declare_parameter("max_obstacle_height", 2.0);
  this->declare_parameter("min_obstacle_height", 0.0);
  this->declare_parameter("inflation_radius", 0.55);
  this->declare_parameter("cost_scaling_factor", 10.0);
  
  // 讀取參數值
  base_frame_ = this->get_parameter("base_frame").as_string();
  map_size_width_ = this->get_parameter("map_size_width").as_double();
  map_size_height_ = this->get_parameter("map_size_height").as_double();
  map_update_frequency_ = this->get_parameter("map_update_frequency").as_double();
  map_resolution_ = this->get_parameter("map_resolution").as_double();
  g_pcl_viz = this->get_parameter("pcl_viz").as_bool();
  max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();
  min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
  cost_scaling_factor_ = this->get_parameter("cost_scaling_factor").as_double();

  RCLCPP_INFO(this->get_logger(), "Local Costmap 參數加載完畢 (Inflation Radius: %.2f)", inflation_radius_);

  // 初始化 TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 初始化 ROS2 發布者和訂閱者
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("campusrover_local_costmap", 10);

  rclcpp::QoS lidar_qos(rclcpp::KeepLast(5));
  lidar_qos.reliable();   // 不可靠，丟幾個 frame 無妨
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points2", lidar_qos, 
    std::bind(&CampusroverCostmap::PointCloud2Callback, this, std::placeholders::_1));

  // 初始化成本地圖結構 (只做一次)
  costmap_.header.frame_id = base_frame_;
  costmap_.info.resolution = map_resolution_;
  costmap_.info.width = map_size_width_ / map_resolution_;    // 計算網格寬度 (cols)
  costmap_.info.height = map_size_height_ / map_resolution_;  // 計算網格高度 (rows)
  
  costmap_.info.origin.position.x = -map_size_width_ / 2.0;
  costmap_.info.origin.position.y = -map_size_height_ / 2.0;
  costmap_.info.origin.position.z = 0;
  costmap_.info.origin.orientation.w = 1;
  
  // 預先分配地圖數據大小，並初始化為 0 (Free)
  costmap_.data.resize(costmap_.info.width * costmap_.info.height, 0);
  RCLCPP_INFO(this->get_logger(), "Local Costmap 初始化完畢 (W: %d, H: %d, Total: %zu)",
    costmap_.info.width, costmap_.info.height, costmap_.data.size());
}

/**
 * @brief 點雲數據回調函數 (ROS2 版本)
 * @param msg 接收到的點雲消息
 * @description 主要處理流程：座標變換 -> 濾波 -> 降採樣 -> 生成成本地圖 -> 發布
 */
void CampusroverCostmap::PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 將 ROS2 消息轉換為 PCL 點雲
  PointCloudPtr cloud_pcl(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud_pcl);
  
  // 將點雲從感測器座標系變換到機器人基礎座標系
  PointCloudPtr cloud_out(new PointCloud);
  try
  {
    // [Gemini 修正]：使用 msg 的時間戳，並給予 0.1 秒緩衝
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform(
      base_frame_, msg->header.frame_id, 
      msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    
    // 執行點雲變換
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    Eigen::Matrix4f tf_matrix = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
    pcl::transformPointCloud(*cloud_pcl, cloud_transformed, tf_matrix);
    *cloud_out = cloud_transformed;
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_WARN(this->get_logger(), "TF2 變換失敗: %s", ex.what());
    return;
  }
  
  // 更新時間戳
  costmap_.header.stamp = this->now();
  
  // 點雲處理流水線
  cropBoxFilter(cloud_out, cloud_out);              // 1. 裁剪不需要的區域
  PointCloudPtr cloud_filtered(new PointCloud);     // 2. 創建濾波後的點雲
  voxelGridFliter(cloud_out, cloud_filtered);       // 3. 體素網格降採樣
  
  // 如果啟用可視化，將點雲加入可視化列表
  if (g_pcl_viz)
  {
    g_cloud_viz_list.push_back(cloud_filtered);
  }
  
  // 將點雲轉換為成本地圖 (直接修改 costmap_ 成員變數)
  toCostmap(cloud_filtered, costmap_);
  costmap_pub_->publish(costmap_);
}

// ... (passThroughFilter 保持不變, 已棄用) ...
void CampusroverCostmap::passThroughFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  
  // Z 軸過濾：保留指定高度範圍內的點
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_obstacle_height_, max_obstacle_height_);
  pass.filter (*cloud_out);
  
  // X 軸過濾：保留地圖寬度範圍內的點
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-map_size_width_/2.0, map_size_width_/2.0);
  pass.filter (*cloud_out);
  
  // Y 軸過濾：保留地圖高度範圍內的點
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-map_size_height_/2.0, map_size_height_/2.0);
  pass.filter (*cloud_out);
}


// ... (cropBoxFilter 保持不變) ...
void CampusroverCostmap::cropBoxFilter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out)
{
  pcl::CropBox<pcl::PointXYZ> box_filter(true);
  box_filter.setInputCloud (cloud_in);
  
  // 第一階段：保留地圖範圍內且在指定高度範圍內的點
  Eigen::Vector4f min_pt (-map_size_width_/2.0, -map_size_height_/2.0, min_obstacle_height_, 1.0f);
  Eigen::Vector4f max_pt ( map_size_width_/2.0,  map_size_height_/2.0, max_obstacle_height_, 1.0f);
  box_filter.setMin (min_pt);
  box_filter.setMax (max_pt);
  box_filter.filter (*cloud_out);
  
  // 第二階段：移除機器人本體範圍內的點 (負向過濾)
  box_filter.setInputCloud (cloud_out);
  box_filter.setNegative(true);  // 啟用負向過濾 (移除範圍內的點)
  
  // Campus Rover 機器人本體尺寸範圍 (長 0.65m, 寬 1.0m, 高 1.9m)
  Eigen::Vector4f campusrover_min_pt (-0.35, -0.5, 0, 1);
  Eigen::Vector4f campusrover_max_pt ( 0.3,   0.5, 1.9, 1);
  box_filter.setMin (campusrover_min_pt);
  box_filter.setMax (campusrover_max_pt);
  box_filter.filter (*cloud_out);
}

// ... (voxelGridFliter 保持不變) ...
void CampusroverCostmap::voxelGridFliter(PointCloudPtr &cloud_in, PointCloudPtr &cloud_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> v_sor;
  v_sor.setInputCloud (cloud_in);
  // 設定體素大小為地圖解析度的 1/5，確保足夠的細節保留
  // v_sor.setLeafSize (map_resolution_/5.0, map_resolution_/5.0, map_resolution_/5.0);
  v_sor.setLeafSize (map_resolution_, map_resolution_, map_resolution_);
  v_sor.filter (*cloud_out);
}


/**
 * @brief 將點雲轉換為成本地圖 (ROS2 版本)
 * @param cloud_in 輸入點雲
 * @param occupancy_grid [in/out] 輸出的占用網格地圖 (將被就地修改)
 * @description 將 3D 點雲投影到 2D 網格，並計算膨脹後的成本值
 */
void CampusroverCostmap::toCostmap(PointCloudPtr &cloud_in, nav_msgs::msg::OccupancyGrid &occupancy_grid)
{
  // [Gemini 修正]：在開始計算前，重置 (Reset) 地圖為 0 (Free)
  // 我們不再使用 clear() 和 push_back()
  std::fill(occupancy_grid.data.begin(), occupancy_grid.data.end(), 0);

  // 快取地圖參數
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  double origin_x = occupancy_grid.info.origin.position.x;
  double origin_y = occupancy_grid.info.origin.position.y;
  
  // 將點雲座標轉換為網格索引
  std::vector<Eigen::Vector2i> obstacle_index;
  for (const auto& pt : cloud_in->points)
  {
    // 將世界座標轉換為網格索引 (col, row)
    int x = (pt.x - origin_x)/resolution;
    int y = (pt.y - origin_y)/resolution;
    
    // [Gemini 修正]：增加邊界檢查
    if (x >= 0 && x < width && y >= 0 && y < height)
    {
        obstacle_index.push_back(Eigen::Vector2i(x, y));
        // 直接在地圖上標記「致命」障礙物
        occupancy_grid.data[y * width + x] = LETHAL_OBSTACLE;
    }
  }
  
  // --- 膨脹 (Inflation) 計算 ---
  
  // 1. 將膨脹半徑 (公尺) 轉換為網格單位
  int inflation_radius_cells = static_cast<int>(inflation_radius_ / resolution);
  
  // 2. 遍歷所有障礙物
  for (const auto& obs_cell : obstacle_index)
  {
      int obs_x = obs_cell(0);
      int obs_y = obs_cell(1);
      
      // 3. 遍歷該障礙物周圍的「膨脹正方形」區域
      for (int y = obs_y - inflation_radius_cells; y <= obs_y + inflation_radius_cells; y++)
      {
          for (int x = obs_x - inflation_radius_cells; x <= obs_x + inflation_radius_cells; x++)
          {
              // 4. 檢查是否在地圖範圍內
              if (x < 0 || x >= width || y < 0 || y >= height)
              {
                  continue; // 跳過地圖外的點
              }
              
              // 5. 計算到障礙物中心的距離 (公尺)
              double dist_x_meters = (x - obs_x) * resolution;
              double dist_y_meters = (y - obs_y) * resolution;
              double distance_in_meters = std::hypot(dist_x_meters, dist_y_meters);
              
              // 6. 如果在膨脹半徑內
              if (distance_in_meters <= inflation_radius_)
              {
                  // 7. 計算成本
                  int8_t cost = computeCost(distance_in_meters);
                  
                  // 8. 更新地圖 (只在「新成本」大於「舊成本」時才寫入)
                  int index = y * width + x;
                  if (occupancy_grid.data[index] < cost)
                  {
                      occupancy_grid.data[index] = cost;
                  }
              }
          }
      }
  }
}


/**
 * @brief 計算成本值 (ROS2 版本)
 * @param dis 到最近障礙物的距離 (公尺)
 * @return 成本值 (0-99)
 * @description [Gemini 修正]：使用標準 Nav2 膨脹公式
 */
int8_t CampusroverCostmap::computeCost(const double dis_meters)
{
    if (dis_meters < 0.0) // 理論上不該發生
    {
        return 0;
    }
    
    // Nav2 標準膨脹公式：
    // cost = (MAX_COST - 1) * exp(-cost_scaling_factor * (distance_from_obstacle - inscribed_radius))
    // 你的實作中 "inscribed_radius" (內切半徑) 為 0.0，因為你是從點開始膨脹
    
    double cost = (INSCRIBED_INFLATED_OBSTACLE - 1) * exp(-1.0 * cost_scaling_factor_ * dis_meters);
    
    // 確保成本值在 0-99 之間
    return static_cast<int8_t>(std::max(0.0, std::min(cost, (double)INSCRIBED_INFLATED_OBSTACLE)));
}
