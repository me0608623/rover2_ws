#ifndef BUBBLE_MPC_SOLVER_HPP_
#define BUBBLE_MPC_SOLVER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bubble_planner/msg/mpc_parameter.hpp"

// Acados includes
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"
#include "acados_solver_bubble_model.h"

class BubbleMpcSolver
{
public:
  BubbleMpcSolver();
  ~BubbleMpcSolver();

  /**
   * @brief 從 bubbles_info 中尋找最近的三個氣泡與對應的障礙物資訊
   * * @param current_state 當前機器人狀態 [x, y, theta]
   * @param bubbles_info 包含氣泡幾何與障礙物資訊的 Path msg
   * @param return_bubble 回傳扁平化的向量，包含3組 (b_x, b_y, b_r, o_x, o_y)
   */
  void GetNearestBubbleAndObs(const std::vector<double> &current_state,
                              const nav_msgs::msg::Path &bubbles_info,
                              std::vector<double> &return_bubble);

  /**
   * @brief 從 bubble_path (導航路徑) 計算參考點與 Waypoint
   */
  std::vector<double> GetRefWayPose(const std::vector<double> &current_state,
                                    const nav_msgs::msg::Path &path,
                                    const double &path_length,
                                    const double &cumulative_length,
                                    const geometry_msgs::msg::PoseStamped &global_path_end);

  void UpdateState(const bubble_planner::msg::MPCParameter::ConstSharedPtr &msg);
  
  void UpdateNoBubbleStatus(const nav_msgs::msg::Path &bubble_path,
                            const nav_msgs::msg::Path &bubbles_info);

  bool IsNoBubbleTriggered() const
  {
    return no_bubble_ > 0.0;
  }

  /**
   * @brief 更新 Solver 所需的所有外部資訊
   * * @param bubble_path 導航用的路徑點 (純路徑)
   * @param bubbles_info 氣泡資訊 (position.z=半徑) 與 障礙物資訊 (orientation.x/y=障礙物座標)
   * @param bubble_path_total_length 路徑總長
   * @param cumulative_distance 累計行駛距離
   * @param global_path_end_pose 全局終點
   * @param if_over_global_end 是否超過終點旗標
   */
  void UpdateInformation(const nav_msgs::msg::Path &bubble_path,
                         const nav_msgs::msg::Path &bubbles_info,
                         const double &bubble_path_total_length,
                         const double &cumulative_distance,
                         const geometry_msgs::msg::PoseStamped &global_path_end_pose);

  void ResetParameter();
  bool InitialParameter();
  bool MPC87InitialParameter();
  geometry_msgs::msg::Twist Compute(nav_msgs::msg::Path &output_path, geometry_msgs::msg::Point &way_point);

  void PrintComputeTime();

private:
  // acados capsule and config
  bubble_model_solver_capsule *capsule_;
  ocp_nlp_config *nlp_config_;
  ocp_nlp_dims *nlp_dims_;
  ocp_nlp_in *nlp_in_;
  ocp_nlp_out *nlp_out_;

  // dimensions and status
  int N_;
  int nx_;
  int nu_;
  int np_;
  int error_n;
  int status_;
  std::vector<double> compute_time;

  // MPC variables
  double x_current_[3];
  double horizon_state_[3];
  double u_current_[2];
  std::array<double, 23> param_{}; // 確保大小符合 gen_model.py 定義 (23個參數)

  // Data Sources
  nav_msgs::msg::Path bubble_path_; // 用於尋找路徑點 (Ref, Way)
  nav_msgs::msg::Path bubbles_info_; // 用於尋找約束條件 (Bubble, Obstacle)
  
  double bubble_path_total_length_;
  double cumulative_distance_;
  geometry_msgs::msg::PoseStamped global_path_end_pose_;

  // Random noise generators
  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> noise_pos_{0.0, 0.05};
  std::normal_distribution<double> noise_ang_{0.0, 0.02};

  static constexpr int kNoBubbleTriggerCount = 2;
  int empty_bubbles_info_count_{0};
  int empty_bubble_path_count_{0};

  double over_final_bubble_{0.0};
  double no_bubble_{0.0};
};

#endif // BUBBLE_MPC_SOLVER_HPP_
