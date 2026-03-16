#include "bubble_mpc_solver.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

using namespace std;

namespace
{
  auto get_logger()
  {
    return rclcpp::get_logger("BubbleMpcSolver");
  }
} // namespace

bubble_planner::msg::MPCParameter mpc_para;

// Status codes:
// 0 - Success (ACADOS_SUCCESS)
// 1 - NaN detected (ACADOS_NAN_DETECTED)
// 2 - Maximum number of iterations reached (ACADOS_MAXITER)
// 3 - Minimum step size reached (ACADOS_MINSTEP)
// 4 - QP solver failed (ACADOS_QP_FAILURE)
// 5 - Solver created (ACADOS_READY)
// 6 - Problem unbounded (ACADOS_UNBOUNDED)
// 7 - Time Out (ACADOS_TIMEOUT)

BubbleMpcSolver::BubbleMpcSolver()
{
  capsule_ = bubble_model_acados_create_capsule();
  status_ = bubble_model_acados_create(capsule_);

  if (status_)
  {
    printf("[BUBBLE_MPC_SOLVER] bubble_model_acados_create() returned status %d. Exiting.\n", status_);
    exit(EXIT_FAILURE);
  }

  nlp_config_ = bubble_model_acados_get_nlp_config(capsule_);
  nlp_dims_ = bubble_model_acados_get_nlp_dims(capsule_);
  nlp_in_ = bubble_model_acados_get_nlp_in(capsule_);
  nlp_out_ = bubble_model_acados_get_nlp_out(capsule_);

  N_ = nlp_dims_->N;
  nx_ = *nlp_dims_->nx;
  nu_ = *nlp_dims_->nu;
  np_ = capsule_->nlp_np;
  printf("[BUBBLE_MPC_SOLVER] Finish MPC initialize. Time horizion id %d, with %d state, %d parameter and %d input.\n", N_, nx_, np_, nu_);
}

BubbleMpcSolver::~BubbleMpcSolver() {}

// 解析 Bubbles Info 訊息格式：
// Bubble Center: pose.position.x, pose.position.y
// Bubble Radius: pose.position.z
// Obstacle Pos : pose.orientation.x, pose.orientation.y
//
// return_bubble 格式 (size 15):
// [0-2]: b0(x,y,r), [3-4]: o0(x,y)
// [5-7]: b1(x,y,r), [8-9]: o1(x,y)
// [10-12]: b2(x,y,r), [13-14]: o2(x,y)
void BubbleMpcSolver::GetNearestBubbleAndObs(const vector<double> &current_state,
                                             const nav_msgs::msg::Path &bubbles_info,
                                             vector<double> &return_bubble)
{
  // 初始化 (雖然下面會覆蓋，但保留作為安全預設值)
  return_bubble.assign(15, -878.0);
  return_bubble[2] = 0.0;
  return_bubble[7] = 0.0;
  return_bubble[12] = 0.0;
  over_final_bubble_ = 0.0;

  if (bubbles_info.poses.empty())
    return;

  double current_x = current_state[0];
  double current_y = current_state[1];

  double min_distance = std::numeric_limits<double>::max();
  int nearest_index = -1;

  // 1. 找到離當前狀態最近的 Bubble Index
  for (size_t i = 0; i < bubbles_info.poses.size(); ++i)
  {
    const auto &b_pos = bubbles_info.poses[i].pose.position;
    double dist = std::hypot(current_x - b_pos.x, current_y - b_pos.y);

    if (dist < min_distance)
    {
      min_distance = dist;
      nearest_index = static_cast<int>(i);
    }
  }

  if (nearest_index == -1)
    return;

  // 2. 取出最近的 3 個 Bubbles
  // 計算實際上有幾顆氣泡可用 (最多算 3 顆)
  const int total_bubbles = static_cast<int>(bubbles_info.poses.size());
  const int real_available_count = std::min(3, total_bubbles - nearest_index);

  if (real_available_count < 3)
  {
    const int last_index = total_bubbles - 1;
    const auto &last_pose = bubbles_info.poses[last_index].pose;
    double last_radius = last_pose.position.z;
    if (last_radius <= 0.0)
      last_radius = 0.05;

    double dir_x = 0.0;
    double dir_y = 0.0;
    if (total_bubbles >= 2)
    {
      const auto &prev_pose = bubbles_info.poses[last_index - 1].pose;
      dir_x = last_pose.position.x - prev_pose.position.x;
      dir_y = last_pose.position.y - prev_pose.position.y;
    }
    else
    {
      dir_x = std::cos(current_state[2]);
      dir_y = std::sin(current_state[2]);
    }

    const double dir_norm = std::hypot(dir_x, dir_y);
    if (dir_norm > 1e-6)
    {
      dir_x /= dir_norm;
      dir_y /= dir_norm;
    }
    else
    {
      dir_x = std::cos(current_state[2]);
      dir_y = std::sin(current_state[2]);
    }

    const double to_current_x = current_x - last_pose.position.x;
    const double to_current_y = current_y - last_pose.position.y;
    const double dist_to_last = std::hypot(to_current_x, to_current_y);
    const double dot = to_current_x * dir_x + to_current_y * dir_y;

    if (dot > 0.0 && dist_to_last > last_radius)
    {
      over_final_bubble_ = 0.02;
    }
  }

  // 強制填滿 3 個 slot (i = 0, 1, 2)
  for (int i = 0; i < 3; ++i)
  {
    // 關鍵修改：計算要讀取的 offset
    // 如果 i 超出了實際可用的氣泡數量，就使用最後一個可用的 offset
    // 例如：只有 1 顆 (real_available_count=1):
    // i=0 -> offset=0
    // i=1 -> offset=0 (複製第0顆)
    // i=2 -> offset=0 (複製第0顆)
    int offset = std::min(i, real_available_count - 1);

    const auto &pose = bubbles_info.poses[nearest_index + offset].pose;

    // 根據定義解析資料
    double b_x = pose.position.x;
    double b_y = pose.position.y;
    double b_r = pose.position.z;

    double o_x = pose.orientation.x;
    double o_y = pose.orientation.y;

    // 填入 return_bubble (5個一組)
    int base = i * 5;
    return_bubble[base] = b_x;
    return_bubble[base + 1] = b_y;
    return_bubble[base + 2] = b_r;
    return_bubble[base + 3] = o_x;
    return_bubble[base + 4] = o_y;
  }
}

int getMode(double val)
{
  // 用來取得 diff 的 mode： >0 回傳 1, <0 回傳 -1, ==0 回傳 0
  if (val > 0.0)
    return 1;
  else if (val < 0.0)
    return -1;
  else
    return 0;
}

int findwaylen(const vector<double> &get_current_state, const nav_msgs::msg::Path &path, int nearest_index)
{
  // 若 nearest_index+1 超出範圍，則回傳預設 offset
  if (nearest_index + 1 >= static_cast<int>(path.poses.size()))
    return 3;

  // 取得 ref 點 (nearest_index+1) 的位置
  double ref_x = path.poses[nearest_index + 1].pose.position.x;
  double ref_y = path.poses[nearest_index + 1].pose.position.y;

  // 當前機器人位置
  double current_x = get_current_state[0];
  double current_y = get_current_state[1];

  int best_offset = 3;        // 預設從 +3 開始
  double best_diff = 0.0;     // 記錄目前最佳 diff 值
  double previous_diff = 0.0; // 記錄上一輪 diff
  int previous_mode = 0;      // 記錄上一輪 diff 的 mode (1:正, -1:負, 0:零)

  // 可調參數
  const double tolerance = -0.01; // diff與previous 相減的容忍值 要 diff - prevoius 要 >= -0.01
  const double max_diff = 0.15;   // diff 的上限

  // 從 nearest_index + 3 到 nearest_index + 11 逐一檢查候選點
  for (int offset = 3; offset <= 11; ++offset)
  {
    int idx = nearest_index + offset;
    if (idx >= static_cast<int>(path.poses.size()))
    {
      break; // 超出範圍則停止
    }

    // 取得候選點位置 (way)
    double way_x = path.poses[idx].pose.position.x;
    double way_y = path.poses[idx].pose.position.y;

    // 計算 ref 到 candidate 的向量與單位法向量
    double v_ref_way_x = way_x - ref_x;
    double v_ref_way_y = way_y - ref_y;
    double l_ref_way = std::sqrt(v_ref_way_x * v_ref_way_x + v_ref_way_y * v_ref_way_y);
    if (l_ref_way < 1e-6) // 避免除以 0
      continue;

    double unit_v_ref_way_x = v_ref_way_x / l_ref_way;
    double unit_v_ref_way_y = v_ref_way_y / l_ref_way;
    double unit_n_ref_way_x = -unit_v_ref_way_y;
    double unit_n_ref_way_y = unit_v_ref_way_x;

    // 計算從機器人至 ref 的向量
    double v_ref_robot_x = current_x - ref_x;
    double v_ref_robot_y = current_y - ref_y;

    // e_contour 為投影至法向量的分量
    double diff = unit_n_ref_way_x * v_ref_robot_x + unit_n_ref_way_y * v_ref_robot_y;

    // 簡單定義 mode：diff > 0 為正，diff < 0 為負，否則為 0
    int current_mode = getMode(diff);

    RCLCPP_INFO(get_logger(), "Offset: %d, Index: %d, Contour diff: %f, mode: %d", offset, idx, diff, current_mode);

    if (offset == 3)
    {
      // 第一筆初始化
      best_offset = offset;
      best_diff = diff;
      previous_diff = diff;
      previous_mode = current_mode;
    }
    else
    {
      // 若 mode 發生改變（包含 diff==0 的情況），依然以絕對值比較
      if (current_mode != previous_mode)
      {
        RCLCPP_INFO(get_logger(), "Mode changed at offset %d (prev diff: %f, mode: %d, current diff: %f, mode: %d).",
                    offset, previous_diff, previous_mode, diff, current_mode);
        double absPrev = std::fabs(previous_diff);
        double absCurr = std::fabs(diff);
        if (absCurr < absPrev)
        {
          best_offset = offset;
          best_diff = diff;
        }
        else
        {
          best_offset = offset - 1;
          best_diff = previous_diff;
        }
        break; // mode 改變後即選定最佳 offset，退出搜尋
      }

      // 若 mode 未變，分正向與負向分支比較
      if (current_mode >= 0) // 正向或0的情況
      {
        if (diff - previous_diff >= tolerance) // tolerance = -0.01
        {
          if (diff <= max_diff)
          {
            best_offset = offset;
            best_diff = diff;
          }
          else
          {
            RCLCPP_INFO(get_logger(), "Offset %d: diff %f exceeds max allowed %f. Stopping search.", offset, diff, max_diff);
            break;
          }
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Contour diff decreases at offset %d (prev: %f, current: %f), stopping search.", offset, previous_diff, diff);
          break;
        }
      }
      else // 負向的情況，依照絕對值比較
      {
        if (std::fabs(diff) - std::fabs(previous_diff) >= tolerance)
        {
          if (std::fabs(diff) <= max_diff)
          {
            best_offset = offset;
            best_diff = diff;
          }
          else
          {
            RCLCPP_INFO(get_logger(), "Offset %d: diff %f exceeds max allowed %f. Stopping search.", offset, diff, max_diff);
            break;
          }
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Contour diff decreases at offset %d (prev: %f, current: %f), stopping search.", offset, previous_diff, diff);
          break;
        }
      }
      // 更新上一輪的 diff 與 mode
      previous_diff = diff;
      previous_mode = current_mode;
    }
  }

  return best_offset;
}

// 1. GetRefWayPose() 函式：整合動態調整及 findwaylen() 的結果
vector<double> BubbleMpcSolver::GetRefWayPose(const vector<double> &get_current_state, const nav_msgs::msg::Path &path, const double &path_length, const double &cumulative_length, const geometry_msgs::msg::PoseStamped &global_path_end)
{
  bool leave_start_flag = false;

  double current_x = get_current_state[0];
  double current_y = get_current_state[1];

  // 取得 path 最後一個索引
  int final_inx = path.poses.size() - 1;

  const double end_dx = global_path_end.pose.position.x - path.poses[final_inx].pose.position.x;
  const double end_dy = global_path_end.pose.position.y - path.poses[final_inx].pose.position.y;
  const double end_match_dist = 0.01;
  const bool end_matches_global = std::hypot(end_dx, end_dy) < end_match_dist;

  // 找出 path 中離目前位置最近的點的索引
  double min_distance = std::numeric_limits<double>::max();
  int nearest_index = -1;
  for (size_t i = 0; i < path.poses.size(); ++i)
  {
    double dx = path.poses[i].pose.position.x - current_x;
    double dy = path.poses[i].pose.position.y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_index = i;
    }
  }

  // 動態調整 way_len
  int way_len = 0;
  if (path_length > 1.0)
  {
    if (cumulative_length <= 0.5)
    {
      way_len = 8;
    }
    else
    {
      // 呼叫 findwaylen() 使用 ref 與候選點計算最佳 offset
      way_len = findwaylen(get_current_state, path, nearest_index);
      leave_start_flag = true;
    }
  }
  else // 當 path_length < 1.0
  {
    if (cumulative_length <= 0.25)
    {
      way_len = 5;
    }
    else
    {
      way_len = 3;
      leave_start_flag = true;
    }
  }

  if (leave_start_flag)
  {
    // 計算 total_length：從目前位置到最近點，再加上從最近點沿 path 依序累加至終點的距離
    double path_distance = 0.0;
    for (int i = nearest_index; i < final_inx; ++i)
    {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      path_distance += std::sqrt(dx * dx + dy * dy);
    }
    double total_length = min_distance + path_distance; // 剩餘當前到path最近點+最近點到終點的距離

    // 當快到 global_path_end (或 path 終點) 時，適度減少 way_len
    if (end_matches_global)
    {
      if (total_length < 2.0)
      {
        way_len = std::min(way_len, 3);
      }
      else if (total_length < 3.0)
      {
        way_len = std::min(way_len, 4);
      }
    }
  }

  // 使用原本條件計算 ref_id 與 way_id
  int ref_pt_offset = 1; // 假設 offset 為 1 (可根據需求調整)
  int ref_id;
  if (nearest_index + ref_pt_offset > final_inx)
  {
    ref_id = final_inx;
  }
  else
  {
    ref_id = nearest_index + ref_pt_offset;
  }

  int way_id;
  if (nearest_index + way_len > final_inx)
  {
    way_id = final_inx;
  }
  else
  {
    way_id = nearest_index + way_len;
  }

  // 準備回傳結果：前 3 個元素為 ref_pose (x, y, theta)，後 3 個為 way_pose (x, y, theta)
  vector<double> result(6, 0.0);

  // ref_pose
  double ref_pose_x = path.poses[ref_id].pose.position.x;
  double ref_pose_y = path.poses[ref_id].pose.position.y;
  double ref_pose_yaw = tf2::getYaw(path.poses[ref_id].pose.orientation);
  if (end_matches_global && ref_id == final_inx)
  {
    ref_pose_x = global_path_end.pose.position.x;
    ref_pose_y = global_path_end.pose.position.y;
    ref_pose_yaw = tf2::getYaw(global_path_end.pose.orientation);
  }
  result[0] = ref_pose_x;
  result[1] = ref_pose_y;
  result[2] = ref_pose_yaw;

  // way_pose
  double way_pose_x = path.poses[way_id].pose.position.x;
  double way_pose_y = path.poses[way_id].pose.position.y;
  double way_pose_yaw = tf2::getYaw(path.poses[ref_id].pose.orientation); // 故意的 這樣比較貼
  if (end_matches_global && way_id == final_inx)
  {
    way_pose_x = global_path_end.pose.position.x;
    way_pose_y = global_path_end.pose.position.y;
  }

  double dist_to_last_pose = std::hypot(way_pose_x - current_x, way_pose_y - current_y);

  if (way_id == final_inx)
  {
    if (end_matches_global)
    {
      way_pose_yaw = tf2::getYaw(global_path_end.pose.orientation);
    }
    else if (dist_to_last_pose < 0.2)
    {
      way_pose_yaw = tf2::getYaw(path.poses[way_id].pose.orientation);
    }
  }

  result[3] = way_pose_x;
  result[4] = way_pose_y;
  result[5] = way_pose_yaw;

  return result;
}

void BubbleMpcSolver::UpdateInformation(const nav_msgs::msg::Path &bubble_path,
                                        const nav_msgs::msg::Path &bubbles_info,
                                        const double &bubble_path_total_length,
                                        const double &cumulative_distance,
                                        const geometry_msgs::msg::PoseStamped &global_path_end_pose)
{
  bubble_path_ = bubble_path;
  bubbles_info_ = bubbles_info;
  bubble_path_total_length_ = bubble_path_total_length;
  cumulative_distance_ = cumulative_distance;
  global_path_end_pose_ = global_path_end_pose;
  UpdateNoBubbleStatus(bubble_path, bubbles_info);
}

void BubbleMpcSolver::UpdateNoBubbleStatus(const nav_msgs::msg::Path &bubble_path,
                                           const nav_msgs::msg::Path &bubbles_info)
{
  if (bubbles_info.poses.empty())
    {
      ++empty_bubbles_info_count_;
      RCLCPP_WARN_STREAM(get_logger(), "No bubble once time");
    }
  else
    empty_bubbles_info_count_ = 0;

  if (bubble_path.poses.size() < 2)
    ++empty_bubble_path_count_;
  else
    empty_bubble_path_count_ = 0;

  if (empty_bubbles_info_count_ >= kNoBubbleTriggerCount || empty_bubble_path_count_ >= kNoBubbleTriggerCount)
    no_bubble_ = 0.02;
  else
    no_bubble_ = 0.0;
}

void BubbleMpcSolver::UpdateState(const bubble_planner::msg::MPCParameter::ConstSharedPtr &msg)
{
  x_current_[0] = msg->position_x;
  x_current_[1] = msg->position_y;
  x_current_[2] = msg->position_theta;

  mpc_para.header = msg->header;
  mpc_para.child_frame_id = msg->child_frame_id;
  mpc_para.position_x = msg->position_x;
  mpc_para.position_y = msg->position_y;
  mpc_para.position_theta = msg->position_theta;
  mpc_para.linear_x = msg->linear_x;
  mpc_para.angular_z = msg->angular_z;
}

void BubbleMpcSolver::ResetParameter()
{
  // reference :  c_generated_code -> acados_solver_bubble_model.c -> bubble_model_acados_create_7_set_nlp_out
  double reset_x_current_[nx_] = {0, 0, 0};
  double reset_u_current_[nu_] = {0, 0};
  std::fill(param_.begin(), param_.end(), 0.0);

  for (int i = 0; i <= N_; i++)
  {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", reset_x_current_);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", reset_u_current_);
    bubble_model_acados_update_params(capsule_, i, param_.data(), np_);
  }

  for (int i = 0; i < nx_; i++)
    x_current_[i] = reset_x_current_[i];
  for (int i = 0; i < nu_; i++)
    u_current_[i] = reset_u_current_[i];

  compute_time.clear();
  error_n = 0;
  cout << "[BUBBLE_MPC_SOLVER] Reset parameter" << endl;
}

bool BubbleMpcSolver::InitialParameter()
{
  x_current_[0] = mpc_para.position_x;
  x_current_[1] = mpc_para.position_y;
  x_current_[2] = mpc_para.position_theta;

  vector<double> x_current_state({x_current_[0], x_current_[1], x_current_[2]});
  vector<double> bubble_obs(15);
  vector<double> ref_way_param(6);

  GetNearestBubbleAndObs(x_current_state, bubbles_info_, bubble_obs);
  ref_way_param = GetRefWayPose(x_current_state, bubble_path_, bubble_path_total_length_, cumulative_distance_, global_path_end_pose_);
  double dist_to_end = bubble_path_total_length_ - cumulative_distance_;
  if (dist_to_end < 0.0)
    dist_to_end = 0.0;

  param_[0] = ref_way_param[0]; // ref x
  param_[1] = ref_way_param[1]; // ref y
  param_[2] = ref_way_param[2]; // ref th
  param_[3] = ref_way_param[3]; // way x
  param_[4] = ref_way_param[4]; // way y
  param_[5] = ref_way_param[5]; // way th

  param_[6] = bubble_obs[3];   // o_x0
  param_[7] = bubble_obs[4];   // o_y0
  param_[8] = bubble_obs[8];   // o_x1
  param_[9] = bubble_obs[9];   // o_y1
  param_[10] = bubble_obs[13]; // o_x2
  param_[11] = bubble_obs[14]; // o_y2

  param_[12] = bubble_obs[0];  // b_x
  param_[13] = bubble_obs[1];  // b_y
  param_[14] = bubble_obs[2];  // b_radius
  param_[15] = bubble_obs[5];  // b_x
  param_[16] = bubble_obs[6];  // b_y
  param_[17] = bubble_obs[7];  // b_radius
  param_[18] = bubble_obs[10]; // b_x
  param_[19] = bubble_obs[11]; // b_y
  param_[20] = bubble_obs[12]; // b_radius

  param_[21] = dist_to_end;

  param_[22] = over_final_bubble_;

  cout << "[BUBBLE_MPC_SOLVER] InitialParameter, use initial guess to update parameter " << endl;

  for (int i = 0; i <= N_; i++)
  {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_current_);
    bubble_model_acados_update_params(capsule_, i, param_.data(), np_); // 初始猜測的
  }

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x_current_);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x_current_);

  // 根據初始猜測求解ocp的最優解, x會根據model, costfuntion, constraint自動求得
  status_ = bubble_model_acados_solve(capsule_);

  if (status_ > 2)
  {
    cout << "[BUBBLE_MPC_SOLVER] InitialParameter current status failed with id :" << status_ << endl;
    return false;
  }

  cout << "[BUBBLE_MPC_SOLVER] InitialParameter, use initial guess's result to update parameter" << endl;
  clock_t time_start = clock();

  for (int i = 0; i <= N_; i++)
  {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", &horizon_state_);

    vector<double> h_state({horizon_state_[0], horizon_state_[1], horizon_state_[2]});
    GetNearestBubbleAndObs(h_state, bubbles_info_, bubble_obs);

    // ref_way_param = GetRefWayPose(horizon_state, bubble_path_);
    ref_way_param = GetRefWayPose(h_state, bubble_path_, bubble_path_total_length_, cumulative_distance_, global_path_end_pose_);

    param_[0] = ref_way_param[0]; // ref x
    param_[1] = ref_way_param[1]; // ref y
    param_[2] = ref_way_param[2]; // ref th
    param_[3] = ref_way_param[3]; // way x
    param_[4] = ref_way_param[4]; // way y
    param_[5] = ref_way_param[5]; // way th

    param_[6] = bubble_obs[3];   // o_x0
    param_[7] = bubble_obs[4];   // o_y0
    param_[8] = bubble_obs[8];   // o_x1
    param_[9] = bubble_obs[9];   // o_y1
    param_[10] = bubble_obs[13]; // o_x2
    param_[11] = bubble_obs[14]; // o_y2

    param_[12] = bubble_obs[0];  // b_x
    param_[13] = bubble_obs[1];  // b_y
    param_[14] = bubble_obs[2];  // b_radius
    param_[15] = bubble_obs[5];  // b_x
    param_[16] = bubble_obs[6];  // b_y
    param_[17] = bubble_obs[7];  // b_radius
    param_[18] = bubble_obs[10]; // b_x
    param_[19] = bubble_obs[11]; // b_y
    param_[20] = bubble_obs[12]; // b_radius

    param_[21] = dist_to_end;

    param_[22] = over_final_bubble_;
    // horizon的
    bubble_model_acados_update_params(capsule_, i, param_.data(), np_);

    cout << "[BUBBLE_MPC_SOLVER] initial parameter horzion, i = " << i << endl;
    cout << "horizon_state(x,y):(" << horizon_state_[0] << "," << horizon_state_[1] << ") " << endl;
    cout << "ref(x,y):(" << ref_way_param[0] << "," << ref_way_param[1] << ") " << endl;
    cout << "way(x,y):(" << ref_way_param[3] << "," << ref_way_param[4] << ") " << endl;
    cout << "dis to way: " << (pow(horizon_state_[0] - ref_way_param[3], 2) + pow(horizon_state_[1] - ref_way_param[4], 2)) << endl;
    cout << "nearest obstacle point(x,y) : (" << bubble_obs[3] << "," << bubble_obs[4] << ")" << endl;
    cout << "second obstacle point(x,y) : (" << bubble_obs[8] << "," << bubble_obs[9] << ")" << endl;
    cout << "third obstacle point(x,y) : (" << bubble_obs[13] << "," << bubble_obs[14] << ")" << endl;
    cout << "Obstacle Current constraint : first = " << (pow(horizon_state_[0] - bubble_obs[3], 2) + pow(horizon_state_[1] - bubble_obs[4], 2)) - pow(0.35, 2) << "," << "second = " << (pow(horizon_state_[0] - bubble_obs[8], 2) + pow(horizon_state_[1] - bubble_obs[9], 2)) - pow(0.35, 2) << "," << "third = " << (pow(horizon_state_[0] - bubble_obs[13], 2) + pow(horizon_state_[1] - bubble_obs[14], 2)) - pow(0.35, 2) << "," << endl;
  }

  clock_t time_end = clock();
  cout << "compute Excuting time:" << double(time_end - time_end) / CLOCKS_PER_SEC << endl;

  return true;
}

bool BubbleMpcSolver::MPC87InitialParameter()
{
  x_current_[0] = mpc_para.position_x;
  x_current_[1] = mpc_para.position_y;
  x_current_[2] = mpc_para.position_theta;

  x_current_[0] += noise_pos_(rng_);
  x_current_[1] += noise_pos_(rng_);
  x_current_[2] += noise_ang_(rng_);

  vector<double> x_current_state({x_current_[0], x_current_[1], x_current_[2]});
  vector<double> bubble_obs(15);
  vector<double> ref_way_param(6);
  GetNearestBubbleAndObs(x_current_state, bubbles_info_, bubble_obs);
  ref_way_param = GetRefWayPose(x_current_state, bubble_path_, bubble_path_total_length_, cumulative_distance_, global_path_end_pose_);
  double dist_to_end = std::max(0.0, bubble_path_total_length_ - cumulative_distance_);

  param_[0] = ref_way_param[0]; // ref x
  param_[1] = ref_way_param[1]; // ref y
  param_[2] = ref_way_param[2]; // ref th
  param_[3] = ref_way_param[3]; // way x
  param_[4] = ref_way_param[4]; // way y
  param_[5] = ref_way_param[5]; // way th

  param_[6] = bubble_obs[3];   // o_x0
  param_[7] = bubble_obs[4];   // o_y0
  param_[8] = bubble_obs[8];   // o_x1
  param_[9] = bubble_obs[9];   // o_y1
  param_[10] = bubble_obs[13]; // o_x2
  param_[11] = bubble_obs[14]; // o_y2

  param_[12] = bubble_obs[0];  // b_x
  param_[13] = bubble_obs[1];  // b_y
  param_[14] = bubble_obs[2];  // b_radius
  param_[15] = bubble_obs[5];  // b_x
  param_[16] = bubble_obs[6];  // b_y
  param_[17] = bubble_obs[7];  // b_radius
  param_[18] = bubble_obs[10]; // b_x
  param_[19] = bubble_obs[11]; // b_y
  param_[20] = bubble_obs[12]; // b_radius

  param_[21] = dist_to_end;

  param_[22] = over_final_bubble_;

  cout << "[BUBBLE_MPC_SOLVER] MPC87InitialParameter, use initial guess to update parameter " << endl;

  for (int i = 0; i <= N_; i++)
  {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_current_);
    bubble_model_acados_update_params(capsule_, i, param_.data(), np_); // 初始猜測的
  }

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x_current_);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x_current_);

  // 根據初始猜測求解ocp的最優解, x會根據model, costfuntion, constraint自動求得
  status_ = bubble_model_acados_solve(capsule_);

  if (status_ > 2)
  {
    cout << "[BUBBLE_MPC_SOLVER] MPC87InitialParameter current status failed with id :" << status_ << endl;
    return false;
  }

  cout << "[BUBBLE_MPC_SOLVER] MPC87InitialParameter, use initial guess's result to update parameter" << endl;
  clock_t time_start = clock();

  for (int i = 0; i <= N_; i++)
  {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", &horizon_state_);

    vector<double> h_state({horizon_state_[0], horizon_state_[1], horizon_state_[2]});
    GetNearestBubbleAndObs(h_state, bubbles_info_, bubble_obs);
    ref_way_param = GetRefWayPose(h_state, bubble_path_, bubble_path_total_length_, cumulative_distance_, global_path_end_pose_);

    param_[0] = ref_way_param[0]; // ref x
    param_[1] = ref_way_param[1]; // ref y
    param_[2] = ref_way_param[2]; // ref th
    param_[3] = ref_way_param[3]; // way x
    param_[4] = ref_way_param[4]; // way y
    param_[5] = ref_way_param[5]; // way th

    param_[6] = bubble_obs[3];   // o_x0
    param_[7] = bubble_obs[4];   // o_y0
    param_[8] = bubble_obs[8];   // o_x1
    param_[9] = bubble_obs[9];   // o_y1
    param_[10] = bubble_obs[13]; // o_x2
    param_[11] = bubble_obs[14]; // o_y2

    param_[12] = bubble_obs[0];  // b_x
    param_[13] = bubble_obs[1];  // b_y
    param_[14] = bubble_obs[2];  // b_radius
    param_[15] = bubble_obs[5];  // b_x
    param_[16] = bubble_obs[6];  // b_y
    param_[17] = bubble_obs[7];  // b_radius
    param_[18] = bubble_obs[10]; // b_x
    param_[19] = bubble_obs[11]; // b_y
    param_[20] = bubble_obs[12]; // b_radius

    param_[21] = dist_to_end;

    param_[22] = over_final_bubble_;
    // horizon的
    bubble_model_acados_update_params(capsule_, i, param_.data(), np_);

    cout << "[BUBBLE_MPC_SOLVER] MPC87InitialParameter parameter horzion, i = " << i << endl;
    cout << "horizon_state(x,y):(" << horizon_state_[0] << "," << horizon_state_[1] << ") " << endl;
    cout << "ref(x,y):(" << ref_way_param[0] << "," << ref_way_param[1] << ") " << endl;
    cout << "way(x,y):(" << ref_way_param[3] << "," << ref_way_param[4] << ") " << endl;
    cout << "dis to way: " << (pow(horizon_state_[0] - ref_way_param[3], 2) + pow(horizon_state_[1] - ref_way_param[4], 2)) << endl;
    cout << "nearest obstacle point(x,y) : (" << bubble_obs[3] << "," << bubble_obs[4] << ")" << endl;
    cout << "second obstacle point(x,y) : (" << bubble_obs[8] << "," << bubble_obs[9] << ")" << endl;
    cout << "third obstacle point(x,y) : (" << bubble_obs[13] << "," << bubble_obs[14] << ")" << endl;
    cout << "Obstacle Current constraint : first = " << (pow(horizon_state_[0] - bubble_obs[3], 2) + pow(horizon_state_[1] - bubble_obs[4], 2)) - pow(0.35, 2) << "," << "second = " << (pow(horizon_state_[0] - bubble_obs[8], 2) + pow(horizon_state_[1] - bubble_obs[9], 2)) - pow(0.35, 2) << "," << "third = " << (pow(horizon_state_[0] - bubble_obs[13], 2) + pow(horizon_state_[1] - bubble_obs[14], 2)) - pow(0.35, 2) << "," << endl;
  }

  clock_t time_end = clock();
  cout << "compute Excuting time:" << double(time_end - time_end) / CLOCKS_PER_SEC << endl;

  return true;
}

geometry_msgs::msg::Twist BubbleMpcSolver::Compute(nav_msgs::msg::Path &output_path, geometry_msgs::msg::Point &way_point)
{
  x_current_[0] = mpc_para.position_x;
  x_current_[1] = mpc_para.position_y;
  x_current_[2] = mpc_para.position_theta;

  clock_t time_p = clock();

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x_current_);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x_current_);

  // Status codes:
  // 0 - Success (ACADOS_SUCCESS)
  // 1 - NaN detected (ACADOS_NAN_DETECTED)
  // 2 - Maximum number of iterations reached (ACADOS_MAXITER)
  // 3 - Minimum step size reached (ACADOS_MINSTEP)
  // 4 - QP solver failed (ACADOS_QP_FAILURE)
  // 5 - Solver created (ACADOS_READY)
  // 6 - Problem unbounded (ACADOS_UNBOUNDED)
  // 7 - Time Out (ACADOS_TIMEOUT)

  // 根據horizon的求解ocp的最優解, x會根據model, costfuntion, constraint自動求得
  status_ = bubble_model_acados_solve(capsule_);
  if (status_ > 2)
  {
    cout << "[BUBBLE_MPC_SOLVER] Compute error status : " << status_ << endl;
    geometry_msgs::msg::Twist cmd_vel_stop;
    cmd_vel_stop.linear.x = 0;
    cmd_vel_stop.angular.z = 0;
    cmd_vel_stop.linear.z = 87;       // error message
    cmd_vel_stop.angular.x = status_; // error message

    MPC87InitialParameter();
    error_n += 1;
    compute_time.push_back(double(clock() - time_p) / CLOCKS_PER_SEC);
    return cmd_vel_stop;
  }

  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", &u_current_);

  nav_msgs::msg::Path predict_path;
  predict_path.header.frame_id = "world";
  predict_path.poses.clear();

  geometry_msgs::msg::Point way_;

  cout << "[BUBBLE_MPC_SOLVER] Compute, use horizon's to update parameter" << endl;
  clock_t time_start = clock();

  vector<double> bubble_obs(15);
  vector<double> ref_way_param(6);
  double dist_to_end = std::max(0.0, bubble_path_total_length_ - cumulative_distance_);

  for (int i = 0; i <= N_; i++)
  {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", &horizon_state_);

    vector<double> h_state({horizon_state_[0], horizon_state_[1], horizon_state_[2]});

    geometry_msgs::msg::PoseStamped predict_pose;
    predict_pose.header.frame_id = "world";
    predict_pose.pose.position.x = horizon_state_[0];
    predict_pose.pose.position.y = horizon_state_[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, horizon_state_[2]);
    predict_pose.pose.orientation.z = q.getZ();
    predict_pose.pose.orientation.w = q.getW();
    predict_path.poses.push_back(predict_pose);

    GetNearestBubbleAndObs(h_state, bubbles_info_, bubble_obs);
    ref_way_param = GetRefWayPose(h_state, bubble_path_, bubble_path_total_length_, cumulative_distance_, global_path_end_pose_);

    param_[0] = ref_way_param[0]; // ref x
    param_[1] = ref_way_param[1]; // ref y
    param_[2] = ref_way_param[2]; // ref th
    param_[3] = ref_way_param[3]; // way x
    param_[4] = ref_way_param[4]; // way y
    param_[5] = ref_way_param[5]; // way th

    param_[6] = bubble_obs[3];   // o_x0
    param_[7] = bubble_obs[4];   // o_y0
    param_[8] = bubble_obs[8];   // o_x1
    param_[9] = bubble_obs[9];   // o_y1
    param_[10] = bubble_obs[13]; // o_x2
    param_[11] = bubble_obs[14]; // o_y2

    param_[12] = bubble_obs[0];  // b_x
    param_[13] = bubble_obs[1];  // b_y
    param_[14] = bubble_obs[2];  // b_radius
    param_[15] = bubble_obs[5];  // b_x
    param_[16] = bubble_obs[6];  // b_y
    param_[17] = bubble_obs[7];  // b_radius
    param_[18] = bubble_obs[10]; // b_x
    param_[19] = bubble_obs[11]; // b_y
    param_[20] = bubble_obs[12]; // b_radius

    param_[21] = dist_to_end;

    param_[22] = over_final_bubble_;

    way_.x = param_[3]; // way x
    way_.y = param_[4]; // way y
    way_.z = param_[5]; // way th

    // horizon的
    bubble_model_acados_update_params(capsule_, i, param_.data(), np_);

    if (i % 2 == 0)
    {
      cout << "[BUBBLE_MPC_SOLVER] compute horzion, i = " << i << endl;
      cout << "horizon_state(x,y):(" << horizon_state_[0] << "," << horizon_state_[1] << ") " << endl;
      cout << "ref(x,y):(" << ref_way_param[0] << "," << ref_way_param[1] << ") " << endl;
      cout << "way(x,y):(" << ref_way_param[3] << "," << ref_way_param[4] << ") " << endl;
      cout << "dis to way: " << (pow(horizon_state_[0] - ref_way_param[3], 2) + pow(horizon_state_[1] - ref_way_param[4], 2)) << endl;
      cout << "nearest obstacle point(x,y) : (" << bubble_obs[3] << "," << bubble_obs[4] << ")" << endl;
      cout << "second obstacle point(x,y) : (" << bubble_obs[8] << "," << bubble_obs[9] << ")" << endl;
      cout << "third obstacle point(x,y) : (" << bubble_obs[13] << "," << bubble_obs[14] << ")" << endl;
      cout << "Obstacle Current constraint : first = " << (pow(horizon_state_[0] - bubble_obs[3], 2) + pow(horizon_state_[1] - bubble_obs[4], 2)) - pow(0.35, 2) << "," << "second = " << (pow(horizon_state_[0] - bubble_obs[8], 2) + pow(horizon_state_[1] - bubble_obs[9], 2)) - pow(0.35, 2) << "," << "third = " << (pow(horizon_state_[0] - bubble_obs[13], 2) + pow(horizon_state_[1] - bubble_obs[14], 2)) - pow(0.35, 2) << "," << endl;
    }
  }

  clock_t time_end = clock();
  cout << "compute Excuting time:" << double(time_end - time_start) / CLOCKS_PER_SEC << endl;

  way_point = way_;
  output_path = predict_path;

  compute_time.push_back(double(clock() - time_p) / CLOCKS_PER_SEC);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = u_current_[0];
  cmd_vel.angular.z = u_current_[1];

  return cmd_vel;
}

void BubbleMpcSolver::PrintComputeTime()
{
  double total_time = 0;
  for (auto time_p : compute_time)
  {
    total_time += time_p;
  }
  cout << "====================================================" << endl;
  cout << "[BUBBLE_MPC_SOLVER_FINISH] " << endl;
  cout << " Total compute numbers:" << compute_time.size() << endl;
  cout << " Total error numbers:" << error_n << endl;
  cout << " Average compute time:" << total_time / compute_time.size() << endl;
  cout << "====================================================" << endl;
}

// way_len = 13 在header裡, ref_pt_offset = 1 在上面
