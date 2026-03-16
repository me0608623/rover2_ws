from acados_template import AcadosOcp, AcadosOcpSolver, ocp_get_default_cmake_builder
from model import export_bubble_model
from casadi import *
import math
import numpy as np # 確保引入 numpy

def gen_model(prediction_steps = 5, horizon_duration = 1, 
              max_v = 0.4, min_v = -0.3, 
              max_w = 0.25, min_w = -0.25,
              angle_threshold = 0.1, distance_threshold = 1.0,
              robot_radius = 0.32):

    model = export_bubble_model(horizon_duration/prediction_steps)
    
    dt = horizon_duration/prediction_steps

    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = prediction_steps # set dimensions
    ocp.solver_options.tf = horizon_duration # set prediction horizon

    # current state variables
    x1 = ocp.model.x[0]
    y1 = ocp.model.x[1]
    th = ocp.model.x[2]

    v = ocp.model.u[0]
    w = ocp.model.u[1]

    # parameter variables
    ref_x = ocp.model.p[0]
    ref_y = ocp.model.p[1]
    ref_th = ocp.model.p[2]

    way_x = ocp.model.p[3]
    way_y = ocp.model.p[4]
    way_th = ocp.model.p[5]

    # --- 獲取三個障礙物的座標參數 ---
    o_x0 = ocp.model.p[6]
    o_y0 = ocp.model.p[7]
    o_x1 = ocp.model.p[8]
    o_y1 = ocp.model.p[9]
    o_x2 = ocp.model.p[10]
    o_y2 = ocp.model.p[11]
    
    b_x0 = ocp.model.p[12]
    b_y0 = ocp.model.p[13]
    b_radius0 = ocp.model.p[14]
    b_x1 = ocp.model.p[15]
    b_y1 = ocp.model.p[16]
    b_radius1 = ocp.model.p[17]
    b_x2 = ocp.model.p[18]
    b_y2 = ocp.model.p[19]
    b_radius2 = ocp.model.p[20]
    
    dist_to_end = ocp.model.p[21]
    
    # Security Mechanism
    over_final_bubble = ocp.model.p[22]

    # ... (中間的 Error 計算與 Cost J_cost 計算部分保持不變) ...
    error_x = way_x - x1
    error_y = way_y - y1
    error_th = way_th - th
    error_th = math.pi * ((1 - cos(error_th))/2)
    # error = vertcat(error_x, error_y, error_th) # 未使用，註解掉

    v_ref_way_x = way_x - ref_x
    v_ref_way_y = way_y - ref_y 
    l_ref_way = sqrt((v_ref_way_x)**2 + (v_ref_way_y)**2)
    unit_v_ref_way_x = v_ref_way_x/l_ref_way
    unit_v_ref_way_y = v_ref_way_y/l_ref_way
    unit_n_ref_way_x = -unit_v_ref_way_y
    unit_n_ref_way_y = unit_v_ref_way_x

    v_robot_ref_x = ref_x - x1  
    v_robot_ref_y = ref_y - y1  
    
    # Bubble 相關計算
    bubble0_robot_dis_x = (x1 - b_x0)
    bubble0_robot_dis_y = (y1 - b_y0)
    bubble0_robot_dis = sqrt(bubble0_robot_dis_x**2 + bubble0_robot_dis_y**2)
    bubble1_robot_dis_x = (x1 - b_x1)
    bubble1_robot_dis_y = (y1 - b_y1)
    bubble1_robot_dis = sqrt(bubble1_robot_dis_x**2 + bubble1_robot_dis_y**2)
    bubble2_robot_dis_x = (x1 - b_x2)
    bubble2_robot_dis_y = (y1 - b_y2)
    bubble2_robot_dis = sqrt(bubble2_robot_dis_x**2 + bubble2_robot_dis_y**2)
    
    robot_in_bubble_0 = b_radius0 - bubble0_robot_dis
    robot_in_bubble_1 = b_radius1 - bubble1_robot_dis
    robot_in_bubble_2 = b_radius2 - bubble2_robot_dis
    robot_in_bubble = fmax(robot_in_bubble_0, fmax(robot_in_bubble_1, robot_in_bubble_2))
    r_min = fmin(b_radius0, fmin(b_radius1, b_radius2))

    e_progress = sqrt(error_x**2 + error_y**2)
    e_contour = unit_n_ref_way_x * v_robot_ref_x + unit_n_ref_way_y * v_robot_ref_y
    e_lag = unit_v_ref_way_x * v_robot_ref_x + unit_v_ref_way_y * v_robot_ref_y
    e_bubble = if_else(robot_in_bubble < 0, -robot_in_bubble, 0)
    
    # Cost Weights
    Q_progress = 5.0
    Q_contour = 5.0
    Q_lag = 1.0
    Q_angle = 5.0
    Q_bubble = 1.0

    J_progress = pow(e_progress, 2)* Q_progress
    J_contour = pow(e_contour, 2)* Q_contour
    J_lag = pow(e_lag, 2)* Q_lag
    J_angle = pow(error_th, 2) * Q_angle 
    J_bubble = pow(e_bubble, 2)* Q_bubble

    # Variable Speed Control Logic
    R_ctrl_v_base = if_else(e_progress < 0.1, SX(10.0),
                    if_else(e_progress < 0.15, SX(5.5),
                    if_else(e_progress < 0.2, SX(3.5),
                    if_else(e_progress < 0.3, SX(2.0),
                    if_else(e_progress < 0.4, SX(1.5),
                    if_else(e_progress < 0.5, SX(1.0),
                    if_else(e_progress < 0.6, SX(0.9),
                    if_else(e_progress < 0.7, SX(0.8),
                    if_else(e_progress < 0.8, SX(0.7),
                    if_else(e_progress < 0.9, SX(0.6), SX(0.5)))))))))))
    
    # 半徑-速度關係：r_lo 以下強制很慢、r_hi 以上正常
    r_lo = SX(0.2)   # 低於這個就很危險（自行依場域調）
    r_hi = SX(0.35)   # 高於這個就當安全

    v_scale = if_else(r_min >= r_hi, SX(1.0),
              if_else(r_min <= r_lo, SX(0.2),
                      SX(0.2) + SX(0.8)*(r_min - r_lo)/(r_hi - r_lo)))
    R_ctrl_v = if_else(dist_to_end < 1.0, R_ctrl_v_base, SX(0.5))
    R_ctrl_v = R_ctrl_v / (v_scale*v_scale + SX(1e-6))
    R_ctrl_w = SX(0.5)

    J_ctrl_v = pow(v, 2)* R_ctrl_v
    J_ctrl_w = pow(w, 2)* R_ctrl_w

    J_cost =  J_progress + J_contour + J_lag + J_angle + J_ctrl_v + J_ctrl_w + J_bubble
    J_cost_e =  J_progress + J_contour + J_lag + J_angle + J_bubble

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = J_cost

    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_e = J_cost_e   

    # 1. 障礙物 0
    h_expr_0 = (x1 - o_x0)**2 + (y1 - o_y0)**2 - robot_radius**2
    h0 = if_else( (((o_x0 -(-878.0)) < 0.01) * ((o_y0 -(-878.0)) < 0.01)),
                878.0,
                h_expr_0)

    # 2. 障礙物 1
    h_expr_1 = (x1 - o_x1)**2 + (y1 - o_y1)**2 - robot_radius**2
    h1 = if_else( (((o_x1 -(-878.0)) < 0.01) * ((o_y1 -(-878.0)) < 0.01)),
                878.0,
                h_expr_1)

    # 3. 障礙物 2
    h_expr_2 = (x1 - o_x2)**2 + (y1 - o_y2)**2 - robot_radius**2
    h2 = if_else( (((o_x2 -(-878.0)) < 0.01) * ((o_y2 -(-878.0)) < 0.01)),
                878.0,
                h_expr_2)
    
    # Security Mechanism
    h_3 = over_final_bubble

    # 將三個障礙物約束條件
    # 這裡順序為: [h0, h1, h2, h3]
    ocp.model.con_h_expr = vertcat(h0, h1, h2, h_3)
    ocp.model.con_h_expr_e = vertcat(h0, h1, h2, h_3)

    # 控制項 u 的上下限
    ocp.constraints.lbu = np.array([min_v, min_w])
    ocp.constraints.ubu = np.array([max_v, max_w])
    ocp.constraints.idxbu = np.array([0, 1])

    # --- 關鍵修改：運行約束的上下限 (對應 vertcat 的順序) ---
    # 對應 [h0, h1, h2, h3]
    # 障礙物距離平方需 > 0 (lb=0.0)，上限設大值
    ocp.constraints.lh = np.array([0.0, 0.0, 0.0, -0.01])
    ocp.constraints.uh = np.array([999.0, 999.0, 999.0, 0.01])     

    # --- 關鍵修改：終點約束的上下限 ---
    ocp.constraints.lh_e = np.array([0.0, 0.0, 0.0, -0.01])
    ocp.constraints.uh_e = np.array([999.0, 999.0, 999.0, 0.01])

    ocp.constraints.x0 = np.array([0, 0, 0]) 
    
    # 參數初始化 (24個參數)
    ocp.parameter_values = np.zeros(23)

    ### qp_solver OPTIONS ###
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'EXACT' 
    ocp.solver_options.ext_cost_num_hess = 0    
    ocp.solver_options.integrator_type = 'DISCRETE' 
    ocp.solver_options.nlp_solver_type = 'SQP' 
    ocp.solver_options.nlp_solver_max_iter = 100 
    ocp.solver_options.nlp_solver_step_length = 1. 
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING' 
    ocp.solver_options.print_level = 0 

    cmake_builder = ocp_get_default_cmake_builder()
    AcadosOcpSolver(ocp, json_file='acados_ocp.json')#, cmake_builder=cmake_builder)

if __name__ == '__main__':
    gen_model()