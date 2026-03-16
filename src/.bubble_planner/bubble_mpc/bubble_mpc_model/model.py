#! /usr/local/python3
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, Function
import math

def export_bubble_model(dt):
    
    model_name = 'bubble_model'

    # States
    x1 = SX.sym('x1')
    y1 = SX.sym('y1')
    th = SX.sym('th')

    x = vertcat(x1, y1, th)

    # Control
    v = SX.sym('v')
    w = SX.sym('w')

    u = vertcat(v, w)

    # parameters
    ref_x = SX.sym('ref_x')
    ref_y = SX.sym('ref_y')
    ref_th = SX.sym('ref_th')
    way_x = SX.sym('way_x')
    way_y = SX.sym('way_y')
    way_th = SX.sym('way_th')
    o_x0 = SX.sym('o_x0')
    o_y0 = SX.sym('o_y0')
    o_x1 = SX.sym('o_x1')
    o_y1 = SX.sym('o_y1')
    o_x2 = SX.sym('o_x2')
    o_y2 = SX.sym('o_y2')
    b_x0 = SX.sym("b_x0")
    b_y0 = SX.sym("b_y0")
    b_radius0 = SX.sym("b_radius0")
    b_x1 = SX.sym("b_x1")
    b_y1 = SX.sym("b_y1")
    b_radius1 = SX.sym("b_radius1")
    b_x2 = SX.sym("b_x2")
    b_y2 = SX.sym("b_y2")
    b_radius2 = SX.sym("b_radius2")
    dist_to_end = SX.sym("dist_to_end")
    
    # Security Mechanism
    over_final_bubble = SX.sym('over_final_bubble')

    p = vertcat(ref_x, ref_y, ref_th, way_x, way_y, way_th, 
                o_x0, o_y0, o_x1, o_y1, o_x2, o_y2,
                b_x0, b_y0, b_radius0, b_x1, b_y1, b_radius1, b_x2, b_y2, b_radius2,
                dist_to_end, over_final_bubble)

    # Dynamic expression
    dx = v * dt * cos(th + w*dt/2)  # px移動量
    dy = v * dt * sin(th + w*dt/2)  # py移動量
    dth = w * dt  # 朝向角移動量
    
    x_next = x1 + dx
    y_next = y1 + dy
    th_next = th + dth

    dyn_expr = vertcat(x_next, y_next, th_next)
    
    model = AcadosModel()
    model.disc_dyn_expr = dyn_expr # acados 的 API name
    model.x = x
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model
