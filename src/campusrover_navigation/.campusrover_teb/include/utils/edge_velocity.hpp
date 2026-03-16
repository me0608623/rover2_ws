#ifndef EDGE_VELOCITY_HPP
#define EDGE_VELOCITY_HPP

#include <cmath>
#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"

class EdgeVelocity : public BaseEdge<2>{
    public:
        std::shared_ptr<TebConfig> cfg;
        Eigen::Vector3d via_point;

        EdgeVelocity(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* conf1 = static_cast<VertexPose*>(vertexs[0].get());
            VertexPose* conf2 = static_cast<VertexPose*>(vertexs[1].get());
            double dt = conf2->time_stamp - conf1->time_stamp;

            Vector2d deltaS = conf2->position() - conf1->position();
            
            double dist = deltaS.norm();
            double angle_diff = EdgeUtils::normalize_theta(conf2->theta() - conf1->theta());

            Vector2d angle_vec ( cos(conf2->theta()), sin(conf2->theta()) );
            double dir = (deltaS.dot(angle_vec) >= 0) ? 1 : -1;

            if (cfg->trajectory.exact_arc_length && fabs(angle_diff) >= cfg->trajectory.exact_arc_length_theta){
                // actual arg length!
                double radius =  dist/(2*sin(angle_diff/2));
                dist = fabs( angle_diff * radius );
            }

            double vel = dir*dist / dt;
            double omega = angle_diff / dt;

            double max_rate = (fabs(omega) > cfg->trajectory.max_rate_omega_min_limit) ? 1 - ((fabs(omega)-cfg->trajectory.max_rate_omega_min_limit) / (cfg->robot.max_vel_theta-cfg->trajectory.max_rate_omega_min_limit)) : 1.0;
            max_rate = (max_rate < cfg->trajectory.max_rate_omega_min_vel) ? cfg->trajectory.max_rate_omega_min_vel : max_rate;
            //max_rate = 1.0;

            double goal_dist = (via_point.topRows(2) - conf2->position()).norm();
            double max_rate_goal = (goal_dist < cfg->trajectory.robot2goal_dist) ?  (goal_dist / cfg->trajectory.robot2goal_dist) : 1.0;
            max_rate_goal = (max_rate_goal < cfg->trajectory.max_rate_goal_min) ? cfg->trajectory.max_rate_goal_min : max_rate_goal;
            //max_rate_goal = 1.0;

            max_rate = (max_rate < max_rate_goal) ? max_rate : max_rate_goal;
            max_rate = (max_rate < cfg->trajectory.max_rate_dynamic_obs) ? max_rate : cfg->trajectory.max_rate_dynamic_obs;
            //max_rate = 1.0;

            error[0] = EdgeUtils::penaltyBoundToInterval(vel, -cfg->robot.max_vel_x_backwards * max_rate, cfg->robot.max_vel_x * max_rate, 0);
            error[1] = EdgeUtils::penaltyBoundToInterval(omega, cfg->robot.max_vel_theta, 0);
            return error;
        }

        void setViaPoint(Eigen::Vector3d _via_point){
            via_point = _via_point;
        }


        MatrixXdd compute_jacobian(std::shared_ptr<Vertex> vertex){
            jacobian = MatrixXdd::Zero(error.rows(), obj_vertex->x.rows());

            for(int i = 0; i < jacobian.cols(); i++){
                obj_vertex->x[i] = obj_vertex->x_origin[i] + DERIV_STEP;
                VectorXdd error_2 = compute_error(vertex->time_stamp);

                obj_vertex->x[i] = obj_vertex->x_origin[i] - DERIV_STEP;
                VectorXdd error_1 = compute_error(vertex->time_stamp);

                obj_vertex->x[i] = obj_vertex->x_origin[i];
                jacobian.col(i) = (error_2 - error_1) / (2*DERIV_STEP);
            }

            jacobian(0, 2) = 0;

            return jacobian;
        }
};

// class EdgeVelocityHolonomic : public BaseEdge<3>{
//     public:

//         double dt;
//         std::shared_ptr<TebConfig> cfg;

//         EdgeVelocityHolonomic(double _dt, std::shared_ptr<TebConfig> _cfg){
//             dt = _dt;
//             cfg = _cfg;
//         }

//         virtual VectorXdd compute_error(double vertex_time_stamp) override{
//             VertexPose* conf1 = static_cast<VertexPose*>(vertexs[0].get());
//             VertexPose* conf2 = static_cast<VertexPose*>(vertexs[1].get());

//             Vector2d deltaS = conf2->position() - conf1->position();
            
//             double cos_theta1 = std::cos(conf1->theta());
//             double sin_theta1 = std::sin(conf1->theta());

//             double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
//             double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
            
//             double vx = r_dx / dt;
//             double vy = r_dy / dt;
//             double omega = EdgeUtils::normalize_theta(conf2->theta() - conf1->theta()) / dt;


//             double max_vel_trans_remaining_y;
//             double max_vel_trans_remaining_x;
//             max_vel_trans_remaining_y = std::sqrt(std::max(0.0, cfg->robot.max_vel_trans * cfg->robot.max_vel_trans - vx * vx)); 
//             max_vel_trans_remaining_x = std::sqrt(std::max(0.0, cfg->robot.max_vel_trans * cfg->robot.max_vel_trans - vy * vy)); 

//             double max_vel_y = std::min(max_vel_trans_remaining_y, cfg->robot.max_vel_y);
//             double max_vel_x = std::min(max_vel_trans_remaining_x, cfg->robot.max_vel_x);
//             double max_vel_x_backwards = std::min(max_vel_trans_remaining_x, cfg->robot.max_vel_x_backwards);

//             error[0] = EdgeUtils::penaltyBoundToInterval(vx, -max_vel_x_backwards, max_vel_x, cfg->optim.penalty_epsilon);
//             error[1] = EdgeUtils::penaltyBoundToInterval(vy, max_vel_y, 0.0); // we do not apply the penalty epsilon here, since the velocity could be close to zero
//             error[2] = EdgeUtils::penaltyBoundToInterval(omega, cfg->robot.max_vel_theta,cfg->optim.penalty_epsilon);
        
//             return error;
//         }

//         void set_param(std::shared_ptr<TebConfig> _cfg){
//             cfg = _cfg;
//         };
// };

#endif
