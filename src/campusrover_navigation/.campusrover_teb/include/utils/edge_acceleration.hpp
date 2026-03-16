#ifndef EDGE_ACCELERATION_HPP
#define EDGE_ACCELERATION_HPP

#include <cmath>
#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"

class EdgeAcceleration : public BaseEdge<2>{
    public:
        std::shared_ptr<TebConfig> cfg;

        EdgeAcceleration(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* pose1 = static_cast<VertexPose*>(vertexs[0].get());
            VertexPose* pose2 = static_cast<VertexPose*>(vertexs[1].get());
            VertexPose* pose3 = static_cast<VertexPose*>(vertexs[2].get());
            double dt_1 = pose2->time_stamp - pose1->time_stamp;
            double dt_2 = pose3->time_stamp - pose2->time_stamp;

            Vector2d diff1 = pose2->position() - pose1->position();
            Vector2d diff2 = pose3->position() - pose2->position();

            double dist1 = diff1.norm();
            double dist2 = diff2.norm();

            double angle_diff1 = EdgeUtils::normalize_theta(pose2->theta() - pose1->theta());
            double angle_diff2 = EdgeUtils::normalize_theta(pose3->theta() - pose2->theta());

            Vector2d angle_vec1 ( cos(pose2->theta()), sin(pose2->theta()) );
            double dir1 = (diff1.dot(angle_vec1) >= 0) ? 1 : -1;

            Vector2d angle_vec2 ( cos(pose3->theta()), sin(pose3->theta()) );
            double dir2 = (diff2.dot(angle_vec2) >= 0) ? 1 : -1;

            if(cfg->trajectory.exact_arc_length){
                if(fabs(angle_diff1) >= cfg->trajectory.exact_arc_length_theta){
                    double radius =  dist1/(2*sin(angle_diff1/2));
                    dist1 = fabs( angle_diff1 * radius ); // actual arg length!
                }
                if(fabs(angle_diff2) >= cfg->trajectory.exact_arc_length_theta){
                    double radius =  dist2/(2*sin(angle_diff2/2));
                    dist2 = fabs( angle_diff2 * radius ); // actual arg length!
                }
            }

            double vel1 = dir1*dist1 / dt_1;
            double vel2 = dir2*dist2 / dt_2;

            // double acc_lin  = (vel2 - vel1)*2 / (dt_1 + dt_2);
            double acc_lin  = (vel2 - vel1) / (cfg->trajectory.update_dt);

            error[0] = EdgeUtils::penaltyBoundToInterval(acc_lin,cfg->robot.acc_lim_x, 0);

            double omega1 = angle_diff1 / dt_1;
            double omega2 = angle_diff2 / dt_2;
            // double acc_rot  = (omega2 - omega1)*2 / (dt_1 + dt_2);
            double acc_rot  = (omega2 - omega1) / (cfg->trajectory.update_dt);

            error[1] = EdgeUtils::penaltyBoundToInterval(acc_rot,cfg->robot.acc_lim_theta, 0);

            return error;
        }
};

#endif 