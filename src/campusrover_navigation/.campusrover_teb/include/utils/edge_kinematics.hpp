#ifndef EDGE_KINEMATICS_HPP
#define EDGE_KINEMATICS_HPP

#include <cmath>
#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "vertex_manager.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"

class EdgeKinematicsDiffDrive : public BaseEdge<2>{
    public:

        std::shared_ptr<TebConfig> cfg;
        std::shared_ptr<VertexManager> vm;

        EdgeKinematicsDiffDrive(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* conf1 = static_cast<VertexPose*>(vertexs[0].get());
            VertexPose* conf2 = static_cast<VertexPose*>(vertexs[1].get());

            Vector2d deltaS = conf2->position() - conf1->position();
            Vector2d deltaS_via_2 = vm->vertex_task[conf2->id]->via_pose.topRows(2) - conf2->position();
            
            error[0] = fabs(( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] - ( sin(conf1->theta())+sin(conf2->theta()) ) * deltaS[0] );

            
            error[1] = 0;
            if(deltaS_via_2.norm() <= cfg->goal_tolerance.viapoint_xy_tolerance){
                Vector2d angle_vec1 ( cos(conf2->theta()), sin(conf2->theta()) );
                Vector2d angle_vec2 ( cos(vm->vertex_task[conf2->id]->via_pose[2]), sin(vm->vertex_task[conf2->id]->via_pose[2]) );

                error[1] = EdgeUtils::penaltyBoundFromBelow(angle_vec1.dot(angle_vec2), cfg->trajectory.forward_drive_theta, 0);
            }else{
                Vector2d angle_vec ( cos(conf2->theta()), sin(conf2->theta()) );

                error[1] = EdgeUtils::penaltyBoundFromBelow(deltaS_via_2.normalized().dot(angle_vec), cfg->trajectory.forward_drive_theta, 0);
            }

            return error;
        }

        void set_vertex_manager(std::shared_ptr<VertexManager> _vm){
            vm = _vm;
        }
};

// class EdgeKinematicsCarlike : public BaseEdge<2>{
//     public:

//         double dt;
//         std::shared_ptr<TebConfig> cfg;

//         EdgeKinematicsCarlike(double _dt, std::shared_ptr<TebConfig> _cfg){
//             dt = _dt;
//             cfg = _cfg;
//         }

//         virtual VectorXdd compute_error(double vertex_time_stamp) override{
//             VertexPose* conf1 = static_cast<VertexPose*>(vertexs[0].get());
//             VertexPose* conf2 = static_cast<VertexPose*>(vertexs[1].get());

//             Vector2d deltaS = conf2->position() - conf1->position();

//             error[0] = fabs( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] - ( sin(conf1->theta())+sin(conf2->theta()) ) * deltaS[0] );

//             double angle_diff = EdgeUtils::normalize_theta( conf2->theta() - conf1->theta() );
//             if (angle_diff == 0){
//                 error[1] = 0; // straight line motion
//             }else if (cfg->trajectory.exact_arc_length){ // use exact computation of the radius
//                 error[1] = EdgeUtils::penaltyBoundFromBelow(fabs(deltaS.norm()/(2*sin(angle_diff/2))), cfg->robot.min_turning_radius, 0.0);
//             }else{
//                 error[1] = EdgeUtils::penaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff), cfg->robot.min_turning_radius, 0.0); 
//             }

//             return error;
//         }

//         void set_param(std::shared_ptr<TebConfig> _cfg){
//             cfg = _cfg;
//         };
// };

#endif