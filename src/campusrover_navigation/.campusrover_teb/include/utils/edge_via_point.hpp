#ifndef EDGE_VIA_POINT_HPP
#define EDGE_VIA_POINT_HPP

#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "vertex_manager.hpp"
#include "vertex_manager.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"

class EdgeViaPoint : public BaseEdge<2>{
    public:

        double max_path_length;
        Eigen::Vector3d via_point;
        std::shared_ptr<TebConfig> cfg;
        std::shared_ptr<VertexManager> vm;

        EdgeViaPoint(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
            max_path_length = cfg->robot.max_vel_x * cfg->trajectory.dt_ref;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* bandpt1 = static_cast<VertexPose*>(vertexs[0].get());
            VertexPose* bandpt2 = static_cast<VertexPose*>(vertexs[1].get());
            
            via_point = vm->vertex_task[bandpt2->id]->via_pose;
            Vector2d deltaS = via_point.topRows(2) - bandpt1->position();

            Vector2d goal;
            if(deltaS.norm() > max_path_length){
                goal = bandpt1->position() +  deltaS.normalized() * max_path_length;
            }else{
                goal = via_point.topRows(2);
            }

            error[0] = (goal - bandpt2->position()).norm();

            error[1] = 0;
            if(deltaS.norm() <= cfg->goal_tolerance.viapoint_xy_tolerance){
                error[1] = fabs(EdgeUtils::normalize_theta(via_point[2] - bandpt2->theta()));
            }else{
                double theta = atan2(deltaS.y(), deltaS.x());
                error[1] = fabs(EdgeUtils::normalize_theta(theta - bandpt2->theta()));
            }

            return error;
        }

        void setViaPoint(Eigen::Vector3d _via_point){
            via_point = _via_point;
        }

        void set_vertex_manager(std::shared_ptr<VertexManager> _vm){
            vm = _vm;
        }
};

#endif

