#ifndef EDGE_SHORTEST_PATH_HPP
#define EDGE_SHORTEST_PATH_HPP

#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"

class EdgeShortestPath : public BaseEdge<2>{
    public:

        double max_path_length;
        std::shared_ptr<TebConfig> cfg;

        EdgeShortestPath(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
            max_path_length = cfg->robot.max_vel_x * cfg->trajectory.dt_ref;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* pose1 = static_cast<VertexPose*>(vertexs[0].get());
            VertexPose* pose2 = static_cast<VertexPose*>(vertexs[1].get());

            error[0] = (pose2->position() - pose1->position()).norm();
            error[1] = fabs(EdgeUtils::normalize_theta(pose2->theta() - pose1->theta()));
            
            return error;
        }
};

#endif