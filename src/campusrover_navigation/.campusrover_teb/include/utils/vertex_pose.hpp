#ifndef VERTEX_POSE_HPP
#define VERTEX_POSE_HPP

#include "fgo/factor_graph_utils.hpp"
#include "teb_config.hpp"
#include "timed_elastic_band_utils.hpp"

class VertexPose : public BaseVertex<3>{
    public:

        VertexPose(VectorXdd _x, bool _fixed) : BaseVertex(_x, _fixed){
        }

        VectorXdd& pose(){
            return x;
        }

        Vector2d position(){
            Vector2d out = x.topRows(2);
            return out;
        }
        
        double& px(){
            return x(0, 0);
        }

        double& py(){
            return x(1, 0);
        }

        double& theta(){
            return x(2, 0);
        }
};

#endif