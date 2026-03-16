#ifndef EDGE_ENV_HPP
#define EDGE_ENV_HPP

#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "vertex_manager.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"
#include "timed_elastic_band_utils.hpp"

class EdgeEnv : public BaseEdge<1>{
    public:

        std::shared_ptr<CostMap> env;
        std::shared_ptr<TebConfig> cfg;
        std::shared_ptr<VertexManager> vm;

        EdgeEnv(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* bandpt1 = static_cast<VertexPose*>(vertexs[0].get());

            error[0] = env->at(bandpt1->px(), bandpt1->py());

            if(error[0] > cfg->obstacles.env_report_threshold){
                vm->report_env_status(bandpt1->id, true, bandpt1->pose());
            }else{
                vm->report_env_status(bandpt1->id, false, bandpt1->pose());
            }

            return error;
        }

        virtual MatrixXdd compute_jacobian(std::shared_ptr<Vertex> vertex) override{
            jacobian = MatrixXdd::Zero(error.rows(), obj_vertex->x.rows());

            for(int i = 0; i < jacobian.cols(); i++){
                obj_vertex->x[i] = obj_vertex->x_origin[i] + env->res*1.5;
                VectorXdd error_2 = compute_error(vertex->time_stamp);

                obj_vertex->x[i] = obj_vertex->x_origin[i] - env->res*1.5;
                VectorXdd error_1 = compute_error(vertex->time_stamp);

                obj_vertex->x[i] = obj_vertex->x_origin[i];
                jacobian.col(i) = (error_2 - error_1) / (3*env->res);
            }

            return jacobian;
        }

        void setEnv(std::shared_ptr<CostMap> _env){            
            env = _env;
        }

        void set_vertex_manager(std::shared_ptr<VertexManager> _vm){
            vm = _vm;
        }
};

#endif