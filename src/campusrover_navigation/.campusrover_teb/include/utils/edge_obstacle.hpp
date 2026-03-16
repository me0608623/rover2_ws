#ifndef EDGE_OBSTACLE_HPP
#define EDGE_OBSTACLE_HPP

#include <cmath>
#include "fgo/factor_graph_utils.hpp"
#include "vertex_pose.hpp"
#include "vertex_manager.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"
#include "timed_elastic_band_utils.hpp"

class EdgeObstacle : public BaseEdge<1>{
    public:

        std::shared_ptr<TebConfig> cfg;
        std::vector<Obstacle> obs;
        std::shared_ptr<VertexManager> vm;

        EdgeObstacle(std::shared_ptr<TebConfig> _cfg){
            cfg = _cfg;
        }

        virtual VectorXdd compute_error(double vertex_time_stamp) override{
            VertexPose* bandpt1 = static_cast<VertexPose*>(vertexs[0].get());

            double dist = 0, dist0, dist1;
            vm->intial_obs_report(bandpt1->id);
            for(int i = 0; i < obs.size(); i++){
                Vector2d mean = obs[i].mean;
                Vector2d mean_ = obs[i].mean + obs[i].speed * bandpt1->time_stamp;

                dist0 = exp(-0.5 * pow((bandpt1->position() - mean).norm(),2) / (sqrt(obs[i].dim + cfg->robot.body_width)/cfg->obstacles.obs_cov_rate));
                dist1 = exp(-0.5 * pow((bandpt1->position() - mean_).norm(),2) / (sqrt(obs[i].dim + cfg->robot.body_width)/cfg->obstacles.obs_cov_rate));
                dist += (dist0 > dist1) ? dist0 : dist1;
                
                if(obs[i].is_dynamic){
                    Obstacle obs_;
                    obs_.mean = mean_;
                    obs_.speed = obs[i].speed;
                    obs_.dim = obs[i].dim;

                    if(vm->is_meet_dynamic(obs_, bandpt1->position())){
                        vm->report_obs_status(bandpt1->id, true, obs_);
                    }else{
                        vm->report_obs_status(bandpt1->id, false, obs_);
                    }
                }
            }
           
            error[0] = dist;

            return error;
        }

        void setObs(std::vector<Obstacle> _obs){
            obs = _obs;
        }

        void set_vertex_manager(std::shared_ptr<VertexManager> _vm){
            vm = _vm;
        }
};
#endif
