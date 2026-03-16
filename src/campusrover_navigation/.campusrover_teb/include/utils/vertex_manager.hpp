#ifndef VERTEX_MANAGER_HPP
#define VERTEX_MANAGER_HPP

#include <iostream>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include "vertex_manager_utils.hpp"
#include "edge_utils.hpp"
#include "teb_config.hpp"
#include "timed_elastic_band_utils.hpp"

class VertexManager{
    public:
        int vertex_num;
        std::unordered_map<int, std::shared_ptr<VertexTask>> vertex_task;

        std::shared_ptr<TebConfig> cfg;
        Vector3d now_pose;
        Vector3d goal_pose;
        std::shared_ptr<CostMap> env;
        Eigen::MatrixXd global_path;
        std::vector<Obstacle> static_obs;
        std::vector<Obstacle> dynamic_obs;
        std::vector<Obstacle> env_obs;
        std::shared_ptr<ViaPt> vp;

        VertexManager(int vertex_size, std::shared_ptr<TebConfig> _cfg);

        void setNowPose(Vector3d _now_pose);
        void setGoalPose(Vector3d _goal_pose);
        void setEnv(std::shared_ptr<CostMap> _env);
        void setObs(std::vector<Obstacle> _obs);
        void setPath(MatrixXd &path);

        void initial_status();

        void intial_obs_report(int vertex_id);
        void report_obs_status(int vertex_id, int status, Obstacle obs);
        void report_env_status(int vertex_id, int status, Vector3d _env_pose);

        void compute_static_via_pt();
        bool update_goal();
        bool is_meet_dynamic(Obstacle obs, Vector2d pt);

    private:
        struct ClusterMap{
            bool is_block = false;
            bool is_compute = false;
            bool is_belong = false;
            int index;
            std::vector<int> item;
        };

        std::vector<int> block_static_obs_idxs;
        std::vector<int> block_dynamic_obs_idxs;
        std::vector<int> block_env_obs_idxs;

        std::vector<Obstacle> filtered_c_obs;

        bool check_is_block();
        void clustering_filtered_obs(std::vector<Obstacle> filtered_obs, std::vector<int> block_obs_idxs);

        void compute_obs_via_pt(bool is_compute_left, bool is_compute_right);

        void filter_env_via_pt();
        bool filter_dynamic_obs(bool &is_update_left, bool &is_update_right);
        bool filter_env_obs();
        bool decide_speed_max_rate();
};

VertexManager::VertexManager(int vertex_size, std::shared_ptr<TebConfig> _cfg){
    vertex_task.clear();
    vertex_num = vertex_size;

    for(int i = 0; i < vertex_size; i++){
        vertex_task[i] = std::make_shared<VertexTask>();
    }

    cfg = _cfg;
    vp = std::make_shared<ViaPt>();
}

void VertexManager::setNowPose(Vector3d _now_pose){
    now_pose = _now_pose;
}

void VertexManager::setGoalPose(Vector3d _goal_pose){
    goal_pose = _goal_pose;
}

void VertexManager::setEnv(std::shared_ptr<CostMap> _env){            
    env = _env;
}

void VertexManager::setObs(std::vector<Obstacle> _obs){
    static_obs.clear();
    for(int i = 0; i < _obs.size(); i++){
        if(!_obs[i].is_dynamic){
            static_obs.push_back(_obs[i]);
        }
    }
}

void VertexManager::setPath(MatrixXd &path){
    global_path = path;
}


void VertexManager::initial_status(){
    for(int i = 0; i < vertex_num; i++){
        vertex_task[i]->via_pose = goal_pose;
        vertex_task[i]->is_obs = false;
        vertex_task[i]->is_env = false;
    }
}

void VertexManager::intial_obs_report(int vertex_id){
    vertex_task[vertex_id]->dynamic_obs.clear();
}

void VertexManager::report_obs_status(int vertex_id, int status, Obstacle obs){
    vertex_task[vertex_id]->is_obs = status;
    if(status){
        vertex_task[vertex_id]->dynamic_obs.push_back(obs);
    }
}

void VertexManager::report_env_status(int vertex_id, int status, Vector3d _env_pose){
    vertex_task[vertex_id]->is_env = status;
    vertex_task[vertex_id]->env_pose = _env_pose;

}

void VertexManager::compute_static_via_pt(){
    vp->set_pose(now_pose, goal_pose);

    filtered_c_obs.clear();
    if(check_is_block()){
        
        clustering_filtered_obs(static_obs, block_static_obs_idxs);

        compute_obs_via_pt(true, true);

        filter_env_via_pt();

        vp->decide_side();

        for(int i = 0; i < vertex_num; i++){
            vertex_task[i]->via_pose = vp->get_via_pose();
        }
    }

    // if(filtered_c_obs.size() == 0){
    //     std::cout << "vp->initial : " << std::endl;

    //     vp->initial();
    // }
}

bool VertexManager::update_goal(){
    bool is_update = false, is_update_left = false, is_update_right = false;

    cfg->trajectory.max_rate_dynamic_obs = 1.0;

    Vector3d origin_goal_pose = vp->get_via_pose();

    if(filter_dynamic_obs(is_update_left, is_update_right)){

        clustering_filtered_obs(dynamic_obs, block_dynamic_obs_idxs);
        
        compute_obs_via_pt(is_update_left, is_update_right);

        filter_env_via_pt();

        vp->decide_side();
    }

    if(filtered_c_obs.size() == 0){
        vp->initial();
    }

    if(origin_goal_pose == vp->get_via_pose()){
        if(filter_env_obs()){

            clustering_filtered_obs(env_obs, block_env_obs_idxs);

            compute_obs_via_pt(!vp->is_left_env, !vp->is_right_env);

            vp->decide_side();
        }
    }

    is_update = (origin_goal_pose == vp->get_via_pose()) ? decide_speed_max_rate() : true;

    if(origin_goal_pose != vp->get_via_pose()){
        for(int i = 0; i < vertex_num; i++){
            vertex_task[i]->via_pose = vp->get_via_pose();
        }
    }

    return is_update;
}

bool VertexManager::is_meet_dynamic(Obstacle obs, Vector2d pt){
    return (obs.mean - pt.topRows(2)).norm() < obs.dim + cfg->robot.body_width;
}

bool VertexManager::check_is_block(){
    bool is_block = false, is_in_area = false;
    bool check_is_block = false;
    double angular_obs_now, angular_goal_now, obs_delta_theta, delta_theta, dist;
    Vector2d deltaS1, deltaS2;
    
    deltaS1 = goal_pose.topRows(2) - now_pose.topRows(2);
    angular_goal_now = atan2(deltaS1.y(), deltaS1.x());

    for(int i = 0; i < static_obs.size(); i++){
        deltaS2 = static_obs[i].mean - now_pose.topRows(2);
        dist = deltaS2.norm();
        
        is_in_area = dist < static_obs[i].dim + cfg->trajectory.robot2goal_dist;
        
        angular_obs_now = atan2(deltaS2.y(), deltaS2.x());
        obs_delta_theta = fabs(asin((cfg->robot.body_width + static_obs[i].dim) / dist));
        delta_theta = fabs(EdgeUtils::normalize_theta(angular_goal_now - angular_obs_now));

        is_block = delta_theta <= obs_delta_theta || ((cfg->robot.body_width + static_obs[i].dim) / dist) > 1;
        
        check_is_block = (is_in_area && is_block) || check_is_block;

        if(is_in_area && is_block){
            block_static_obs_idxs.push_back(i);
        }
    }

    return check_is_block;
}

void VertexManager::clustering_filtered_obs(std::vector<Obstacle> filtered_obs, std::vector<int> block_obs_idxs){
    int size = filtered_obs.size();
    double dist;
    std::vector<ClusterMap> cluster_maps;

    // register cluster maps 
    for(int i = 0; i < size; i++){
        ClusterMap cm;
        cm.index = i;
        for(int j = 0; j < size; j++){
            if(i != j){
                if((filtered_obs[i].mean - filtered_obs[j].mean).norm() < filtered_obs[i].dim + filtered_obs[j].dim + 2*cfg->robot.body_width){
                    cm.item.push_back(j);
                }
            }
        }
        cluster_maps.push_back(cm);
    }

    // mark is_block in cluster maps
    for(int i = 0; i < block_obs_idxs.size(); i++){
        cluster_maps[block_obs_idxs[i]].is_block = true;
    }

    // for(int i = 0; i < size; i++){
    //     std::cout << "index : " << cluster_maps[i].index << ">> itme : ";

    //     for(int j = 0; j < cluster_maps[i].item.size(); j++){
    //         std::cout << cluster_maps[i].item[j] << ",";
    //     }

    //     std::cout << std::endl;
    // }

    // add item to belong index and mark is_block 
    for(int i = 0; i < size; i++){
        if(!cluster_maps[i].is_belong){
            cluster_maps[i].is_compute = true;
            cluster_maps[i].is_belong = true;

            for(int j = 0; j < cluster_maps[i].item.size(); j++){
                cluster_maps[cluster_maps[i].item[j]].is_belong = true;
            }

            for(int j = 0; j < cluster_maps[i].item.size(); j++){

                for(int k = 0; k < cluster_maps[cluster_maps[i].item[j]].item.size(); k++){
                    if(!cluster_maps[cluster_maps[cluster_maps[i].item[j]].item[k]].is_belong){
                        cluster_maps[i].item.push_back(cluster_maps[cluster_maps[i].item[j]].item[k]);
                        cluster_maps[cluster_maps[cluster_maps[i].item[j]].item[k]].is_belong = true;

                    }
                }

                cluster_maps[cluster_maps[i].item[j]].is_belong = true;
                cluster_maps[i].is_block = cluster_maps[i].is_block || cluster_maps[cluster_maps[i].item[j]].is_block;
            }
        }
    }
    
    // get clustering filter result
    filtered_c_obs.clear();
    for(int i = 0; i < cluster_maps.size(); i++){
        if(cluster_maps[i].is_compute && cluster_maps[i].is_block){
            filtered_c_obs.push_back(filtered_obs[cluster_maps[i].index]);

            for(int j = 0; j < cluster_maps[i].item.size(); j++){
                filtered_c_obs.push_back(filtered_obs[cluster_maps[i].item[j]]);
            }
        }
    }
}

void VertexManager::compute_obs_via_pt(bool is_compute_left, bool is_compute_right){
    bool is_via_left = true, is_via_right = true;
    double angular_goal_now, angular_obs_now, asin_rate, delta_theta, len_left, len_right;
    double theta_left_delta, theta_right_delta, max_theta_left = 0, min_theta_right = 0;;
    double theta_left, theta_right, via_theta_left, via_theta_right;
    Vector2d deltaS_obs_now;
    Vector2d deltaS_goal_now = goal_pose.topRows(2) - now_pose.topRows(2);


    for(int i = 0; i < filtered_c_obs.size(); i++){
        deltaS_obs_now = filtered_c_obs[i].mean - now_pose.topRows(2);

        angular_goal_now = atan2(deltaS_goal_now.y(), deltaS_goal_now.x());
        angular_obs_now = atan2(deltaS_obs_now.y(), deltaS_obs_now.x());
        asin_rate = (cfg->robot.body_width + filtered_c_obs[i].dim) / deltaS_obs_now.norm();
        delta_theta = fabs(asin(((asin_rate <= 1) ? asin_rate : 1)));

        theta_left = EdgeUtils::normalize_theta(angular_obs_now + delta_theta);
        theta_right = EdgeUtils::normalize_theta(angular_obs_now - delta_theta);

        theta_left_delta = EdgeUtils::normalize_theta(theta_left - angular_goal_now);
        theta_right_delta = EdgeUtils::normalize_theta(theta_right - angular_goal_now);

        if(theta_left_delta >= 0){
            if(theta_left_delta > max_theta_left){
                max_theta_left = theta_left_delta;
                via_theta_left = theta_left;
                len_left = deltaS_obs_now.norm();
            }
        }else{
            if(theta_right_delta >= 0){
                is_via_left = false;
            }
        }

        if(theta_right_delta < 0){
            if(theta_right_delta < min_theta_right){
                min_theta_right = theta_right_delta;
                via_theta_right = theta_right;
                len_right = deltaS_obs_now.norm();
            }
        }else{
            if(theta_left_delta < 0){
                is_via_right = false;
            }
        }
    }

    if(is_compute_left){
        vp->left_pose = Vector3d(len_left*cos(via_theta_left) + now_pose[0], len_left*sin(via_theta_left) + now_pose[1], via_theta_left);
        vp->is_left_via = is_via_left;
    }

    if(is_compute_right){
        vp->right_pose = Vector3d(len_right*cos(via_theta_right) + now_pose[0], len_right*sin(via_theta_right) + now_pose[1], via_theta_right);
        vp->is_right_via = is_via_right;
    }

}

void VertexManager::filter_env_via_pt(){
    vp->is_left_env = (!vp->is_left_env) ? env->at(vp->left_pose[0], vp->left_pose[1]) != 0 : true;
    vp->is_right_env = (!vp->is_right_env) ? env->at(vp->right_pose[0], vp->right_pose[1]) != 0 : true;
}

bool VertexManager::filter_env_obs(){
    bool is_add_obstalce = false;

    env_obs = static_obs;
    block_env_obs_idxs = block_static_obs_idxs;
    for(int i = 0; i < vertex_num; i++){
        if(vertex_task[i]->is_env){
            Vector3d min_path_pose;

            if(!TEBUtils::get_goal_pt(vertex_task[i]->env_pose, min_path_pose, global_path, 0)){
                continue;
            }
            
            bool is_via_ok;
            double delta_theta, env_theta, len;

            Vector2d deltaS_min_now = min_path_pose.topRows(2) - now_pose.topRows(2);
            Vector2d deltaS_env_now = vertex_task[i]->env_pose.topRows(2) - now_pose.topRows(2);
            double angular_min_now = atan2(deltaS_min_now.y(), deltaS_min_now.x());
            double angular_env_now = atan2(deltaS_env_now.y(), deltaS_env_now.x());

            double len_env_now = deltaS_env_now.norm();

            double dir = EdgeUtils::normalize_theta(angular_min_now-angular_env_now);
            if(dir != 0){
                Vector3d env_via;
                dir = (dir > 0) ? 1 : -1;

                for(double j = 1; j <= 5 ; j+=0.5){
                    delta_theta = fabs(atan2(j*env->res, len_env_now));

                    env_theta = EdgeUtils::normalize_theta(angular_env_now + dir*delta_theta);

                    len = sqrt(pow(len_env_now, 2) + pow(j*env->res, 2));

                    env_via = Vector3d(len*cos(env_theta) + now_pose[0], len*sin(env_theta) + now_pose[1], env_theta);

                    is_via_ok = env->at(env_via.x(), env_via.y()) == 0;

                    if(dir == 1 && is_via_ok){
                        Obstacle o;
                        o.dim = 0;
                        o.mean = env_via.topRows(2);
                        env_obs.push_back(o);
                        block_env_obs_idxs.push_back(env_obs.size()-1);

                        vp->is_right_env = true;
                        is_add_obstalce = true;

                        break;
                    }else if(dir == -1 && is_via_ok){
                        Obstacle o;
                        o.dim = 0;
                        o.mean = env_via.topRows(2);
                        env_obs.push_back(o);
                        block_env_obs_idxs.push_back(env_obs.size()-1);

                        vp->is_left_env = true;
                        is_add_obstalce = true;
                        break;
                    }
                }
            }else{
                std::cout << "path and enviroment is too close !!" << std::endl;
            }
        }
    }

    return is_add_obstalce;
}

bool VertexManager::filter_dynamic_obs(bool &is_update_left, bool &is_update_right){
    bool is_add_obstalce = false;

    dynamic_obs = static_obs;
    block_dynamic_obs_idxs = block_static_obs_idxs;
    for(int i = 2; i < vertex_num; i++){
        for(int j = 0; j < vertex_task[i]->dynamic_obs.size(); j++){

            double obs_theta = atan2(vertex_task[i]->dynamic_obs[j].speed.y(), vertex_task[i]->dynamic_obs[j].speed.x());
            double angular_obs_now = EdgeUtils::normalize_theta(obs_theta - now_pose[2]);

            if(fabs(angular_obs_now) >= M_PI/2){
                if(angular_obs_now >= 0){
                    if(vp->is_via_right() || vp->is_via_goal()){
                        dynamic_obs.push_back(vertex_task[i]->dynamic_obs[j]);
                        block_dynamic_obs_idxs.push_back(dynamic_obs.size() - 1);
                        is_add_obstalce = true;
                        is_update_right = true;
                    }
                    vp->is_left_via = false;

                }else{
                    if(vp->is_via_left() || vp->is_via_goal()){
                        dynamic_obs.push_back(vertex_task[i]->dynamic_obs[j]);
                        block_dynamic_obs_idxs.push_back(dynamic_obs.size() - 1);
                        is_add_obstalce = true;
                        is_update_left = true;
                    }
                    vp->is_right_via = false;
                }
            }
        }
    }

    if(is_add_obstalce){
        if(is_update_left && is_update_right){
            is_add_obstalce = false;
        }
    }

    return is_add_obstalce;
}

bool VertexManager::decide_speed_max_rate(){
    bool is_update = false;

    for(int i = 2; i < vertex_num; i++){
        if(vertex_task[i]->is_obs){
            double max_rate = 1 - (1.0/(i-1));
            cfg->trajectory.max_rate_dynamic_obs = (max_rate < cfg->trajectory.max_rate_dynamic_obs) ? max_rate : cfg->trajectory.max_rate_dynamic_obs;
            
            is_update = true;
        }
    }

    return is_update;
}

#endif