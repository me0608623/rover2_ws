#ifndef TIMED_ELASTIC_BAND_HPP
#define TIMED_ELASTIC_BAND_HPP

#include "utils/fgo/factor_graph_utils.hpp"
#include "utils/fgo/factor_graph.hpp"
#include "utils/fgo/optimization_algorithm.hpp"
#include "utils/fgo/solver.hpp"
#include "utils/fgo/linear_solver.hpp"

#include "utils/edge_utils.hpp"
#include "utils/edge_acceleration.hpp"
#include "utils/edge_kinematics.hpp"
#include "utils/edge_obstacle.hpp"
#include "utils/edge_shortest_path.hpp"
#include "utils/edge_velocity.hpp"
#include "utils/edge_via_point.hpp"
#include "utils/edge_env.hpp"
#include "utils/vertex_pose.hpp"
#include "utils/vertex_manager.hpp"
#include "utils/teb_config.hpp"

#include "utils/timed_elastic_band_utils.hpp"

class TimedElasticBand{
    public:
        Eigen::MatrixXd global_path;
        std::vector<Vector3d> path;
        std::vector<Vector3d> opt_path;
        std::shared_ptr<VertexManager> vm;

        TimedElasticBand(std::shared_ptr<TebConfig> _cfg);
        void gen_initial_teb(Vector3d now_pose, Vector3d _goal_pose, int step, bool is_clear);
        void gen_hyper_graph();
        void optimize_hyper_graph(int iterate_num_1, UpdateType mode);
        
        bool check_arrive(Vector3d now_pose, Vector3d goal_pose);
        void get_cmd(double &lv, double &av);
        void set_obs(std::vector<Obstacle> _obs);
        void set_env(std::shared_ptr<CostMap> _env);
        void set_path(MatrixXd &path);

    private:
        bool is_first = true;
        double dt;
        double path_length;
        Vector3d goal_pose;
        Vector3d last_pose;
        std::shared_ptr<FactorGraph> graph;
        std::shared_ptr<Solver> s;
        // std::shared_ptr<MultiObjectiveOptimization> opt;
        std::shared_ptr<TebConfig> cfg;
        std::shared_ptr<CostMap> env;
        std::vector<Obstacle> obs;

        void gen_vertex();
        void gen_edge_nh();
        void gen_edge_vel();
        void gen_edge_acc();
        void gen_edge_obs();
        void gen_edge_env();
        void gen_edge_via_pt();
        void gen_edge_shortest_path();
};

TimedElasticBand::TimedElasticBand(std::shared_ptr<TebConfig> _cfg){
    obs.clear();
    cfg = _cfg;
    dt = cfg->trajectory.dt_ref;

    graph = std::make_shared<FactorGraph>();
    std::shared_ptr<LinearSolverBridge> ls = std::make_shared<LinearSolverBridge>(LinearSolverBridgeType::NORMAL);
    s = std::make_shared<Solver>(ls);
    // opt = std::make_shared<MultiObjectiveOptimization>(graph, s);

    // vm = std::make_shared<VertexManager>(cfg->trajectory.predict_step+2, cfg);
}

bool TimedElasticBand::check_arrive(Vector3d now_pose, Vector3d goal_pose){
    bool is_xy_arrive = ((goal_pose.topRows(2) - now_pose.topRows(2)).norm() <= cfg->goal_tolerance.viapoint_xy_tolerance) ? true : false;
    bool is_yaw_arrive = (fabs(EdgeUtils::normalize_theta(goal_pose[2] - now_pose[2])) <= cfg->goal_tolerance.viapoint_yaw_tolerance) ? true : false;
    
    return is_xy_arrive && is_yaw_arrive;
}

void TimedElasticBand::get_cmd(double &lv, double &av){
    static double last_lv = 0, last_av = 0;
    Vector2d deltaS = opt_path[2].topRows(2) - opt_path[1].topRows(2);
    double dist = deltaS.norm();
    double angle_diff = EdgeUtils::normalize_theta((opt_path[2][2] - opt_path[1][2]));

    Vector2d angle_vec ( cos(opt_path[2][2]), sin(opt_path[2][2]) );
    double dir = (deltaS.dot(angle_vec) >= 0) ? 1 : -1;

    if (cfg->trajectory.exact_arc_length && fabs(angle_diff) >= cfg->trajectory.exact_arc_length_theta){
        // actual arg length!
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius );
    }

    lv = dir * dist/dt;
    av = angle_diff/dt;

    if(EdgeUtils::penaltyBoundToInterval(lv, -cfg->robot.max_vel_x_backwards, cfg->robot.max_vel_x, 0) > 0){
    	lv = (lv >= 0) ? (last_lv + cfg->robot.max_vel_x)/2 : (last_lv + -cfg->robot.max_vel_x_backwards)/2;
    }

    //lv = (EdgeUtils::penaltyBoundToInterval(lv, -cfg->robot.max_vel_x_backwards, cfg->robot.max_vel_x, 0) > 0) ? 0 : lv;
    av = (EdgeUtils::penaltyBoundToInterval(av, cfg->robot.max_vel_theta, 0) > 0) ? (last_av + cfg->robot.max_vel_theta)/2 : av;
    
    last_lv = lv;
    last_av = av;
}

void TimedElasticBand::set_obs(std::vector<Obstacle> _obs){
    obs = _obs;
}

void TimedElasticBand::set_env(std::shared_ptr<CostMap> _env){
    env = _env;
}

void TimedElasticBand::set_path(MatrixXd &path){
    global_path = path;
}

void TimedElasticBand::gen_initial_teb(Vector3d now_pose, Vector3d _goal_pose, int step, bool is_clear){
    goal_pose = _goal_pose;
    Vector3d pose = now_pose;
    Vector2d delta = goal_pose.topRows(2) - pose.topRows(2);
    double theta = atan2(delta[1], delta[0]);

    path_length = delta.norm();
    double init_resolution = delta.norm() / step;

    Vector2d ds ;

    ds << init_resolution*cos(theta), init_resolution*sin(theta);

    if(is_clear){
        path.clear();
    }

    path.push_back(pose);
    pose[2] = _goal_pose[2];
    for(int i = 0; i<step; i+=1){
        pose.topRows(2) = pose.topRows(2) + ds;
        pose[2] = (i == step-1) ? _goal_pose[2] : theta;
        path.push_back(pose);
    }

    if(pose != goal_pose){
        path.push_back(goal_pose);
    }
}

void TimedElasticBand::gen_vertex(){
    if(is_first){
        graph->add_vertex(std::make_shared<VertexPose>(path[0], false));
        graph->vertexs_container[0]->time_stamp = -cfg->trajectory.update_dt;

    }else{
        graph->add_vertex(std::make_shared<VertexPose>(last_pose, false));
        graph->vertexs_container[0]->time_stamp = -cfg->trajectory.update_dt;

    }
    graph->add_vertex(std::make_shared<VertexPose>(path[0], false));
    graph->vertexs_container[1]->time_stamp = 0;

    for(int i = 1; i < path.size(); i++){
        graph->add_vertex(std::make_shared<VertexPose>(path[i], i!=path.size()-1));
        // graph->add_vertex(std::make_shared<VertexPose>(path[i], true));
        graph->vertexs_container[i+1]->time_stamp = dt * i;
    }
}

void TimedElasticBand::gen_edge_nh(){
    for(int i = 2; i < graph->vertexs_container.size(); i++){
        std::shared_ptr<EdgeKinematicsDiffDrive> edge = std::make_shared<EdgeKinematicsDiffDrive>(cfg);
        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_kinematics_nh;
        edge->weight(1, 1) = cfg->optim.weight_kinematics_forward_drive;
        edge->set_vertex_manager(vm);

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i-1]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_edge_vel(){
    for(int i = 2; i < graph->vertexs_container.size(); i++){
        std::shared_ptr<EdgeVelocity> edge = std::make_shared<EdgeVelocity>(cfg);
        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_max_vel_x;
        edge->weight(1, 1) = cfg->optim.weight_max_vel_theta;
        edge->setViaPoint(goal_pose);

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i-1]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_edge_acc(){
    for(int i = 2; i < graph->vertexs_container.size()-1; i++){
        std::shared_ptr<EdgeAcceleration> edge = std::make_shared<EdgeAcceleration>(cfg);
        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_acc_lim_x;
        edge->weight(1, 1) = cfg->optim.weight_acc_lim_theta;

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i-2]);
        edge->add_vertex(graph->vertexs_container[i-1]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_edge_via_pt(){
    for(int i = 1; i < graph->vertexs_container.size(); i++){
        std::shared_ptr<EdgeViaPoint> edge = std::make_shared<EdgeViaPoint>(cfg);
        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_viapoint_dist;
        edge->weight(1, 1) = cfg->optim.weight_viapoint_angular;
        // edge->setViaPoint(goal_pose);
        edge->set_vertex_manager(vm);

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i-1]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_edge_obs(){
    for(int i = 2; i < graph->vertexs_container.size(); i++){
        std::shared_ptr<EdgeObstacle> edge = std::make_shared<EdgeObstacle>(cfg);
        edge->is_compute = i == 2;

        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_obstacle;
        edge->setObs(obs);
        edge->set_vertex_manager(vm);

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_edge_env(){
    for(int i = 2; i < cfg->trajectory.env_detect_step; i++){
        std::shared_ptr<EdgeEnv> edge = std::make_shared<EdgeEnv>(cfg);
        edge->is_compute = i == 2;

        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_env;
        edge->setEnv(env);
        edge->set_vertex_manager(vm);

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_edge_shortest_path(){
    for(int i = 2; i < graph->vertexs_container.size(); i++){
        std::shared_ptr<EdgeShortestPath> edge = std::make_shared<EdgeShortestPath>(cfg);
        graph->vertexs_container[i]->add_edge(edge);

        edge->weight(0, 0) = cfg->optim.weight_shortest_path_dist;
        edge->weight(1, 1) = cfg->optim.weight_shortest_path_angular;

        edge->add_obj_vertex(graph->vertexs_container[i]);
        edge->add_vertex(graph->vertexs_container[i-1]);
        edge->add_vertex(graph->vertexs_container[i]);
        graph->add_edge(edge);
    }
}

void TimedElasticBand::gen_hyper_graph(){
    graph->initial();

    gen_vertex();
    vm = std::make_shared<VertexManager>(graph->vertexs_container.size(), cfg);
    vm->setEnv(env);
    vm->setObs(obs);
    vm->setPath(global_path);
    vm->setNowPose(graph->vertexs_container[1]->x);
    vm->setGoalPose(goal_pose);
    vm->initial_status();

    gen_edge_nh();
    gen_edge_vel();
    gen_edge_acc();

    gen_edge_obs();
    gen_edge_env();

    gen_edge_via_pt();
    gen_edge_shortest_path();

    // graph->show_factor_graph(1);
}

void TimedElasticBand::optimize_hyper_graph(int iterate_num_1, UpdateType mode){
    MultiObjectiveOptimization opt(graph, s);

    // s->set_L_M_tao(1e-5);
    // std::cout << "optimize" << std::endl;
    clock_t start, end;

    start = clock();
    vm->compute_static_via_pt();
    opt.optimize(iterate_num_1, -1, -1, SolverType::LEVENBERG_MARQUARDT, mode, false);
    for(int i = 0; i< cfg->obstacles.obs_check_time; i++){
        opt.optimize(iterate_num_1, -1, -1, SolverType::LEVENBERG_MARQUARDT, mode, false);
        
        if(!vm->update_goal()){
            break;
        }
    }
    end = clock();
    
    if(((double) (end - start)) / CLOCKS_PER_SEC > cfg->trajectory.update_dt){
        std::cout << "total compute time : " << ((double) (end - start)) / CLOCKS_PER_SEC << std::endl;
    }

    opt_path.clear();
    for(int i = 0; i < graph->vertexs_container.size(); i++){
        // std::cout << "vertex " << i << " mse : " <<  graph->vertexs_container[i]->last_mse << std::endl;
        opt_path.push_back(graph->vertexs_container[i]->x);
    }

    is_first = false;
    last_pose = opt_path[1];
}
 
#endif
