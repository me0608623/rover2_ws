#ifndef VERTEX_MANAGER_UTILS_HPP
#define VERTEX_MANAGER_UTILS_HPP

#include <vector>
#include <Eigen/Dense>
#include "edge_utils.hpp"
#include "timed_elastic_band_utils.hpp"

using namespace Eigen;

struct VertexTask{
    std::vector<Obstacle> dynamic_obs;
    Vector3d via_pose;
    Vector3d env_pose;
    bool is_env;
    bool is_obs;
};

class ViaPt{
    public:
        Vector3d now_pose;
        Vector3d goal_pose;
        Vector3d left_pose;
        Vector3d right_pose;

        bool is_left_use = false;
        bool is_right_use = false;
        bool is_left_env = false;
        bool is_right_env = false;
        bool is_left_via = false;
        bool is_right_via = false;

        ViaPt(){

        }

        ViaPt(Vector3d _now_pose, Vector3d _goal_pose){
            now_pose = _now_pose;
            goal_pose = _goal_pose;
        }

        void set_pose(Vector3d _now_pose, Vector3d _goal_pose){
            now_pose = _now_pose;
            goal_pose = _goal_pose;
        }

        void initial(){
            is_left_env = false;
            is_right_env = false;
            is_left_via = false;
            is_right_via = false;
        }

        void decide_side(){
            is_left_use = !is_left_env && is_left_via;
            is_right_use = !is_right_env && is_right_via;
        }

        Vector3d get_via_pose(){
            if(is_left_use && is_right_use){
                Vector2d ds_l = (left_pose.topRows(2) - now_pose.topRows(2));
                Vector2d ds_r = (right_pose.topRows(2) - now_pose.topRows(2));

                double theta_l = atan2(ds_l.y(), ds_l.x());
                double theta_r = atan2(ds_r.y(), ds_r.x());

                double d_theta_l = fabs(EdgeUtils::normalize_theta(theta_l - now_pose[2]));
                double d_theta_r = fabs(EdgeUtils::normalize_theta(theta_r - now_pose[2]));

                return (d_theta_l < d_theta_r) ? left_pose : right_pose;

            }else if(is_left_use || is_right_use){
                
                return (is_left_use) ? left_pose : right_pose;

            }else{
                return goal_pose;
            }
        }

        bool get_via_status(){
            return is_left_use || is_right_use;
        }

        bool is_via_left(){
            return left_pose == get_via_pose();
        }

        bool is_via_right(){
            return right_pose == get_via_pose();
        }

        bool is_via_goal(){
            return goal_pose == get_via_pose();
        }
};


#endif