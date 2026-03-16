#ifndef TEB_NODE_HPP
#define TEB_NODE_HPP

#include "timed_elastic_band.hpp"
// #include "timed_elastic_band_v2.hpp"

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <campusrover_msgs/ImgLabel.h>
#include <campusrover_msgs/TrackedObstacle.h>
#include <campusrover_msgs/TrackedObstacleArray.h>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

TimedElasticBand *teb;
std::shared_ptr<TebConfig> tc;
boost::mutex teb_mutex;


Eigen::MatrixXd global_path;
boost::mutex global_path_mutex;

std::vector<Obstacle> obstacles;
campusrover_msgs::TrackedObstacleArray obs;
boost::mutex obs_mutex;

std::shared_ptr<CostMap> env_cost_map;
boost::mutex env_cost_map_mutex;

bool enable_teb = true, is_arrive = false;

// ros param
int thread_num;
string map_frame, base_frame;

ros::Publisher pub, pub1, pub2, pub3;

void get_param(ros::NodeHandle n_private){

    n_private.param<double>("robot_max_vel_x",              tc->robot.max_vel_x,                0.4);
    n_private.param<double>("robot_max_vel_x_backwards",    tc->robot.max_vel_x_backwards,      0.3);
    n_private.param<double>("robot_max_vel_theta",          tc->robot.max_vel_theta,            0.2);
    n_private.param<double>("robot_acc_lim_x",              tc->robot.acc_lim_x,                0.2);
    n_private.param<double>("robot_acc_lim_theta",          tc->robot.acc_lim_theta,            0.075);
    n_private.param<double>("robot_body_width",             tc->robot.body_width,               0.4);

    n_private.param<double>("goal_tolerance_viapoint_xy_tolerance",     tc->goal_tolerance.viapoint_xy_tolerance,         5e-2);
    n_private.param<double>("goal_tolerance_viapoint_yaw_tolerance",    tc->goal_tolerance.viapoint_yaw_tolerance,        180);
    tc->goal_tolerance.viapoint_yaw_tolerance = M_PI / tc->goal_tolerance.viapoint_yaw_tolerance;

    n_private.param<double>("obstacles_obs_cov_rate",                   tc->obstacles.obs_cov_rate,                 6);
    n_private.param<double>("obstacles_env_inflation_radius",           tc->obstacles.env_inflation_radius,         0.8);
    n_private.param<double>("obstacles_env_report_threshold",           tc->obstacles.env_report_threshold,         10);
    n_private.param<int>("obstacles_obs_check_time",                    tc->obstacles.obs_check_time,               5);
    // tc->obstacles.obs_cov_rate = 1 / tc->obstacles.obs_cov_rate;

    n_private.param<int>("trajectory_predict_step",                 tc->trajectory.predict_step,                 30);
    n_private.param<int>("trajectory_iterate_num",                  tc->trajectory.iterate_num,                  300);
    n_private.param<int>("trajectory_env_detect_step",              tc->trajectory.env_detect_step,              10);
    n_private.param<double>("trajectory_update_dt",                 tc->trajectory.update_dt,                    0.1);
    n_private.param<double>("trajectory_dt_ref",                    tc->trajectory.dt_ref,                       0.3);
    n_private.param<bool>("trajectory_exact_arc_length",            tc->trajectory.exact_arc_length,             true);
    n_private.param<double>("trajectory_exact_arc_length_theta",    tc->trajectory.exact_arc_length_theta,       45);
    n_private.param<double>("trajectory_robot2goal_dist",           tc->trajectory.robot2goal_dist,              1.7);
    n_private.param<double>("trajectory_max_rate_omega_min_limit",  tc->trajectory.max_rate_omega_min_limit,     0.1);
    n_private.param<double>("trajectory_max_rate_omega_min_vel",    tc->trajectory.max_rate_omega_min_vel,       0.1);
    n_private.param<double>("trajectory_max_rate_goal_min",         tc->trajectory.max_rate_goal_min,            0.05);
    n_private.param<double>("trajectory_forward_drive_theta",       tc->trajectory.forward_drive_theta,          0.707);
    tc->trajectory.exact_arc_length_theta = M_PI/tc->trajectory.exact_arc_length_theta;
    if(tc->trajectory.env_detect_step > tc->trajectory.predict_step){
        tc->trajectory.env_detect_step = tc->trajectory.predict_step;
        ROS_INFO("trajectory_env_detect_step is bigger than trajectory_predict_step, so adjust to lower value");
    }

    n_private.param<double>("optim_penalty_epsilon",                    tc->optim.penalty_epsilon,                  0.1);
    n_private.param<double>("optim_weight_max_vel_x",                   tc->optim.weight_max_vel_x,                 10);
    n_private.param<double>("optim_weight_max_vel_theta",               tc->optim.weight_max_vel_theta,             10);
    n_private.param<double>("optim_weight_acc_lim_x",                   tc->optim.weight_acc_lim_x,                 10);
    n_private.param<double>("optim_weight_acc_lim_theta",               tc->optim.weight_acc_lim_theta,             10);
    n_private.param<double>("optim_weight_kinematics_nh",               tc->optim.weight_kinematics_nh,             500);
    n_private.param<double>("optim_weight_kinematics_forward_drive",    tc->optim.weight_kinematics_forward_drive,  1.0);
    n_private.param<double>("optim_weight_shortest_path_dist",          tc->optim.weight_shortest_path_dist,        0.5);
    n_private.param<double>("optim_weight_shortest_path_angular",       tc->optim.weight_shortest_path_angular,     0.25);
    n_private.param<double>("optim_weight_obstacle",                    tc->optim.weight_obstacle,                  100);
    n_private.param<double>("optim_weight_env",                         tc->optim.weight_env,                       100);
    n_private.param<double>("optim_weight_viapoint_dist",               tc->optim.weight_viapoint_dist,             5);
    n_private.param<double>("optim_weight_viapoint_angular",            tc->optim.weight_viapoint_angular,          1);

    n_private.param<int>("thread_num",              thread_num,             2);
    n_private.param<string>("map_frame",           map_frame,               "map");
    n_private.param<string>("base_frame",           base_frame,             "base_link");
}

double get_yaw(geometry_msgs::Pose pose){
    tf::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

void global_path_callback(const nav_msgs::Path::ConstPtr& msg){
    ROS_INFO("get global path");
    is_arrive = false;
    int size = msg->poses.size();

    global_path_mutex.lock();
        global_path = Eigen::MatrixXd::Zero(size, 3);

        for(int i = 0; i < size; i++){
            global_path(i, 0) = msg->poses[i].pose.position.x;
            global_path(i, 1) = msg->poses[i].pose.position.y;
            global_path(i, 2) = get_yaw(msg->poses[i].pose);
        }
    global_path_mutex.unlock();
}

void obstacle_callback(const campusrover_msgs::TrackedObstacleArray::ConstPtr& msg){
    obs_mutex.lock();
    obstacles.clear();
    int obs_size = msg->obstacles.size();
    for(int i = 0; i < obs_size; i++){
        Obstacle o;
        o.mean << msg->obstacles[i].pose.position.x,
                msg->obstacles[i].pose.position.y;
        
        o.sigma << msg->obstacles[i].dimensions.x, 0,
                0, msg->obstacles[i].dimensions.x;

        o.speed << msg->obstacles[i].velocity.linear.x,
                msg->obstacles[i].velocity.linear.y;

        o.dim = msg->obstacles[i].dimensions.x;

        o.is_dynamic = msg->obstacles[i].is_dynamic;

        obstacles.push_back(o);
    }
    obs_mutex.unlock();
}

void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    env_cost_map_mutex.lock();
    env_cost_map = std::make_shared<CostMap>(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);

    int size = env_cost_map->width * env_cost_map->height;
    if(size == msg->data.size()){
        for(int i = 0; i < size; i++){
            if((int)msg->data[i] > 0){
                env_cost_map->data[i] = (int)msg->data[i];
                // std::cout << (int)msg->data[i] << ", " << env_cost_map->data[i] << std::endl;

            }else{
                env_cost_map->data[i] = 0;
            }
        }
    }else{
        ROS_INFO("CostMap info error");
    }
    env_cost_map_mutex.unlock();
}

void enabel_teb_callback(const std_msgs::Bool::ConstPtr& msg){
    enable_teb = msg->data;
}

void teb_update_callback(const ros::TimerEvent& event){
    bool is_get_tf = true;
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::Pose now_pose;
    geometry_msgs::TransformStamped tf_stamped;
    geometry_msgs::Twist cmd;
    Eigen::Matrix<double, 3, 1> now_pose_, goal_pose;

    cmd.linear.x = 0;
    cmd.angular.z = 0;

    try{
        tf_stamped = tf_buffer.lookupTransform(map_frame, base_frame, ros::Time::now(), ros::Duration(0.1));
        // tf2::doTransform(_now_pose, now_pose, tf_stamped);
        // tf_buffer.transform<geometry_msgs::PoseStamped>(_now_pose, now_pose, map_frame);
        now_pose.position.x = tf_stamped.transform.translation.x;
        now_pose.position.y = tf_stamped.transform.translation.y;
        now_pose.position.z = tf_stamped.transform.translation.z;
        now_pose.orientation.x = tf_stamped.transform.rotation.x;
        now_pose.orientation.y = tf_stamped.transform.rotation.y;
        now_pose.orientation.z = tf_stamped.transform.rotation.z;
        now_pose.orientation.w = tf_stamped.transform.rotation.w;

        now_pose_ << now_pose.position.x,
                    now_pose.position.y,
                    get_yaw(now_pose);

    }catch(tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
        is_get_tf = false;
    }

    global_path_mutex.lock();
    bool is_find_goal = (is_get_tf) ? TEBUtils::get_goal_pt(now_pose_, goal_pose, global_path, tc->trajectory.robot2goal_dist) : false;
    teb->set_path(global_path);
    global_path_mutex.unlock();

    if(is_find_goal){

        // goal_pose[0] = obstacles[0].mean.x(),
        // goal_pose[1] = obstacles[0].mean.y(),
        // goal_pose[2] = 0;

        if(teb->check_arrive(now_pose_, goal_pose)){
            if(!is_arrive){
                ROS_INFO("Arrive Goal !!");
                is_arrive = true;
            }
        }else{
            is_arrive = false;

            teb->gen_initial_teb(now_pose_, goal_pose, tc->trajectory.predict_step, true);
            teb->set_obs(obstacles);
            teb->set_env(env_cost_map);
            teb->gen_hyper_graph();

            teb_mutex.lock();
            teb->optimize_hyper_graph(tc->trajectory.iterate_num, UpdateType::SEQUENCE);

            if(enable_teb){
                teb->get_cmd(cmd.linear.x, cmd.angular.z);
            }

            teb_mutex.unlock();

            tf2::Quaternion myQuaternion;
            geometry_msgs::PointStamped pt;
            geometry_msgs::PoseArray pa;
            geometry_msgs::PoseArray pa_i;

            pt.header.frame_id = map_frame;
            pt.header.stamp = ros::Time::now();
            // pt.point = goal_pose.position;
            pt.point.x = teb->vm->vertex_task[0]->via_pose[0];
            pt.point.y = teb->vm->vertex_task[0]->via_pose[1];

            pa.header.frame_id = map_frame;
            pa.header.stamp = ros::Time::now();

            pa_i.header = pa.header;

            for(int i = 0; i < teb->opt_path.size(); i++){
                geometry_msgs::Pose p;
                myQuaternion.setRPY(0, 0, teb->opt_path[i][2]);

                p.position.x = teb->opt_path[i][0];
                p.position.y = teb->opt_path[i][1];
                p.position.z = 0;
                p.orientation.w = myQuaternion.getW();
                p.orientation.x = myQuaternion.getX();
                p.orientation.y = myQuaternion.getY();
                p.orientation.z = myQuaternion.getZ();
                
                pa.poses.push_back(p);

                myQuaternion.setRPY(0, 0, teb->path[i][2]);

                p.position.x = teb->path[i][0];
                p.position.y = teb->path[i][1];
                p.position.z = 0;
                p.orientation.w = myQuaternion.getW();
                p.orientation.x = myQuaternion.getX();
                p.orientation.y = myQuaternion.getY();
                p.orientation.z = myQuaternion.getZ();
                
                pa_i.poses.push_back(p);
            }

            pub1.publish(pa);
            pub2.publish(pt);
            pub3.publish(pa_i);
        }
    }
    
    pub.publish(cmd);
}

#endif