#ifndef TEB_CONFIG_HPP
#define TEB_CONFIG_HPP

#include <mutex>
#include <Eigen/Core>
#include <Eigen/StdVector>

class TebConfig{
    public:

        //! Robot related parameters
        struct Robot{
            double max_vel_x;
            double max_vel_x_backwards;
            double max_vel_theta;
            double acc_lim_x;
            double acc_lim_theta;
            double body_width;
            double transform_toleranc;
        } robot;

        //! Goal tolerance related parameters
        struct GoalTolerance{
            double viapoint_xy_tolerance;
            double viapoint_yaw_tolerance;
        } goal_tolerance;

        struct Obstacles{
            double obs_cov_rate;
            double env_inflation_radius;
            double env_report_threshold;
            int obs_check_time;
        } obstacles;

        struct Trajectory{
            int predict_step;
            int iterate_num;
            int env_detect_step;
            double update_dt;
            double dt_ref;
            bool exact_arc_length;
            double exact_arc_length_theta;
            double robot2goal_dist;
            double max_rate_omega_min_limit;
            double max_rate_omega_min_vel;
            double max_rate_goal_min;
            double max_rate_dynamic_obs;
            double forward_drive_theta;

        } trajectory;

        //! Optimization related parameters
        struct Optimization{
            double penalty_epsilon;
            double weight_max_vel_x;
            double weight_max_vel_theta;
            double weight_acc_lim_x;
            double weight_acc_lim_theta;
            double weight_kinematics_nh;
            double weight_kinematics_forward_drive;
            double weight_shortest_path_dist;
            double weight_shortest_path_angular;
            double weight_obstacle;
            double weight_env;
            double weight_viapoint_dist;
            double weight_viapoint_angular;
                    
            double weight_prefer_rotdir;

        } optim;

        TebConfig(){
            // Robot
            robot.max_vel_x = 0.4;
            robot.max_vel_x_backwards = 0.3;
            robot.max_vel_theta = 0.2;
            robot.acc_lim_x = 0.2;
            robot.acc_lim_theta = 0.075;
            robot.body_width = 0.4;

            // GoalTolerance
            goal_tolerance.viapoint_xy_tolerance = 5e-2;
            goal_tolerance.viapoint_yaw_tolerance = M_PI/180;

            // Obstacles
            obstacles.obs_cov_rate = 1/6;
            obstacles.env_inflation_radius = 0.8;
            obstacles.env_report_threshold = 10;
            obstacles.obs_check_time = 5;

            // trajectory
            trajectory.predict_step = 30;
            trajectory.iterate_num = 300;
            trajectory.env_detect_step = 10;
            trajectory.update_dt = 0.1;
            trajectory.dt_ref = 0.3;
            trajectory.exact_arc_length = true;
            trajectory.exact_arc_length_theta = M_PI/45;
            trajectory.robot2goal_dist =1.7;
            trajectory.max_rate_omega_min_limit = 0.1;
            trajectory.max_rate_omega_min_vel = 0.1;
            trajectory.max_rate_goal_min = 0.05;
            trajectory.max_rate_dynamic_obs = 1.0;
            trajectory.forward_drive_theta = 0.707;
            
            // Optimization
            optim.penalty_epsilon = 0.1;
            
            optim.weight_max_vel_x = 10;
            optim.weight_max_vel_theta = 10;
            optim.weight_acc_lim_x = 10;
            optim.weight_acc_lim_theta = 10;
            optim.weight_kinematics_nh = 500;
            optim.weight_kinematics_forward_drive = 1;
            optim.weight_shortest_path_dist = 0.5;
            optim.weight_shortest_path_angular = 0.25;
            optim.weight_obstacle = 100;
            optim.weight_env = 100;
            optim.weight_viapoint_dist = 5;
            optim.weight_viapoint_angular = 1;
            optim.weight_prefer_rotdir = 50;
        }
        std::mutex& configMutex() {return config_mutex_;}

    private:
        std::mutex config_mutex_; //!< Mutex for config accesses and changes

};

#endif