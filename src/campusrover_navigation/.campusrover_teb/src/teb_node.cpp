#include "teb_node.hpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "teb_node");
    ros::NodeHandle nh;
    ros::NodeHandle n_private("~");


    ros::Time::init();

    tc = std::make_shared<TebConfig>();

    get_param(n_private);

    teb = new TimedElasticBand(tc);

    pub = nh.advertise<geometry_msgs::Twist>("teb_cmd", 10);
    pub1 = nh.advertise<geometry_msgs::PoseArray>("teb_planner", 10);
    pub2 = nh.advertise<geometry_msgs::PointStamped>("teb_planner_goal", 10);
    pub3 = nh.advertise<geometry_msgs::PoseArray>("teb_planner_init", 10);
    
    // cli = nh.serviceClient<campusrover_msgs::ImgLabel>("img_lable_srv");

    ros::Subscriber sub0 = nh.subscribe("/global_path", 1, global_path_callback);
    ros::Subscriber sub1 = nh.subscribe("/tracked_label_obstacle", 1, obstacle_callback);
    ros::Subscriber sub2 = nh.subscribe("/global_costmap", 10, costmap_callback);
    ros::Subscriber sub3 = nh.subscribe("/enabel_teb", 1, enabel_teb_callback);
    // ros::Subscriber sub2 = nh.subscribe("/global_costmap", 10, costmap_callback);

    ros::Timer trackers_update = nh.createTimer(ros::Duration(tc->trajectory.update_dt), teb_update_callback);

    ros::MultiThreadedSpinner spinner(thread_num);
    spinner.spin();

    return 0;
}