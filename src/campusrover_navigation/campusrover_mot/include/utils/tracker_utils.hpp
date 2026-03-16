#ifndef TRACKER_UTILS_HPP
#define TRACKER_UTILS_HPP

#include <Eigen/Dense>

enum class ObjectStatus{
    STATIC,
    DYNAMIC
};

struct BoundaryPt{
    Eigen::MatrixXd pt_3X1;
    double theta;
};

struct TrackerParam{
    int history_length;
    double anchor_dist_threshold;
    double speed_threshold;

    TrackerParam();
};

TrackerParam::TrackerParam(){
    history_length = 10;
    anchor_dist_threshold = 0.1;
    speed_threshold = 0.1;
}

#endif