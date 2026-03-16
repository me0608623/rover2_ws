#ifndef MOT_UTILS_HPP
#define MOT_UTILS_HPP

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ExtendedObjectTrackingParam{
    double detection_area[6];
    double track_dead_time;
    double track_older_age;
    double cluster_dist;
    int false_alarm[2];
    int split_false_alarm[2];
    double weight_min_tolerate;
    double cov_scale;
    double inherit_ratio;

    ExtendedObjectTrackingParam();
    void set_detection_area(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
    void set_false_alarm(double min, double max);
};

namespace DataTransform{
    inline void pcl2matrix(pcl::PointCloud<pcl::PointXYZ> &pcl_in, Eigen::MatrixXd &matrix_out);
    inline void matrix2pcl(Eigen::MatrixXd &matrix_in, pcl::PointCloud<pcl::PointXYZ> &pcl_out, bool is_project_z);
};

namespace BoxFilter{
    inline void eigen_box_filter(Eigen::MatrixXd &matrix_in, Eigen::MatrixXd &matrix_out, double filter_area[6]);
    inline void pcl_box_filter(pcl::PointCloud<pcl::PointXYZ> &pcl_in, pcl::PointCloud<pcl::PointXYZ> &pcl_out, double filter_area[6]);
}

ExtendedObjectTrackingParam::ExtendedObjectTrackingParam(){
    detection_area[0] = 0;
    detection_area[1] = 20;
    detection_area[2] = -20;
    detection_area[3] = 20;
    detection_area[4] = -1;
    detection_area[5] = 3;

    track_dead_time = 2;
    track_older_age = 2;
    cluster_dist = 1;

    false_alarm[0] = 10;
    false_alarm[1] = 1000;

    split_false_alarm[0] = 0;
    split_false_alarm[1] = 1000;

    weight_min_tolerate = 0.01;
    cov_scale = 6;
    inherit_ratio = 0.8;
}

void ExtendedObjectTrackingParam::set_detection_area(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max){
    detection_area[0] = x_min;
    detection_area[1] = x_max;
    detection_area[2] = y_min;
    detection_area[3] = y_max;
    detection_area[4] = z_min;
    detection_area[5] = z_max;
}

void ExtendedObjectTrackingParam::set_false_alarm(double min, double max){
    false_alarm[0] = min;
    false_alarm[1] = max;
    split_false_alarm[1] = max;
}

inline void DataTransform::pcl2matrix(pcl::PointCloud<pcl::PointXYZ> &pcl_in, Eigen::MatrixXd &matrix_out){
    int size = pcl_in.size();
    matrix_out = Eigen::MatrixXd(size, 3);

    for(int i = 0; i < size; i++){
        matrix_out(i, 0) = pcl_in[i].x;
        matrix_out(i, 1) = pcl_in[i].y;
        matrix_out(i, 2) = pcl_in[i].z;
    }
}

inline void DataTransform::matrix2pcl(Eigen::MatrixXd &matrix_in, pcl::PointCloud<pcl::PointXYZ> &pcl_out, bool is_project){
    int size = matrix_in.rows();
    pcl_out.clear();

    pcl::PointXYZ pt;
    for(int i = 0; i < size; i++){
        pt.x = matrix_in(i, 0);
        pt.y = matrix_in(i, 1);
        pt.z = (is_project) ? 0 : matrix_in(i, 2);

        pcl_out.push_back(pt);
    }
}

inline void BoxFilter::eigen_box_filter(Eigen::MatrixXd &matrix_in, Eigen::MatrixXd &matrix_out, double filter_area[6]){
    int size = matrix_in.rows();
    std::vector<int> filter_index;

    for(int i = 0; i < size; i++){
        if(matrix_in(i, 0) >= filter_area[0] && matrix_in(i, 0) <= filter_area[1] && matrix_in(i, 1) >= filter_area[2] && matrix_in(i, 1) <= filter_area[3] && matrix_in(i, 2) >= filter_area[4] && matrix_in(i, 2) <= filter_area[5]){
            filter_index.push_back(i);
        }
    }

    size = filter_index.size();
    matrix_out = Eigen::MatrixXd(filter_index.size(), 3);
    for(int i = 0; i < size; i++){
        matrix_out.row(i) = matrix_in.row(filter_index[i]);
    }
}

inline void BoxFilter::pcl_box_filter(pcl::PointCloud<pcl::PointXYZ> &pcl_in, pcl::PointCloud<pcl::PointXYZ> &pcl_out, double filter_area[6]){
    int size = pcl_in.size();
    std::vector<int> filter_index;

    for(int i = 0; i < size; i++){
        if(pcl_in[i].x >= filter_area[0] && pcl_in[i].x <= filter_area[1] && pcl_in[i].y >= filter_area[2] && pcl_in[i].y <= filter_area[3] && pcl_in[i].z >= filter_area[4] && pcl_in[i].z <= filter_area[5]){
            filter_index.push_back(i);
        }
    }

    size = filter_index.size();
    pcl_out.clear();
    for(int i = 0; i < size; i++){
        pcl_out.push_back(pcl_in[filter_index[i]]);
    }
}

#endif