#ifndef MOT_HPP
#define MOT_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include "utils/mot_utils.hpp"
#include "utils/mean_tracker.hpp"
#include "utils/mean_cov_tracker.hpp"
#include "utils/tracker_utils.hpp"

using namespace Eigen;

enum class SplitStatus{
    UNSPLIT,
    INHERIT,
    UNINHERIT
};

template<class Tracker>
class ExtendedObjectTracking{
    public:
        std::vector<Tracker> trackers;
        double now_time;
        ExtendedObjectTrackingParam param;
        TrackerParam tracker_param;

        int uid;

        MatrixXd detection_points;
        MatrixXd unmatch_points;
        MatrixXd weight_matrix;

        ExtendedObjectTracking(double _now_time, ExtendedObjectTrackingParam _param, TrackerParam _tracker_param);
        void update(MatrixXd input_data, MatrixXd now_pose, double _now_time, double dt, bool is_filter);

    private:
        bool false_alarm_filter(int size);
        void pcl_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in, std::vector<pcl::PointIndices> &clusters, int filter[2]);

        void data_association(double dt);
        SplitStatus split_track(MatrixXd &match_points, int tracks_idx);
        void add_new_track();
        std::vector<int> delete_track(MatrixXd now_pose);
        void merge_track(MatrixXd &old_tracks_points, std::vector<int> old_tracks_idx);
};

template<class Tracker>
bool ExtendedObjectTracking<Tracker>::false_alarm_filter(int size){
    return size > param.false_alarm[0] && size < param.false_alarm[1];
}

template<class Tracker>
void ExtendedObjectTracking<Tracker>::pcl_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in, std::vector<pcl::PointIndices> &clusters, int filter[2]){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;

    kdtree->setInputCloud(pts_in);
    
    clustering.setClusterTolerance(param.cluster_dist);
    clustering.setMinClusterSize(filter[0]);
    clustering.setMaxClusterSize(filter[1]);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(pts_in);

    clustering.extract(clusters);
}

template<class Tracker>
void ExtendedObjectTracking<Tracker>::data_association(double dt){
    int rows = detection_points.rows(), cols = trackers.size();
    double weight, coeff;
    Matrix<double, 2, 1> mean;
    Matrix<double, 2, 2> sigma;
    Matrix<double, 2, 1> speed;
    Matrix<double, 2, 1> p_m_delta;
    Matrix<double, 1, 1> result;

    weight_matrix = MatrixXd(rows, cols);
    for(int j = 0; j < cols; j++){

        trackers[j].predict(dt, mean, sigma, speed);

        sigma = sigma * param.cov_scale;
        coeff = 1 / sqrt(sigma.determinant());

        for(int i = 0; i < rows; i++){
            p_m_delta = detection_points.row(i).leftCols(2).transpose() - mean;

            result = -0.5 * (p_m_delta.transpose() * sigma.inverse() * p_m_delta);
            weight =  coeff * exp((float)result(0, 0));

            weight_matrix(i, j) = (weight >= param.weight_min_tolerate) ? weight : 0;
        }
    }
}

template<class Tracker>
SplitStatus ExtendedObjectTracking<Tracker>::split_track(MatrixXd &match_points, int tracks_idx){
    SplitStatus status = SplitStatus::UNSPLIT;

    if(false_alarm_filter(match_points.rows())){
        pcl::PointCloud<pcl::PointXYZ> pts;
        std::vector<pcl::PointIndices> clusters;

        DataTransform::matrix2pcl(match_points, pts, true);
        pcl_cluster(pts.makeShared(), clusters, param.split_false_alarm);

        if(clusters.size() > 1){
            int sub_points_num = 0, inherit_idx = -1;
            double inherit_points_num = match_points.rows()*param.inherit_ratio;
            MatrixXd sub_points;
            
            for(int i = 0; i < clusters.size(); i++){
                if(clusters[i].indices.size() > sub_points_num){
                    sub_points_num = clusters[i].indices.size();
                    inherit_idx = i;
                }
            }

            if(sub_points_num > inherit_points_num  && false_alarm_filter(sub_points_num)){
                status = SplitStatus::INHERIT;

            }else{
                status = SplitStatus::UNINHERIT;
                inherit_idx = -1;

            }

            for(int i = 0; i < clusters.size(); i++){
                if(false_alarm_filter(clusters[i].indices.size())){
                    int size = clusters[i].indices.size();
                    sub_points = MatrixXd(size, 3);

                    for(int j = 0; j < size; j++){
                        sub_points.row(j) = match_points.row(clusters[i].indices[j]);
                    }
                    
                    if(i != inherit_idx){
                        // std::cout << "add track id : " << uid << " by split from : "<< trackers[tracks_idx].uid << std::endl;
                        trackers.push_back(Tracker(sub_points, now_time, uid, tracker_param));
                        uid++;

                    }else{
                        trackers[tracks_idx].points = sub_points;
                    }
                }
            }
        }
    }

    return status;
}

template<class Tracker>
void ExtendedObjectTracking<Tracker>::add_new_track(){

    if(unmatch_points.rows() >= param.false_alarm[0]){
        pcl::PointCloud<pcl::PointXYZ> pts;
        std::vector<pcl::PointIndices> clusters;
        MatrixXd sub_points;

        DataTransform::matrix2pcl(unmatch_points, pts, true);
        pcl_cluster(pts.makeShared(), clusters, param.false_alarm);

        for(int i = 0; i < clusters.size(); i++){
            int size = clusters[i].indices.size();
            sub_points = MatrixXd(size, 3);

            for(int j = 0; j < size; j++){
                sub_points.row(j) = unmatch_points.row(clusters[i].indices[j]);

            }

            // // std::cout << "add track id : " << uid << " by new" << std::endl;
            trackers.push_back(Tracker(sub_points, now_time, uid, tracker_param));
            uid++;
        }
    }
}

template<class Tracker>
std::vector<int> ExtendedObjectTracking<Tracker>::delete_track(MatrixXd now_pose){
    std::vector<int> old_tracks_idx;

    for(int i = 0; i < trackers.size(); ){
        if((now_time - trackers[i].update_time) >=param.track_dead_time){
            // std::cout << "delete track id : " << trackers[i].uid << " by out of time" << std::endl;
            trackers.erase(trackers.begin()+i);

        }else if(trackers[i].mean_2X1(0, 0) - now_pose(0, 0) < param.detection_area[0] || trackers[i].mean_2X1(0, 0) - now_pose(0, 0) > param.detection_area[1] || trackers[i].mean_2X1(1, 0) - now_pose(1, 0) < param.detection_area[2] || trackers[i].mean_2X1(1, 0) - now_pose(1, 0) > param.detection_area[3]){
            // std::cout << "delete track id : " << trackers[i].uid << " by out of range" << std::endl;
            trackers.erase(trackers.begin()+i);

        }else if(trackers[i].is_kill){
            // std::cout << "delete track id : " << trackers[i].uid << " by is_kill" << std::endl;
            trackers.erase(trackers.begin()+i);

        }else{
            if((now_time - trackers[i].born_time) >= param.track_older_age && trackers[i].status == ObjectStatus::STATIC){
                old_tracks_idx.push_back(i);
            }
            i++;
        }
    }

    return old_tracks_idx;
}

template<class Tracker>
void ExtendedObjectTracking<Tracker>::merge_track(MatrixXd &old_tracks_points, std::vector<int> old_tracks_idx){
    pcl::PointCloud<pcl::PointXYZ> pts;
    std::vector<pcl::PointIndices> clusters;
    MatrixXd sub_points;

    DataTransform::matrix2pcl(old_tracks_points, pts, true);
    pcl_cluster(pts.makeShared(), clusters, param.false_alarm);

    for(int i = 0; i < clusters.size(); i++){
        int size = clusters[i].indices.size();
        sub_points = MatrixXd(size, 3);

        for(int j = 0; j < size; j++){
            sub_points.row(j) = old_tracks_points.row(clusters[i].indices[j]);

        }

        MatrixXd mean_2X1 = sub_points.leftCols(2).colwise().mean().transpose();    

        int min_idx = 0;
        double dist = pow(trackers[old_tracks_idx[0]].mean_2X1(0, 0)-mean_2X1(0, 0), 2) + pow(trackers[old_tracks_idx[0]].mean_2X1(1, 0)-mean_2X1(1, 0), 2);
        double min_dist = dist;

        for(int j = 1; j < old_tracks_idx.size(); j++){
            dist = pow(trackers[old_tracks_idx[j]].mean_2X1(0, 0)-mean_2X1(0, 0), 2) + pow(trackers[old_tracks_idx[j]].mean_2X1(1, 0)-mean_2X1(1, 0), 2);
            
            if(dist < min_dist){
                min_dist = dist;
                min_idx = j;
            }
        }

        trackers[old_tracks_idx[min_idx]].points = sub_points;
        trackers[old_tracks_idx[min_idx]].merge_update();
        old_tracks_idx.erase(old_tracks_idx.begin() + min_idx);
    }

    for(int i = old_tracks_idx.size()-1; i >= 0; i--){
        // std::cout << "delete track id : " << trackers[old_tracks_idx[i]].uid << " by merge" << std::endl;
        trackers.erase(trackers.begin() + old_tracks_idx[i]);
    }
}

template<class Tracker>
ExtendedObjectTracking<Tracker>::ExtendedObjectTracking(double _now_time, ExtendedObjectTrackingParam _param, TrackerParam _tracker_param){
    trackers.clear();
    now_time = _now_time;
    param = _param;
    tracker_param = _tracker_param;
    uid = 0;
}

template<class Tracker>
void ExtendedObjectTracking<Tracker>::update(MatrixXd input_data, MatrixXd now_pose, double _now_time, double dt, bool is_filter){
    int size;
    now_time = _now_time;
    std::vector<int> filter_index;
    clock_t start, end;
    // clock_t start_, end_;

    // std::cout << "input data : " << input_data.rows() << std::endl;

    start = clock();
    // std::cout << "filter input data ..." << std::endl;
    // start_ = clock();
    if(is_filter){
        BoxFilter::eigen_box_filter(input_data, detection_points, param.detection_area);

    }else{
        detection_points = input_data;
    }

    // end_ = clock();
    // std::cout << "filter input time : " << ((double) (end_ - start_)) / CLOCKS_PER_SEC << std::endl;
    // std::cout << "detection points : "  << detection_points.rows() << std::endl;

    // std::cout << "data association ..." << std::endl;
    // start_ = clock();
    data_association(dt);
    // end_ = clock();
    // std::cout << "data association time : " << ((double) (end_ - start_)) / CLOCKS_PER_SEC << std::endl;

    // std::cout << "update track ..." << std::endl;
    // start_ = clock();
    int match_size = 0;
    if(weight_matrix.cols() > 0){
        int idx, tracks_num = trackers.size();

        for(int i = 0; i < tracks_num; i++){
            filter_index.clear();

            size = detection_points.rows();
            for(int j = 0; j < size; j++){
                if(weight_matrix.row(j).maxCoeff(&idx) > 0){
                    if(idx == i){
                        filter_index.push_back(j);
                    }
                }
            }

            match_size += filter_index.size();
            size = filter_index.size();
            MatrixXd match_points(size, 3);
            for(int j = 0; j < size; j++){
                match_points.row(j) = detection_points.row(filter_index[j]);
            }

            SplitStatus status = split_track(match_points, i);

            switch(status){
                case SplitStatus::INHERIT : {
                    trackers[i].update_time = now_time;
                    trackers[i].update(dt, true);

                    break;
                }

                case SplitStatus::UNINHERIT : {
                    trackers[i].is_kill = true;

                    break;
                }

                case SplitStatus::UNSPLIT : {
                    if(false_alarm_filter(filter_index.size())){
                        trackers[i].points = match_points;
                        trackers[i].update_time = now_time;
                        trackers[i].update(dt, true);

                    }else{
                        trackers[i].update(dt, false);
                    }

                    break;
                }
            }
        }
    }
    // end_ = clock();
    // std::cout << "update track time : " << ((double) (end_ - start_)) / CLOCKS_PER_SEC << std::endl;
    // std::cout << "total match points : " << match_size << std::endl;

    // std::cout << "add new track ..." << std::endl;
    // start_ = clock();
    if(weight_matrix.cols() > 0){
        filter_index.clear();

        size = detection_points.rows();
        for(int i = 0; i < size; i++){
            if(weight_matrix.row(i).maxCoeff() == 0){
                filter_index.push_back(i);
            }
        }

        // std::cout << "unmatch_points : " << filter_index.size() << std::endl;
        size = filter_index.size();
        unmatch_points = MatrixXd(size, 3);
        for(int i = 0; i < size; i++){
            unmatch_points.row(i) = detection_points.row(filter_index[i]);
        }
    }else{
        unmatch_points = MatrixXd(detection_points.rows(), 3);
        unmatch_points = detection_points;
    }

    add_new_track();
    // end_ = clock();
    // std::cout << "add new track time : " << ((double) (end_ - start_)) / CLOCKS_PER_SEC << std::endl;

    // std::cout << "delete track ..." << std::endl;
    // start_ = clock();
    std::vector<int> old_tracks_idx = delete_track(now_pose);
    // end_ = clock();
    // std::cout << "delete track time : " << ((double) (end_ - start_)) / CLOCKS_PER_SEC << std::endl;

    // std::cout << "merge track ..." << std::endl;
    // start_ = clock();
    if(old_tracks_idx.size() > 1){
        MatrixXd old_tracks_points;
        int total_rows = 0;
        for(int i = 0; i < old_tracks_idx.size(); i++){
            total_rows += trackers[old_tracks_idx[i]].points.rows();
        }

        old_tracks_points = MatrixXd::Zero(total_rows, 3);

        int now_row = 0;
        for(int i = 0; i < old_tracks_idx.size(); i++){
            old_tracks_points.middleRows(now_row, trackers[old_tracks_idx[i]].points.rows()) = trackers[old_tracks_idx[i]].points;
            now_row += trackers[old_tracks_idx[i]].points.rows();
        }


        merge_track(old_tracks_points, old_tracks_idx);
    }

    // end_ = clock();
    // std::cout << "merge track time : " << ((double) (end_ - start_)) / CLOCKS_PER_SEC << std::endl;

    end = clock();
    // std::cout << "now trackers num : " << trackers.size() << std::endl;
    if(((double) (end - start)) / CLOCKS_PER_SEC > dt){
        std::cout << "total compute time : " << ((double) (end - start)) / CLOCKS_PER_SEC << std::endl;
    }
    // std::cout << ((double) (end - start)) / CLOCKS_PER_SEC << std::endl;
}

#endif