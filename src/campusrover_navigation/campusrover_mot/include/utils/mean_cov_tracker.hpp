#ifndef MEAN_COV_TRACKER_HPP
#define MEAN_COV_TRACKER_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "kalman_filter.hpp"
#include "tracker_utils.hpp"

using namespace Eigen;

class MeanCovTracker{
    public:
        TrackerParam param;

        MatrixXd points;
        double update_time;
        double born_time;

        int uid;
        int label;
        std::string name;

        bool is_kill;
        int detection_count;

        ObjectStatus status;
        int status_int;

        MatrixXd mean_2X1;
        MatrixXd mean_3X1;

        MatrixXd covariance_2X2;
        MatrixXd cov_eigen_value;
        MatrixXd cov_eigen_vector;

        MatrixXd cov_s_r_2X1;
        MatrixXd mean_s_r_4X1;

        MatrixXd speed_2X1;

        MatrixXd anchor_mean_2X1;

        std::pair<BoundaryPt, BoundaryPt> h_fov_range;
        std::pair<BoundaryPt, BoundaryPt> v_fov_range;

        std::vector<MatrixXd> history_mean_2X1;

        std::shared_ptr<KalmanFilter> kf;

        MeanCovTracker(MatrixXd _points, double _time, int _uid, TrackerParam _param);
        void update(double dt, bool is_detected);
        void merge_update();
        void predict(double dt, Matrix<double, 2, 1> &mean_out, Matrix<double, 2, 2> &cov_out, Matrix<double, 2, 1> &speed_out);

    private:
        MatrixXd cov_compute(MatrixXd mat);
        void judge_status();
        void record_history(MatrixXd mat);
        std::pair<BoundaryPt, BoundaryPt> fov_range_compute(MatrixXd &pts, MatrixXd center_pt, int numerator, int denominator);

};

MatrixXd MeanCovTracker::cov_compute(MatrixXd mat){
    MatrixXd centered = mat.rowwise() - mat.colwise().mean();

    return (centered.transpose() * centered) / double(mat.rows() - 1);
}

void MeanCovTracker::judge_status(){
    MatrixXd history_mean = MatrixXd::Zero(2, 1);
    double len = (double)history_mean_2X1.size();

    for(int i = 0; i < history_mean_2X1.size(); i++){
        history_mean += (history_mean_2X1[i]/len);
    }

    if(status == ObjectStatus::STATIC){
        if(pow(history_mean(0, 0) - anchor_mean_2X1(0, 0), 2) + pow(history_mean(1, 0) - anchor_mean_2X1(1, 0), 2) > pow(param.anchor_dist_threshold, 2)){
            if(pow(speed_2X1(0, 0), 2) + pow(speed_2X1(1, 0), 2) > pow(param.speed_threshold, 2)){
                status = ObjectStatus::DYNAMIC;
                status_int = 1;

            }else{
                anchor_mean_2X1 = history_mean;
                status = ObjectStatus::STATIC;
                status_int = 0;
            }
        }
    }else if(status == ObjectStatus::DYNAMIC){
        if(pow(speed_2X1(0, 0), 2) + pow(speed_2X1(1, 0), 2) < pow(param.speed_threshold, 2)){
                anchor_mean_2X1 = history_mean;
                status = ObjectStatus::STATIC;
                status_int = 0;
        }
    }
}

void MeanCovTracker::record_history(MatrixXd mat){
    history_mean_2X1.push_back(mat);
    if(history_mean_2X1.size() > param.history_length){
        history_mean_2X1.erase(history_mean_2X1.begin());
    }
}

std::pair<BoundaryPt, BoundaryPt> MeanCovTracker::fov_range_compute(MatrixXd &pts, MatrixXd center_pt, int numerator, int denominator){
    int size = pts.rows();
    double theta = atan2(center_pt(numerator, 0), center_pt(denominator, 0));
    std::pair<BoundaryPt, BoundaryPt> range;

    range.first.theta = theta;
    range.first.pt_3X1 = center_pt;
    range.second = range.first;

    for(int i = 0; i < size; i++){
        theta = atan2(pts(i, numerator), pts(i, denominator));
        
        if(theta < range.first.theta){
            range.first.theta = theta;
            range.first.pt_3X1 = pts.row(i).transpose();

        }else if(theta > range.second.theta){
            range.second.theta = theta;
            range.second.pt_3X1 = pts.row(i).transpose();
        }
    }

    return range;
}

MeanCovTracker::MeanCovTracker(MatrixXd _points, double _time, int _uid, TrackerParam _param){
    points = _points;
    born_time = _time;
    update_time = _time;
    uid = _uid;
    param = _param;

    label = 0;

    is_kill = false;
    detection_count = 1;
    status = ObjectStatus::STATIC;
    status_int = 0;
    
    mean_2X1 = points.leftCols(2).colwise().mean().transpose();
    mean_3X1 = points.colwise().mean().transpose();
    covariance_2X2 = cov_compute(points.leftCols(2));

    EigenSolver<MatrixXd> es(covariance_2X2);

    cov_eigen_value = es.eigenvalues().real();
    cov_eigen_vector = es.eigenvectors().real();

    cov_s_r_2X1 = MatrixXd::Zero(2, 1);
    cov_s_r_2X1 << cov_eigen_value(0, 0)*cov_eigen_value(1, 0),
                   cov_eigen_value(0, 0)/cov_eigen_value(1, 0);

    mean_s_r_4X1 = MatrixXd::Zero(4, 1);
    speed_2X1 = MatrixXd::Zero(2, 1);

    anchor_mean_2X1 = MatrixXd::Zero(2, 1);

    h_fov_range = fov_range_compute(points, mean_3X1, 1, 0);
    v_fov_range = fov_range_compute(points, mean_3X1, 2, 0);

    history_mean_2X1.push_back(speed_2X1);

    kf = std::make_shared<KalmanFilter>(7,4); //7, 4 >> mean_x, mean_y, cov_eigen_value_product, cov_eigen_value_ratio, mean_x_, mean_y_, cov_eigen_value_product_,

    kf->x << mean_2X1(0, 0),
             mean_2X1(1, 0),
             cov_s_r_2X1(0, 0),
             cov_s_r_2X1(1, 0),
             speed_2X1(0, 0),
             speed_2X1(1, 0),
             0;

    kf->F << 1, 0, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 0, 1,
             0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 1;

    kf->H << 1, 0, 0, 0, 1, 0, 0,
             0, 1, 0, 0, 0, 1, 0,
             0, 0, 1, 0, 0, 0, 1,
             0, 0, 0, 1, 0, 0, 0;

    kf->P << 1, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, 0,
             0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 500, 0, 0,
             0, 0, 0, 0, 0, 500, 0,
             0, 0, 0, 0, 0, 0, 500;

    kf->Q << 0.0001, 0, 0, 0, 0.0001, 0, 0,
             0, 0.0001, 0, 0, 0, 0.0001, 0,
             0, 0, 0.001, 0, 0, 0, 0.0001,
             0, 0, 0, 0.001, 0, 0, 0, 
             0.01, 0, 0, 0, 0.01, 0, 0,
             0, 0.01, 0, 0, 0, 0.01, 0,
             0, 0, 0.1, 0, 0, 0, 0.1;

    kf->R << 0.0225, 0, 0, 0,
             0, 0.0225, 0, 0,
             0, 0, 0.4, 0,
             0, 0, 0, 0.4;
}

void MeanCovTracker::update(double dt, bool is_detected){
    kf->H(0, 4) = dt;
    kf->H(1, 5) = dt;
    kf->H(2, 6) = dt;

    kf->predict();
    if(is_detected){
        detection_count++;
        mean_2X1 = points.leftCols(2).colwise().mean().transpose();
        mean_3X1 = points.colwise().mean().transpose();
        record_history(speed_2X1);

        covariance_2X2 = cov_compute(points.leftCols(2));

        EigenSolver<MatrixXd> es(covariance_2X2);

        cov_eigen_value = es.eigenvalues().real();
        cov_eigen_vector = es.eigenvectors().real();

        cov_s_r_2X1 << cov_eigen_value(0, 0)*cov_eigen_value(1, 0),
                       cov_eigen_value(0, 0)/cov_eigen_value(1, 0);
                   
        mean_s_r_4X1.topRows(2) = mean_2X1;
        mean_s_r_4X1.bottomRows(2) = cov_s_r_2X1;

        kf->update(mean_s_r_4X1);
        
        speed_2X1 << kf->x(4, 0),
                     kf->x(5, 0);

        h_fov_range = fov_range_compute(points, mean_3X1, 1, 0);
        v_fov_range = fov_range_compute(points, mean_3X1, 2, 0);
    }else{
        mean_2X1 = kf->x.topRows(2);

        speed_2X1 << kf->x(4, 0),
                     kf->x(5, 0);
    }

    judge_status();
}

void MeanCovTracker::merge_update(){
    mean_2X1 = points.leftCols(2).colwise().mean().transpose();
    mean_3X1 = points.colwise().mean().transpose();
    history_mean_2X1.clear();

    h_fov_range = fov_range_compute(points, mean_3X1, 1, 0);
    v_fov_range = fov_range_compute(points, mean_3X1, 2, 0);
}

void MeanCovTracker::predict(double dt, Matrix<double, 2, 1> &mean_out, Matrix<double, 2, 2> &cov_out, Matrix<double, 2, 1> &speed_out){
    kf->F(0, 4) = dt;
    kf->F(1, 5) = dt;
    kf->F(2, 6) = dt;

    MatrixXd pre = kf->get_predict();

    mean_out = pre.topRows(2);

    double eigen_value0 = sqrt(pre(2, 0)*pre(3, 0));
    double eigen_value1 = pre(2, 0)/eigen_value0;

    Matrix2d diag_eigen_value;
    diag_eigen_value << eigen_value0, 0,
                        0, eigen_value1;

    cov_out = cov_eigen_vector.inverse()*diag_eigen_value*cov_eigen_vector.inverse().transpose();

    speed_out << pre(4, 0),
                 pre(5, 0);
}

#endif