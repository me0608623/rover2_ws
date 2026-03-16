#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter{
    public :
        int dim_x;
        int dim_z;

        MatrixXd x;

        MatrixXd F;
        MatrixXd H;

        MatrixXd P;

        MatrixXd Q;
        MatrixXd R;

        KalmanFilter(int _dim_x, int _dim_z);
        MatrixXd get_predict();
        void predict();
        void update(MatrixXd z);
        
};

KalmanFilter::KalmanFilter(int _dim_x, int _dim_z){
    dim_x = _dim_x;
    dim_z = _dim_z;

    x = MatrixXd::Zero(dim_x, 1);
    
    F = MatrixXd::Zero(dim_x, dim_x);
    H = MatrixXd::Zero(dim_z, dim_x);

    P = MatrixXd::Zero(dim_x, dim_x);

    Q = MatrixXd::Zero(dim_x, dim_x);
    R = MatrixXd::Zero(dim_z, dim_z);
}

void KalmanFilter::predict(){
    x = F * x;
    P = F * P * F.transpose() + Q;
}

MatrixXd KalmanFilter::get_predict(){
    return F * x;
}

void KalmanFilter::update(MatrixXd z){
    MatrixXd s = H * P * H.transpose() + R;
    MatrixXd k = P * H.transpose() * s.inverse();

    x = x + k * (z - H * x);

    MatrixXd I = MatrixXd::Identity(dim_x,dim_x);

    P = (I - k * H) * P;
}

#endif