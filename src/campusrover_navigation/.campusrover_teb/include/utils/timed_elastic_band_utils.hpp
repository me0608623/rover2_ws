#ifndef TIMED_ELASTIC_BAND_UTILS_HPP
#define TIMED_ELASTIC_BAND_UTILS_HPP

#include <Eigen/Dense>

using namespace Eigen;

struct Obstacle{
    Vector2d mean;
    Vector2d speed;
    Matrix2d sigma;
    double dim;
    bool is_dynamic;
};

class CostMap{
    public:
        uint32_t width;
        uint32_t height;
        double res;
        double offset_x;
        double offset_y;
        int* data;
    
        CostMap(uint32_t _width, uint32_t _height, float _res, double _offset_x, double _offset_y){
            width = _width;
            height = _height;
            res = _res;
            offset_x = _offset_x;
            offset_y = _offset_y;

            data = new int[width*height];
        };

        ~CostMap(){
            delete data;
        };

        double at(double x, double y){
            idx_row = (y-offset_y)/res;
            idx_col = (x-offset_x)/res;

            if(idx_col >= 0 && idx_col < width && idx_row >= 0 && idx_row < height){
                return (double)data[idx_row*width+idx_col];
            }
            return 1000;

        };

        int& at(int row, int col){
            return data[row*width+col];
        };

    private:
        int idx_row;
        int idx_col;
};

namespace TEBUtils{
    inline bool get_goal_pt(Vector3d now_pose, Vector3d &goal_pose, MatrixXd &path, double goal_dist){
        int min_idx, size = path.rows();

        if(size > 0){
            MatrixXd dist = MatrixXd::Zero(size, 1);
            for(int i = 0; i < size; i++){
                dist(i, 0) = (path.row(i).leftCols(2) - now_pose.topRows(2).transpose()).norm();
            }

            dist.col(0).minCoeff(&min_idx);

            if(goal_dist == 0){
                goal_pose = path.row(min_idx).transpose();

            }else{
                goal_pose = path.row(size-1).transpose();
                Vector2d now_pose_ = path.row(min_idx).leftCols(2);
                if((goal_pose.topRows(2) - now_pose_).norm() > goal_dist){

                    dist = MatrixXd::Ones(size, 1);

                    for(int i = min_idx; i < size; i++){
                        dist(i, 0) = fabs((path.row(i).leftCols(2) - now_pose_.transpose()).norm() - goal_dist);
                    }

                    dist.col(0).minCoeff(&min_idx);
                    goal_pose = path.row(min_idx).transpose();
                }
            }
        }

        return size != 0;
    }
};

#endif