#ifndef LINEAR_SOLVER_HPP
#define LINEAR_SOLVER_HPP

#include <Eigen/Dense>
#include "factor_graph_utils.hpp"

using namespace Eigen;

enum class LinearSolverBridgeType{
    NORMAL,
    FULL_PIV_LU,
    PARTIAL_PIV_LU,
    LDLT,
    LLT
};

class LinearSolverBridge{
    public:
    
        LinearSolverBridge(LinearSolverBridgeType type){
            switch(type){
                case LinearSolverBridgeType::NORMAL : {
                    solver = [](MatrixXdd& mat_A, VectorXdd& mat_B, VectorXdd& mat_out){
                        mat_out = -1 * mat_A.inverse() * mat_B;
                    };

                    break;
                }

                case LinearSolverBridgeType::FULL_PIV_LU : {
                    solver = [](MatrixXdd& mat_A, VectorXdd& mat_B, VectorXdd& mat_out){
                        mat_out = -1 * mat_A.fullPivLu().solve(mat_B);
                    };

                    break;
                }

                case LinearSolverBridgeType::PARTIAL_PIV_LU : {
                    solver = [](MatrixXdd& mat_A, VectorXdd& mat_B, VectorXdd& mat_out){
                        mat_out = -1 * mat_A.partialPivLu().solve(mat_B);
                    };

                    break;
                }

                case LinearSolverBridgeType::LDLT : {
                    solver = [](MatrixXdd& mat_A, VectorXdd& mat_B, VectorXdd& mat_out){
                        mat_out = -1 * mat_A.ldlt().solve(mat_B);
                    };

                    break;
                }

                case LinearSolverBridgeType::LLT : {
                    solver = [](MatrixXdd& mat_A, VectorXdd& mat_B, VectorXdd& mat_out){
                        mat_out = -1 * mat_A.llt().solve(mat_B);
                    };

                    break;
                }
            }
        };

        std::function<void(MatrixXdd& , VectorXdd& , VectorXdd& )> solver;
};

#endif