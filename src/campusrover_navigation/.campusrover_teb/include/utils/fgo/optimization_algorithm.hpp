#ifndef OPTIMIZATION_ALGORITHM_HPP 
#define OPTIMIZATION_ALGORITHM_HPP

#include "factor_graph_utils.hpp"
#include "factor_graph.hpp"
#include "solver.hpp"

enum class SolverType{
    GRADIENT_DECENT,
    GAUSS_NEWTON,
    LEVENBERG_MARQUARDT
};

enum class UpdateType{
    SEQUENCE,
    FINAL,
};

class MultiObjectiveOptimization{
    public:
        std::shared_ptr<Solver> optimizer;
        std::shared_ptr<FactorGraph> fg;

        MultiObjectiveOptimization(std::shared_ptr<FactorGraph> _fg, std::shared_ptr<Solver> _optimizer);
        void optimize(int iterate_num, double mse_min, double delta_mse_min, SolverType type, UpdateType mode, bool verbose);

    private:
        VectorXdd error;
        MatrixXdd jacobian;
        
        std::function<double(std::shared_ptr<Vertex>)> err_func = [](std::shared_ptr<Vertex> vertex) -> double {
            double error_norm = 0;
            VectorXdd error;
            
            int edge_num = vertex->edges.size();
            for(int i = 0; i < edge_num; i++){
                error = vertex->edges[i]->compute_error(vertex->time_stamp);
                error_norm += (error.transpose() * vertex->edges[i]->weight * error)(0, 0);
            }
            
            return error_norm;
        };
};

MultiObjectiveOptimization::MultiObjectiveOptimization(std::shared_ptr<FactorGraph> _fg, std::shared_ptr<Solver> _optimizer){
    fg = _fg;
    optimizer = _optimizer;
}

void MultiObjectiveOptimization::optimize(int iterate_num, double mse_min, double delta_mse_min, SolverType type, UpdateType mode, bool verbose){
    int vertex_num = fg->vertexs_container.size();
    int vertex_fixed_num = vertex_num;
    
    int edge_total = 0;

    for(int iterate = 0; iterate < iterate_num; iterate++){
        if(vertex_fixed_num > 0){
            vertex_fixed_num = 0;

            if(verbose){
                std::cout << "===============" << std::endl;
                std::cout << "iteratie num : " << iterate << std::endl;
            }

            for(int i = 0; i < vertex_num; i++){
                if(fg->vertexs_container[i]->fixed){
                    vertex_fixed_num++;
                    
                    double mse = 0;
                    double error_norm = 0;
                    fg->vertexs_container[i]->hessian.setZero();
                    fg->vertexs_container[i]->gradient.setZero();
                    
                    int edge_num = fg->vertexs_container[i]->edges.size();
                    edge_total += edge_num;
                    for(int j = 0; j < edge_num; j++){
                        if(fg->vertexs_container[i]->edges[j]->is_compute){

                            error = fg->vertexs_container[i]->edges[j]->compute_error(fg->vertexs_container[i]->time_stamp);
                            jacobian = fg->vertexs_container[i]->edges[j]->compute_jacobian(fg->vertexs_container[i]);
                            
                            error_norm += (error.transpose() * fg->vertexs_container[i]->edges[j]->weight * error)[0];

                            fg->vertexs_container[i]->hessian += jacobian.transpose() * fg->vertexs_container[i]->edges[j]->weight * jacobian;
                            fg->vertexs_container[i]->gradient += jacobian.transpose() * fg->vertexs_container[i]->edges[j]->weight * error;
                            
                            mse += error_norm / fg->vertexs_container[i]->edges[j]->weight.cwiseSqrt().sum();

                        }else{
                            fg->vertexs_container[i]->edges[j]->compute_error(fg->vertexs_container[i]->time_stamp);
                        }
                    }

                    switch(type){
                        case SolverType::GRADIENT_DECENT : {
                            optimizer->gradient_decent(fg->vertexs_container[i]);
                        }
                        
                        case SolverType::GAUSS_NEWTON : {
                            optimizer->gauss_newton_update(fg->vertexs_container[i]);
                            break;
                        }

                        case SolverType::LEVENBERG_MARQUARDT : {
                            optimizer->L_M_update(fg->vertexs_container[i], iterate==0, error_norm, err_func);
                            break;
                        }
                    }

                    if(mode == UpdateType::SEQUENCE){
                        fg->vertexs_container[i]->update();
                    }

                    mse /= edge_num;
                    if(mse <= mse_min && mse_min != -1){
                        fg->vertexs_container[i]->fixed = false;
                    }
                    
                    if(fabs(fg->vertexs_container[i]->last_mse - mse) <= delta_mse_min && delta_mse_min != -1){
                        fg->vertexs_container[i]->fixed = false;
                    }
                    fg->vertexs_container[i]->last_mse = mse;
                    
                    if(verbose){
                        std::cout << "vertex id : "<< fg->vertexs_container[i]->id << ", mse : " << mse << std::endl;
                    }
                }
            }
            
            if(mode == UpdateType::FINAL){
                fg->update_vertex();
            }
        }else{
            std::cout << "## stop iteratie by no vertex can update ##" << std::endl;
            break;
        }
    }
}

#endif