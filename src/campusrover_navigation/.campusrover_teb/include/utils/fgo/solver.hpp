#ifndef SOLVER_HPP 
#define SOLVER_HPP

#include "factor_graph_utils.hpp"
#include "linear_solver.hpp"

class Solver{
    public:

        Solver(std::shared_ptr<LinearSolverBridge> _linear_solver);
        void set_verbose(bool _verbose);
        void set_gradient_decent_lr(double _lr);
        void set_L_M_tao(double _tao);
        void gradient_decent(std::shared_ptr<Vertex> vertex);
        void gauss_newton_update(std::shared_ptr<Vertex> vertex);
        void L_M_update(std::shared_ptr<Vertex> vertex, bool is_init, double error, std::function<double(std::shared_ptr<Vertex>)> error_func);

    private:
        bool verbose;
        double tao;
        double lr;
        MatrixXdd I;
        VectorXdd delta;
        MatrixXdd hessian_;
        std::shared_ptr<LinearSolverBridge> linear_solver;
        
        inline double max_matrix_diagonale(MatrixXdd& mat){
            double max = mat(0, 0);

            for(int i = 0; i < mat.rows(); i++){
                if(mat(i, i) > max){
                    max = mat(i, i);
                }
            }

            return max;
        };
};

Solver::Solver(std::shared_ptr<LinearSolverBridge> _linear_solver){
    tao = 1e-6;
    lr = 1e-9;
    verbose = false;
    linear_solver = _linear_solver;
}

void Solver::set_verbose(bool _verbose){
    verbose = _verbose;
}

void Solver::set_L_M_tao(double _tao){
    tao = _tao;
}

void Solver::set_gradient_decent_lr(double _lr){
    lr = _lr;
}

void Solver::gradient_decent(std::shared_ptr<Vertex> vertex){
    delta = -1* lr *  vertex->gradient;

    vertex->opt_x = vertex->x + delta;
}

void Solver::gauss_newton_update(std::shared_ptr<Vertex> vertex){

    if(vertex->hessian.determinant() != 0){
        linear_solver->solver(vertex->hessian, vertex->gradient, delta);

        vertex->opt_x = vertex->x + delta;
    }else{
        I = MatrixXdd::Identity(vertex->x.rows(), vertex->x.rows());
        double u = tao*max_matrix_diagonale(vertex->hessian);
        hessian_ = vertex->hessian + u*I;

        linear_solver->solver(hessian_, vertex->gradient, delta);

        vertex->opt_x = vertex->x + delta;
        if(verbose){
            std::cout << "vertex id : " << vertex->id << " is not invertible" << std::endl;
        }
    }
}

void Solver::L_M_update(std::shared_ptr<Vertex> vertex, bool is_init, double error, std::function<double(std::shared_ptr<Vertex>)> error_func){
    I = MatrixXdd::Identity(vertex->x.rows(), vertex->x.rows());

    vertex->u = is_init ? tao*max_matrix_diagonale(vertex->hessian) : vertex->u;
    vertex->v = is_init ? 2 : vertex->v;

    hessian_ = vertex->hessian + vertex->u*I;

    linear_solver->solver(hessian_, vertex->gradient, delta);

    vertex->x += delta;
    double error_ = error_func(vertex);

    // delta_F = F(x) - F(x_), where F(x) = (f(x).t * f(x)) / 2
    double delta_F = (error - error_) / 2;
    
    // delta_L = h.t * (u*h - j.t*f)/2
    double delta_L = (delta.transpose() * (vertex->u * delta -  vertex->gradient))[0] / 2;

    if(delta_L > DERIV_STEP && delta_F != 0){
        double roi = delta_F/delta_L ;
        
        if(roi > 0){
            vertex->opt_x = vertex->x;
            vertex->x = vertex->x_origin;

            vertex->u *= (1.0/3.0 > 1-pow(2*roi-1, 3)) ? 1.0/3.0 : 1-pow(2*roi-1, 3);
            vertex->v = 2;
        }else{
            vertex->u *= vertex->v;
            vertex->v *= 2;

            vertex->opt_x = vertex->x_origin;
            vertex->x = vertex->x_origin;
        }

    }else{
        vertex->opt_x = vertex->x_origin;
        vertex->x = vertex->x_origin;
    }
}

#endif