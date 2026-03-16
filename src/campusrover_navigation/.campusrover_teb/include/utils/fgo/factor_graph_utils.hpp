#ifndef FACTOR_GRAPH_UTILS_HPP
#define FACTOR_GRAPH_UTILS_HPP

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <iostream>

#define DERIV_STEP 1e-6

using namespace Eigen;

typedef Eigen::Matrix<double, -1, 1, 0, 5, 1> VectorXdd;
typedef Eigen::Matrix<double, -1, -1, 0, 5, 5> MatrixXdd;
// typedef Eigen::Matrix<double, -1, 1> VectorXdd;
// typedef Eigen::Matrix<double, -1, -1> MatrixXdd;

class Vertex;
class Edge;

class Vertex{
    public:
        int id;
        double time_stamp;
        bool fixed;
        double u, v;            //use for L-M optimization
        double last_mse;             
        VectorXdd x;
        VectorXdd x_origin;
        VectorXdd opt_x;
        MatrixXdd hessian;
        VectorXdd gradient;
        std::vector<std::shared_ptr<Edge>> edges;

        Vertex(VectorXdd _x, bool _fixed);
        void add_edge(std::shared_ptr<Edge> edge);
        virtual void updat_data(VectorXdd _x);
        void update();
};

template<int vertex_size>
class BaseVertex : public Vertex{
    public:
        BaseVertex(VectorXdd _x, bool _fixed) : Vertex(_x, _fixed){
            hessian = MatrixXdd::Zero(vertex_size, vertex_size);
            gradient = VectorXdd::Zero(vertex_size);
        }
};

class Edge{
    public:
        int id;
        double time_stamp;
        bool is_compute;
        VectorXdd error;
        MatrixXdd jacobian;
        MatrixXdd weight;
        std::vector<std::shared_ptr<Vertex>> vertexs;
        std::shared_ptr<Vertex> obj_vertex;

        Edge();
        void add_vertex(std::shared_ptr<Vertex> vertex);
        void add_obj_vertex(std::shared_ptr<Vertex> vertex);
        virtual VectorXdd compute_error(double vertex_time_stamp);
        virtual MatrixXdd compute_jacobian(std::shared_ptr<Vertex> vertex);
};

template<int edge_size>
class BaseEdge : public Edge{
    public:
        BaseEdge(){
            error = VectorXdd::Zero(edge_size);
            weight = MatrixXdd::Identity(edge_size, edge_size);
        }
};

Vertex::Vertex(VectorXdd _x, bool _fixed){
    x = _x;
    x_origin = x;
    opt_x = x;
    fixed = _fixed;
    edges.clear();
}

void Vertex::add_edge(std::shared_ptr<Edge> edge){
    edges.push_back(edge);
}

void Vertex::updat_data(VectorXdd _x){
    x = _x;
    x_origin = x;
}

void Vertex::update(){
    x = opt_x;
    x_origin = opt_x;
}


Edge::Edge(){
    vertexs.clear();
    is_compute = true;
}

void Edge::add_vertex(std::shared_ptr<Vertex> vertex){
    vertexs.push_back(vertex);
}

void Edge::add_obj_vertex(std::shared_ptr<Vertex> vertex){
    obj_vertex = vertex;
}


VectorXdd Edge::compute_error(double vertex_time_stamp){
    return VectorXdd::Zero(0);
}

MatrixXdd Edge::compute_jacobian(std::shared_ptr<Vertex> vertex){
    jacobian = MatrixXdd::Zero(error.rows(), obj_vertex->x.rows());

    for(int i = 0; i < jacobian.cols(); i++){
        obj_vertex->x[i] = obj_vertex->x_origin[i] + DERIV_STEP;
        VectorXdd error_2 = compute_error(vertex->time_stamp);

        obj_vertex->x[i] = obj_vertex->x_origin[i] - DERIV_STEP;
        VectorXdd error_1 = compute_error(vertex->time_stamp);

        obj_vertex->x[i] = obj_vertex->x_origin[i];
        jacobian.col(i) = (error_2 - error_1) / (2*DERIV_STEP);
    }
    return jacobian;
}

#endif