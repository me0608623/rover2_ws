#ifndef FACTOR_GRAPH_HPP 
#define FACTOR_GRAPH_HPP

#include "factor_graph_utils.hpp"

class FactorGraph{
    public:
        std::vector<std::shared_ptr<Vertex>> vertexs_container;
        std::vector<std::shared_ptr<Edge>> edges_container;

        FactorGraph();
        void initial();
        void add_vertex(std::shared_ptr<Vertex> vertex);
        void add_edge(std::shared_ptr<Edge> edge);
        void show_factor_graph(int show_type);
        void update_vertex();

    private:
        int vertex_uid;
        int edge_uid;
};

FactorGraph::FactorGraph(){
    vertexs_container.clear();
    edges_container.clear();
    vertex_uid = 0;
    edge_uid = 0;
}

void FactorGraph::initial(){
    for(int i = 0; i < vertexs_container.size(); i++){
        vertexs_container[i]->edges.clear();
    }

    for(int i = 0; i < edges_container.size(); i++){
        edges_container[i]->vertexs.clear();
    }

    vertexs_container.clear();
    edges_container.clear();
    vertex_uid = 0;
    edge_uid = 0;
}

void FactorGraph::add_vertex(std::shared_ptr<Vertex> vertex){
    vertex->id = vertex_uid;
    vertexs_container.push_back(vertex);
    vertex_uid ++;
}

void FactorGraph::add_edge(std::shared_ptr<Edge> edge){
    edge->id = edge_uid;
    edges_container.push_back(edge);
    edge_uid ++;
}

void FactorGraph::show_factor_graph(int show_type){
    std::cout << "###################" << std::endl;
    std::cout << "## factor_graph ##" << std::endl;

    if(show_type == 2){
        for(int i = 0; i < edges_container.size(); i++){
            std::cout << "###################" << std::endl;
            std::cout << "edge's id : " << edges_container[i]->id  << std::endl;
            std::cout << "edges_container type : " << typeid(*edges_container[i]).name() << std::endl;
            std::cout << "<< connection vertex >>" << std::endl;

            for(int j = 0; j < edges_container[i]->vertexs.size(); j++){
                std::cout << "vertex's id : " << edges_container[i]->vertexs[j]->id << ",\ttype : " << typeid(*edges_container[i]->vertexs[j]).name() << std::endl;
            }
        }
    }else if(show_type == 1){
        for(int i = 0; i < vertexs_container.size(); i++){
            std::cout << "###################" << std::endl;
            std::cout << "vertex's id : " << vertexs_container[i]->id  << std::endl;
            std::cout << "vertex type : " << typeid(*vertexs_container[i]).name() << std::endl;
            std::cout << "vertex fixed : " << vertexs_container[i]->fixed << std::endl;
            std::cout << "<< connection edge >>" << std::endl;

            for(int j = 0; j < vertexs_container[i]->edges.size(); j++){
                std::cout << "edge's id : " << vertexs_container[i]->edges[j]->id << ",\ttype : " << typeid(*vertexs_container[i]->edges[j]).name() << std::endl;
            }
        }
    }
    std::cout << "###################" << std::endl;
}

void FactorGraph::update_vertex(){
    for(int i = 0; i < vertexs_container.size(); i++){
        vertexs_container[i]->update();
    }
}

#endif