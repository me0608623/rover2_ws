#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <campusrover_msgs/msg/node_info.hpp>
#include <campusrover_msgs/msg/working_floor.hpp>
#include <campusrover_msgs/srv/routing_path.hpp>
#include <campusrover_msgs/srv/module_info.hpp>

using std::string;
using std::vector;

class NodeRouting{
    public:
        class Pose{
            public:
                double px;
                double py;
                double pz;
                double ow;
                double ox;
                double oy;
                double oz;
        };

        class Node{
            public:
                string name;
                NodeRouting::Pose pose;
        };
        
        class Linkage{
            public:
                int index;
                vector<int> item;
        };

        class Dijkstra{
            public:
                int nodeName;
                double weight;
                int last_nodeName;
        };

        class ModuleLinkage{
            public:
                ModuleLinkage();
                ModuleLinkage(vector<vector<string> > node_module);
                ModuleLinkage(vector<NodeRouting::Node> node_info);
                ModuleLinkage(vector<vector<string> > node_module, vector<NodeRouting::Node> node_info);

                void csvFile_linkage(string nodeModuleFile, string nodeInfoFile);
                void module_linkage();

                vector<string> encrypt_table;
                vector<vector<double> > weight_table;
                vector<NodeRouting::Linkage> linkage_table;
                vector<NodeRouting::Pose> nodeInfo_table;
                
            private:
                vector<NodeRouting::Node> nodeInfo;
                vector<vector<string> > nodeModule;
        };

        class RoutingEngine{
            public:
                RoutingEngine(vector<string> encrypt_table, vector<NodeRouting::Linkage> linkage_table, vector<vector<double> > weight_table);
                
                void init_routing(int start);
                int find_linkageTable_indexAddr(int index);
                void node_routing(string origin, vector<string> destination);

                vector<vector<int> > output_nodeArray;
                vector<vector<string> > output_denodeArray;

            private:
                vector<string> encryptTable;
                vector<vector<double> > weightTable;
                vector<NodeRouting::Linkage> linkageTable;
                vector<Dijkstra> dijkstraTable;                   //dijkstra_table[0] : node name, dijkstra_table[1]: distance, dijkstra_table[2]: last node name      
        };

        class NodeConnection{
            public:
                NodeConnection(vector<string> encrypt_table, vector<NodeRouting::Pose> nodeInfo_table);

                void common_connection(vector<vector<int> > nodeArray, double resolution);
                void BezierCurve_connection(vector<vector<int> > nodeArray, double linear_resolution, double bezier_length, double bezier_resolution);
                void BSplineCurve_connection(vector<vector<int> > nodeArray, double linear_resolution, int BSpline_k, double BSpline_resolution);

                void linear_path(NodeRouting::Pose start_pt, NodeRouting::Pose end_pt, double resolution, bool isInsert);
                double blend_function(double u, int i, int k);      //u = 0~1, i = pt order, k = layer

                void addpath_orienation();

                vector<vector<NodeRouting::Pose> >output_pathArray;
            private:
                vector<double> knotList;
                vector<string> encryptTable;
                vector<NodeRouting::Pose> path;
                vector<NodeRouting::Pose> nodeInfoTable;
        };
};