#include "node_routing.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <Eigen/Dense>

NodeRouting::NodeConnection::NodeConnection(std::vector<std::string> encrypt_table, std::vector<NodeRouting::Pose> nodeInfo_table)
    : encryptTable(encrypt_table), nodeInfoTable(nodeInfo_table) {
    path.clear();
    output_pathArray.clear();
    knotList.clear();
}

void NodeRouting::NodeConnection::linear_path(NodeRouting::Pose start_pt, NodeRouting::Pose end_pt, double resolution, bool isInsert) {
    double dx = end_pt.px - start_pt.px;
    double dy = end_pt.py - start_pt.py;

    double distance = std::sqrt(dx * dx + dy * dy);
    int num_pt = static_cast<int>(std::floor(distance / resolution));
    if (num_pt <= 0) num_pt = 1; // Avoid division by zero

    dx /= num_pt;
    dy /= num_pt;

    std::vector<NodeRouting::Pose> temp_path;

    for (int k = 0; k <= num_pt; ++k) {
        NodeRouting::Pose temp;
        temp.px = start_pt.px + dx * k;
        temp.py = start_pt.py + dy * k;
        temp.pz = 0;
        temp.ox = 0;
        temp.oy = 0;
        temp.oz = 0;
        temp.ow = 1;
        if (isInsert) {
            temp_path.push_back(temp);
        } else {
            path.push_back(temp);
        }
    }

    if (isInsert) {
        path.insert(path.begin(), temp_path.rbegin(), temp_path.rend());
    }
}

void NodeRouting::NodeConnection::common_connection(std::vector<std::vector<int>> nodeArray, double resolution) {
    output_pathArray.clear();

    for (const auto& nodes : nodeArray) {
        path.clear();

        for (size_t j = 0; j < nodes.size() - 1; ++j) {
            linear_path(nodeInfoTable[nodes[j]], nodeInfoTable[nodes[j + 1]], resolution, false);
        }

        path.push_back(nodeInfoTable[nodes.back()]);
        output_pathArray.push_back(path);
    }
}

void NodeRouting::NodeConnection::BezierCurve_connection(std::vector<std::vector<int>> nodeArray, double linear_resolution, double bezier_length, double bezier_resolution) {
    output_pathArray.clear();

    for (const auto& nodes : nodeArray) {
        path.clear();
        if (nodes.size() < 3) {
            RCLCPP_WARN(rclcpp::get_logger("NodeConnection"), "Insufficient points for Bezier Curve connection.");
            continue;
        }

        NodeRouting::Pose last_Bezier_pt;
        for (size_t j = 0; j < nodes.size() - 2; ++j) {
            NodeRouting::Pose Bezier_pt[3];
            Bezier_pt[1] = nodeInfoTable[nodes[j + 1]];

            double dx, dy, distance, theta;

            if (j == 0) {
                dx = nodeInfoTable[nodes[j]].px - Bezier_pt[1].px;
                dy = nodeInfoTable[nodes[j]].py - Bezier_pt[1].py;
            } else {
                dx = last_Bezier_pt.px - Bezier_pt[1].px;
                dy = last_Bezier_pt.py - Bezier_pt[1].py;
            }

            distance = std::sqrt(dx * dx + dy * dy);
            if (distance > bezier_length) {
                theta = std::atan2(dy, dx);
                Bezier_pt[0].px = bezier_length * std::cos(theta) + Bezier_pt[1].px;
                Bezier_pt[0].py = bezier_length * std::sin(theta) + Bezier_pt[1].py;
            } else {
                Bezier_pt[0] = (j == 0) ? nodeInfoTable[nodes[j]] : last_Bezier_pt;
            }

            dx = nodeInfoTable[nodes[j + 2]].px - Bezier_pt[1].px;
            dy = nodeInfoTable[nodes[j + 2]].py - Bezier_pt[1].py;
            distance = std::sqrt(dx * dx + dy * dy);

            if (distance > bezier_length) {
                theta = std::atan2(dy, dx);
                Bezier_pt[2].px = bezier_length * std::cos(theta) + Bezier_pt[1].px;
                Bezier_pt[2].py = bezier_length * std::sin(theta) + Bezier_pt[1].py;
            } else {
                Bezier_pt[2] = nodeInfoTable[nodes[j + 2]];
            }

            if (j == 0) {
                linear_path(nodeInfoTable[nodes[0]], Bezier_pt[0], linear_resolution, false);
            } else {
                linear_path(last_Bezier_pt, Bezier_pt[0], linear_resolution, false);
            }

            for (double k = 0; k <= 1; k += bezier_resolution) {
                NodeRouting::Pose temp;
                temp.px = std::pow(1 - k, 2) * Bezier_pt[0].px + 2 * k * (1 - k) * Bezier_pt[1].px + std::pow(k, 2) * Bezier_pt[2].px;
                temp.py = std::pow(1 - k, 2) * Bezier_pt[0].py + 2 * k * (1 - k) * Bezier_pt[1].py + std::pow(k, 2) * Bezier_pt[2].py;
                temp.pz = 0;
                temp.ox = 0;
                temp.oy = 0;
                temp.oz = 0;
                temp.ow = 1;
                path.push_back(temp);
            }

            last_Bezier_pt = Bezier_pt[2];
        }

        linear_path(last_Bezier_pt, nodeInfoTable[nodes.back()], linear_resolution, false);
        path.push_back(nodeInfoTable[nodes.back()]);
        output_pathArray.push_back(path);
    }
}

double NodeRouting::NodeConnection::blend_function(double u, int i, int k) {
    if (k == 1) {
        return (knotList[i] <= u && u < knotList[i + 1]) ? 1.0 : 0.0;
    }
    double left = (knotList[k + i - 1] - knotList[i]);
    double right = (knotList[k + i] - knotList[i + 1]);

    double b1 = (left > 0) ? (u - knotList[i]) / left * blend_function(u, i, k - 1) : 0.0;
    double b2 = (right > 0) ? (knotList[i + k] - u) / right * blend_function(u, i + 1, k - 1) : 0.0;

    return b1 + b2;
}

void NodeRouting::NodeConnection::BSplineCurve_connection(std::vector<std::vector<int>> nodeArray, double linear_resolution, int BSpline_k, double BSpline_resolution) {
    output_pathArray.clear();

    for (const auto& nodes : nodeArray) {
        path.clear();
        knotList.clear();

        int p_num = nodes.size();
        int knot_list_size = p_num + BSpline_k;

        for (int j = 0; j < knot_list_size; ++j) {
            knotList.push_back(static_cast<double>(j) / (knot_list_size - 1));
        }

        for (double u = 0; u <= 1; u += BSpline_resolution) {
            NodeRouting::Pose BSpline_pt = {0, 0, 0, 0, 0, 0, 1};

            for (int j = 0; j < p_num; ++j) {
                double basis = blend_function(u, j, BSpline_k);
                BSpline_pt.px += basis * nodeInfoTable[nodes[j]].px;
                BSpline_pt.py += basis * nodeInfoTable[nodes[j]].py;
            }

            if (knotList[BSpline_k - 1] <= u && u <= knotList[p_num]) {
                path.push_back(BSpline_pt);
            }
        }

        linear_path(nodeInfoTable[nodes[0]], path.front(), linear_resolution, true);
        linear_path(path.back(), nodeInfoTable[nodes.back()], linear_resolution, false);
        path.push_back(nodeInfoTable[nodes.back()]);
        output_pathArray.push_back(path);
    }
}

void NodeRouting::NodeConnection::addpath_orienation() {
    for (auto& path : output_pathArray) {
        for (size_t j = 0; j < path.size() - 1; ++j) {
            double delta_x = path[j + 1].px - path[j].px;
            double delta_y = path[j + 1].py - path[j].py;
            double yaw = std::atan2(delta_y, delta_x);

            // Eigen::Vector3d ea0(yaw, 0, 0);
            // Eigen::Matrix3d R = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()) *
            //                     Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY()) *
            //                     Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
            // Eigen::Quaterniond q(R);

            Eigen::Vector3d ea0(yaw, 0, 0);

            Eigen::Quaterniond q = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());


            path[j].ox = q.x();
            path[j].oy = q.y();
            path[j].oz = q.z();
            path[j].ow = q.w();
        }
    }
}
