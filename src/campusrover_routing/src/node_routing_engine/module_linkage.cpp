#include "node_routing.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <cmath>

NodeRouting::ModuleLinkage::ModuleLinkage() {
    encrypt_table.clear();
    weight_table.clear();
    linkage_table.clear();
    nodeInfo_table.clear();
    nodeInfo.clear();
    nodeModule.clear();
}

NodeRouting::ModuleLinkage::ModuleLinkage(std::vector<std::vector<std::string>> node_module) {
    encrypt_table.clear();
    weight_table.clear();
    linkage_table.clear();
    nodeInfo_table.clear();
    nodeInfo.clear();
    nodeModule = node_module;
}

NodeRouting::ModuleLinkage::ModuleLinkage(std::vector<NodeRouting::Node> node_info) {
    encrypt_table.clear();
    weight_table.clear();
    linkage_table.clear();
    nodeInfo_table.clear();
    nodeModule.clear();
    nodeInfo = node_info;
}

NodeRouting::ModuleLinkage::ModuleLinkage(std::vector<std::vector<std::string>> node_module, std::vector<NodeRouting::Node> node_info) {
    encrypt_table.clear();
    weight_table.clear();
    linkage_table.clear();
    nodeInfo_table.clear();
    nodeModule = node_module;
    nodeInfo = node_info;
}

void NodeRouting::ModuleLinkage::module_linkage() {
    bool isWeightTable = true;

    if (nodeInfo.size() >= nodeModule.size()) {
        // Build Encrypt table
        for (const auto& node : nodeInfo) {
            encrypt_table.push_back(node.name);
        }

        // Build linkage table
        for (const auto& module : nodeModule) {
            NodeRouting::Linkage temp;

            for (size_t j = 0; j < module.size(); ++j) {
                auto it = std::find(encrypt_table.begin(), encrypt_table.end(), module[j]);
                if (it != encrypt_table.end()) {
                    int index = static_cast<int>(std::distance(encrypt_table.begin(), it));
                    if (j == 0) {
                        temp.index = index;
                    } else {
                        temp.item.push_back(index);
                    }
                } else {
                    isWeightTable = false;
                    RCLCPP_WARN(rclcpp::get_logger("ModuleLinkage"), "No data found for: %s", module[j].c_str());
                }
            }

            linkage_table.push_back(temp);
        }

        // Build weight table
        if (isWeightTable) {
            size_t size = nodeInfo.size();
            weight_table.resize(size, std::vector<double>(size, -1.0));

            for (const auto& linkage : linkage_table) {
                int index = linkage.index;
                for (const auto& item : linkage.item) {
                    weight_table[index][item] =
                        std::sqrt(std::pow(nodeInfo[index].pose.px - nodeInfo[item].pose.px, 2) +
                                  std::pow(nodeInfo[index].pose.py - nodeInfo[item].pose.py, 2));
                }
            }
        }

        // Build node info table
        for (const auto& node : nodeInfo) {
            nodeInfo_table.push_back(node.pose);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("ModuleLinkage"), "Missing node info data!!");
    }
}

void NodeRouting::ModuleLinkage::csvFile_linkage(std::string moduleFile, std::string nodeInfoFile) {
    std::string data;

    if (nodeInfoFile != "null") {
        std::ifstream inFile1(nodeInfoFile);
        if (inFile1) {
            while (std::getline(inFile1, data)) {
                std::stringstream ss(data);
                std::string str;
                std::vector<std::string> line_array;
                while (std::getline(ss, str, ',')) {
                    line_array.push_back(str);
                }
                if (line_array.size() >= 8) {
                    NodeRouting::Node temp;
                    temp.name = line_array[0];
                    temp.pose.px = std::stod(line_array[1]);
                    temp.pose.py = std::stod(line_array[2]);
                    temp.pose.pz = std::stod(line_array[3]);
                    temp.pose.ow = std::stod(line_array[4]);
                    temp.pose.ox = std::stod(line_array[5]);
                    temp.pose.oy = std::stod(line_array[6]);
                    temp.pose.oz = std::stod(line_array[7]);
                    nodeInfo.push_back(temp);
                }
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ModuleLinkage"), "Failed to open node info file: %s", nodeInfoFile.c_str());
        }
    }

    if (moduleFile != "null") {
        std::ifstream inFile2(moduleFile);
        if (inFile2) {
            while (std::getline(inFile2, data)) {
                std::stringstream ss(data);
                std::string str;
                std::vector<std::string> line_array;
                while (std::getline(ss, str, ',')) {
                    line_array.push_back(str);
                }
                nodeModule.push_back(line_array);
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ModuleLinkage"), "Failed to open module file: %s", moduleFile.c_str());
        }
    }

    module_linkage();
}
