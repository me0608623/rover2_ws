#include "node_routing.h"
#include <iostream>
#include <cmath>

NodeRouting::RoutingEngine::RoutingEngine(
    std::vector<std::string> encrypt_table,
    std::vector<NodeRouting::Linkage> linkage_table,
    std::vector<std::vector<double>> weight_table)
    : encryptTable(std::move(encrypt_table)),
      linkageTable(std::move(linkage_table)),
      weightTable(std::move(weight_table))
{
    dijkstraTable.clear();
    output_nodeArray.clear();
    output_denodeArray.clear();
}

void NodeRouting::RoutingEngine::init_routing(int start) {
    dijkstraTable.clear();
    for (size_t i = 0; i < encryptTable.size(); i++) {
        NodeRouting::Dijkstra temp;
        temp.nodeName = i;
        temp.weight = (i == static_cast<size_t>(start)) ? 0 : -1;
        temp.last_nodeName = (i == static_cast<size_t>(start)) ? -1 : -2;
        dijkstraTable.push_back(temp);
    }
}

int NodeRouting::RoutingEngine::find_linkageTable_indexAddr(int index) {
    for (size_t i = 0; i < linkageTable.size(); i++) {
        if (linkageTable[i].index == index) {
            return i;
        }
    }
    std::cerr << "[RoutingEngine] No index found at: " << index << std::endl;
    return -1;
}

void NodeRouting::RoutingEngine::node_routing(std::string origin, std::vector<std::string> destination) {
    std::vector<int> task_array;
    std::vector<int> node_array;
    std::vector<std::string> denode_array;

    // Encrypt the origin and destination
    auto it = std::find(encryptTable.begin(), encryptTable.end(), origin);
    if (it != encryptTable.end()) {
        task_array.push_back(std::distance(encryptTable.begin(), it));
    }

    for (const auto& dest : destination) {
        auto it_dest = std::find(encryptTable.begin(), encryptTable.end(), dest);
        if (it_dest != encryptTable.end()) {
            task_array.push_back(std::distance(encryptTable.begin(), it_dest));
        }
    }

    for (size_t i = 0; i < destination.size(); i++) {
        init_routing(task_array[i]);

        int now_indexAddr = find_linkageTable_indexAddr(task_array[i]);
        if (now_indexAddr < 0) continue;

        int element = 0;
        for (const auto& l : linkageTable) {
            element += l.item.size();
        }

        std::vector<int> next_indexAddr;
        for (const auto& item : linkageTable[now_indexAddr].item) {
            int index = linkageTable[now_indexAddr].index;
            dijkstraTable[item].weight = weightTable[index][item];
            dijkstraTable[item].last_nodeName = index;
            int addr = find_linkageTable_indexAddr(item);
            if (addr >= 0) {
                next_indexAddr.push_back(addr);
            }
        }
        element -= linkageTable[now_indexAddr].item.size();

        while (element > 0 && !next_indexAddr.empty()) {
            std::vector<int> last_indexAddr = next_indexAddr;
            next_indexAddr.clear();

            for (const auto& now_index : last_indexAddr) {
                for (const auto& item : linkageTable[now_index].item) {
                    int index = linkageTable[now_index].index;
                    double last_weight = dijkstraTable[item].weight;
                    double next_weight = weightTable[index][item] + dijkstraTable[index].weight;

                    if (last_weight == -1 || next_weight < last_weight) {
                        dijkstraTable[item].weight = next_weight;
                        dijkstraTable[item].last_nodeName = index;
                        int addr = find_linkageTable_indexAddr(item);
                        if (addr >= 0) {
                            next_indexAddr.push_back(addr);
                        }
                    }
                }
                element -= linkageTable[now_index].item.size();
            }
        }

        // Find routing
        node_array.clear();
        denode_array.clear();
        int last_node = task_array[i + 1];

        while (true) {
            node_array.insert(node_array.begin(), dijkstraTable[last_node].nodeName);
            denode_array.insert(denode_array.begin(), encryptTable[dijkstraTable[last_node].nodeName]);
            last_node = dijkstraTable[last_node].last_nodeName;

            if (last_node == -1) break;
            if (last_node == -2) {
                std::cerr << "[RoutingEngine] No path between "
                          << task_array[i] << " and " << task_array[i + 1] << std::endl;
                node_array.clear();
                break;
            }
        }

        output_nodeArray.push_back(node_array);
        output_denodeArray.push_back(denode_array);
    }
}
