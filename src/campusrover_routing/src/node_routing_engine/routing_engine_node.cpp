#include "node_routing.h"

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <campusrover_msgs/srv/module_info.hpp>
#include <campusrover_msgs/msg/working_floor.hpp>
#include <campusrover_msgs/srv/routing_path.hpp>

class RoutingEngineNode : public rclcpp::Node {
public:
    RoutingEngineNode() : Node("routing_engine_node"), sw_(false) {
        declare_parameters();
        get_parameters();

        client_ = this->create_client<campusrover_msgs::srv::ModuleInfo>("get_route_info");
        sub_ = this->create_subscription<campusrover_msgs::msg::WorkingFloor>(
            "working_floor", 10,
            std::bind(&RoutingEngineNode::sub_callback, this, std::placeholders::_1)
        );

        server_ = this->create_service<campusrover_msgs::srv::RoutingPath>(
            "generation_path",
            std::bind(&RoutingEngineNode::server_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    bool use_csv_, enable_one_way_, path_orientation_, sw_;
    int BSpline_k_;
    double path_resolution_, bezier_length_, bezier_resolution_, BSpline_resolution_;
    std::string fpath_1_, fpath_2_, fpath_3_, f_nodeinfo_;
    std::string path_frame_, connect_method_;
    std::string f_path_;

    std::vector<std::string> encrypt_table_;
    std::vector<std::vector<double>> weight_table_;
    std::vector<NodeRouting::Linkage> linkage_table_;
    std::vector<NodeRouting::Pose> nodeInfo_table_;

    rclcpp::Client<campusrover_msgs::srv::ModuleInfo>::SharedPtr client_;
    rclcpp::Subscription<campusrover_msgs::msg::WorkingFloor>::SharedPtr sub_;
    rclcpp::Service<campusrover_msgs::srv::RoutingPath>::SharedPtr server_;

    void declare_parameters() {
        this->declare_parameter("enable_one_way", true);
        this->declare_parameter("use_csv", true);
        this->declare_parameter("path_orientation", false);
        this->declare_parameter("file_path1", "ggg.csv");
        this->declare_parameter("file_path2", "ggg.csv");
        this->declare_parameter("file_path3", "ggg.csv");
        this->declare_parameter("file_node_info", "ggg.csv");
        this->declare_parameter("connect_method", "common");
        this->declare_parameter("path_resolution", 0.05);
        this->declare_parameter("bezier_length", 3.0);
        this->declare_parameter("bezier_resolution", 0.01);
        this->declare_parameter("BSpline_k", 3);
        this->declare_parameter("BSpline_resolution", 0.001);
        this->declare_parameter("path_frame", "map");
    }

    void get_parameters() {
        this->get_parameter("enable_one_way", enable_one_way_);
        this->get_parameter("use_csv", use_csv_);
        this->get_parameter("path_orientation", path_orientation_);
        this->get_parameter("file_path1", fpath_1_);
        this->get_parameter("file_path2", fpath_2_);
        this->get_parameter("file_path3", fpath_3_);
        this->get_parameter("file_node_info", f_nodeinfo_);
        this->get_parameter("connect_method", connect_method_);
        this->get_parameter("path_resolution", path_resolution_);
        this->get_parameter("bezier_length", bezier_length_);
        this->get_parameter("bezier_resolution", bezier_resolution_);
        this->get_parameter("BSpline_k", BSpline_k_);
        this->get_parameter("BSpline_resolution", BSpline_resolution_);
        this->get_parameter("path_frame", path_frame_);
    }

    void sub_callback(const campusrover_msgs::msg::WorkingFloor::SharedPtr msg) {
        int floor;
        int size1, size2;

        auto mdlif_req = std::make_shared<campusrover_msgs::srv::ModuleInfo::Request>();

        NodeRouting::Node node;
        std::vector<NodeRouting::Node> node_array;
        std::vector<std::vector<std::string>> module;
        
        if(use_csv_){
            RCLCPP_INFO(this->get_logger(), "use data from csv !!");

            floor = std::stoi(msg->floor);

            if(floor == 1){
                f_path_ = fpath_1_;
            } else if(floor > 1 && floor < 12){
                f_path_ = fpath_2_;
            } else if(floor == 12){
                f_path_ = fpath_3_;
            }

            NodeRouting::ModuleLinkage ml(node_array);
            ml.csvFile_linkage(f_path_, f_nodeinfo_);
            encrypt_table_ = ml.encrypt_table;
            weight_table_ = ml.weight_table;
            linkage_table_  = ml.linkage_table;
            nodeInfo_table_ = ml.nodeInfo_table;

            if(encrypt_table_.size() == 0){
                RCLCPP_WARN(this->get_logger(), "lose some node info data !!");
            }
            sw_ = 1;

        } else {
            RCLCPP_INFO(this->get_logger(), "use data from database !!");

            // 填 Service Request
            mdlif_req->building = msg->building;
            mdlif_req->floor = msg->floor;

            if (!client_->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_WARN(this->get_logger(), "Service not available yet");
                sw_ = 0;
                return;
            }
            auto node_array_ptr = std::make_shared<std::vector<NodeRouting::Node>>();
            auto module_ptr = std::make_shared<std::vector<std::vector<std::string>>>();
            auto future_result = client_->async_send_request(mdlif_req,
                [this, node_array_ptr, module_ptr](rclcpp::Client<campusrover_msgs::srv::ModuleInfo>::SharedFuture mdlif_res) {
                    auto res = mdlif_res.get();
                    path_frame_ = res->frame_id;

                    int size1 = res->node.size();
                    for (int i = 0; i < size1; i++) {
                        NodeRouting::Node node;
                        node.name = res->node[i].name;
                        node.pose.px = res->node[i].pose.position.x;
                        node.pose.py = res->node[i].pose.position.y;
                        node.pose.pz = res->node[i].pose.position.z;
                        node.pose.ow = res->node[i].pose.orientation.w;
                        node.pose.ox = res->node[i].pose.orientation.x;
                        node.pose.oy = res->node[i].pose.orientation.y;
                        node.pose.oz = res->node[i].pose.orientation.z;
                        node_array_ptr->push_back(node);
                    }
                    // 模組連接
                    for (int i = 0; i < size1; i++) {
                        std::vector<std::string> temp;
                        temp.push_back(res->node[i].name);
                        module_ptr->push_back(temp);
                    }

                    int size2 = res->connections.size();
                    auto &module = *module_ptr;
                    for (int i = 0; i < size1; i++) {
                        for (int j = 0; j < size2; j++) {
                            if (enable_one_way_) {
                                if (module[i][0] == res->connections[j].connection[0]) {
                                    module[i].push_back(res->connections[j].connection[1]);
                                }
                            } else {
                                if (module[i][0] == res->connections[j].connection[0]) {
                                    module[i].push_back(res->connections[j].connection[1]);
                                } else if (module[i][0] == res->connections[j].connection[1]) {
                                    module[i].push_back(res->connections[j].connection[0]);
                                }
                            }
                        }
                    }
                    NodeRouting::ModuleLinkage ml(*module_ptr, *node_array_ptr);
                    ml.module_linkage();
                    encrypt_table_ = ml.encrypt_table;
                    weight_table_ = ml.weight_table;
                    linkage_table_  = ml.linkage_table;
                    nodeInfo_table_ = ml.nodeInfo_table;
                    if(encrypt_table_.size() == 0){
                        RCLCPP_WARN(this->get_logger(), "lose some node info data !!");
                    }
                    RCLCPP_INFO(this->get_logger(), "get data from database !!");
                    sw_ = 1; // 資料準備完成
                }
            );

        }
    }


    void server_callback(const std::shared_ptr<campusrover_msgs::srv::RoutingPath::Request> req,
                         std::shared_ptr<campusrover_msgs::srv::RoutingPath::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Request received");
        if (!sw_) {
            RCLCPP_WARN(this->get_logger(), "No data for server yet.");
            return;
        }

        NodeRouting::RoutingEngine re(encrypt_table_, linkage_table_, weight_table_);
        NodeRouting::NodeConnection nc(encrypt_table_, nodeInfo_table_);

        re.node_routing(req->origin, req->destination);
        RCLCPP_INFO(this->get_logger(), "Request received1");
        if (connect_method_ == "common") {
            nc.common_connection(re.output_nodeArray, path_resolution_);
        } else if (connect_method_ == "BezierCurve") {
            nc.BezierCurve_connection(re.output_nodeArray, path_resolution_, bezier_length_, bezier_resolution_);
        } else if (connect_method_ == "BSplineCurve") {
            nc.BSplineCurve_connection(re.output_nodeArray, path_resolution_, BSpline_k_, BSpline_resolution_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown connection method specified.");
        }
        RCLCPP_INFO(this->get_logger(), "Request received2");
        if (path_orientation_) nc.addpath_orienation();
        RCLCPP_INFO(this->get_logger(), "Request received3");
        for (const auto& path : nc.output_pathArray) {
            nav_msgs::msg::Path out_path;
            out_path.header.frame_id = path_frame_;
            // out_path.header.stamp = this->now();
            out_path.header.stamp = rclcpp::Clock().now();
            

            for (const auto& pt : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = out_path.header;
                pose.pose.position.x = pt.px;
                pose.pose.position.y = pt.py;
                pose.pose.position.z = pt.pz;
                pose.pose.orientation.x = pt.ox;
                pose.pose.orientation.y = pt.oy;
                pose.pose.orientation.z = pt.oz;
                pose.pose.orientation.w = pt.ow;
                out_path.poses.push_back(pose);
            }
            RCLCPP_INFO(this->get_logger(), "Request received4");
            res->routing.push_back(out_path);
            RCLCPP_INFO(this->get_logger(), "Request received5");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutingEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
