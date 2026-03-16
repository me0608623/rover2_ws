#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <campusrover_msgs/srv/module_info.hpp>
#include <campusrover_msgs/msg/working_floor.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace std::chrono_literals;
using namespace campusrover_msgs::msg;
using namespace campusrover_msgs::srv;

class RoutesVisualization : public rclcpp::Node
{
public:
    RoutesVisualization() : Node("routes_visualization_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
        sub_ = this->create_subscription<WorkingFloor>("working_floor", 10, std::bind(&RoutesVisualization::callingData, this, std::placeholders::_1));
        client_ = this->create_client<ModuleInfo>("/get_route_info");

        marker_id_ = 0;
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<WorkingFloor>::SharedPtr sub_;
    rclcpp::Client<ModuleInfo>::SharedPtr client_;

    std::map<std::string, geometry_msgs::msg::Pose> room_coordinate_;
    std::map<int, std::vector<std::string>> room_connections_;
    std::vector<std::string> room_list_;
    visualization_msgs::msg::MarkerArray marker_all_;
    std::string pub_frame_ = "world";
    int marker_id_;

    void pubConnections()
    {
        for (const auto &[i, connections] : room_connections_)
        {
            for (size_t j = 0; j < connections.size() - 1; j++)
            {
                visualization_msgs::msg::Marker connect_line;
                connect_line.header.frame_id = pub_frame_;
                connect_line.header.stamp = now();
                connect_line.ns = "coordinate_visualization";
                connect_line.id = marker_id_++;
                connect_line.type = visualization_msgs::msg::Marker::LINE_LIST;
                connect_line.action = visualization_msgs::msg::Marker::ADD;
                connect_line.scale.x = 0.15;
                connect_line.scale.y = 0.0;
                connect_line.scale.z = 0.0;
                connect_line.color.g = 1.0f;
                connect_line.color.a = 1.0f;

                geometry_msgs::msg::Point p1, p2;
                if (room_coordinate_.count(connections[j]) == 0 || room_coordinate_.count(connections[j+1]) == 0) {
                    RCLCPP_WARN(this->get_logger(), "Connection refers to unknown node: %s or %s",
                                connections[j].c_str(), connections[j+1].c_str());
                    continue;
                }

                auto coord1 = room_coordinate_.at(connections[j]);
                auto coord2 = room_coordinate_.at(connections[j + 1]);

                p1.x = coord1.position.x;
                p1.y = coord1.position.y;
                p1.z = coord1.position.z;

                p2.x = coord2.position.x;
                p2.y = coord2.position.y;
                p2.z = coord2.position.z;

                connect_line.points.push_back(p1);
                connect_line.points.push_back(p2);

                marker_all_.markers.push_back(connect_line);
            }
        }
        marker_pub_->publish(marker_all_);
    }

    void pubPoints()
    {
        for (const auto &room_name : room_list_)
        {
            visualization_msgs::msg::Marker marker, text_marker;
            auto room_pose = room_coordinate_.at(room_name);

            marker.header.frame_id = pub_frame_;
            marker.header.stamp = now();
            marker.ns = "routes_visualization";
            marker.id = marker_id_++;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = room_pose;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
            marker.lifetime = rclcpp::Duration::from_seconds(0);
            
            
            text_marker = marker;
            text_marker.id = marker_id_++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.text = room_name;
            text_marker.pose.position.y -= 0.4;
            text_marker.pose.position.z = 0.1;
            text_marker.scale.x = text_marker.scale.y = text_marker.scale.z = 0.6;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 0.0f;
            text_marker.color.b = 0.0f;

            if (room_name.at(0) == 'c')
            {
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.scale.x = marker.scale.y = 0.5;
                marker.scale.z = 0.1;
            }
            else
            {
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.scale.x = 2.0;
                marker.scale.y = 0.2;
                marker.scale.z = 0.1;
            }

            marker_all_.markers.push_back(marker);
            marker_all_.markers.push_back(text_marker);
            
        }

        pubConnections();
    }

    void callingData(const WorkingFloor::SharedPtr msg)
    {
        auto request = std::make_shared<ModuleInfo::Request>();
        request->building = msg->building;
        request->floor = msg->floor;

        marker_all_.markers.clear();
        room_list_.clear();
        room_coordinate_.clear();
        room_connections_.clear();

        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to be available...");
        }

        // auto result = client_->async_send_request(request);
        // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     auto response = result.get();
        //     pub_frame_ = "world";

        //     for (const auto &node : response->node)
        //     {
        //         room_list_.push_back(node.name);
        //         room_coordinate_[node.name] = node.pose;
        //     }

        //     for (size_t i = 0; i < response->connections.size(); i++)
        //     {
        //         room_connections_[i] = response->connections[i].connection;
        //     }

        //     // Clear all previous markers
        //     visualization_msgs::msg::Marker delete_all;
        //     delete_all.header.frame_id = pub_frame_;
        //     delete_all.header.stamp = now();
        //     delete_all.ns = "coordinate_visualization";
        //     delete_all.id = 0;
        //     delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        //     marker_all_.markers.push_back(delete_all);

        //     marker_pub_->publish(marker_all_);
        //     pubPoints();
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to call /get_route_info service.");
        // }
        auto result = client_->async_send_request(request,
            [this](rclcpp::Client<ModuleInfo>::SharedFuture future) {
                try {
                    auto response = future.get();
                    pub_frame_ = "world";

                    room_list_.clear();
                    room_coordinate_.clear();
                    room_connections_.clear();
                    marker_all_.markers.clear();

                    for (const auto &node : response->node)
                    {
                        room_list_.push_back(node.name);
                        room_coordinate_[node.name] = node.pose;
                    }

                    for (size_t i = 0; i < response->connections.size(); i++)
                    {
                        room_connections_[i] = response->connections[i].connection;
                    }

                    // Clear all previous markers
                    visualization_msgs::msg::Marker delete_all;
                    delete_all.header.frame_id = pub_frame_;
                    delete_all.header.stamp = now();
                    delete_all.ns = "coordinate_visualization";
                    delete_all.id = 0;
                    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
                    marker_all_.markers.push_back(delete_all);

                    marker_pub_->publish(marker_all_);
                    pubPoints();
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service callback failed: %s", e.what());
                }
            });

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoutesVisualization>());
    rclcpp::shutdown();
    return 0;
}
