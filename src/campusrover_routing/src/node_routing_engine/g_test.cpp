#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <campusrover_msgs/srv/routing_path.hpp>

using namespace std::chrono_literals;

class RoutingEngineNode : public rclcpp::Node
{
public:
    RoutingEngineNode() : Node("routing_engine_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("origin", "a");
        this->declare_parameter<std::string>("destination", "a");
        this->get_parameter("origin", origin_);
        this->get_parameter("destination", destination_);

        // Publishers
        d_pub1_ = this->create_publisher<nav_msgs::msg::Path>("debug_path1", 10);
        d_pub2_ = this->create_publisher<nav_msgs::msg::Path>("debug_path2", 10);

        // Client
        d_client_ = this->create_client<campusrover_msgs::srv::RoutingPath>("/Generation_path");

        // Wait for service (optional)
        if (!d_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Service /Generation_path not available.");
        }

        // Subscriber
        d_sub_ = this->create_subscription<std_msgs::msg::String>(
            "oddddd", 10, std::bind(&RoutingEngineNode::d_callback, this, std::placeholders::_1));
    }

private:
    void d_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto request = std::make_shared<campusrover_msgs::srv::RoutingPath::Request>();
        request->origin = origin_;
        request->destination.push_back(msg->data);
        request->destination.push_back(destination_);

        auto future_result = d_client_->async_send_request(request,
            [this](rclcpp::Client<campusrover_msgs::srv::RoutingPath>::SharedFuture response){
                auto result = response.get();
                if(result.routing.size() >= 2){
                    d_pub1_->publish(result.routing[0]);
                    d_pub2_->publish(result.routing[1]);
                }else{
                    RCLCPP_WARN(this->get_logger(), "Service returned less than 2 paths.");
                }
            }
        );
    }

    std::string origin_, destination_;
    rclcpp::Client<campusrover_msgs::srv::RoutingPath>::SharedPtr d_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr d_pub1_, d_pub2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr d_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutingEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
