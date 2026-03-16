#include "rclcpp/rclcpp.hpp"
#include "campusrover_msgs/srv/encoder_count.hpp"


#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {
  int reduction_ratio_ = 10;
  int encoder_res_ = 10000;
  double line;
  int start_count_l, start_count_r;
  double diameter_l=0, diameter_r=0, base_length=0;

  // ros::init(argc, argv, "calibration"); //ROS1寫法
  // ros::NodeHandle n;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("calibration");

  // ros::ServiceClient client = n.serviceClient<campusrover_msgs::EncoderCount>("drive_encoder");
  auto client = node->create_client<campusrover_msgs::srv::EncoderCount>("drive_encoder");
  if (!client->wait_for_service(std::chrono::seconds(5))){
    RCLCPP_ERROR(node->get_logger(), "Service drive_encoder not available.");
    return 1;
  }


  // campusrover_msgs::EncoderCount srv;
 

  std::cout << "Calibration Mode" << std::endl;
  std::cout << "---------------------------------------------------------------" << std::endl;

  std::cout << "Wheel Radius Calibration" << std::endl;
  std::cout << "---------------------------------------------------------------" << std::endl;
  std::cout << "Type in straight line distanst: ";
  std::cin >> line;
  std::cin.ignore();

  for (int i=0; i<2; i++) {
    std::cout << i+1 << " times straight line calibration (forward)" << std::endl;
    std::cout << "Press enter to start" << std::endl;
    if (std::cin.get() == '\n') {
      auto request = std::make_shared<campusrover_msgs::srv::EncoderCount::Request>();
      auto result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
        return 1;
      }
      
      // client.call(srv);
      // start_count_l = srv.response.left_count;
      // start_count_r = srv.response.right_count;
      auto response = result_future.get();
      start_count_l = response->left_count;
      start_count_r = response->right_count;

      std::cout << "Press enter to end" << std::endl;
      if (std::cin.get() == '\n') {
        // client.call(srv);
        // diameter_l -= (reduction_ratio_*encoder_res_)*line/((srv.response.left_count-start_count_l)*M_PI);
        // diameter_r += (reduction_ratio_*encoder_res_)*line/((srv.response.right_count-start_count_r)*M_PI);
        result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
          return 1;
        }
        response = result_future.get();
        diameter_l -= (reduction_ratio_ * encoder_res_) * line / ((response->left_count - start_count_l) * M_PI);
        diameter_r += (reduction_ratio_ * encoder_res_) * line / ((response->right_count - start_count_r) * M_PI);
      }
    }
    std::cout << i+1 << " times straight line calibration (backward)" << std::endl;
    std::cout << "Press enter to start" << std::endl;
    if (std::cin.get() == '\n') {
      // client.call(srv);
      // start_count_l = srv.response.left_count;
      // start_count_r = srv.response.right_count;
      auto request = std::make_shared<campusrover_msgs::srv::EncoderCount::Request>();
      auto result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
        return 1;
      }
      auto response = result_future.get();
      start_count_l = response->left_count;
      start_count_r = response->right_count;

      std::cout << "Press enter to end" << std::endl;
      if (std::cin.get() == '\n') {
        // client.call(srv);
        // diameter_l -= (reduction_ratio_*encoder_res_)*line/((srv.response.left_count-start_count_l)*M_PI);
        // diameter_r += (reduction_ratio_*encoder_res_)*line/((srv.response.right_count-start_count_r)*M_PI);
        result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
          return 1;
        }
        response = result_future.get();
        diameter_l -= (reduction_ratio_ * encoder_res_) * line / ((response->left_count - start_count_l) * M_PI);
        diameter_r += (reduction_ratio_ * encoder_res_) * line / ((response->right_count - start_count_r) * M_PI);
      }
    }
    std::cout << "---------------------------------------------------------------" << std::endl;
  }
  diameter_l = diameter_l/4;
  diameter_r = diameter_r/4;
  std::cout << "Right wheel radius: " << diameter_r << std::endl;
  std::cout << "Left wheel radius: " << diameter_l << std::endl;
  std::cout << "---------------------------------------------------------------" << std::endl;

  std::cout << "Base Length Calibration" << std::endl;
  std::cout << "---------------------------------------------------------------" << std::endl;
  
  for (int i = 0; i < 2; i++) {
    std::cout << i + 1 << " times rotate calibration 5 turns (clockwise)" << std::endl;
    std::cout << "Press enter to start" << std::endl;
    if (std::cin.get() == '\n') {
      auto request = std::make_shared<campusrover_msgs::srv::EncoderCount::Request>();
      auto result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
        return 1;
      }
      auto response = result_future.get();
      start_count_l = response->left_count;
      start_count_r = response->right_count;

      std::cout << "Press enter to end" << std::endl;
      if (std::cin.get() == '\n') {
        result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
          return 1;
        }
        response = result_future.get();
        double outer_dis = -(response->left_count - start_count_l) * diameter_l / (encoder_res_ * reduction_ratio_);
        double inner_dis = (response->right_count - start_count_r) * diameter_r / (encoder_res_ * reduction_ratio_);
        base_length += (outer_dis - inner_dis) / (2 * 5);
      }
    }

    std::cout << i + 1 << " times rotate calibration 5 turns (anticlockwise)" << std::endl;
    std::cout << "Press enter to start" << std::endl;
    if (std::cin.get() == '\n') {
      auto request = std::make_shared<campusrover_msgs::srv::EncoderCount::Request>();
      auto result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
        return 1;
      }
      auto response = result_future.get();
      start_count_l = response->left_count;
      start_count_r = response->right_count;

      std::cout << "Press enter to end" << std::endl;
      if (std::cin.get() == '\n') {
        result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Failed to call service drive_encoder");
          return 1;
        }
        response = result_future.get();
        double outer_dis = (response->right_count - start_count_r) * diameter_r / (encoder_res_ * reduction_ratio_);
        double inner_dis = -(response->left_count - start_count_l) * diameter_l / (encoder_res_ * reduction_ratio_);
        base_length += (outer_dis - inner_dis) / (2 * 5);
      }
    }
    std::cout << "---------------------------------------------------------------" << std::endl;
  }

  base_length = base_length/4;
  std::cout << "Base length: " << base_length << std::endl;

  rclcpp::shutdown();
  return 0;
}
