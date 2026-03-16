#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <string>
#include <vector>
#include "path_builder_core.hpp"

class PathBuilderVisualizer
{
public:
    // PathBuilderVisualizer();
    explicit PathBuilderVisualizer(rclcpp::Clock::SharedPtr clock);
    ~PathBuilderVisualizer();

    void visualizeCandidates(const std::vector<bsp::PathGeometry> &candidates, size_t best_idx, std::string frame_id);
    void visualizeDebugPoint(const geometry_msgs::msg::Point &pt, std::string frame_id);

    visualization_msgs::msg::MarkerArray getCandidateMarkers();
    geometry_msgs::msg::PointStamped getDebugPointMsg();

    // Bubble
    void VisualizeBubble(const nav_msgs::msg::Path &bubbles);
    visualization_msgs::msg::MarkerArray GetBubbleMarkers() const;
    visualization_msgs::msg::MarkerArray GetBubbleIdMarkers() const;

private:
    rclcpp::Clock::SharedPtr clock_;
    visualization_msgs::msg::MarkerArray candidate_markers_;
    geometry_msgs::msg::PointStamped debug_point_;

    visualization_msgs::msg::MarkerArray marker_array_;
    visualization_msgs::msg::MarkerArray marker_id_array_;
    size_t last_count_{0};
    std::string last_frame_id_;
};
