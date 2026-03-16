#include "path_builder/path_builder_visualizer.hpp"

#include <algorithm>

PathBuilderVisualizer::PathBuilderVisualizer(rclcpp::Clock::SharedPtr clock) : clock_(clock) {}
PathBuilderVisualizer::~PathBuilderVisualizer() = default;

void PathBuilderVisualizer::visualizeCandidates(const std::vector<bsp::PathGeometry> &candidates, size_t best_idx, std::string frame_id)
{
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    size_t h_idx = candidates.empty() ? 0 : std::min(best_idx, candidates.size() - 1);

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = clock_->now();
        m.ns = "candidates";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;
        if (i == h_idx)
        {
            m.color.r = 0.1;
            m.color.g = 1.0;
            m.color.b = 0.1;
            m.color.a = 1.0; // Green for selected
            m.scale.x = 0.08;
        }
        else
        {
            m.color.r = 1.0;
            m.color.g = 0.6;
            m.color.b = 0.2;
            m.color.a = 0.6; // Orange for others
        }
        for (const auto &p : candidates[i].pts)
        {
            geometry_msgs::msg::Point gp;
            gp.x = p.x;
            gp.y = p.y;
            m.points.push_back(gp);
        }
        arr.markers.push_back(m);
    }
    candidate_markers_ = arr;
}

void PathBuilderVisualizer::visualizeDebugPoint(const geometry_msgs::msg::Point &pt, std::string frame_id)
{
    debug_point_.header.frame_id = frame_id;
    debug_point_.header.stamp = clock_->now();
    debug_point_.point = pt;
}

visualization_msgs::msg::MarkerArray PathBuilderVisualizer::getCandidateMarkers() { return candidate_markers_; }
geometry_msgs::msg::PointStamped PathBuilderVisualizer::getDebugPointMsg() { return debug_point_; }

void PathBuilderVisualizer::VisualizeBubble(const nav_msgs::msg::Path &bubbles)
{
    visualization_msgs::msg::MarkerArray ma_circle, ma_text;
    const std::string ns_circle = "bubble_marker";
    const std::string ns_text = "bubble_marker_id";
    const size_t new_count = bubbles.poses.size();

    if (!last_frame_id_.empty() && last_frame_id_ != bubbles.header.frame_id)
    {
        for (size_t id = 0; id < last_count_; ++id)
        {
            visualization_msgs::msg::Marker del_circle, del_text;
            del_circle.header.frame_id = last_frame_id_;
            del_circle.header.stamp = clock_->now();
            del_circle.ns = ns_circle;
            del_circle.id = static_cast<int>(id);
            del_circle.action = visualization_msgs::msg::Marker::DELETE;
            ma_circle.markers.push_back(del_circle);

            del_text.header.frame_id = last_frame_id_;
            del_text.header.stamp = clock_->now();
            del_text.ns = ns_text;
            del_text.id = static_cast<int>(id);
            del_text.action = visualization_msgs::msg::Marker::DELETE;
            ma_text.markers.push_back(del_text);
        }
        last_count_ = 0;
    }

    for (size_t i = 0; i < new_count; ++i)
    {
        const auto &pose = bubbles.poses[i].pose;
        const double radius = pose.position.z;

        visualization_msgs::msg::Marker circle;
        circle.header.frame_id = bubbles.header.frame_id;
        circle.header.stamp = clock_->now();
        circle.ns = ns_circle;
        circle.id = static_cast<int>(i);
        circle.type = visualization_msgs::msg::Marker::CYLINDER;
        circle.action = visualization_msgs::msg::Marker::ADD;

        circle.pose = pose;
        circle.pose.position.z = 0.0;
        circle.pose.orientation.x = 0.0;
        circle.pose.orientation.y = 0.0;
        circle.pose.orientation.z = 0.0;
        circle.pose.orientation.w = 1.0;

        circle.scale.x = radius * 2.0;
        circle.scale.y = radius * 2.0;
        circle.scale.z = 0.05;
        circle.color.r = 1.0f;
        circle.color.g = 0.0f;
        circle.color.b = 0.0f;
        circle.color.a = 0.3f;
        circle.lifetime = rclcpp::Duration::from_seconds(0.0);
        ma_circle.markers.push_back(circle);

        visualization_msgs::msg::Marker text;
        text.header = circle.header;
        text.ns = ns_text;
        text.id = static_cast<int>(i);
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;

        text.pose = circle.pose;
        text.scale.z = 0.4;
        text.color.r = 0.0f;
        text.color.g = 0.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;
        text.text = std::to_string(i);
        text.lifetime = rclcpp::Duration::from_seconds(0.0);
        ma_text.markers.push_back(text);
    }

    if (last_count_ > new_count)
    {
        for (size_t id = new_count; id < last_count_; ++id)
        {
            visualization_msgs::msg::Marker del_circle, del_text;
            del_circle.header.frame_id = bubbles.header.frame_id;
            del_circle.header.stamp = clock_->now();
            del_circle.ns = ns_circle;
            del_circle.id = static_cast<int>(id);
            del_circle.action = visualization_msgs::msg::Marker::DELETE;
            ma_circle.markers.push_back(del_circle);

            del_text.header.frame_id = bubbles.header.frame_id;
            del_text.header.stamp = clock_->now();
            del_text.ns = ns_text;
            del_text.id = static_cast<int>(id);
            del_text.action = visualization_msgs::msg::Marker::DELETE;
            ma_text.markers.push_back(del_text);
        }
    }

    marker_array_ = ma_circle;
    marker_id_array_ = ma_text;
    last_count_ = new_count;
    last_frame_id_ = bubbles.header.frame_id;
}

visualization_msgs::msg::MarkerArray PathBuilderVisualizer::GetBubbleMarkers() const
{
    return marker_array_;
}

visualization_msgs::msg::MarkerArray PathBuilderVisualizer::GetBubbleIdMarkers() const
{
    return marker_id_array_;
}
