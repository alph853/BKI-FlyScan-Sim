#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace flyscan
{
namespace navigation
{

struct CollisionConfig
{
    double robot_radius = 0.5;
    double safety_margin = 0.3;
    double min_obstacle_distance = 1.0;
    double collision_check_resolution = 0.1;
};

class CollisionChecker
{
public:
    CollisionChecker(const CollisionConfig& config = CollisionConfig{});

    bool is_pose_safe(
        const geometry_msgs::msg::PoseStamped& pose,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud);

    bool is_path_safe(
        const nav_msgs::msg::Path& path,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud);

    double get_min_obstacle_distance(
        const geometry_msgs::msg::PoseStamped& pose,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud);

    bool emergency_stop_required(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud);

    void update_obstacle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);

private:
    bool check_point_collision(
        const geometry_msgs::msg::Point& point,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud);

    std::vector<geometry_msgs::msg::Point> discretize_path_segment(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& end);

    CollisionConfig config_;
    sensor_msgs::msg::PointCloud2::SharedPtr cached_obstacle_cloud_;
};

} // namespace navigation
} // namespace flyscan