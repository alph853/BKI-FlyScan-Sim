#pragma once

#include <vector>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "flyscan_interfaces/msg/frontier_array.hpp"

namespace flyscan 
{
namespace navigation
{

struct PlannerConfig
{
    double max_velocity = 2.0;
    double max_acceleration = 1.0;
    double waypoint_tolerance = 0.5;
    double planning_horizon = 10.0;
    double safety_margin = 1.0;
};

class PathPlanner
{
public:
    PathPlanner(const PlannerConfig& config = PlannerConfig{});

    nav_msgs::msg::Path plan_to_pose(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud = nullptr);

    nav_msgs::msg::Path plan_frontier_exploration(
        const geometry_msgs::msg::PoseStamped& start,
        const flyscan_interfaces::msg::FrontierArray& frontiers,
        const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud = nullptr);

    nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path& raw_path);

    void update_obstacle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);

private:
    geometry_msgs::msg::PoseStamped select_best_frontier(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const flyscan_interfaces::msg::FrontierArray& frontiers);

    std::vector<geometry_msgs::msg::PoseStamped> generate_waypoints(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal);

    bool is_path_safe(const nav_msgs::msg::Path& path);

    PlannerConfig config_;
    sensor_msgs::msg::PointCloud2::SharedPtr obstacle_cloud_;
};

} // namespace navigation
} // namespace flyscan