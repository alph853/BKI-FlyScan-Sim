#include "flyscan_navigation/path_planner.hpp"

#include <cmath>
#include <algorithm>

namespace flyscan
{
namespace navigation
{

PathPlanner::PathPlanner(const PlannerConfig& config)
    : config_(config)
{
}

nav_msgs::msg::Path PathPlanner::plan_to_pose(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (obstacle_cloud)
    {
        update_obstacle_cloud(obstacle_cloud);
    }

    nav_msgs::msg::Path path;
    path.header = start.header;

    auto waypoints = generate_waypoints(start, goal);

    for (const auto& waypoint : waypoints)
    {
        path.poses.push_back(waypoint);
    }

    if (is_path_safe(path))
    {
        return smooth_path(path);
    }

    nav_msgs::msg::Path empty_path;
    empty_path.header = start.header;
    return empty_path;
}

nav_msgs::msg::Path PathPlanner::plan_frontier_exploration(
    const geometry_msgs::msg::PoseStamped& start,
    const flyscan_interfaces::msg::FrontierArray& frontiers,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (obstacle_cloud)
    {
        update_obstacle_cloud(obstacle_cloud);
    }

    if (frontiers.frontiers.empty())
    {
        nav_msgs::msg::Path empty_path;
        empty_path.header = start.header;
        return empty_path;
    }

    auto best_frontier = select_best_frontier(start, frontiers);
    return plan_to_pose(start, best_frontier, obstacle_cloud);
}

nav_msgs::msg::Path PathPlanner::smooth_path(const nav_msgs::msg::Path& raw_path)
{
    if (raw_path.poses.size() < 3)
    {
        return raw_path;
    }

    nav_msgs::msg::Path smoothed_path = raw_path;

    for (size_t i = 1; i < smoothed_path.poses.size() - 1; ++i)
    {
        const auto& prev = raw_path.poses[i - 1];
        const auto& curr = raw_path.poses[i];
        const auto& next = raw_path.poses[i + 1];

        const double alpha = 0.3;
        
        smoothed_path.poses[i].pose.position.x = 
            curr.pose.position.x + alpha * (prev.pose.position.x + next.pose.position.x - 2 * curr.pose.position.x);
        smoothed_path.poses[i].pose.position.y = 
            curr.pose.position.y + alpha * (prev.pose.position.y + next.pose.position.y - 2 * curr.pose.position.y);
        smoothed_path.poses[i].pose.position.z = 
            curr.pose.position.z + alpha * (prev.pose.position.z + next.pose.position.z - 2 * curr.pose.position.z);
    }

    return smoothed_path;
}

void PathPlanner::update_obstacle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
    obstacle_cloud_ = cloud;
}

geometry_msgs::msg::PoseStamped PathPlanner::select_best_frontier(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const flyscan_interfaces::msg::FrontierArray& frontiers)
{
    geometry_msgs::msg::PoseStamped best_frontier;
    double best_score = -1.0;

    for (const auto& frontier : frontiers.frontiers)
    {
        double distance = std::sqrt(
            std::pow(frontier.center.x - current_pose.pose.position.x, 2) +
            std::pow(frontier.center.y - current_pose.pose.position.y, 2) +
            std::pow(frontier.center.z - current_pose.pose.position.z, 2));

        double score = frontier.size / (1.0 + distance);

        if (score > best_score)
        {
            best_score = score;
            best_frontier.header = current_pose.header;
            best_frontier.pose.position = frontier.center;
            best_frontier.pose.orientation.w = 1.0; // Default quaternion
        }
    }

    return best_frontier;
}

std::vector<geometry_msgs::msg::PoseStamped> PathPlanner::generate_waypoints(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;

    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double dz = goal.pose.position.z - start.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    int num_waypoints = static_cast<int>(std::ceil(distance / config_.waypoint_tolerance));
    num_waypoints = std::max(2, num_waypoints);

    for (int i = 0; i <= num_waypoints; ++i)
    {
        double t = static_cast<double>(i) / num_waypoints;
        
        geometry_msgs::msg::PoseStamped waypoint = start;
        waypoint.pose.position.x = start.pose.position.x + t * dx;
        waypoint.pose.position.y = start.pose.position.y + t * dy;
        waypoint.pose.position.z = start.pose.position.z + t * dz;

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

bool PathPlanner::is_path_safe(const nav_msgs::msg::Path& path)
{
    // TODO: Implement actual collision checking with obstacle cloud
    return true;
}

} // namespace navigation
} // namespace flyscan