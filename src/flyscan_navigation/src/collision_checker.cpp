#include "flyscan_navigation/collision_checker.hpp"

#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace flyscan
{
namespace navigation
{

CollisionChecker::CollisionChecker(const CollisionConfig& config)
    : config_(config)
{
}

bool CollisionChecker::is_pose_safe(
    const geometry_msgs::msg::PoseStamped& pose,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (!obstacle_cloud)
    {
        return true;
    }

    return get_min_obstacle_distance(pose, obstacle_cloud) > config_.min_obstacle_distance;
}

bool CollisionChecker::is_path_safe(
    const nav_msgs::msg::Path& path,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (!obstacle_cloud || path.poses.empty())
    {
        return true;
    }

    for (size_t i = 0; i < path.poses.size() - 1; ++i)
    {
        auto points = discretize_path_segment(path.poses[i], path.poses[i + 1]);
        
        for (const auto& point : points)
        {
            if (check_point_collision(point, obstacle_cloud))
            {
                return false;
            }
        }
    }

    return true;
}

double CollisionChecker::get_min_obstacle_distance(
    const geometry_msgs::msg::PoseStamped& pose,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (!obstacle_cloud)
    {
        return std::numeric_limits<double>::max();
    }

    double min_distance = std::numeric_limits<double>::max();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*obstacle_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*obstacle_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*obstacle_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        double dx = *iter_x - pose.pose.position.x;
        double dy = *iter_y - pose.pose.position.y;
        double dz = *iter_z - pose.pose.position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        min_distance = std::min(min_distance, distance);
    }

    return min_distance;
}

bool CollisionChecker::emergency_stop_required(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (!obstacle_cloud)
    {
        return false;
    }

    double min_distance = get_min_obstacle_distance(current_pose, obstacle_cloud);
    return min_distance < (config_.robot_radius + config_.safety_margin);
}

void CollisionChecker::update_obstacle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
    cached_obstacle_cloud_ = cloud;
}

bool CollisionChecker::check_point_collision(
    const geometry_msgs::msg::Point& point,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacle_cloud)
{
    if (!obstacle_cloud)
    {
        return false;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*obstacle_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*obstacle_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*obstacle_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        double dx = *iter_x - point.x;
        double dy = *iter_y - point.y;
        double dz = *iter_z - point.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (distance < (config_.robot_radius + config_.safety_margin))
        {
            return true;
        }
    }

    return false;
}

std::vector<geometry_msgs::msg::Point> CollisionChecker::discretize_path_segment(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& end)
{
    std::vector<geometry_msgs::msg::Point> points;

    double dx = end.pose.position.x - start.pose.position.x;
    double dy = end.pose.position.y - start.pose.position.y;
    double dz = end.pose.position.z - start.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    int num_points = static_cast<int>(std::ceil(distance / config_.collision_check_resolution));
    num_points = std::max(1, num_points);

    for (int i = 0; i <= num_points; ++i)
    {
        double t = static_cast<double>(i) / num_points;
        
        geometry_msgs::msg::Point point;
        point.x = start.pose.position.x + t * dx;
        point.y = start.pose.position.y + t * dy;
        point.z = start.pose.position.z + t * dz;

        points.push_back(point);
    }

    return points;
}

} // namespace navigation
} // namespace flyscan