#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/enums.hpp"
#include <flyscan_interfaces/srv/set_control_mode.hpp>

#include <vector>
#include <memory>
#include <mutex>

namespace flyscan {
namespace exploration {

using flyscan::common::OperationStatus;
using flyscan::common::NodeType;
using flyscan::common::ControlMode;
using flyscan::core::BaseNode;

struct Frontier {
    geometry_msgs::msg::Point center;
    std::vector<geometry_msgs::msg::Point> points;
    Eigen::Vector2d centroid;
    double size;
    double distance_to_robot;
    double exploration_value;
    int cluster_id;
    double utility_score;
    double distance_score;
    double size_score;
    double information_gain;
};

class FrontierExplorer : public BaseNode {
public:
    explicit FrontierExplorer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
        const std::string& node_name = "frontier_explorer",
        const NodeType& node_type = NodeType::kExploration,
        const std::vector<std::string>& capabilities = {"frontier_detection", "autonomous_exploration", "mapping"}
    );
    
    ~FrontierExplorer();

protected:
    // ============================================================================
    // Lifecycle Management (BaseNode overrides)
    // ============================================================================
    
    OperationStatus HandleConfigure() override;
    OperationStatus HandleActivate() override;
    OperationStatus HandleDeactivate() override;
    OperationStatus HandleCleanup() override;
    OperationStatus HandleShutdown() override;
    OperationStatus HandleError() override;

private:
    // ============================================================================
    // Core Exploration Methods
    // ============================================================================
    
    /**
     * @brief Callback for map data updates
     * @param msg Occupancy grid message
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Callback for odometry updates
     * @param msg Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Timer callback for exploration algorithm execution
     */
    void explorationTimerCallback();
    
    // ============================================================================
    // Frontier Detection and Analysis
    // ============================================================================
    
    /**
     * @brief Detects frontiers using simple grid-based approach
     * @param grid Occupancy grid to analyze
     * @return Vector of detected frontiers
     * @details Finds free cells adjacent to unknown cells, then groups nearby ones
     */
    std::vector<Frontier> DetectSimpleFrontiers(const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief Groups nearby frontier cells into clusters
     * @param raw_frontiers Vector of individual frontier cells
     * @param group_distance Maximum distance to group frontiers together
     * @return Vector of grouped frontier clusters
     */
    std::vector<Frontier> GroupNearbyFrontiers(const std::vector<Frontier>& raw_frontiers, double group_distance);
    
    /**
     * @brief Selects closest frontier to robot
     * @param frontiers Vector of candidate frontiers
     * @return Shared pointer to closest frontier (nullptr if empty)
     */
    std::shared_ptr<Frontier> SelectClosestFrontier(const std::vector<Frontier>& frontiers);
    
    void publishExplorationGoal(const Frontier& frontier);
    void publishFrontierVisualization(const std::vector<Frontier>& frontiers);
    
    /**
     * @brief Updates robot position from TF transforms
     * @return Success status
     */
    bool UpdateRobotPosition();
    

    // ============================================================================
    // ROS2 Publishers and Subscribers
    // ============================================================================
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_viz_publisher_;

    rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedPtr set_control_mode_client_;
    
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    
    // ============================================================================
    // Transform and State Management
    // ============================================================================
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ============================================================================
    // Thread-Safe Data Access
    // ============================================================================
    
    std::mutex map_mutex_;
    std::mutex odom_mutex_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    
    // ============================================================================
    // State Management
    // ============================================================================
    
    geometry_msgs::msg::Point robot_position_;
    bool exploration_active_;
    bool exploration_complete_;
    
    // ============================================================================
    // ROS Parameters (cached from parameter server)
    // ============================================================================
    
    double exploration_radius_;
    double exploration_rate_;
    double max_frontier_distance_;
    std::string robot_frame_;
    std::string map_frame_;
};

} // namespace exploration
} // namespace flyscan