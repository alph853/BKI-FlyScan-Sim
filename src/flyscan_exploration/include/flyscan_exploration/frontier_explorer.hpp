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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/enums.hpp"
#include <flyscan_interfaces/srv/set_control_mode.hpp>
#include <flyscan_interfaces/msg/frontier_array.hpp>

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
    std::vector<cv::Point> pixel_points;
};

struct DBSCANPoint {
    cv::Point position;
    int cluster_id;
    bool visited;
    bool is_core;
    
    DBSCANPoint() : cluster_id(-1), visited(false), is_core(false) {}
    explicit DBSCANPoint(const cv::Point& pos) : position(pos), cluster_id(-1), visited(false), is_core(false) {}
};

struct FrontierHistory {
    geometry_msgs::msg::Point position;
    rclcpp::Time timestamp;
    
    FrontierHistory() = default;
    FrontierHistory(const geometry_msgs::msg::Point& pos, const rclcpp::Time& time)
        : position(pos), timestamp(time) {}
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
    void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    
    /**
     * @brief Timer callback for exploration algorithm execution
     */
    void ExplorationTimerCallback();
    
    // ============================================================================
    // Frontier Detection and Analysis
    // ============================================================================
    
    /**
     * @brief Optimized frontier detection using DFS flood-fill
     * @param grid Occupancy grid to analyze
     * @return Vector of detected frontiers using DBSCAN clustering
     * @details Uses DFS flood-fill for efficiency
     */
    std::vector<Frontier> DetectFrontiers(const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief DFS flood-fill to find connected frontier regions
     * @param grid Occupancy grid
     * @param start_x Starting x coordinate
     * @param start_y Starting y coordinate
     * @param visited Visited mask
     * @param frontier_points Output vector of frontier points
     */
    void DfsFloodFill(const nav_msgs::msg::OccupancyGrid& grid, int start_x, int start_y, 
                      std::vector<std::vector<bool>>& visited, std::vector<cv::Point>& frontier_points);
    
    /**
     * @brief DBSCAN clustering algorithm for frontier points
     * @param points Frontier points to cluster
     * @param eps Distance threshold for clustering
     * @param min_points Minimum points required for a cluster
     * @return Vector of frontier clusters
     */
    std::vector<Frontier> DbscanClustering(const std::vector<cv::Point>& points, 
                                         double eps, int min_points,
                                         const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief Calculate utility function for frontier selection
     * @param frontier Frontier to evaluate
     * @param grid Occupancy grid for information gain calculation
     * @return Utility score
     */
    double CalculateUtility(Frontier& frontier, const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief Ranks frontiers by utility in decreasing order
     * @param frontiers Vector of candidate frontiers
     * @param grid Occupancy grid for utility calculation
     * @return Vector of frontiers ranked by decreasing utility
     */
    std::vector<Frontier> RankFrontiersByUtility(std::vector<Frontier>& frontiers, const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief Publishes ranked frontiers for px4_controller consumption
     * @param ranked_frontiers Vector of frontiers in decreasing utility order
     */
    void PublishRankedFrontiers(const std::vector<Frontier>& ranked_frontiers);
    
    
    /**
     * @brief Get neighbors within eps distance for DBSCAN
     * @param points All frontier points
     * @param point_idx Index of point to find neighbors for
     * @param eps Distance threshold
     * @return Vector of neighbor indices
     */
    std::vector<size_t> GetNeighbors(const std::vector<cv::Point>& points, size_t point_idx, double eps);
    
    // IsInCameraFOV function removed - safe navigation now handles optimal positioning
    
    
    /**
     * @brief Detect if new frontier is similar to recent frontiers (stuck condition)
     * @param new_frontier The new frontier position to check
     * @return True if stuck condition detected (similar frontiers)
     */
    bool DetectSimilarFrontiers(const geometry_msgs::msg::Point& new_frontier);
    
    /**
     * @brief Updates robot position from TF transforms
     * @return Success status
     */
    bool UpdateRobotPosition();
    

    // ============================================================================
    // ROS2 Publishers and Subscribers
    // ============================================================================
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    
    rclcpp::Publisher<flyscan_interfaces::msg::FrontierArray>::SharedPtr frontiers_publisher_;

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
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    
    // ============================================================================
    // State Management
    // ============================================================================
    
    geometry_msgs::msg::Point robot_position_;
    bool exploration_active_;
    bool exploration_complete_;
    
    // Frontier similarity tracking for stuck detection
    std::vector<FrontierHistory> frontier_history_;
    
    // ============================================================================
    // ROS Parameters (cached from parameter server)
    // ============================================================================
    
    double exploration_radius_;
    double exploration_rate_;
    double max_frontier_distance_;
    std::string robot_frame_;
    std::string map_frame_;
    
    // New parameters for optimization
    double min_frontier_size_;          // Minimum frontier cluster size
    double dbscan_eps_;                 // DBSCAN epsilon parameter
    int dbscan_min_points_;            // DBSCAN minimum points
    double information_radius_;         // Radius for information gain calculation
    double distance_weight_;            // Weight for distance in utility function
    double size_weight_;                // Weight for size in utility function
    double information_weight_;         // Weight for information gain
    double min_utility_threshold_;      // Minimum utility threshold
    bool prefer_far_frontiers_;         // Whether to prefer far or near frontiers
    double min_exploration_distance_;   // Minimum distance for frontier consideration
    double frontier_spread_factor_;     // Exponential factor for distance preference
    
    // Similarity detection parameters
    double similarity_threshold_;        // Distance threshold for similar frontiers
    double similarity_time_threshold_;   // Time threshold for stuck detection (seconds)
    
    // Camera cone filtering parameters removed - safe navigation handles positioning
};

} // namespace exploration
} // namespace flyscan