#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

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
    double size;
    double distance_to_robot;
    double exploration_value;
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
    
    std::vector<Frontier> detectFrontiers(const nav_msgs::msg::OccupancyGrid& map);
    std::vector<cv::Point> findFrontierCells(const cv::Mat& map_image);
    std::vector<Frontier> clusterFrontierCells(const std::vector<cv::Point>& frontier_cells,
                                               const nav_msgs::msg::OccupancyGrid& map);
    Frontier selectBestFrontier(const std::vector<Frontier>& frontiers);
    void publishExplorationGoal(const Frontier& frontier);
    void publishFrontierVisualization(const std::vector<Frontier>& frontiers);
    
    bool isValidCell(int x, int y, const cv::Mat& map) const;
    bool isFrontierCell(int x, int y, const cv::Mat& map) const;
    double calculateExplorationValue(const Frontier& frontier) const;
    geometry_msgs::msg::Point mapToWorld(int map_x, int map_y, 
                                         const nav_msgs::msg::OccupancyGrid& map) const;
    cv::Point worldToMap(double world_x, double world_y, 
                         const nav_msgs::msg::OccupancyGrid& map) const;

    // ============================================================================
    // ROS2 Publishers and Subscribers
    // ============================================================================
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_viz_publisher_;

    rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedPtr set_control_mode_client_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    
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
    
    // ============================================================================
    // ROS Parameters (cached from parameter server)
    // ============================================================================
    
    double exploration_radius_;
    double min_frontier_size_;
    double frontier_cluster_distance_;
    double exploration_rate_;
};

} // namespace exploration
} // namespace flyscan