/**
 * @file navigator.hpp
 * @brief Navigation node for autonomous path planning and trajectory generation
 *
 * This file contains the Navigator class which provides path planning and trajectory
 * generation capabilities for autonomous navigation, including frontier exploration
 * and point-to-point navigation.
 *
 * @author UAV team@/flyscan
 * @date 2025/01/14
 */

#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "flyscan_interfaces/msg/frontier_array.hpp"
#include "flyscan_interfaces/srv/set_control_mode.hpp"
#include "flyscan_interfaces/action/follow_path.hpp"
#include "flyscan_interfaces/action/navigate_to_pose3_d.hpp"

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/enums.hpp"
#include "flyscan_navigation/path_planner.hpp"
#include "flyscan_navigation/collision_checker.hpp"

namespace flyscan {
namespace navigation {

using flyscan::common::OperationStatus;
using flyscan::common::NodeType;
using flyscan::common::ControlMode;
using flyscan::core::BaseNode;

/**
 * @brief Navigation node for autonomous path planning and trajectory generation
 * 
 * This class provides autonomous navigation capabilities including path planning,
 * trajectory generation, and obstacle avoidance for UAV exploration missions.
 */
class Navigator : public BaseNode {
public:
    explicit Navigator(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
        const std::string& node_name = "navigator",
        const NodeType& node_type = NodeType::kNavigation,
        const std::vector<std::string>& capabilities = {"path_planning", "trajectory_generation", "obstacle_avoidance"}
    );

    ~Navigator();

private:
    using FollowPath = flyscan_interfaces::action::FollowPath;
    using NavigateToPose3D = flyscan_interfaces::action::NavigateToPose3D;
    using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
    using GoalHandleNavigateToPose3D = rclcpp_action::ServerGoalHandle<NavigateToPose3D>;

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
    // Core Navigation Methods
    // ============================================================================
    
    /**
     * @brief Configure navigation components (called by HandleConfigure)
     */
    void configure();
    
    /**
     * @brief Activate navigation components (called by HandleActivate)
     */
    void activate();
    
    /**
     * @brief Deactivate navigation components (called by HandleDeactivate)
     */
    void deactivate();
    
    /**
     * @brief Cleanup navigation resources (called by HandleCleanup)
     */
    void cleanup();

    /**
     * @brief Callback for frontier updates from exploration
     * @param msg Frontier array message
     */
    void frontiers_callback(const flyscan_interfaces::msg::FrontierArray::SharedPtr msg);
    
    /**
     * @brief Callback for point cloud updates from sensors
     * @param msg Point cloud message
     */
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ============================================================================
    // Action Server Methods
    // ============================================================================

    rclcpp_action::GoalResponse handle_follow_path_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowPath::Goal> goal);
    
    rclcpp_action::CancelResponse handle_follow_path_cancel(
        const std::shared_ptr<GoalHandleFollowPath> goal_handle);
    
    void handle_follow_path_accepted(
        const std::shared_ptr<GoalHandleFollowPath> goal_handle);

    rclcpp_action::GoalResponse handle_navigate_to_pose_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPose3D::Goal> goal);
    
    rclcpp_action::CancelResponse handle_navigate_to_pose_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose3D> goal_handle);
    
    void handle_navigate_to_pose_accepted(
        const std::shared_ptr<GoalHandleNavigateToPose3D> goal_handle);

    void execute_follow_path(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
    void execute_navigate_to_pose(const std::shared_ptr<GoalHandleNavigateToPose3D> goal_handle);

    /**
     * @brief Request control mode change from controller
     * @param mode Control mode to request
     */
    void request_control_mode(const std::string& mode);
    
    /**
     * @brief Publish trajectory to controller
     * @param path Path to publish
     */
    void publish_trajectory_to_controller(const nav_msgs::msg::Path& path);

    /**
     * @brief Get topic prefix for multi-drone support
     * @return Topic prefix string
     */
    std::string GetTopicPrefix() const;

    /**
     * @brief Get navigator-specific topic name for multi-drone support
     * @param topic_suffix The topic suffix
     * @return Topic name with appropriate navigator prefix
     */
    std::string GetNavigatorTopicName(const std::string& topic_suffix) const;

    // ============================================================================
    // ROS2 Publishers, Subscribers, and Services
    // ============================================================================

    rclcpp::Subscription<flyscan_interfaces::msg::FrontierArray>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr heartbeat_pub_;

    rclcpp::Client<flyscan_interfaces::srv::SetControlMode>::SharedPtr control_mode_client_;

    rclcpp_action::Server<FollowPath>::SharedPtr follow_path_server_;
    rclcpp_action::Server<NavigateToPose3D>::SharedPtr navigate_to_pose_server_;

    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<CollisionChecker> collision_checker_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    
    // ============================================================================
    // State Management
    // ============================================================================
    
    /// Exit flag for main loops
    std::atomic<bool> should_exit_{false};
    std::atomic<bool> autonomous_mode_active_{false};
    
    // Navigation state
    nav_msgs::msg::Path current_trajectory_;
    flyscan_interfaces::msg::FrontierArray::SharedPtr latest_frontiers_;
    std::mutex trajectory_mutex_;
    std::mutex frontiers_mutex_;
    
    // ============================================================================
    // ROS Parameters (cached from parameter server)
    // ============================================================================
    
    int drone_id_;
    double max_velocity_;
    double max_acceleration_;
    double waypoint_tolerance_;
    double planning_horizon_;
    double safety_margin_;
    double robot_radius_;
    double min_obstacle_distance_;
};

} // namespace navigation
} // namespace flyscan

RCLCPP_COMPONENTS_REGISTER_NODE(flyscan::navigation::Navigator)