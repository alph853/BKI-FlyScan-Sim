/**
 * @file px4_controller.hpp
 * @author Flyscan
 * @brief PX4 controller node for autonomous UAVs with mode management
 * @version 0.1
 * @date 2025-07-11
 * @copyright Copyright (c) 2025
 */

#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <iostream>
#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <flyscan_interfaces/srv/set_control_mode.hpp>
#include <flyscan_interfaces/srv/navigate_to_pose.hpp>
#include <flyscan_interfaces/msg/teleop_command.hpp>
#include <flyscan_interfaces/msg/frontier_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/enums.hpp"

namespace flyscan {
namespace drone_controller {

using flyscan::common::OperationStatus;
using flyscan::common::NodeType;
using flyscan::common::ControlMode;
using flyscan::core::BaseNode;


enum class NavigationState {
    kIdle,             // Not navigating
    kPlanning,         // Computing safe path
    kNavigating,       // Following computed path
    kObstacleAvoidance // Avoiding obstacles
};

struct Position {
    float north_m = 0.0f;   ///< North position in meters (NED frame)
    float east_m = 0.0f;    ///< East position in meters (NED frame)  
    float down_m = 0.0f;    ///< Down position in meters (NED frame)
    float yaw_deg = 0.0f;   ///< Yaw angle in degrees
};

/**
 * @brief Advanced PX4 controller node for autonomous UAVs with mode management
 * 
 * This class provides a centralized PX4 autopilot control system that supports multiple
 * operation modes including manual control, teleoperation, and autonomous flight.
 */

class PX4Controller : public BaseNode {
public:
    explicit PX4Controller(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
        const std::string& node_name = "px4_controller",
        const NodeType& node_type = NodeType::kController,
        const std::vector<std::string>& capabilities = {"px4_controller", "mode_switching", "teleop_control"}
    );

    ~PX4Controller();

    // ============================================================================
    // Public Flight Control Interface
    // ============================================================================

    /**
     * @brief Arm the vehicle
     * @param wait_for_confirmation If true, wait for arm state confirmation
     * @param timeout_ms Timeout in milliseconds for confirmation (default 5000ms)
     * @return OperationStatus indicating success or failure
     */
    OperationStatus Arm(bool wait_for_confirmation = true, int timeout_ms = 10000);


    /**
     * @brief Takeoff to default position and enter offboard mode
     * @param wait_for_confirmation If true, wait for takeoff confirmation  
     * @param timeout_ms Timeout in milliseconds for confirmation (default 10000ms)
     * @return OperationStatus indicating success or failure
     * @note This function arms the vehicle if not already armed
     */
    OperationStatus TakeOff(bool wait_for_confirmation = true, int timeout_ms = 10000);

    /**
     * @brief Initiate landing sequence
     * @param wait_for_confirmation If true, wait for landing confirmation
     * @param timeout_ms Timeout in milliseconds for confirmation (default 15000ms)
     * @return OperationStatus indicating success or failure
     */
    OperationStatus Land(bool wait_for_confirmation = true, int timeout_ms = 15000);

    /**
     * @brief Start offboard mode with timeout
     * @param timeout_ms Timeout in milliseconds
     * @return OperationStatus indicating success or failure
     */
    OperationStatus StartOffboardMode();
    
    /**
     * @brief Stop offboard mode and return to manual control
     * @return OperationStatus indicating success or failure
     */
    OperationStatus StopOffboardMode();

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
    // Core PX4 Communication Methods
    // ============================================================================

    /**
     * @brief Callback for vehicle local position updates
     * @param msg Vehicle local position message
     */
    void VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    
    /**
     * @brief Callback for vehicle status updates
     * @param msg Vehicle status message
     */
    void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    
    /**
     * @brief Callback for vehicle land detection updates
     * @param msg Vehicle land detected message
     */
    void VehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    
    /**
     * @brief Callback for teleop command messages
     * @param msg Teleop command message
     */
    void TeleopCommandCallback(const flyscan_interfaces::msg::TeleopCommand::SharedPtr msg);

    /**
     * @brief Callback for ranked frontiers list
     * @param msg FrontierArray message with ranked frontiers
     */
    void FrontiersRankedCallback(const flyscan_interfaces::msg::FrontierArray::SharedPtr msg);
    
    /**
     * @brief Process frontier goal in autonomous mode
     * @param frontier_goal The frontier goal to process
     */
    void ProcessFrontierGoal(const geometry_msgs::msg::PoseStamped& frontier_goal);
    

    /**
     * @brief Timer callback for continuous setpoint publishing
     */
    void SetpointTimerCallback();
    
    /**ArmVehicle
     * @brief Publish offboard control mode message
     */
    void PublishOffboardControlMode();
    
    /**
     * @brief Publish trajectory setpoint based on current teleop command
     */
    void PublishTrajectorySetpoint();
    
    /**
     * @brief Publish vehicle command to PX4
     * @param command Command ID
     * @param param1 First parameter
     * @param param2 Second parameter
     */
    void PublishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

    /**
     * @brief Send arm/disarm command to the vehicle
     * @param arm_vehicle If true, arm the vehicle; if false, disarm
     */
    void SendArmDisarmCommand(bool arm_vehicle);

    /**
     * @brief Transform body frame movement to world frame using current yaw
     * @param body_x Body frame X movement (forward/backward)
     * @param body_y Body frame Y movement (left/right)
     * @param current_yaw_rad Current yaw angle in radians
     * @param world_x Output world frame X movement (north/south)
     * @param world_y Output world frame Y movement (east/west)
     */
    void TransformBodyToWorld(float body_x, float body_y, float current_yaw_rad, float& world_x, float& world_y);

    /**
     * @brief Transform frontier position considering camera field of view
     * @param frontier_pos Frontier position from exploration
     * @param optimal_pos Output optimal drone position for camera coverage
     * @param optimal_yaw Output optimal yaw angle to face frontier
     */
    void CalculateForwardCameraDirection(const geometry_msgs::msg::Point& frontier_pos,
                                        geometry_msgs::msg::Point& forward_pos, float& forward_yaw);
                                        
    void TransformPointCloudFromCamera(const pcl::PointCloud<pcl::PointXYZ>::Ptr& camera_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& world_cloud);

    /**
     * @brief Get topic prefix for multi-drone support
     * @return Empty string for drone 1, "/px4_{drone_id}" for others
     */
    std::string GetTopicPrefix() const;

    /**
     * @brief Get controller-specific topic name for multi-drone support
     * @param topic_suffix The topic suffix (e.g., "/teleop_command")
     * @return Topic name with appropriate controller prefix
     */
    std::string GetControllerTopicName(const std::string& topic_suffix) const;

    /**
     * @brief Get camera-specific topic name for multi-drone support
     * @param topic_name The full topic name (e.g., "/camera/depth/points")
     * @return Topic name with appropriate drone prefix
     */
    std::string GetCameraTopicName(const std::string& topic_name) const;

    // ============================================================================
    // Safe Navigation Methods
    // ============================================================================
    
    /**
     * @brief Check if path to target is obstacle-free using point cloud
     * @param start Start position
     * @param end Target position
     * @return True if path is clear
     */
    bool IsPathClearPointCloud(const geometry_msgs::msg::Point& start, 
                              const geometry_msgs::msg::Point& end);
    
    /**
     * @brief Calculate next navigation step towards target
     * @param current_pos Current position
     * @param target_pos Target position
     * @return Next step position
     */
    geometry_msgs::msg::Point CalculateNextNavigationStep(const geometry_msgs::msg::Point& current_pos,
                                                         const geometry_msgs::msg::Point& target_pos);
    
    /**
     * @brief Pick next frontier from ranked list when current path is blocked
     * @return True if a new frontier was selected, false if no alternatives available
     */
    bool SelectNextFrontierFromList();
    
    /**
     * @brief Execute obstacle avoidance maneuver
     * @param obstacle_direction Direction of detected obstacle (radians)
     * @return New safe direction to navigate
     */
    double ComputeAvoidanceDirection(double obstacle_direction);
    
    /**
     * @brief Timer callback for point cloud-based navigation execution
     */
    void PointCloudNavigationTimerCallback();
    
        
    /**
     * @brief Callback for obstacle/map data
     */
    void ObstacleMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Callback for point cloud obstacle data
     */
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // ============================================================================
    // Mode Management Methods
    // ============================================================================
    
         /**
      * @brief Service callback for mode switching
      * @param request Mode switch request
      * @param response Mode switch response
      */
     void HandleSetControlModeService(
         const std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Request> request,
         std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Response> response);
    
    /**
     * @brief Switch to specified control mode
     * @param new_mode Target control mode
     * @return True if mode switch successful
     */
    OperationStatus SwitchToMode(ControlMode new_mode);
    
    /**
     * @brief Enter manual mode
     * @return True if successful
     */
    OperationStatus EnterManualMode();
    
    /**
     * @brief Enter teleop mode with interactive keyboard control
     * @return True if successful
     */
    OperationStatus EnterTeleopMode();
    
    /**
     * @brief Enter autonomous mode for exploration
     * @return True if successful
     */
    OperationStatus EnterAutonomousMode();
    
    /**
     * @brief Enter mission mode for waypoint execution
     * @return True if successful
     */
    OperationStatus EnterMissionMode();
    
    // ============================================================================
    // ROS2 Publishers and Subscribers
    // ============================================================================
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
    rclcpp::Subscription<flyscan_interfaces::msg::TeleopCommand>::SharedPtr teleop_command_sub_;
    rclcpp::Subscription<flyscan_interfaces::msg::FrontierArray>::SharedPtr frontiers_ranked_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

    rclcpp::Service<flyscan_interfaces::srv::SetControlMode>::SharedPtr set_control_mode_service_;
    
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    rclcpp::TimerBase::SharedPtr safe_navigation_timer_;

    // Timer-based wait state checking
    rclcpp::TimerBase::SharedPtr arm_check_timer_;
    rclcpp::TimerBase::SharedPtr inair_check_timer_;
    rclcpp::TimerBase::SharedPtr offboard_check_timer_;
    
    // ============================================================================
    // State Management
    // ============================================================================
    
    /// Exit flag for main loops
    std::atomic<bool> should_exit_{false};
    std::atomic<bool> armed_{false};
    std::atomic<bool> in_air_{false};
    std::atomic<ControlMode> current_mode_{ControlMode::kManual};
    
    
    // Safe navigation state
    std::atomic<NavigationState> navigation_state_{NavigationState::kIdle};
    geometry_msgs::msg::PoseStamped current_navigation_target_;
    std::vector<geometry_msgs::msg::Point> current_waypoints_;
    size_t current_waypoint_index_{0};
    nav_msgs::msg::OccupancyGrid::SharedPtr obstacle_map_;
    double navigation_max_velocity_{1.0};
    double navigation_safety_margin_{1.5};
    int navigation_attempt_counter_{0};
    std::mutex navigation_mutex_;
    
    // Point cloud navigation state
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    std::mutex point_cloud_mutex_;
    
    // ============================================================================
    // Thread-Safe
    // ============================================================================
    
    Position current_position_setpoint_;
    flyscan_interfaces::msg::TeleopCommand current_teleop_command_;
    geometry_msgs::msg::PoseStamped current_exploration_goal_;
    flyscan_interfaces::msg::FrontierArray current_frontiers_ranked_;
    size_t current_frontier_index_{0};  // Index of currently targeted frontier
    std::mutex position_setpoint_mutex_;
    std::mutex teleop_command_mutex_;
    std::mutex exploration_goal_mutex_;
    std::mutex frontiers_mutex_;
    
    px4_msgs::msg::VehicleLocalPosition current_position_;
    px4_msgs::msg::VehicleStatus current_status_;
    mutable std::mutex position_mutex_;
    mutable std::mutex status_mutex_;
    std::mutex mode_mutex_;
    
    // ============================================================================
    // ROS Parameters (cached from parameter server)
    // ============================================================================
    
    int setpoint_rate_ms_;
    float takeoff_altitude_;
    double min_flight_altitude_;      // Minimum altitude to maintain (NED: negative is up)
    float teleop_position_step_;
    float yaw_step_;
    ControlMode initial_control_mode_;
    
    // Camera parameters for frontier exploration
    double camera_fov_horizontal_;       // Horizontal field of view in degrees
    double optimal_camera_distance_;     // Optimal distance to maintain from frontier
    double camera_offset_x_;             // Camera forward offset from drone center (meters)
    double camera_offset_z_;             // Camera down offset from drone center (meters, NED)
    
    
    // Safe navigation parameters
    double waypoint_tolerance_;         // Distance tolerance to consider waypoint reached
    double obstacle_detection_range_;   // Range for obstacle detection (m)
    double min_obstacle_distance_;      // Minimum distance to maintain from obstacles (m)
    double path_smoothing_factor_;      // Factor for path smoothing (0.0 to 1.0)
    
    // Point cloud navigation parameters
    double point_cloud_downsample_leaf_size_;  // Voxel grid leaf size for downsampling
    double obstacle_check_ahead_distance_;     // Distance to check ahead for obstacles
    double navigation_step_size_;              // Step size for gradual navigation
    int max_navigation_attempts_;              // Maximum attempts when path is blocked
    double autonomous_fixed_height_;           // Fixed height to maintain in autonomous mode
    
    // Multi-drone support
    int drone_id_;                            // Drone ID for multi-drone support (1 = first drone)
    double yaw_tolerance_;                     // Yaw tolerance in degrees for navigation
    
    // Enhanced obstacle avoidance for small drones
    double drone_radius_;                      // Physical radius of the drone
    int min_obstacle_points_;                  // Minimum points needed to consider as obstacle

};

} // namespace drone_controller
} // namespace flyscan


RCLCPP_COMPONENTS_REGISTER_NODE(flyscan::drone_controller::PX4Controller)