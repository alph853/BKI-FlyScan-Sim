/**
 * @file px4_controller.cpp
 * @brief Advanced PX4 Controller implementation with unified data flow
 *
 * This file contains the implementation of the PX4Controller class with
 * centralized mode management, lifecycle state handling, and unified
 * control flow with clear priority system:
 * 
 * PRIORITY SYSTEM (highest to lowest):
 * 1. Teleop commands (manual override, highest priority)
 * 2. Recovery maneuvers (autonomous safety system)
 * 3. Frontier navigation (autonomous exploration)
 *
 * @author UAV team@/flyscan
 * @date 2025/01/14
 */

#include <chrono>
#include <functional>
#include <csignal>
#include <cmath>
#include <future>
#include <complex>
#include <algorithm>

#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "flyscan_drone_controller/px4_controller.hpp"
#include "flyscan_common/sigint_handler.hpp"
#include "flyscan_interfaces/msg/frontier_array.hpp"

namespace flyscan {
namespace drone_controller {

PX4Controller::PX4Controller(const rclcpp::NodeOptions & options,
                             const std::string& node_name,
                             const NodeType& node_type,
                             const std::vector<std::string>& capabilities
                             )
    : flyscan::core::BaseNode(options, node_name, node_type, capabilities) {

    RCLCPP_INFO(this->get_logger(), "Initializing Advanced PX4 Controller Node: %s", node_name.c_str());
    
    // Declare ROS parameters with default values
    if (!this->has_parameter("setpoint_rate_ms")) {
        this->declare_parameter("setpoint_rate_ms", 50);
    }
    if (!this->has_parameter("takeoff_altitude")) {
        this->declare_parameter("takeoff_altitude", -1.5);  // NED: negative is up
    }
    if (!this->has_parameter("min_flight_altitude")) {
        this->declare_parameter("min_flight_altitude", -1.0);  // NED: minimum 1m above ground
    }
    if (!this->has_parameter("teleop_position_step")) {
        this->declare_parameter("teleop_position_step", 0.5);
    }
    if (!this->has_parameter("yaw_step")) {
        this->declare_parameter("yaw_step", 15.0);
    }
    if (!this->has_parameter("initial_control_mode")) {
        this->declare_parameter("initial_control_mode", static_cast<int>(ControlMode::kManual));
    }
    if (!this->has_parameter("camera_fov_horizontal")) {
        this->declare_parameter("camera_fov_horizontal", 90.0);
    }
    if (!this->has_parameter("optimal_camera_distance")) {
        this->declare_parameter("optimal_camera_distance", 3.0);
    }
    if (!this->has_parameter("camera_offset_x")) {
        this->declare_parameter("camera_offset_x", 0.3);  // Camera forward offset from drone center
    }
    if (!this->has_parameter("camera_offset_z")) {
        this->declare_parameter("camera_offset_z", -0.05); // Camera down offset from drone center (NED: negative is down)
    }

    // Safe navigation parameters
    if (!this->has_parameter("waypoint_tolerance")) {
        this->declare_parameter("waypoint_tolerance", 0.3);
    }
    if (!this->has_parameter("obstacle_detection_range")) {
        this->declare_parameter("obstacle_detection_range", 5.0);
    }
    if (!this->has_parameter("min_obstacle_distance")) {
        this->declare_parameter("min_obstacle_distance", 0.8);  // Reduced for small drone
    }
    if (!this->has_parameter("path_smoothing_factor")) {
        this->declare_parameter("path_smoothing_factor", 0.3);
    }
    
    // Point cloud navigation parameters
    if (!this->has_parameter("point_cloud_downsample_leaf_size")) {
        this->declare_parameter("point_cloud_downsample_leaf_size", 0.15);  // Coarser sampling for small obstacles
    }
    if (!this->has_parameter("obstacle_check_ahead_distance")) {
        this->declare_parameter("obstacle_check_ahead_distance", 1.2);  // Reduced for small drone
    }
    if (!this->has_parameter("navigation_step_size")) {
        this->declare_parameter("navigation_step_size", 1.0);  // Smaller steps for tight navigation
    }
    if (!this->has_parameter("drone_radius")) {
        this->declare_parameter("drone_radius", 0.25);  // Small drone physical radius
    }
    if (!this->has_parameter("min_obstacle_points")) {
        this->declare_parameter("min_obstacle_points", 3);  // Minimum points to consider obstacle
    }
    if (!this->has_parameter("max_navigation_attempts")) {
        this->declare_parameter("max_navigation_attempts", 20);
    }
    if (!this->has_parameter("yaw_tolerance")) {
        this->declare_parameter("yaw_tolerance", 5.0);  // Yaw tolerance in degrees
    }
    if (!this->has_parameter("autonomous_fixed_height")) {
        this->declare_parameter("autonomous_fixed_height", -0.8);  // NED: negative is up
    }
    
    // Multi-drone support parameter
    if (!this->has_parameter("drone_id")) {
        this->declare_parameter("drone_id", 0);
    }

    RCLCPP_INFO(this->get_logger(), "Starting with parameter-based configuration - use set_control_mode service to switch modes");
}

PX4Controller::~PX4Controller() {
    RCLCPP_INFO(this->get_logger(), "PX4Controller destructor called");

    should_exit_ = true;

    RCLCPP_INFO(this->get_logger(), "PX4Controller shutdown complete");
}

// ============================================================================
// Lifecycle Management Implementation
// ============================================================================

OperationStatus PX4Controller::HandleConfigure() {
    RCLCPP_INFO(this->get_logger(), "Configuring PX4 Controller...");
    
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    try {
        // Cache ROS parameters
        setpoint_rate_ms_ = this->get_parameter("setpoint_rate_ms").as_int();
        takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
        min_flight_altitude_ = this->get_parameter("min_flight_altitude").as_double();
        teleop_position_step_ = this->get_parameter("teleop_position_step").as_double();
        yaw_step_ = this->get_parameter("yaw_step").as_double();
        initial_control_mode_ = static_cast<ControlMode>(this->get_parameter("initial_control_mode").as_int());
        camera_fov_horizontal_ = this->get_parameter("camera_fov_horizontal").as_double();
        optimal_camera_distance_ = this->get_parameter("optimal_camera_distance").as_double();
        camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
        camera_offset_z_ = this->get_parameter("camera_offset_z").as_double();
        
        
        // Cache safe navigation parameters
        waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
        obstacle_detection_range_ = this->get_parameter("obstacle_detection_range").as_double();
        min_obstacle_distance_ = this->get_parameter("min_obstacle_distance").as_double();
        path_smoothing_factor_ = this->get_parameter("path_smoothing_factor").as_double();
        
        // Cache point cloud navigation parameters
        point_cloud_downsample_leaf_size_ = this->get_parameter("point_cloud_downsample_leaf_size").as_double();
        obstacle_check_ahead_distance_ = this->get_parameter("obstacle_check_ahead_distance").as_double();
        navigation_step_size_ = this->get_parameter("navigation_step_size").as_double();
        max_navigation_attempts_ = this->get_parameter("max_navigation_attempts").as_int();
        autonomous_fixed_height_ = this->get_parameter("autonomous_fixed_height").as_double();
        yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
        drone_radius_ = this->get_parameter("drone_radius").as_double();
        min_obstacle_points_ = this->get_parameter("min_obstacle_points").as_int();
        
        // Cache drone_id for multi-drone support
        drone_id_ = this->get_parameter("drone_id").as_int();

        RCLCPP_INFO(this->get_logger(), "Cached parameters: drone_id=%d, setpoint_rate=%dms, takeoff_alt=%.2f, min_alt=%.2f, pos_step=%.2f, yaw_step=%.1f,initial_mode=%s",
                   drone_id_, setpoint_rate_ms_, takeoff_altitude_, min_flight_altitude_, teleop_position_step_, yaw_step_,
                   ControlModeToString(initial_control_mode_).c_str());
                   
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
 
        // Create publishers with drone-specific topics
        std::string topic_prefix = GetTopicPrefix();
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            topic_prefix + "/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            topic_prefix + "/fmu/in/trajectory_setpoint", qos);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            topic_prefix + "/fmu/in/vehicle_command", qos);

        RCLCPP_INFO(this->get_logger(), "Created PX4 command publishers");
        
        // Create subscribers with drone-specific topics
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            topic_prefix + "/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&PX4Controller::VehicleLocalPositionCallback, this, _1));

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            topic_prefix + "/fmu/out/vehicle_status_v1", qos,
            std::bind(&PX4Controller::VehicleStatusCallback, this, _1));

        vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            topic_prefix + "/fmu/out/vehicle_land_detected", qos,
            std::bind(&PX4Controller::VehicleLandDetectedCallback, this, _1));
        
        teleop_command_sub_ = this->create_subscription<flyscan_interfaces::msg::TeleopCommand>(
            GetControllerTopicName("/teleop_command"), qos,
            std::bind(&PX4Controller::TeleopCommandCallback, this, _1));

        frontiers_ranked_sub_ = this->create_subscription<flyscan_interfaces::msg::FrontierArray>(
            "/frontiers_ranked", qos,
            std::bind(&PX4Controller::FrontiersRankedCallback, this, _1));
            
        obstacle_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            std::bind(&PX4Controller::ObstacleMapCallback, this, _1));
            
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            GetCameraTopicName("/camera/depth/points"), qos,
            std::bind(&PX4Controller::PointCloudCallback, this, _1));

        // create services
        set_control_mode_service_ = this->create_service<flyscan_interfaces::srv::SetControlMode>(
            GetControllerTopicName("/set_control_mode"),
            std::bind(&PX4Controller::HandleSetControlModeService, this, _1, _2));

        // Create timers
        setpoint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(setpoint_rate_ms_),
            std::bind(&PX4Controller::SetpointTimerCallback, this));
        setpoint_timer_->cancel();
        
        safe_navigation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz for navigation updates
            std::bind(&PX4Controller::PointCloudNavigationTimerCallback, this));
        safe_navigation_timer_->cancel();
        
        current_point_cloud_ = nullptr;
        kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

        RCLCPP_INFO(this->get_logger(), "Created setpoint timer (%d ms interval)", setpoint_rate_ms_);

        // Initialize position setpoint to safe values
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = takeoff_altitude_;  // NED: negative is up
        current_position_setpoint_.yaw_deg = 0.0f;

        RCLCPP_INFO(this->get_logger(), "PX4 Controller configured successfully");
        return OperationStatus::kOK;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure PX4 Controller: %s", e.what());
        return OperationStatus::kNotInitialized;
    }
}

OperationStatus PX4Controller::HandleActivate() {
    RCLCPP_INFO(this->get_logger(), "Activating PX4 Controller...");

    // Switch to the configured initial control mode
    OperationStatus switch_status = SwitchToMode(initial_control_mode_);
    if (switch_status != OperationStatus::kOK) {
        RCLCPP_WARN(this->get_logger(), "Failed to switch to initial mode %s, defaulting to MANUAL", 
                   ControlModeToString(initial_control_mode_).c_str());
        current_mode_ = ControlMode::kManual;
    }

    RCLCPP_INFO(this->get_logger(), "PX4 Controller activated in %s mode", 
                ControlModeToString(current_mode_).c_str());
    
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::HandleDeactivate() {
    RCLCPP_INFO(this->get_logger(), "Deactivating PX4 Controller...");

    RCLCPP_INFO(this->get_logger(), "PX4 Controller deactivated");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::HandleCleanup() {
    RCLCPP_INFO(this->get_logger(), "Cleaning up PX4 Controller...");
    
    should_exit_ = true;
    
    if (setpoint_timer_) {
        setpoint_timer_->cancel();
    }
    
    if (safe_navigation_timer_) {
        safe_navigation_timer_->cancel();
    }
    
    RCLCPP_INFO(this->get_logger(), "PX4 Controller cleanup complete");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::HandleShutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down PX4 Controller...");
    return HandleCleanup();
}

OperationStatus PX4Controller::HandleError() {
    RCLCPP_ERROR(this->get_logger(), "PX4 Controller error state - attempting recovery...");
    
    current_mode_ = ControlMode::kManual;
    
    RCLCPP_INFO(this->get_logger(), "Returned to MANUAL mode for safety");
    return OperationStatus::kOK;
}

// ============================================================================
// Mode Management Implementation
// ============================================================================

void PX4Controller::HandleSetControlModeService(
    const std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Request> request,
    std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Response> response) 
{
    
    ControlMode requested_mode = static_cast<ControlMode>(request->mode);
    ControlMode previous_mode;
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        previous_mode = current_mode_;
    }

    std::string previous_mode_str = ControlModeToString(previous_mode);
    std::string requested_mode_str = ControlModeToString(requested_mode);
    std::string move_str = "(from " + previous_mode_str + " to " + requested_mode_str + ")";
    
    RCLCPP_INFO(this->get_logger(), "Mode switch requested: %s", move_str.c_str());

    OperationStatus switch_status = SwitchToMode(requested_mode);
    
    response->success = (switch_status == OperationStatus::kOK);
    response->previous_mode = static_cast<uint8_t>(previous_mode);

    switch (switch_status) {
        case OperationStatus::kOK:
            response->message = "Mode switched successfully " + move_str;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            break;
        case OperationStatus::kNotImplemented:
            response->message = "Mode switch not implemented " + move_str;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            break;
        default:
            response->message = "Failed to switch " + move_str;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            break;
    }
}

OperationStatus PX4Controller::SwitchToMode(ControlMode new_mode) 
{
    OperationStatus status = OperationStatus::kNotImplemented;
    switch (new_mode) {
        case ControlMode::kManual:
            status = EnterManualMode();
            break;
        case ControlMode::kTeleop:
            status = EnterTeleopMode();
            safe_navigation_timer_->cancel();
            break;
        case ControlMode::kAutonomous:
            status = EnterAutonomousMode();
            break;
        case ControlMode::kMission:
            status = EnterMissionMode();
            break;
        case ControlMode::kRTL:
            RCLCPP_WARN(this->get_logger(), "RTL mode not yet implemented");
            status = OperationStatus::kNotImplemented;
            break;
        case ControlMode::kLand:
            RCLCPP_WARN(this->get_logger(), "LAND mode not yet implemented");
            status = OperationStatus::kNotImplemented;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s", ControlModeToString(new_mode).c_str());
            status = OperationStatus::kNotImplemented;
            break;
    }

    if (status == OperationStatus::kOK) {
        current_mode_ = new_mode;
        RCLCPP_INFO(this->get_logger(), "Successfully switched to %s mode", 
                     ControlModeToString(new_mode).c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter %s mode - remaining in %s mode",
                     ControlModeToString(new_mode).c_str(), ControlModeToString(current_mode_).c_str());
        current_mode_ = ControlMode::kManual;
    }

    return status;
}

OperationStatus PX4Controller::EnterManualMode() {
    RCLCPP_INFO(this->get_logger(), "Entering MANUAL mode");
    
    // Cancel frontier checking timer
    return OperationStatus::kOK;
}


OperationStatus PX4Controller::EnterTeleopMode() {
    RCLCPP_INFO(this->get_logger(), "Entering TELEOP mode");
    
    // Cancel frontier checking timer
    OperationStatus offboard_status = StartOffboardMode();
    if (offboard_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode - TELEOP mode activation failed");
        return offboard_status;
    }

    RCLCPP_INFO(this->get_logger(), "TELEOP mode active with offboard control - ready for teleop commands");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::EnterAutonomousMode() 
{
    RCLCPP_INFO(this->get_logger(), "Entering AUTONOMOUS mode for exploration");
    
    if (current_mode_ == ControlMode::kAutonomous) {
        RCLCPP_INFO(this->get_logger(), "Already in AUTONOMOUS mode - no action taken");
        return OperationStatus::kOK;
    }
    safe_navigation_timer_->reset();
    navigation_state_ = NavigationState::kIdle;

    OperationStatus offboard_status = StartOffboardMode();
    if (offboard_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode - AUTONOMOUS mode activation failed");
        return offboard_status;
    }
    
    RCLCPP_INFO(this->get_logger(), "AUTONOMOUS mode active - ready for exploration goals");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::EnterMissionMode() {
    RCLCPP_INFO(this->get_logger(), "Entering MISSION mode for waypoint execution");

    if (current_mode_ == ControlMode::kMission) {
        return OperationStatus::kOK;
    }

    OperationStatus offboard_status = StartOffboardMode();
    if (offboard_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode - MISSION mode activation failed");
        return offboard_status;
    }

    RCLCPP_INFO(this->get_logger(), "MISSION mode active - ready for waypoint missions");
    return OperationStatus::kOK;
}

void PX4Controller::FrontiersRankedCallback(const flyscan_interfaces::msg::FrontierArray::SharedPtr msg) 
{
    if (current_mode_ != ControlMode::kAutonomous) {
        return;
    }
    std::lock_guard<std::mutex> lock(frontiers_mutex_);
    current_frontiers_ranked_ = *msg;

    if (navigation_state_ == NavigationState::kNavigating) {
        RCLCPP_DEBUG(this->get_logger(), "Navigation in progress, cannot process new frontiers");
        return;
    }

    current_frontier_index_ = 0;
    navigation_state_ = NavigationState::kNavigating;

    geometry_msgs::msg::PoseStamped new_frontier_goal;
    new_frontier_goal.header.stamp = this->now();
    new_frontier_goal.header.frame_id = "map";
    new_frontier_goal.pose.position = current_frontiers_ranked_.frontiers[current_frontier_index_].center;
    new_frontier_goal.pose.orientation.w = 1.0;

    ProcessFrontierGoal(new_frontier_goal); 
    RCLCPP_DEBUG(this->get_logger(), "Received %zu ranked frontiers, reset index to 0", 
                msg->frontiers.size());
}

void PX4Controller::ProcessFrontierGoal(const geometry_msgs::msg::PoseStamped& frontier_goal) 
{
    RCLCPP_INFO(this->get_logger(), "Processing frontier: (%.2f, %.2f, %.2f)", 
               frontier_goal.pose.position.x, frontier_goal.pose.position.y, frontier_goal.pose.position.z);

    // Calculate forward movement direction based on camera scanning cone
    geometry_msgs::msg::Point forward_position;
    float forward_yaw;
    CalculateForwardCameraDirection(frontier_goal.pose.position, forward_position, forward_yaw);

    RCLCPP_INFO(this->get_logger(), "Started forward navigation in camera scanning direction: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
                forward_position.x, forward_position.y, forward_position.z, forward_yaw);

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = this->now();
    target_pose.header.frame_id = "map";
    target_pose.pose.position = forward_position;
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, forward_yaw * M_PI / 180.0);
    target_pose.pose.orientation = tf2::toMsg(q);
    
    current_navigation_target_ = target_pose;
    navigation_max_velocity_ = 1.0;
    navigation_safety_margin_ = drone_radius_ + 0.3;  // Dynamic margin based on drone size
    navigation_attempt_counter_ = 0;

    RCLCPP_INFO(this->get_logger(), "Started point cloud navigation to target (%.2f, %.2f, %.2f)", 
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

}

bool PX4Controller::SelectNextFrontierFromList() {
    std::lock_guard<std::mutex> lock(frontiers_mutex_);
    
    // Check if we have any frontiers available
    if (current_frontiers_ranked_.frontiers.empty()) {
        RCLCPP_WARN(this->get_logger(), "No frontiers available in ranked list");
        return false;
    }
    
    // Move to next frontier in the list
    current_frontier_index_++;
    
    // Check if we've exhausted all available frontiers
    if (current_frontier_index_ >= current_frontiers_ranked_.frontiers.size()) {
        current_frontier_index_ = 0;
        navigation_state_ = NavigationState::kIdle;
    }
    
    // Get the next frontier from the list
    const auto& next_frontier = current_frontiers_ranked_.frontiers[current_frontier_index_];
    
    RCLCPP_INFO(this->get_logger(), "Selecting next frontier from list: frontier %zu/%zu at (%.2f, %.2f) with utility %.3f",
                current_frontier_index_ + 1, current_frontiers_ranked_.frontiers.size(),
                next_frontier.center.x, next_frontier.center.y, next_frontier.utility_score);
    
    // Create PoseStamped message for the new frontier
    geometry_msgs::msg::PoseStamped new_frontier_goal;
    new_frontier_goal.header.stamp = this->now();
    new_frontier_goal.header.frame_id = "map";
    new_frontier_goal.pose.position = next_frontier.center;
    new_frontier_goal.pose.orientation.w = 1.0;
    
    ProcessFrontierGoal(new_frontier_goal);

    return true;
}



// ============================================================================
// Core PX4 Communication and Control Implementation
// ============================================================================

void PX4Controller::VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_position_ = *msg;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "VehicleLocalPositionCallback: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
        current_position_.x, current_position_.y, current_position_.z, 
        current_position_.heading * 180.0f / M_PI);
}

void PX4Controller::VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_ = *msg;
    bool was_armed = armed_;
    armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    if (was_armed != armed_) {
        RCLCPP_INFO(this->get_logger(), "Armed state changed: %s", armed_ ? "ARMED" : "DISARMED");
    }
}

void PX4Controller::VehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    bool was_in_air = in_air_;
    in_air_ = !msg->landed;
    if (was_in_air != in_air_) {
        RCLCPP_INFO(this->get_logger(), "In-air state changed: %s", in_air_ ? "IN AIR" : "LANDED");
    }
}

void PX4Controller::TeleopCommandCallback(const flyscan_interfaces::msg::TeleopCommand::SharedPtr msg) {
    // Only process teleop commands in teleop mode
    if (current_mode_ != ControlMode::kTeleop) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received TeleopCommand: %s", msg->command.c_str());
    
    {
        std::lock_guard<std::mutex> lock(teleop_command_mutex_);
        current_teleop_command_ = *msg;
    }

    if (msg->command == "exit_teleop") {
        RCLCPP_INFO(this->get_logger(), "Received exit teleop command");
        SwitchToMode(ControlMode::kManual);
        return;
    }

    if (msg->command == "takeoff") {
        RCLCPP_INFO(this->get_logger(), "Received takeoff command - going to safe altitude");
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = takeoff_altitude_;  // Use configured takeoff altitude
        current_position_setpoint_.yaw_deg = 0.0f;
        return;
    }
    
    if (msg->command == "land") {
        RCLCPP_INFO(this->get_logger(), "Received land command - going to NED (0,0,0)");
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = 0.0f;
        current_position_setpoint_.yaw_deg = 0.0f;
        return;
    }

    // Get current yaw for body frame transformation
    float current_yaw_rad = 0.0f;
    {
        std::lock_guard<std::mutex> position_lock(position_mutex_);
        current_yaw_rad = current_position_.heading;
    }
    
    std::lock_guard<std::mutex> pos_lock(position_setpoint_mutex_);
    
    if (msg->command == "hold_position" || msg->command == "stop") {
        RCLCPP_INFO(this->get_logger(), "Holding current position");
        return;
    }

    // Apply movement commands using first-person pov
    if (msg->command == "forward" || msg->command == "backward" || msg->command == "right" || msg->command == "left") {
        // Define body frame movements
        float body_x = 0.0f; // forward/backward
        float body_y = 0.0f; // left/right
        
        if (msg->command == "forward") {
            body_x = teleop_position_step_;
        } else if (msg->command == "backward") {
            body_x = -teleop_position_step_;
        } else if (msg->command == "right") {
            body_y = teleop_position_step_;
        } else if (msg->command == "left") {
            body_y = -teleop_position_step_;
        }

        // Transform body frame to world frame using current yaw
        float world_x, world_y;
        this->TransformBodyToWorld(body_x, body_y, current_yaw_rad, world_x, world_y);
        
        // Apply the transformed movement to world frame coordinates
        current_position_setpoint_.north_m += world_x;
        current_position_setpoint_.east_m += world_y;
        
    } else if (msg->command == "up") {
        current_position_setpoint_.down_m -= teleop_position_step_;  // NED: negative is up
    } else if (msg->command == "down") {
        // Enforce minimum altitude when going down
        float new_altitude = current_position_setpoint_.down_m + teleop_position_step_;
        current_position_setpoint_.down_m = std::min(new_altitude, static_cast<float>(min_flight_altitude_));
    } else if (msg->command == "yaw_right") {
        current_position_setpoint_.yaw_deg += yaw_step_;
        // Normalize yaw to [-180, 180]
        while (current_position_setpoint_.yaw_deg > 180.0f) {
            current_position_setpoint_.yaw_deg -= 360.0f;
        }
    } else if (msg->command == "yaw_left") {
        current_position_setpoint_.yaw_deg -= yaw_step_;
        // Normalize yaw to [-180, 180]
        while (current_position_setpoint_.yaw_deg < -180.0f) {
            current_position_setpoint_.yaw_deg += 360.0f;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Updated teleop setpoint: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f (current_yaw=%.1f°)", 
                current_position_setpoint_.north_m, current_position_setpoint_.east_m, 
                current_position_setpoint_.down_m, current_position_setpoint_.yaw_deg, current_yaw_rad * 180.0f / M_PI);
}

OperationStatus PX4Controller::StartOffboardMode() 
{
    RCLCPP_INFO(this->get_logger(), "Starting offboard mode sequence...");
    SendArmDisarmCommand(true);

    px4_msgs::msg::VehicleLocalPosition current_position;
    {
        std::lock_guard<std::mutex> pos_lock(position_mutex_);
        current_position = current_position_;
    }

    if (std::isnan(current_position.x) || std::isnan(current_position.y) || std::isnan(current_position.z)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot enter AUTONOMOUS mode: Invalid position data");
        return OperationStatus::kNotInitialized;
    }

    {
        std::lock_guard<std::mutex> setpoint_lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = current_position.x;
        current_position_setpoint_.east_m  = current_position.y;
        current_position_setpoint_.down_m = current_position.z;
        current_position_setpoint_.yaw_deg = current_position.heading * 180.0f / M_PI;
    }

    rclcpp::WallRate rate(1000.0 / setpoint_rate_ms_);

    // Send a few setpoints before switching to offboard mode
    for (int i = 0; i < 10; ++i) {
        PublishOffboardControlMode();
        PublishTrajectorySetpoint();
        rate.sleep();
    }

    // Switch to offboard mode
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    setpoint_timer_->reset();

    RCLCPP_INFO(this->get_logger(), "Offboard mode started successfully");
    return OperationStatus::kOK;
}

void PX4Controller::SetpointTimerCallback() {
    if (should_exit_) {
        return;
    }

    PublishOffboardControlMode();
    PublishTrajectorySetpoint();
}

void PX4Controller::PublishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    offboard_control_mode_publisher_->publish(msg);
}

void PX4Controller::PublishTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};

    Position cmd;
    {
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        cmd = current_position_setpoint_;
    }
    
    msg.position = {cmd.north_m, cmd.east_m, cmd.down_m};
    msg.yaw = cmd.yaw_deg * M_PI / 180.0f; // Convert to radians
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Publishing TrajectorySetpoint: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
                 cmd.north_m, cmd.east_m, cmd.down_m, cmd.yaw_deg);
    trajectory_setpoint_publisher_->publish(msg);
}


void PX4Controller::PublishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    RCLCPP_DEBUG(this->get_logger(), "Publishing VehicleCommand: cmd=%d, param1=%.2f, param2=%.2f",
                 command, param1, param2);
    vehicle_command_publisher_->publish(msg);
}

void PX4Controller::SendArmDisarmCommand(bool arm_vehicle) {
    float param1 = arm_vehicle ? 1.0f : 0.0f;
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, param1);
    RCLCPP_INFO(this->get_logger(), "Sent %s command to vehicle", arm_vehicle ? "ARM" : "DISARM");
}

void PX4Controller::TransformBodyToWorld(float body_x, float body_y, float current_yaw_rad, float& world_x, float& world_y) {
    float cos_yaw = std::cos(current_yaw_rad);
    float sin_yaw = std::sin(current_yaw_rad);
    
    world_x = body_x * cos_yaw - body_y * sin_yaw;
    world_y = body_x * sin_yaw + body_y * cos_yaw;
    // auto rotated = std::polar(1.0f, current_yaw_rad) * std::complex<float>(body_x, body_y);
    // world_x = rotated.real();
    // world_y = rotated.imag();
}

std::string PX4Controller::GetTopicPrefix() const {
    return (drone_id_ == 0) ? "" : "/px4_" + std::to_string(drone_id_);
}

std::string PX4Controller::GetControllerTopicName(const std::string& topic_suffix) const {
    return (drone_id_ == 0) ? "/px4_controller" + topic_suffix : "/px4_controller_" + std::to_string(drone_id_) + topic_suffix;
}

std::string PX4Controller::GetCameraTopicName(const std::string& topic_name) const {
    return (drone_id_ == 0) ? topic_name : "/px4_" + std::to_string(drone_id_) + topic_name;
}

void PX4Controller::CalculateForwardCameraDirection(const geometry_msgs::msg::Point& frontier_pos, 
                                                    geometry_msgs::msg::Point& forward_pos, float& forward_yaw) {
    // Get current robot position and heading
    float current_x, current_y, current_z, current_heading;
    {
        std::lock_guard<std::mutex> position_lock(position_mutex_);
        current_x = current_position_.x;
        current_y = current_position_.y;
        current_z = current_position_.z;
        current_heading = current_position_.heading;
    }
    
    // Calculate camera position in world frame
    double cam_world_x = current_x + camera_offset_x_ * cos(current_heading);
    double cam_world_y = current_y + camera_offset_x_ * sin(current_heading);
    double cam_world_z = current_z + camera_offset_z_;
    
    // Calculate direction from camera position to frontier
    double dx = frontier_pos.x - cam_world_x;
    double dy = frontier_pos.y - cam_world_y;
    double distance_to_frontier = sqrt(dx*dx + dy*dy);
    
    // Calculate yaw to face the frontier from camera perspective
    forward_yaw = atan2(dy, dx) * 180.0f / M_PI;
    
    // Move forward in the camera scanning cone direction
    // Account for the camera's cone-shaped scan area
    double forward_distance;
    if (distance_to_frontier > optimal_camera_distance_) {
        // Move closer to get within optimal camera range
        forward_distance = distance_to_frontier - optimal_camera_distance_ * 0.5;
    } else {
        // Move a smaller step to stay in camera range
        forward_distance = std::min(distance_to_frontier * 0.3, 1.0);
    }
    
    if (distance_to_frontier > 0.1) {
        // Move toward the frontier considering camera offset
        double unit_dx = dx / distance_to_frontier;
        double unit_dy = dy / distance_to_frontier;
        
        // Calculate forward position accounting for camera scanning geometry
        forward_pos.x = current_x + unit_dx * forward_distance;
        forward_pos.y = current_y + unit_dy * forward_distance;
    } else {
        // Very close to frontier, move slightly forward in current heading direction
        forward_pos.x = current_x + cos(current_heading) * 0.5;
        forward_pos.y = current_y + sin(current_heading) * 0.5;
    }
    
    // Set altitude for forward movement
    if (current_mode_ == ControlMode::kAutonomous) {
        forward_pos.z = autonomous_fixed_height_;
        RCLCPP_DEBUG(this->get_logger(), "Using autonomous fixed height: %.2f", forward_pos.z);
    } else {
        // Maintain safe altitude
        forward_pos.z = (frontier_pos.z != 0.0) ? frontier_pos.z : current_z;
        if (forward_pos.z > min_flight_altitude_) {
            forward_pos.z = min_flight_altitude_;
            RCLCPP_DEBUG(this->get_logger(), "Enforced minimum altitude: %.2f", forward_pos.z);
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Camera-aware forward direction: frontier(%.2f,%.2f,%.2f) -> forward(%.2f,%.2f,%.2f), yaw=%.1f°, cam_pos(%.2f,%.2f)",
                frontier_pos.x, frontier_pos.y, frontier_pos.z,
                forward_pos.x, forward_pos.y, forward_pos.z, forward_yaw, cam_world_x, cam_world_y);
}


// ============================================================================
// Unified Navigation System Implementation
// ============================================================================

bool PX4Controller::IsPathClearPointCloud(const geometry_msgs::msg::Point& start, 
                                         const geometry_msgs::msg::Point& end) {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
    
    if (!current_point_cloud_ || current_point_cloud_->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "No point cloud available for path checking");
        return true; // Assume clear if no point cloud
    }
    
    // Calculate path vector
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double dz = end.z - start.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    if (distance < 0.1) return true; // Very close, assume clear
    
    // Sample points along the path with appropriate density for small drone
    int num_samples = static_cast<int>(distance / 0.25) + 1; // Sample every 25cm for efficiency
    double step_x = dx / num_samples;
    double step_y = dy / num_samples;
    double step_z = dz / num_samples;
    
    for (int i = 0; i <= num_samples; ++i) {
        pcl::PointXYZ query_point;
        query_point.x = start.x + i * step_x;
        query_point.y = start.y + i * step_y;
        query_point.z = start.z + i * step_z;
        
        // Check for nearby obstacles using KD-tree with adjusted safety margin
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        
        // Dynamic safety margin based on drone velocity and context
        double dynamic_safety_margin = std::max(navigation_safety_margin_, drone_radius_ + 0.2);
        
        int num_neighbors = kdtree_->radiusSearch(query_point, dynamic_safety_margin, 
                                                point_indices, point_distances);
        
        if (num_neighbors >= min_obstacle_points_) {  // Require minimum points for obstacle
            // Check if nearby points represent real obstacles for small drone
            int valid_obstacle_points = 0;
            for (int j = 0; j < num_neighbors; ++j) {
                const pcl::PointXYZ& obstacle_point = current_point_cloud_->points[point_indices[j]];
                
                // Filter out ground points and consider altitude-relevant obstacles
                // In NED frame: higher z values are further down, so obstacles above drone have lower z
                double altitude_diff = obstacle_point.z - query_point.z;
                double horizontal_dist = sqrt(point_distances[j]);

                // More lenient altitude tolerance for small drone (±50cm instead of ±30cm)
                // Only count as obstacle if close enough horizontally and at relevant altitude
                if (altitude_diff >= -0.5 && altitude_diff <= 0.5 && horizontal_dist <= dynamic_safety_margin) {
                    valid_obstacle_points++;
                }
            }
            
            // Only consider as blocked if enough obstacle points found
            if (valid_obstacle_points >= min_obstacle_points_) {
                RCLCPP_DEBUG(this->get_logger(), "Obstacle detected: %d valid points within %.2fm dynamic safety margin", 
                            valid_obstacle_points, dynamic_safety_margin);
                return false;
            }
        }
    }
    
    return true;
}

geometry_msgs::msg::Point PX4Controller::CalculateNextNavigationStep(const geometry_msgs::msg::Point& current_pos,
                                                                      const geometry_msgs::msg::Point& target_pos) {
    // Calculate direction vector to target
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;
    double dz = target_pos.z - current_pos.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    geometry_msgs::msg::Point next_step;
    
    if (distance <= navigation_step_size_) {
        // Close to target, move directly to it
        next_step = target_pos;
    } else {
        // Calculate unit vector and step forward
        double unit_dx = dx / distance;
        double unit_dy = dy / distance;
        double unit_dz = dz / distance;
        
        next_step.x = current_pos.x + unit_dx * navigation_step_size_;
        next_step.y = current_pos.y + unit_dy * navigation_step_size_;
        next_step.z = current_pos.z + unit_dz * navigation_step_size_;
    }
    
    // In autonomous mode, use fixed height; otherwise enforce minimum flight altitude
    if (current_mode_ == ControlMode::kAutonomous) {
        next_step.z = autonomous_fixed_height_;
        RCLCPP_DEBUG(this->get_logger(), "Navigation step using autonomous fixed height: %.2f", next_step.z);
    } else {
        // Enforce minimum flight altitude (NED frame: negative is up)
        if (next_step.z > min_flight_altitude_) {
            next_step.z = min_flight_altitude_;
            RCLCPP_DEBUG(this->get_logger(), "Navigation step altitude clamped to minimum: %.2f", next_step.z);
        }
    }
    
    return next_step;
}

double PX4Controller::ComputeAvoidanceDirection(double obstacle_direction) {
    // Simple avoidance: turn 90 degrees from obstacle direction
    double avoidance_direction = obstacle_direction + M_PI/2;
    
    // Normalize to [-pi, pi]
    while (avoidance_direction > M_PI) avoidance_direction -= 2*M_PI;
    while (avoidance_direction < -M_PI) avoidance_direction += 2*M_PI;
    
    return avoidance_direction;
}

void PX4Controller::PointCloudNavigationTimerCallback() 
{
    if (navigation_state_ != NavigationState::kNavigating) {
        RCLCPP_DEBUG(this->get_logger(), "Point cloud navigation timer callback triggered but not in navigating state");
        return;
    }
    // ========== NAVIGATION EXECUTION ==========
    RCLCPP_DEBUG(this->get_logger(), "Point cloud navigation timer callback triggered");
    
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    
    // Get current position
    geometry_msgs::msg::Point current_pos;
    {
        std::lock_guard<std::mutex> pos_lock(position_mutex_);
        current_pos.x = current_position_.x;
        current_pos.y = current_position_.y;
        current_pos.z = current_position_.z;
    }
    
    // Check if target is reached
    double dx = current_navigation_target_.pose.position.x - current_pos.x;
    double dy = current_navigation_target_.pose.position.y - current_pos.y;
    double dz = current_navigation_target_.pose.position.z - current_pos.z;
    double distance_to_target = sqrt(dx*dx + dy*dy + dz*dz);
    
    if (distance_to_target < waypoint_tolerance_) {
        navigation_state_ = NavigationState::kIdle;
        RCLCPP_INFO(this->get_logger(), "Point cloud navigation completed - reached target");
        return;
    }

    // Calculate next step towards target
    geometry_msgs::msg::Point next_step = CalculateNextNavigationStep(current_pos, current_navigation_target_.pose.position);
    
    // Check for obstacles in the path using point cloud
    // Calculate desired yaw to face the target direction
    double desired_yaw_rad = atan2(dy, dx);
    double desired_yaw_deg = desired_yaw_rad * 180.0 / M_PI;
    
    // Get current yaw
    double current_yaw_deg;
    {
        std::lock_guard<std::mutex> pos_lock(position_mutex_);
        current_yaw_deg = current_position_.heading * 180.0 / M_PI;
    }
    
    // Calculate yaw error (normalize to [-180, 180])
    double yaw_error = desired_yaw_deg - current_yaw_deg;
    while (yaw_error > 180.0) yaw_error -= 360.0;
    while (yaw_error < -180.0) yaw_error += 360.0;
    
    // YAW-FIRST NAVIGATION: Rotate first, then move forward when properly oriented
    if (std::abs(yaw_error) > yaw_tolerance_) {
        // Need to rotate first - camera must face forward (+X direction)
        {
            std::lock_guard<std::mutex> setpoint_lock(position_setpoint_mutex_);
            // Hold current position while rotating
            current_position_setpoint_.north_m = current_pos.x;
            current_position_setpoint_.east_m = current_pos.y;
            current_position_setpoint_.down_m = current_pos.z;
            // Set desired yaw
            current_position_setpoint_.yaw_deg = desired_yaw_deg;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Rotating to face target: current_yaw=%.1f°, desired_yaw=%.1f°, error=%.1f°", 
                    current_yaw_deg, desired_yaw_deg, yaw_error);
    } else if (IsPathClearPointCloud(current_pos, next_step)) {
        // Path is clear, move to next step
        {
            std::lock_guard<std::mutex> setpoint_lock(position_setpoint_mutex_);
            current_position_setpoint_.north_m = next_step.x;
            current_position_setpoint_.east_m = next_step.y;
            
            // Use fixed height in autonomous mode, otherwise enforce minimum altitude
            if (current_mode_ == ControlMode::kAutonomous) {
                current_position_setpoint_.down_m = autonomous_fixed_height_;
            } else {
                // Enforce minimum altitude before setting position
                double safe_altitude = std::min(next_step.z, min_flight_altitude_);
                current_position_setpoint_.down_m = safe_altitude;
            }

            // Face movement direction
            current_position_setpoint_.yaw_deg = desired_yaw_deg;

        }
        RCLCPP_DEBUG(this->get_logger(), "Moving to next step: (%.2f, %.2f, %.2f), distance to target: %.2f", 
                    next_step.x, next_step.y, next_step.z, distance_to_target);
        
    } else {
        RCLCPP_INFO(this->get_logger(), "Obstacle detected in path to current frontier, trying next frontier from list");
        
        if (current_mode_ == ControlMode::kAutonomous && SelectNextFrontierFromList()) {
            RCLCPP_INFO(this->get_logger(), "Successfully switched to next frontier from list");
            return;
        } else {
            RCLCPP_WARN(this->get_logger(), "No alternative frontier available or not in autonomous mode, holding position");
            
            navigation_attempt_counter_++;
            if (navigation_attempt_counter_ > max_navigation_attempts_) {
                RCLCPP_INFO(this->get_logger(), "Max navigation attempts reached, aborting navigation");
                navigation_state_ = NavigationState::kIdle;
            }
        }
    }
}

void PX4Controller::ObstacleMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    obstacle_map_ = msg;
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Updated obstacle map: %dx%d, resolution: %.3f",
                         msg->info.width, msg->info.height, msg->info.resolution);
}

void PX4Controller::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
    
    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    
    if (raw_cloud->empty()) {
        return;
    }
    
    // Transform point cloud from camera frame to drone/world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    TransformPointCloudFromCamera(raw_cloud, transformed_cloud);
    
    // Downsample the transformed point cloud for efficiency
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(transformed_cloud);
    voxel_filter.setLeafSize(point_cloud_downsample_leaf_size_, 
                             point_cloud_downsample_leaf_size_, 
                             point_cloud_downsample_leaf_size_);
    voxel_filter.filter(*filtered_cloud);
    
    // Update current point cloud
    current_point_cloud_ = filtered_cloud;
    
    // Update KD-tree for efficient nearest neighbor search
    if (!current_point_cloud_->empty()) {
        kdtree_->setInputCloud(current_point_cloud_);
    }
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Processed point cloud: %zu raw -> %zu transformed -> %zu filtered points", 
                         raw_cloud->size(), transformed_cloud->size(), filtered_cloud->size());
}

void PX4Controller::TransformPointCloudFromCamera(const pcl::PointCloud<pcl::PointXYZ>::Ptr& camera_cloud,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& world_cloud) {
    // Get current drone position and orientation
    float drone_x, drone_y, drone_z, drone_heading;
    {
        std::lock_guard<std::mutex> position_lock(position_mutex_);
        drone_x = current_position_.x;
        drone_y = current_position_.y;
        drone_z = current_position_.z;
        drone_heading = current_position_.heading;
    }
    
    world_cloud->clear();
    world_cloud->reserve(camera_cloud->size());
    
    // Transform each point from camera frame to world frame
    for (const auto& cam_point : camera_cloud->points) {
        // Skip invalid points
        if (!std::isfinite(cam_point.x) || !std::isfinite(cam_point.y) || !std::isfinite(cam_point.z)) {
            continue;
        }
        
        // Camera frame: X=forward, Y=left, Z=up (typical camera convention)
        // Drone frame: X=north, Y=east, Z=down (NED convention)
        
        // Transform from camera frame to drone body frame
        // Camera pointing forward: cam_x -> drone_x, cam_y -> -drone_y, cam_z -> -drone_z
        double body_x = cam_point.x;  // Camera forward = drone forward
        double body_y = -cam_point.y; // Camera left = drone right (flip for NED)
        double body_z = -cam_point.z; // Camera up = drone down (flip for NED)
        
        // Add camera offset to get point relative to drone center
        body_x += camera_offset_x_;  // Camera offset forward from drone center
        body_z += camera_offset_z_;  // Camera offset down from drone center
        
        // Rotate from drone body frame to world frame using drone heading
        double cos_heading = std::cos(drone_heading);
        double sin_heading = std::sin(drone_heading);
        
        double world_x = body_x * cos_heading - body_y * sin_heading;
        double world_y = body_x * sin_heading + body_y * cos_heading;
        double world_z = body_z;
        
        // Translate to world coordinates
        pcl::PointXYZ world_point;
        world_point.x = drone_x + world_x;
        world_point.y = drone_y + world_y;
        world_point.z = drone_z + world_z;
        
        world_cloud->push_back(world_point);
    }
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Transformed %zu camera points to world frame at drone pos (%.2f,%.2f,%.2f) heading %.2f°",
                         world_cloud->size(), drone_x, drone_y, drone_z, drone_heading * 180.0 / M_PI);
}


} // namespace drone_controller
} // namespace flyscan



int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    
    flyscan::drone_controller::PX4Controller::SharedPtr controller;

    try {
        controller = std::make_shared<flyscan::drone_controller::PX4Controller>();
        flyscan::common::SetupSigintHandler(controller, "px4_controller_main");
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(controller->get_node_base_interface());

        auto configure_result = controller->configure();
        if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("px4_controller_main"), "Failed to configure PX4Controller");
            return 1;
        }

        auto activate_result = controller->activate();
        if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("px4_controller_main"), "Failed to activate PX4Controller");
            return 1;
        }

        RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), 
                    "PX4Controller is active and ready for mode switching");
        
        // Get the drone_id parameter to show the correct service call
        int drone_id = controller->get_parameter("drone_id").as_int();
        std::string service_name = (drone_id == 1) ? "/px4_controller/set_control_mode" : "/px4_controller_" + std::to_string(drone_id) + "/set_control_mode";
        
        RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), 
                    "Use: ros2 service call %s flyscan_interfaces/srv/SetControlMode \"{mode: 1}\"", service_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), 
                    "Mode 0=MANUAL, 1=TELEOP, 2=AUTONOMOUS, 3=MISSION, 4=RTL, 5=LAND");

        executor.spin();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("px4_controller_main"), "Exception in main: %s", e.what());
        if (controller) {
            controller->shutdown();
        }
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), "PX4Controller main loop complete");
    return 0;
}