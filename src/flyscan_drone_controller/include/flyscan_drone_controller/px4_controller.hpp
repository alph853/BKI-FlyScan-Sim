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
#include <flyscan_interfaces/msg/teleop_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "flyscan_core/base_node.hpp"
#include "flyscan_common/enums.hpp"

namespace flyscan {
namespace drone_controller {

using flyscan::common::OperationStatus;
using flyscan::common::NodeType;
using flyscan::common::ControlMode;
using flyscan::core::BaseNode;

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
     * @brief Callback for exploration goal messages
     * @param msg Exploration goal message
     */
    void ExplorationGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr exploration_goal_sub_;

    rclcpp::Service<flyscan_interfaces::srv::SetControlMode>::SharedPtr set_control_mode_service_;
    
    rclcpp::TimerBase::SharedPtr setpoint_timer_;

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
    
    // ============================================================================
    // Thread-Safe
    // ============================================================================
    
    Position current_position_setpoint_;
    flyscan_interfaces::msg::TeleopCommand current_teleop_command_;
    geometry_msgs::msg::PoseStamped current_exploration_goal_;
    std::mutex position_setpoint_mutex_;
    std::mutex teleop_command_mutex_;
    std::mutex exploration_goal_mutex_;
    
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
    float teleop_position_step_;
    float yaw_step_;
    ControlMode initial_control_mode_;
    
};

} // namespace drone_controller
} // namespace flyscan


RCLCPP_COMPONENTS_REGISTER_NODE(flyscan::drone_controller::PX4Controller)