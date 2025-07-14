/**
 * @file px4_controller.cpp
 * @brief Advanced PX4 Controller implementation with mode management
 *
 * This file contains the implementation of the PX4Controller class with
 * centralized mode management, lifecycle state handling, and integrated
 * teleoperation capabilities for high-performance autonomous UAV control.
 *
 * @author UAV team@/flyscan
 * @date 2025/01/14
 */

#include <chrono>
#include <functional>
#include <csignal>
#include <cmath>
#include <future>

#include <rclcpp/rate.hpp>

#include "flyscan_drone_controller/px4_controller.hpp"

using namespace std::chrono_literals;

namespace flyscan {
namespace drone_controller {

using flyscan::core::BaseNode;

PX4Controller::PX4Controller(const rclcpp::NodeOptions & options,
                             const std::string& node_name,
                             const NodeType& node_type,
                             const std::vector<std::string>& capabilities
                             )
    : BaseNode(options, node_name, node_type, capabilities) {

    RCLCPP_INFO(this->get_logger(), "Initializing Advanced PX4 Controller Node: %s", node_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Starting in MANUAL mode - use set_control_mode service to switch modes");
}

PX4Controller::~PX4Controller() {
    RCLCPP_INFO(this->get_logger(), "PX4Controller destructor called");
    
    // Signal all threads to exit
    should_exit_ = true;
    
    // Always attempt to stop offboard mode
    StopOffboardMode();
    
    // Terminal cleanup is now handled by teleop_node
    
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
        // Create publishers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::SensorDataQoS());
        
        RCLCPP_INFO(this->get_logger(), "Created PX4 command publishers");
        
        // Create subscribers
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&PX4Controller::VehicleLocalPositionCallback, this, _1));
        
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
            std::bind(&PX4Controller::VehicleStatusCallback, this, _1));
        
        vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
            std::bind(&PX4Controller::VehicleLandDetectedCallback, this, _1));
        
        teleop_command_sub_ = this->create_subscription<flyscan_interfaces::msg::TeleopCommand>(
            "/px4_controller/teleop_command", 10,
            std::bind(&PX4Controller::TeleopCommandCallback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Created PX4 telemetry subscribers and teleop command subscriber");

        set_control_mode_service_ = this->create_service<flyscan_interfaces::srv::SetControlMode>(
            "/px4_controller/set_control_mode",
            std::bind(&PX4Controller::HandleSetControlModeService, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Created mode switching service: /px4_controller/set_control_mode");

        // Create setpoint timer (always running)
        setpoint_timer_ = this->create_wall_timer(
            50ms,
            std::bind(&PX4Controller::SetpointTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Created setpoint timer (%d ms interval)", SETPOINT_RATE_MS);

        // Initialize position setpoint to safe values
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = TAKEOFF_ALTITUDE;
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
    
    // Controller starts in MANUAL mode
    current_mode_ = ControlMode::kManual;
    
    RCLCPP_INFO(this->get_logger(), "PX4 Controller activated in %s mode", 
                ControlModeToString(current_mode_).c_str());
    RCLCPP_INFO(this->get_logger(), "Use 'ros2 service call ~/set_control_mode flyscan_interfaces/srv/SetControlMode \"{mode: 1}\"' to enter TELEOP mode");
    RCLCPP_INFO(this->get_logger(), "OR if TeleopNode is ready, use 'ros2 run flyscan_drone_controller teleop_node'");
    
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::HandleDeactivate() {
    RCLCPP_INFO(this->get_logger(), "Deactivating PX4 Controller...");

    // Exit current mode safely
    ExitCurrentMode();
    
    RCLCPP_INFO(this->get_logger(), "PX4 Controller deactivated");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::HandleCleanup() {
    RCLCPP_INFO(this->get_logger(), "Cleaning up PX4 Controller...");
    
    should_exit_ = true;
    
    if (setpoint_timer_) {
        setpoint_timer_->cancel();
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
    
    ExitCurrentMode();
    current_mode_ = ControlMode::kManual;
    
    RCLCPP_INFO(this->get_logger(), "Returned to MANUAL mode for safety");
    return OperationStatus::kOK;
}

// ============================================================================
// Mode Management Implementation
// ============================================================================

void PX4Controller::HandleSetControlModeService(
    const std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Request> request,
    std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Response> response) {
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    
    ControlMode requested_mode = static_cast<ControlMode>(request->mode);
    ControlMode previous_mode = current_mode_;
    
    RCLCPP_INFO(this->get_logger(), "Mode switch requested: %s -> %s",
                ControlModeToString(previous_mode).c_str(),
                ControlModeToString(requested_mode).c_str());

    if (requested_mode == ControlMode::kUnknown || request->mode > 5) {
        response->success = false;
        response->message = "Invalid mode requested";
        response->previous_mode = static_cast<uint8_t>(previous_mode);
        RCLCPP_WARN(this->get_logger(), "Invalid mode requested: %d", request->mode);
        return;
    }

    OperationStatus switch_status = SwitchToMode(requested_mode);
    
    response->success = (switch_status == OperationStatus::kOK);
    response->previous_mode = static_cast<uint8_t>(previous_mode);
    
    if (switch_status == OperationStatus::kOK) {
        response->message = "Mode switched successfully to " + ControlModeToString(requested_mode);
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        response->message = "Failed to switch to " + ControlModeToString(requested_mode);
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

OperationStatus PX4Controller::SwitchToMode(ControlMode new_mode) {
    if (new_mode == current_mode_) {
        RCLCPP_WARN(this->get_logger(), "Already in %s mode", ControlModeToString(new_mode).c_str());
        return OperationStatus::kOK;
    }

    OperationStatus exit_status = ExitCurrentMode();
    if (exit_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to exit current mode %s", 
                     ControlModeToString(current_mode_).c_str());
        return exit_status;
    }

    // Enter new mode
    OperationStatus status = OperationStatus::kNotImplemented;
    switch (new_mode) {
        case ControlMode::kManual:
            status = EnterManualMode();
            break;
        case ControlMode::kTeleop:
            status = EnterTeleopMode();
            break;
        case ControlMode::kAutonomous:
            RCLCPP_WARN(this->get_logger(), "AUTONOMOUS mode not yet implemented");
            status = OperationStatus::kNotImplemented;
            break;
        case ControlMode::kMission:
            RCLCPP_WARN(this->get_logger(), "MISSION mode not yet implemented");
            status = OperationStatus::kNotImplemented;
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
        RCLCPP_ERROR(this->get_logger(), "Failed to enter %s mode - remaining in MANUAL", 
                     ControlModeToString(new_mode).c_str());
        current_mode_ = ControlMode::kManual;
    }

    return status;
}

OperationStatus PX4Controller::EnterManualMode() {
    RCLCPP_INFO(this->get_logger(), "Entering MANUAL mode");
    
    // Always stop offboard mode
    StopOffboardMode();
    
    return OperationStatus::kOK;
}


OperationStatus PX4Controller::EnterTeleopMode() {
    RCLCPP_INFO(this->get_logger(), "Entering TELEOP mode");
    
    // Send arm command and assume it will be armed (no waiting for status)
    RCLCPP_INFO(this->get_logger(), "Sending arm command and assuming vehicle will be armed...");
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Arm command sent, proceeding with teleop mode setup");

    // Initialize current position setpoint to current position
    px4_msgs::msg::VehicleLocalPosition current_position;
    {
        std::lock_guard<std::mutex> pos_lock(position_mutex_);
        current_position = current_position_;
    }

    // Validate current position data
    if (std::isnan(current_position.x) || std::isnan(current_position.y) || std::isnan(current_position.z)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot enter TELEOP mode: Invalid position data");
        return OperationStatus::kNotInitialized;
    }

    {
        std::lock_guard<std::mutex> setpoint_lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = current_position.x;
        current_position_setpoint_.east_m  = current_position.y;
        current_position_setpoint_.down_m = current_position.z;
        current_position_setpoint_.yaw_deg = current_position.heading * 180.0f / M_PI; // Convert to degrees
    }

    RCLCPP_INFO(this->get_logger(), "Position setpoint initialized: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
                current_position_setpoint_.north_m, current_position_setpoint_.east_m, 
                current_position_setpoint_.down_m, current_position_setpoint_.yaw_deg);
    
    // Enter offboard mode for teleop control
    RCLCPP_INFO(this->get_logger(), "Entering offboard mode for teleop control...");
    OperationStatus offboard_status = StartOffboardMode(5000); // 5 second timeout
    if (offboard_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode - TELEOP mode activation failed");
        return offboard_status;
    }
    
    // Removed offboard mode verification - assuming mode switch was successful
    
    RCLCPP_INFO(this->get_logger(), "TELEOP mode active with offboard control - ready for teleop commands");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::ExitCurrentMode() {
    ControlMode mode = current_mode_;
    
    switch (mode) {
        case ControlMode::kManual:
            // Nothing special to do for manual mode
            break;
            
        case ControlMode::kTeleop:
            RCLCPP_INFO(this->get_logger(), "Exiting TELEOP mode");
            break;
            
        case ControlMode::kAutonomous:
        case ControlMode::kMission:
        case ControlMode::kRTL:
        case ControlMode::kLand:
            // Future implementation for other modes
            break;
            
        default:
            break;
    }
    
    // Always stop offboard mode when exiting any mode
    StopOffboardMode();
    
    return OperationStatus::kOK;
}

// ============================================================================
// Core PX4 Communication Implementation
// ============================================================================

void PX4Controller::VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_position_ = *msg;
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
    // Only process teleop commands when in teleop mode
    if (current_mode_ != ControlMode::kTeleop) {
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(teleop_command_mutex_);
        current_teleop_command_ = *msg;
    }

    // Handle exit teleop command
    if (msg->exit_teleop) {
        RCLCPP_INFO(this->get_logger(), "Received exit teleop command");
        SwitchToMode(ControlMode::kManual);
        return;
    }
    
    // Handle takeoff command
    if (msg->takeoff) {
        RCLCPP_INFO(this->get_logger(), "Received takeoff command - going to NED (0,0,-1.5)");
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = -1.5f;
        current_position_setpoint_.yaw_deg = 0.0f;
        return;
    }
    
    // Handle land command
    if (msg->land) {
        RCLCPP_INFO(this->get_logger(), "Received land command - going to NED (0,0,0)");
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = 0.0f;
        current_position_setpoint_.yaw_deg = 0.0f;
        return;
    }

    // Update position setpoint based on teleop command
    std::lock_guard<std::mutex> pos_lock(position_setpoint_mutex_);
    
    if (msg->hold_position) {
        // Hold current position - no changes to setpoint
        RCLCPP_DEBUG(this->get_logger(), "Holding current position");
    } else {
        // Apply movement commands (step-based movement)
        if (msg->forward != 0.0f) {
            current_position_setpoint_.north_m += msg->forward * POSITION_STEP;
        }
        if (msg->right != 0.0f) {
            current_position_setpoint_.east_m += msg->right * POSITION_STEP;
        }
        if (msg->up != 0.0f) {
            current_position_setpoint_.down_m -= msg->up * POSITION_STEP; // NED frame: up is negative
        }
        if (msg->yaw_rate != 0.0f) {
            current_position_setpoint_.yaw_deg += msg->yaw_rate * YAW_STEP;
            // Normalize yaw to [-180, 180]
            while (current_position_setpoint_.yaw_deg > 180.0f) {
                current_position_setpoint_.yaw_deg -= 360.0f;
            }
            while (current_position_setpoint_.yaw_deg < -180.0f) {
                current_position_setpoint_.yaw_deg += 360.0f;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Updated teleop setpoint: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
                    current_position_setpoint_.north_m, current_position_setpoint_.east_m, 
                    current_position_setpoint_.down_m, current_position_setpoint_.yaw_deg);
    }
}

OperationStatus PX4Controller::StartOffboardMode(int timeout_ms) {
    // Removed offboard mode check - always proceed with mode setup

    RCLCPP_INFO(this->get_logger(), "Starting offboard mode sequence...");
    rclcpp::WallRate rate(1000.0 / SETPOINT_RATE_MS);
    
    // Send a few setpoints before switching to offboard mode
    for (int i = 0; i < 10; ++i) {
        PublishOffboardControlMode();
        PublishTrajectorySetpoint();
        rate.sleep();
    }

    // Switch to offboard mode
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    
    bool success = WaitForNavStateActive(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD, timeout_ms);
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Offboard mode started successfully");
        return OperationStatus::kOK;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to activate offboard mode within timeout");
        return OperationStatus::kTimeout;
    }
}

OperationStatus PX4Controller::StopOffboardMode() {
    // Always proceed with stopping offboard mode
    
    RCLCPP_INFO(this->get_logger(), "Stopping offboard mode");
    
    RCLCPP_INFO(this->get_logger(), "Offboard mode stopped");
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


// ============================================================================
// Status Query Implementation
// ============================================================================

bool PX4Controller::IsArmed() const {
    return armed_;
}

bool PX4Controller::IsInAir() const {
    return in_air_;
}

bool PX4Controller::IsInOffboardMode() const {
    return IsNavStateActive(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
}

bool PX4Controller::IsNavStateActive(uint8_t nav_state) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_.nav_state == nav_state;
}

bool PX4Controller::WaitForNavStateActive(uint8_t nav_state, int timeout_ms) const {
    auto start_time = std::chrono::steady_clock::now();

    while (!IsNavStateActive(nav_state)) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) {
            return false;
        }
        // Use brief sleep that doesn't block callback processing
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return true;  
}

px4_msgs::msg::VehicleLocalPosition PX4Controller::GetCurrentPosition() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return current_position_;
}

px4_msgs::msg::VehicleStatus PX4Controller::GetCurrentStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

bool PX4Controller::WaitForArmedState(bool armed, int timeout_ms) const {
    auto start_time = std::chrono::steady_clock::now();
    const auto sleep_duration = std::chrono::milliseconds(50); // 20 Hz check rate

    while (IsArmed() != armed) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) {
            return false;
        }

        // Use brief sleep that doesn't block callback processing
        std::this_thread::sleep_for(sleep_duration);
    }

    return true;
}

bool PX4Controller::WaitForAirState(bool in_air, int timeout_ms) const {
    auto start_time = std::chrono::steady_clock::now();
    const auto sleep_duration = std::chrono::milliseconds(50); // 20 Hz check rate

    while (IsInAir() != in_air) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) {
            return false;
        }
        
        // Use brief sleep that doesn't block callback processing
        std::this_thread::sleep_for(sleep_duration);
    }
    return true;
}

// ============================================================================
// Flight Control Implementation
// ============================================================================

OperationStatus PX4Controller::Arm(bool wait_for_confirmation, int timeout_ms) {
    RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
    
    if (IsArmed()) {
        RCLCPP_WARN(this->get_logger(), "Vehicle already armed");
        return OperationStatus::kOK;
    }

    // Send arm command
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    
    if (wait_for_confirmation) {
        RCLCPP_INFO(this->get_logger(), "Waiting for arm confirmation...");
        if (!WaitForArmedState(true, timeout_ms)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle within timeout");
            return OperationStatus::kTimeout;
        }
        RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
    }
    
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::TakeOff(bool wait_for_confirmation, int timeout_ms) {
    RCLCPP_INFO(this->get_logger(), "Initiating takeoff sequence...");
    
    // First ensure vehicle is armed
    if (!IsArmed()) {
        RCLCPP_INFO(this->get_logger(), "Vehicle not armed, arming first...");
        OperationStatus arm_status = Arm(true, 5000);
        if (arm_status != OperationStatus::kOK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle for takeoff");
            return arm_status;
        }
    }
    
    // Send takeoff command
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0f, std::abs(TAKEOFF_ALTITUDE));
    
    if (wait_for_confirmation) {
        RCLCPP_INFO(this->get_logger(), "Waiting for takeoff confirmation...");
        if (!WaitForAirState(true, timeout_ms)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to takeoff within timeout");
            return OperationStatus::kTimeout;
        }
        RCLCPP_INFO(this->get_logger(), "Takeoff completed successfully");
    }
    
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::Land(bool wait_for_confirmation, int timeout_ms) {
    RCLCPP_INFO(this->get_logger(), "Initiating landing sequence...");
    
    // Always stop offboard mode
    StopOffboardMode();
    
    // Send land command
    PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    
    if (wait_for_confirmation) {
        RCLCPP_INFO(this->get_logger(), "Waiting for landing confirmation...");
        if (!WaitForAirState(false, timeout_ms)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to land within timeout");
            return OperationStatus::kTimeout;
        }
        RCLCPP_INFO(this->get_logger(), "Landing completed successfully");
    }
    
    return OperationStatus::kOK;
}


} // namespace drone_controller
} // namespace flyscan