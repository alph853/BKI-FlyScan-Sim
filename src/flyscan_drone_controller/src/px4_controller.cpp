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

#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "flyscan_drone_controller/px4_controller.hpp"
#include "flyscan_drone_controller/constants.hpp"
#include "flyscan_common/sigint_handler.hpp"

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
    this->declare_parameter("setpoint_rate_ms", 50);
    this->declare_parameter("takeoff_altitude", -1.5f);  // NED frame (negative is up)
    this->declare_parameter("position_step", 0.5f);      // meters per step
    this->declare_parameter("yaw_step", 15.0f);          // degrees per step
    this->declare_parameter("eight_shape_speed", 0.5f);  // m/s for 8-shape pattern
    this->declare_parameter("initial_control_mode", static_cast<int>(ControlMode::kManual));
    
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
        position_step_ = this->get_parameter("position_step").as_double();
        yaw_step_ = this->get_parameter("yaw_step").as_double();
        initial_control_mode_ = static_cast<ControlMode>(this->get_parameter("initial_control_mode").as_int());
        
        RCLCPP_INFO(this->get_logger(), "Cached parameters: setpoint_rate=%dms, takeoff_alt=%.2f, pos_step=%.2f, yaw_step=%.1f, initial_mode=%s",
                   setpoint_rate_ms_, takeoff_altitude_, position_step_, yaw_step_, 
                   ControlModeToString(initial_control_mode_).c_str());
                   
        auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
 
        // Create publishers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            topic::PX4_OFFBOARD_CONTROL_MODE, qos);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            topic::PX4_TRAJECTORY_SETPOINT, qos);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            topic::PX4_VEHICLE_COMMAND, qos);

        RCLCPP_INFO(this->get_logger(), "Created PX4 command publishers");
        
        // Create subscribers
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            topic::PX4_VEHICLE_LOCAL_POSITION, qos,
            std::bind(&PX4Controller::VehicleLocalPositionCallback, this, _1));

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            topic::PX4_VEHICLE_STATUS, qos,
            std::bind(&PX4Controller::VehicleStatusCallback, this, _1));

        vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            topic::PX4_VEHICLE_LAND_DETECTED, qos,
            std::bind(&PX4Controller::VehicleLandDetectedCallback, this, _1));
        
        teleop_command_sub_ = this->create_subscription<flyscan_interfaces::msg::TeleopCommand>(
            topic::TELEOP_COMMAND, qos,
            std::bind(&PX4Controller::TeleopCommandCallback, this, _1));

        exploration_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic::EXPLORATION_GOAL, qos,
            std::bind(&PX4Controller::ExplorationGoalCallback, this, _1));

        // create services
        set_control_mode_service_ = this->create_service<flyscan_interfaces::srv::SetControlMode>(
            srv::SET_CONTROL_MODE,
            std::bind(&PX4Controller::HandleSetControlModeService, this, _1, _2));

        // Create timers
        setpoint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(setpoint_rate_ms_),
            std::bind(&PX4Controller::SetpointTimerCallback, this));
        setpoint_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Created setpoint timer (%d ms interval)", setpoint_rate_ms_);

        // Initialize position setpoint to safe values
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = takeoff_altitude_;
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
    std::shared_ptr<flyscan_interfaces::srv::SetControlMode::Response> response) {
    
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
            break;
        case ControlMode::kAutonomous:
            status = EnterAutonomousMode();
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
    
    return OperationStatus::kOK;
}


OperationStatus PX4Controller::EnterTeleopMode() {
    RCLCPP_INFO(this->get_logger(), "Entering TELEOP mode");

    OperationStatus offboard_status = StartOffboardMode();
    if (offboard_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode - TELEOP mode activation failed");
        return offboard_status;
    }

    RCLCPP_INFO(this->get_logger(), "TELEOP mode active with offboard control - ready for teleop commands");
    return OperationStatus::kOK;
}

OperationStatus PX4Controller::EnterAutonomousMode() {
    RCLCPP_INFO(this->get_logger(), "Entering AUTONOMOUS mode for exploration");

    if (current_mode_ == ControlMode::kAutonomous) {
        return OperationStatus::kOK;
    }

    OperationStatus offboard_status = StartOffboardMode();
    if (offboard_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode - AUTONOMOUS mode activation failed");
        return offboard_status;
    }

    RCLCPP_INFO(this->get_logger(), "AUTONOMOUS mode active - ready for exploration goals");
    return OperationStatus::kOK;
}

void PX4Controller::ExplorationGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (current_mode_ != ControlMode::kAutonomous) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received exploration goal: (%.2f, %.2f, %.2f)", 
               msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    {
        std::lock_guard<std::mutex> lock(exploration_goal_mutex_);
        current_exploration_goal_ = *msg;
    }

    {
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = msg->pose.position.x;
        current_position_setpoint_.east_m = msg->pose.position.y;
        current_position_setpoint_.down_m = msg->pose.position.z;
        
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_position_setpoint_.yaw_deg = yaw * 180.0f / M_PI;
    }

    RCLCPP_INFO(this->get_logger(), "Updated autonomous setpoint: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
                current_position_setpoint_.north_m, current_position_setpoint_.east_m, 
                current_position_setpoint_.down_m, current_position_setpoint_.yaw_deg);
}

// ============================================================================
// Core PX4 Communication Implementation
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
        RCLCPP_INFO(this->get_logger(), "Received takeoff command - going to NED (0,0,-1.5)");
        std::lock_guard<std::mutex> lock(position_setpoint_mutex_);
        current_position_setpoint_.north_m = 0.0f;
        current_position_setpoint_.east_m = 0.0f;
        current_position_setpoint_.down_m = -1.5f;
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

    std::lock_guard<std::mutex> pos_lock(position_setpoint_mutex_);
    
    if (msg->command == "hold_position" || msg->command == "stop") {
        RCLCPP_INFO(this->get_logger(), "Holding current position");
    } else {
        // Apply movement commands (step-based movement)
        
        if (msg->command == "forward") {
            current_position_setpoint_.north_m += position_step_;
        } else if (msg->command == "backward") {
            current_position_setpoint_.north_m -= position_step_;
        } else if (msg->command == "right") {
            current_position_setpoint_.east_m += position_step_;
        } else if (msg->command == "left") {
            current_position_setpoint_.east_m -= position_step_;
        } else if (msg->command == "up") {
            current_position_setpoint_.down_m -= position_step_;
        } else if (msg->command == "down") {
            current_position_setpoint_.down_m += position_step_;
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

        RCLCPP_INFO(this->get_logger(), "Updated teleop setpoint: N=%.2f, E=%.2f, D=%.2f, Yaw=%.1f", 
                    current_position_setpoint_.north_m, current_position_setpoint_.east_m, 
                    current_position_setpoint_.down_m, current_position_setpoint_.yaw_deg);
    }
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
        RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), 
                    "Use: ros2 service call /px4_controller/set_control_mode flyscan_interfaces/srv/SetControlMode \"{mode: 1}\"");
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