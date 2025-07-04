#include "flyscan_drone_controller/px4_controller.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace flyscan_drone_controller {

PX4Controller::PX4Controller(const std::string& node_name)
    : Node(node_name) {
    
    RCLCPP_INFO(this->get_logger(), "Initializing PX4 Controller");
    setupPublishersAndSubscribers();
    setupTimers();
}

PX4Controller::~PX4Controller() {
    should_exit_ = true;
    
    if (offboard_active_) {
        stopOffboardMode();
    }
}

void PX4Controller::setupPublishersAndSubscribers() {
    RCLCPP_INFO(this->get_logger(), "Setting up publishers and subscribers...");
    
    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", rclcpp::SensorDataQoS());
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", rclcpp::SensorDataQoS());
    
    RCLCPP_INFO(this->get_logger(), "Created publishers for offboard_control_mode, trajectory_setpoint, and vehicle_command");
    
    // Subscribers
    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        std::bind(&PX4Controller::vehicleLocalPositionCallback, this, std::placeholders::_1));
    
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
        std::bind(&PX4Controller::vehicleStatusCallback, this, std::placeholders::_1));
    
    vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
        std::bind(&PX4Controller::vehicleLandDetectedCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Created subscribers for vehicle_local_position, vehicle_status, and vehicle_land_detected");
}

void PX4Controller::setupTimers() {
    RCLCPP_INFO(this->get_logger(), "Setting up timers...");
    // Timer for setpoint streaming
    setpoint_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(SETPOINT_RATE_MS),
        std::bind(&PX4Controller::setpointTimerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Created setpoint timer with %d ms interval", SETPOINT_RATE_MS);
}

bool PX4Controller::initialize() {
    RCLCPP_INFO(this->get_logger(), "PX4 Controller initialized successfully");
    return true;
}

void PX4Controller::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // RCLCPP_DEBUG(this->get_logger(), "Received vehicle local position: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_position_ = *msg;
}

void PX4Controller::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received vehicle status: arming_state=%d, nav_state=%d", 
    //             msg->arming_state, msg->nav_state);
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_ = *msg;
    bool was_armed = armed_;
    armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    if (was_armed != armed_) {
        RCLCPP_INFO(this->get_logger(), "Armed state changed: %s", armed_ ? "ARMED" : "DISARMED");
    }
}

void PX4Controller::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    bool was_in_air = in_air_;
    in_air_ = !msg->landed;
    if (was_in_air != in_air_) {
        RCLCPP_INFO(this->get_logger(), "In-air state changed: %s", in_air_ ? "IN AIR" : "LANDED");
    }
}

void PX4Controller::takeoffToPosition() {
    RCLCPP_INFO(this->get_logger(), "Starting takeoff sequence");
    RCLCPP_INFO(this->get_logger(), "Current armed state: %s, in_air: %s", 
                armed_ ? "ARMED" : "DISARMED", in_air_ ? "IN AIR" : "LANDED");
    
    // Set default takeoff position
    current_teleop_command_.north_m = 0.0f;
    current_teleop_command_.east_m = 0.0f;
    current_teleop_command_.down_m = TAKEOFF_ALTITUDE;
    current_teleop_command_.yaw_deg = 0.0f;
    
    RCLCPP_INFO(this->get_logger(), "Set takeoff position: [%.2f, %.2f, %.2f, %.2f deg]",
                current_teleop_command_.north_m, current_teleop_command_.east_m, 
                current_teleop_command_.down_m, current_teleop_command_.yaw_deg);
    
    // If not already armed, arm the vehicle
    if (!armed_) {
        RCLCPP_INFO(this->get_logger(), "Vehicle not armed, sending arm command...");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        
        // Wait a bit for arming to complete
        RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for arming to complete...");
        std::this_thread::sleep_for(2s);
        RCLCPP_INFO(this->get_logger(), "Armed state after wait: %s", armed_ ? "ARMED" : "DISARMED");
    } else {
        RCLCPP_INFO(this->get_logger(), "Vehicle already armed");
    }
    
    startOffboardMode();
    takeoff_complete_ = true;
    RCLCPP_INFO(this->get_logger(), "Takeoff sequence completed");
}

void PX4Controller::startOffboardMode() {
    if (offboard_active_) {
        RCLCPP_WARN(this->get_logger(), "Offboard mode already active");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting offboard mode sequence...");
    
    // Send a few setpoints before starting offboard mode
    RCLCPP_INFO(this->get_logger(), "Sending 10 initial setpoints before switching to offboard...");
    for (int i = 0; i < 10; ++i) {
        publishOffboardControlMode();
        publishTrajectorySetpoint();
        std::this_thread::sleep_for(std::chrono::milliseconds(SETPOINT_RATE_MS));
    }
    
    // Switch to offboard mode
    RCLCPP_INFO(this->get_logger(), "Sending command to switch to offboard mode (VEHICLE_CMD_DO_SET_MODE)");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    
    offboard_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Offboard mode started - continuous setpoint streaming will begin");
}

void PX4Controller::stopOffboardMode() {
    if (!offboard_active_) return;
    
    RCLCPP_INFO(this->get_logger(), "Stopping offboard mode");
    
    should_exit_ = true;
    offboard_active_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Offboard mode stopped");
}

void PX4Controller::setpointTimerCallback() {
    if (!offboard_active_ || should_exit_) {
        // RCLCPP_DEBUG(this->get_logger(), "Skipping setpoint - offboard_active: %s, should_exit: %s", 
        //              offboard_active_ ? "true" : "false", should_exit_ ? "true" : "false");
        return;
    }
    
    // RCLCPP_DEBUG(this->get_logger(), "Timer callback - publishing setpoints");
    publishOffboardControlMode();
    publishTrajectorySetpoint();
}

void PX4Controller::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    offboard_control_mode_publisher_->publish(msg);
}

void PX4Controller::publishTrajectorySetpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    
    TeleopCommand cmd;
    {
        std::lock_guard<std::mutex> lock(teleop_command_mutex_);
        cmd = current_teleop_command_;
    }
    
    msg.position = {cmd.north_m, cmd.east_m, cmd.down_m};
    msg.yaw = cmd.yaw_deg * M_PI / 180.0f; // Convert to radians
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    // RCLCPP_DEBUG(this->get_logger(), "Publishing TrajectorySetpoint: pos=[%.2f, %.2f, %.2f], yaw=%.2f rad, timestamp=%lu",
    //              msg.position[0], msg.position[1], msg.position[2], msg.yaw, msg.timestamp);
    trajectory_setpoint_publisher_->publish(msg);
}

void PX4Controller::publishVehicleCommand(uint16_t command, float param1, float param2) {
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
    
    RCLCPP_INFO(this->get_logger(), "Publishing VehicleCommand: cmd=%d, param1=%.2f, param2=%.2f, timestamp=%lu",
                command, param1, param2, msg.timestamp);
    vehicle_command_publisher_->publish(msg);
}

void PX4Controller::updateTeleopCommand(const TeleopCommand& cmd) {
    RCLCPP_DEBUG(this->get_logger(), "Updating teleop command: [%.2f, %.2f, %.2f, %.2f deg]",
                 cmd.north_m, cmd.east_m, cmd.down_m, cmd.yaw_deg);
    std::lock_guard<std::mutex> lock(teleop_command_mutex_);
    current_teleop_command_ = cmd;
}

void PX4Controller::land() {
    RCLCPP_INFO(this->get_logger(), "Initiating landing sequence...");
    
    stopOffboardMode();
    
    RCLCPP_INFO(this->get_logger(), "Sending VEHICLE_CMD_NAV_LAND command");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    
    RCLCPP_INFO(this->get_logger(), "Landing command sent successfully");
}

bool PX4Controller::isArmed() const {
    return armed_;
}

bool PX4Controller::isInAir() const {
    return in_air_;
}

bool PX4Controller::isOffboardActive() const {
    return offboard_active_;
}

px4_msgs::msg::VehicleLocalPosition PX4Controller::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return current_position_;
}

px4_msgs::msg::VehicleStatus PX4Controller::getCurrentStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

} // namespace flyscan_drone_controller