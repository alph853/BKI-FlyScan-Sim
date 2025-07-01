/**
 * @file controller_node.cpp
 * @brief Implementation of ROS2 controller node
 * @author Your Name
 * @date 2025
 */

#include "flyscan_drone_controller/controller_node.hpp"

ControllerNode::ControllerNode() : Node("px4_controller_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing PX4 Controller Node...");
    
    // Initialize parameters
    initializeParameters();
    
    // Create PX4Controller instance
    px4_controller_ = std::make_unique<PX4Controller>();
    
    // Setup ROS2 interfaces
    setupRosInterfaces();
    
    // Setup controller callbacks
    setupControllerCallbacks();
    
    // Auto-connect if enabled
    if (auto_connect_) {
        if (connectToAutopilot()) {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to autopilot");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to autopilot");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "PX4 Controller Node initialized successfully");
}

ControllerNode::~ControllerNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down PX4 Controller Node...");
    
    if (px4_controller_) {
        px4_controller_->requestShutdown();
    }
}

void ControllerNode::initializeParameters() {
    // Declare parameters with default values
    this->declare_parameter<std::string>("connection_url", "udp://:14540");
    this->declare_parameter<double>("status_publish_rate", 1.0);
    this->declare_parameter<bool>("auto_connect", true);
    
    // Get parameter values
    connection_url_ = this->get_parameter("connection_url").as_string();
    status_publish_rate_ = this->get_parameter("status_publish_rate").as_double();
    auto_connect_ = this->get_parameter("auto_connect").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  connection_url: %s", connection_url_.c_str());
    RCLCPP_INFO(this->get_logger(), "  status_publish_rate: %.1f Hz", status_publish_rate_);
    RCLCPP_INFO(this->get_logger(), "  auto_connect: %s", auto_connect_ ? "true" : "false");
}

void ControllerNode::setupRosInterfaces() {
    // Publishers
    position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/position", 10);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "~/battery", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/status", 10);
    log_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/log", 10);
    
    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", 10,
        std::bind(&ControllerNode::cmdVelCallback, this, std::placeholders::_1));
    
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "~/command", 10,
        std::bind(&ControllerNode::commandCallback, this, std::placeholders::_1));
    
    // Services
    arm_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/arm",
        std::bind(&ControllerNode::armService, this, std::placeholders::_1, std::placeholders::_2));
    
    disarm_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/disarm",
        std::bind(&ControllerNode::disarmService, this, std::placeholders::_1, std::placeholders::_2));
    
    takeoff_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/takeoff",
        std::bind(&ControllerNode::takeoffService, this, std::placeholders::_1, std::placeholders::_2));
    
    land_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/land",
        std::bind(&ControllerNode::landService, this, std::placeholders::_1, std::placeholders::_2));
    
    rtl_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/rtl",
        std::bind(&ControllerNode::rtlService, this, std::placeholders::_1, std::placeholders::_2));
    
    teleop_service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/teleop",
        std::bind(&ControllerNode::teleopService, this, std::placeholders::_1, std::placeholders::_2));
    
    sample_flight_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/sample_flight",
        std::bind(&ControllerNode::sampleFlightService, this, std::placeholders::_1, std::placeholders::_2));
    
    // Timers
    auto timer_period = std::chrono::duration<double>(1.0 / status_publish_rate_);
    status_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&ControllerNode::statusTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "ROS2 interfaces setup complete");
}

void ControllerNode::setupControllerCallbacks() {
    if (!px4_controller_) return;
    
    // Set log callback
    px4_controller_->setLogCallback(
        std::bind(&ControllerNode::onLogMessage, this, std::placeholders::_1));
    
    // Set position callback
    px4_controller_->setPositionCallback(
        std::bind(&ControllerNode::onPositionUpdate, this, 
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    // Set battery callback
    px4_controller_->setBatteryCallback(
        std::bind(&ControllerNode::onBatteryUpdate, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Controller callbacks setup complete");
}

bool ControllerNode::connectToAutopilot() {
    if (!px4_controller_) {
        RCLCPP_ERROR(this->get_logger(), "PX4Controller not initialized");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Connecting to autopilot at: %s", connection_url_.c_str());
    
    if (px4_controller_->connect(connection_url_)) {
        px4_controller_->setupTelemetry();
        RCLCPP_INFO(this->get_logger(), "Connected and telemetry setup complete");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to autopilot");
        return false;
    }
}

// === ROS2 Callback Functions ===

void ControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!px4_controller_ || !px4_controller_->isConnected()) {
        return;
    }
    
    // Convert ROS2 Twist to MAVSDK velocity command
    // Note: This assumes the drone is in offboard mode
    px4_controller_->setOffboardVelocity(
        msg->linear.x,   // Forward/backward
        msg->linear.y,   // Left/right
        -msg->linear.z,  // Up/down (ROS uses +Z up, MAVSDK uses +Z down)
        msg->angular.z * 180.0 / M_PI  // Yaw rate (convert rad/s to deg/s)
    );
}

void ControllerNode::commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!px4_controller_ || !px4_controller_->isConnected()) {
        RCLCPP_WARN(this->get_logger(), "Cannot execute command - not connected");
        return;
    }
    
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());
    
    // Simple command parser
    if (command == "arm") {
        px4_controller_->arm();
    } else if (command == "disarm") {
        px4_controller_->disarm();
    } else if (command == "takeoff") {
        px4_controller_->takeoff();
    } else if (command == "land") {
        px4_controller_->land();
    } else if (command == "rtl") {
        px4_controller_->returnToLaunch();
    } else if (command == "emergency_stop") {
        px4_controller_->emergencyStop();
    } else if (command == "sample_flight") {
        px4_controller_->executeSampleFlight();
    } else if (command == "take_photo") {
        px4_controller_->takePhoto();
    } else if (command == "status") {
        px4_controller_->printStatus();
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
    }
}

void ControllerNode::statusTimerCallback() {
    publishStatus();
}

// === ROS2 Service Callbacks ===

void ControllerNode::armService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (px4_controller_ && px4_controller_->isConnected()) {
        bool success = px4_controller_->arm();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Arm service completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arm service failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot arm - not connected to autopilot");
    }
}

void ControllerNode::disarmService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (px4_controller_ && px4_controller_->isConnected()) {
        bool success = px4_controller_->disarm();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Disarm service completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Disarm service failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot disarm - not connected to autopilot");
    }
}

void ControllerNode::takeoffService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (px4_controller_ && px4_controller_->isConnected()) {
        bool success = px4_controller_->takeoff();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Takeoff service completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Takeoff service failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot takeoff - not connected to autopilot");
    }
}

void ControllerNode::landService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (px4_controller_ && px4_controller_->isConnected()) {
        bool success = px4_controller_->land();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Land service completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Land service failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot land - not connected to autopilot");
    }
}

void ControllerNode::rtlService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (px4_controller_ && px4_controller_->isConnected()) {
        bool success = px4_controller_->returnToLaunch();
        if (success) {
            RCLCPP_INFO(this->get_logger(), "RTL service completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "RTL service failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot RTL - not connected to autopilot");
    }
}

void ControllerNode::teleopService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (!px4_controller_ || !px4_controller_->isConnected()) {
        response->success = false;
        response->message = "Not connected to autopilot";
        return;
    }
    
    if (request->data) {
        // Start teleop mode
        px4_controller_->startTeleopMode();
        response->success = true;
        response->message = "Teleop mode started";
        RCLCPP_INFO(this->get_logger(), "Teleop mode started via service");
    } else {
        // Stop teleop mode
        px4_controller_->stopTeleopMode();
        response->success = true;
        response->message = "Teleop mode stopped";
        RCLCPP_INFO(this->get_logger(), "Teleop mode stopped via service");
    }
}

void ControllerNode::sampleFlightService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (px4_controller_ && px4_controller_->isConnected()) {
        px4_controller_->executeSampleFlight();
        RCLCPP_INFO(this->get_logger(), "Sample flight started via service");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot start sample flight - not connected to autopilot");
    }
}

// === PX4Controller Callback Functions ===

void ControllerNode::onLogMessage(const std::string& message) {
    // Publish to ROS2 log topic
    auto msg = std_msgs::msg::String();
    msg.data = message;
    log_pub_->publish(msg);
    
    // Also log to ROS2 console (with reduced verbosity)
    static int log_counter = 0;
    if (log_counter++ % 10 == 0) { // Only log every 10th message to avoid spam
        RCLCPP_INFO(this->get_logger(), "PX4: %s", message.c_str());
    }
}

void ControllerNode::onPositionUpdate(float lat, float lon, float alt) {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    
    // Note: This is a simplified conversion
    // For proper GPS->local coordinate conversion, use a proper transform
    msg.pose.position.x = lon;  // Longitude as X (simplified)
    msg.pose.position.y = lat;  // Latitude as Y (simplified)
    msg.pose.position.z = alt;  // Altitude as Z
    
    // Orientation would need attitude data
    msg.pose.orientation.w = 1.0;
    
    position_pub_->publish(msg);
}

void ControllerNode::onBatteryUpdate(float battery_percent) {
    auto msg = sensor_msgs::msg::BatteryState();
    msg.header.stamp = this->get_clock()->now();
    msg.percentage = battery_percent / 100.0f;
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    
    battery_pub_->publish(msg);
}

// === Utility Functions ===

void ControllerNode::publishStatus() {
    if (!px4_controller_) return;
    
    auto msg = std_msgs::msg::String();
    
    if (px4_controller_->isConnected()) {
        msg.data = "Connected|" + flightModeToString(px4_controller_->getCurrentMode());
    } else {
        msg.data = "Disconnected|IDLE";
    }
    
    status_pub_->publish(msg);
}

std::string ControllerNode::flightModeToString(FlightMode mode) {
    switch (mode) {
        case FlightMode::IDLE: return "IDLE";
        case FlightMode::MANUAL_TELEOP: return "TELEOP";
        case FlightMode::SAMPLE_FLIGHT: return "SAMPLE_FLIGHT";
        case FlightMode::MISSION: return "MISSION";
        case FlightMode::OFFBOARD_DEMO: return "OFFBOARD_DEMO";
        default: return "UNKNOWN";
    }
}

// === Main Function for Node ===

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ControllerNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Node error: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}