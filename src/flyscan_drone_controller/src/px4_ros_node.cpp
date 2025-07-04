#include "flyscan_drone_controller/px4_ros_node.hpp"
#include "flyscan_drone_controller/util.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace flyscan_drone_controller {

PX4RosNode::PX4RosNode(const std::string& connection_url)
    : Node("px4_ros_node") {
    
    RCLCPP_INFO(this->get_logger(), "Initializing PX4 ROS Node with connection: %s", connection_url.c_str());
    
    px4_controller_ = std::make_unique<PX4Controller>("px4_controller_internal");
    setupRosInterfaces();
}

PX4RosNode::~PX4RosNode() {
    RCLCPP_INFO(this->get_logger(), "PX4 ROS Node shutting down");
}

bool PX4RosNode::initialize() {
    if (!px4_controller_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PX4 Controller");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "PX4 ROS Node initialized successfully");
    return true;
}

void PX4RosNode::setupRosInterfaces() {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&PX4RosNode::cmdVelCallback, this, std::placeholders::_1));
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    armed_pub_ = this->create_publisher<std_msgs::msg::Bool>("armed", 10);
    in_air_pub_ = this->create_publisher<std_msgs::msg::Bool>("in_air", 10);
    
    arm_service_ = this->create_service<std_srvs::srv::Trigger>(
        "arm", std::bind(&PX4RosNode::armService, this, std::placeholders::_1, std::placeholders::_2));
    
    takeoff_service_ = this->create_service<std_srvs::srv::Trigger>(
        "takeoff", std::bind(&PX4RosNode::takeoffService, this, std::placeholders::_1, std::placeholders::_2));
    
    land_service_ = this->create_service<std_srvs::srv::Trigger>(
        "land", std::bind(&PX4RosNode::landService, this, std::placeholders::_1, std::placeholders::_2));
    
    telemetry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / TELEMETRY_RATE_HZ)),
        std::bind(&PX4RosNode::publishTelemetry, this));
}

void PX4RosNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!px4_controller_->isOffboardActive()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Received cmd_vel but offboard mode not active");
        return;
    }
    
    // Get current position and apply velocity command as incremental position changes
    auto current_pos = px4_controller_->getCurrentPosition();
    
    TeleopCommand cmd;
    cmd.north_m = current_pos.x + (msg->linear.x * POSITION_STEP);
    cmd.east_m = current_pos.y + (msg->linear.y * POSITION_STEP);
    cmd.down_m = current_pos.z + (msg->linear.z * POSITION_STEP);
    cmd.yaw_deg = msg->angular.z * YAW_STEP;  // Direct yaw command
    cmd.yaw_deg = util::MathUtils::constrainAngle(cmd.yaw_deg);
    
    px4_controller_->updateTeleopCommand(cmd);
}

void PX4RosNode::armService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Note: PX4Controller handles arming internally during takeoff
    // This service is provided for compatibility
    response->success = px4_controller_->isArmed();
    response->message = px4_controller_->isArmed() ? "Already armed" : "Use takeoff service to arm and takeoff";
}

void PX4RosNode::takeoffService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
        px4_controller_->takeoffToPosition();
        response->success = true;
        response->message = "Takeoff initiated";
        RCLCPP_INFO(this->get_logger(), "Takeoff service called successfully");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Takeoff failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "Takeoff service failed: %s", e.what());
    }
}

void PX4RosNode::landService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
        px4_controller_->land();
        response->success = true;
        response->message = "Landing initiated";
        RCLCPP_INFO(this->get_logger(), "Land service called successfully");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Landing failed: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "Land service failed: %s", e.what());
    }
}

void PX4RosNode::publishTelemetry() {
    try {
        auto current_pos = px4_controller_->getCurrentPosition();
        auto current_status = px4_controller_->getCurrentStatus();
        
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = current_pos.x;
        pose_msg.pose.position.y = current_pos.y;
        pose_msg.pose.position.z = -current_pos.z;
        pose_pub_->publish(pose_msg);
        
        // Note: GPS coordinates not directly available from VehicleLocalPosition
        // Would need to subscribe to VehicleGlobalPosition separately
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = "gps";
        gps_msg.latitude = 0.0;  // Placeholder
        gps_msg.longitude = 0.0; // Placeholder
        gps_msg.altitude = 0.0;  // Placeholder
        gps_pub_->publish(gps_msg);
        
        auto armed_msg = std_msgs::msg::Bool();
        armed_msg.data = px4_controller_->isArmed();
        armed_pub_->publish(armed_msg);
        
        auto in_air_msg = std_msgs::msg::Bool();
        in_air_msg.data = px4_controller_->isInAir();
        in_air_pub_->publish(in_air_msg);
        
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Error publishing telemetry: %s", e.what());
    }
}

} // namespace flyscan_drone_controller