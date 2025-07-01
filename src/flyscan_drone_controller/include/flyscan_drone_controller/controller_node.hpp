/**
 * @file controller_node.hpp
 * @brief ROS2 node wrapper for PX4Controller
 * @author Your Name
 * @date 2025
 * 
 * This ROS2 node provides a simple interface to the PX4Controller class,
 * keeping the MAVSDK functionality decoupled from ROS2.
 */

#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "px4_controller.hpp"
#include <memory>
#include <string>

/**
 * @class ControllerNode
 * @brief Simple ROS2 node that wraps the PX4Controller
 * 
 * This node provides basic ROS2 interfaces while keeping the core
 * MAVSDK functionality independent. It's designed to be minimal
 * and easily extensible for future functionality.
 */
class ControllerNode : public rclcpp::Node {
private:
    // Core controller instance
    std::unique_ptr<PX4Controller> px4_controller_;
    
    // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    
    // ROS2 Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    
    // ROS2 Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr arm_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disarm_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr takeoff_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr land_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr rtl_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sample_flight_service_;
    
    // ROS2 Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Node parameters
    std::string connection_url_;
    double status_publish_rate_;
    bool auto_connect_;

public:
    /**
     * @brief Constructor
     */
    ControllerNode();
    
    /**
     * @brief Destructor
     */
    ~ControllerNode();

private:
    /**
     * @brief Initialize ROS2 parameters
     */
    void initializeParameters();
    
    /**
     * @brief Setup ROS2 publishers, subscribers, and services
     */
    void setupRosInterfaces();
    
    /**
     * @brief Setup callbacks for PX4Controller
     */
    void setupControllerCallbacks();
    
    /**
     * @brief Connect to PX4 autopilot
     * @return true if successful
     */
    bool connectToAutopilot();
    
    // === ROS2 Callback Functions ===
    
    /**
     * @brief Velocity command callback (for teleop via ROS2)
     * @param msg Twist message with velocity commands
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief String command callback for high-level commands
     * @param msg String command message
     */
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    
    /**
     * @brief Timer callback for periodic status publishing
     */
    void statusTimerCallback();
    
    // === ROS2 Service Callbacks ===
    
    /**
     * @brief Arm service callback
     */
    void armService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief Disarm service callback
     */
    void disarmService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief Takeoff service callback
     */
    void takeoffService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief Land service callback
     */
    void landService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief Return to launch service callback
     */
    void rtlService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief Teleop mode service callback
     */
    void teleopService(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    /**
     * @brief Sample flight service callback
     */
    void sampleFlightService(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    // === PX4Controller Callback Functions ===
    
    /**
     * @brief Callback for log messages from PX4Controller
     * @param message Log message
     */
    void onLogMessage(const std::string& message);
    
    /**
     * @brief Callback for position updates from PX4Controller
     * @param lat Latitude (degrees)
     * @param lon Longitude (degrees) 
     * @param alt Altitude (meters)
     */
    void onPositionUpdate(float lat, float lon, float alt);
    
    /**
     * @brief Callback for battery updates from PX4Controller
     * @param battery_percent Battery percentage (0-100)
     */
    void onBatteryUpdate(float battery_percent);
    
    // === Utility Functions ===
    
    /**
     * @brief Publish current system status
     */
    void publishStatus();
    
    /**
     * @brief Convert FlightMode enum to string
     * @param mode Flight mode
     * @return String representation
     */
    std::string flightModeToString(FlightMode mode);
};

#endif // CONTROLLER_NODE_HPP