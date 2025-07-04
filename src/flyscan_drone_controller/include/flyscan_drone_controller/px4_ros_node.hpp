#pragma once

#include "flyscan_drone_controller/px4_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <memory>
#include <string>

namespace flyscan_drone_controller {

class PX4RosNode : public rclcpp::Node {
public:
    explicit PX4RosNode(const std::string& connection_url);
    ~PX4RosNode();

    bool initialize();

private:
    void setupRosInterfaces();
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void armService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void takeoffService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void landService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void publishTelemetry();
    
    std::unique_ptr<PX4Controller> px4_controller_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr in_air_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
    
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    
    static constexpr double TELEMETRY_RATE_HZ = 1.0;
    static constexpr float POSITION_STEP = 0.5f;
    static constexpr float YAW_STEP = 10.0f;
};

} // namespace flyscan_drone_controller