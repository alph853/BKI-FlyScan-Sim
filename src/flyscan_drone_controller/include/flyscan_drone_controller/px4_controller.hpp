#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <iostream>
#include <chrono>

namespace flyscan_drone_controller {

struct TeleopCommand {
    float north_m = 0.0f;
    float east_m = 0.0f;
    float down_m = 0.0f;
    float yaw_deg = 0.0f;
};

class PX4Controller : public rclcpp::Node {
public:
    explicit PX4Controller(const std::string& node_name = "px4_controller");
    ~PX4Controller();

    bool initialize();
    
    void takeoffToPosition();
    void stopOffboardMode();
    void updateTeleopCommand(const TeleopCommand& cmd);
    void land();
    
    bool isArmed() const;
    bool isInAir() const;
    bool isOffboardActive() const;
    
    px4_msgs::msg::VehicleLocalPosition getCurrentPosition() const;
    px4_msgs::msg::VehicleStatus getCurrentStatus() const;

private:
    void setupPublishersAndSubscribers();
    void setupTimers();
    
    void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    
    void startOffboardMode();
    void setpointTimerCallback();
    
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
    
    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    
    std::atomic<bool> should_exit_{false};
    std::atomic<bool> offboard_active_{false};
    std::atomic<bool> takeoff_complete_{false};
    std::atomic<bool> armed_{false};
    std::atomic<bool> in_air_{false};
    
    TeleopCommand current_teleop_command_;
    std::mutex teleop_command_mutex_;
    
    px4_msgs::msg::VehicleLocalPosition current_position_;
    px4_msgs::msg::VehicleStatus current_status_;
    mutable std::mutex position_mutex_;
    mutable std::mutex status_mutex_;
    
    uint64_t offboard_setpoint_counter_{0};
    
    static constexpr float TAKEOFF_ALTITUDE = -1.5f;
    static constexpr float POSITION_STEP = 0.5f;
    static constexpr float YAW_STEP = 10.0f;
    static constexpr int SETPOINT_RATE_MS = 50;
};

} // namespace flyscan_drone_controller