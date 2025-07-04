#pragma once

#include "flyscan_drone_controller/px4_controller.hpp"
#include "flyscan_drone_controller/util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <atomic>

namespace flyscan_drone_controller {

class TeleopController : public rclcpp::Node {
public:
    explicit TeleopController(const std::string& connection_url);
    ~TeleopController();
    
    void run();

private:
    void teleopControlLoop();
    
    std::string connection_url_;
    std::unique_ptr<PX4Controller> px4_controller_;
    std::atomic<bool> should_exit_{false};
    bool initialized_{false};
    
    static constexpr float POSITION_STEP = 0.5f;
    static constexpr float YAW_STEP = 10.0f;
};

} // namespace flyscan_drone_controller