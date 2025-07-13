/**
 * @file px4_controller_main.cpp
 * @brief Main entry point for the centralized PX4 Controller node
 *
 * This file contains the main function that initializes and runs the PX4Controller
 * node with lifecycle management and mode switching capabilities.
 *
 * @author UAV team@/flyscan
 * @date 2025/01/14
 */

#include <memory>
#include <csignal>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "flyscan_drone_controller/px4_controller.hpp"
#include "flyscan_drone_controller/terminal_controller.hpp"

using flyscan::drone_controller::PX4Controller;

void signalHandler(int signal) {
    RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), 
                "Received signal %d, shutting down gracefully...", signal);
    
    // Cleanup terminal controller
    flyscan_drone_controller::util::TerminalController::cleanup();
    
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Setup signal handlers for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    try {
        // Create PX4Controller node
        auto controller = std::make_shared<PX4Controller>();

        RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), 
                    "Starting PX4 Controller with lifecycle management");
        
        // Use MultiThreadedExecutor for service calls and lifecycle management
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(controller->get_node_base_interface());
        
        // Configure the node
        auto configure_result = controller->configure();
        if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_ERROR(rclcpp::get_logger("px4_controller_main"), "Failed to configure PX4Controller");
            return 1;
        }

        // Activate the node
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

        // Spin the executor
        executor.spin();

        // Deactivate and cleanup on shutdown
        RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), "Shutting down PX4Controller...");

        auto deactivate_result = controller->deactivate();
        if (deactivate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_WARN(rclcpp::get_logger("px4_controller_main"), "Failed to deactivate PX4Controller properly");
        }

        auto cleanup_result = controller->cleanup();
        if (cleanup_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            RCLCPP_WARN(rclcpp::get_logger("px4_controller_main"), "Failed to cleanup PX4Controller properly");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("px4_controller_main"), 
                     "Exception in main: %s", e.what());
        flyscan_drone_controller::util::TerminalController::cleanup();
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("px4_controller_main"), "PX4Controller shutdown complete");
    return 0;
}