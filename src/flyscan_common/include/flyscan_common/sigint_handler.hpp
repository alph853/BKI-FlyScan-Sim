/**
 * @file sigint_handler.hpp
 * @brief Generic SIGINT handler for lifecycle nodes
 *
 * Provides a template-based SIGINT handler for graceful shutdown of ROS2 lifecycle nodes.
 * This utility generalizes the signal handling pattern used across multiple nodes.
 *
 * @author UAV team@/flyscan
 * @date 2025/08/03
 */

#pragma once

#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace flyscan {
namespace common {

// Global variables to store handler data (needed for C-style signal handler)
namespace detail {
    inline std::shared_ptr<rclcpp_lifecycle::LifecycleNode> g_node;
    inline std::string g_logger_name;
    
    inline void SigintCallback(int signum) {
        (void)signum;  // Suppress unused parameter warning
        
        if (g_node) {
            RCLCPP_INFO(g_node->get_logger(), "SIGINT received, triggering shutdown transition");

            auto shutdown_result = g_node->shutdown();
            if (shutdown_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
                RCLCPP_WARN(rclcpp::get_logger(g_logger_name), 
                            "Failed to shutdown %s properly", g_logger_name.c_str());
            } else {
                RCLCPP_INFO(rclcpp::get_logger(g_logger_name), 
                            "%s shutdown completed successfully", g_logger_name.c_str());
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger(g_logger_name), 
                        "%s instance not available for shutdown", g_logger_name.c_str());
        }
        rclcpp::shutdown();
        g_node.reset();
    }
} // namespace detaiil

/**
 * @brief Sets up a SIGINT handler for a lifecycle node
 * 
 * This function registers a signal handler that will properly shutdown the given node
 * when SIGINT is received.
 * 
 * @tparam NodeType The type of the lifecycle node
 * @param node Shared pointer to the node instance
 * @param logger_name Name to use for logging during shutdown
 */
template<typename NodeType>
void SetupSigintHandler(std::shared_ptr<NodeType> node, const std::string& logger_name) {
    // Store the node and logger name in global variables
    detail::g_node = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node);
    detail::g_logger_name = logger_name;

    // Register the signal handler
    signal(SIGINT, detail::SigintCallback);
}

} // namespace common
} // namespace flyscan