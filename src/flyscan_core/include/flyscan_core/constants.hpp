/**
 * @file constants.hpp
 * @brief Constants specific to flyscan_core package
 * 
 * This file contains constants used by core nodes like BaseNode and LifeMonitor.
 * 
 * @author UAV Team
 * @date 2025-07-30
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace flyscan {
namespace core {
namespace constants {

namespace srv {
    constexpr const char* REGISTER_NODE     = "/life_monitor/register_node";
    constexpr const char* UNREGISTER_NODE   = "/life_monitor/unregister_node";
    constexpr const char* GET_REGISTERED    = "/life_monitor/get_registered";
    constexpr const char* REQUEST_RECOVERY  = "/life_monitor/request_recovery";
} // namespace srv

namespace qos {
    inline const rclcpp::QoS SUBSCRIPTION_QOS{10};
    inline const rclcpp::QoS PUBLISH_QOS{10};
}

namespace timer {
    constexpr const uint16_t REQUEST_TIMEOUT_MS  = 10000;
    constexpr const uint16_t SERVICE_TIMEOUT_MS  = 10000;

    constexpr std::chrono::milliseconds HEARTBEAT_PUB_PERIOD{1000};
    constexpr std::chrono::milliseconds HEARTBEAT_MONITOR_PERIOD{1000};
    const rclcpp::Duration HEARTBEAT_TIMEOUT_DURATION 
        = rclcpp::Duration::from_seconds(3.0);
}

} // namespace constants
} // namespace core
} // namespace flyscan