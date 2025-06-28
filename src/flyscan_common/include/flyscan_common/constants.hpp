/**
 * @file constants.hpp
 * @brief Global constants for topics, services, QoS, and timers used by Flyscan UAV nodes.
 * 
 * This file centralizes static strings and configurations to ensure
 * consistency across nodes and packages.
 * 
 * @author UAV Team
 * @date 2025-06-17
 */


#pragma once

#include "rclcpp/rclcpp.hpp"

namespace flyscan {

namespace common {

namespace constants {

namespace topic {
    // Devices topics
    constexpr const char* CAMERA_IMAGE  = "/camera/image_raw";
    constexpr const char* DEPTH_IMAGE   = "/camera/depth_image";
    constexpr const char* IMU           = "/imu/data";

    constexpr const char* ODOM          = "/odom";
    constexpr const char* HEARTBEAT     = "/heartbeat";

    // Map topics
    constexpr const char* OCCUPANCY_MAP     = "/occupancy_map";
    constexpr const char* EXPLORATION_MAP   = "/exploration_map";
    constexpr const char* SEMANTIC_MAP      = "/semantic_map";
    constexpr const char* HYPERMAP_MAP      = "/hypermap_map";  
    
    // Application topics
    constexpr const char* APPLICATION_GOAL = "/application/goal";
} // namespace topic

namespace srv {
    constexpr const char* HYPERMAP_QUERY    = "/hypermap/query";
    constexpr const char* NAVIGATE_TO_POSE  = "/navigate_to_pose";
    
    constexpr const char* REGISTER_NODE     = "/life_monitor/register_node";
    constexpr const char* UNREGISTER_NODE   = "/life_monitor/unregister_node";
    constexpr const char* GET_REGISTERED    = "/life_monitor/get_registered";
    constexpr const char* REQUEST_RECOVERY  = "/life_monitor/request_recovery";
} // namespace srv

namespace qos {
    inline const rclcpp::QoS SENSOR_QOS = rclcpp::SensorDataQoS();
    inline const rclcpp::QoS SUBSCRIPTION_QOS{10};
    inline const rclcpp::QoS PUBLISH_QOS{10};
}

namespace timer {
    constexpr const uint16_t REQUEST_TIMEOUT_MS  = 5000;
    constexpr const uint16_t SERVICE_TIMEOUT_MS  = 10000;

    constexpr std::chrono::milliseconds HEARTBEAT_PUB_PERIOD{1000};
    constexpr std::chrono::milliseconds HEARTBEAT_MONITOR_PERIOD{1000};
    const rclcpp::Duration HEARTBEAT_TIMEOUT_DURATION 
        = rclcpp::Duration::from_seconds(3.0);
}

} // namespace constants
}  // namespace common
}  // namespace flyscan