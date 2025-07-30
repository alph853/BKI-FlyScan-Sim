/**
 * @file request_util.hpp
 * @brief Utility functions for ROS2 service request operations
 *
 * This header file contains utility functions for synchronous service requests.
 *
 * @author UAV team@/flyscan
 * @date 2025/06/18
 */

#pragma once

#include <future>
#include <optional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
// Constants removed - using direct values for timeouts

namespace flyscan {
namespace common {

// template<typename ServiceT>
// typename ServiceT::Response::SharedPtr
// sync_send_request(
//     const typename rclcpp::Client<ServiceT>::SharedPtr & client,
//     const typename ServiceT::Request::SharedPtr & request,
//     uint16_t service_timeout = constants::timer::SERVICE_TIMEOUT_MS,
//     uint16_t request_timeout = constants::timer::REQUEST_TIMEOUT_MS
// )
// {
//     using namespace std::chrono_literals;
//     auto service_timeout_ms = std::chrono::milliseconds(service_timeout);
//     auto request_timeout_ms = std::chrono::milliseconds(request_timeout);
    
//     std::string service_name = std::string(client->get_service_name());
//     if (!client || !client->wait_for_service(service_timeout_ms)) {
//         throw std::runtime_error("Service [" + service_name + "] not available.");
//     }

//     auto prom   = std::make_shared<std::promise<typename ServiceT::Response::SharedPtr>>();
//     auto future = prom->get_future();

//     client->async_send_request(request,
//         [prom](typename rclcpp::Client<ServiceT>::SharedFuture result) {
//             prom->set_value(result.get());
//         });

//     auto status = std::future_status::timeout;
//     while (rclcpp::ok() && status != std::future_status::ready) {
//         status = future.wait_for(50ms);
//     }

//     if (future.wait_for(request_timeout_ms) != std::future_status::ready) {
//         throw std::runtime_error("Service call to [" + service_name + "] timed out after " +
//                                  std::to_string(request_timeout) + " ms.");
//     }
//     return future.get();
// }

template<typename ServiceT>
typename ServiceT::Response::SharedPtr
sync_send_request(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request,
    std::chrono::milliseconds service_timeout = std::chrono::milliseconds(2000),
    std::chrono::milliseconds request_timeout = std::chrono::milliseconds(2000)
)
{
    std::string service_name = client->get_service_name();

    if (!client) {
        throw std::runtime_error("Client handle is null for service [" + service_name + "]");
    }

    if (!node_base) {
        throw std::runtime_error("Node base interface is null for service [" + service_name + "]");
    }

    // Wait for the server to be available with interruptible polling
    auto start_time = std::chrono::steady_clock::now();
    while (!client->service_is_ready()) {
        if (!rclcpp::ok()) {
            throw std::runtime_error("Service [" + service_name + "] wait interrupted (shutdown).");
        }
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed >= service_timeout) {
            throw std::runtime_error("Service [" + service_name + "] not available after timeout.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto future = client->async_send_request(request);

    // This blocks while spinning callbacks, respecting ROS shutdown.
    auto result = rclcpp::spin_until_future_complete(node_base, future, request_timeout);

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        return future.get();
    } else if (result == rclcpp::FutureReturnCode::TIMEOUT) {
        throw std::runtime_error("Service [" + service_name + "] call timed out.");
    } else if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
        throw std::runtime_error("Service [" + service_name + "] call interrupted (node shutdown).");
    } else {
        throw std::runtime_error("Unknown spin_until_future_complete result for service [" + service_name + "].");
    }
}

} // namespace common
} // namespace flyscan