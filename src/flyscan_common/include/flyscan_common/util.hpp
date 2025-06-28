/**
 * @file util.hpp
 * @brief Utility functions for common operations
 *
 * This header file contains utility functions for common operations.
 *
 * @author UAV team@/flyscan
 * @date 2025/06/18
 */

#pragma once

#include <future>
#include <optional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "flyscan_common/constants.hpp"

namespace flyscan {
namespace common {

template<typename ServiceT>
typename ServiceT::Response::SharedPtr
call_service_sync(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request,
    uint16_t service_timeout = constants::timer::SERVICE_TIMEOUT_MS,
    uint16_t request_timeout = constants::timer::REQUEST_TIMEOUT_MS
)
{
    using namespace std::chrono_literals;
    auto service_timeout_ms = std::chrono::milliseconds(service_timeout);
    auto request_timeout_ms = std::chrono::milliseconds(request_timeout);
    
    std::string service_name = std::string(client->get_service_name());
    if (!client || !client->wait_for_service(service_timeout_ms)) {
        throw std::runtime_error("Service [" + service_name + "] not available.");
    }

    auto prom   = std::make_shared<std::promise<typename ServiceT::Response::SharedPtr>>();
    auto future = prom->get_future();

    client->async_send_request(request,
        [prom](typename rclcpp::Client<ServiceT>::SharedFuture result) {
            prom->set_value(result.get());
        });

    if (future.wait_for(request_timeout_ms) != std::future_status::ready) {
        throw std::runtime_error("Service call to [" + service_name + "] timed out after " +
                                 std::to_string(request_timeout) + " ms.");
    }
    return future.get();
}


} // namespace common
} // namespace flyscan