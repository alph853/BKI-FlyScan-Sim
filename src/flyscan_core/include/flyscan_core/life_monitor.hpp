/**
 * @file life_monitor.hpp
 * @brief LifeMonitorServer node for monitoring autonomous UAV nodes
 *
 * This header file contains the LifeMonitorServer node that manages registration,
 * bond-based monitoring, and recovery of autonomous UAV nodes.
 *
 * @author UAV team@/flyscan
 * @date 2025/06/09
 * @version 1.0
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <shared_mutex>
#include <rclcpp_components/register_node_macro.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "flyscan_interfaces/msg/node_heartbeat.hpp"
#include "flyscan_interfaces/srv/register_node.hpp"
#include "flyscan_interfaces/srv/unregister_node.hpp"
#include "flyscan_interfaces/srv/get_registered_nodes.hpp"
#include "flyscan_interfaces/msg/node_info.hpp"

#include "flyscan_common/enums.hpp"
#include "flyscan_core/constants.hpp"

namespace flyscan {
namespace core  {

// Type aliases
using LifecycleState = rclcpp_lifecycle::State;
using LifecycleStateMsg = lifecycle_msgs::msg::State;
using NodeType = flyscan::common::NodeType;
using NodeHeartbeatMsg = flyscan_interfaces::msg::NodeHeartbeat;
using RegisterNodeSrv = flyscan_interfaces::srv::RegisterNode;
using UnregisterNodeSrv = flyscan_interfaces::srv::UnregisterNode;
using GetRegisteredNodesSrv = flyscan_interfaces::srv::GetRegisteredNodes;
using NodeInfoMsg = flyscan_interfaces::msg::NodeInfo;

/**
 * @brief Structure to hold registered node information with bond
 */
struct RegisteredNodeInfo
{
    std::string node_name;
    std::string node_namespace;
    std::string version;
    NodeType    node_type;
    std::vector<std::string> capabilities;
    rclcpp::Subscription<LifecycleStateMsg>::SharedPtr state_subscription;
    rclcpp::Subscription<NodeHeartbeatMsg>::SharedPtr heartbeat_subscription;
    LifecycleState current_state;
    rclcpp::Time registration_time;
    rclcpp::Time last_heartbeat;
};

/**
 * @brief LifeMonitor node for managing autonomous UAV nodes with bond-based monitoring
 */
class LifeMonitor : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    LifeMonitor(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructor
     */
    virtual ~LifeMonitor();

private:
    /**
     * @brief Callback for registration request from a node
     * @param request Registration request
     * @param response Registration response
     */
    void HandleRegisterNode(
        const std::shared_ptr<RegisterNodeSrv::Request> request,
        std::shared_ptr<RegisterNodeSrv::Response> response);

    /**
     * @brief Unregister a node and break bond
     * @param request Unregistration request
     * @param response Unregistration response
     */
    void HandleUnregisterNode(
        const std::shared_ptr<UnregisterNodeSrv::Request> request,
        std::shared_ptr<UnregisterNodeSrv::Response> response);

    /**
     * @brief Get registered nodes list
     * @param request Get nodes request
     * @param response Get nodes response
     */
    void HandleGetRegisteredNodes(
        const std::shared_ptr<GetRegisteredNodesSrv::Request> request,
        std::shared_ptr<GetRegisteredNodesSrv::Response> response);

    /**
     * @brief Callback that handles heartbeat messages from individual nodes
     * @param msg Heartbeat message
     * @param node_id The ID of the node
     */
    void HandleHeartbeatTimeout(const std::string& node_id);

    /**
     * @brief Generate unique registration ID
     * @param node_name Node name
     * @param node_namespace Node namespace label
     * @return Pair of registration ID and heartbeat topic
     */
    std::pair<std::string, std::string> GenerateRegistrationInfo(
        const std::string& node_name, 
        const std::string& node_namespace);

    // Thread-safe access to registered nodes
    mutable std::shared_mutex m_nodes_mutex;
    std::unordered_map<std::string, RegisteredNodeInfo> m_registered_nodes;
    
    // ROS 2 interfaces
    rclcpp::Service<RegisterNodeSrv>::SharedPtr         m_register_service;
    rclcpp::Service<UnregisterNodeSrv>::SharedPtr       m_unregister_service;
    rclcpp::Service<GetRegisteredNodesSrv>::SharedPtr   m_get_nodes_service;

    rclcpp::Subscription<NodeHeartbeatMsg>::SharedPtr   m_heartbeat_sub;
    rclcpp::TimerBase::SharedPtr                        m_node_monitor_timer;

};

}  // namespace core
}  // namespace flyscan

RCLCPP_COMPONENTS_REGISTER_NODE(flyscan::core::LifeMonitor)