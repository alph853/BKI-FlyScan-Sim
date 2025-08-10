/**
 * @file base_node.hpp
 * @brief Base class for autonomous drone nodes with lifecycle management and bond-based monitoring.
 *
 * This header file contains the base class for all autonomous drone nodes. It provides
 * lifecycle management, bond-based monitoring, state management, and automatic registration
 * with the LifeMonitor.
 *
 * @author UAV team@/flyscan
 * @date 2025/06/18
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "flyscan_interfaces/msg/node_heartbeat.hpp"
#include "flyscan_interfaces/srv/register_node.hpp"
#include "flyscan_interfaces/srv/unregister_node.hpp"
#include "flyscan_interfaces/srv/request_recovery.hpp"

#include "flyscan_common/enums.hpp"

namespace flyscan {
namespace core {

// Type aliases
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using LifecycleState = rclcpp_lifecycle::State;
using LifecycleStateMsg = lifecycle_msgs::msg::State;
using OperationStatus = flyscan::common::OperationStatus;
using NodeType = flyscan::common::NodeType;
using NodeHeartbeatMsg = flyscan_interfaces::msg::NodeHeartbeat;
using RegisterNodeSrv = flyscan_interfaces::srv::RegisterNode;
using UnregisterNodeSrv = flyscan_interfaces::srv::UnregisterNode;
using RequestRecoverySrv = flyscan_interfaces::srv::RequestRecovery;

/**
 * @brief Base class for autonomous UAV nodes with lifecycle and bond-based monitoring
 */
class BaseNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /**
     * @brief Constructor for BaseNode
     * @param options ROS2 node options
     * @param node_name Name of the node
     * @param node_type Type of the node
     * @param capabilities List of node capabilities
     */
    BaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
             const std::string& node_name = "base_node",
             const NodeType& node_type = NodeType::kBaseNode,
             const std::vector<std::string>& capabilities = {"base_node"}
             );

    /**
     * @brief Destructor
     */
    virtual ~BaseNode();

protected:
    /**
     * @brief Configure node (transition to Inactive)
     * @param state previous lifecycle state
     * @return CallbackReturn indicating success or failure
     */
    LifecycleCallbackReturn on_configure(const LifecycleState& state) override;

    /**
     * @brief Activate node (transition to Active)
     * @param state previous lifecycle state
     * @return CallbackReturn indicating success or failure
     */
    LifecycleCallbackReturn on_activate(const LifecycleState& state) override;

    /**
     * @brief Deactivate node (transition to Inactive)
     * @param state previous lifecycle state
     * @return CallbackReturn indicating success or failure
     */
    LifecycleCallbackReturn on_deactivate(const LifecycleState& state) override;

    /**
     * @brief Cleanup node (transition to Unconfigured)
     * @param state previous lifecycle state
     * @return CallbackReturn indicating success or failure
     */
    LifecycleCallbackReturn on_cleanup(const LifecycleState& state) override;

    /**
     * @brief Shutdown node
     * @param state previous lifecycle state
     * @return CallbackReturn indicating success or failure
     */
    LifecycleCallbackReturn on_shutdown(const LifecycleState& state) override;

    /**
     * @brief Node error
     * @param state previous lifecycle state
     * @return CallbackReturn indicating success or failure
     */
    LifecycleCallbackReturn on_error(const LifecycleState& state) override;

    /**
     * @brief Called when node is configured
     * @return OperationStatus indicating success or failure
     */
    virtual OperationStatus HandleConfigure();

    /**
     * @brief Called when node is activated
     * @return OperationStatus indicating success or failure
     */
    virtual OperationStatus HandleActivate();

    /**
     * @brief Called when node is deactivated
     * @return OperationStatus indicating success or failure
     */
    virtual OperationStatus HandleDeactivate();

    /**
     * @brief Called when node is cleaned up
     * @return OperationStatus indicating success or failure
     */
    virtual OperationStatus HandleCleanup();

    /**
     * @brief Called when node is shutdown
     * @return OperationStatus indicating success or failure
     */
    virtual OperationStatus HandleShutdown();

    /**
     * @brief Called when node has an error
     * @return OperationStatus indicating success or failure
     */
    virtual OperationStatus HandleError();

    NodeType                 node_type_;
    std::vector<std::string> capabilities_;
    std::string              node_id_;
    std::string              heartbeat_topic_;

    rclcpp::Client<RegisterNodeSrv>::SharedPtr      register_client_;
    rclcpp::Client<UnregisterNodeSrv>::SharedPtr    unregister_client_;

    rclcpp::Publisher<NodeHeartbeatMsg>::SharedPtr  heartbeat_pub_;
    rclcpp::TimerBase::SharedPtr                    heartbeat_timer_;

    rclcpp::Service<RequestRecoverySrv>::SharedPtr  request_recovery_service_;

    

private:
    /**
     * @brief Register node with LifeMonitor and create bond
     * @return OperationStatus indicating success or failure
     */
    OperationStatus RegisterWithLifeMonitor();

    /**
     * @brief Unregister node from LifeMonitor and break bond
     * @return OperationStatus indicating success or failure
     */
    OperationStatus UnregisterFromLifeMonitor();

    /**
     * @brief Setup heartbeat publishing after successful registration
     */
    void SetupHeartbeatPublishing();

    /**
     * @brief Handle recovery request from LifeMonitor
     * @param request Recovery request
     * @param response Recovery response
     */
    void HandleRecoveryRequest(
        std::shared_ptr<RequestRecoverySrv::Request> request,
        std::shared_ptr<RequestRecoverySrv::Response> response);

};

}  // namespace core
}  // namespace flyscan