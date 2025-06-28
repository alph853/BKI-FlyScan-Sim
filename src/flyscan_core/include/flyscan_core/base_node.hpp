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

#include "flyscan_common/enums.hpp"
#include "flyscan_common/types.hpp"

namespace flyscan {
namespace core {

using LifecycleNode = flyscan::common::core_types::LifecycleNode;

using LifecycleCallbackReturn   = flyscan::common::core_types::LifecycleCallbackReturn;
using LifecycleState            = flyscan::common::core_types::LifecycleState;
using LifecycleStateMsg         = flyscan::common::core_types::LifecycleStateMsg;
using LifecycleTransition       = flyscan::common::core_types::LifecycleTransition;

using NodeHeartbeatMsg          = flyscan::common::core_types::NodeHeartbeatMsg;

using RegisterNodeSrv           = flyscan::common::core_types::RegisterNodeSrv;
using UnregisterNodeSrv         = flyscan::common::core_types::UnregisterNodeSrv;
using RequestRecoverySrv        = flyscan::common::core_types::RequestRecoverySrv;

using OperationStatus = flyscan::common::OperationStatus;
using NodeType        = flyscan::common::NodeType;

/**
 * @brief Base class for autonomous UAV nodes with lifecycle and bond-based monitoring
 * @note This node requires execution in a MultiThreadedExecutor for blocking service calls
 * like RegisterWithLifeMonitor(). Single-threaded execution may deadlock or hang.
 */
class BaseNode : public LifecycleNode
{
public:
    /**
     * @brief Constructor for BaseNode
     * @param node_name Name of the node
     * @param node_type Type of the node
     * @param capabilities List of node capabilities
     */
    BaseNode(const std::string& node_name, 
             const NodeType& node_type,
             const std::vector<std::string>& capabilities);

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

    NodeType                 m_node_type;
    std::vector<std::string> m_capabilities;
    std::string              m_node_id;
    std::string              m_heartbeat_topic;

    rclcpp::Client<RegisterNodeSrv>::SharedPtr      m_register_client;
    rclcpp::Client<UnregisterNodeSrv>::SharedPtr    m_unregister_client;

    rclcpp::Publisher<NodeHeartbeatMsg>::SharedPtr  m_heartbeat_pub;
    rclcpp::TimerBase::SharedPtr                    m_heartbeat_timer;

    rclcpp::Service<RequestRecoverySrv>::SharedPtr  m_request_recovery_service;

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