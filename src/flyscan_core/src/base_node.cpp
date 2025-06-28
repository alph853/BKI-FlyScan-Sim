/**
* @file base_node.cpp
* @brief Base node implementation
*
* This file contains the implementation of the BaseNode class.
* It provides the base functionality for all autonomous drone nodes.
*
* @author UAV team@/flyscan
* @date 2025/06/18
*/


#include "flyscan_core/base_node.hpp"
#include "flyscan_common/util.hpp"
#include "flyscan_common/constants.hpp"

namespace flyscan {
namespace core {

BaseNode::BaseNode(const std::string& node_name, 
                   const NodeType& node_type,
                   const std::vector<std::string>& capabilities)
    : LifecycleNode(node_name)
    , m_node_type(node_type)
    , m_capabilities(capabilities)

    , m_node_id("")
    , m_heartbeat_topic("")
    , m_register_client(nullptr)
    , m_unregister_client(nullptr)

    , m_heartbeat_pub(nullptr)
    , m_heartbeat_timer(nullptr)
    , m_request_recovery_service(nullptr)
{
    std::stringstream ss;
    ss << "BaseNode initializing with name: " << node_name 
       << ", type: " << NodeTypeToString(m_node_type) 
       << ", capabilities: [";
    for (const auto& cap : m_capabilities)
        ss << cap << ", ";
    ss << "]";

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

BaseNode::~BaseNode()
{
    RCLCPP_INFO(this->get_logger(), "Node shutting down");
    
    if (UnregisterFromLifeMonitor() != OperationStatus::kOK)
        RCLCPP_WARN(this->get_logger(), "Failed to unregister from LifeMonitor");
    else
        RCLCPP_INFO(this->get_logger(), "Successfully unregistered from LifeMonitor");
}

LifecycleCallbackReturn BaseNode::on_configure(const LifecycleState& state)
{
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s.\nConfiguring...", this->get_name(), state.label().c_str());
    
    /* 
    * Service Consumers
    */
    using namespace std::placeholders;
    namespace srv = flyscan::common::constants::srv;

    m_register_client   = this->create_client<RegisterNodeSrv>(srv::REGISTER_NODE);
    m_unregister_client = this->create_client<UnregisterNodeSrv>(srv::UNREGISTER_NODE);

    /*
    * Service Providers
    */
    m_request_recovery_service = this->create_service<RequestRecoverySrv>(
        srv::REQUEST_RECOVERY,
        std::bind(&BaseNode::HandleRecoveryRequest, this, _1, _2));
    
    /*
    * Register with LifeMonitor
    */
    if (this->RegisterWithLifeMonitor() != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to register with LifeMonitor");
        return LifecycleCallbackReturn::FAILURE;
    }

    /*
    * Publishers
    */
    namespace qos   = flyscan::common::constants::qos;
    namespace timer = flyscan::common::constants::timer;

    m_heartbeat_pub   = this->create_publisher<NodeHeartbeatMsg>(m_heartbeat_topic, qos::PUBLISH_QOS);
    m_heartbeat_timer = this->create_wall_timer(timer::HEARTBEAT_PUB_PERIOD, 
        [this]() {
            if (this->get_current_state().id() != LifecycleStateMsg::PRIMARY_STATE_ACTIVE) {
                RCLCPP_WARN(this->get_logger(), "Node %s is not active, skipping heartbeat", this->get_name());
                return;
            }

            auto msg = std::make_shared<NodeHeartbeatMsg>();
            msg->node_id    = m_node_id;
            msg->stamp      = this->now();
            m_heartbeat_pub->publish(*msg);
        });

    /*
    ******** BaseNode Initialization Done ********
    */

    if (HandleConfigure() == OperationStatus::kOK) {
        RCLCPP_INFO(this->get_logger(), "HandleConfigure successful");
        return LifecycleCallbackReturn::SUCCESS;
    }
    
    RCLCPP_ERROR(this->get_logger(), "Configuration failed");
    return LifecycleCallbackReturn::FAILURE;
}

LifecycleCallbackReturn BaseNode::on_activate(const LifecycleState& state)
{
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s.\nActivating...", this->get_name(), state.label().c_str());
    
    OperationStatus status = HandleActivate();
    if (status == OperationStatus::kOK) {
        RCLCPP_INFO(this->get_logger(), "Activation successful");
        return LifecycleCallbackReturn::SUCCESS;
    } 
    
    if (status == OperationStatus::kServiceNA) {
        // TODO: Implement service not available handling
    }
    RCLCPP_ERROR(this->get_logger(), "Activation failed: %s", OperationStatusToString(status).c_str());
    return LifecycleCallbackReturn::FAILURE;
}

LifecycleCallbackReturn BaseNode::on_deactivate(const LifecycleState& state)
{
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s", this->get_name(), state.label().c_str());
    RCLCPP_INFO(this->get_logger(), "Deactivating...");
    
    OperationStatus status = HandleDeactivate();
    if (status == OperationStatus::kOK) {
        RCLCPP_INFO(this->get_logger(), "Deactivation successful");
        return LifecycleCallbackReturn::SUCCESS;
    } 
    
    if (status == OperationStatus::kServiceNA) {
        // TODO: Implement service not available handling
    }
    
    RCLCPP_ERROR(this->get_logger(), "Deactivation failed: %s", OperationStatusToString(status).c_str());
    return LifecycleCallbackReturn::FAILURE;
}

LifecycleCallbackReturn BaseNode::on_cleanup(const LifecycleState& state)
{   
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s", this->get_name(), state.label().c_str());
    RCLCPP_INFO(this->get_logger(), "Cleaning up...");
    
    m_register_client.reset();
    m_unregister_client.reset();
    m_request_recovery_service.reset();
    
    if (HandleCleanup() == OperationStatus::kOK) {
        RCLCPP_INFO(this->get_logger(), "Cleanup successful");
        return LifecycleCallbackReturn::SUCCESS;
    }
    
    RCLCPP_ERROR(this->get_logger(), "Cleanup failed");
    return LifecycleCallbackReturn::FAILURE;
}


LifecycleCallbackReturn BaseNode::on_shutdown(const LifecycleState& state)
{
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s", this->get_name(), state.label().c_str());
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    
    if (HandleShutdown() == OperationStatus::kOK) {
        RCLCPP_INFO(this->get_logger(), "Shutdown successful");
        return LifecycleCallbackReturn::SUCCESS;
    }
    
    RCLCPP_ERROR(this->get_logger(), "Shutdown failed");
    return LifecycleCallbackReturn::FAILURE;
}

LifecycleCallbackReturn BaseNode::on_error(const LifecycleState& state)
{
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s", this->get_name(), state.label().c_str());
    RCLCPP_INFO(this->get_logger(), "Handling error...");
    
    if (HandleError() == OperationStatus::kOK) {
        RCLCPP_INFO(this->get_logger(), "Handling error successful");
        return LifecycleCallbackReturn::SUCCESS;
    }
    
    RCLCPP_ERROR(this->get_logger(), "Handling error failed");
    return LifecycleCallbackReturn::FAILURE;
}

// Virtual methods - default implementations
OperationStatus BaseNode::HandleConfigure()
{
    RCLCPP_DEBUG(this->get_logger(), "Default configuration");
    return OperationStatus::kOK;
}

OperationStatus BaseNode::HandleActivate()
{
    RCLCPP_DEBUG(this->get_logger(), "Default activation");
    return OperationStatus::kOK;
}

OperationStatus BaseNode::HandleDeactivate()
{
    RCLCPP_DEBUG(this->get_logger(), "Default deactivation");
    return OperationStatus::kOK;
}

OperationStatus BaseNode::HandleCleanup()
{
    RCLCPP_DEBUG(this->get_logger(), "Default cleanup");
    return OperationStatus::kOK;
}

OperationStatus BaseNode::HandleShutdown()
{
    RCLCPP_DEBUG(this->get_logger(), "Default shutdown");
    return OperationStatus::kOK;
}

OperationStatus BaseNode::HandleError()
{
    RCLCPP_DEBUG(this->get_logger(), "Default error handling");
    return OperationStatus::kNotImplemented;
}

OperationStatus BaseNode::RegisterWithLifeMonitor()
{
    RCLCPP_INFO(this->get_logger(), "Registering with LifeMonitor");

    auto request = std::make_shared<RegisterNodeSrv::Request>();
    request->node_name         = this->get_name();
    request->node_type         = static_cast<uint8_t>(m_node_type);
    request->node_namespace    = this->get_namespace();
    request->capabilities      = m_capabilities;
    
    try {
        auto response = flyscan::common::call_service_sync<RegisterNodeSrv>(
            m_register_client, request);

        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Registration failed");
            return OperationStatus::kMalformedInput;
        }

        m_node_id         = response->node_id;
        m_heartbeat_topic = response->heartbeat_topic;
        RCLCPP_INFO(this->get_logger(), "Node %s registered: ID = %s, Topic = %s",
                    this->get_name(), m_node_id.c_str(), m_heartbeat_topic.c_str());

        return OperationStatus::kOK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        return OperationStatus::kServiceNA;
    }
}

OperationStatus BaseNode::UnregisterFromLifeMonitor()
{
    RCLCPP_INFO(this->get_logger(), "Unregistering from LifeMonitor");
    
    if (m_node_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Not registered");
        return OperationStatus::kNotInitialized;
    }
    
    auto request        = std::make_shared<UnregisterNodeSrv::Request>();
    request->node_id    = m_node_id;
    request->node_name  = this->get_name();
    request->node_namespace = this->get_namespace();
    
    try {
        auto response = flyscan::common::call_service_sync<UnregisterNodeSrv>
            (m_unregister_client, request);
    
        if (!response->success)
        {
            RCLCPP_ERROR(this->get_logger(), "Unregistration failed");
            return OperationStatus::kMalformedInput;
        }
        m_node_id.clear();
        m_heartbeat_topic.clear();
        RCLCPP_INFO(this->get_logger(), "Node %s unregistered: ID = %s",
                    this->get_name(), m_node_id.c_str());

        return OperationStatus::kOK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        return OperationStatus::kServiceNA;
    }
}

void BaseNode::HandleRecoveryRequest(
    std::shared_ptr<RequestRecoverySrv::Request> request,
    std::shared_ptr<RequestRecoverySrv::Response> response)
{
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Handling recovery request for node %s", this->get_name());

    // TODO: Implement recovery request handling
    response->success = false;
    response->message = "Recovery request not implemented";
}

}  // namespace core
}  // namespace flyscan