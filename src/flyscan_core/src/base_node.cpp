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
#include "flyscan_core/constants.hpp"
#include <rclcpp/qos.hpp>

namespace flyscan {
namespace core {

BaseNode::BaseNode(const rclcpp::NodeOptions & options,
                   const std::string& node_name,
                   const NodeType& node_type,
                   const std::vector<std::string>& capabilities)
    : LifecycleNode(node_name, options)
    , node_type_(node_type)
    , capabilities_(capabilities)

    , node_id_("")
    , heartbeat_topic_("")
    , register_client_(nullptr)
    , unregister_client_(nullptr)

    , heartbeat_pub_(nullptr)
    , heartbeat_timer_(nullptr)
    , request_recovery_service_(nullptr)
{
    std::stringstream ss;
    ss << "[";
    for (const auto& cap : capabilities_)
        ss << cap << ", ";
    ss << "]"; 

    RCLCPP_INFO(this->get_logger(), "BaseNode initializing with name: %s, type: %s, capabilities: %s",
                node_name.c_str(), NodeTypeToString(node_type_).c_str(), ss.str().c_str());
}

BaseNode::~BaseNode()
{
    RCLCPP_INFO(this->get_logger(), "BaseNode destructor called");
    
    // Stop timers to prevent further callbacks during shutdown
    if (heartbeat_timer_) {
        heartbeat_timer_->cancel();
        heartbeat_timer_.reset();
    }
    
    // Reset publishers and clients
    heartbeat_pub_.reset();
    register_client_.reset();
    unregister_client_.reset();
    request_recovery_service_.reset();
    
    RCLCPP_INFO(this->get_logger(), "BaseNode shutdown complete");
}

LifecycleCallbackReturn BaseNode::on_configure(const LifecycleState& state)
{
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s.\nConfiguring...", this->get_name(), state.label().c_str());
    
    /* 
    * Service Consumers
    */
    using namespace std::placeholders;
    register_client_   = this->create_client<RegisterNodeSrv>(
        srv::REGISTER_NODE, rclcpp::ServicesQoS());
    unregister_client_ = this->create_client<UnregisterNodeSrv>(
        srv::UNREGISTER_NODE, rclcpp::ServicesQoS());

    /*
    * Service Providers
    */
    request_recovery_service_ = this->create_service<RequestRecoverySrv>(
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
    
    RCLCPP_ERROR(this->get_logger(), "Deactivation failed: %s", OperationStatusToString(status).c_str());
    return LifecycleCallbackReturn::FAILURE;
}

LifecycleCallbackReturn BaseNode::on_cleanup(const LifecycleState& state)
{   
    RCLCPP_INFO(this->get_logger(), "Node %s in state %s", this->get_name(), state.label().c_str());
    RCLCPP_INFO(this->get_logger(), "Cleaning up...");
    
    register_client_.reset();
    unregister_client_.reset();
    request_recovery_service_.reset();
    
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
    
    auto operation_status = this->UnregisterFromLifeMonitor();
    if (operation_status != OperationStatus::kOK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to unregister from LifeMonitor: %s", 
                     OperationStatusToString(operation_status).c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Unregister from LifeMonitor successfully");
    }
    
    if (heartbeat_timer_) {
        heartbeat_timer_->cancel();
    }
    
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

    using namespace std::placeholders;

    auto request = std::make_shared<RegisterNodeSrv::Request>();
    request->node_name         = this->get_name();
    request->node_type         = static_cast<uint8_t>(node_type_);
    request->node_namespace    = this->get_namespace();
    request->capabilities      = capabilities_;
    
    if (!register_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Registration service not available");
        return OperationStatus::kServiceNA;
    }

    // Send async request with callback
    auto response_callback = [this](rclcpp::Client<RegisterNodeSrv>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                node_id_ = response->node_id;
                heartbeat_topic_ = response->heartbeat_topic;
                RCLCPP_INFO(this->get_logger(), "Node %s registered successfully: ID = %s, Topic = %s",
                            this->get_name(), node_id_.c_str(), heartbeat_topic_.c_str());
                SetupHeartbeatPublishing();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Registration failed: %s", response->message.c_str());
                // Trigger error state transition
                this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Registration service call failed: %s", e.what());
            // Trigger error state transition
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR);
        }
    };

    register_client_->async_send_request(request, response_callback);
    
    RCLCPP_INFO(this->get_logger(), "Registration request sent, continuing with configuration");
    return OperationStatus::kOK;
}

void BaseNode::SetupHeartbeatPublishing()
{
    if (heartbeat_topic_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot setup heartbeat - topic is empty");
        return;
    }

    heartbeat_pub_ = this->create_publisher<NodeHeartbeatMsg>(heartbeat_topic_, rclcpp::SensorDataQoS());
    heartbeat_timer_ = this->create_wall_timer(timer::HEARTBEAT_PUB_PERIOD, 
        [this]() {
            if (this->get_current_state().id() != LifecycleStateMsg::PRIMARY_STATE_ACTIVE) {
                RCLCPP_DEBUG(this->get_logger(), "Node %s is not active, skipping heartbeat", this->get_name());
                return;
            }

            if (node_id_.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "Node ID not set, skipping heartbeat");
                return;
            }

            auto msg = std::make_shared<NodeHeartbeatMsg>();
            msg->node_id = node_id_;
            msg->stamp   = this->now();
            heartbeat_pub_->publish(*msg);
        });
    
    RCLCPP_INFO(this->get_logger(), "Heartbeat publishing setup complete for topic: %s", heartbeat_topic_.c_str());
}

OperationStatus BaseNode::UnregisterFromLifeMonitor()
{
    RCLCPP_INFO(this->get_logger(), "Unregistering from LifeMonitor");
    
    if (!unregister_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Registration service not available");
        return OperationStatus::kServiceNA;
    }
    
    auto request = std::make_shared<UnregisterNodeSrv::Request>();
    request->node_id = node_id_;
    request->node_name = this->get_name();
    request->node_namespace = this->get_namespace();

    auto response_callback = [this](rclcpp::Client<UnregisterNodeSrv>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Node %s unregistered successfully", this->get_name());
            } else {
                RCLCPP_WARN(this->get_logger(), "Unregistration failed: %s", response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Unregister service call failed: %s", e.what());
        }
    };

    try {
        unregister_client_->async_send_request(request, response_callback);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to send unregister request: %s", e.what());
    }

    // Clear local state immediately since we're shutting down
    node_id_.clear();
    heartbeat_topic_.clear();

    return OperationStatus::kOK;
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