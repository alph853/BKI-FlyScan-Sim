#include <cassert>
#include <chrono>
#include <sstream>

#include "flyscan_core/life_monitor.hpp"
#include "flyscan_core/constants.hpp"

namespace flyscan {
namespace core {

LifeMonitor::LifeMonitor(const rclcpp::NodeOptions & options)
    : rclcpp::Node("life_monitor", options)
{
    RCLCPP_INFO(this->get_logger(), "LifeMonitor node initializing");

    using namespace std::placeholders;
    using namespace flyscan::core::constants;

    /*
     * Services providers
    */
    m_register_service = this->create_service<RegisterNodeSrv>(
        srv::REGISTER_NODE,
        std::bind(&LifeMonitor::HandleRegisterNode, this, _1, _2));
    
    m_unregister_service = this->create_service<UnregisterNodeSrv>(
        srv::UNREGISTER_NODE,
        std::bind(&LifeMonitor::HandleUnregisterNode, this, _1, _2));
    
    m_get_nodes_service = this->create_service<GetRegisteredNodesSrv>(
        srv::GET_REGISTERED,
        std::bind(&LifeMonitor::HandleGetRegisteredNodes, this, _1, _2));

    /*
     * Timers
    */
    namespace timer = flyscan::core::constants::timer;

    m_node_monitor_timer = this->create_wall_timer(
        timer::HEARTBEAT_MONITOR_PERIOD,
        [this]() {
            std::unique_lock<std::shared_mutex> lock(m_nodes_mutex);

            for (auto& pair : m_registered_nodes) {
                auto& node_info = pair.second;

                auto timeout_duration = timer::HEARTBEAT_TIMEOUT_DURATION;
                if (now() - node_info.last_heartbeat <= timeout_duration) {
                    return;

                RCLCPP_DEBUG(this->get_logger(), "Node %s heartbeat timeout", node_info.node_name.c_str());
                this->HandleHeartbeatTimeout(pair.first);
                // TODO: Implement recovery mechanism
                // InitiateNodeRecovery(pair.first);
                }
            }
        });

    RCLCPP_INFO(this->get_logger(), "LifeMonitor node ready");
}

LifeMonitor::~LifeMonitor()
{
    RCLCPP_INFO(this->get_logger(), "LifeMonitor node shutting down");
    m_registered_nodes.clear();
}

void LifeMonitor::HandleRegisterNode(
    const std::shared_ptr<RegisterNodeSrv::Request> request,
    std::shared_ptr<RegisterNodeSrv::Response> response)
{
    auto pair = GenerateRegistrationInfo(request->node_name, request->node_namespace);
    std::string node_id         = pair.first;
    std::string heartbeat_topic = pair.second;
    
    RCLCPP_INFO(this->get_logger(), "Registering node: %s with heartbeat topic: %s", 
                request->node_name.c_str(), heartbeat_topic.c_str());
    
    // Check if node is already registered
    auto it = m_registered_nodes.find(node_id);
    if (it != m_registered_nodes.end()) {
        RCLCPP_WARN(this->get_logger(), "Node %s already registered", request->node_name.c_str());
        response->success = false;
        response->node_id = node_id;
        response->message = "Node already registered";
        return;
    }
    
    namespace qos = flyscan::core::constants::qos;
    std::string state_topic = "/" + request->node_name + "/state";

    auto state_subscription = this->create_subscription<lifecycle_msgs::msg::State>(
        state_topic, qos::SUBSCRIPTION_QOS,
        [this, node_id](const LifecycleStateMsg::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received state update from: %s, state: %u", 
                        node_id.c_str(), msg->id);
            
            std::unique_lock<std::shared_mutex> lock(m_nodes_mutex);
            
            auto it = m_registered_nodes.find(node_id);
            if (it == m_registered_nodes.end()) {
                RCLCPP_WARN(this->get_logger(), "Node %s not found", node_id.c_str());
                return;
            }

            auto& node_info             = it->second;
            node_info.current_state     = LifecycleState(msg->id, msg->label);

            RCLCPP_INFO(this->get_logger(), "Node %s state updated to: %u", 
                        node_id.c_str(), msg->id);
        }
    );
    auto heartbeat_subscription = this->create_subscription<NodeHeartbeatMsg>(
        heartbeat_topic, qos::SUBSCRIPTION_QOS,
        [this, node_id](const NodeHeartbeatMsg::SharedPtr msg) {
            if (node_id != msg->node_id) {
                RCLCPP_WARN(this->get_logger(), "Received heartbeat with mismatched node ID: %s (expected: %s)", 
                            msg->node_id.c_str(), node_id.c_str());
                return;  // Ignore mismatched heartbeats
            }
            RCLCPP_DEBUG(this->get_logger(), "Received heartbeat from: %s", node_id.c_str());
            
            std::unique_lock<std::shared_mutex> lock(m_nodes_mutex);
            
            auto it = m_registered_nodes.find(node_id);
            if (it == m_registered_nodes.end()) {
                RCLCPP_WARN(this->get_logger(), "Node %s not found for heartbeat", node_id.c_str());
                return;
            }

            auto& node_info = it->second;
            node_info.last_heartbeat = msg->stamp;
        }
    );

    // Create registered node info
    RegisteredNodeInfo node_info;
    node_info.node_name     = request->node_name;
    node_info.node_type     = static_cast<NodeType>(request->node_type);
    node_info.node_namespace= request->node_namespace;
    node_info.capabilities  = request->capabilities;
    node_info.state_subscription        = state_subscription;
    node_info.heartbeat_subscription    = heartbeat_subscription;
    node_info.current_state     = LifecycleState(LifecycleStateMsg::PRIMARY_STATE_UNCONFIGURED, "Unconfigured");
    node_info.registration_time = this->now();
    node_info.last_heartbeat    = node_info.registration_time;

    // Store the node and bond ID mapping
    m_registered_nodes[node_id] = node_info;

    response->success   = true;
    response->node_id   = node_id;
    response->heartbeat_topic = heartbeat_topic;
    response->message   = "Node registered successfully with heartbeat monitoring";
    
    RCLCPP_INFO(this->get_logger(), "Node %s registered with given ID: %s, Heartbeat Topic: %s", 
                request->node_name.c_str(), node_id.c_str(), heartbeat_topic.c_str());
}

void LifeMonitor::HandleUnregisterNode(
    const std::shared_ptr<UnregisterNodeSrv::Request> request,
    std::shared_ptr<UnregisterNodeSrv::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Unregistering node: %s", request->node_name.c_str());
    
    std::unique_lock<std::shared_mutex> lock(m_nodes_mutex);
    
    auto it = m_registered_nodes.find(request->node_id);
    if (it != m_registered_nodes.end()) {
        it->second.state_subscription.reset();
        m_registered_nodes.erase(it);
        
        response->success = true;
        response->message = "Node unregistered successfully";
        RCLCPP_INFO(this->get_logger(), "Node %s unregistered", request->node_name.c_str());
    } else {
        response->success = false;
        response->message = "Node not found";
        RCLCPP_WARN(this->get_logger(), "Failed to unregister node %s - not found", 
                    request->node_name.c_str());
    }
}

void LifeMonitor::HandleGetRegisteredNodes(
    const std::shared_ptr<GetRegisteredNodesSrv::Request> request,
    std::shared_ptr<GetRegisteredNodesSrv::Response> response)
{
    (void)request;  
    RCLCPP_DEBUG(this->get_logger(), "Getting registered nodes list");
    
    std::shared_lock<std::shared_mutex> lock(m_nodes_mutex);
    
    for (const auto& pair : m_registered_nodes) {
        const auto& node_info   = pair.second;
        NodeInfoMsg info;
        
        info.node_id        = pair.first;
        info.node_name      = node_info.node_name;
        info.node_type      = static_cast<uint8_t>(node_info.node_type);
        info.node_namespace = node_info.node_namespace;
        info.capabilities   = node_info.capabilities;

        info.state_id       = node_info.current_state.id();
        info.state_label    = node_info.current_state.label();

        info.last_heartbeat = node_info.last_heartbeat;
        info.registration_time = node_info.registration_time;

        response->nodes.push_back(info);
    }
}

std::pair<std::string, std::string> LifeMonitor::GenerateRegistrationInfo(
    const std::string& node_name, 
    const std::string& node_namespace)
{
    std::stringstream ss;
    ss << node_name << "@" << node_namespace;
    return std::make_pair(ss.str(), "/" + node_name + "/bond");
}

}  // namespace core
}  // namespace flyscan

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<flyscan::core::LifeMonitor>(options);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    
    try {
        exec.spin();
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(
        rclcpp::get_logger("life_monitor"),
        "Unhandled exception in executor.spin(): %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}