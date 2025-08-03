# FlyScan Core Package

## Overview
The `flyscan_core` package provides the foundational infrastructure for the FlyScan system, including lifecycle management, node monitoring, and base class functionality. It implements a robust monitoring system that ensures system health and provides automatic recovery capabilities.

## Purpose
- Provide base class functionality for all FlyScan nodes
- Implement lifecycle management for system components
- Monitor node health and status through heartbeat mechanism
- Manage node registration and recovery processes
- Ensure system reliability and fault tolerance

## Components

### BaseNode Class
Abstract base class that all FlyScan nodes inherit from, providing standardized lifecycle management and monitoring integration.

**Features:**
- Lifecycle state management (Configure, Activate, Deactivate, Cleanup, Shutdown)
- Automatic registration with LifeMonitor
- Heartbeat publishing for health monitoring
- Recovery service handling
- Node type and capability management

**Services:**
- **Provides:** `/life_monitor/request_recovery` (RequestRecovery)
- **Consumes:** `/life_monitor/register_node`, `/life_monitor/unregister_node`

### LifeMonitor Node
Central monitoring node that tracks all registered nodes and manages system health.

**Features:**
- Node registration and unregistration management
- Heartbeat monitoring with timeout detection
- System-wide health status tracking
- Node recovery coordination
- Registration information management

**Services:**
- **Provides:** `/life_monitor/register_node`, `/life_monitor/unregister_node`, `/life_monitor/get_registered`
- **Subscribes to:** Node-specific heartbeat topics

### Constants
Centralized constant definitions for the core system.

**Categories:**
- Service endpoints and topic names
- QoS settings for publishers and subscribers
- Timeout and timing configurations

## Sequence Diagrams

### Node Initialization and Registration
```mermaid
sequenceDiagram
    participant App as Application
    participant Base as BaseNode
    participant Monitor as LifeMonitor
    participant ROS as ROS2 System

    App->>+Base: BaseNode(options, name, type, capabilities)
    Base->>Base: LifecycleNode(node_name, options)
    Base->>Base: Initialize member variables
    Base->>Base: Log initialization info
    Base-->>-App: Node created
    
    App->>+Base: configure() lifecycle transition
    Base->>Base: on_configure(state)
    Base->>ROS: create_client(register_node)
    Base->>ROS: create_client(unregister_node)
    Base->>ROS: create_service(request_recovery)
    
    Base->>+Monitor: RegisterWithLifeMonitor()
    Base->>Base: Create registration request
    Base->>Monitor: sync_send_request(register_request)
    Monitor->>Monitor: HandleRegisterNode()
    Monitor->>Monitor: GenerateRegistrationInfo()
    Monitor->>Monitor: Store node info in m_registered_nodes
    Monitor->>Monitor: Setup heartbeat subscription
    Monitor-->>Base: Registration response (node_id, heartbeat_topic)
    Base->>Base: SetupHeartbeatPublishing()
    Base->>ROS: create_publisher(heartbeat_topic)
    Base->>ROS: create_timer(heartbeat_period)
    Monitor-->>-Base: Registration complete
    
    Base->>Base: HandleConfigure() (virtual)
    Base-->>-App: Configuration success
```

### Heartbeat Monitoring Sequence
```mermaid
sequenceDiagram
    participant Base as BaseNode
    participant Timer as Heartbeat Timer
    participant Monitor as LifeMonitor
    participant Watchdog as Monitor Timer

    loop Heartbeat Publishing
        Timer->>+Base: Timer callback
        Base->>Base: Create NodeHeartbeatMsg
        Base->>Base: Set timestamp and node_id
        Base->>Monitor: Publish heartbeat message
        Base-->>-Timer: Heartbeat sent
    end
    
    loop Health Monitoring
        Monitor->>+Watchdog: Monitor timer callback
        Watchdog->>Monitor: Check all registered nodes
        
        loop For each registered node
            Monitor->>Monitor: Calculate time since last heartbeat
            alt Heartbeat timeout exceeded
                Monitor->>Monitor: HandleHeartbeatTimeout(node_id)
                Monitor->>Monitor: Log timeout warning
                Monitor->>Monitor: Optionally trigger recovery
            else Heartbeat within timeout
                Monitor->>Monitor: Continue monitoring
            end
        end
        
        Watchdog-->>-Monitor: Monitoring cycle complete
    end
```

### Node Lifecycle State Transitions
```mermaid
sequenceDiagram
    participant Client as Lifecycle Client
    participant Base as BaseNode
    participant Child as ChildNode
    participant Monitor as LifeMonitor

    Client->>+Base: configure() transition
    Base->>Base: on_configure(state)
    Base->>Monitor: Register with LifeMonitor
    Base->>Child: HandleConfigure() (virtual)
    Child-->>Base: OperationStatus::kOK
    Base-->>Client: SUCCESS
    
    Client->>+Base: activate() transition
    Base->>Base: on_activate(state)
    Base->>Child: HandleActivate() (virtual)
    Child-->>Base: OperationStatus::kOK
    Base-->>Client: SUCCESS
    
    Client->>+Base: deactivate() transition
    Base->>Base: on_deactivate(state)
    Base->>Child: HandleDeactivate() (virtual)
    Child-->>Base: OperationStatus::kOK
    Base-->>Client: SUCCESS
    
    Client->>+Base: cleanup() transition
    Base->>Base: on_cleanup(state)
    Base->>Child: HandleCleanup() (virtual)
    Child-->>Base: OperationStatus::kOK
    Base-->>Client: SUCCESS
    
    Client->>+Base: shutdown() transition
    Base->>Base: on_shutdown(state)
    Base->>Monitor: Unregister from LifeMonitor
    Base->>Child: HandleShutdown() (virtual)
    Child-->>Base: OperationStatus::kOK
    Base-->>Client: SUCCESS
```

### Service Request Handling
```mermaid
sequenceDiagram
    participant Node as External Node
    participant Monitor as LifeMonitor
    participant Storage as Node Registry

    Node->>+Monitor: RegisterNode service request
    Monitor->>Monitor: HandleRegisterNode()
    Monitor->>Monitor: GenerateRegistrationInfo(name, namespace)
    Monitor->>Monitor: Create RegisteredNodeInfo struct
    
    Monitor->>+Storage: Store in m_registered_nodes[node_id]
    Storage->>Storage: Set node_name, namespace, type, capabilities
    Storage->>Storage: Set registration_time = now()
    Storage->>Storage: Initialize current_state
    Storage-->>-Monitor: Storage complete
    
    Monitor->>Monitor: Create heartbeat subscription
    Monitor->>Monitor: Create state subscription (if needed)
    Monitor-->>-Node: Response with node_id and heartbeat_topic
    
    Node->>+Monitor: UnregisterNode service request
    Monitor->>Monitor: HandleUnregisterNode()
    Monitor->>+Storage: Remove from m_registered_nodes[node_id]
    Storage-->>-Monitor: Removal complete
    Monitor-->>-Node: Unregistration success
    
    Node->>+Monitor: GetRegisteredNodes service request
    Monitor->>Monitor: HandleGetRegisteredNodes()
    Monitor->>+Storage: Read all registered nodes
    Storage-->>Monitor: Node list data
    Monitor->>Monitor: Create NodeInfo messages
    Monitor-->>-Node: List of registered nodes
```

### Recovery Process Flow
```mermaid
sequenceDiagram
    participant Monitor as LifeMonitor
    participant Base as BaseNode
    participant Recovery as Recovery System
    participant Child as ChildNode

    Monitor->>Monitor: HandleHeartbeatTimeout(node_id)
    Monitor->>Monitor: Log timeout warning
    
    opt Recovery enabled
        Monitor->>+Base: RequestRecovery service call
        Base->>Base: HandleRecoveryRequest()
        Base->>Child: Implement recovery logic (virtual)
        Child->>Child: Attempt state recovery
        Child->>Child: Reset internal state
        Child->>Child: Restart critical processes
        Child-->>Base: Recovery status
        Base-->>-Monitor: Recovery response
        
        alt Recovery successful
            Monitor->>Monitor: Log recovery success
            Monitor->>Monitor: Continue monitoring
        else Recovery failed
            Monitor->>Monitor: Log recovery failure
            Monitor->>Recovery: Escalate to system recovery
        end
    end
```

## Key Features
- **Lifecycle Management**: Standardized state transitions for all system nodes
- **Health Monitoring**: Continuous heartbeat-based health checking
- **Automatic Registration**: Seamless integration with system monitoring
- **Recovery Capabilities**: Built-in recovery mechanisms for failed nodes
- **Thread Safety**: Thread-safe node registry with shared mutex protection
- **Service Integration**: Comprehensive service-based communication
- **Extensible Design**: Virtual methods for custom node behavior

## Design Patterns
- **Template Method Pattern**: BaseNode provides lifecycle framework with virtual hooks
- **Observer Pattern**: LifeMonitor observes node health through heartbeats
- **Registry Pattern**: Centralized node registration and management
- **Command Pattern**: Service-based recovery and management commands
- **RAII**: Automatic resource management through destructors

## Dependencies
- `rclcpp`: ROS2 C++ client library
- `rclcpp_lifecycle`: Lifecycle node support
- `rclcpp_components`: Component registration
- `flyscan_common`: Common types and utilities
- `flyscan_interfaces`: Custom message and service definitions
- `lifecycle_msgs`: Standard lifecycle messages

## Usage Examples

### Creating a Custom Node
```cpp
#include "flyscan_core/base_node.hpp"

class MyCustomNode : public flyscan::core::BaseNode
{
public:
    MyCustomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : BaseNode(options, "my_custom_node", 
                   flyscan::common::NodeType::kPerception,
                   {"sensor_processing", "data_analysis"})
    {
    }

protected:
    OperationStatus HandleConfigure() override {
        // Custom configuration logic
        return OperationStatus::kOK;
    }
    
    OperationStatus HandleActivate() override {
        // Custom activation logic
        return OperationStatus::kOK;
    }
};
```

### Lifecycle Management
```cpp
auto node = std::make_shared<MyCustomNode>();
auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
executor->add_node(node->get_node_base_interface());

// Transition through lifecycle states
node->configure();
node->activate();
// Node is now active and monitored
node->deactivate();
node->cleanup();
```

### Monitoring System Status
```cpp
auto monitor = std::make_shared<flyscan::core::LifeMonitor>(rclcpp::NodeOptions());
// LifeMonitor automatically handles registration and monitoring
// Use GetRegisteredNodes service to query system status
```