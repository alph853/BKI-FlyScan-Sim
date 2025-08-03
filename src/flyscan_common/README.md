# FlyScan Common Package

## Overview
The `flyscan_common` package provides shared utility functions, type definitions, enums, and common functionality used across all FlyScan system components. It serves as the foundational library ensuring consistency and code reuse throughout the project.

## Purpose
- Define common data types and enumerations
- Provide mathematical utility functions for drone operations
- Offer string formatting utilities for logging and display
- Implement synchronous service request utilities
- Establish standardized return codes and status definitions

## Components

### Header Files

#### enums.hpp
Defines common enumerations used throughout the FlyScan system.

**Enumerations:**
- `OperationStatus`: Function return status codes (OK, Pending, OutOfMemory, etc.)
- `NodeType`: Classification of node types (Navigation, Perception, Controller, etc.)  
- `ControlMode`: Drone control modes (Manual, Teleop, Autonomous, Mission, RTL, Land)

**Utility Functions:**
- `OperationStatusToString()`: Convert status enum to string
- `NodeTypeToString()`: Convert node type enum to string
- `ControlModeToString()`: Convert control mode enum to string

#### math_util.hpp
Mathematical utility functions for drone control calculations.

**Functions:**
- `constrainAngle()`: Constrain angle to [-180, 180] degrees
- `calculateDistance3D()`: 3D Euclidean distance calculation
- `calculateDistance2D()`: 2D Euclidean distance calculation
- `isWithinTolerance()`: Tolerance-based value comparison

**Constants:**
- `DEG_TO_RAD`: Degrees to radians conversion factor
- `RAD_TO_DEG`: Radians to degrees conversion factor

#### string_util.hpp
String formatting utilities for display and logging.

**Functions:**
- `formatPosition()`: Format NED coordinates as readable string
- `formatFloat()`: Format float with specified precision
- `formatPercentage()`: Format value as percentage string

#### types.hpp
Type aliases and definitions for ROS2 message types and service definitions.

**Namespaces:**
- `core_types`: Lifecycle and service type aliases
- `vision_types`: Sensor message type aliases

#### request_util.hpp
Utility functions for synchronous ROS2 service requests.

**Functions:**
- `sync_send_request()`: Synchronous service request with timeout handling

## Sequence Diagrams

### Service Request Utility Flow
```mermaid
sequenceDiagram
    participant Client as Service Client
    participant Util as sync_send_request()
    participant Server as Service Server
    participant ROS as ROS2 System

    Client->>+Util: sync_send_request(client, request, timeouts)
    Util->>Util: Validate client and node_base
    
    loop Service Availability Check
        Util->>Server: client->service_is_ready()
        alt Service not ready
            Server-->>Util: false
            Util->>Util: Check timeout
            Util->>Util: sleep_for(100ms)
        else Service ready
            Server-->>Util: true
            Note over Util: Break loop
        end
    end
    
    Util->>Server: client->async_send_request(request)
    Server->>Server: Process request
    
    Util->>ROS: rclcpp::spin_until_future_complete(future, timeout)
    
    alt SUCCESS
        ROS-->>Util: FutureReturnCode::SUCCESS
        Util->>Util: future.get()
        Util-->>Client: Response::SharedPtr
    else TIMEOUT
        ROS-->>Util: FutureReturnCode::TIMEOUT
        Util->>Util: throw runtime_error("timed out")
        Util-->>Client: Exception
    else INTERRUPTED
        ROS-->>Util: FutureReturnCode::INTERRUPTED
        Util->>Util: throw runtime_error("interrupted")
        Util-->>Client: Exception
    end
    
    Util-->>-Client: Response or Exception
```

### Enum Conversion Sequence
```mermaid
sequenceDiagram
    participant App as Application Code
    participant Enum as Enum Functions
    participant Output as String Output

    App->>+Enum: OperationStatusToString(status)
    Enum->>Enum: switch(status)
    
    alt kOK
        Enum->>Output: "SUCCESS"
    else kPending
        Enum->>Output: "PENDING"
    else kOutOfMemory
        Enum->>Output: "OUT_OF_MEMORY"
    else kMalformedInput
        Enum->>Output: "MALFORMED_INPUT"
    else kNotInitialized
        Enum->>Output: "NOT_INITIALIZED"
    else kServiceNA
        Enum->>Output: "SERVICE_NOT_AVAILABLE"
    else kTimeout
        Enum->>Output: "TIMEOUT"
    else kNotImplemented
        Enum->>Output: "NOT_IMPLEMENTED"
    else default
        Enum->>Output: "UNKNOWN"
    end
    
    Enum-->>-App: String representation
```

### Mathematical Utility Usage
```mermaid
sequenceDiagram
    participant Control as Control System
    participant Math as Math Utilities
    participant Result as Calculation Result

    Control->>+Math: constrainAngle(angle_deg)
    Math->>Math: while(angle > 180.0f) angle -= 360.0f
    Math->>Math: while(angle < -180.0f) angle += 360.0f
    Math-->>Control: Constrained angle
    
    Control->>+Math: calculateDistance3D(x1,y1,z1, x2,y2,z2)
    Math->>Math: dx = x2 - x1
    Math->>Math: dy = y2 - y1
    Math->>Math: dz = z2 - z1
    Math->>Math: sqrt(dx*dx + dy*dy + dz*dz)
    Math-->>-Control: 3D distance
    
    Control->>+Math: isWithinTolerance(value, target, tolerance)
    Math->>Math: abs(value - target) <= tolerance
    Math-->>-Control: Boolean result
```

### String Formatting Flow
```mermaid
sequenceDiagram
    participant Logger as Logging System
    participant Format as String Utilities
    participant Stream as StringStream

    Logger->>+Format: formatPosition(north, east, down, precision)
    Format->>+Stream: stringstream ss
    Format->>Stream: ss << fixed << setprecision(precision)
    Format->>Stream: ss << "N:" << north << "m, E:" << east << "m, D:" << down << "m"
    Stream->>Format: Formatted string
    Format-->>-Logger: Position string
    
    Logger->>+Format: formatFloat(value, precision)
    Format->>+Stream: stringstream ss
    Format->>Stream: ss << fixed << setprecision(precision) << value
    Stream->>Format: Formatted string
    Format-->>-Logger: Float string
    
    Logger->>+Format: formatPercentage(value, precision)
    Format->>+Stream: stringstream ss
    Format->>Stream: ss << fixed << setprecision(precision) << (value * 100.0f) << "%"
    Stream->>Format: Formatted string
    Format-->>-Logger: Percentage string
```

### Type Definition Usage
```mermaid
sequenceDiagram
    participant Node as ROS2 Node
    participant Types as Type Definitions
    participant Message as Message System

    Node->>+Types: core_types::NodeHeartbeatMsg
    Types-->>Node: flyscan_interfaces::msg::NodeHeartbeat
    
    Node->>+Types: core_types::RegisterNodeSrv
    Types-->>Node: flyscan_interfaces::srv::RegisterNode
    
    Node->>+Types: vision_types::ImageMsg
    Types-->>Node: sensor_msgs::msg::Image
    
    Node->>+Types: vision_types::OdometryMsg
    Types-->>Node: nav_msgs::msg::Odometry
    
    Node->>Message: Use aliased types for consistency
```

## Key Features
- **Centralized Definitions**: Common enums and types used system-wide
- **Mathematical Utilities**: Optimized inline functions for drone calculations
- **Service Utilities**: Robust synchronous service request handling
- **String Formatting**: Consistent formatting for logging and display
- **Type Safety**: Strong typing through enumerations
- **Header-only Design**: All utilities implemented as inline functions for performance

## Design Patterns
- **Namespace Organization**: Clear separation of functionality domains
- **Template-based Service Utilities**: Generic service request handling
- **Inline Functions**: Zero-overhead utility functions
- **Exception Safety**: Proper error handling in service utilities
- **Const Correctness**: Immutable constants and proper const usage

## Dependencies
- `rclcpp`: ROS2 C++ client library
- `rclcpp_lifecycle`: Lifecycle node support
- `flyscan_interfaces`: Custom message and service definitions
- `sensor_msgs`: Standard sensor message types
- `nav_msgs`: Navigation message types
- Standard C++ libraries: `<cmath>`, `<string>`, `<future>`, `<chrono>`

## Usage Examples

### Using Enumerations
```cpp
#include "flyscan_common/enums.hpp"
using namespace flyscan::common;

OperationStatus status = OperationStatus::kOK;
std::string status_str = OperationStatusToString(status); // "SUCCESS"

ControlMode mode = ControlMode::kAutonomous;
std::string mode_str = ControlModeToString(mode); // "AUTONOMOUS"
```

### Mathematical Calculations
```cpp
#include "flyscan_common/math_util.hpp"
using namespace flyscan::common;

float angle = 190.0f;
float constrained = constrainAngle(angle); // -170.0f

float distance = calculateDistance3D(0,0,0, 3,4,5); // 7.07...
bool within = isWithinTolerance(10.1f, 10.0f, 0.2f); // true
```

### Service Requests
```cpp
#include "flyscan_common/request_util.hpp"
using namespace flyscan::common;

auto response = sync_send_request<MyService>(
    node_base, client, request,
    std::chrono::milliseconds(2000), // service timeout
    std::chrono::milliseconds(5000)  // request timeout
);
```