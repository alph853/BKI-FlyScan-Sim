# FLyScan Drone Controller

## Overview

The FLyScan Drone Controller is a comprehensive PX4 autopilot control system built using MAVSDK C++. This package provides complete control over PX4-based drones, including teleoperation, autonomous flights, mission execution, and advanced sensor integration for scanning applications.

## Architecture

The controller consists of two main components:

1. **PX4Controller** (`px4_controller.hpp/cpp`) - Core MAVSDK integration and flight control
2. **ControllerNode** (`controller_node.cpp`) - ROS 2 wrapper for external system integration

## Core Components

### 1. Connection Management
- **Connection Types**: Supports UDP, TCP, and serial connections to PX4 autopilot
- **System Discovery**: Automatic discovery and initialization of autopilot systems
- **Health Monitoring**: Continuous monitoring of system health and connection status

### 2. Flight Control System

#### Basic Flight Operations
- **Arming/Disarming**: Safe vehicle arming with pre-flight checks
- **Takeoff/Landing**: Automated takeoff to specified altitude and landing sequences
- **Return to Launch (RTL)**: Emergency return to home position
- **Emergency Stop**: Immediate disarm and stop all operations

#### Advanced Flight Modes

##### Manual Teleoperation
Real-time keyboard control with the following mapping:
- `W/S`: Forward/Backward movement
- `A/D`: Left/Right movement  
- `Q/E`: Up/Down movement
- `J/L`: Yaw rotation (left/right)
- `SPACE`: Stop all movement
- `ESC`: Exit teleop mode

##### Offboard Control
Precise position and velocity control using MAVSDK offboard mode:
- **Position Control**: NED (North-East-Down) coordinate positioning
- **Velocity Control**: Body-frame velocity commands with yaw rate
- **Real-time Updates**: 20Hz control loop for smooth operation

##### Autonomous Missions
Supports multiple mission patterns:
- **Square Pattern**: Rectangular flight path with configurable size
- **Circle Pattern**: Circular flight path approximated with waypoints
- **Figure-8 Pattern**: Complex figure-8 maneuver with varying altitude

### 3. Sensor Integration

#### Camera System
The drone integrates with MAVLink-compatible cameras for:
- **Photo Capture**: Single photo capture with automatic mode switching
- **Video Recording**: Start/stop video recording capabilities
- **Mode Management**: Automatic switching between photo and video modes

*Note: Camera operations require a MAVLink-compatible camera with component ID 1*

#### Gimbal Control
3-axis gimbal control for camera stabilization and pointing:
- **Attitude Control**: Precise roll, pitch, and yaw positioning
- **Stabilization Modes**: YawLock and YawFollow modes
- **Real-time Control**: Smooth gimbal movements during flight

*Component: The gimbal system provides camera stabilization and precise pointing for scanning applications*

### 4. Telemetry System

#### Real-time Data Streaming
- **Position**: GPS coordinates (latitude, longitude, altitude)
- **Attitude**: Euler angles (roll, pitch, yaw)
- **Velocity**: NED frame velocity vectors
- **Battery**: Voltage, current, and remaining percentage
- **Flight Status**: Armed state, in-air status, flight mode

#### Data Rates
- Position: 2 Hz
- Velocity: 2 Hz  
- Altitude: 5 Hz
- Battery: 1 Hz

### 5. Parameter Management

#### MAVSDK Parameters
The system provides access to all PX4 parameters through MAVSDK:

##### Flight Control Parameters
- `MPC_XY_VEL_MAX`: Maximum horizontal velocity (m/s)
- `MPC_Z_VEL_MAX_UP`: Maximum ascent velocity (m/s)
- `MPC_Z_VEL_MAX_DN`: Maximum descent velocity (m/s)
- `MPC_TKO_SPEED`: Takeoff velocity (m/s)
- `MPC_LAND_SPEED`: Landing velocity (m/s)

##### Navigation Parameters
- `NAV_RCL_ACT`: Return mode action
- `NAV_RCL_LT`: Return mode loiter time
- `RTL_RETURN_ALT`: Return altitude (m)

##### Safety Parameters
- `COM_RC_LOSS_T`: RC loss timeout (s)
- `COM_OF_LOSS_T`: Optical flow loss timeout (s)
- `BAT_LOW_THR`: Low battery threshold (V)
- `BAT_CRIT_THR`: Critical battery threshold (V)

##### Sensor Parameters
- `CAL_ACC*`: Accelerometer calibration values
- `CAL_GYRO*`: Gyroscope calibration values
- `CAL_MAG*`: Magnetometer calibration values

*Parameter Management: Use `getParamFloat()`/`setParamFloat()` for float parameters and `getParamInt()`/`setParamInt()` for integer parameters*

## Hardware Requirements

### Autopilot
- PX4-compatible flight controller (Pixhawk 4, Pixhawk 6C, etc.)
- Minimum firmware version: PX4 v1.12+

### Optional Components
- **Gimbal**: 3-axis MAVLink-compatible gimbal (e.g., Storm32, Tarot)
- **Camera**: MAVLink camera or camera connected via gimbal
- **Companion Computer**: For running this software (Raspberry Pi 4, Jetson Nano, etc.)

## Software Dependencies

### Required
- **MAVSDK**: C++ library for MAVLink communication
- **ROS 2**: Robot Operating System 2 (Humble/Iron/Rolling)
- **pthread**: For threading support

### Optional
- **OpenCV**: For camera integration (if using custom camera interface)
- **GeographicLib**: For advanced GPS coordinate transformations

## Usage Examples

### Basic Connection and Takeoff
```cpp
#include "flyscan_drone_controller/px4_controller.hpp"

PX4Controller controller;

// Connect to PX4 via UDP (default SITL connection)
if (controller.connect("udp://:14540")) {
    controller.setupTelemetry();
    
    if (controller.waitForReady(30)) {
        controller.arm();
        controller.takeoff(10.0f); // 10 meters altitude
        
        // Perform operations...
        
        controller.land();
        controller.disarm();
    }
}
```

### Mission Execution
```cpp
// Create and execute a circular mission
if (controller.createSampleMission(1, 50.0f)) { // Circle pattern, 50m radius
    controller.startMission();
    
    // Monitor mission progress...
    // Mission will execute autonomously
}
```

### Manual Control
```cpp
// Start teleoperation mode
controller.startTeleopMode();
// User can now control with keyboard
// ESC to exit teleop mode
```

## Safety Features

### Pre-flight Checks
- System health verification
- GPS fix validation
- Battery level checking
- Sensor calibration status

### Runtime Safety
- Connection monitoring with automatic failsafe
- Battery monitoring with low-level warnings
- Emergency stop functionality
- Graceful shutdown on connection loss

### Operational Limits
- Maximum velocity limits enforced
- Altitude boundaries respected
- Geofence compliance (if configured)

## Troubleshooting

### Common Issues

#### Connection Problems
- **Symptom**: "No autopilot found within timeout"
- **Solution**: Check connection string, ensure PX4 is running, verify network connectivity

#### MAVSDK Version Issues  
- **Symptom**: Compilation errors with camera/gimbal functions
- **Solution**: This package is compatible with MAVSDK v2.0+. Older versions require API modifications.

#### Telemetry Issues
- **Symptom**: No telemetry data received
- **Solution**: Call `setupTelemetry()` after successful connection, check parameter `SER_TEL1_BAUD`

#### Mission Upload Failures
- **Symptom**: Mission upload returns error
- **Solution**: Ensure vehicle is armed, check GPS lock, verify mission item validity

### Debugging Tips

1. **Enable Debug Logging**: Set log callback to capture detailed MAVSDK messages
2. **Check System Status**: Use `printStatus()` to monitor real-time system state
3. **Verify Parameters**: Check critical PX4 parameters affecting flight behavior
4. **Monitor Telemetry**: Observe position, velocity, and attitude data for anomalies

## Configuration

### Connection Strings
- **SITL/Gazebo**: `"udp://:14540"`
- **Real Hardware (USB)**: `"serial:///dev/ttyACM0:57600"`
- **Real Hardware (Telemetry)**: `"udp://192.168.1.100:14550"`
- **TCP Connection**: `"tcp://192.168.1.100:5760"`

### ROS 2 Integration
The ControllerNode provides ROS 2 services for external integration:
- `/flyscan/arm`: Arm the vehicle
- `/flyscan/takeoff`: Takeoff to default altitude
- `/flyscan/land`: Land the vehicle
- `/flyscan/rtl`: Return to launch
- `/flyscan/sample_flight`: Execute sample flight pattern

## Development

### Building
```bash
cd flyscan_ws
colcon build --packages-select flyscan_drone_controller
source install/setup.bash
```

### Testing
```bash
# Launch PX4 SITL first
make px4_sitl gazebo

# Run the controller
ros2 run flyscan_drone_controller controller_node
```

## License

This project is licensed under the MIT License. See LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Implement changes with proper testing
4. Submit a pull request with detailed description

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review PX4 documentation: https://docs.px4.io/
3. MAVSDK documentation: https://mavsdk.mavlink.io/
4. Create an issue in the project repository