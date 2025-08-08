# Waypoint Mission System

The waypoint mission system allows you to define and execute predefined flight plans using YAML configuration files. The drone will autonomously navigate through a sequence of waypoints, performing specified actions at each location.

## Features

- **YAML-based Mission Definition**: Easy-to-edit mission files with waypoints, timings, and actions
- **Autonomous Execution**: Full integration with PX4 controller for safe navigation
- **Real-time Point Cloud Navigation**: Uses depth camera for obstacle avoidance (no reliance on unstable 2D maps)
- **Altitude Safety**: Enforces minimum 1m flight altitude throughout the mission
- **Flexible Mission Control**: Start, pause, resume, and abort missions programmatically
- **Completion Actions**: Configure what happens when mission completes (hover, land, RTL)

## Quick Start

### 1. Launch the Waypoint Mission Node

```bash
# Launch with default sample mission
ros2 launch flyscan_exploration waypoint_mission.launch.py

# Launch with custom mission file
ros2 launch flyscan_exploration waypoint_mission.launch.py mission_file:=/path/to/your_mission.yaml

# Launch without auto-start (manual control)
ros2 launch flyscan_exploration waypoint_mission.launch.py auto_start:=false
```

### 2. Control Mission Execution

```bash
# Check if mission mode is available (should show mode 3 = MISSION)
ros2 service call /px4_controller/set_control_mode flyscan_interfaces/srv/SetControlMode "{mode: 3}"

# If auto_start:=false, you can manually start missions via ROS 2 service calls
# (Service interface can be added in future iterations)
```

## Mission File Format

### Basic Structure

```yaml
mission:
  name: "your_mission_name"
  description: "Description of what this mission does"
  
  # Mission parameters
  waypoint_tolerance: 0.5      # Distance tolerance to consider waypoint reached (m)
  hover_time: 3.0              # Default hover time at each waypoint (seconds)
  max_velocity: 1.0            # Maximum velocity between waypoints (m/s)
  safety_margin: 1.5           # Safety margin for obstacle avoidance (m)
  
  # Waypoint sequence
  waypoints:
    - position: [x, y, z]      # Position in map frame (NED: negative z = up)
      yaw: 0.0                 # Heading in degrees
      work_time: 2.0           # Time to hover at this waypoint (seconds)
      description: "Optional description"
      
  # Mission completion
  completion_action: "hover"    # Options: "hover", "land", "rtl"
  completion_position: [0, 0, -1.5]  # Where to go after completion
```

### Example Mission Files

#### Simple Survey Mission
```yaml
mission:
  name: "rectangular_survey"
  description: "Survey a rectangular area"
  waypoint_tolerance: 0.5
  hover_time: 2.0
  max_velocity: 1.0
  safety_margin: 1.5
  
  waypoints:
    - position: [0, 0, -1.5]    # Start: origin, 1.5m altitude
      yaw: 0.0
      work_time: 1.0
      description: "Takeoff point"
      
    - position: [10, 0, -2.0]   # East 10m, climb to 2m
      yaw: 90.0
      work_time: 3.0
      description: "Survey point 1"
      
    - position: [10, 10, -2.0]  # North 10m
      yaw: 180.0
      work_time: 3.0
      description: "Survey point 2"
      
    - position: [0, 10, -2.0]   # West 10m
      yaw: 270.0
      work_time: 3.0
      description: "Survey point 3"
      
    - position: [0, 0, -1.5]    # Return home, descend
      yaw: 0.0
      work_time: 2.0
      description: "Landing point"
  
  completion_action: "hover"
  completion_position: [0, 0, -1.5]
```

#### Inspection Mission
```yaml
mission:
  name: "structure_inspection"
  description: "Inspect a structure from multiple angles"
  waypoint_tolerance: 0.3
  hover_time: 5.0              # Longer hover for detailed inspection
  max_velocity: 0.5            # Slower for precision
  safety_margin: 2.0           # Larger safety margin near structures
  
  waypoints:
    - position: [5, 0, -2.0]    # Approach from east
      yaw: 270.0                # Face west toward structure
      work_time: 8.0            # Extended inspection time
      description: "East inspection point"
      
    - position: [0, 5, -2.5]    # North side, higher altitude
      yaw: 180.0                # Face south toward structure
      work_time: 8.0
      description: "North inspection point"
      
    - position: [-5, 0, -2.0]   # West side
      yaw: 90.0                 # Face east toward structure
      work_time: 8.0
      description: "West inspection point"
  
  completion_action: "rtl"      # Return to launch when complete
```

## Coordinate System

- **Frame**: All positions are in the `map` frame
- **Units**: Positions in meters, yaw in degrees
- **Altitude**: Uses NED (North-East-Down) convention:
  - Positive Z = downward (below ground)
  - Negative Z = upward (above ground)
  - `-1.5` = 1.5 meters above ground level
  - `-2.0` = 2.0 meters above ground level

## Safety Features

### Altitude Safety
- **Minimum altitude**: 1m above ground is enforced automatically
- **Gradual navigation**: Point cloud-based step-by-step movement
- **Real-time obstacle detection**: Uses depth camera data, not unreliable 2D maps

### Navigation Safety
- **Point cloud obstacle avoidance**: Real-time detection using `/camera/depth/points`
- **Configurable safety margins**: Adjustable clearance around obstacles
- **Progressive movement**: Small steps with continuous obstacle checking
- **Fallback behavior**: Alternative path finding when direct routes are blocked

### Mission Safety
- **State validation**: Comprehensive state machine with error handling
- **Service timeouts**: Prevents hanging on unresponsive services
- **Abort capability**: Emergency abort functionality
- **Mode switching verification**: Confirms controller mode changes

## Mission States

The mission system uses these states:

- `kIdle`: Mission not started
- `kLoaded`: Mission loaded from file, ready to start
- `kSwitchingMode`: Switching PX4 controller to mission mode
- `kNavigating`: Moving to current waypoint
- `kWorking`: Hovering and performing work at waypoint
- `kCompleted`: Mission finished successfully
- `kAborted`: Mission stopped due to error
- `kPaused`: Mission temporarily paused

## Parameters

### Launch Parameters
- `mission_file`: Path to YAML mission file
- `auto_start`: Automatically start mission when node activates
- `mission_rate`: Mission update rate in Hz (default: 5.0)

### Runtime Parameters
- `waypoint_tolerance`: Distance to consider waypoint reached
- `hover_time`: Default time to hover at waypoints
- `max_velocity`: Maximum navigation velocity
- `safety_margin`: Obstacle avoidance safety margin

## Integration with PX4 Controller

The waypoint mission system integrates seamlessly with the enhanced PX4 controller:

1. **Mode Switching**: Automatically switches to `MISSION` mode (mode 3)
2. **Point Cloud Navigation**: Uses real-time depth camera data for safe navigation
3. **Altitude Enforcement**: Respects minimum flight altitude settings
4. **Service Integration**: Uses `/px4_controller/navigate_to_pose` for movement commands

## Troubleshooting

### Mission Won't Start
- Check if PX4 controller is running: `ros2 node list | grep px4_controller`
- Verify mission file syntax: ensure valid YAML and required fields
- Check mode switching: `ros2 service call /px4_controller/set_control_mode ...`

### Navigation Issues
- Ensure depth camera is publishing: `ros2 topic echo /camera/depth/points --max_wait_time 1`
- Check for obstacle map conflicts: disable other navigation nodes
- Verify waypoint altitudes: ensure negative values for above-ground positions

### Performance Issues
- Adjust `mission_rate` parameter (lower for slower systems)
- Increase `waypoint_tolerance` for less precise waypoint following
- Reduce `max_velocity` for more stable navigation

## Future Enhancements

- **Mission progress visualization** in RViz
- **Dynamic waypoint modification** during execution
- **GPS waypoint support** for outdoor missions
- **Advanced completion actions** (custom behaviors)
- **Mission templates** for common survey patterns
- **Sensor-triggered actions** at waypoints
- **Multi-drone mission coordination**

## Example Usage

```bash
# Terminal 1: Launch PX4 controller
ros2 launch flyscan_bringup px4_controller_composed.launch.py

# Terminal 2: Launch waypoint mission with sample mission
ros2 launch flyscan_exploration waypoint_mission.launch.py

# Terminal 3: Monitor progress
ros2 topic echo /px4_controller/navigation_status
```

The drone will automatically:
1. Load the sample mission (5 waypoints in rectangular pattern)
2. Switch PX4 controller to mission mode
3. Navigate to each waypoint using point cloud obstacle avoidance
4. Hover at each waypoint for the specified duration
5. Complete the mission and hover at the final position

Safe, autonomous, and reliable waypoint execution!