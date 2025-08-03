# FlyScan Bringup Package

## Overview

The `flyscan_bringup` package serves as the main orchestration layer for the FlyScan system. It contains launch files that coordinate the startup of all system components, from simulation environment to perception and control nodes.

## Purpose

- Orchestrate system-wide launch sequences
- Manage node dependencies and startup timing
- Configure different operational modes (simulation vs deployment)
- Coordinate inter-node communication and data flow
- Provide centralized configuration management

## Components

### Main Launch Files

#### flyscan.launch.py

Primary system launcher that brings up the complete FlyScan stack.

**Launch Arguments:**

- `mode`: Operation mode ('sim' or 'deploy')
- `use_sim_time`: Use simulation time synchronization

**Launched Components:**

- PX4 simulation environment (sim mode only)
- Core system utilities
- Life monitoring system
- PX4 controller
- Semantic perception pipeline
- RTAB-Map SLAM (sim mode only)

#### utils.launch.py

Utility components launcher providing essential bridge and visualization services.

**Launch Arguments:**

- `use_sim_time`: Simulation time flag
- `mode`: Operation mode
- `rtabmap_launch`: Enable RTAB-Map
- `life_monitor`: Enable life monitoring
- `px4_controller`: Enable PX4 controller
- `semantic_perception`: Enable perception pipeline

**Launched Components:**

- PX4-ROS2 bridge
- Gazebo-ROS2 bridge (sim mode)
- Static transform publishers
- Visualization nodes
- RViz2 interface

#### rtabmap_3d.launch.py

RTAB-Map 3D SLAM launcher for mapping and localization.

**Launch Arguments:**

- `mode`: Operation mode
- `use_sim_time`: Simulation time flag
- `launch_viz`: Enable RTAB-Map visualization
- `log_level`: RTAB-Map logging level

## Sequence Diagrams

### System Startup Sequence

```mermaid
sequenceDiagram
    participant User as User/Launcher
    participant Main as flyscan.launch.py
    participant Utils as utils.launch.py
    participant PX4Sim as PX4 Simulation
    participant RTABMap as RTAB-Map

    User->>+Main: ros2 launch flyscan_bringup flyscan.launch.py
    Main->>Main: DeclareLaunchArgument(mode, use_sim_time)
    
    alt mode == 'sim'
        Main->>+PX4Sim: IncludeLaunchDescription(px4_sim.launch.py)
        PX4Sim->>PX4Sim: Start Gazebo simulation
        PX4Sim->>PX4Sim: Spawn drone model
        PX4Sim-->>-Main: Simulation ready
    end
    
    Main->>+Utils: IncludeLaunchDescription(utils.launch.py)
    Utils->>Utils: Launch bridge nodes
    Utils->>Utils: Setup static transforms
    Utils->>Utils: Start visualization nodes
    Utils-->>-Main: Utils ready
    
    Main->>Main: Node(life_monitor)
    Main->>Main: Node(px4_controller)
    Main->>Main: Node(semantic_perception)
    
    alt mode == 'sim'
        Main->>+RTABMap: IncludeLaunchDescription(rtabmap_3d.launch.py)
        RTABMap->>RTABMap: Initialize SLAM system
        RTABMap->>RTABMap: Setup subscriptions
        RTABMap-->>-Main: SLAM ready
    end
    
    Main-->>-User: System fully launched
```

### Utils Launch Sequence

```mermaid
sequenceDiagram
    participant Launcher as Launch System
    participant Utils as utils.launch.py
    participant Bridge as PX4 Bridge
    participant GZ as Gazebo Bridge
    participant TF as Transform System
    participant Viz as Visualization

    Launcher->>+Utils: generate_launch_description()
    Utils->>Utils: DeclareLaunchArgument(all parameters)
    
    Utils->>+Bridge: Node(px4_ros_bridge)
    Bridge->>Bridge: Subscribe to /fmu/out/vehicle_odometry
    Bridge->>Bridge: Publish to /odom
    Bridge-->>-Utils: Bridge active
    
    alt mode == 'sim'
        Utils->>+GZ: Node(gz_bridge with camera/imu topics)
        GZ->>GZ: Bridge camera topics
        GZ->>GZ: Bridge IMU data
        GZ->>GZ: Bridge clock signals
        GZ-->>-Utils: Gazebo bridge active
    end
    
    Utils->>+TF: static_transform_publisher(camera→base_link)
    Utils->>TF: static_transform_publisher(imu→base_link)
    TF->>TF: Publish static transforms
    TF-->>-Utils: Transforms published
    
    alt mode == 'sim'
        Utils->>+Viz: Node(visualization_node)
        Utils->>Viz: Node(video_streamer)
        Utils->>Viz: ExecuteProcess(rviz2)
        Viz->>Viz: Setup visualization pipeline
        Viz-->>-Utils: Visualization ready
    end
    
    Utils->>Utils: TimerAction(3.0s delay for controller/perception)
    Utils-->>-Launcher: All utils launched
```

### RTAB-Map Launch Sequence

```mermaid
sequenceDiagram
    participant Launcher as Launch System
    participant RTABMap as rtabmap_3d.launch.py
    participant Config as Configuration
    participant SLAMNode as RTAB-Map SLAM
    participant VizNode as RTAB-Map Viz

    Launcher->>+RTABMap: generate_launch_description()
    RTABMap->>RTABMap: DeclareLaunchArgument(mode, use_sim_time, launch_viz, log_level)
    
    RTABMap->>+Config: PathJoinSubstitution(rtabmap_config.yaml)
    Config-->>-RTABMap: Configuration path resolved
    
    RTABMap->>+SLAMNode: Node(rtabmap_slam/rtabmap)
    SLAMNode->>SLAMNode: Load parameters from config file
    SLAMNode->>SLAMNode: Setup topic remappings
    Note over SLAMNode: Topics: rgb/image, depth/image,<br/>scan_cloud, odom, imu
    SLAMNode->>SLAMNode: Apply arguments (--delete_db_on_start, --log-level)
    SLAMNode-->>-RTABMap: SLAM node ready
    
    alt launch_viz == 'true' and mode == 'sim'
        RTABMap->>+VizNode: Node(rtabmap_viz/rtabmap_viz)
        VizNode->>VizNode: Load same configuration
        VizNode->>VizNode: Setup identical remappings
        VizNode->>VizNode: Start visualization interface
        VizNode-->>-RTABMap: Visualization ready
    end
    
    RTABMap-->>-Launcher: RTAB-Map system launched
```

### Node Startup Timing Sequence

```mermaid
sequenceDiagram
    participant Launch as Launch System
    participant Bridge as Bridges
    participant Core as Core Nodes
    participant Control as Controller
    participant Perception as Perception
    participant SLAM as RTAB-Map

    Launch->>+Bridge: Start immediately
    Bridge->>Bridge: PX4-ROS bridge
    Bridge->>Bridge: Gazebo bridge
    Bridge->>Bridge: Static transforms
    Bridge-->>-Launch: Infrastructure ready
    
    Launch->>+Core: Start immediately
    Core->>Core: Life monitor
    Core-->>-Launch: Monitoring active
    
    Note over Launch: 3-second timer delay
    
    Launch->>+Control: TimerAction(3.0s)
    Control->>Control: PX4 controller initialization
    Control-->>-Launch: Control ready
    
    Launch->>+Perception: TimerAction(3.0s)
    Perception->>Perception: Semantic perception
    Perception-->>-Launch: Perception ready
    
    Launch->>+SLAM: Start after core systems
    SLAM->>SLAM: RTAB-Map initialization
    SLAM-->>-Launch: SLAM operational
```

## Key Features

- **Multi-mode Operation**: Supports both simulation and deployment configurations
- **Dependency Management**: Proper sequencing of node startup with timing controls
- **Bridge Integration**: Seamless connection between PX4, Gazebo, and ROS2
- **Transform Management**: Automated static transform setup for sensor frames
- **Modular Architecture**: Conditional launching based on operational requirements
- **Configuration Management**: Centralized parameter and configuration handling

## Configuration Files

- `config/gz_bridge_config.yaml`: Gazebo-ROS2 bridge configuration
- `config/rtabmap_config.yaml`: RTAB-Map SLAM parameters

## Dependencies

- `flyscan_simulation`: PX4 simulation environment
- `flyscan_core`: Core system components
- `flyscan_drone_controller`: Flight control system
- `flyscan_perception`: Perception pipeline
- `flyscan_bridges`: Bridge components
- `rtabmap_slam`: 3D SLAM system
- `ros_gz_bridge`: Gazebo-ROS2 bridge
- `tf2_ros`: Transform system

## Usage Examples

### Basic Simulation Launch

```bash
ros2 launch flyscan_bringup flyscan.launch.py mode:=sim
```

### Deployment Mode

```bash
ros2 launch flyscan_bringup flyscan.launch.py mode:=deploy use_sim_time:=false
```

### Utilities Only

```bash
ros2 launch flyscan_bringup utils.launch.py mode:=sim px4_controller:=true semantic_perception:=true
```

### RTAB-Map with Visualization

```bash
ros2 launch flyscan_bringup rtabmap_3d.launch.py launch_viz:=true log_level:=info
```
