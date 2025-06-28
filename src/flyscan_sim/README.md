# FlyS can Simulation

A comprehensive simulation package for PX4 drone with RTAB-Map SLAM in a warehouse environment using Gazebo and ROS2 Humble.

## Features

- PX4 SITL integration with Gazebo simulation
- X500 drone with OakD-Lite depth camera and IMU
- RTAB-Map SLAM for real-time mapping and localization  
- RViz2 visualization with custom configuration
- Warehouse environment with walls and obstacles
- GUI/Headless simulation options

## Dependencies

Make sure you have the following packages installed:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-gz-bridge
sudo apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-rtabmap-slam
sudo apt install ros-humble-rtabmap-viz
sudo apt install ros-humble-tf2-ros
```

## Build

Build the workspace:

```bash
cd /home/ttd/Documents/flyscan_ws/flyscan_ws
colcon build --packages-select flyscan_sim
source install/setup.bash
```

## Usage

### Basic Simulation (with GUI)

```bash
ros2 launch flyscan_sim flyscan.launch.py
```

### Headless Simulation

```bash
ros2 launch flyscan_sim flyscan.launch.py headless:=true
```

### Custom World

```bash
ros2 launch flyscan_sim flyscan.launch.py world:=warehouse
```

## Launch Sequence

The launch file automatically starts components in this order:

1. **Gazebo Simulation** - Starts with warehouse world
2. **Drone Spawning** - Spawns X500 with depth camera after 3s
3. **PX4 SITL** - Starts PX4 autopilot after 5s  
4. **ROS2 Bridge** - Bridges Gazebo topics to ROS2 after 8s
5. **TF Publishers** - Sets up coordinate transforms after 10s
6. **RTAB-Map SLAM** - Starts SLAM after 12s
7. **RViz2** - Launches visualization after 15s
8. **RTAB-Map Viz** - Optional additional visualization after 17s

## Available Topics

### Camera Topics
- `/camera` - RGB camera image
- `/camera/camera_info` - Camera calibration info
- `/depth_camera` - Depth camera image  
- `/depth_camera/camera_info` - Depth camera calibration
- `/depth_camera/points` - Point cloud data

### Drone Topics
- `/imu` - IMU sensor data
- `/model/x500_depth/pose` - Drone pose
- `/model/x500_depth/odometry` - Drone odometry

### RTAB-Map Topics
- `/rtabmap/grid_map` - Occupancy grid map
- `/rtabmap/mapPath` - SLAM trajectory
- `/rtabmap/localization_pose` - Localization pose

## Parameters

The RTAB-Map node is configured with optimized parameters for drone SLAM:

- `frame_id: base_link` - Base coordinate frame
- `subscribe_depth: true` - Uses depth camera
- `approx_sync: true` - Approximate timestamp synchronization
- `Reg/Force3DoF: true` - Forces 3DOF mode for better stability
- `Vis/MinInliers: 15` - Minimum visual features for loop closure

## Troubleshooting

### PX4 Path Issues
If PX4 fails to start, verify the path in the launch file:
```python
px4_dir = '/home/ttd/Documents/flyscan_ws/flyscan_ws/PX4-Autopilot'
```

### Missing Models
If Gazebo models are not found, ensure `GZ_SIM_RESOURCE_PATH` includes:
- `PX4-Autopilot/Tools/simulation/gz/models`
- `PX4-Autopilot/Tools/simulation/gz/worlds`

### RTAB-Map Database
The SLAM database is stored at `~/.ros/rtabmap.db`. Delete this file to reset the map:
```bash
rm ~/.ros/rtabmap.db
```

## File Structure

```
src/flyscan_sim/
├── launch/
│   └── flyscan.launch.py     # Main launch file
├── worlds/
│   └── warehouse.sdf         # Warehouse simulation world  
├── rviz/
│   └── flyscan_rtabmap.rviz  # RViz configuration
├── CMakeLists.txt            # Build configuration
├── package.xml               # Package dependencies
└── README.md                 # This file
```