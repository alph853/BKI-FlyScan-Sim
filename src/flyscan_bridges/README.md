# FlyScan Bridges Package

## Overview

The `flyscan_bridges` package provides bridge functionality between PX4 autopilot system and ROS2. It handles coordinate frame transformations and message format conversions to ensure seamless integration between PX4 and the ROS2 ecosystem.

## Purpose

- Convert PX4-specific messages to standard ROS2 messages
- Handle coordinate frame transformations (NED ↔ ENU)
- Provide odometry data in ROS2 standard format
- Broadcast transform trees for navigation stack integration

## Components

### PX4ROSBridge Node

The main bridge node that handles PX4 to ROS2 message conversion.

#### Topics

- **Subscribes**
  - `/fmu/out/vehicle_odometry` (px4_msgs/VehicleOdometry)
- **Publishes**
  - `/odom` (nav_msgs/Odometry)
- **Broadcasts**
  - TF transforms (odom → base_link)

#### Parameters

- `use_sim_time`: Boolean flag for simulation time synchronization
