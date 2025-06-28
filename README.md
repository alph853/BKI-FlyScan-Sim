# Autonomous UAV System

A comprehensive autonomous unmanned aerial vehicle (UAV) system built with ROS 2, featuring lifecycle-managed nodes for exploration, semantic mapping, SLAM, and navigation.

## Overview

This project implements a fully autonomous UAV system capable of:
- **Visual-Inertial Odometry (VIO)** for real-time pose estimation
- **Pose Graph SLAM** for robust localization and mapping
- **Semantic Mapping** for environment understanding and object detection
- **Autonomous Exploration** with frontier detection and path planning
- **Multi-layer Map Fusion** through the Hypermap Server
- **Intelligent Navigation** with obstacle avoidance
- **Application-level Mission Control** for high-level task execution
- **Lifecycle Management** with fault detection and recovery

## System Architecture

### Node Hierarchy and Dependencies
<!--  -->
The system follows a carefully orchestrated startup sequence managed by the LifeMonitor:

### Core Components

xxx

### Run

#### ROS2 Node CLI

```bash
ros2 node list
```

#### Lifecycle Nodes CLI

```bash
ros2 lifecycle list
ros2 lifecycle get ns/<node_name>
ros2 lifecycle set ns/<node_name> <transition>
```
Transitions are: `configure, activate, deactivate, cleanup, shutdown`