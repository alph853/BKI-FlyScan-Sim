# FlyScan Workspace: Implementation Guide

This README is optimized for an automated coding agent to implement, test, and integrate the navigation split and PX4 controller interfaces efficiently.

Summary

- Controller (flyscan_drone_controller): single source of truth for PX4 modes, arming, offboard heartbeat, setpoint execution, and safety interlocks.
- Navigation (flyscan_navigation): consumes frontiers, plans safe paths/trajectories, commands the controller via Actions/topics, manages missions.
- Exploration (flyscan_exploration): produces and ranks frontiers; never changes modes; offers on-demand frontier computation as an Action.
- Interfaces (flyscan_interfaces): services, actions, and messages used across packages.

Repository layout (expected)

- src/flyscan_drone_controller: PX4Controller node, TeleopNode.
- src/flyscan_navigation: NavigatorNode, local planner plugins, collision checker (to be created).
- src/flyscan_exploration: FrontierExplorer node, frontier extraction logic.
- src/flyscan_interfaces: shared msgs/srvs/actions.
- launch/: bringup and SITL launch files (to be added/extended).

Control authority and arbitration (must implement)

- Only PX4Controller changes modes, arms/disarms, and streams offboard setpoints.
- Requesters: TeleopNode (teleop), NavigatorNode (autonomous/mission) call SetControlMode.
- Priorities: Safety > Teleop > Autonomous > Mission. Teleop can preempt immediately.
- Ownership: extend SetControlMode.srv with caller_id, reason, optional ttl; controller publishes current owner and events.

Interfaces (final contracts)

- Services (PX4Controller):
  - /px4_controller/set_control_mode: request mode change (extend with caller_id, reason, ttl). Backward-compatible defaults.
  - /px4_controller/arm (optional convenience): bool arm.
- Actions (PX4Controller):
  - /px4_controller/follow_path: goal nav_msgs/Path or custom Trajectory; feedback progress; result success/aborted.
  - /px4_controller/navigate_to_pose_3d: goal geometry_msgs/PoseStamped + yaw.
- Topics:
  - Controller subscribes: /navigator/trajectory (if not using action).
  - Controller publishes: /controller/vehicle_state (pose/vel/armed/mode), /controller/events (mode changes, failsafes).
  - Teleop publishes: /px4_controller/teleop_command.
- Frontier explorer:
  - Publishes: /frontiers_ranked (FrontierArray). QoS: reliable, keep_last(1), consider transient_local.
  - Action: /frontier_explorer/compute_frontiers for ROI and long-running runs.

Frontier computation model

- Default: event-driven on map updates (debounced and rate-limited), publish topic with latest FrontierArray.
- On-demand: Action for ROI-bounded or expensive recomputes with feedback/cancel.
- Optional: low-rate periodic timer (0.2–1 Hz) in separate thread when updates are sparse.

Multi-robot namespacing

- All nodes accept parameter drone_id (0 default). Resolve interface names as /px4_controller or /px4_controller_{id}.
- Apply the same pattern to /frontiers_ranked and navigation topics/actions.

Implementation plan (backlog for coding agent)

1. Interfaces

- Add Actions in flyscan_interfaces:
  - action/FollowPath.action
  - action/NavigateToPose3D.action
  - action/ComputeFrontiers.action
- If needed, add msg/TrajectoryPoint.msg and msg/Trajectory.msg; otherwise use nav_msgs/Path first and iterate later.

1. PX4Controller updates (src/flyscan_drone_controller)

- Implement Action servers: follow_path and navigate_to_pose_3d.
- Add publishers: /controller/vehicle_state and /controller/events.
- Extend SetControlMode handling (caller_id, reason, ttl) with priority-based arbitration and ownership TTL.
- Keep offboard setpoint loop deterministic; preallocate messages and minimize logging in the loop.

1. Navigator (new package src/flyscan_navigation)

- Create CMakeLists.txt and package.xml with rclcpp, nav_msgs, geometry_msgs, action_msgs, pluginlib dependencies.
- Implement NavigatorNode:
  - Subscribes to /frontiers_ranked and /controller/vehicle_state.
  - Selects next goal frontier; plans a safe path using LocalPlanner plugin.
  - Sends FollowPath Action goal to PX4Controller; handles feedback and replans on blockage.
  - Requests mode: call /px4_controller/set_control_mode for autonomous when mission is active.
- Define LocalPlanner plugin interface and provide a simple 2D implementation (line-of-sight with inflation) as baseline.

1. FrontierExplorer (src/flyscan_exploration)

- Ensure it only publishes FrontierArray and optionally serves ComputeFrontiers Action.
- Convert periodic timer to event-driven recompute triggered by map updates with debounce/rate limit.
- Cache last FrontierArray; publish only on material change.

1. TeleopNode adjustments (src/flyscan_drone_controller)

- Keep SetControlMode call for teleop mode.
- Honor controller arbitration responses; on ESC, send exit_teleop command and release ownership if applicable.

1. Launch and simulation

- Add launch files:
  - controller.launch.py (PX4Controller with drone_id and QoS params).
  - navigation.launch.py (NavigatorNode with plugin params).
  - exploration.launch.py (FrontierExplorer with compute params).
  - bringup_sitl.launch.py composing the above plus PX4 SITL and sim world.
- Use ROS 2 components to co-locate Navigator and FrontierExplorer in one container if desired.

Build

- Source ROS 2 and build with colcon:

```shell
colcon build --symlink-install --packages-select flyscan_interfaces flyscan_drone_controller flyscan_exploration flyscan_navigation
source install/setup.fish
```

- For a full build:

```shell
colcon build --symlink-install
source install/setup.fish
```

Run (examples)

- Controller only:

```shell
ros2 launch flyscan_drone_controller controller.launch.py drone_id:=0
```

- Navigation and exploration:

```shell
ros2 launch flyscan_navigation navigation.launch.py drone_id:=0
ros2 launch flyscan_exploration exploration.launch.py drone_id:=0
```

- Full bringup (SITL):

```shell
ros2 launch flyscan_bringup bringup_sitl.launch.py drone_id:=0
```

- Controller publishes /controller/vehicle_state and /controller/events for debugging.
- Use rosbag to record /frontiers_ranked, /controller/*, and planner inputs for deterministic replays.
- Unit tests: mock Navigator and Controller to validate each side independently.


Conventions

- Coding: C++17, rclcpp components where possible; pluginlib for planners; avoid heavy work in timers.
- QoS: reliable keep_last(1) for control/feedback; consider transient_local for frontier outputs.
- Namespacing: use drone_id to scope all interfaces; follow /px4_controller or /px4_controller_id pattern.
- Safety first: controller enforces geofence/altitude guards and failsafe transitions.
