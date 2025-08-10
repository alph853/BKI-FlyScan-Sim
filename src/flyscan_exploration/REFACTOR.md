#

Control authority and mode transitions

- Single source of truth: PX4Controller is the only node that talks MAVLink/PX4 modes, arming/disarming, and maintains the offboard setpoint heartbeat. Keep all mode switching here.
- Who requests mode changes:
  - TeleopNode requests TELEOP via SetControlMode service.
  - NavigatorNode requests AUTONOMOUS or MISSION via the same SetControlMode service.
  - Optional: a small Mode Arbiter library inside PX4Controller validates requests against vehicle state, priorities, and safety guards.
- Rationale: centralizes safety and PX4 quirks, simplifies testing, and avoids races between behavior nodes.

Recommended mode/state machine (inside PX4Controller)

States:

- kManual (default)
- kTeleop
- kAutonomous
- kMission
- kFailsafeHold (hover/loiter)
- kFailsafeLand (command land)

Transitions (preconditions checked inside controller):

- Manual → Teleop: armed? if not, arm; start offboard; begin teleop setpoint stream.
- Manual → Autonomous/Mission: arm; start offboard; wait for first trajectory before accepting.
- Teleop ↔ Autonomous/Mission: brief hold during switch; keep offboard alive.
- Any → FailsafeHold: on loss of trajectory, obstacle emergency stop from navigator, or degraded sensors.
- FailsafeHold → FailsafeLand: on prolonged fault or explicit command.

Mode arbitration and priorities

- Priority order: Safety > Teleop > Autonomous > Mission.
- The controller enforces:
  - Teleop can preempt Autonomous/Mission immediately.
  - Navigator can only switch modes when no teleop preemption is active.
  - All mode changes are logged and published on /controller/events.

Interfaces and contracts (final)

- Services (PX4Controller):
  - /px4_controller/set_control_mode (existing, add caller_id, reason, optional ttl)
  - /px4_controller/arm (bool arm) [optional convenience]
- Actions (PX4Controller):
  - /px4_controller/follow_path (goal: nav_msgs/Path or flyscan Trajectory, feedback: progress, result: success)
  - /px4_controller/navigate_to_pose_3d (goal: PoseStamped + yaw)
- Topics:
  - Controller subscribes: /navigator/trajectory (if not using action)
  - Controller publishes: /controller/vehicle_state (pose/vel/armed/mode), /controller/events (mode changes, failsafes)
  - Teleop publishes: /px4_controller/teleop_command

Multi-robot namespacing

- Keep the drone_id strategy already used by TeleopNode.
- All controller interfaces live under a namespace: /px4_controller_{id}/...
- Navigator and Explorer accept a drone_id parameter and resolve topic/service names via the same helper to avoid mismatches.

Performance and real-time hygiene

- Controller:
  - Avoid dynamic allocations and heavy logging in setpoint loop; preallocate messages.
  - Use intra-process comms for state publishers; QoS: keep_last(1), reliable for setpoints; best_effort for high-rate telemetry if needed.
- Navigator:
  - Separate executor (multi-threaded) for planning; isolate CPU cores if possible.
  - Use downsampled point clouds and incremental costmap updates.
  - Time-bound planning; on timeout, send emergency hold to controller.

Safety and failsafe behaviors

- Heartbeats:
  - Navigator sends heartbeat with each trajectory chunk; controller aborts to kFailsafeHold if it expires.
- Geofence and altitude guard enforced in controller.
- Loss of position or EKF health: controller transitions to kFailsafeLand.
- Emergency stop service/topic accepted by controller from any node (highest priority).

Deployment and bringup

- Launch order: controller → navigator → explorer → teleop (optional).
- Lifecycle nodes: controller activates only after PX4 connection; navigator activates after maps ready; explorer after navigator ready.
- Parameters profiles: sim.yaml vs real.yaml; limits and QoS tuned per profile.

FrontierExplorer placement (package boundaries)

- Keep frontier_explorer in its own package flyscan_exploration. It is an exploration/perception producer that detects and ranks frontiers; Navigator consumes them and owns behaviors and control interactions.
- Benefits: clear separation of concerns, easier swapping of exploration algorithms, independent testing/benchmarking, optional reuse outside this project.
- Contract: publish flyscan_interfaces/msg/FrontierArray; never call controller services or change modes.
- Topic: /frontiers_ranked (FrontierArray) produced by FrontierExplorer; consumed by NavigatorNode.
- Namespacing: respect drone_id; topics under /namespace/frontiers_ranked.
- On-demand recompute: expose an Action, not a Service, for expensive runs and ROI requests.

When to use which:

- Topic + event-driven: default path; cheap and reactive; cached latest result is always available to late joiners.
- Action ComputeFrontiers: long-running (>100 ms) or ROI-bounded computation, needs feedback and cancellation.
- Service RecomputeFrontiers: only if computation is guaranteed fast and bounded (<50–100 ms). Otherwise prefer Action.
- Periodic timer: optional low-rate safety net (e.g., 0.2–1 Hz) in a separate thread/executor if map updates are sparse.

API shape:

- Topic: /frontiers_ranked (FrontierArray). QoS: reliable, keep_last(1). Consider transient_local if you want last result for late subscribers.
- Action: /frontier_explorer/compute_frontiers
  - Goal: optional ROI (pose, radius/box), optional compute budget, ranking policy
  - Feedback: progress percent, frontier count
  - Result: FrontierArray, stats (duration_ms, pruned_count)
- Parameters:
  - recompute_min_period_ms, debounce_ms
  - roi_default_radius, min_frontier_size
  - max_compute_time_ms (used to early-stop and return best-effort set)

Execution model and performance:

- Use a multi-threaded executor for frontier_explorer; run compute in a dedicated worker to avoid blocking subscriptions.
- Cancel and restart: if a new map update arrives, cancel the in-flight Action compute (if any) and restart with the latest map.
- Caching: store last FrontierArray atomically; publish only if materially changed (thresholds on count/delta pose).
- For 3D: offload TSDF/ESDF updates to GPU libs (e.g., nvblox); run frontier extraction on downsampled voxels.

Navigator integration:

- Navigator must not perform frontier computation; it orchestrates and can request ROI-limited runs via the Action.

Migration checklist

- [ ] Create flyscan_navigation package (NavigatorNode, planners, collision checker).
- [ ] Move map/point-cloud subscriptions and safe navigation logic from PX4Controller to NavigatorNode.
- [ ] Add FollowPath and NavigateToPose3D actions to flyscan_interfaces and implement servers in PX4Controller.
- [ ] Add /controller/vehicle_state and /controller/events publishers.
- [ ] Update TeleopNode to handle rejection if another owner holds mode; add ESC to request release.
- [ ] Remove any controller mode/service calls from frontier_explorer; it must only publish FrontierArray (Navigator owns mode changes).
- [ ] Implement event-driven frontier recompute on map updates with debounce and rate limiting.
- [ ] Add frontier_explorer ComputeFrontiers Action with ROI and feedback; keep topic publication with QoS keep_last(1) (optionally transient_local).
- [ ] (Optional) Add a low-rate periodic timer (0.2–1 Hz) in a separate thread as a safety net when maps are static.
