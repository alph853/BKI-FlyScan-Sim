# Autonomous Drone Frontier Exploration System

## Table of Contents

- [Overview](#overview)
- [Mathematical Formulation](#mathematical-formulation)
- [Operational Flow](#operational-flow)
- [Algorithm Implementation](#algorithm-implementation)
- [Configuration](#configuration)
- [Performance Analysis](#performance-analysis)
- [Integration](#integration)

## Overview

The Autonomous Drone Frontier Exploration System implements an intelligent exploration strategy for unmanned aerial vehicles (UAVs) using frontier-based exploration with advanced clustering and utility optimization. The system autonomously discovers unknown areas in occupancy grid maps while optimizing for exploration efficiency, safety, and completeness.

### Key Features
- **Frontier Detection**: DFS-based flood-fill algorithm with circular ROI optimization
- **Intelligent Clustering**: DBSCAN algorithm for frontier grouping
- **Multi-criteria Utility Function**: Distance, size, and information gain optimization
- **Adaptive ROI Management**: Dynamic exploration radius adjustment
- **Stuck Detection**: Similar frontier detection with temporal analysis
- **ROS2 Lifecycle Integration**: Managed node state transitions

## Mathematical Formulation

### 1. Frontier Detection

A **frontier** is defined as the boundary between known free space and unknown space in an occupancy grid. Mathematically:

```
F(x,y) = {(x,y) | grid(x,y) = free AND ∃(x',y') ∈ N₈(x,y) : grid(x',y') = unknown}
```

Where:
- `grid(x,y)` represents the occupancy value at cell (x,y)
- `N₈(x,y)` is the 8-connected neighborhood
- `free` corresponds to occupancy values < 50
- `unknown` corresponds to occupancy value = -1

### 2. Region of Interest (ROI) Constraint

To optimize computational efficiency, frontier detection is constrained to a circular ROI around the robot:

```
ROI(x,y) = {(x,y) | ||P_robot - P(x,y)||₂ ≤ r_roi}
```

Where:
- `P_robot = (x_robot, y_robot)` is the robot's position in world coordinates
- `P(x,y)` is the world coordinate of grid cell (x,y)
- `r_roi` is the adaptive ROI radius

### 3. DBSCAN Clustering

Frontier points are grouped using DBSCAN clustering to form coherent exploration targets:

**Distance Metric:**
```
d(p₁, p₂) = ||p₁ - p₂||₂
```

**Core Point Definition:**
```
|N_ε(p)| ≥ min_points
```

**Cluster Formation:**
- Core points with distance ≤ ε are grouped
- Border points are assigned to nearest cluster
- Noise points are discarded

### 4. Utility Function

Each frontier cluster is evaluated using a multi-criteria utility function:

```
U(F_i) = w_d · S_distance(F_i) + w_s · S_size(F_i) + w_I · S_information(F_i)
```

#### Distance Score (Proximity Preference)
```
S_distance(F_i) = 1 / (1 + d(robot, F_i) / 10.0)
```

#### Size Score (Larger Frontiers Preferred)
```
S_size(F_i) = min(|F_i| / 20.0, 1.0)
```

#### Information Gain Score
```
S_information(F_i) = N_unknown(F_i, r_info) / N_total(F_i, r_info)
```

Where:
- `N_unknown(F_i, r_info)` = number of unknown cells within radius r_info of frontier centroid
- `N_total(F_i, r_info)` = total cells within radius r_info

### 5. Adaptive ROI Management

The ROI radius adapts based on frontier availability:

```
r_roi(t+1) = {
    min(r_roi(t) + 2.0, r_max)     if N_frontiers = 0
    max(r_roi(t) - 1.0, r_min)     if N_frontiers > 10
    min(r_roi(t) + 1.0, r_max)     if 0 < N_frontiers < 3
    r_roi(t)                       otherwise
}
```

### 6. Similarity Detection (Stuck Prevention)

Frontier similarity is detected using spatial-temporal analysis:

```
Similar(F_new, F_history) = {
    true   if Σ[d(F_new, F_i) ≤ θ_sim] / |F_history| > 0.8
    false  otherwise
}
```

Where:
- `F_history` contains frontiers from last `t_threshold` seconds
- `θ_sim` is the similarity distance threshold
- History is maintained with temporal filtering

## Operational Flow

### System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Map Source    │───▶│ Frontier         │───▶│   PX4           │
│   (RTAB-Map)    │    │ Explorer         │    │ Controller      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌──────────────────┐
                       │  Goal Publisher  │
                       │ (/exploration_   │
                       │     _goal)       │
                       └──────────────────┘
```

### Stage-by-Stage Operation

#### Stage 1: Initialization and Activation
```cpp
HandleActivate() {
    // Switch to autonomous mode
    SetControlMode(AUTONOMOUS)
    
    // Start exploration timer
    exploration_active_ = true
    exploration_timer_->reset()
}
```

#### Stage 2: Map and Position Validation
```cpp
ExplorationTimerCallback() {
    // Validate map availability
    if (!current_map_) return WAIT
    
    // Update robot position via TF
    if (!UpdateRobotPosition()) return WAIT
}
```

#### Stage 3: Frontier Detection with DFS
```cpp
DetectFrontiers(grid) {
    visited = zeros(height, width)
    frontier_points = []
    
    for each cell (x,y) in grid:
        if !visited[x,y] AND IsWithinROI(x,y):
            if IsFree(x,y) AND HasUnknownNeighbor(x,y):
                cluster = DfsFloodFill(x, y, visited)
                frontier_points.extend(cluster)
    
    return DbscanClustering(frontier_points)
}
```

#### Stage 4: DBSCAN Clustering
```cpp
DbscanClustering(points) {
    for each point p in points:
        if !p.visited:
            neighbors = GetNeighbors(p, eps)
            if len(neighbors) >= min_points:
                ExpandCluster(p, neighbors)
    
    return ConvertToFrontiers(clusters)
}
```

#### Stage 5: Utility Calculation and Selection
```cpp
SelectBestFrontier(frontiers) {
    for each frontier f:
        f.utility = CalculateUtility(f)
    
    sort(frontiers, key=utility, reverse=True)
    
    for each frontier f in sorted_frontiers:
        if f.utility < min_threshold: break
        if !DetectSimilarFrontiers(f.center):
            return f
    
    return nullptr  // Exploration complete
}
```

#### Stage 6: Goal Publication
```cpp
PublishExplorationGoal(frontier) {
    goal.pose.position = frontier.center
    goal.header.frame_id = "map"
    goal_publisher_->publish(goal)
}
```

### Lifecycle State Machine

```
┌─────────────┐  configure  ┌─────────────┐  activate   ┌─────────────┐
│ UNCONFIGURED│────────────▶│  INACTIVE   │────────────▶│   ACTIVE    │
└─────────────┘             └─────────────┘             └─────────────┘
                                   ▲                           │
                                   │ deactivate                │
                                   └───────────────────────────┘
```

## Algorithm Implementation

### DFS Flood-Fill Algorithm

The DFS implementation uses a stack-based approach for memory efficiency:

```cpp
void DfsFloodFill(grid, start_x, start_y, visited, frontier_points) {
    stack<Point> stack;
    stack.push(Point(start_x, start_y));
    
    while (!stack.empty()) {
        current = stack.top(); stack.pop();
        
        if (OutOfBounds(current) || visited[current] || !IsWithinROI(current))
            continue;
            
        if (!IsFree(current)) continue;
        
        visited[current] = true;
        frontier_points.push_back(current);
        
        // Add 4-connected neighbors
        stack.push(Point(current.x + 1, current.y));
        stack.push(Point(current.x - 1, current.y));
        stack.push(Point(current.x, current.y + 1));
        stack.push(Point(current.x, current.y - 1));
    }
}
```

### DBSCAN Implementation Details

**Core Point Detection:**
```cpp
bool IsCore(point_idx, points, eps, min_points) {
    neighbors = GetNeighbors(points, point_idx, eps);
    return neighbors.size() >= min_points;
}
```

**Cluster Expansion:**
```cpp
void ExpandCluster(point_idx, neighbors, cluster_id) {
    points[point_idx].cluster_id = cluster_id;
    
    queue<int> seed_set(neighbors);
    
    while (!seed_set.empty()) {
        current = seed_set.front(); seed_set.pop();
        
        if (!points[current].visited) {
            points[current].visited = true;
            current_neighbors = GetNeighbors(current, eps);
            
            if (current_neighbors.size() >= min_points) {
                seed_set.insert(current_neighbors);
            }
        }
        
        if (points[current].cluster_id == -1) {
            points[current].cluster_id = cluster_id;
        }
    }
}
```

### Information Gain Calculation

The information gain is calculated by sampling unknown cells within a circular region:

```cpp
double CalculateInformationGain(frontier, grid, info_radius) {
    center_x = WorldToGrid(frontier.center.x);
    center_y = WorldToGrid(frontier.center.y);
    radius_cells = info_radius / grid.resolution;
    
    unknown_count = 0;
    total_count = 0;
    
    for (dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx*dx + dy*dy <= radius_cells*radius_cells) {
                x = center_x + dx;
                y = center_y + dy;
                
                if (InBounds(x, y)) {
                    if (grid.data[y * width + x] == -1) {
                        unknown_count++;
                    }
                    total_count++;
                }
            }
        }
    }
    
    return (total_count > 0) ? (double)unknown_count / total_count : 0.0;
}
```

## Configuration

### Parameter Categories

#### Core Exploration Parameters
```yaml
exploration_radius: 10.0      # Base exploration range (m)
exploration_rate: 1.0         # Timer frequency (Hz)
max_frontier_distance: 50.0   # Maximum frontier distance (m)
```

#### ROI Management
```yaml
roi_radius: 15.0              # Initial ROI radius (m)
min_roi_radius: 5.0           # Minimum ROI radius (m)  
max_roi_radius: 25.0          # Maximum ROI radius (m)
```

#### DBSCAN Clustering
```yaml
dbscan_eps: 3.0               # Clustering distance threshold (grid cells)
dbscan_min_points: 3          # Minimum points per cluster
min_frontier_size: 5.0        # Minimum cluster size for valid frontier
```

#### Utility Function Weights
```yaml
distance_weight: 0.3          # Weight for distance score
size_weight: 0.3              # Weight for size score  
information_weight: 0.4       # Weight for information gain
min_utility_threshold: 0.2    # Minimum utility for frontier selection
```

#### Similarity Detection
```yaml
similarity_threshold: 2.0     # Distance threshold for similar frontiers (m)
similarity_time_threshold: 30.0 # Time window for stuck detection (s)
```

### Tuning Guidelines

**For Large Environments:**
- Increase `roi_radius` and `max_roi_radius`
- Reduce `exploration_rate` to 0.5 Hz
- Increase `information_weight` to 0.5

**For Dense Environments:**
- Decrease `dbscan_eps` to 2.0
- Increase `min_frontier_size` to 8.0
- Increase `distance_weight` to 0.4

**For Real-time Performance:**
- Decrease `max_roi_radius` to 15.0
- Increase `exploration_rate` to 2.0 Hz
- Set `min_utility_threshold` to 0.3

## Performance Analysis

### Computational Complexity

| Algorithm | Time Complexity | Space Complexity |
|-----------|----------------|------------------|
| DFS Flood-Fill | O(n) | O(n) |
| DBSCAN | O(n log n) | O(n) |
| Utility Calculation | O(k·r²) | O(1) |
| ROI Filtering | O(n) | O(1) |

Where:
- n = number of frontier points
- k = number of frontier clusters  
- r = information radius in grid cells

### Memory Usage

**Grid Processing:** O(W × H) for visited array
**Frontier Storage:** O(F × P) where F = frontier count, P = points per frontier
**History Tracking:** O(H) where H = history window size

### Timing Benchmarks

On typical warehouse environments (100×100m, 0.05m resolution):
- **Frontier Detection:** ~10-50ms
- **DBSCAN Clustering:** ~5-20ms
- **Utility Calculation:** ~1-5ms per frontier
- **Total Cycle Time:** ~50-200ms

## Integration

### ROS2 Interface

**Subscribed Topics:**
- `/map` (nav_msgs/OccupancyGrid) - Occupancy grid from SLAM

**Published Topics:**  
- `/exploration_goal` (geometry_msgs/PoseStamped) - Selected frontier goal

**Service Clients:**
- `/px4_controller/set_control_mode` - Switch to autonomous mode

**TF Dependencies:**
- `map` → `base_link` transform for robot localization

### Launch Integration

```xml
<node pkg="flyscan_exploration" exec="frontier_explorer" name="frontier_explorer">
    <param from="$(find-pkg-share flyscan_exploration)/config/frontier_explorer.yaml"/>
</node>
```

### System Dependencies

```bash
# Required ROS2 packages
sudo apt install ros-humble-nav-msgs
sudo apt install ros-humble-tf2-ros  
sudo apt install ros-humble-tf2-geometry-msgs

# Required system libraries  
sudo apt install libopencv-dev
sudo apt install libeigen3-dev
```

### Integration with Navigation Stack

The frontier explorer integrates with the broader autonomous navigation system:

1. **SLAM Integration:** Receives occupancy grids from RTAB-Map SLAM
2. **Path Planning:** Goals are consumed by PX4 flight controller
3. **Lifecycle Management:** Managed by LifeMonitor system
4. **Safety Integration:** Cooperates with obstacle avoidance systems

### Monitoring and Debugging

**Log Levels:**
- `INFO`: Frontier selection and major state changes
- `DEBUG`: Detailed algorithm steps and parameter values  
- `WARN`: Stuck conditions and recovery attempts

**Key Metrics:**
- Frontiers detected per cycle
- Average utility scores
- ROI radius adaptations
- Exploration completion percentage

---

*This documentation covers the complete mathematical formulation and operational implementation of the autonomous drone frontier exploration system. For additional technical details, refer to the source code comments and configuration files.*
