sequenceDiagram
    participant CAM as RGB-D Camera (Sim)
    participant IMU as IMU (Sim)
    participant VIO as VioNode
    participant SLAM as SLAMNode
    participant SEM as SemanticNode
    participant EXP as ExplorationNode
    participant HYP as HypermapServer
    participant APP as ApplicationNode
    participant NAV as NavigationNode
    participant MON as LifeMonitor

    Note over VIO, MON: bonds establishment

    CAM ->> VIO:    publish /camera/image_raw
    IMU ->> VIO:    publish /imu/inertial
    VIO -->>SLAM:   publish /odom

    CAM -)  SLAM:   publish /camera/depth_image
    SLAM--) HYP:    publish /occupancy_map
    SLAM--) EXP:    forward /occupancy_map

    SLAM-)  EXP:    forward /occupancy_map
    VIO -)  EXP:    publish /odom
    EXP --) HYP:    publish /exploration_map

    CAM -)  SEM:    publish /camera/image_raw, /camera/depth_image
    SLAM-)  SEM:    forward /occupancy_map
    SEM --) HYP:    publish /semantic_map & convex-hull areas
    HYP -)  HYP:    fuse occupancy, exploration & semantic layers
    APP ->> HYP:    QueryMap(request)
    HYP -->>APP:    QueryMap(response: goal)
    APP ->> NAV: NavigateToPose(goal)
    NAV --) APP: feedback
    NAV --) APP: result
