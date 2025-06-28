sequenceDiagram
    participant D as Drone System
    participant SLAM as SLAM Module
    participant CV as Computer Vision
    participant PP as Path Planner
    participant QR as QR Decoder
    participant INV as Inventory Manager
    participant DS as Dashboard/Server
    participant DB as Database

    Note over D: System Initialization
    D->>D: Initialize sensors (LiDAR, cameras, IMU)
    D->>DS: Send system status (online)
    DS->>D: Acknowledge & send mission parameters

    Note over D, SLAM: Autonomous Exploration & Mapping
    D->>SLAM: Start SLAM process
    loop Exploration Phase
        D->>SLAM: Stream sensor data (LiDAR, visual)
        SLAM->>SLAM: Build/update occupancy grid map
        SLAM->>PP: Provide current map state
        PP->>PP: Generate exploration waypoints
        PP->>D: Send next exploration target
        D->>D: Navigate to target
        
        alt Every 30 seconds or significant update
            D->>DS: Send map progress update
            D->>DS: Send current position & battery status
        end
    end

    Note over D, CV: Inventory Detection & Processing
    loop Warehouse Scanning
        D->>CV: Capture shelf/area images
        CV->>CV: Detect shelves, products, QR codes
        
        alt QR Code Detected
            CV->>QR: Process QR code image
            QR->>QR: Decode stock/location data
            QR->>INV: Validate decoded information
            INV->>INV: Update local inventory cache
            
            Note over D, DS: Minimal Real-time Updates
            D->>DS: Send QR detection event (ID, location, timestamp)
            DS->>DB: Log inventory scan
        end
        
        alt Anomaly Detected
            CV->>CV: Detect missing items, misplaced stock
            D->>DS: Send anomaly alert (critical priority)
        end
        
        PP->>PP: Plan next scanning waypoint
        D->>PP: Request optimal path to next target
        PP->>D: Provide navigation commands
    end

    Note over D, DS: Periodic Status Updates
    loop Every 2 minutes
        D->>DS: Send telemetry (battery, position, system health)
        DS->>D: Send acknowledgment
        
        alt Battery Low
            DS->>D: Send return-to-base command
            D->>PP: Plan route to charging station
        end
    end

    Note over D, INV: Mission Completion
    D->>INV: Generate inventory summary
    INV->>INV: Compile scan results & statistics
    D->>DS: Send mission completion report
    DS->>DB: Store complete inventory data
    DS->>DS: Generate warehouse analytics
    
    D->>D: Return to base for charging
    D->>DS: Send mission end status