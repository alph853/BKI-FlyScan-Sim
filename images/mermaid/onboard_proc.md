sequenceDiagram
    participant D as Drone
    participant DS as Dashboard/Server
    participant DB as Database

    D->>DS: Initiate "ONLINE" signal
    DS->>D: Send mission parameters
    
    loop Warehouse Scanning
        D->>D: Explore & map environment, path planning, QR processing
        
        D->>DS: Validate "[decoded QR]"
        DS-->>D: Send QR validity
        D->>D: plan new pose (next view based on QR validity)
        DS->>DB: Log inventory events
    end
    
    loop Every 1 minutes
        D->>DS: Status update (position, battery, etc)
    end
    
    D->>DS: Mission complete + full report
    DS->>DB: Store complete inventory data