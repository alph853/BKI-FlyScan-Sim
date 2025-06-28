sequenceDiagram
    participant Drone
    participant Server
    participant Database
    participant Dashboard

    loop For each shelf/pallet
        Drone->>Server: Stream video frame
        Server-->>Drone: Command pose adjustment

        alt QR not in view
            Server->>Drone: Command new pose
            Drone-->>Server: Adjust pose and send new frame
        end

        Server-->>Server: Decode QR from image
        Server->>Database: Validate "[decoded QR]"
        Database-->>Server: Send QR validity

        alt QR invalid - Try to adjust view to detect a valid QR
            Server->>Drone: "Invalid QR – adjust pose"
            Drone-->>Server: Adjust pose and send new frame
            Server->>Server: Decode QR from new image
            Server->>Database: Re-validate "[decoded QR]"
        else QR valid - Update and continue
            Server->>Drone: "Valid QR"
            Server->>Server: Save stock metadata
            Server->>Dashboard: Push updated stock entry
        end
    end

    Server->>Drone: "Return to home"
    Drone-->>Server: Landing complete
