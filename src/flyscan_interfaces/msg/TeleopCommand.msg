# TeleopCommand.msg
# Message for sending teleop commands from teleop node to px4_controller

# Movement commands (normalized -1.0 to 1.0)
float32 forward     # Positive: forward, Negative: backward
float32 right       # Positive: right, Negative: left  
float32 up          # Positive: up, Negative: down
float32 yaw_rate    # Positive: clockwise, Negative: counterclockwise

# Special commands
bool hold_position  # True to hold current position
bool exit_teleop    # True to exit teleop mode
bool takeoff        # True to takeoff to NED (0,0,-1.5)
bool land           # True to land at NED (0,0,0)