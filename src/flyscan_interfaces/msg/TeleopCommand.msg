# TeleopCommand.msg
# Message for sending teleop commands from teleop node to px4_controller

# Single command field to indicate the action to take
string command      # Possible values: "forward", "backward", "left", "right", "up", "down", 
                    # "yaw_left", "yaw_right", "hold_position", "exit_teleop", "takeoff", "land", "stop"