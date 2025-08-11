# Current vehicle state information
std_msgs/Header header

# Current pose and velocity
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped velocity

# Control mode
string control_mode

# Vehicle status
bool armed
bool offboard_mode
bool landed

# Safety status
bool geofence_violated
float32 battery_voltage
uint8 gps_fix_type

# Current trajectory tracking
bool following_trajectory
uint32 current_waypoint
float32 trajectory_progress