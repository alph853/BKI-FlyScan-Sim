# Controller event message for logging mode changes and failsafes
std_msgs/Header header

# Event types
uint8 EVENT_MODE_CHANGE = 1
uint8 EVENT_FAILSAFE_TRIGGERED = 2
uint8 EVENT_FAILSAFE_CLEARED = 3
uint8 EVENT_EMERGENCY_STOP = 4
uint8 EVENT_GEOFENCE_VIOLATION = 5
uint8 EVENT_LOW_BATTERY = 6
uint8 EVENT_GPS_LOST = 7

uint8 event_type
string event_description

# Mode change details (if applicable)
string previous_mode
string new_mode
string caller_id
string reason

# Priority level
uint8 PRIORITY_LOW = 1
uint8 PRIORITY_MEDIUM = 2
uint8 PRIORITY_HIGH = 3
uint8 PRIORITY_CRITICAL = 4

uint8 priority